/*
 * gpadc_app.c
 *
 * GPADC application task for the DA14706.
 *
 * Channel switching strategy
 * --------------------------
 * ad_gpadc_open() is called ONCE at startup. ad_gpadc_reconfig() is used
 * to switch between Ch0 and Ch1 without releasing the adapter handle.
 *
 * This is possible because the input-pin check in ad_gpadc_reconfig() has
 * been removed in the local copy of ad_gpadc.c (see that file for the full
 * rationale). reconfig() now only rewrites ADC control registers via
 * hw_gpadc_configure() — no semaphore release, no LDO cycle, no IO mux
 * reconfiguration.
 *
 * Synchronisation guarantee
 * -------------------------
 * reconfig() is called only after ad_gpadc_read_nof_conv() returns.
 * read_nof_conv() is synchronous: it blocks on OS_EVENT_WAIT until the
 * hardware fires its completion interrupt, so no conversion is ever in
 * progress when reconfig() is called. The read_in_progress guard inside
 * reconfig() is a second safety layer that will never trigger in this flow.
 *
 * COM power domain
 * ----------------
 * hw_sys_pd_com_enable() is called once at startup with no matching
 * disable. This holds the COM domain reference count at >= 1 permanently,
 * preventing the AHT20 task's ad_i2c_close() from ever powering down the
 * ADC LDO between reconfig() calls.
 *
 * S&H settling after reconfig
 * ----------------------------
 * After each reconfig() call a single dummy read is performed before the
 * real measurement. This lets the sample-and-hold capacitor fully charge
 * to the new input before the result is used.
 */

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include "osal.h"
#include "ad_gpadc.h"
#include "hw_sys.h"
#include "platform_devices.h"
#include "include/gpadc_app.h"

/* -----------------------------------------------------------------------
 * Shared temperature / humidity from aht20_task.c
 * ----------------------------------------------------------------------- */
extern volatile float   g_last_temp_c;
extern volatile uint8_t g_last_hum_percent;

/* -----------------------------------------------------------------------
 * Channel aliases
 * ----------------------------------------------------------------------- */
#define CHAN0_DEVICE   ADC_CH0_DEVICE
#define CHAN1_DEVICE   ADC_CH1_DEVICE

/* -----------------------------------------------------------------------
 * Compile-time options
 *
 * GPADC_MEASURE_RATE  — print achieved sample-pair rate once per second.
 *                        Set to 0 once the rate is confirmed.
 * GPADC_PRINT_SAMPLES — print mV values (decimated to ~100 lines/s).
 *                        Set PRINT_DIVISOR to 1 to print every sample
 *                        (only do this when rate is low, e.g. during
 *                        signal shape validation).
 * ----------------------------------------------------------------------- */
#define GPADC_MEASURE_RATE    0
#define GPADC_PRINT_SAMPLES   1
#define GPADC_PRINT_DIVISOR   6     /* print 1 in every N pairs (~100/s at 600Hz) */

/* -----------------------------------------------------------------------
 * gpadc_app_init — empty; hardware init is centralised in main.c.
 * ----------------------------------------------------------------------- */
void gpadc_app_init(void) {}

/* -----------------------------------------------------------------------
 * gpadc_app_task
 * ----------------------------------------------------------------------- */
void gpadc_app_task(void *pvParameters)
{
        uint16_t raw;
        uint16_t dummy;
        int      ret;

        /*
         * Permanently hold the COM power domain so that the AHT20 task's
         * ad_i2c_close() can never drop the reference count to zero and
         * power down the ADC LDO while we are between reconfig() calls.
         */
        hw_sys_pd_com_enable();

        /*
         * Open the GPADC handle once and hold it for the task lifetime.
         * ad_gpadc_open() acquires the resource semaphore, enables the
         * COM domain (reference count goes to 2 here — we hold 2 counts
         * total now, which is fine), waits for LDO OK, and initialises
         * the hardware for Ch0.
         */
        ad_gpadc_handle_t h = ad_gpadc_open(CHAN0_DEVICE);
        if (!h) {
                printf("[GPADC] FATAL: open failed\r\n");
                OS_TASK_DELETE(NULL);
                return;
        }

        /* Warm-up dummy reads — one per channel after power-on */
        ad_gpadc_read_nof_conv(h, 1, &dummy);                  /* Ch0 warm-up */

        ret = ad_gpadc_reconfig(h, CHAN1_DEVICE->drv);
        if (ret != AD_GPADC_ERROR_NONE) {
                printf("[GPADC] FATAL: reconfig to ch1 failed at warmup (%d)\r\n", ret);
                OS_TASK_DELETE(NULL);
                return;
        }
        ad_gpadc_read_nof_conv(h, 1, &dummy);                  /* Ch1 warm-up */

        /* Switch back to Ch0 ready for the main loop */
        ad_gpadc_reconfig(h, CHAN0_DEVICE->drv);

#if GPADC_MEASURE_RATE
        uint32_t sample_count        = 0;
        uint32_t tick_start          = OS_GET_TICK_COUNT();
        const uint32_t ticks_per_sec = 1000U / OS_TICK_PERIOD;
#endif

        for (;;) {

                /* ---- Read Ch0 (currently configured) ---- */
                ret = ad_gpadc_read_nof_conv(h, 1, &raw);
                float mv0 = (ret == AD_GPADC_ERROR_NONE)
                        ? (float)ad_gpadc_conv_to_mvolt(CHAN0_DEVICE->drv, raw)
                        : 0.0f;

                /* ---- Switch to Ch1 ---- */
                ad_gpadc_reconfig(h, CHAN1_DEVICE->drv);
                ad_gpadc_read_nof_conv(h, 1, &dummy);          /* S&H settle */

                /* ---- Read Ch1 ---- */
                ret = ad_gpadc_read_nof_conv(h, 1, &raw);
                float mv1 = (ret == AD_GPADC_ERROR_NONE)
                        ? (float)ad_gpadc_conv_to_mvolt(CHAN1_DEVICE->drv, raw)
                        : 0.0f;

                /* ---- Switch back to Ch0 for next iteration ---- */
                ad_gpadc_reconfig(h, CHAN0_DEVICE->drv);
                ad_gpadc_read_nof_conv(h, 1, &dummy);          /* S&H settle */

                /* ---- Snapshot temperature ---- */
                float   temp_c   = g_last_temp_c;
                uint8_t humidity = g_last_hum_percent;

#if GPADC_MEASURE_RATE
                sample_count++;
                uint32_t now     = OS_GET_TICK_COUNT();
                uint32_t elapsed = (now >= tick_start)
                        ? (now - tick_start)
                        : (0xFFFFFFFFU - tick_start + now + 1U);
                if (elapsed >= ticks_per_sec) {
                        printf("[RATE] %"PRIu32" sample-pairs/s\r\n", sample_count);
                        sample_count = 0;
                        tick_start   = now;
                }
#endif

#if GPADC_PRINT_SAMPLES
                static uint32_t print_counter = 0;
                if (++print_counter >= GPADC_PRINT_DIVISOR) {
                        print_counter = 0;
                        printf("%d,%d,%d,%d\n",
                                (int)mv0,
                                (int)mv1,
                                (int)(temp_c * 10),
                                (int)humidity);
                }
#else
                (void)mv0; (void)mv1; (void)temp_c; (void)humidity;
#endif
        }

        /* Unreachable — but if ever reached, clean up properly */
        ad_gpadc_close(h, false);
        hw_sys_pd_com_disable();
}
