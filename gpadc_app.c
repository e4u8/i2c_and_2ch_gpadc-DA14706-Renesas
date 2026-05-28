/*
 * gpadc_app.c
 *
 * GPADC application task for the DA14706.
 * Reads Ch0 and Ch1 sequentially at full speed.
 * Prints a combined terminal line:
 *
 *   Ch0: <raw> | <volts> V    Ch1: <raw> | <volts> V    Temp: <deg> C
 *
 * Temperature is produced by the separate aht20_task (slower, every 2 s).
 * The last measured value is kept in the shared volatile variable
 * g_last_temp_c, declared in aht20_task.c and extern'd here.
 */

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include "osal.h"
#include "ad_gpadc.h"
#include "platform_devices.h"
#include "include/gpadc_app.h"

/* -----------------------------------------------------------------------
 * Shared temperature from aht20_task.c
 * Updated every ~2 s by the AHT20 task.
 * Declared volatile because it is written by a different FreeRTOS task.
 * ----------------------------------------------------------------------- */
extern volatile float g_last_temp_c;
extern volatile uint8_t g_last_hum_percent;

/* -----------------------------------------------------------------------
 * Rate measurement
 * ----------------------------------------------------------------------- */
#define GPADC_MEASURE_RATE   1

/* -----------------------------------------------------------------------
 * Channel aliases
 * ----------------------------------------------------------------------- */
#define CHAN0_DEVICE   ADC_CH0_DEVICE
#define CHAN1_DEVICE   ADC_CH1_DEVICE

/* -----------------------------------------------------------------------
 * Calibration coefficients (empirical, 3.6 V attenuator setting)
 * Replace with measured values when available.
 * ----------------------------------------------------------------------- */
#define OFFSET_MV_CH0    0.0f
#define GAIN_CH0         1.0f

#define OFFSET_MV_CH1    0.0f
#define GAIN_CH1         1.0f

/* -----------------------------------------------------------------------
 * Voltage reference and ADC resolution for Volts output
 * DA14706 GPADC: 16-bit result (after oversampling), VREF = 3600 mV
 * ad_gpadc_conv_to_mvolt() already gives mV; divide by 1000 for Volts.
 * ----------------------------------------------------------------------- */

/* -----------------------------------------------------------------------
 * correct_mv()
 * Applies a two-point linear calibration (offset + gain) to the raw mV
 * value returned by ad_gpadc_conv_to_mvolt().
 * ----------------------------------------------------------------------- */
static float correct_mv(uint32_t mv_uncalibrated, float offset, float gain)
{
        if ((float)mv_uncalibrated < offset) return 0.0f;
        return ((float)mv_uncalibrated - offset) / gain;
}

/* -----------------------------------------------------------------------
 * gpadc_app_init — hardware init is centralised in main.c/prvSetupHardware.
 * ----------------------------------------------------------------------- */
void gpadc_app_init(void) {}

/* -----------------------------------------------------------------------
 * gpadc_app_task
 *
 * Runs forever:
 *   1. Open Ch0 → read → close
 *   2. Open Ch1 → read → close
 *   3. Snapshot the shared temperature (single volatile read = safe on CM33)
 *   4. Print combined line
 * ----------------------------------------------------------------------- */
void gpadc_app_task(void *pvParameters)
{
        static uint16_t raw0;
        static uint16_t raw1;
        static float    mv0;
        static float    mv1;
        static uint16_t dummy;
        int ret = 0;

        /* --- One-time warmup dummy read per channel ---
         * Lets the sample-and-hold capacitor settle after the mux switches.
         */
        ad_gpadc_handle_t h_warm = ad_gpadc_open(CHAN0_DEVICE);
        if (h_warm) {
                ad_gpadc_read_nof_conv(h_warm, 1, &dummy);
                ad_gpadc_close(h_warm, false);
        }
        h_warm = ad_gpadc_open(CHAN1_DEVICE);
        if (h_warm) {
                ad_gpadc_read_nof_conv(h_warm, 1, &dummy);
                ad_gpadc_close(h_warm, false);
        }

        #if GPADC_MEASURE_RATE
                uint32_t pair_count   = 0;
                uint32_t tick_start   = OS_GET_TICK_COUNT();
                uint32_t ticks_per_sec = OS_GET_TICK_FREQ();   /* FreeRTOS configTICK_RATE_HZ */
        #endif
        
        for (;;) {

                /* ---- CH0: Open → Read → Close ---- */
                ad_gpadc_handle_t h0 = ad_gpadc_open(CHAN0_DEVICE);
                if (!h0) {
                        printf("[GPADC] open ch0 failed\r\n");
                        continue;
                }
                ret = ad_gpadc_read_nof_conv(h0, 1, &raw0);
                mv0 = (ret == AD_GPADC_ERROR_NONE)
                        ? correct_mv(ad_gpadc_conv_to_mvolt(CHAN0_DEVICE->drv, raw0),
                                     OFFSET_MV_CH0, GAIN_CH0)
                        : 0.0f;
                ad_gpadc_close(h0, false);

                /* ---- CH1: Open → Read → Close ---- */
                ad_gpadc_handle_t h1 = ad_gpadc_open(CHAN1_DEVICE);
                if (!h1) {
                        printf("[GPADC] open ch1 failed\r\n");
                        continue;
                }
                ret = ad_gpadc_read_nof_conv(h1, 1, &raw1);
                mv1 = (ret == AD_GPADC_ERROR_NONE)
                        ? correct_mv(ad_gpadc_conv_to_mvolt(CHAN1_DEVICE->drv, raw1),
                                     OFFSET_MV_CH1, GAIN_CH1)
                        : 0.0f;
                ad_gpadc_close(h1, false);

                /* ---- Snapshot last temperature (volatile single read) ---- */
                float temp_c = g_last_temp_c;
                uint8_t humidity = g_last_hum_percent;

                /* ---- Integer + fractional split (newlib-nano has no float printf) ---- */
                /* mV → integer millivolt, then split for display */
                /*
                int mv0_int  = (int)mv0;
                int mv1_int  = (int)mv1;

                // Volts from mV
                int v0_int   = mv0_int / 1000;
                int v0_frac  = mv0_int % 1000;

                int v1_int   = mv1_int / 1000;
                int v1_frac  = mv1_int % 1000;

                int t_int    = (int)temp_c;
                int t_frac   = (int)((temp_c - t_int) * 100);  // 2 decimal places
                */

                /* ---- Combined output line ---- */
                /* If I want to see the results from a serial terminal
                printf("Ch0: %5u | %d.%03d V    Ch1: %5u | %d.%03d V    Temp: %d.%02d C\r\n",
                        raw0,  v0_int, v0_frac,
                        raw1,  v1_int, v1_frac,
                        t_int, t_frac);
                */
                /* If I want to see the result from the uart_analyzer */
                printf("%d,%d,%d,%d \n",
                    (int)mv0,
                    (int)mv1,
                    (int)(temp_c * 10),   // e.g. 23.45°C → 234
                    (int)humidity);

                #if GPADC_MEASURE_RATE
                        pair_count++;
                        uint32_t now     = OS_GET_TICK_COUNT();
                        uint32_t elapsed = (now >= tick_start)
                            ? (now - tick_start)
                            : (0xFFFFFFFFU - tick_start + now + 1U);
                        if (elapsed >= ticks_per_sec) {
                                printf("[RATE] %"PRIu32" sample-pairs/s\r\n", pair_count);
                                pair_count = 0;
                                tick_start = now;
                        }
                #endif
        }
}
