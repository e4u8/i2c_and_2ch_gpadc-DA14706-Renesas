/*
 * aht20_task.c
 *
 * Periodically reads temperature from the AHT20 sensor via I2C.
 * Exposes g_last_temp_c as a volatile float that gpadc_app_task reads
 * to include the most recent temperature in its combined print line.
 *
 * Sample rate: every 2000 ms (AHT20 measurement cycle is ~80 ms,
 * so 2 s gives plenty of margin and keeps I2C bus load minimal).
 *
 * The libdriver pattern (DRIVER_AHT20_LINK_* macros) is preserved
 * exactly as in the original AHT20 project.
 */

#include <stdint.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "osal.h"

#include "drivers/aht20/driver_aht20.h"
#include "drivers/aht20/driver_aht20_interface.h"

/* -----------------------------------------------------------------------
 * Shared temperature variable
 *
 * Written only by this task; read (snapshot) by gpadc_app_task.
 * 'volatile' ensures the compiler does not cache the value in a register
 * across tasks. A CM33 32-bit aligned float read/write is atomic on the
 * bus, so no mutex is needed for this single-producer / single-consumer
 * pattern.
 *
 * Initial value 0.0f � gpadc_app_task will print 0.00 C until the first
 * AHT20 reading arrives (~2 s after boot).
 * ----------------------------------------------------------------------- */
volatile float g_last_temp_c = 0.0f;
volatile uint8_t g_last_hum_percent = 0;

/* -----------------------------------------------------------------------
 * Task configuration
 * ----------------------------------------------------------------------- */
#define AHT20_TASK_NAME        "aht20"
#define AHT20_TASK_STACK_SIZE  (configMINIMAL_STACK_SIZE + 512)
#define AHT20_TASK_PRIORITY    (tskIDLE_PRIORITY + 1)   // (OS_TASK_PRIORITY_NORMAL)

#define AHT20_SAMPLE_PERIOD_MS  2000

/* -----------------------------------------------------------------------
 * AHT20 task
 * ----------------------------------------------------------------------- */
static aht20_handle_t aht20_handle;

static void aht20_task(void *pvParameters)
{
        uint8_t  res;
        uint32_t temp_raw;
        float    temp_c;
        uint32_t hum_raw;
        uint8_t  hum_percent;

        /* Link interface functions (libdriver pattern) */
        DRIVER_AHT20_LINK_INIT(&aht20_handle, aht20_handle_t);
        DRIVER_AHT20_LINK_IIC_INIT(&aht20_handle,       aht20_interface_iic_init);
        DRIVER_AHT20_LINK_IIC_DEINIT(&aht20_handle,     aht20_interface_iic_deinit);
        DRIVER_AHT20_LINK_IIC_READ_CMD(&aht20_handle,   aht20_interface_iic_read_cmd);
        DRIVER_AHT20_LINK_IIC_WRITE_CMD(&aht20_handle,  aht20_interface_iic_write_cmd);
        DRIVER_AHT20_LINK_DELAY_MS(&aht20_handle,       aht20_interface_delay_ms);
        DRIVER_AHT20_LINK_DEBUG_PRINT(&aht20_handle,    aht20_interface_debug_print);

        /* Initialise sensor � retry indefinitely on failure */
        res = aht20_init(&aht20_handle);
        if (res != 0) {
                aht20_interface_debug_print("AHT20 init failed: %d\r\n", res);
                for (;;) { vTaskDelay(pdMS_TO_TICKS(2000)); }
        }
        aht20_interface_debug_print("AHT20 initialized OK\r\n");

        for (;;) {
                res = aht20_read_temperature_humidity(
                        &aht20_handle,
                        &temp_raw, &temp_c,
                        &hum_raw,  &hum_percent);

                if (res == 0) {
                        /* Update shared variable */
                        g_last_temp_c = temp_c;
                        g_last_hum_percent = hum_percent;



                        /* Optional: also print raw AHT20 data for debugging.
                         * Comment this out if you only want the combined GPADC line.
                        int t_int  = (int)temp_c;
                        int t_frac = (int)((temp_c - t_int) * 100);
                        aht20_interface_debug_print(
                                "[AHT20] T = %d.%02d C (raw: %lu), H = %u %% (raw: %lu)\r\n",
                                t_int, t_frac,
                                (unsigned long)temp_raw,
                                (unsigned int)hum_percent,
                                (unsigned long)hum_raw);
                        */
                } else {
                        aht20_interface_debug_print("AHT20 read error: %d\r\n", res);
                }

                vTaskDelay(pdMS_TO_TICKS(AHT20_SAMPLE_PERIOD_MS));
        }
}

/* -----------------------------------------------------------------------
 * aht20_task_start � call once from system_init in main.c
 * ----------------------------------------------------------------------- */
void aht20_task_start(void)
{
        BaseType_t ret = xTaskCreate(
                aht20_task,
                AHT20_TASK_NAME,
                AHT20_TASK_STACK_SIZE,
                NULL,
                AHT20_TASK_PRIORITY,
                NULL);
        OS_ASSERT(ret == pdPASS);
}
