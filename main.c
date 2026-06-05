/**
 ****************************************************************************************
 *
 * @file main.c
 *
 * @brief Combined GPADC (2-channel) + AHT20 Temperature/Humidity Application
 *        DA14706 Renesas � FreeRTOS / SYSCPU context
 *
 * Template base: GPADC-2channels project.
 * Added:         AHT20 I2C task (from AHT20-i2c_sensor project).
 *
 * Two independent FreeRTOS tasks run concurrently:
 *   - gpadc_app_task : reads Ch0 & Ch1 as fast as possible, prints combined line
 *   - aht20_task     : reads temperature every 2 s, updates shared volatile variable
 *
 ****************************************************************************************
 */

#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include "osal.h"
#include "resmgmt.h"
#include "hw_cpm.h"
#include "hw_gpio.h"
#include "hw_watchdog.h"
#include "sys_clock_mgr.h"
#include "sys_power_mgr.h"
#include "hw_sys.h"

/* GPADC adapter */
#include "ad_gpadc.h"

/* I2C adapter  � needed for AHT20 */
#include "ad_i2c.h"

#include "platform_devices.h"   /* merged: ADC_CH0_DEVICE, ADC_CH1_DEVICE, I2C_DEVICE_MASTER */
#include "peripheral_setup.h"   /* pin definitions (copied from AHT20 project) */

/* Application task headers */
#include "include/gpadc_app.h"
#include "include/aht20_task.h"

/* -----------------------------------------------------------------------
 * Task priorities
 * ----------------------------------------------------------------------- */
#define mainGPADC_TASK_PRIORITY     ( OS_TASK_PRIORITY_NORMAL )
/* aht20_task priority is defined inside aht20_task.c (tskIDLE_PRIORITY + 1) */

/* -----------------------------------------------------------------------
 * Retained handles
 * ----------------------------------------------------------------------- */
__RETAINED static OS_TASK prvGPADCTask_h;
static OS_TASK xHandle;

/* -----------------------------------------------------------------------
 * Forward declarations
 * ----------------------------------------------------------------------- */
static void prvSetupHardware(void);

/* -----------------------------------------------------------------------
 * System initialisation task
 * ----------------------------------------------------------------------- */
static void system_init(void *pvParameters)
{
        OS_BASE_TYPE status;

        REG_SETF(GPREG, DEBUG_REG, SYS_CPU_FREEZE_EN, 0);

#if defined CONFIG_RETARGET
        extern void retarget_init(void);
#endif

        cm_sys_clk_init(sysclk_XTAL32M);
        cm_apb_set_clock_divider(apb_div1);
        cm_ahb_set_clock_divider(ahb_div1);
        cm_lp_clk_init();

        prvSetupHardware();

#if defined CONFIG_RETARGET
        retarget_init();
#endif

        /* Sleep mode � keep idle so both tasks can run freely */
        pm_sleep_mode_set(pm_mode_idle);
        pm_set_sys_wakeup_mode(pm_sys_wakeup_mode_fast);

        /* ---- Create GPADC task ---- */
        status = OS_TASK_CREATE("GPADC",
                        gpadc_app_task,
                        NULL,
                        1024 * OS_STACK_WORD_SIZE,
                        mainGPADC_TASK_PRIORITY,
                        prvGPADCTask_h);
        OS_ASSERT(status == OS_TASK_CREATE_SUCCESS);

        /* ---- Create AHT20 task ---- */
        aht20_task_start();   /* defined in aht20_task.c � creates its own FreeRTOS task */

        /* SysInit work is done */
        OS_TASK_DELETE(xHandle);
}

/* -----------------------------------------------------------------------
 * main()
 * ----------------------------------------------------------------------- */
int main(void)
        {
        OS_BASE_TYPE status;

        status = OS_TASK_CREATE("SysInit",
                        system_init,
                        (void *)0,
                        OS_MINIMAL_TASK_STACK_SIZE,
                        OS_TASK_PRIORITY_HIGHEST,
                        xHandle);
        OS_ASSERT(status == OS_TASK_CREATE_SUCCESS);

        OS_TASK_SCHEDULER_RUN();

        for (;;);
}

/* -----------------------------------------------------------------------
 * periph_init � called by pm_system_init(), configures ADC GPIO pins.
 * I2C pins are managed by the adapter (ad_i2c_io_config in prvSetupHardware).
 * ----------------------------------------------------------------------- */
static void periph_init(void)
{
#if (DEVICE_FAMILY == DA1470X)
        hw_gpio_configure_pin(HW_GPIO_PORT_0, HW_GPIO_PIN_5,
                              HW_GPIO_MODE_INPUT, HW_GPIO_FUNC_ADC, false);
        hw_gpio_configure_pin(HW_GPIO_PORT_0, HW_GPIO_PIN_6,
                              HW_GPIO_MODE_INPUT, HW_GPIO_FUNC_ADC, false);
#endif
}

/* -----------------------------------------------------------------------
 * prvSetupHardware � hardware initialisation for GPADC + I2C
 * ----------------------------------------------------------------------- */
static void prvSetupHardware(void)
{
        pm_system_init(periph_init);

        /* Enable the COM power domain before handling GPIO pins */
        hw_sys_pd_com_enable();

        /* Configure GPADC IO in 'off' state (adapter pattern) */
        ad_gpadc_io_config(ADC_CH0_DEVICE->id, ADC_CH0_DEVICE->io, AD_IO_CONF_OFF);
        ad_gpadc_io_config(ADC_CH1_DEVICE->id, ADC_CH1_DEVICE->io, AD_IO_CONF_OFF);

        /* Configure I2C IO in 'off' state (adapter pattern) */
#if dg_configI2C_ADAPTER
        ad_i2c_io_config(((ad_i2c_controller_conf_t *)I2C_DEVICE_MASTER)->id,
                         ((ad_i2c_controller_conf_t *)I2C_DEVICE_MASTER)->io,
                         AD_IO_CONF_OFF);
#endif

        hw_sys_pd_com_disable();
}

/* -----------------------------------------------------------------------
 * FreeRTOS hook functions
 * ----------------------------------------------------------------------- */
OS_APP_MALLOC_FAILED(void) { ASSERT_ERROR(0); }
OS_APP_IDLE(void)          { }
OS_APP_STACK_OVERFLOW(OS_TASK pxTask, char *pcTaskName) { (void)pxTask; (void)pcTaskName; ASSERT_ERROR(0); }
OS_APP_TICK(void)          { }
