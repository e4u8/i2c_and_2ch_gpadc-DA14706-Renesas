/**
 ****************************************************************************************
 *
 * @file platform_devices.c
 *
 * @brief Merged platform devices � GPADC (Ch0, Ch1) + I2C master (AHT20)
 *
 * Combines:
 *   - GPADC-2channels project: ADC_CH0_DEVICE, ADC_CH1_DEVICE
 *   - AHT20-i2c_sensor project: I2C_DEVICE_MASTER
 *
 * Copyright (C) 2015-2022 Dialog Semiconductor.
 *
 ****************************************************************************************
 */

#include "hw_gpio.h"
#include "hw_dma.h"
#include "ad_gpadc.h"
#include "ad_i2c.h"
#include "platform_devices.h"
#include "peripheral_setup.h"   /* I2C pin definitions + I2C_SLAVE_ADDRESS */

/* =======================================================================
 * GPADC � two single-ended channels
 * ======================================================================= */
#if dg_configGPADC_ADAPTER || dg_configUSE_HW_GPADC

/* IO configurations */
const ad_gpadc_io_conf_t io_conf_ch0 = {
        .input0 = {
                .port = HW_GPIO_PORT_0,
                .pin  = HW_GPIO_PIN_5,
                .on   = { HW_GPIO_MODE_INPUT, HW_GPIO_FUNC_ADC, true  },
                .off  = { HW_GPIO_MODE_INPUT, HW_GPIO_FUNC_GPIO, false },
        },
        .input1 = {
                .port = HW_GPIO_PORT_NONE,
                .pin  = HW_GPIO_PIN_NONE,
        },
        .voltage_level = HW_GPIO_POWER_VDD1V8P,
};

const ad_gpadc_io_conf_t io_conf_ch1 = {
        .input0 = {
                .port = HW_GPIO_PORT_0,
                .pin  = HW_GPIO_PIN_6,
                .on   = { HW_GPIO_MODE_INPUT, HW_GPIO_FUNC_ADC, true  },
                .off  = { HW_GPIO_MODE_INPUT, HW_GPIO_FUNC_GPIO, false },
        },
        .input1 = {
                .port = HW_GPIO_PORT_NONE,
                .pin  = HW_GPIO_PIN_NONE,
        },
        .voltage_level = HW_GPIO_POWER_VDD1V8P,
};

/* DMA config — shared by both channels (GPADC is a single hardware resource;
 * ch0 and ch1 are never active simultaneously so sharing is correct).
 * channel must be EVEN (hardware constraint on the GPADC trigger mux). */
#if HW_GPADC_DMA_SUPPORT
static gpadc_dma_cfg dma_cfg_adc = {
        .channel         = HW_DMA_CHANNEL_0,
        .prio            = HW_DMA_PRIO_2,
        .circular        = false,
        .irq_nr_of_trans = 0,
};
#endif

/* Driver configurations */
const ad_gpadc_driver_conf_t drv_conf_ch0 = {
        .input_mode       = HW_GPADC_INPUT_MODE_SINGLE_ENDED,
        .positive         = HW_GPADC_INP_P0_5,
        .temp_sensor      = HW_GPADC_NO_TEMP_SENSOR,
        .sample_time      = 4,
        .continuous       = true,
        .interval         = 0,
        .input_attenuator = HW_GPADC_INPUT_VOLTAGE_UP_TO_3V6,
        .chopping         = true,
        .oversampling     = HW_GPADC_OVERSAMPLING_4_SAMPLES,
        .result_mode      = HW_GPADC_RESULT_NORMAL,
#if HW_GPADC_DMA_SUPPORT
        .dma_setup        = &dma_cfg_adc,
#endif
};

const ad_gpadc_driver_conf_t drv_conf_ch1 = {
        .input_mode       = HW_GPADC_INPUT_MODE_SINGLE_ENDED,
        .positive         = HW_GPADC_INP_P0_6,
        .temp_sensor      = HW_GPADC_NO_TEMP_SENSOR,
        .sample_time      = 4,
        .continuous       = true,
        .interval         = 0,
        .input_attenuator = HW_GPADC_INPUT_VOLTAGE_UP_TO_3V6,
        .chopping         = true,
        .oversampling     = HW_GPADC_OVERSAMPLING_4_SAMPLES,
        .result_mode      = HW_GPADC_RESULT_NORMAL,
#if HW_GPADC_DMA_SUPPORT
        .dma_setup        = &dma_cfg_adc,
#endif
};

/* Controller configurations */
const ad_gpadc_controller_conf_t conf_ch0 = {
        .id  = HW_GPADC_1,
        .io  = &io_conf_ch0,
        .drv = &drv_conf_ch0,
};

const ad_gpadc_controller_conf_t conf_ch1 = {
        .id  = HW_GPADC_1,
        .io  = &io_conf_ch1,
        .drv = &drv_conf_ch1,
};

gpadc_device ADC_CH0_DEVICE = &conf_ch0;
gpadc_device ADC_CH1_DEVICE = &conf_ch1;

#endif /* dg_configGPADC_ADAPTER || dg_configUSE_HW_GPADC */


/* =======================================================================
 * I2C � master, for AHT20 sensor on MikroBUS #2 (P1_11 SDA, P1_12 SCL)
 * ======================================================================= */
#if (dg_configI2C_ADAPTER && dg_configUSE_HW_I2C)

static __CONST ad_i2c_io_conf_t io_i2c_master = {
        .scl = {
                .port = I2C_MASTER_SCL_PORT,
                .pin  = I2C_MASTER_SCL_PIN,
                .on   = { HW_GPIO_MODE_OUTPUT_OPEN_DRAIN, HW_GPIO_FUNC_I2C_SCL, false },
                .off  = { HW_GPIO_MODE_INPUT,             HW_GPIO_FUNC_GPIO,    false },
        },
        .sda = {
                .port = I2C_MASTER_SDA_PORT,
                .pin  = I2C_MASTER_SDA_PIN,
                .on   = { HW_GPIO_MODE_OUTPUT_OPEN_DRAIN, HW_GPIO_FUNC_I2C_SDA, false },
                .off  = { HW_GPIO_MODE_INPUT,             HW_GPIO_FUNC_GPIO,    false },
        },
        .voltage_level = HW_GPIO_POWER_VDD1V8P,
};

static __CONST ad_i2c_driver_conf_t drv_i2c_master = {
        .i2c = {
                .speed     = HW_I2C_SPEED_STANDARD,
                .mode      = HW_I2C_MODE_MASTER,
                .addr_mode = HW_I2C_ADDRESSING_7B,
                .address   = I2C_SLAVE_ADDRESS,   /* 0x38 � AHT20 */
        },
#if (MAIN_PROCESSOR_BUILD)
        .dma_channel = HW_DMA_CHANNEL_INVALID,
#endif
};

__CONST ad_i2c_controller_conf_t dev_i2c_master = {
        .id  = HW_I2C1,
        .io  = &io_i2c_master,
        .drv = &drv_i2c_master,
};

PERIPHERAL_DEVICE I2C_DEVICE_MASTER = &dev_i2c_master;

#endif /* dg_configI2C_ADAPTER && dg_configUSE_HW_I2C */
