/**
 ****************************************************************************************
 *
 * @file platform_devices.c
 *
 * @brief Merged platform devices — GPADC (Ch0, Ch1) + I2C master (AHT20)
 *
 * Combines:
 *   - GPADC-2channels project: ADC_CH0_DEVICE, ADC_CH1_DEVICE
 *   - AHT20-i2c_sensor project: I2C_DEVICE_MASTER
 *
 * Copyright (C) 2015-2022 Dialog Semiconductor.
 *
 * -----------------------------------------------------------------------
 * ADC TIMING — what changed and why
 * -----------------------------------------------------------------------
 *
 * ADC_CLK = DivN_clk / 2 = 32 MHz / 2 = 16 MHz
 * One ADC_CLK period = 62.5 ns
 *
 * sample_time field: conversion time = sample_time × 8 ADC_CLK cycles
 *
 *   OLD: sample_time = 4  → 4 × 8 = 32 cycles = 2.0 µs per internal sample
 *   NEW: sample_time = 1  → 1 × 8 =  8 cycles = 0.5 µs per internal sample
 *
 *   Minimum meaningful value is 1 (sample_time = 0 means 0 cycles, which
 *   gives the S&H capacitor no settling time at all — avoid for GPIO inputs).
 *   sample_time = 1 gives 8 ADC_CLK cycles = 500 ns settling time.
 *   For a source impedance ≤ ~10 kΩ this is comfortable (RC << 500 ns).
 *   If your signal conditioning output impedance is higher, increase to 2.
 *
 * oversampling: changed from 4 SAMPLES → 1 SAMPLE (no hardware averaging)
 *
 *   OLD: HW_GPADC_OVERSAMPLING_4_SAMPLES  → 4 internal samples averaged
 *        with chopping=true this becomes 8 internal acquisitions per call
 *        → 8 × 8 cycles × 62.5 ns = 4.0 µs ADC engine time alone
 *
 *   NEW: HW_GPADC_OVERSAMPLING_1_SAMPLE   → 1 internal sample
 *        with chopping=true this becomes 2 internal acquisitions per call
 *        → 2 × 8 cycles × 62.5 ns = 1.0 µs ADC engine time alone
 *
 *   We are doing software RMS over a full mains period (N >> 100 samples),
 *   so hardware averaging buys us nothing useful — the RMS accumulation IS
 *   the averaging.  Removing hardware averaging is safe here.
 *
 * chopping: kept TRUE
 *
 *   Chopping cancels the ADC's own DC offset by taking one sample with the
 *   input polarity normal and one with it reversed, then averaging.  This
 *   costs a factor of ×2 in time but is cheap insurance against systematic
 *   offset errors in your voltage/current measurements.  Keep it.
 *
 * result_mode: kept HW_GPADC_RESULT_NORMAL
 *
 *   NORMAL aligns the result MSB-left in 16 bits with unused LSBs zeroed.
 *   With 1-sample oversampling the hardware ENOB is 10 bits, so the lower
 *   6 bits of the 16-bit result will always be zero — that is expected and
 *   correct.  ad_gpadc_conv_to_mvolt() handles this automatically.
 *
 * Net effect (hardware only, excluding adapter open/close overhead):
 *   OLD: ~4 µs per channel  →  both channels ≈ 8 µs hardware time
 *   NEW: ~1 µs per channel  →  both channels ≈ 2 µs hardware time
 *
 * The dominant overhead at this point becomes the adapter open/close cycle
 * (semaphore acquire + IO reconfigure + LDO settle ≈ several hundred µs).
 * The approach to push past that limit is described in gpadc_app.c.
 *
 ****************************************************************************************
 */

#include "hw_gpio.h"
#include "ad_gpadc.h"
#include "ad_i2c.h"
#include "platform_devices.h"
#include "peripheral_setup.h"   /* I2C pin definitions + I2C_SLAVE_ADDRESS */

/* =======================================================================
 * GPADC — two single-ended channels
 * ======================================================================= */
#if dg_configGPADC_ADAPTER || dg_configUSE_HW_GPADC

/* IO configurations — unchanged */
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

/*
 * Driver configurations
 *
 * Changes from original:
 *   sample_time : 4 → 1   (32 ADC_CLK cycles → 8 ADC_CLK cycles settling)
 *   oversampling: HW_GPADC_OVERSAMPLING_4_SAMPLES → HW_GPADC_OVERSAMPLING_1_SAMPLE
 *
 * Everything else is unchanged.
 */
const ad_gpadc_driver_conf_t drv_conf_ch0 = {
        .input_mode       = HW_GPADC_INPUT_MODE_SINGLE_ENDED,
        .positive         = HW_GPADC_INP_P0_5,
        .temp_sensor      = HW_GPADC_NO_TEMP_SENSOR,
        .sample_time      = 1,                              /* was 4 */
        .continuous       = false,
        .interval         = 0,
        .input_attenuator = HW_GPADC_INPUT_VOLTAGE_UP_TO_3V6,
        .chopping         = true,
        .oversampling     = HW_GPADC_OVERSAMPLING_1_SAMPLE, /* was HW_GPADC_OVERSAMPLING_4_SAMPLES */
        .result_mode      = HW_GPADC_RESULT_NORMAL,
#if HW_GPADC_DMA_SUPPORT
        .dma_setup        = NULL,
#endif
};

const ad_gpadc_driver_conf_t drv_conf_ch1 = {
        .input_mode       = HW_GPADC_INPUT_MODE_SINGLE_ENDED,
        .positive         = HW_GPADC_INP_P0_6,
        .temp_sensor      = HW_GPADC_NO_TEMP_SENSOR,
        .sample_time      = 1,                              /* was 4 */
        .continuous       = false,
        .interval         = 0,
        .input_attenuator = HW_GPADC_INPUT_VOLTAGE_UP_TO_3V6,
        .chopping         = true,
        .oversampling     = HW_GPADC_OVERSAMPLING_1_SAMPLE, /* was HW_GPADC_OVERSAMPLING_4_SAMPLES */
        .result_mode      = HW_GPADC_RESULT_NORMAL,
#if HW_GPADC_DMA_SUPPORT
        .dma_setup        = NULL,
#endif
};

/* Controller configurations — unchanged */
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
 * I2C — master, for AHT20 sensor on MikroBUS #2 (P1_11 SDA, P1_12 SCL)
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
                .address   = I2C_SLAVE_ADDRESS,   /* 0x38 — AHT20 */
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
