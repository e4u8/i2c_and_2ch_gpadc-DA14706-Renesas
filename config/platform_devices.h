/**
 ****************************************************************************************
 *
 * @file platform_devices.h
 *
 * @brief Merged platform devices header � GPADC + I2C
 *
 ****************************************************************************************
 */

#ifndef CONFIG_PLATFORM_DEVICES_H_
#define CONFIG_PLATFORM_DEVICES_H_

#ifndef __CONST
#define __CONST const
#endif

typedef __CONST void* PERIPHERAL_DEVICE;

/* GPADC channel device handles */

/* GPADC device type � always expose so main.c can see ->id and ->io */
#include "ad_gpadc.h"
typedef __CONST ad_gpadc_controller_conf_t* gpadc_device;

#if dg_configGPADC_ADAPTER || dg_configUSE_HW_GPADC
extern gpadc_device ADC_CH0_DEVICE;
extern gpadc_device ADC_CH1_DEVICE;
#endif

/* I2C master device handle (for AHT20) */
#if dg_configI2C_ADAPTER
extern PERIPHERAL_DEVICE I2C_DEVICE_MASTER;
#endif

#endif /* CONFIG_PLATFORM_DEVICES_H_ */
