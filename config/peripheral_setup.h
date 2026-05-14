/**
 ****************************************************************************************
 *
 * @file peripheral_setup.h
 *
 * @brief Pin definitions for I2C (AHT20) — MikroBUS #2
 *        Copied from the AHT20-i2c_sensor project into the combined project.
 *        ADC pin definitions live directly in platform_devices.c.
 *
 ****************************************************************************************
 */

#ifndef CONFIG_PERIPHERAL_SETUP_H_
#define CONFIG_PERIPHERAL_SETUP_H_

#include "hw_gpio.h"

/* Select MikroBUS slot (1 or 2).
 * If using MikroBUS #1, update the SCL/SDA pins accordingly
 * and set this to 1 in your custom_config_*.h:
 *   #define MIKRO_BUS  1
 */
#ifndef MIKRO_BUS
#define MIKRO_BUS 2
#endif

/* I2C pin assignment — MikroBUS #2 */
#define I2C_MASTER_SCL_PORT     ( HW_GPIO_PORT_1 )
#define I2C_MASTER_SCL_PIN      ( HW_GPIO_PIN_12 )

#define I2C_MASTER_SDA_PORT     ( HW_GPIO_PORT_1 )
#define I2C_MASTER_SDA_PIN      ( HW_GPIO_PIN_11 )

/* AHT20 I2C address (7-bit) */
#define I2C_ADDR_AHT20          ( 0x38u )
#define I2C_SLAVE_ADDRESS       ( I2C_ADDR_AHT20 )

#endif /* CONFIG_PERIPHERAL_SETUP_H_ */
