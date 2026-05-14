/*
 * driver_aht20_interface.c
 *
 * Implements the libdriver AHT20 interface using the DA1470x SDK I2C adapter.
 *
 * NOTES:
 *  - AHT20 default 7-bit address is 0x38, configured in platform_devices.c.
 *    The 'addr' parameter passed by libdriver is ignored - address is already
 *    set in the I2C device descriptor (drv_i2c_master).
 *  - The adapter is opened and closed around every transaction so the power
 *    manager can enter extended sleep between reads.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>

#include "sdk_defs.h"               /* loads dg_config* flags first */
#include "hw_i2c.h"                 /* defines HW_I2C_F_ADD_STOP */
#include "ad_i2c.h"                 /* now compiles with full I2C adapter API */
#include "osal.h"

#include "driver_aht20_interface.h"
#include "driver_aht20.h"
#include "platform_devices.h"

static ad_i2c_handle_t aht20_i2c_handle = NULL;

/* ---------- libdriver interface implementations ---------- */

uint8_t aht20_interface_iic_init(void)
{
    return 0;   // Adapter life-cycle is managed per-transaction
}

uint8_t aht20_interface_iic_deinit(void)
{
    return 0;   // Adapter life-cycle is managed per-transaction
}

uint8_t aht20_interface_iic_write_cmd(uint8_t addr, uint8_t *buf, uint16_t len)
{
    int rc;

    aht20_i2c_handle = ad_i2c_open(I2C_DEVICE_MASTER);
    if (aht20_i2c_handle == NULL) {
        return 1;
    }

    rc = ad_i2c_write(aht20_i2c_handle, buf, (size_t)len, HW_I2C_F_ADD_STOP);
    ad_i2c_close(aht20_i2c_handle, false);
    aht20_i2c_handle = NULL;

    return (rc == 0) ? 0 : 1;
}

uint8_t aht20_interface_iic_read_cmd(uint8_t addr, uint8_t *buf, uint16_t len)
{
    int rc;

    aht20_i2c_handle = ad_i2c_open(I2C_DEVICE_MASTER);
    if (aht20_i2c_handle == NULL) {
        return 1;
    }

    rc = ad_i2c_read(aht20_i2c_handle, buf, (size_t)len, HW_I2C_F_ADD_STOP);
    ad_i2c_close(aht20_i2c_handle, false);
    aht20_i2c_handle = NULL;

    return (rc == 0) ? 0 : 1;
}

void aht20_interface_delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void aht20_interface_debug_print(const char *const fmt, ...)
{
    va_list args;
    char buf[128];

    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    printf("%s", buf);
}
