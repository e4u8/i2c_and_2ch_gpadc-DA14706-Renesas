/*
 * aht20_task.h
 *
 *  Created on: Feb 27, 2026
 *      Author: User
 */

#ifndef AHT20_TASK_H_
#define AHT20_TASK_H_

#include <stdint.h>

/* Start the AHT20 - I2C task.
 * This creates a FreeRTOS task that reads temperature and humidity values from sensor AHT20 using the I2C protocol,
 * uses 7-bit addresses and prints results.
 * No parameters are required; the function returns immediately (task created).
 */
void aht20_task_start(void);

/*
 * Only the public functions are included. Not the
 * internal static helpers (these are in the .c file)
 * */

#endif /* AHT20_TASK_H_ */
