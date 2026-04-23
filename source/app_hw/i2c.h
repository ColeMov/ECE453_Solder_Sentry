/*
 * i2c.h
 *
 *  Created on: Jan 21, 2022
 *      Author: Joe Krachey
 */

#ifndef __I2C_H__
#define __I2C_H__

#include "main.h"
#include "ece453_pins.h"

/* Macros */
/* 50 kHz: reliable on this board's SDA/SCL wiring; AMG8834 bulk reads
   start NAK'ing at 100 kHz. Bump carefully if wiring is improved. */
#define I2C_MASTER_FREQUENCY 50000u

/* Public Global Variables */
extern cyhal_i2c_t i2c_master_obj;
extern SemaphoreHandle_t Semaphore_I2C;


/* Public API */

/** Initialize the I2C bus to the specified module site
 *
 * @param - None
 */
cy_rslt_t i2c_init(module_site_t module_site);

/** Re-initialize the I2C peripheral on the previously-initialized pins.
 *  Call after a wedged bus error to recover. Safe to call from tasks;
 *  takes the Semaphore_I2C briefly. Returns CY_RSLT_SUCCESS if re-init
 *  succeeded, error otherwise.
 */
cy_rslt_t i2c_reset_bus(void);

#endif /* I2C_H_ */
