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

/* Slower bus for the TOF (Module Site 2) — phantom ACKs every few
 * addresses indicate wire-length / pullup issues; 10 kHz is the slowest
 * reasonable I2C speed and tolerates very dirty buses. */
#define I2C_TOF_FREQUENCY    25000u

/* Public Global Variables */
extern cyhal_i2c_t i2c_master_obj;          /* Module Site 0: IR sensor (P10.0/P10.1) */
extern SemaphoreHandle_t Semaphore_I2C;
extern cyhal_i2c_t i2c_master_obj_site2;    /* Module Site 2: TOF sensor (P6.4/P6.5) */
extern SemaphoreHandle_t Semaphore_I2C_site2;


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
