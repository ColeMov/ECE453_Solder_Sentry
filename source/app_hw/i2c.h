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
#define I2C_MASTER_FREQUENCY 100000u

/* Public Global Variables */
extern cyhal_i2c_t i2c_master_obj;
extern SemaphoreHandle_t Semaphore_I2C;


/* Public API */

/** Initialize the I2C bus to the specified module site
 *
 * @param - None
 */
cy_rslt_t i2c_init(module_site_t module_site);

#endif /* I2C_H_ */
