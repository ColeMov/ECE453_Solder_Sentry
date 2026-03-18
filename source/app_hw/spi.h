/*
 *  Created on: Jan 18, 2022
 *      Author: Joe Krachey
 */

#ifndef SPI_H__
#define SPI_H__

#include "main.h"
#include "ece453_pins.h"

#define SPI_FREQ			1000000

/* Public Global Variables */
extern cyhal_spi_t mSPI;
extern SemaphoreHandle_t Semaphore_SPI;

/* Public API */
cy_rslt_t spi_init(module_site_t module_site);

#endif 
