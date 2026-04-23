/**
 * @file i2c.c
 * @author Joe Krachey (jkrachey@wisc.edu)
 * @brief 
 * @version 0.1
 * @date 2025-08-06
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "i2c.h"
#include "ece453_pins.h"

cyhal_i2c_t i2c_master_obj;
SemaphoreHandle_t Semaphore_I2C;

// Define the I2C master configuration structure
cyhal_i2c_cfg_t i2c_master_config =
	{
		CYHAL_I2C_MODE_MASTER,
		0, // address is not used for master mode
		I2C_MASTER_FREQUENCY};

/* Saved at i2c_init so i2c_reset_bus can re-init on the same pins. */
static cyhal_gpio_t s_sda_pin = NC;
static cyhal_gpio_t s_scl_pin = NC;
static bool s_i2c_initialized = false;

/** Initialize the I2C bus to the specified module site
 *
 * @param - None
 */
cy_rslt_t i2c_init(module_site_t module_site)
{
	cyhal_gpio_t sda = NC;
	cyhal_gpio_t scl = NC;

	// Set the pins for the I2C interface based on the module site
	switch (module_site)
	{
		case MODULE_SITE_0:
		{
			sda = MOD_0_PIN_I2C_SDA;
			scl = MOD_0_PIN_I2C_SCL;
			break;
		}
		case MODULE_SITE_1:
		{
			sda = MOD_1_PIN_I2C_SDA;
			scl = MOD_1_PIN_I2C_SCL;
			break;
		}
		case MODULE_SITE_2:
		{
			sda = MOD_2_PIN_I2C_SDA;
			scl = MOD_2_PIN_I2C_SCL;
			break;
		}
		default:
		{
			/* Any return value that is not CY_RSLT_SUCCESS can be
			   used to indicate an error */
			return CY_RSLT_MODULE_DRIVER_SCB;
		}
	}

	cy_rslt_t rslt;

	// Initialize I2C master, set the SDA and SCL pins and assign a new clock
	rslt = cyhal_i2c_init(&i2c_master_obj, sda, scl, NULL);

	if (rslt != CY_RSLT_SUCCESS)
	{
		return rslt;
	}

	// Configure the I2C resource to be master
	rslt = cyhal_i2c_configure(&i2c_master_obj, &i2c_master_config);

	if (rslt != CY_RSLT_SUCCESS)
	{
		return rslt;
	}

	/* Create the binary semaphore that will ensure mutual exclusion while using
	   the I2C bus */
	Semaphore_I2C = xSemaphoreCreateBinary();

	/* Need to give the semaphore so that the first task that attempts to take
	   the semaphore is successful */
	xSemaphoreGive(Semaphore_I2C);

	s_sda_pin = sda;
	s_scl_pin = scl;
	s_i2c_initialized = true;

	return CY_RSLT_SUCCESS;
}

cy_rslt_t i2c_reset_bus(void)
{
	cy_rslt_t rslt;

	if (!s_i2c_initialized)
	{
		return CY_RSLT_MODULE_DRIVER_SCB;
	}

	if (Semaphore_I2C != NULL)
	{
		xSemaphoreTake(Semaphore_I2C, portMAX_DELAY);
	}

	cyhal_i2c_free(&i2c_master_obj);

	rslt = cyhal_i2c_init(&i2c_master_obj, s_sda_pin, s_scl_pin, NULL);
	if (rslt == CY_RSLT_SUCCESS)
	{
		rslt = cyhal_i2c_configure(&i2c_master_obj, &i2c_master_config);
	}

	if (Semaphore_I2C != NULL)
	{
		xSemaphoreGive(Semaphore_I2C);
	}

	return rslt;
}
