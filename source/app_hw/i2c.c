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
cyhal_i2c_t i2c_master_obj_site2;
SemaphoreHandle_t Semaphore_I2C_site2;

// Define the I2C master configuration structure
cyhal_i2c_cfg_t i2c_master_config =
	{
		CYHAL_I2C_MODE_MASTER,
		0, // address is not used for master mode
		I2C_MASTER_FREQUENCY};

static const cyhal_i2c_cfg_t i2c_master_config_tof =
	{
		CYHAL_I2C_MODE_MASTER,
		0,
		I2C_TOF_FREQUENCY};

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
	cyhal_i2c_t *handle = NULL;
	SemaphoreHandle_t *sem = NULL;

	switch (module_site)
	{
		case MODULE_SITE_0:
		{
			sda = MOD_0_PIN_I2C_SDA;
			scl = MOD_0_PIN_I2C_SCL;
			handle = &i2c_master_obj;
			sem    = &Semaphore_I2C;
			break;
		}
		case MODULE_SITE_1:
		{
			/* Module Site 1 (P9.0/P9.1) is owned by task_audio's private
			 * cyhal_i2c_t. Don't double-init from here. */
			return CY_RSLT_MODULE_DRIVER_SCB;
		}
		case MODULE_SITE_2:
		{
			sda = MOD_2_PIN_I2C_SDA;
			scl = MOD_2_PIN_I2C_SCL;
			handle = &i2c_master_obj_site2;
			sem    = &Semaphore_I2C_site2;
			break;
		}
		default:
			return CY_RSLT_MODULE_DRIVER_SCB;
	}

	cy_rslt_t rslt;

	rslt = cyhal_i2c_init(handle, sda, scl, NULL);
	if (rslt != CY_RSLT_SUCCESS)
	{
		return rslt;
	}

	const cyhal_i2c_cfg_t *cfg =
		(module_site == MODULE_SITE_2) ? &i2c_master_config_tof : &i2c_master_config;
	rslt = cyhal_i2c_configure(handle, cfg);
	if (rslt != CY_RSLT_SUCCESS)
	{
		return rslt;
	}

	*sem = xSemaphoreCreateBinary();
	xSemaphoreGive(*sem);

	/* Track only the legacy single-bus state (Module Site 0) here so
	 * i2c_reset_bus() keeps recovering the IR-side bus on errors. */
	if (module_site == MODULE_SITE_0)
	{
		s_sda_pin = sda;
		s_scl_pin = scl;
		s_i2c_initialized = true;
	}

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
