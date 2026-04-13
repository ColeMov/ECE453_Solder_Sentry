/**
 * @file main.c
 * @author Joe Krachey (jkrachey@wisc.edu)
 * @brief 
 * @version 0.1
 * @date 2024-05-14
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "main.h"
#include "source/app_hw/task_console.h"
#include "source/app_hw/task_blink.h"
#include "source/app_hw/task_ble.h"
#include "source/app_hw/i2c.h"
#include "source/app_hw/task_ir_sensor.h"
#include "source/app_hw/task_servo_ctrl.h"

int main(void)
{
    cy_rslt_t rslt;

    /* Initialize the device and board peripherals */
    rslt = cybsp_init();
    CY_ASSERT(rslt == CY_RSLT_SUCCESS);

    __enable_irq();

    task_console_init();
    task_print_info("FW: Solder Sentry BLE build");

    /* Servos: start before IR so the tracker is ready when frames arrive */
    task_servo_ctrl_init();

    /* I2C must be up before IR sensor task starts */
    rslt = i2c_init(MODULE_SITE_1);
    if (rslt == CY_RSLT_SUCCESS)
    {
        task_ir_sensor_init();
    }
    else
    {
        task_print_warning("I2C init failed (0x%08lX) — IR sensor disabled", (unsigned long)rslt);
    }

#ifdef COMPONENT_BLESS
    task_print_info("FW: COMPONENT_BLESS is ON");
    task_ble_init();
#else
    task_print_warning("BLE: COMPONENT_BLESS not enabled in build");
#endif

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler();
    
    for (;;)
    {
    }
}

/* [] END OF FILE */