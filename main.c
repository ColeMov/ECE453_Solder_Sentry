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
#include "source/app_hw/task_dc_motor.h"
#include "source/app_hw/i2c.h"
#include "source/app_hw/task_ir_sensor.h"
#include "source/app_hw/task_servo_ctrl.h"

int main(void)
{
    cy_rslt_t rslt;

    /* Initialize the device and board peripherals */
    rslt = cybsp_init() ;
    CY_ASSERT(rslt == CY_RSLT_SUCCESS);

    __enable_irq();

    task_console_init();

    task_blink_init();

    task_servo_ctrl_init();

    rslt = i2c_init(MODULE_SITE_1);
    CY_ASSERT(rslt == CY_RSLT_SUCCESS);

    task_ir_sensor_init();

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler();
    
    for (;;)
    {
    }
}

/* [] END OF FILE */