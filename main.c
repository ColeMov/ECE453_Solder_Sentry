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
#include "source/app_hw/task_tof.h"

int main(void)
{
    cy_rslt_t rslt;

    /* Initialize the device and board peripherals */
    rslt = cybsp_init() ;
    CY_ASSERT(rslt == CY_RSLT_SUCCESS);

    __enable_irq();

    task_console_init();

    //task_tof_init();

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler();
    
    for (;;)
    {
    }
}

/* [] END OF FILE */