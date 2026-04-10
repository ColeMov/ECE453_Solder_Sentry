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
#include "source/app_hw/task_captouch.h"
#include "source/app_hw/task_ble.h"

int main(void)
{
    cy_rslt_t rslt;

    /* Initialize the device and board peripherals */
    rslt = cybsp_init() ;
    CY_ASSERT(rslt == CY_RSLT_SUCCESS);

    __enable_irq();

    task_console_init();
    task_print_info("FW: BLE debug build active");

#ifdef COMPONENT_BLESS
    task_print_info("FW: COMPONENT_BLESS is ON");
    task_ble_init();       /* BLE notifications -> phone via Nordic UART Service */
#else
    task_print_warning("BLE: COMPONENT_BLESS not enabled in build");
#endif

    // task_tof_init();  /* ToF distance sensor - uncomment if using VL53Lx */

    task_captouch_init();  /* IQS228B capacitive touch - copper pad */

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler();
    
    for (;;)
    {
    }
}

/* [] END OF FILE */