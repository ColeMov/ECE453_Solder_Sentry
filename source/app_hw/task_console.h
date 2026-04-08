/******************************************************************************
* File Name: uart_debug.h
*
* Description: This file contains the macros that are used for UART based
*              debug.
*
* Related Document: See README.md
*
*******************************************************************************
* (c) 2019-2020, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
* Include guard
*******************************************************************************/
#ifndef __TASK_CONSOLE_H_
#define __TASK_CONSOLE_H_

/*******************************************************************************
* Header files
*******************************************************************************/
#include "main.h"


#define CONSOLE_INTERRUPTS

 /* Macro Definitions */
 #define BAUD_RATE       115200
 #define UART_DELAY      10u
 #define INT_PRIORITY    3
 #define DATA_BITS_8     8
 #define STOP_BITS_1     1

#define CONSOLE_SCB     SCB5
#define CONSOLE_TX_PIN  P5_1
#define CONSOLE_RX_PIN  P5_0

#define DEBUG_ENABLE    (true)

/* Queue length of message queue used for debug message printing */
#define DEBUG_QUEUE_SIZE        (16u)

/* Maximum allowed length of debug message */
#define DEBUG_MESSAGE_MAX_LEN   (100u)

#define MAX_OUTPUT_LENGTH       (1024)
#define MAX_INPUT_LENGTH        (1024)

#define INT_PRIORITY_CONSOLE	3

/* Queue handle for debug message Queue */
extern QueueHandle_t debug_message_q;

/* Debug message type */
typedef enum
{
    none = 0,
    info = 1,
    warning = 2,
    error = 3
} debug_message_type_t;

/* Structure for storing debug message data */
typedef struct
{
    const char* str_ptr;
    debug_message_type_t message_type;
} debug_message_data_t;

typedef struct 
{
    BaseType_t status;
    uint16_t data_len;
    void *data;
} cli_data_rx_t;

extern QueueHandle_t q_cli_data_rx;

/* (true) enables UART based debug and (false) disables it.
 * Note that enabling debug reduces performance, power efficiency and
 * increases code size.
 */


#if (DEBUG_ENABLE)

#define task_print(...)         task_debug_printf(none, __VA_ARGS__)
#define task_print_info(...)    task_debug_printf(info, __VA_ARGS__)
#define task_print_warning(...) task_debug_printf(warning, __VA_ARGS__)
#define task_print_error(...)   task_debug_printf(error, __VA_ARGS__)

/* DebugPrintf is not thread-safe, and should not be used inside task */
#define debug_printf(...)       printf(__VA_ARGS__)

/*******************************************************************************
 * Function prototype
 ******************************************************************************/
void task_debug_printf(debug_message_type_t messageType, char* stringPtr, ...);
void task_console_init(void);
void task_console_tx(void *param);
void task_console_rx(void *param);



/* Declaration of empty or default value macros if the debug is not enabled
 * for efficient code generation.
 */
#else

/*******************************************************************************
 * Function prototype
 ******************************************************************************/
#define task_debug_init(void)
#define task_debug_printf(...)
#define task_print(...)
#define task_print_info(...)
#define task_print_warnig(...)
#define task_print_error(...)
#define debug_printf(...)

#endif /* DEBUG_ENABLE */

#endif /* UART_DEBUG_H_ */
