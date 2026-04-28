/******************************************************************************
 * File Name: uart_debug.c
 *
 * Description: This file contains the task that is used for thread-safe UART
 *              based debug.
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
 * Include header files
 ******************************************************************************/
#include "task_console.h"
#include "task_ble.h"
#include "FreeRTOS_CLI.h"
#include <string.h>

//*******************************************************************************
// The console will be made up of the UART ISR along with two FreeRTOS
// tasks.
//
// The UART ISR will buffer data entered by the user into an array of characters.
// When the user presses the ENTER key, a Task Notification will be sent to the
// the task_console_rx task.  The only interrupt that will trigger the UART ISR
// will be CYHAL_UART_IRQ_RX_NOT_EMPTY.
//
// task_console_rx will parse the string entered by the user.  This will be
// accomplished using FreeRTOS-CLI.  Once the string has been parsed,
// CYHAL_UART_IRQ_RX_NOT_EMPTY interrupts will be re-enabled.
//
// task_console_tx will be used to print all messages to the console.  This task
// will wait for messages to arrive in its message queue.  It will then print
// out the messages in the order that they were received.  Transmitting data to
// the console will be accomplished using the retarget-io libraries supplied
// by the HAL.
//*******************************************************************************

/*******************************************************************************
 * Macros
 ******************************************************************************/

/*******************************************************************************
 * External Global Variables
 ******************************************************************************/

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
char pcOutputString[MAX_OUTPUT_LENGTH];
char pcInputString[MAX_INPUT_LENGTH];
int8_t cInputIndex = 0;

cyhal_uart_t console_uart_obj;

/* Initialize the UART configuration structure */
const cyhal_uart_cfg_t console_uart_config =
    {
        .data_bits = DATA_BITS_8,
        .stop_bits = STOP_BITS_1,
        .parity = CYHAL_UART_PARITY_NONE,
        .rx_buffer = NULL,
        .rx_buffer_size = 0};

QueueHandle_t q_console_tx;
QueueHandle_t q_cli_data_rx;
TaskHandle_t Task_Console_Rx_Handle;

/*******************************************************************************
 * Function Name: ece453_console_event_handler
 ********************************************************************************
 * Summary:
 * UART Handler used to receive characters from the console.
 *
 * Parameters:
 *  void
 *
 * Return:
 *
 *
 *******************************************************************************/
void console_event_handler(void *handler_arg, cyhal_uart_event_t event)
{
    (void)handler_arg;
    uint32_t value;
    cy_rslt_t status;
    char c;
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    if ((event & CYHAL_UART_IRQ_TX_ERROR) == CYHAL_UART_IRQ_TX_ERROR)
    {
        /* An error occurred in Tx */
        /* Insert application code to handle Tx error */
    }
    else if ((event & CYHAL_UART_IRQ_RX_NOT_EMPTY) == CYHAL_UART_IRQ_RX_NOT_EMPTY)
    {

        // Received a character.
        // ADD CODE to receive a character using the HAL UART API
        status = cyhal_uart_getc(&console_uart_obj, (uint8_t *)&value, 1);

        c = (char)value;

        if (status == CY_RSLT_SUCCESS)
        {
            // Echo out the character
            // ADD CODE to transmit a character using the HAL UART API
            cyhal_uart_putc(&console_uart_obj, value);

            // If the ISR detects that the user has pressed the ENTER key,
            // Send a task notification to Task_Console
            if (c == '\n' || c == '\r')
            {
                cyhal_uart_enable_event(
                    &console_uart_obj,                                 // retarget-io uart object
                    (cyhal_uart_event_t)(CYHAL_UART_IRQ_RX_NOT_EMPTY), // Events to enable/disable
                    INT_PRIORITY_CONSOLE,                              // Priority
                    false);

                vTaskNotifyGiveFromISR(Task_Console_Rx_Handle, &xHigherPriorityTaskWoken);
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
            else
            {
                if (c == '\b')
                {
                    // backspace character, so delete the previous character in the array
                    cInputIndex--;
                    pcInputString[cInputIndex] = 0;
                }
                else
                {
                    pcInputString[cInputIndex] = c;
                    cInputIndex++;
                }
            }
        }
    }
}

/*******************************************************************************
 * Function Name: task_console_tx
 ********************************************************************************
 * Summary:
 *  This task is responsible for printing ALL messages to the console.  It
 *  assumes that the retarget-io library is being utilized to send data
 *  to the UART
 *
 * Parameters:
 *  void *param : Task parameter defined during task creation (unused)
 *
 *******************************************************************************/
void task_console_tx(void *param)
{
    debug_message_data_t message_data;

    /* Variable used to store the return values of RTOS APIs */
    BaseType_t rtos_api_result;

    /* Remove warning for unused parameter */
    (void)param;

    /* Repeatedly running part of the task */
    for (;;)
    {
        rtos_api_result = xQueueReceive(q_console_tx, &message_data,
                                        portMAX_DELAY);

        if (rtos_api_result == pdPASS)
        {
            switch (message_data.message_type)
            {
            case none:
            {
                Cy_SCB_UART_PutString(CONSOLE_SCB, (char *)message_data.str_ptr);
                break;
            }
            case info:
            {
                Cy_SCB_UART_PutString(CONSOLE_SCB, "\033[0;32m[Info   ]\033[0m    : ");
                Cy_SCB_UART_PutString(CONSOLE_SCB, (char *)message_data.str_ptr);
                Cy_SCB_UART_PutString(CONSOLE_SCB, "\n\r");
                break;
            }
            case warning:
            {
                Cy_SCB_UART_PutString(CONSOLE_SCB, "\033[0;33m[WARNING]\033[0m    : ");
                Cy_SCB_UART_PutString(CONSOLE_SCB, (char *)message_data.str_ptr);
                Cy_SCB_UART_PutString(CONSOLE_SCB, "\n\r");
                break;
            }
            case error:
            {
                
                Cy_SCB_UART_PutString(CONSOLE_SCB, "\033[0;31m[ERROR  ]\033[0m    : ");
                Cy_SCB_UART_PutString(CONSOLE_SCB, (char *)message_data.str_ptr);
                Cy_SCB_UART_PutString(CONSOLE_SCB, "\n\r");
                break;
            }
            default:
            {
                break;
            }
            }

            /* free the message buffer allocated by the message sender */
            vPortFree((char *)message_data.str_ptr);
        }
        /* Task has timed out and received no commands during an interval of
         * portMAXDELAY ticks
         */
        else
        {
        }
    }
}

/*******************************************************************************
 * Function Name: task_console_rx
 ********************************************************************************
 * Summary:
 *  This task is responsible for receiving data from the console and parsing the
 *  commands using FreeRTOS-CLI
 * Parameters:
 *  void *param : Task parameter defined during task creation (unused)
 *
 *******************************************************************************/
void task_console_rx(void *param)
{
    BaseType_t xMoreDataToFollow;

    /* Preserve pre-scheduler boot logs; skip the clear-screen escape. */
    task_print("\n\r");
    task_print("********************************\n\r");
    task_print("* ECE453 Command Line Interface\n\r");
    task_print("* Date: %s\n\r", __DATE__);
    task_print("* Time: %s\n\r", __TIME__);
    task_print("********************************\n\r");
    task_print("-> ");
    while (1)
    {
        // Wait for ISR to let us know that a new command has arrived
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        do
        {
            /* Send the command string to the command interpreter.  Any
            output generated by the command interpreter will be placed in the
            pcOutputString buffer. */
            xMoreDataToFollow = FreeRTOS_CLIProcessCommand(
                pcInputString,    /* The command string.*/
                pcOutputString,   /* The output buffer. */
                MAX_OUTPUT_LENGTH /* The size of the output buffer. */
            );

            /* Write the output generated by the command interpreter to the console. */
            task_print(pcOutputString);

        } while (xMoreDataToFollow != pdFALSE);

        task_print("\n\r-> ");

        /* All the strings generated by the input command have been sent.
        Processing of the command is complete.  Clear the input string ready
        to receive the next command. */
        cInputIndex = 0;
        memset(pcInputString, 0x00, MAX_INPUT_LENGTH);

        // Turn Console Rx Interrupts back on.
        cyhal_uart_enable_event(
            &console_uart_obj,                          // retarget-io uart object
            (cyhal_uart_event_t)(CYHAL_UART_IRQ_RX_NOT_EMPTY), // Events to enable/disable
            INT_PRIORITY_CONSOLE,                              // Priority
            true);                                             // Enable events
    }
}

/*******************************************************************************
 * Function Name: task_debug_printf
 ********************************************************************************
 * Summary:
 *  This function sends messages to the debug Queue.
 *
 *******************************************************************************/
void task_debug_printf(debug_message_type_t message_type, char *str_ptr, ...)
{
    debug_message_data_t message_data;
    char *message_buffer;
    char *task_name;
    uint32_t length = 0;
    va_list args;

    /* Allocate the message buffer */
    message_buffer = pvPortMalloc(DEBUG_MESSAGE_MAX_LEN);

    if (message_buffer)
    {
        va_start(args, str_ptr);
        if (message_type != none)
        {
            task_name = pcTaskGetName(xTaskGetCurrentTaskHandle());
            length = snprintf(message_buffer, DEBUG_MESSAGE_MAX_LEN, "%-16s : ",
                              task_name);
        }

        vsnprintf((message_buffer + length), (DEBUG_MESSAGE_MAX_LEN - length),
                  str_ptr, args);

        va_end(args);

        message_data.message_type = message_type;
        message_data.str_ptr = message_buffer;

        /* Forward a copy to the BLE notification queue (non-blocking, best-effort).
         * Append "\n" so the desktop can split notification packets back into
         * lines — without a delimiter the line buffer accumulates forever and
         * regex telemetry parsing (tof/fan/track/...) never fires. */
#ifdef COMPONENT_BLESS
        if (q_ble_tx != NULL && message_type != none)
        {
            ble_tx_item_t ble_item;
            size_t copy_max = sizeof(ble_item.msg) - 2u;
            strncpy(ble_item.msg, message_buffer, copy_max);
            ble_item.msg[copy_max] = '\0';
            size_t copied = strlen(ble_item.msg);
            ble_item.msg[copied] = '\n';
            ble_item.msg[copied + 1u] = '\0';
            xQueueSendToBack(q_ble_tx, &ble_item, 0u);
        }
#endif /* COMPONENT_BLESS */

        /* The receiver task is responsible to free the memory from here on */
        if (pdPASS != xQueueSendToBack(q_console_tx, &message_data, 0u))
        {
            /* Failed to send the message into the queue */
            vPortFree(message_buffer);
        }
    }
    else
    {
        /* pvPortMalloc failed. Handle error */
    }
}

/*******************************************************************************
 * Function Name: task_console_init
 ********************************************************************************
 * Summary:
 *  Initializes the underlying Task and Queue used for printing debug
 *  messages.
 *
 *******************************************************************************/
void hw_console_init(void)
{
    cy_rslt_t rslt;
    uint32_t actual_baud;

    rslt = cyhal_uart_init(
        &console_uart_obj,
        CONSOLE_TX_PIN,
        CONSOLE_RX_PIN,
        NC,
        NC,
        NULL,
        &console_uart_config);
    CY_ASSERT(rslt == CY_RSLT_SUCCESS);

    // Set the Baud Rate
    rslt = cyhal_uart_set_baud(&console_uart_obj, BAUD_RATE, &actual_baud);
    CY_ASSERT(rslt == CY_RSLT_SUCCESS);

    // Register the Uart Interrupt Handler
    cyhal_uart_register_callback(&console_uart_obj, console_event_handler, NULL);

    // Turn on Rx interrupts
    cyhal_uart_enable_event(
        &console_uart_obj,                                 // uart object
        (cyhal_uart_event_t)(CYHAL_UART_IRQ_RX_NOT_EMPTY), // Events to enable/disable
        INT_PRIORITY_CONSOLE,                              // Priority
        true);
}

/*******************************************************************************
 * Function Name: task_console_init
 ********************************************************************************
 * Summary:
 *  Initializes the underlying Task and Queue used for printing debug
 *  messages.
 *
 *******************************************************************************/
static BaseType_t cli_handler_ble_reset(char *pcWriteBuffer,
                                        size_t xWriteBufferLen,
                                        const char *pcCommandString)
{
    (void)pcCommandString;
    configASSERT(pcWriteBuffer);
    snprintf(pcWriteBuffer, xWriteBufferLen, "Resetting MCU...\r\n");
    vTaskDelay(pdMS_TO_TICKS(50u));
    NVIC_SystemReset();
    return pdFALSE;
}

static const CLI_Command_Definition_t xBleReset =
{
    "ble_reset",
    "\r\nble_reset : full MCU reset (mirrors hardware reset button)\r\n",
    cli_handler_ble_reset,
    0
};

void task_console_init(void)
{
    hw_console_init();

    // Set up the Queue for task_console_tx
    q_console_tx = xQueueCreate(DEBUG_QUEUE_SIZE,
                                   sizeof(debug_message_data_t));

    FreeRTOS_CLIRegisterCommand(&xBleReset);


    // Create the transmit task
    xTaskCreate(
        task_console_tx,
        "Console Tx",
        5*configMINIMAL_STACK_SIZE,
        NULL,
        configMAX_PRIORITIES - 6,
        NULL);

    // Create the receive task
    xTaskCreate(
        task_console_rx,
        "ConsoleRx",
        5*configMINIMAL_STACK_SIZE,
        NULL,
        configMAX_PRIORITIES - 6,
        &Task_Console_Rx_Handle);
};
