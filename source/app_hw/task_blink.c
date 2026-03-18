/**
 * @file task_blink.c
 * @author Joe Krachey (jkrachey@wisc.edu)
 * @brief
 * The LED found on the CY8CPROTO-063-BLE board will be controlled using a FreeRTOS task
 * and a FreeRTOS timer.
 *
 * The FreeRTOS timer is used to toggle the LED on/off at a rate of 500mS.
 *
 * The FreeRTOS task receives commands from a FreeRTOS queue. Other FreeRTOS tasks can send
 * BLINK_ON or BLINK_OFF commands to enable/disable the FreeRTOS timer that controls the LED.
 *
 * A FreeRTOS CLI command that allows the user to enable/disable the LED from a serial console
 * (task_console.c).
 *
 * @version 0.1
 * @date 2024-07-11
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "task_blink.h"
#include <string.h>

/******************************************************************************/
/* Function Declarations                                                      */
/******************************************************************************/
static void timer_callback_blink(TimerHandle_t xTimer);

static void task_blink(void *param);

static BaseType_t cli_handler_blink(
    char *pcWriteBuffer,
    size_t xWriteBufferLen,
    const char *pcCommandString);

/******************************************************************************/
/* Global Variables                                                           */
/******************************************************************************/
/* Queue used to send commands used to control the status LED */
QueueHandle_t q_blink;

/* Timer Handle used to toggle the status LED on/off */
TimerHandle_t blink_timer;

/* The CLI command definition for the blink command */
static const CLI_Command_Definition_t xBlink =
    {
        "blink",                      /* command text */
        "\r\nblink < on | off >\r\n", /* command help text */
        cli_handler_blink,            /* The function to run. */
        1                             /* The user can enter 1 parameters */
};

/******************************************************************************/
/* Static Function Definitions                                                */
/******************************************************************************/

/* This is the callback function for the blink_timer.  When the FreeRTOS timer
   expires, this function is called to toggle the the status LED on/off.*/
static void timer_callback_blink(TimerHandle_t xTimer)
{
  /* Toggle the LED */
  cyhal_gpio_toggle(PIN_LED);
}

/* This task receives commands from the blink message queue to either enable
   or disable blinking blink_timer.*/
static void task_blink(void *param)
{
  blink_message_type_t message_type;

  /* Suppress warning for unused parameter */
  (void)param;

  /* Repeatedly running part of the task */
  for (;;)
  {
    // Check the Queue.  If nothing was in the queue, we should return pdFALSE
    xQueueReceive(q_blink, &message_type, portMAX_DELAY);

    if (message_type == BLINK_OFF)
    {
      /* Turn the LED off */
      cyhal_gpio_write(PIN_LED, 1);
      xTimerStop(blink_timer, portMAX_DELAY);
    }

    else if (message_type == BLINK_ON)
    {
      xTimerStart(blink_timer, portMAX_DELAY);
    }
  }
}

/* FreeRTOS CLI Handler for the 'blink' command */
static BaseType_t cli_handler_blink(
    char *pcWriteBuffer,
    size_t xWriteBufferLen,
    const char *pcCommandString)
{
  const char *pcParameter;
  blink_message_type_t blink_type;

  BaseType_t xParameterStringLength, xReturn;

  /* Remove compile time warnings about unused parameters, and check the
  write buffer is not NULL.  NOTE - for simplicity, this example assumes the
  write buffer length is adequate, so does not check for buffer overflows. */
  (void)pcCommandString;
  (void)xWriteBufferLen;
  configASSERT(pcWriteBuffer);

  /* Obtain the parameter string. */
  pcParameter = FreeRTOS_CLIGetParameter(
      pcCommandString,        /* The command string itself. */
      1,                      /* Return the 1st parameter. */
      &xParameterStringLength /* Store the parameter string length. */
  );
  /* Sanity check something was returned. */
  configASSERT(pcParameter);

  /* Copy ONLY the parameter to pcWriteBuffer */
  memset(pcWriteBuffer, 0x00, xWriteBufferLen);
  strncat(pcWriteBuffer, pcParameter, xParameterStringLength);

  if (strcmp(pcWriteBuffer, "on") == 0)
  {
    blink_type = BLINK_ON;
  }
  else if (strcmp(pcWriteBuffer, "off") == 0)
  {
    blink_type = BLINK_OFF;
  }
  else
  {
    /* Return a string indicating an invalid parameter. */
    memset(pcWriteBuffer, 0x00, xWriteBufferLen);
    sprintf(pcWriteBuffer, "Error... invalid parameter\n\r");
    xReturn = pdFALSE;
    return xReturn;
  }

  //  Send the message to the blink task
  xQueueSendToBack(q_blink, &blink_type, portMAX_DELAY);

  /* Nothing to return, so zero out the pcWriteBuffer. */
  memset(pcWriteBuffer, 0, xWriteBufferLen);

  /* Indicate that the command has completed */
  xReturn = pdFALSE;

  return xReturn;
}

/******************************************************************************/
/* Public Function Definitions                                                */
/******************************************************************************/
void task_blink_init(void)
{
  cy_rslt_t rslt;

  // Initialize the pin that controls the LED
  rslt = cyhal_gpio_init(PIN_LED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 1);
  CY_ASSERT(CY_RSLT_SUCCESS == rslt);

  /* Create the Queue used to control blinking of the status LED*/
  q_blink = xQueueCreate(1, sizeof(blink_message_type_t));

  /* Register the CLI command */
  FreeRTOS_CLIRegisterCommand(&xBlink);

  /* Create the Blink Timer */
  blink_timer = xTimerCreate(
      "Blink Timer",
      pdMS_TO_TICKS(500),
      true,
      0,
      timer_callback_blink);

  /* Create the task that will control the status LED */
  xTaskCreate(
      task_blink,
      "Task_Blink",
      configMINIMAL_STACK_SIZE,
      NULL,
      configMAX_PRIORITIES - 6,
      NULL);
}