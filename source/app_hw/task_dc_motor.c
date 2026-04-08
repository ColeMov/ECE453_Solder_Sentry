/**
 * @file task_dc_motor.c
 * @author Cole Movsessian (movsessian@wisc.edu)
 * @brief
 * A DC motor control task using FreeRTOS and an H-bridge motor driver (DRV8251).
 * The motor can be controlled via a FreeRTOS CLI command that accepts direction
 * (forward/reverse/stop/brake) and speed (0-100 percent duty cycle).
 *
 * The motor speed is controlled by applying PWM to the active direction control pin.
 * Motor direction is controlled using two logic inputs (IN1, IN2) to the H-bridge:
 *   IN1=0, IN2=0: Coast (disabled, High-Z)
 *   IN1=0, IN2=1: Reverse (PWM on IN2 controls speed)
 *   IN1=1, IN2=0: Forward (PWM on IN1 controls speed)
 *   IN1=1, IN2=1: Brake (low-side slow decay)
 *
 * A FreeRTOS CLI command allows the user to control the motor from a serial console
 * (task_console.c).
 *
 * @version 0.1
 * @date 2026-02-11
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "task_dc_motor.h"
#include "task_console.h"

/******************************************************************************/
/* Function Declarations                                                      */
/******************************************************************************/
static void task_dc_motor(void *param);

static BaseType_t cli_handler_dc_motor(
    char *pcWriteBuffer,
    size_t xWriteBufferLen,
    const char *pcCommandString);

/******************************************************************************/
/* Global Variables                                                           */
/******************************************************************************/
/* Queue used to send commands to control the DC motor */
QueueHandle_t q_dc_motor;

/* PWM objects for motor control (IN1 for forward, IN2 for reverse) */
static cyhal_pwm_t pwm_in1;
static cyhal_pwm_t pwm_in2;

/* The CLI command definition for the dc_motor command */
static const CLI_Command_Definition_t xDCMotor =
    {
        "motor",                                                      /* command text */
        "\r\nmotor < forward | reverse | coast | brake > [speed]\r\n" /* command help text */
        "  speed: 0-100 (percent duty cycle, default 100)\r\n",
        cli_handler_dc_motor,                                        /* The function to run. */
        -1                                                            /* Variable number of parameters */
};

/******************************************************************************/
/* Static Function Definitions                                                */
/******************************************************************************/

/**
 * @brief Updates the motor state based on direction and speed
 *
 * Implements H-bridge control per DRV8251 datasheet:
 *   IN1=0, IN2=0: Coast (High-Z)
 *   IN1=0, IN2=1: Reverse
 *   IN1=1, IN2=0: Forward
 *   IN1=1, IN2=1: Brake
 *
 * @param message Pointer to dc_motor_message_t containing direction and speed
 */
static void update_motor_state(const dc_motor_message_t *message)
{
    cy_rslt_t rslt;

    switch (message->direction)
    {
    case DC_MOTOR_COAST:
    {
        /* Coast mode: IN1=0, IN2=0 (both 0% duty cycle) */
        task_debug_printf(info, "Motor: coasting (High-Z)");
        rslt = cyhal_pwm_set_duty_cycle(&pwm_in1, 0, DC_MOTOR_PWM_FREQUENCY);
        CY_ASSERT(rslt == CY_RSLT_SUCCESS);
        rslt = cyhal_pwm_set_duty_cycle(&pwm_in2, 0, DC_MOTOR_PWM_FREQUENCY);
        CY_ASSERT(rslt == CY_RSLT_SUCCESS);
        break;
    }

    case DC_MOTOR_FORWARD:
    {
        /* Forward mode: IN1 active with PWM for speed, IN2=0 */
        task_debug_printf(info, "Motor: running forward at %d%% speed", message->speed_percent);
        rslt = cyhal_pwm_set_duty_cycle(&pwm_in1, message->speed_percent, DC_MOTOR_PWM_FREQUENCY);
        CY_ASSERT(rslt == CY_RSLT_SUCCESS);
        rslt = cyhal_pwm_set_duty_cycle(&pwm_in2, 0, DC_MOTOR_PWM_FREQUENCY);
        CY_ASSERT(rslt == CY_RSLT_SUCCESS);
        break;
    }

    case DC_MOTOR_REVERSE:
    {
        /* Reverse mode: IN2 active with PWM for speed, IN1=0 */
        task_debug_printf(info, "Motor: running reverse at %d%% speed", message->speed_percent);
        rslt = cyhal_pwm_set_duty_cycle(&pwm_in1, 0, DC_MOTOR_PWM_FREQUENCY);
        CY_ASSERT(rslt == CY_RSLT_SUCCESS);
        rslt = cyhal_pwm_set_duty_cycle(&pwm_in2, message->speed_percent, DC_MOTOR_PWM_FREQUENCY);
        CY_ASSERT(rslt == CY_RSLT_SUCCESS);
        break;
    }

    case DC_MOTOR_BRAKE:
    {
        /* Brake mode: IN1=1, IN2=1 (both 100% duty cycle) */
        task_debug_printf(info, "Motor: braking (low-side slow decay)");
        rslt = cyhal_pwm_set_duty_cycle(&pwm_in1, 100, DC_MOTOR_PWM_FREQUENCY);
        CY_ASSERT(rslt == CY_RSLT_SUCCESS);
        rslt = cyhal_pwm_set_duty_cycle(&pwm_in2, 100, DC_MOTOR_PWM_FREQUENCY);
        CY_ASSERT(rslt == CY_RSLT_SUCCESS);
        break;
    }

    default:
    {
        /* Invalid direction, coast the motor */
        task_debug_printf(warning, "Motor: invalid direction, coasting (High-Z)");
        rslt = cyhal_pwm_set_duty_cycle(&pwm_in1, 0, DC_MOTOR_PWM_FREQUENCY);
        CY_ASSERT(rslt == CY_RSLT_SUCCESS);
        rslt = cyhal_pwm_set_duty_cycle(&pwm_in2, 0, DC_MOTOR_PWM_FREQUENCY);
        CY_ASSERT(rslt == CY_RSLT_SUCCESS);
        break;
    }
    }
}

/**
 * @brief FreeRTOS task that receives motor commands from a queue
 *
 * @param param Task parameter (unused)
 */
static void task_dc_motor(void *param)
{
    dc_motor_message_t motor_message;

    /* Suppress warning for unused parameter */
    (void)param;

    /* Repeatedly running part of the task */
    for (;;)
    {
        /* Check the Queue for motor commands */
        if (xQueueReceive(q_dc_motor, &motor_message, portMAX_DELAY) == pdPASS)
        {
            /* Update motor state based on received message */
            update_motor_state(&motor_message);
        }
    }
}

/**
 * @brief FreeRTOS CLI Handler for the 'motor' command
 *
 * Usage:
 *   motor forward [speed]   - Run motor forward at specified speed (0-100%)
 *   motor reverse [speed]   - Run motor reverse at specified speed (0-100%)
 *   motor coast             - Coast the motor (High-Z)
 *   motor brake [speed]     - Brake the motor (low-side slow decay, optional speed ignored)
 *
 * @param pcWriteBuffer Buffer to write response to
 * @param xWriteBufferLen Length of write buffer
 * @param pcCommandString Full command string entered by user
 *
 * @return pdFALSE if command is complete, pdTRUE if more data to follow
 */
static BaseType_t cli_handler_dc_motor(
    char *pcWriteBuffer,
    size_t xWriteBufferLen,
    const char *pcCommandString)
{
    const char *pcParameter;
    dc_motor_message_t motor_message;
    BaseType_t xParameterStringLength, xReturn;
    uint8_t speed_percent = 100; /* Default speed */
    char parameter_buffer[32];   /* Buffer to hold parameter string */

    /* Remove compile time warnings about unused parameters */
    (void)xWriteBufferLen;
    configASSERT(pcWriteBuffer);

    /* Initialize return value */
    xReturn = pdFALSE;

    /* Get the first parameter (direction) */
    pcParameter = FreeRTOS_CLIGetParameter(
        pcCommandString,        /* The command string itself */
        1,                      /* Return the 1st parameter */
        &xParameterStringLength /* Store the parameter string length */
    );

    if (pcParameter == NULL)
    {
        memset(pcWriteBuffer, 0x00, xWriteBufferLen);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Error: direction required (forward|reverse|stop|brake)\r\n");
        return xReturn;
    }

    /* Copy parameter to buffer for comparison */
    memset(parameter_buffer, 0x00, sizeof(parameter_buffer));
    strncpy(parameter_buffer, pcParameter, 
            (xParameterStringLength < sizeof(parameter_buffer) - 1) ? 
            xParameterStringLength : sizeof(parameter_buffer) - 1);

    /* Parse direction parameter */
    if (strcmp(parameter_buffer, "forward") == 0)
    {
        motor_message.direction = DC_MOTOR_FORWARD;
    }
    else if (strcmp(parameter_buffer, "reverse") == 0)
    {
        motor_message.direction = DC_MOTOR_REVERSE;
    }
    else if (strcmp(parameter_buffer, "brake") == 0)
    {
        motor_message.direction = DC_MOTOR_BRAKE;
    }
    else if (strcmp(parameter_buffer, "coast") == 0)
    {
        motor_message.direction = DC_MOTOR_COAST;
    }
    else
    {
        memset(pcWriteBuffer, 0x00, xWriteBufferLen);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Error: invalid direction '%s' (forward|reverse|coast|brake)\r\n", 
                 parameter_buffer);
        return xReturn;
    }

    /* Get optional speed parameter */
    pcParameter = FreeRTOS_CLIGetParameter(
        pcCommandString,        /* The command string itself */
        2,                      /* Return the 2nd parameter */
        &xParameterStringLength /* Store the parameter string length */
    );

    if (pcParameter != NULL && motor_message.direction != DC_MOTOR_COAST)
    {
        /* Parse speed percentage */
        memset(parameter_buffer, 0x00, sizeof(parameter_buffer));
        strncpy(parameter_buffer, pcParameter,
                (xParameterStringLength < sizeof(parameter_buffer) - 1) ?
                xParameterStringLength : sizeof(parameter_buffer) - 1);

        speed_percent = (uint8_t)atoi(parameter_buffer);

        /* Validate speed range */
        if (speed_percent > 100)
        {
            speed_percent = 100;
        }
    }

    /* For stop command, speed is ignored */
    if (motor_message.direction == DC_MOTOR_COAST)
    {
        speed_percent = 0;
    }

    /* Set the speed in the message */
    motor_message.speed_percent = speed_percent;

    /* Send the message to the motor control task */
    if (xQueueSendToBack(q_dc_motor, &motor_message, pdMS_TO_TICKS(100)) != pdPASS)
    {
        memset(pcWriteBuffer, 0x00, xWriteBufferLen);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Error: failed to send command to motor queue\r\n");
        return xReturn;
    }

    /* Provide feedback about the command */
    memset(pcWriteBuffer, 0x00, xWriteBufferLen);
    switch (motor_message.direction)
    {
    case DC_MOTOR_FORWARD:
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Motor: forward at %d%% speed\r\n", speed_percent);
        break;

    case DC_MOTOR_REVERSE:
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Motor: reverse at %d%% speed\r\n", speed_percent);
        break;

    case DC_MOTOR_COAST:
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Motor: coasting (High-Z)\r\n");
        break;

    default:
        break;
    }

    return xReturn;
}

/******************************************************************************/
/* Public Function Definitions                                                */
/******************************************************************************/

/**
 * @brief Initialize the DC motor task and CLI command
 *
 * Sets up:
 * - PWM outputs on IN1 and IN2 for H-bridge control
 * - FreeRTOS queue for motor commands
 * - FreeRTOS task for motor control
 * - FreeRTOS CLI command handler
 *
 * PWM frequency is set to 1000 Hz, suitable for most DC motor applications.
 */
void task_dc_motor_init(void)
{
    cy_rslt_t rslt;

    /* Initialize PWM for IN1 (forward control) */
    rslt = cyhal_pwm_init(&pwm_in1, DC_MOTOR_IN1_PIN, NULL);
    if(rslt != CY_RSLT_SUCCESS)
    {
        task_debug_printf(error, "Failed to initialize PWM for IN1: %d", rslt);
        return;
    }

    /* Set PWM frequency and duty cycle for IN1 */
    rslt = cyhal_pwm_set_duty_cycle(&pwm_in1, 0, DC_MOTOR_PWM_FREQUENCY);
    if(rslt != CY_RSLT_SUCCESS)
    {
        task_debug_printf(error, "Failed to set duty cycle for IN1: %d", rslt);
        return;
    }

    /* Initialize PWM for IN2 (reverse control) */
    rslt = cyhal_pwm_init(&pwm_in2, DC_MOTOR_IN2_PIN, NULL);
    if(rslt != CY_RSLT_SUCCESS)
    {
        task_debug_printf(error, "Failed to initialize PWM for IN2: %d", rslt);
        return;
    }

    /* Set PWM frequency and duty cycle for IN2 */
    rslt = cyhal_pwm_set_duty_cycle(&pwm_in2, 0, DC_MOTOR_PWM_FREQUENCY);
    if(rslt != CY_RSLT_SUCCESS)
    {
        task_debug_printf(error, "Failed to set duty cycle for IN2: %d", rslt);
        return;
    }

    /* Create the queue for motor control commands */
    q_dc_motor = xQueueCreate(1, sizeof(dc_motor_message_t));
    if (q_dc_motor == NULL)
    {
        task_debug_printf(error, "Failed to create motor command queue");
        return;
    }

    /* Register the CLI command */
    FreeRTOS_CLIRegisterCommand(&xDCMotor);

    /* Create the motor control task */
    xTaskCreate(
        task_dc_motor,
        "Task_DC_Motor",
        configMINIMAL_STACK_SIZE,
        NULL,
        configMAX_PRIORITIES - 6,
        NULL);

    /* Start the PWM outputs */
    rslt = cyhal_pwm_start(&pwm_in1);
    if(rslt != CY_RSLT_SUCCESS)    {
        task_debug_printf(error, "Failed to start PWM for IN1: %d", rslt);
        return;
    }

    rslt = cyhal_pwm_start(&pwm_in2);
    if(rslt != CY_RSLT_SUCCESS)
    {
        task_debug_printf(error, "Failed to start PWM for IN2: %d", rslt);
        return;
    }
}