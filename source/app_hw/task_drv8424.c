/**
 * @file task_drv8424.c
 * @author Tanay Gupta (tgupta43@wisc.edu)
 * @brief
 * DRV8424 STEP/DIR stepper motor driver task for FreeRTOS.
 *
 *   STEP    — Each rising edge advances the motor one microstep.
 *   DIR     — Sets direction: high = clockwise, low = counter-clockwise.
 *   nSLEEP  — Active-low sleep control (low = sleep, high = active).
 *   nFAULT  — Open-drain fault output (driven low by DRV8424 on fault).
 *
 * This task accepts commands via a FreeRTOS queue (q_drv8424).  Each command
 * specifies a direction, speed (0-100%), and optional step count (0 = run
 * continuously until a new command arrives).
 *
 * A CLI command "drv8424" is registered so the user can control the motor
 * from the serial console.
 *
 * @version 0.1
 * @date 2026-02-16
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "task_drv8424.h"
#include "task_console.h"

/******************************************************************************/
/* Function Declarations                                                      */
/******************************************************************************/
static void task_drv8424(void *param);

static BaseType_t cli_handler_drv8424(
	char *pcWriteBuffer,
	size_t xWriteBufferLen,
	const char *pcCommandString);

/******************************************************************************/
/* Global Variables                                                           */
/******************************************************************************/
QueueHandle_t q_drv8424;

static const CLI_Command_Definition_t xDRV8424 =
	{
		"drv8424",
		"\r\ndrv8424 < cw | ccw | stop | diag | pulse > [speed] [steps]\r\n"
		"  diag  : read all pin states\r\n"
		"  pulse : wake + 10 slow STEP pulses (50ms each)\r\n"
		"  speed : 0-100 (percent, default 100)\r\n"
		"  steps : number of steps (default 0 = continuous)\r\n",
		cli_handler_drv8424,
		-1};

/******************************************************************************/
/* Static Function Definitions                                                */
/******************************************************************************/

/**
 * @brief Generate a single STEP pulse (rising edge triggers the DRV8424).
 *
 * The DRV8424 latches the DIR pin on the rising edge of STEP.  The pulse
 * width must meet the minimum tSTEP specification (~1.7 us typ).  We hold
 * the pin high for DRV8424_STEP_PULSE_US before driving it low again.
 */
static void generate_step_pulse(void)
{
	cyhal_gpio_write(DRV8424_STEP_PIN, 1u);
	Cy_SysLib_DelayUs(DRV8424_STEP_PULSE_US);
	cyhal_gpio_write(DRV8424_STEP_PIN, 0u);
}

/**
 * @brief Wake the DRV8424 from sleep by asserting nSLEEP high.
 *
 * After nSLEEP goes high the device needs tWAKE before it will accept
 * STEP/DIR commands.
 */
static void drv8424_wake(void)
{
	cyhal_gpio_write(DRV8424_NSLEEP_PIN, 1u);
	vTaskDelay(pdMS_TO_TICKS(DRV8424_TWAKE_MS));
	task_debug_printf(info, "DRV8424: awake (nSLEEP=1)");
}

/**
 * @brief Put the DRV8424 into low-power sleep (nSLEEP low).
 *
 * In sleep mode all H-bridge outputs are disabled and quiescent current
 * drops to the sleep-mode specification.
 */
static void drv8424_sleep(void)
{
	cyhal_gpio_write(DRV8424_NSLEEP_PIN, 0u);
	task_debug_printf(info, "DRV8424: sleep (nSLEEP=0)");
}

/**
 * @brief Set the DIR pin for the desired rotation direction.
 *
 * @param direction DRV8424_CW sets DIR high, DRV8424_CCW sets DIR low.
 */
static void drv8424_set_direction(drv8424_direction_t direction)
{
	if (direction == DRV8424_CW)
	{
		cyhal_gpio_write(DRV8424_DIR_PIN, 1u);
	}
	else
	{
		cyhal_gpio_write(DRV8424_DIR_PIN, 0u);
	}
}

/**
 * @brief Check whether the DRV8424 is reporting a fault.
 *
 * The EN/nFAULT pin is driven low by the DRV8424 when an overcurrent,
 * overtemperature, or undervoltage fault is detected.
 *
 * @return true if a fault is active (pin reads low), false otherwise.
 */
static bool drv8424_check_fault(void)
{
	return (cyhal_gpio_read(DRV8424_NFAULT_PIN) == 0u);
}

/**
 * @brief Convert a speed percentage (0-100) to a step delay in milliseconds.
 *
 * 100% speed maps to the shortest delay (fastest stepping).
 * 0% speed maps to the longest delay (slowest stepping).
 *
 * @param speed_percent Speed as a percentage 0-100.
 * @return Step-to-step delay in milliseconds.
 */
static uint32_t step_delay_from_speed(uint8_t speed_percent)
{
	if (speed_percent == 0u)
	{
		return DRV8424_STEP_DELAY_MAX_MS;
	}

	uint32_t delay_range = DRV8424_STEP_DELAY_MAX_MS - DRV8424_STEP_DELAY_MIN_MS;
	uint32_t scaled = (delay_range * (100u - speed_percent)) / 100u;

	return DRV8424_STEP_DELAY_MIN_MS + scaled;
}

/**
 * @brief Main DRV8424 FreeRTOS task.
 *
 * State machine:
 *   STOPPED — device is in sleep mode, task blocks on queue.
 *   RUNNING — device is awake, task generates STEP pulses at the
 *             commanded rate while polling the queue (non-blocking)
 *             for new commands.  If a finite step count was requested,
 *             the task stops after that many pulses.
 *
 * @param param Unused task parameter.
 */
static void task_drv8424(void *param)
{
	drv8424_message_t current_cmd = {DRV8424_STOP, 0u, 0u};
	drv8424_message_t new_cmd;
	uint32_t step_delay_ms = DRV8424_STEP_DELAY_MAX_MS;
	uint16_t steps_remaining = 0u;
	bool is_running = false;
	bool skip_fault_check = false;

	(void)param;

	/* Start in sleep mode */
	drv8424_sleep();

	for (;;)
	{
		/* ---- STOPPED state ---- */
		if (!is_running)
		{
			/* Block until a command arrives */
			if (xQueueReceive(q_drv8424, &current_cmd, portMAX_DELAY) == pdPASS)
			{
				if (current_cmd.direction == DRV8424_STOP || current_cmd.speed_percent == 0u)
				{
					/* Remain stopped */
					drv8424_sleep();
					task_debug_printf(info, "DRV8424: stop command received");
					continue;
				}

				/* Transition to running */
				drv8424_wake();
				drv8424_set_direction(current_cmd.direction);
				step_delay_ms = step_delay_from_speed(current_cmd.speed_percent);
				steps_remaining = current_cmd.steps;
				is_running = true;
				skip_fault_check = true;

				task_debug_printf(info, "DRV8424: %s at %d%% speed (%lu ms/step)%s",
								 (current_cmd.direction == DRV8424_CW) ? "CW" : "CCW",
								 current_cmd.speed_percent,
								 (unsigned long)step_delay_ms,
								 (current_cmd.steps > 0u) ? "" : " continuous");

				if (current_cmd.steps > 0u)
				{
					task_debug_printf(info, "DRV8424: stepping %u steps", current_cmd.steps);
				}
			}
			continue;
		}

		/* ---- RUNNING state ---- */

		/* Check for a new command (non-blocking) */
		if (xQueueReceive(q_drv8424, &new_cmd, 0) == pdPASS)
		{
			current_cmd = new_cmd;

			if (current_cmd.direction == DRV8424_STOP || current_cmd.speed_percent == 0u)
			{
				drv8424_sleep();
				is_running = false;
				task_debug_printf(info, "DRV8424: stopped");
				continue;
			}

			/* Update parameters on the fly */
			drv8424_set_direction(current_cmd.direction);
			step_delay_ms = step_delay_from_speed(current_cmd.speed_percent);
			steps_remaining = current_cmd.steps;

			task_debug_printf(info, "DRV8424: updated — %s at %d%% speed (%lu ms/step)",
							 (current_cmd.direction == DRV8424_CW) ? "CW" : "CCW",
							 current_cmd.speed_percent,
							 (unsigned long)step_delay_ms);
		}

		/* Check for faults (skip first pass after wake to let DRV8424 stabilize) */
		if (skip_fault_check)
		{
			skip_fault_check = false;
		}
		else if (drv8424_check_fault())
		{
			task_debug_printf(error, "DRV8424: FAULT detected (nFAULT low)!");
			drv8424_sleep();
			is_running = false;
			continue;
		}

		/* Generate one step pulse */
		generate_step_pulse();

		/* Handle finite step count */
		if (current_cmd.steps > 0u)
		{
			if (steps_remaining > 0u)
			{
				steps_remaining--;
			}

			if (steps_remaining == 0u)
			{
				task_debug_printf(info, "DRV8424: completed %u steps", current_cmd.steps);
				drv8424_sleep();
				is_running = false;
				continue;
			}
		}

		/* Wait for the step interval */
		vTaskDelay(pdMS_TO_TICKS(step_delay_ms));
	}
}

/**
 * @brief FreeRTOS CLI handler for the 'drv8424' command.
 *
 * Usage:
 *   drv8424 cw [speed] [steps]   — Step clockwise
 *   drv8424 ccw [speed] [steps]  — Step counter-clockwise
 *   drv8424 stop                 — Stop the motor and enter sleep
 *
 * @param pcWriteBuffer   Buffer to write CLI response text into.
 * @param xWriteBufferLen Length of the write buffer.
 * @param pcCommandString Full command string entered by the user.
 * @return pdFALSE (command completes in one call).
 */
static BaseType_t cli_handler_drv8424(
	char *pcWriteBuffer,
	size_t xWriteBufferLen,
	const char *pcCommandString)
{
	const char *pcParameter;
	drv8424_message_t motor_message;
	BaseType_t xParameterStringLength, xReturn;
	uint8_t speed_percent = 100u;
	uint16_t steps = 0u;
	char parameter_buffer[32];

	(void)xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	xReturn = pdFALSE;

	/* ---- Parameter 1: direction ---- */
	pcParameter = FreeRTOS_CLIGetParameter(
		pcCommandString,
		1,
		&xParameterStringLength);

	if (pcParameter == NULL)
	{
		memset(pcWriteBuffer, 0x00, xWriteBufferLen);
		snprintf(pcWriteBuffer, xWriteBufferLen,
				 "Error: direction required (cw|ccw|stop)\r\n");
		return xReturn;
	}

	memset(parameter_buffer, 0x00, sizeof(parameter_buffer));
	strncpy(parameter_buffer, pcParameter,
			(xParameterStringLength < (BaseType_t)(sizeof(parameter_buffer) - 1u)) ? (size_t)xParameterStringLength : (sizeof(parameter_buffer) - 1u));

	if (strcmp(parameter_buffer, "diag") == 0)
	{
		/* Read and report all pin states */
		memset(pcWriteBuffer, 0x00, xWriteBufferLen);
		snprintf(pcWriteBuffer, xWriteBufferLen,
				 "STEP(P5_4)=%d DIR(P5_5)=%d nSLEEP(P10_4)=%d nFAULT(P10_3)=%d\r\n",
				 (int)cyhal_gpio_read(DRV8424_STEP_PIN),
				 (int)cyhal_gpio_read(DRV8424_DIR_PIN),
				 (int)cyhal_gpio_read(DRV8424_NSLEEP_PIN),
				 (int)cyhal_gpio_read(DRV8424_NFAULT_PIN));
		return pdFALSE;
	}
	else if (strcmp(parameter_buffer, "high") == 0)
	{
		/* Drive STEP pin high and hold — measure with multimeter */
		cyhal_gpio_write(DRV8424_STEP_PIN, 1u);
		memset(pcWriteBuffer, 0x00, xWriteBufferLen);
		snprintf(pcWriteBuffer, xWriteBufferLen,
				 "STEP pin held HIGH. Measure P5_4 now.\r\n");
		return pdFALSE;
	}
	else if (strcmp(parameter_buffer, "low") == 0)
	{
		/* Drive STEP pin low */
		cyhal_gpio_write(DRV8424_STEP_PIN, 0u);
		memset(pcWriteBuffer, 0x00, xWriteBufferLen);
		snprintf(pcWriteBuffer, xWriteBufferLen,
				 "STEP pin driven LOW.\r\n");
		return pdFALSE;
	}
	else if (strcmp(parameter_buffer, "pulse") == 0)
	{
		/* Wake and check fault state while awake */
		cyhal_gpio_write(DRV8424_NSLEEP_PIN, 1u);
		Cy_SysLib_Delay(5);
		int fault_after_wake = (int)cyhal_gpio_read(DRV8424_NFAULT_PIN);
		cyhal_gpio_write(DRV8424_DIR_PIN, 1u);

		memset(pcWriteBuffer, 0x00, xWriteBufferLen);

		if (fault_after_wake == 0)
		{
			snprintf(pcWriteBuffer, xWriteBufferLen,
					 "FAULT while awake! nFAULT=%d. Check VM power & motor wiring.\r\n",
					 fault_after_wake);
			cyhal_gpio_write(DRV8424_NSLEEP_PIN, 0u);
			return pdFALSE;
		}

		snprintf(pcWriteBuffer, xWriteBufferLen,
				 "nSLEEP=1, DIR=1, nFAULT=%d. Pulsing STEP 10x...\r\n",
				 fault_after_wake);

		for (int i = 0; i < 10; i++)
		{
			cyhal_gpio_write(DRV8424_STEP_PIN, 1u);
			Cy_SysLib_Delay(50);
			cyhal_gpio_write(DRV8424_STEP_PIN, 0u);
			Cy_SysLib_Delay(50);
		}
		return pdFALSE;
	}
	else if (strcmp(parameter_buffer, "cw") == 0)
	{
		motor_message.direction = DRV8424_CW;
	}
	else if (strcmp(parameter_buffer, "ccw") == 0)
	{
		motor_message.direction = DRV8424_CCW;
	}
	else if (strcmp(parameter_buffer, "stop") == 0)
	{
		motor_message.direction = DRV8424_STOP;
	}
	else
	{
		memset(pcWriteBuffer, 0x00, xWriteBufferLen);
		snprintf(pcWriteBuffer, xWriteBufferLen,
				 "Error: invalid direction '%s' (cw|ccw|stop)\r\n",
				 parameter_buffer);
		return xReturn;
	}

	/* ---- Parameter 2: speed (optional) ---- */
	pcParameter = FreeRTOS_CLIGetParameter(
		pcCommandString,
		2,
		&xParameterStringLength);

	if (pcParameter != NULL && motor_message.direction != DRV8424_STOP)
	{
		memset(parameter_buffer, 0x00, sizeof(parameter_buffer));
		strncpy(parameter_buffer, pcParameter,
				(xParameterStringLength < (BaseType_t)(sizeof(parameter_buffer) - 1u)) ? (size_t)xParameterStringLength : (sizeof(parameter_buffer) - 1u));

		speed_percent = (uint8_t)atoi(parameter_buffer);

		if (speed_percent > 100u)
		{
			speed_percent = 100u;
		}
	}

	/* ---- Parameter 3: steps (optional) ---- */
	pcParameter = FreeRTOS_CLIGetParameter(
		pcCommandString,
		3,
		&xParameterStringLength);

	if (pcParameter != NULL && motor_message.direction != DRV8424_STOP)
	{
		memset(parameter_buffer, 0x00, sizeof(parameter_buffer));
		strncpy(parameter_buffer, pcParameter,
				(xParameterStringLength < (BaseType_t)(sizeof(parameter_buffer) - 1u)) ? (size_t)xParameterStringLength : (sizeof(parameter_buffer) - 1u));

		steps = (uint16_t)atoi(parameter_buffer);
	}

	/* Force speed to 0 on stop */
	if (motor_message.direction == DRV8424_STOP)
	{
		speed_percent = 0u;
		steps = 0u;
	}

	motor_message.speed_percent = speed_percent;
	motor_message.steps = steps;

	/* Send command to the DRV8424 task */
	if (xQueueSendToBack(q_drv8424, &motor_message, pdMS_TO_TICKS(100)) != pdPASS)
	{
		memset(pcWriteBuffer, 0x00, xWriteBufferLen);
		snprintf(pcWriteBuffer, xWriteBufferLen,
				 "Error: failed to send command to drv8424 queue\r\n");
		return xReturn;
	}

	/* CLI feedback */
	memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	switch (motor_message.direction)
	{
	case DRV8424_CW:
		if (steps > 0u)
		{
			snprintf(pcWriteBuffer, xWriteBufferLen,
					 "DRV8424: cw at %d%% speed, %u steps\r\n",
					 speed_percent, steps);
		}
		else
		{
			snprintf(pcWriteBuffer, xWriteBufferLen,
					 "DRV8424: cw at %d%% speed, continuous\r\n",
					 speed_percent);
		}
		break;
	case DRV8424_CCW:
		if (steps > 0u)
		{
			snprintf(pcWriteBuffer, xWriteBufferLen,
					 "DRV8424: ccw at %d%% speed, %u steps\r\n",
					 speed_percent, steps);
		}
		else
		{
			snprintf(pcWriteBuffer, xWriteBufferLen,
					 "DRV8424: ccw at %d%% speed, continuous\r\n",
					 speed_percent);
		}
		break;
	case DRV8424_STOP:
		snprintf(pcWriteBuffer, xWriteBufferLen,
				 "DRV8424: stopped\r\n");
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
 * @brief Initialize the DRV8424 driver task.
 *
 * Configures GPIO pins for STEP, DIR, nSLEEP (outputs) and nFAULT (input).
 * Creates the command queue, registers the CLI command, and spawns the
 * FreeRTOS task.
 *
 * Call from main() before vTaskStartScheduler().
 */
void task_drv8424_init(void)
{
	cy_rslt_t rslt;
	bool gpio_ok = true;

	/* STEP pin — output, starts low */
	rslt = cyhal_gpio_init(DRV8424_STEP_PIN, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0u);
	if (rslt != CY_RSLT_SUCCESS)
	{
		task_debug_printf(error, "DRV8424: failed to init STEP pin: 0x%08lX", (unsigned long)rslt);
		gpio_ok = false;
	}

	/* DIR pin — output, starts low (CCW default) */
	rslt = cyhal_gpio_init(DRV8424_DIR_PIN, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0u);
	if (rslt != CY_RSLT_SUCCESS)
	{
		task_debug_printf(error, "DRV8424: failed to init DIR pin: 0x%08lX", (unsigned long)rslt);
		gpio_ok = false;
	}

	/* nSLEEP pin — output, starts low (sleep mode) */
	rslt = cyhal_gpio_init(DRV8424_NSLEEP_PIN, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0u);
	if (rslt != CY_RSLT_SUCCESS)
	{
		task_debug_printf(error, "DRV8424: failed to init nSLEEP pin: 0x%08lX", (unsigned long)rslt);
		gpio_ok = false;
	}

	/* nFAULT pin — input with pull-up (open-drain output from DRV8424) */
	rslt = cyhal_gpio_init(DRV8424_NFAULT_PIN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, 1u);
	if (rslt != CY_RSLT_SUCCESS)
	{
		task_debug_printf(error, "DRV8424: failed to init nFAULT pin: 0x%08lX", (unsigned long)rslt);
		gpio_ok = false;
	}

	if (gpio_ok)
	{
		task_debug_printf(info, "DRV8424: GPIO pins initialized");
	}
	else
	{
		task_debug_printf(warning, "DRV8424: one or more GPIO pins failed to init (pin conflict?)");
	}

	/* Create the command queue */
	q_drv8424 = xQueueCreate(1, sizeof(drv8424_message_t));
	if (q_drv8424 == NULL)
	{
		task_debug_printf(error, "DRV8424: failed to create command queue");
		return;
	}

	/* Register the CLI command */
	FreeRTOS_CLIRegisterCommand(&xDRV8424);

	/* Create the motor control task */
	xTaskCreate(
		task_drv8424,
		"Task_DRV8424",
		2 * configMINIMAL_STACK_SIZE,
		NULL,
		configMAX_PRIORITIES - 6,
		NULL);

	task_debug_printf(info, "DRV8424: driver task initialized");
}
