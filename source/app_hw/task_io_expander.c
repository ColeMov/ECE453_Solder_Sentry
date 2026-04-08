/**
 * @file io_expander.c
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-09-01
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "task_io_expander.h"

/******************************************************************************/
/* Function Declarations                                                      */
/******************************************************************************/
static void task_io_expander(void *param);

static BaseType_t cli_handler_ioxp(
	char *pcWriteBuffer,
	size_t xWriteBufferLen,
	const char *pcCommandString);

/******************************************************************************/
/* Global Variables                                                           */
/******************************************************************************/
/* Queue used to send commands used to io expander */
QueueHandle_t q_io_expander_req;
QueueHandle_t q_io_expander_rsp;

/* The CLI command definition for the ioxp command */
static const CLI_Command_Definition_t cmd_ioxp =
	{
		"ioxp",							   /* command text */
		"\r\nioxp <r|w> <addr> <val>\r\n", /* command help text */
		cli_handler_ioxp,				   /* The function to run. */
		-1								   /* The user can enter any number of parameters */
};

/******************************************************************************/
/* Static Function Definitions                                                */
/******************************************************************************/

/** Write a register on the TCA9534
 *
 * @param reg The reg address to read
 * @param value The value to be written
 *
 */
static void io_expander_write_reg(uint8_t reg, uint8_t value)
{
	cy_rslt_t rslt;
	uint8_t write_buffer[2];

	write_buffer[0] = reg;
	write_buffer[1] = value;

	/* There may be multiple I2C devices on the same bus, so we must take
	   the semaphore used to ensure mutual exclusion before we begin any
	   I2C transactions
	 */
	xSemaphoreTake(Semaphore_I2C, portMAX_DELAY);

	/* Use cyhal_i2c_master_write to write the required data to the device. */
	rslt = cyhal_i2c_master_write(
		&i2c_master_obj,		  // I2C Object
		TCA9534_SUBORDINATE_ADDR, // I2C Address
		write_buffer,			  // Array of data to write
		2,						  // Number of bytes to write
		0,						  // Block until completed
		true);					  // Generate Stop Condition

	if (rslt != CY_RSLT_SUCCESS)
	{
		task_print_error("Error writing to the IO Expander");
	}

	/* Give up control of the I2C bus */
	xSemaphoreGive(Semaphore_I2C);
}

/** Read a register on the TCA9534
 *
 * @param reg The reg address to read
 *
 */
static uint8_t io_expander_read_reg(uint8_t reg)
{
	cy_rslt_t rslt;

	uint8_t write_buffer[1];
	uint8_t read_buffer[1];

	write_buffer[0] = reg;

	/* There may be multiple I2C devices on the same bus, so we must take
	   the semaphore used to ensure mutual exclusion before we begin any
	   I2C transactions
	 */
	xSemaphoreTake(Semaphore_I2C, portMAX_DELAY);

	/* Use cyhal_i2c_master_write to write the required data to the device. */
	/* Send the register address, do not generate a stop condition.  This will result in */
	/* a restart condition. */
	rslt = cyhal_i2c_master_write(
		&i2c_master_obj,
		TCA9534_SUBORDINATE_ADDR, // I2C Address
		write_buffer,			  // Array of data to write
		1,						  // Number of bytes to write
		0,						  // Block until completed
		false);					  // Do NOT generate Stop Condition

	if (rslt != CY_RSLT_SUCCESS)
	{
		task_print_error("Error writing reg address to the IO Expander");
	}
	else
	{

		/* Use cyhal_i2c_master_read to read the required data from the device. */
		// The register address has already been set in the write above, so read a single byte
		// of data.
		rslt = cyhal_i2c_master_read(
			&i2c_master_obj,		  // I2C Object
			TCA9534_SUBORDINATE_ADDR, // I2C Address
			read_buffer,			  // Read Buffer
			1,						  // Number of bytes to read
			0,						  // Block until completed
			true);					  // Generate Stop Condition

		if (rslt != CY_RSLT_SUCCESS)
		{
			task_print_error("Error Reading from the IO Expander");
			read_buffer[0] = 0xFF;
		}
	}

	/* Give up control of the I2C bus */
	xSemaphoreGive(Semaphore_I2C);

	return read_buffer[0];
}

/**
 * @brief
 * This function parses the list of parameters to determine the type of
 * operation that is requested. The 'operation' field data structure will
 * be updated.
 *
 * The value will only be used on a write operation.
 *
 * @param pcWriteBuffer
 * Array used to return a string to the CLI parser
 * @param xWriteBufferLen
 * The length of the write buffer
 * @param pcCommandString
 * The list of parameters entered by the user
 * @param packet
 * A pointer to the data structure that will be sent to the io expander task.
 * @return BaseType_t
 * pdPASS if a valid operation is found.
 * pdFAIL if an invalid operation is found.
 */
static BaseType_t cli_handler_ioxp_get_operation(
	char *pcWriteBuffer,
	size_t xWriteBufferLen,
	const char *pcCommandString,
	io_expander_packet_t *packet)
{
	const char *pcParameter;
	BaseType_t lParameterStringLength;
	BaseType_t xReturn;

	/* Obtain the parameter string. */
	pcParameter = FreeRTOS_CLIGetParameter(
		pcCommandString,		/* The command string itself. */
		1,						/* Return the 1st parameter. */
		&lParameterStringLength /* Store the parameter string length. */
	);
	/* Sanity check something was returned. */
	configASSERT(pcParameter);

	memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	strncat(pcWriteBuffer, pcParameter, lParameterStringLength);

	/* Verify that the first parameter is either 'w' or 'r' */
	if ((strcmp(pcWriteBuffer, "r")) == 0)
	{
		packet->operation = IOXP_OP_READ;
		xReturn = pdTRUE;
	}
	else if ((strcmp(pcWriteBuffer, "w")) == 0)
	{
		packet->operation = IOXP_OP_WRITE;
		xReturn = pdTRUE;
	}
	else
	{
		/* The first parameter is invalid*/
		packet->operation = IOXP_OP_INVALID;
		xReturn = pdFAIL;
	}

	return xReturn;
}

/**
 * @brief
 * This function parses the list of parameters for the address of the
 * register that will be written. The 'address' field data structure will
 * be updated.
 *
 * The value will only be used on a write operation.
 *
 * @param pcWriteBuffer
 * Array used to return a string to the CLI parser
 * @param xWriteBufferLen
 * The length of the write buffer
 * @param pcCommandString
 * The list of parameters entered by the user
 * @param packet
 * A pointer to the data structure that will be sent to the io expander task.
 * @return BaseType_t
 * pdPASS if a valid register address is found.
 * pdFAIL if an invalid register address is found.
 */
static BaseType_t cli_handler_ioxp_get_register(
	char *pcWriteBuffer,
	size_t xWriteBufferLen,
	const char *pcCommandString,
	io_expander_packet_t *packet)
{
	const char *pcParameter;
	BaseType_t lParameterStringLength;
	BaseType_t xReturn;
	char *end_ptr;
	uint32_t result;

	/* Obtain the register address string. */
	pcParameter = FreeRTOS_CLIGetParameter(
		pcCommandString,		/* The command string itself. */
		2,						/* Return the 2nd parameter. */
		&lParameterStringLength /* Store the parameter string length. */
	);
	/* Sanity check something was returned. */
	configASSERT(pcParameter);

	memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	strncat(pcWriteBuffer, pcParameter, lParameterStringLength);

	/* Convert the stirng to an address */
	result = strtol(pcWriteBuffer, &end_ptr, 16);
	if (*end_ptr != '\0')
	{
		packet->address = IOXP_ADDR_INVALID;
		xReturn = pdFAIL;
	}
	else
	{
		switch ((io_expander_reg_addr_t)result)
		{
		case IOXP_ADDR_INPUT_PORT:
		case IOXP_ADDR_OUTPUT_PORT:
		case IOXP_ADDR_POLARITY:
		case IOXP_ADDR_CONFIG:
		{
			packet->address = (io_expander_reg_addr_t)result;
			xReturn = pdPASS;
			break;
		}
		default:
		{
			packet->address = IOXP_ADDR_INVALID;
			xReturn = pdFAIL;
			break;
		}
		}
	}

	return xReturn;
}

/**
 * @brief
 * This function parses the list of parameters for the 3rd parameter
 * and sets the 'value' field data structure
 *
 * The value will only be used on a write operation.
 *
 * @param pcWriteBuffer
 * Array used to return a string to the CLI parser
 * @param xWriteBufferLen
 * The length of the write buffer
 * @param pcCommandString
 * The list of parameters entered by the user
 * @param packet
 * A pointer to the data structure that will be sent to the io expander task.
 * @return BaseType_t
 * pdPASS if a valid value was found.
 * pdFAIL if the value is invalid
 */
static BaseType_t cli_handler_ioxp_get_value(
	char *pcWriteBuffer,
	size_t xWriteBufferLen,
	const char *pcCommandString,
	io_expander_packet_t *packet)
{
	const char *pcParameter;
	BaseType_t lParameterStringLength;
	BaseType_t xReturn;
	char *end_ptr;
	uint32_t result;

	/* Obtain the register address string. */
	pcParameter = FreeRTOS_CLIGetParameter(
		pcCommandString,		/* The command string itself. */
		3,						/* Return the 3rd parameter. */
		&lParameterStringLength /* Store the parameter string length. */
	);
	/* Sanity check something was returned. */
	configASSERT(pcParameter);

	memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	strncat(pcWriteBuffer, pcParameter, lParameterStringLength);

	/* Convert the stirng to an address */
	result = strtol(pcWriteBuffer, &end_ptr, 16);
	if (*end_ptr != '\0')
	{
		packet->value = 0xFF;
		xReturn = pdFAIL;
	}
	else
	{
		packet->value = (uint8_t)result;
		xReturn = pdPASS;
	}

	return xReturn;
}

/**
 * @brief
 * FreeRTOS CLI Handler for the 'ioxp' command.
 *
 * This function will be executed from task_console_rx() when
 * FreeRTOS_CLIProcessCommand() is called.
 *
 * @param pcWriteBuffer
 * Array used to return a string to the CLI parser
 * @param xWriteBufferLen
 * The length of the write buffer
 * @param pcCommandString
 * The list of parameters entered by the user
 * @return BaseType_t
 * pdPASS if a valid value was found.
 * pdFAIL if the value is invalid
 */
static BaseType_t cli_handler_ioxp(
	char *pcWriteBuffer,
	size_t xWriteBufferLen,
	const char *pcCommandString)
{
	BaseType_t xReturn;
	io_expander_packet_t ioxp_packet;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	(void)pcCommandString;
	(void)xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	/* Get the ioxp operation type */
	xReturn = cli_handler_ioxp_get_operation(
		pcWriteBuffer,
		xWriteBufferLen,
		pcCommandString,
		&ioxp_packet);

	/* Return if the user entered an invalid operation*/
	if (xReturn == pdFAIL)
	{
		/* Clear the return string */
		memset(pcWriteBuffer, 0x00, xWriteBufferLen);
		sprintf(pcWriteBuffer, "\n\r\tInvalid IOXP Operation");
		return xReturn;
	}

	/* Get the ioxp register address */
	xReturn = cli_handler_ioxp_get_register(
		pcWriteBuffer,
		xWriteBufferLen,
		pcCommandString,
		&ioxp_packet);

	/* Return if the user entered an invalid operation*/
	if (xReturn == pdFAIL)
	{
		/* Clear the return string */
		memset(pcWriteBuffer, 0x00, xWriteBufferLen);
		sprintf(pcWriteBuffer, "\n\r\tInvalid IOXP Register Addr");
		return xReturn;
	}

	/* Check to see if this is a read operation */
	if (ioxp_packet.operation == IOXP_OP_READ)
	{
		/* Indicate which queue to return the data to */
		ioxp_packet.return_queue = q_io_expander_rsp;

		/* Send the packet to the io_expander request queue  */
		xQueueSend(q_io_expander_req, &ioxp_packet, portMAX_DELAY);

		/* Wait for the data to be returned */
		xQueueReceive(q_io_expander_rsp, &ioxp_packet, portMAX_DELAY);

		/* Return the value that was read as a string. */
		memset(pcWriteBuffer, 0x00, xWriteBufferLen);
		sprintf(pcWriteBuffer, "\n\r\t0x%08lx", ioxp_packet.value);

		/* Indicate to the main parser that the command was completed */
		xReturn = pdFAIL;
		return xReturn;
	}
	else
	{
		/* Get the ioxp write value */
		xReturn = cli_handler_ioxp_get_value(
			pcWriteBuffer,
			xWriteBufferLen,
			pcCommandString,
			&ioxp_packet);

		/* Return if the user entered an invalid operation*/
		if (xReturn == pdFAIL)
		{
			memset(pcWriteBuffer, 0x00, xWriteBufferLen);
			sprintf(pcWriteBuffer, "\n\r\tInvalid IOXP write value");
			return xReturn;
		}

		/* Send the packet to the io_expander queue  */
		xQueueSend(q_io_expander_req, &ioxp_packet, portMAX_DELAY);

		/* Return an empty string since there is nothing to return on a write */
		memset(pcWriteBuffer, 0x00, xWriteBufferLen);

		xReturn = pdFAIL;
		return xReturn;
	}
}

/******************************************************************************/
/* Public Function Definitions                                                */
/******************************************************************************/

/**
 * @brief
 * Task used to monitor the reception of command packets sent the io expander
 * @param param
 * Unused
 */
void task_io_expander(void *param)
{
	io_expander_packet_t packet;
	uint32_t read_value = 0;

	while (1)
	{
		/* Wait for a message */
		xQueueReceive(q_io_expander_req, &packet, portMAX_DELAY);

		if (packet.operation == IOXP_OP_READ)
		{
			/* Read the register value requested */
			/* ADD CODE */

			/* Update the value being returned */
			packet.value = read_value;

			/*
			 * Send the data back to the task that made requested to read
			 * the data.
			 *
			 * The task that requests the data MUST be sure
			 * to set the return_queue field of the struct to an initialized
			 * queue.  See cli_handler_ioxp() above for an example of how to send a
			 * read request and then wait for the response.
			 */
			xQueueSend(packet.return_queue, &packet, portMAX_DELAY);

			/* ADD CODE */
			/* Delete once the driver has been completed */
			read_value++;
		}
		else if (packet.operation == IOXP_OP_WRITE)
		{
			/* Write to the IO Expander register */
			/* ADD CODE */
		}
	}
}

/**
 * @brief
 * Initializes software resources related to the operation of
 * the IO Expander.  This function expects that the I2C bus had already
 * been initialized prior to the start of FreeRTOS.
 */
void task_io_exp_init(void)
{
	/* Create the Queue used to control blinking of the status LED*/
	q_io_expander_req = xQueueCreate(1, sizeof(io_expander_packet_t));
	
	q_io_expander_rsp = xQueueCreate(1, sizeof(io_expander_packet_t));

	/* Register the CLI command */
	FreeRTOS_CLIRegisterCommand(&cmd_ioxp);

	/* Create the task that will control the status LED */
	xTaskCreate(
		task_io_expander,
		"Task IO Exp",
		configMINIMAL_STACK_SIZE,
		NULL,
		configMAX_PRIORITIES - 6,
		NULL);
}
