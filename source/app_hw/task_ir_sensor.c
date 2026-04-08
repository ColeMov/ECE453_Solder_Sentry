#include "task_ir_sensor.h"

/******************************************************************************/
/* Function Declarations                                                      */
/******************************************************************************/
static void task_ir_sensor(void *param);

static BaseType_t cli_handler_ir(
	char *pcWriteBuffer,
	size_t xWriteBufferLen,
	const char *pcCommandString);

static cy_rslt_t amg8834_write_reg(uint8_t reg, uint8_t value);
static cy_rslt_t amg8834_read_reg(uint8_t reg, uint8_t *value);
static cy_rslt_t amg8834_read_block(uint8_t start_reg, uint8_t *buffer, uint16_t len);
static cy_rslt_t amg8834_init_device(amg8834_fps_t fps);
static void amg8834_frame_to_grid(const amg8834_frame_t *frame, amg8834_grid_t *grid);
static bool cli_appendf(char *buffer, size_t buffer_len, size_t *used, const char *fmt, ...);
static int32_t celsius_to_centi(float celsius);
static bool cli_append_temp(char *buffer, size_t buffer_len, size_t *used, float celsius);

/******************************************************************************/
/* Global Variables                                                           */
/******************************************************************************/
QueueHandle_t q_ir_sensor_frame;

static SemaphoreHandle_t g_ir_frame_mutex;
static amg8834_frame_t g_ir_latest_frame;
static amg8834_grid_t g_ir_latest_grid;
static bool g_ir_frame_valid = false;

static const CLI_Command_Definition_t xIRSensor =
	{
		"ir",                           /* command text */
		"\r\nir\r\n",                /* command help text */
		cli_handler_ir,                  /* The function to run. */
		0                                /* No parameters */
};

/******************************************************************************/
/* Static Function Definitions                                                */
/******************************************************************************/

static bool cli_appendf(char *buffer, size_t buffer_len, size_t *used, const char *fmt, ...)
{
	va_list args;
	int n;
	size_t remaining;

	if ((buffer == NULL) || (used == NULL) || (*used >= buffer_len))
	{
		return false;
	}

	remaining = buffer_len - *used;

	va_start(args, fmt);
	n = vsnprintf(&buffer[*used], remaining, fmt, args);
	va_end(args);

	if (n < 0)
	{
		return false;
	}

	if ((size_t)n >= remaining)
	{
		*used = buffer_len - 1u;
		return false;
	}

	*used += (size_t)n;
	return true;
}

static int32_t celsius_to_centi(float celsius)
{
	float scaled;

	scaled = celsius * 100.0f;
	if (scaled >= 0.0f)
	{
		scaled += 0.5f;
	}
	else
	{
		scaled -= 0.5f;
	}

	return (int32_t)scaled;
}

static bool cli_append_temp(char *buffer, size_t buffer_len, size_t *used, float celsius)
{
	int32_t centi;
	int32_t abs_centi;

	centi = celsius_to_centi(celsius);
	abs_centi = (centi < 0) ? (-centi) : centi;

	return cli_appendf(
		buffer,
		buffer_len,
		used,
		"%c%ld.%02ld",
		(centi < 0) ? '-' : '+',
		(long)(abs_centi / 100),
		(long)(abs_centi % 100));
}

static cy_rslt_t amg8834_write_reg(uint8_t reg, uint8_t value)
{
	cy_rslt_t rslt;
	uint8_t write_buffer[2] = {reg, value};

	xSemaphoreTake(Semaphore_I2C, portMAX_DELAY);

	rslt = cyhal_i2c_master_write(
		&i2c_master_obj,
		AMG8834_I2C_ADDR_DEFAULT,
		write_buffer,
		sizeof(write_buffer),
		0,
		true);

	xSemaphoreGive(Semaphore_I2C);
	return rslt;
}

static cy_rslt_t amg8834_read_reg(uint8_t reg, uint8_t *value)
{
	cy_rslt_t rslt;

	xSemaphoreTake(Semaphore_I2C, portMAX_DELAY);

	rslt = cyhal_i2c_master_write(
		&i2c_master_obj,
		AMG8834_I2C_ADDR_DEFAULT,
		&reg,
		1,
		0,
		false);

	if (rslt == CY_RSLT_SUCCESS)
	{
		rslt = cyhal_i2c_master_read(
			&i2c_master_obj,
			AMG8834_I2C_ADDR_DEFAULT,
			value,
			1,
			0,
			true);
	}

	xSemaphoreGive(Semaphore_I2C);
	return rslt;
}

static cy_rslt_t amg8834_read_block(uint8_t start_reg, uint8_t *buffer, uint16_t len)
{
	cy_rslt_t rslt;

	xSemaphoreTake(Semaphore_I2C, portMAX_DELAY);

	rslt = cyhal_i2c_master_write(
		&i2c_master_obj,
		AMG8834_I2C_ADDR_DEFAULT,
		&start_reg,
		1,
		0,
		false);

	if (rslt == CY_RSLT_SUCCESS)
	{
		rslt = cyhal_i2c_master_read(
			&i2c_master_obj,
			AMG8834_I2C_ADDR_DEFAULT,
			buffer,
			len,
			0,
			true);
	}

	xSemaphoreGive(Semaphore_I2C);
	return rslt;
}

static cy_rslt_t amg8834_init_device(amg8834_fps_t fps)
{
	cy_rslt_t rslt;
	uint8_t stat;

	rslt = amg8834_write_reg((uint8_t)AMG8834_REG_PCTL, (uint8_t)AMG8834_POWER_NORMAL);
	if (rslt != CY_RSLT_SUCCESS)
	{
		return rslt;
	}

	rslt = amg8834_write_reg((uint8_t)AMG8834_REG_RST, (uint8_t)AMG8834_RESET_INITIAL);
	if (rslt != CY_RSLT_SUCCESS)
	{
		return rslt;
	}

	vTaskDelay(pdMS_TO_TICKS(5));

	rslt = amg8834_write_reg((uint8_t)AMG8834_REG_INTC, 0x00);
	if (rslt != CY_RSLT_SUCCESS)
	{
		return rslt;
	}

	rslt = amg8834_write_reg((uint8_t)AMG8834_REG_FPSC, (uint8_t)fps);
	if (rslt != CY_RSLT_SUCCESS)
	{
		return rslt;
	}

	rslt = amg8834_read_reg((uint8_t)AMG8834_REG_STAT, &stat);
	if (rslt == CY_RSLT_SUCCESS)
	{
		task_print_info("AMG8834 init complete (STAT=0x%02X, FPS=%u)", stat, (unsigned int)fps);
	}

	return rslt;
}

static void amg8834_frame_to_grid(const amg8834_frame_t *frame, amg8834_grid_t *grid)
{
	uint32_t row;
	uint32_t col;

	configASSERT(frame != NULL);
	configASSERT(grid != NULL);

	grid->timestamp_ticks = frame->timestamp_ticks;
	grid->thermistor_c = frame->thermistor_c;

	for (row = 0; row < AMG8834_ROWS; row++)
	{
		for (col = 0; col < AMG8834_COLS; col++)
		{
			uint32_t idx = (row * AMG8834_COLS) + col;
			grid->pixels_c[row][col] = frame->pixels_c[idx];
		}
	}
}

static void task_ir_sensor(void *param)
{
	cy_rslt_t rslt;
	TickType_t xLastWakeTime;
	amg8834_frame_t frame;

	(void)param;

	for (;;)
	{
		rslt = amg8834_init_device(AMG8834_FPS_10);
		if (rslt == CY_RSLT_SUCCESS)
		{
			break;
		}

		task_print_error("AMG8834 init failed (0x%08lX), retrying", (unsigned long)rslt);
		vTaskDelay(pdMS_TO_TICKS(1000));
	}

	xLastWakeTime = xTaskGetTickCount();

	for (;;)
	{
		rslt = amg8834_read_frame(&frame);
		if (rslt == CY_RSLT_SUCCESS)
		{
			xSemaphoreTake(g_ir_frame_mutex, portMAX_DELAY);
			g_ir_latest_frame = frame;
			amg8834_frame_to_grid(&frame, &g_ir_latest_grid);
			g_ir_frame_valid = true;
			xSemaphoreGive(g_ir_frame_mutex);

			if (xQueueSendToBack(q_ir_sensor_frame, &frame, 0) != pdPASS)
			{
				amg8834_frame_t stale;
				(void)xQueueReceive(q_ir_sensor_frame, &stale, 0);
				(void)xQueueSendToBack(q_ir_sensor_frame, &frame, 0);
			}
		}
		else
		{
			task_print_warning("AMG8834 frame read failed (0x%08lX)", (unsigned long)rslt);
		}

		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(AMG8834_POLL_PERIOD_MS));
	}
}

static BaseType_t cli_handler_ir(
	char *pcWriteBuffer,
	size_t xWriteBufferLen,
	const char *pcCommandString)
{
	BaseType_t xReturn;
	static bool s_stream_active = false;
	static bool s_header_pending = false;
	static uint32_t s_next_row = 0;
	static amg8834_frame_t s_frame;
	bool have_frame = false;
	size_t used = 0;
	uint32_t row;
	uint32_t col;

	(void)pcCommandString;
	configASSERT(pcWriteBuffer);
	xReturn = pdFALSE;

	memset(pcWriteBuffer, 0, xWriteBufferLen);

	if (!s_stream_active)
	{
		xSemaphoreTake(g_ir_frame_mutex, portMAX_DELAY);
		have_frame = g_ir_frame_valid;
		if (have_frame)
		{
			s_frame = g_ir_latest_frame;
		}
		xSemaphoreGive(g_ir_frame_mutex);

		if (!have_frame)
		{
			(void)cli_appendf(pcWriteBuffer, xWriteBufferLen, &used, "AMG8834: no frame available yet\r\n");
			return pdFALSE;
		}

		s_next_row = 0;
		s_stream_active = true;
		s_header_pending = true;
	}

	if (s_header_pending)
	{
		if (!cli_appendf(
				pcWriteBuffer,
				xWriteBufferLen,
				&used,
				"AMG8834 @ tick %lu, thermistor ",
				(unsigned long)s_frame.timestamp_ticks))
		{
			s_stream_active = false;
			s_header_pending = false;
			return pdFALSE;
		}

		if (!cli_append_temp(pcWriteBuffer, xWriteBufferLen, &used, s_frame.thermistor_c))
		{
			s_stream_active = false;
			s_header_pending = false;
			return pdFALSE;
		}

		if (!cli_appendf(pcWriteBuffer, xWriteBufferLen, &used, " C\r\n"))
		{
			s_stream_active = false;
			s_header_pending = false;
			return pdFALSE;
		}

		s_header_pending = false;
		return pdTRUE;
	}

	row = s_next_row;
	if (row < AMG8834_ROWS)
	{
		if (!cli_appendf(pcWriteBuffer, xWriteBufferLen, &used, "[%lu]", (unsigned long)row))
		{
			s_stream_active = false;
			s_header_pending = false;
			return pdFALSE;
		}

		for (col = 0; col < AMG8834_COLS; col++)
		{
			uint32_t idx = row * AMG8834_COLS + col;
			if (!cli_appendf(pcWriteBuffer, xWriteBufferLen, &used, " "))
			{
				s_stream_active = false;
				s_header_pending = false;
				return pdFALSE;
			}
			if (!cli_append_temp(pcWriteBuffer, xWriteBufferLen, &used, s_frame.pixels_c[idx]))
			{
				s_stream_active = false;
				s_header_pending = false;
				return pdFALSE;
			}
		}

		if (!cli_appendf(pcWriteBuffer, xWriteBufferLen, &used, "\r\n"))
		{
			s_stream_active = false;
			s_header_pending = false;
			return pdFALSE;
		}

		row++;
	}

	s_next_row = row;
	if (s_next_row < AMG8834_ROWS)
	{
		xReturn = pdTRUE;
	}
	else
	{
		s_stream_active = false;
		xReturn = pdFALSE;
	}

	return xReturn;
}

/******************************************************************************/
/* Public Function Definitions                                                */
/******************************************************************************/

void task_ir_sensor_init(void)
{
	if (Semaphore_I2C == NULL)
	{
		task_print_error("AMG8834 init requires i2c_init(MODULE_SITE_1) before scheduler start");
		return;
	}

	q_ir_sensor_frame = xQueueCreate(AMG8834_QUEUE_LEN, sizeof(amg8834_frame_t));
	configASSERT(q_ir_sensor_frame != NULL);

	g_ir_frame_mutex = xSemaphoreCreateMutex();
	configASSERT(g_ir_frame_mutex != NULL);

	FreeRTOS_CLIRegisterCommand(&xIRSensor);

	xTaskCreate(
		task_ir_sensor,
		"Task_IR",
		configMINIMAL_STACK_SIZE * 4,
		NULL,
		configMAX_PRIORITIES - 6,
		NULL);
}

bool amg8834_get_latest_grid(amg8834_grid_t *grid_out)
{
	bool have_frame;

	if ((grid_out == NULL) || (g_ir_frame_mutex == NULL))
	{
		return false;
	}

	xSemaphoreTake(g_ir_frame_mutex, portMAX_DELAY);
	have_frame = g_ir_frame_valid;
	if (have_frame)
	{
		*grid_out = g_ir_latest_grid;
	}
	xSemaphoreGive(g_ir_frame_mutex);

	return have_frame;
}

bool amg8834_get_hottest_pixel(uint32_t *row_out, uint32_t *col_out, float *temp_c_out)
{
	bool have_frame;
	uint32_t row;
	uint32_t col;
	uint32_t max_row = 0;
	uint32_t max_col = 0;
	float max_temp = 0.0f;

	if ((row_out == NULL) || (col_out == NULL) || (temp_c_out == NULL) || (g_ir_frame_mutex == NULL))
	{
		return false;
	}

	xSemaphoreTake(g_ir_frame_mutex, portMAX_DELAY);
	have_frame = g_ir_frame_valid;
	if (have_frame)
	{
		max_temp = g_ir_latest_grid.pixels_c[0][0];
		for (row = 0; row < AMG8834_ROWS; row++)
		{
			for (col = 0; col < AMG8834_COLS; col++)
			{
				float temp = g_ir_latest_grid.pixels_c[row][col];
				if (temp > max_temp)
				{
					max_temp = temp;
					max_row = row;
					max_col = col;
				}
			}
		}
	}
	xSemaphoreGive(g_ir_frame_mutex);

	if (!have_frame)
	{
		return false;
	}

	*row_out = max_row;
	*col_out = max_col;
	*temp_c_out = max_temp;

	return true;
}

cy_rslt_t amg8834_read_frame(amg8834_frame_t *frame)
{
	cy_rslt_t rslt;
	uint8_t therm_raw[2];
	uint8_t pixel_raw[AMG8834_PIXEL_DATA_BYTES];
	uint32_t i;

	if (frame == NULL)
	{
		return CY_RSLT_TYPE_ERROR;
	}

	rslt = amg8834_read_block((uint8_t)AMG8834_REG_TTHL, therm_raw, sizeof(therm_raw));
	if (rslt != CY_RSLT_SUCCESS)
	{
		return rslt;
	}

	rslt = amg8834_read_block((uint8_t)AMG8834_REG_PIXEL_OFFSET, pixel_raw, sizeof(pixel_raw));
	if (rslt != CY_RSLT_SUCCESS)
	{
		return rslt;
	}

	frame->timestamp_ticks = xTaskGetTickCount();

	{
		uint16_t raw = ((uint16_t)(therm_raw[1] & 0x0Fu) << 8) | therm_raw[0];
		frame->thermistor_c = amg8834_convert_signed_mag12_to_celsius(raw, AMG8834_THERM_LSB_DEGC);
	}

	for (i = 0; i < AMG8834_PIXEL_COUNT; i++)
	{
		uint16_t raw = ((uint16_t)(pixel_raw[(2u * i) + 1u] & 0x0Fu) << 8) | pixel_raw[2u * i];
		frame->pixels_c[i] = amg8834_convert_signed_mag12_to_celsius(raw, AMG8834_PIXEL_LSB_DEGC);
	}

	return CY_RSLT_SUCCESS;
}

float amg8834_convert_signed_mag12_to_celsius(uint16_t raw, float lsb_deg_c)
{
	uint16_t val;
	float magnitude;

	val = raw & 0x0FFFu;
	magnitude = (float)(val & 0x07FFu) * lsb_deg_c;

	if ((val & 0x0800u) != 0u)
	{
		return -magnitude;
	}

	return magnitude;
}
