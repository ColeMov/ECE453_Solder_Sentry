#ifndef __TASK_IR_SENSOR_H__
#define __TASK_IR_SENSOR_H__

#include "main.h"
#include "cyhal.h"
#include "i2c.h"
#include "task_console.h"

/**
 * Grid-EYE IR Sensor (AMG8834) control task.
 *
 * The task samples the AMG8834 at a fixed rate (10 Hz by default), converts raw
 * thermopile values to temperature (degC), and publishes each 8x8 frame to a
 * FreeRTOS queue for use by other tasks and CLI diagnostics.
 *
 * Datasheet notes used here (AMG88xx family: AMG8833/AMG8834):
 * - Pixel data starts at register 0x80, 64 pixels, 2 bytes per pixel.
 * - Pixel resolution is 0.25 degC/LSB.
 * - Thermistor register is 0x0E/0x0F, resolution is 0.0625 degC/LSB.
 * - Typical frame rate options are 10 FPS or 1 FPS.
 */

/* AMG8834 I2C addresses (selected by address pin state) */
#define AMG8834_I2C_ADDR_LOW             (0x68u)
#define AMG8834_I2C_ADDR_HIGH            (0x69u)
#define AMG8834_I2C_ADDR_DEFAULT         AMG8834_I2C_ADDR_HIGH

/* Grid-EYE frame geometry */
#define AMG8834_ROWS                     (8u)
#define AMG8834_COLS                     (8u)
#define AMG8834_PIXEL_COUNT              (AMG8834_ROWS * AMG8834_COLS)
#define AMG8834_PIXEL_DATA_BYTES         (AMG8834_PIXEL_COUNT * 2u)

/* Sensor conversion constants */
#define AMG8834_PIXEL_LSB_DEGC           (0.25f)
#define AMG8834_THERM_LSB_DEGC           (0.0625f)

/* Task configuration */
#define AMG8834_POLL_HZ                  (10u)
#define AMG8834_POLL_PERIOD_MS           (1000u / AMG8834_POLL_HZ)
#define AMG8834_QUEUE_LEN                (4u)

/* AMG8834 register map */
typedef enum
{
	AMG8834_REG_PCTL         = 0x00,
	AMG8834_REG_RST          = 0x01,
	AMG8834_REG_FPSC         = 0x02,
	AMG8834_REG_INTC         = 0x03,
	AMG8834_REG_STAT         = 0x04,
	AMG8834_REG_SCLR         = 0x05,
	AMG8834_REG_AVE          = 0x07,
	AMG8834_REG_INTHL        = 0x08,
	AMG8834_REG_INTHH        = 0x09,
	AMG8834_REG_INTLL        = 0x0A,
	AMG8834_REG_INTLH        = 0x0B,
	AMG8834_REG_IHYSL        = 0x0C,
	AMG8834_REG_IHYSH        = 0x0D,
	AMG8834_REG_TTHL         = 0x0E,
	AMG8834_REG_TTHH         = 0x0F,
	AMG8834_REG_INT_TABLE    = 0x10,
	AMG8834_REG_PIXEL_OFFSET = 0x80
} amg8834_reg_t;

typedef enum
{
	AMG8834_POWER_NORMAL = 0x00,
	AMG8834_POWER_SLEEP  = 0x01,
	AMG8834_POWER_STBY60 = 0x20,
	AMG8834_POWER_STBY10 = 0x21
} amg8834_power_mode_t;

typedef enum
{
	AMG8834_RESET_FLAG    = 0x30,
	AMG8834_RESET_INITIAL = 0x3F
} amg8834_reset_t;

typedef enum
{
	AMG8834_FPS_10 = 0x00,
	AMG8834_FPS_1  = 0x01
} amg8834_fps_t;

typedef struct
{
	TickType_t timestamp_ticks;
	float thermistor_c;
	float pixels_c[AMG8834_PIXEL_COUNT];
} amg8834_frame_t;

typedef struct
{
	TickType_t timestamp_ticks;
	float thermistor_c;
	float pixels_c[AMG8834_ROWS][AMG8834_COLS];
} amg8834_grid_t;

/* Queue of amg8834_frame_t produced by task_ir_sensor. */
extern QueueHandle_t q_ir_sensor_frame;

/* Initializes the AMG8834 task, queue, and CLI command(s). */
void task_ir_sensor_init(void);

/* Reads one full 8x8 frame and thermistor value from the sensor. */
cy_rslt_t amg8834_read_frame(amg8834_frame_t *frame);

/* Copies the latest 8x8 grid sample for consumers such as stepper control tasks. */
bool amg8834_get_latest_grid(amg8834_grid_t *grid_out);

/* Returns the hottest pixel location from the latest frame. */
bool amg8834_get_hottest_pixel(uint32_t *row_out, uint32_t *col_out, float *temp_c_out);

/* Converts a 12-bit signed-magnitude sample to degC using the provided LSB. */
float amg8834_convert_signed_mag12_to_celsius(uint16_t raw, float lsb_deg_c);



#endif