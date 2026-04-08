#ifndef __TASK_SERVO_CTRL_H__
#define __TASK_SERVO_CTRL_H__

#include "main.h"
#include "cyhal_pwm.h"
#include "ece453_pins.h"
#include "task_console.h"

/*
 * Override these in a project header or build defines if wiring changes.
 */
#ifndef SERVO_PAN_PWM_PIN
#define SERVO_PAN_PWM_PIN          MOD_2_PIN_IO_0
#endif

#ifndef SERVO_TILT_PWM_PIN
#define SERVO_TILT_PWM_PIN         MOD_2_PIN_IO_1
#endif

#define SERVO_PWM_FREQUENCY_HZ     (50u)
#define SERVO_MIN_PULSE_US         (1000u)
#define SERVO_MAX_PULSE_US         (2000u)
#define SERVO_MIN_DEG              (0u)
#define SERVO_MAX_DEG              (180u)
#define SERVO_PAN_MIN_PULSE_US     (700u)
#define SERVO_PAN_MAX_PULSE_US     (1800u)
#define SERVO_TILT_MIN_PULSE_US    (500u)
#define SERVO_TILT_MAX_PULSE_US    (1000u)
#define SERVO_PAN_INVERT           (false)
#define SERVO_TILT_INVERT          (true)
#define SERVO_DEFAULT_PAN_DEG      (90u)
#define SERVO_DEFAULT_TILT_DEG     (90u)
#define SERVO_CTRL_QUEUE_LEN       (8u)
#define SERVO_TRACK_IR_ENABLE      (true)
#define SERVO_TRACK_PERIOD_MS      (30u)
#define SERVO_TRACK_STEP_DEG       (4u)
#define SERVO_TRACK_DEADBAND_PIX   (1u)
#define SERVO_CAL_MIN_PULSE_US     (500u)
#define SERVO_CAL_MAX_PULSE_US     (2600u)
#define SERVO_CAL_DEFAULT_STEP_US  (25u)
#define SERVO_CAL_DEFAULT_DWELL_MS (250u)
#define SERVO_CAL_MAX_STEPS        (300u)

typedef enum
{
    SERVO_AXIS_PAN = 0,
    SERVO_AXIS_TILT = 1
} servo_axis_t;

typedef enum
{
    SERVO_CTRL_CMD_SET_ANGLE = 0,
    SERVO_CTRL_CMD_SWEEP = 1
} servo_ctrl_cmd_t;

typedef struct
{
    servo_ctrl_cmd_t cmd;
    bool set_pan;
    bool set_tilt;
    uint16_t pan_deg;
    uint16_t tilt_deg;
    servo_axis_t sweep_axis;
    uint32_t sweep_start_us;
    uint32_t sweep_end_us;
    uint32_t sweep_step_us;
    uint32_t sweep_dwell_ms;
} servo_ctrl_message_t;

extern QueueHandle_t q_servo_ctrl;

void task_servo_ctrl_init(void);

#endif
