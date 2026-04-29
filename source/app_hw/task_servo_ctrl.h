#ifndef __TASK_SERVO_CTRL_H__
#define __TASK_SERVO_CTRL_H__

#include "main.h"
#include "cyhal_pwm.h"
#include "ece453_pins.h"
#include "task_console.h"

/*
 * Per board pinout: DUAL_SERVOS → P9.3 SERVO_HZNTL (pan), P9.2 SERVO_VERT (tilt).
 * Override these in a project header or build defines if wiring changes.
 */
/* Physical wiring swaps the silkscreen mapping: pan servo is on P9.2, tilt on P9.3. */
#ifndef SERVO_PAN_PWM_PIN
#define SERVO_PAN_PWM_PIN          P9_2
#endif

#ifndef SERVO_TILT_PWM_PIN
#define SERVO_TILT_PWM_PIN         P9_3
#endif

#define SERVO_PWM_FREQUENCY_HZ     (50u)
#define SERVO_MIN_PULSE_US         (1000u)
#define SERVO_MAX_PULSE_US         (2000u)
#define SERVO_MIN_DEG              (0u)
#define SERVO_MAX_DEG              (180u)
#define SERVO_PAN_MIN_PULSE_US     (800u)
#define SERVO_PAN_MAX_PULSE_US     (1600u)
#define SERVO_TILT_MIN_PULSE_US    (500u)
#define SERVO_TILT_MAX_PULSE_US    (1000u)
#define SERVO_PAN_INVERT           (false)
#define SERVO_TILT_INVERT          (true)
#define SERVO_DEFAULT_PAN_DEG      (90u)
#define SERVO_DEFAULT_TILT_DEG     (90u)
#define SERVO_CTRL_QUEUE_LEN       (8u)
#define SERVO_TRACK_IR_ENABLE      (true)
/* Flip these if the tracker runs away from the hot spot instead of toward it. */
#define SERVO_TRACK_INVERT_COL     (true)
#define SERVO_TRACK_INVERT_ROW     (true)

/* Gate: only track when hotspot - frame_mean >= this. Rejects sensor noise
   while letting any real target through (iron saturates ~80 degC vs 22 ambient,
   hand surface ~32 degC vs 22 ambient — both clear a 5 degC gate). An iron
   in frame always outranks a human torso in the "hottest pixel" pick, so no
   absolute threshold is needed to keep the tracker locked on the iron. */
#define SERVO_TRACK_MIN_CONTRAST_C (5.0f)
/* Absolute floor on hot-pixel temp. Skin peaks ~34 degC, so 45 degC
   cleanly rejects body heat while staying well below iron temps (80+ degC). */
#define SERVO_TRACK_MIN_ABS_TEMP_C (40.0f)
/* Tracker now fires once per NEW AMG frame via a timestamp gate, so this
   fixed period is legacy. Kept because unused-code warnings may refer to it. */
#define SERVO_TRACK_PERIOD_MS      (110u)
/* Servo-deg commanded per pixel of error. Below sensor's ~7.5 deg/pixel FOV
   so loop gain < 1. 6 gives gain 0.8 — leaves headroom for physical servo
   lag (PWM command runs ahead of actual rotor position) that the ramp-based
   prediction can't see. Going higher lets users "fake out" the tracker by
   moving the target faster than the servo can physically follow. */
#define SERVO_TRACK_STEP_DEG       (6u)
#define SERVO_TRACK_DEADBAND_PIX   (1u)
/* Parallax: sensor is ~40 mm off fan pivot in pan axis. At ~30 cm working
   distance that's atan(40/300) ≈ 7.6° ≈ 1 pixel of column offset. Flip
   the sign (-1 / +1) depending on which side of the fan the sensor sits. */
#define SERVO_TRACK_PAN_PIX_BIAS   (1)

/* Ramp commands at ~500 °/s — past even high-torque hobby servos so
   motion speed is capped by hardware, not software. */
#define SERVO_RAMP_PERIOD_MS       (20u)
#define SERVO_RAMP_STEP_DEG        (10u)
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
bool task_servo_ctrl_get_tracking(void);
void task_servo_ctrl_set_tracking(bool enable);

/* Temporarily freeze the tracker without changing the user-facing
 * tracking-enabled state. Used by task_tof to stop the tracker chasing
 * a too-close object while a TOF safety trip is active. The GUI dot
 * continues to show Auto-track (or PAUSED while paused:1 is on); the
 * servos just don't move until suppress(false) is called. */
void task_servo_ctrl_suppress_tracking(bool suppress);

/* Enable / disable IR-hottest-point auto-tracking at runtime. Used by the
 * TOF safety logic (pause when something blocks the line of sight) and by
 * the BLE 'track:0|1' command. */
void task_servo_ctrl_set_tracking(bool enabled);

/* Set absolute pan / tilt angles in degrees (0..180). Clamped internally.
 * Used by the BLE 'servo:p,t' joystick command. */
void task_servo_ctrl_set_angles(uint16_t pan_deg, uint16_t tilt_deg);

#endif
