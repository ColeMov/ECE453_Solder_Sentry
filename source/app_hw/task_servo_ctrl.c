#include "task_servo_ctrl.h"
#include "task_ir_sensor.h"
#include "task_fan.h"

/* Auto-fan: kill the fan after this long without seeing iron-class
 * heat through the tracker. Iron detected → restore to full duty. */
#define FAN_AUTO_OFF_TIMEOUT_MS    (3000u)
#define FAN_AUTO_ON_DUTY           (100u)

/*******************************************************************************/
/* Function Declarations                                                       */
/*******************************************************************************/
static void task_servo_ctrl(void *param);
static bool servo_set_angle(cyhal_pwm_t *pwm_obj, uint16_t deg, uint32_t min_pulse_us, uint32_t max_pulse_us, bool invert);
static uint16_t servo_clamp_deg(int32_t deg);
static bool parse_deg_param(const char *pcCommandString, UBaseType_t param_index, int32_t *deg_out);

static BaseType_t cli_handler_servo_pan(
    char *pcWriteBuffer,
    size_t xWriteBufferLen,
    const char *pcCommandString);

static BaseType_t cli_handler_servo_tilt(
    char *pcWriteBuffer,
    size_t xWriteBufferLen,
    const char *pcCommandString);

static BaseType_t cli_handler_servo_set(
    char *pcWriteBuffer,
    size_t xWriteBufferLen,
    const char *pcCommandString);

static BaseType_t cli_handler_servo_cal(
    char *pcWriteBuffer,
    size_t xWriteBufferLen,
    const char *pcCommandString);

static BaseType_t cli_handler_servo_track(
    char *pcWriteBuffer,
    size_t xWriteBufferLen,
    const char *pcCommandString);

static bool parse_axis_param(const char *pcCommandString, UBaseType_t param_index, servo_axis_t *axis_out);
static bool servo_set_pulse_us(cyhal_pwm_t *pwm_obj, uint32_t pulse_us);
static bool servo_run_sweep(const servo_ctrl_message_t *msg);
static void servo_track_hottest_ir_point(void);
static void servo_fan_gate_tick(void);

/* Updated whenever the tracker confirms iron-class heat in frame.
 * 0 = never seen yet (don't auto-cut on boot until we've had a
 * chance to look). */
static volatile TickType_t g_last_iron_seen_ts = 0u;

/*******************************************************************************/
/* Global Variables                                                            */
/*******************************************************************************/
QueueHandle_t q_servo_ctrl;

static cyhal_pwm_t g_pwm_pan;
static cyhal_pwm_t g_pwm_tilt;

static uint16_t g_pan_deg = SERVO_DEFAULT_PAN_DEG;
static uint16_t g_tilt_deg = SERVO_DEFAULT_TILT_DEG;
static uint16_t g_pan_target_deg = SERVO_DEFAULT_PAN_DEG;
static uint16_t g_tilt_target_deg = SERVO_DEFAULT_TILT_DEG;
static bool g_servo_track_enabled = SERVO_TRACK_IR_ENABLE;
/* Set by task_tof when a too-close trip happens — freezes the tracker
 * (no servo motion) without changing the user-facing tracking state.
 * Cleared when distance crosses back above the leave threshold. */
static volatile bool g_servo_track_suppressed = false;

static void servo_ramp_tick(void);

static const CLI_Command_Definition_t xServoPan =
{
    "servo_pan",
    "\r\nservo_pan <0-180>\r\n"
    " Set pan servo angle in degrees\r\n",
    cli_handler_servo_pan,
    1
};

static const CLI_Command_Definition_t xServoTilt =
{
    "servo_tilt",
    "\r\nservo_tilt <0-180>\r\n"
    " Set tilt servo angle in degrees\r\n",
    cli_handler_servo_tilt,
    1
};

static const CLI_Command_Definition_t xServoSet =
{
    "servo_set",
    "\r\nservo_set <pan 0-180> <tilt 0-180>\r\n"
    " Set both pan and tilt angles in degrees\r\n",
    cli_handler_servo_set,
    2
};

static const CLI_Command_Definition_t xServoCal =
{
    "servo_cal",
    "\r\nservo_cal <pan|tilt> <start_us> <end_us> [step_us] [dwell_ms]\r\n"
    " Slowly sweeps pulse width and prints each step for limit calibration\r\n",
    cli_handler_servo_cal,
    -1
};

static const CLI_Command_Definition_t xServoTrack =
{
    "servo_track",
    "\r\nservo_track [on|off|status]\r\n"
    " Enable/disable hottest-point auto-tracking\r\n",
    cli_handler_servo_track,
    -1
};

/*******************************************************************************/
/* Static Function Definitions                                                 */
/*******************************************************************************/

static uint16_t servo_clamp_deg(int32_t deg)
{
    if (deg < (int32_t)SERVO_MIN_DEG)
    {
        return SERVO_MIN_DEG;
    }

    if (deg > (int32_t)SERVO_MAX_DEG)
    {
        return SERVO_MAX_DEG;
    }

    return (uint16_t)deg;
}

static bool parse_deg_param(const char *pcCommandString, UBaseType_t param_index, int32_t *deg_out)
{
    const char *pcParameter;
    BaseType_t xParameterStringLength;
    char parameter_buffer[16];
    size_t copy_len;
    char *end_ptr;
    long parsed;

    if (deg_out == NULL)
    {
        return false;
    }

    pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, param_index, &xParameterStringLength);
    if ((pcParameter == NULL) || (xParameterStringLength <= 0))
    {
        return false;
    }

    copy_len = (size_t)xParameterStringLength;
    if (copy_len > (sizeof(parameter_buffer) - 1u))
    {
        copy_len = sizeof(parameter_buffer) - 1u;
    }

    memset(parameter_buffer, 0, sizeof(parameter_buffer));
    strncpy(parameter_buffer, pcParameter, copy_len);
    parameter_buffer[copy_len] = '\0';

    parsed = strtol(parameter_buffer, &end_ptr, 10);
    if ((end_ptr == parameter_buffer) || (*end_ptr != '\0'))
    {
        return false;
    }

    *deg_out = (int32_t)parsed;
    return true;
}

static bool parse_axis_param(const char *pcCommandString, UBaseType_t param_index, servo_axis_t *axis_out)
{
    const char *pcParameter;
    BaseType_t xParameterStringLength;
    char parameter_buffer[16];
    size_t copy_len;

    if (axis_out == NULL)
    {
        return false;
    }

    pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, param_index, &xParameterStringLength);
    if ((pcParameter == NULL) || (xParameterStringLength <= 0))
    {
        return false;
    }

    copy_len = (size_t)xParameterStringLength;
    if (copy_len > (sizeof(parameter_buffer) - 1u))
    {
        copy_len = sizeof(parameter_buffer) - 1u;
    }

    memset(parameter_buffer, 0, sizeof(parameter_buffer));
    strncpy(parameter_buffer, pcParameter, copy_len);
    parameter_buffer[copy_len] = '\0';

    if (strcmp(parameter_buffer, "pan") == 0)
    {
        *axis_out = SERVO_AXIS_PAN;
        return true;
    }

    if (strcmp(parameter_buffer, "tilt") == 0)
    {
        *axis_out = SERVO_AXIS_TILT;
        return true;
    }

    return false;
}

static bool servo_set_angle(cyhal_pwm_t *pwm_obj, uint16_t deg, uint32_t min_pulse_us, uint32_t max_pulse_us, bool invert)
{
    uint32_t pulse_us;
    uint32_t span_us;
    uint32_t period_us;
    float duty_cycle;
    cy_rslt_t rslt;
    uint16_t effective_deg;

    if (max_pulse_us <= min_pulse_us)
    {
        return false;
    }

    effective_deg = invert ? (uint16_t)(SERVO_MAX_DEG - deg) : deg;

    span_us = max_pulse_us - min_pulse_us;
    pulse_us = min_pulse_us + ((span_us * (uint32_t)effective_deg) / SERVO_MAX_DEG);
    period_us = 1000000u / SERVO_PWM_FREQUENCY_HZ;
    duty_cycle = ((float)pulse_us * 100.0f) / (float)period_us;

    rslt = cyhal_pwm_set_duty_cycle(pwm_obj, duty_cycle, SERVO_PWM_FREQUENCY_HZ);
    return (rslt == CY_RSLT_SUCCESS);
}

static bool servo_set_pulse_us(cyhal_pwm_t *pwm_obj, uint32_t pulse_us)
{
    uint32_t period_us;
    float duty_cycle;
    cy_rslt_t rslt;

    period_us = 1000000u / SERVO_PWM_FREQUENCY_HZ;
    duty_cycle = ((float)pulse_us * 100.0f) / (float)period_us;

    rslt = cyhal_pwm_set_duty_cycle(pwm_obj, duty_cycle, SERVO_PWM_FREQUENCY_HZ);
    return (rslt == CY_RSLT_SUCCESS);
}

static bool servo_run_sweep(const servo_ctrl_message_t *msg)
{
    cyhal_pwm_t *target_pwm;
    const char *axis_name;
    int32_t start_us;
    int32_t end_us;
    int32_t step_us;
    int32_t pulse_us;

    if (msg == NULL)
    {
        return false;
    }

    if (msg->sweep_axis == SERVO_AXIS_PAN)
    {
        target_pwm = &g_pwm_pan;
        axis_name = "pan";
    }
    else
    {
        target_pwm = &g_pwm_tilt;
        axis_name = "tilt";
    }

    start_us = (int32_t)msg->sweep_start_us;
    end_us = (int32_t)msg->sweep_end_us;
    step_us = (int32_t)msg->sweep_step_us;

    if (step_us <= 0)
    {
        return false;
    }

    task_print_warning("servo_cal %s sweep start: %ld -> %ld us, step=%ld us, dwell=%lu ms",
                       axis_name,
                       (long)start_us,
                       (long)end_us,
                       (long)step_us,
                       (unsigned long)msg->sweep_dwell_ms);

    if (start_us <= end_us)
    {
        for (pulse_us = start_us; pulse_us <= end_us; pulse_us += step_us)
        {
            if (!servo_set_pulse_us(target_pwm, (uint32_t)pulse_us))
            {
                task_print_error("servo_cal %s failed at %ld us", axis_name, (long)pulse_us);
                return false;
            }

            task_print_info("servo_cal %s pulse=%ld us", axis_name, (long)pulse_us);
            vTaskDelay(pdMS_TO_TICKS(msg->sweep_dwell_ms));
        }
    }
    else
    {
        for (pulse_us = start_us; pulse_us >= end_us; pulse_us -= step_us)
        {
            if (!servo_set_pulse_us(target_pwm, (uint32_t)pulse_us))
            {
                task_print_error("servo_cal %s failed at %ld us", axis_name, (long)pulse_us);
                return false;
            }

            task_print_info("servo_cal %s pulse=%ld us", axis_name, (long)pulse_us);
            vTaskDelay(pdMS_TO_TICKS(msg->sweep_dwell_ms));
        }
    }

    task_print_warning("servo_cal %s sweep complete", axis_name);
    return true;
}

static void servo_track_hottest_ir_point(void)
{
    static TickType_t s_last_frame_ts = 0;
    TickType_t frame_ts;
    uint32_t hot_row;
    uint32_t hot_col;
    float hot_temp_c;
    int32_t row_error;
    int32_t col_error;
    int32_t next_pan;
    int32_t next_tilt;
    bool changed = false;

    if (!g_servo_track_enabled || g_servo_track_suppressed)
    {
        return;
    }

    /* Fresh-frame gate: only act once per new IR frame (10 fps = 100 ms).
       This synchronizes tracker decisions with sensor data so we never
       waste a cycle on a stale frame and never skip a fresh one. */
    frame_ts = amg8834_get_latest_timestamp();
    if (frame_ts == 0u || frame_ts == s_last_frame_ts)
    {
        return;
    }
    s_last_frame_ts = frame_ts;

    float mean_temp_c;
    if (!amg8834_get_hottest_and_mean(&hot_row, &hot_col, &hot_temp_c, &mean_temp_c))
    {
        return;
    }

    /* Absolute gate rejects body heat (skin peaks ~34 degC) so the tracker
       only locks onto iron-class heat sources. Contrast gate still required
       to reject near-uniform scenes where even the "hottest" pixel is noise. */
    if (hot_temp_c < SERVO_TRACK_MIN_ABS_TEMP_C)
    {
        return;
    }

    /* Iron-presence stamp uses ONLY the absolute-temp gate. When the
     * iron sits close to the sensor it warms every pixel — hot - mean
     * collapses below SERVO_TRACK_MIN_CONTRAST_C, the tracker bails
     * before stamping, and the fan-gate 3s-times out exactly when
     * the user is actively soldering. Steering still demands contrast
     * (next gate); fan presence does not. */
    g_last_iron_seen_ts = xTaskGetTickCount();

    if ((hot_temp_c - mean_temp_c) < SERVO_TRACK_MIN_CONTRAST_C)
    {
        return;
    }

    /* 8x8 Grid-EYE center is between pixels 3 and 4; signed error from raw
       hot-pixel location (no EMA — temporal smoothing lags fast targets into
       phantom middle positions that fall in deadband and freeze the tracker).
       PAN_PIX_BIAS shifts aim column to compensate for sensor/fan parallax. */
    row_error = (int32_t)hot_row - (int32_t)(AMG8834_ROWS / 2u);
    col_error = (int32_t)hot_col - (int32_t)(AMG8834_COLS / 2u)
                - (int32_t)SERVO_TRACK_PAN_PIX_BIAS;

    if (SERVO_TRACK_INVERT_COL)
    {
        col_error = -col_error;
    }
    if (SERVO_TRACK_INVERT_ROW)
    {
        row_error = -row_error;
    }

    (void)hot_temp_c;

    /* Predict where the error will land once already-commanded motion finishes.
       g_pan_target_deg - g_pan_deg is in-flight servo motion (ramp hasn't
       caught up yet). After inversion, commanded pan increase reduces col_error
       at ~1 pixel per FOV_DEG_PER_PIXEL servo degrees. Subtracting in-flight
       motion prevents stacking new commands on top of unfinished ones — this
       is what causes overshoot at high gain even when gain itself is stable. */
    const float kFovDegPerPixel = 7.5f;
    int32_t pending_pan = (int32_t)g_pan_target_deg - (int32_t)g_pan_deg;
    int32_t pending_tilt = (int32_t)g_tilt_target_deg - (int32_t)g_tilt_deg;
    float predicted_col = (float)col_error - (float)pending_pan / kFovDegPerPixel;
    float predicted_row = (float)row_error - (float)pending_tilt / kFovDegPerPixel;

    int32_t col_pred = (int32_t)(predicted_col + ((predicted_col >= 0.0f) ? 0.5f : -0.5f));
    int32_t row_pred = (int32_t)(predicted_row + ((predicted_row >= 0.0f) ? 0.5f : -0.5f));
    int32_t col_abs = (col_pred < 0) ? -col_pred : col_pred;
    int32_t row_abs = (row_pred < 0) ? -row_pred : row_pred;

    if (col_abs > (int32_t)SERVO_TRACK_DEADBAND_PIX)
    {
        int32_t step = (int32_t)SERVO_TRACK_STEP_DEG * col_abs;
        next_pan = (int32_t)g_pan_target_deg + ((col_pred > 0) ? step : -step);
        g_pan_target_deg = servo_clamp_deg(next_pan);
        changed = true;
    }

    /* Positive row is lower in image; tilt down for positive row error. */
    if (row_abs > (int32_t)SERVO_TRACK_DEADBAND_PIX)
    {
        int32_t step = (int32_t)SERVO_TRACK_STEP_DEG * row_abs;
        next_tilt = (int32_t)g_tilt_target_deg + ((row_pred > 0) ? step : -step);
        g_tilt_target_deg = servo_clamp_deg(next_tilt);
        changed = true;
    }

    (void)changed;
}

/* Runs every SERVO_RAMP_PERIOD_MS. Nudges the current angle one small
   step toward the target and pushes the new PWM value, so the servo
   moves continuously rather than in whiplash jumps. */
static void servo_ramp_tick(void)
{
    const int32_t step = (int32_t)SERVO_RAMP_STEP_DEG;
    int32_t diff;
    bool pan_changed = false;
    bool tilt_changed = false;

    diff = (int32_t)g_pan_target_deg - (int32_t)g_pan_deg;
    if (diff > step)
    {
        g_pan_deg = servo_clamp_deg((int32_t)g_pan_deg + step);
        pan_changed = true;
    }
    else if (diff < -step)
    {
        g_pan_deg = servo_clamp_deg((int32_t)g_pan_deg - step);
        pan_changed = true;
    }
    else if (diff != 0)
    {
        g_pan_deg = g_pan_target_deg;
        pan_changed = true;
    }

    diff = (int32_t)g_tilt_target_deg - (int32_t)g_tilt_deg;
    if (diff > step)
    {
        g_tilt_deg = servo_clamp_deg((int32_t)g_tilt_deg + step);
        tilt_changed = true;
    }
    else if (diff < -step)
    {
        g_tilt_deg = servo_clamp_deg((int32_t)g_tilt_deg - step);
        tilt_changed = true;
    }
    else if (diff != 0)
    {
        g_tilt_deg = g_tilt_target_deg;
        tilt_changed = true;
    }

    if (pan_changed)
    {
        (void)servo_set_angle(&g_pwm_pan, g_pan_deg, SERVO_PAN_MIN_PULSE_US, SERVO_PAN_MAX_PULSE_US, SERVO_PAN_INVERT);
    }
    if (tilt_changed)
    {
        (void)servo_set_angle(&g_pwm_tilt, g_tilt_deg, SERVO_TILT_MIN_PULSE_US, SERVO_TILT_MAX_PULSE_US, SERVO_TILT_INVERT);
    }
}

static void task_servo_ctrl(void *param)
{
    servo_ctrl_message_t msg;

    (void)param;

    for (;;)
    {
        if (xQueueReceive(q_servo_ctrl, &msg, pdMS_TO_TICKS(SERVO_RAMP_PERIOD_MS)) == pdPASS)
        {
            if (msg.cmd == SERVO_CTRL_CMD_SWEEP)
            {
                if (!servo_run_sweep(&msg))
                {
                    task_print_error("servo_cal execution failed");
                }
                continue;
            }

            /* Manual set-angle always wins: disable auto-tracking so the
               tracker does not immediately overwrite the commanded angle.
               task_servo_ctrl_set_tracking emits track:0 for the GUI. */
            if ((msg.set_pan || msg.set_tilt) && g_servo_track_enabled)
            {
                task_servo_ctrl_set_tracking(false);
            }

            if (msg.set_pan)
            {
                g_pan_target_deg = servo_clamp_deg((int32_t)msg.pan_deg);
            }

            if (msg.set_tilt)
            {
                g_tilt_target_deg = servo_clamp_deg((int32_t)msg.tilt_deg);
            }
        }

        /* Tracker self-rate-limits internally to SERVO_TRACK_PERIOD_MS. */
        servo_track_hottest_ir_point();
        servo_fan_gate_tick();
        servo_ramp_tick();
    }
}

/* Auto-fan: only intervenes while auto-tracking is on. Manual
 * mode (joystick) leaves fan duty alone — the user is in control.
 * On boot or after a long no-iron stretch we cut the fan; the
 * moment the tracker confirms iron we restore full duty. */
static void servo_fan_gate_tick(void)
{
    if (!g_servo_track_enabled)
    {
        return;
    }

    TickType_t now = xTaskGetTickCount();
    uint8_t fan_now = task_fan_get_duty();
    bool iron_recent = (g_last_iron_seen_ts != 0u) &&
                       ((now - g_last_iron_seen_ts) <
                        pdMS_TO_TICKS(FAN_AUTO_OFF_TIMEOUT_MS));

    if (iron_recent && fan_now == 0u)
    {
        task_fan_set_duty(FAN_AUTO_ON_DUTY);
    }
    else if (!iron_recent && fan_now != 0u)
    {
        task_fan_set_duty(0u);
    }
}

static BaseType_t cli_handler_servo_pan(
    char *pcWriteBuffer,
    size_t xWriteBufferLen,
    const char *pcCommandString)
{
    servo_ctrl_message_t msg;
    int32_t pan_deg;

    configASSERT(pcWriteBuffer);
    memset(pcWriteBuffer, 0, xWriteBufferLen);

    if (!parse_deg_param(pcCommandString, 1, &pan_deg))
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: servo_pan requires numeric angle\r\n");
        return pdFALSE;
    }

    msg.set_pan = true;
    msg.set_tilt = false;
    msg.cmd = SERVO_CTRL_CMD_SET_ANGLE;
    msg.pan_deg = servo_clamp_deg(pan_deg);
    msg.tilt_deg = 0u;

    if (xQueueSendToBack(q_servo_ctrl, &msg, pdMS_TO_TICKS(100)) != pdPASS)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: failed to send pan command\r\n");
        return pdFALSE;
    }

    snprintf(pcWriteBuffer, xWriteBufferLen, "Pan set to %u deg\r\n", (unsigned int)msg.pan_deg);
    return pdFALSE;
}

static BaseType_t cli_handler_servo_tilt(
    char *pcWriteBuffer,
    size_t xWriteBufferLen,
    const char *pcCommandString)
{
    servo_ctrl_message_t msg;
    int32_t tilt_deg;

    configASSERT(pcWriteBuffer);
    memset(pcWriteBuffer, 0, xWriteBufferLen);

    if (!parse_deg_param(pcCommandString, 1, &tilt_deg))
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: servo_tilt requires numeric angle\r\n");
        return pdFALSE;
    }

    msg.set_pan = false;
    msg.set_tilt = true;
    msg.cmd = SERVO_CTRL_CMD_SET_ANGLE;
    msg.pan_deg = 0u;
    msg.tilt_deg = servo_clamp_deg(tilt_deg);

    if (xQueueSendToBack(q_servo_ctrl, &msg, pdMS_TO_TICKS(100)) != pdPASS)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: failed to send tilt command\r\n");
        return pdFALSE;
    }

    snprintf(pcWriteBuffer, xWriteBufferLen, "Tilt set to %u deg\r\n", (unsigned int)msg.tilt_deg);
    return pdFALSE;
}

static BaseType_t cli_handler_servo_set(
    char *pcWriteBuffer,
    size_t xWriteBufferLen,
    const char *pcCommandString)
{
    servo_ctrl_message_t msg;
    int32_t pan_deg;
    int32_t tilt_deg;

    configASSERT(pcWriteBuffer);
    memset(pcWriteBuffer, 0, xWriteBufferLen);

    if (!parse_deg_param(pcCommandString, 1, &pan_deg) || !parse_deg_param(pcCommandString, 2, &tilt_deg))
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: servo_set requires two numeric angles\r\n");
        return pdFALSE;
    }

    msg.set_pan = true;
    msg.set_tilt = true;
    msg.cmd = SERVO_CTRL_CMD_SET_ANGLE;
    msg.pan_deg = servo_clamp_deg(pan_deg);
    msg.tilt_deg = servo_clamp_deg(tilt_deg);

    if (xQueueSendToBack(q_servo_ctrl, &msg, pdMS_TO_TICKS(100)) != pdPASS)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: failed to send servo_set command\r\n");
        return pdFALSE;
    }

    snprintf(
        pcWriteBuffer,
        xWriteBufferLen,
        "Pan=%u deg, Tilt=%u deg\r\n",
        (unsigned int)msg.pan_deg,
        (unsigned int)msg.tilt_deg);

    return pdFALSE;
}

static BaseType_t cli_handler_servo_cal(
    char *pcWriteBuffer,
    size_t xWriteBufferLen,
    const char *pcCommandString)
{
    servo_ctrl_message_t msg;
    int32_t start_us;
    int32_t end_us;
    int32_t step_us = (int32_t)SERVO_CAL_DEFAULT_STEP_US;
    int32_t dwell_ms = (int32_t)SERVO_CAL_DEFAULT_DWELL_MS;
    int32_t delta_us;
    uint32_t steps;
    int32_t temp;
    const char *axis_name;

    configASSERT(pcWriteBuffer);
    memset(pcWriteBuffer, 0, xWriteBufferLen);

    if (!parse_axis_param(pcCommandString, 1, &msg.sweep_axis) ||
        !parse_deg_param(pcCommandString, 2, &start_us) ||
        !parse_deg_param(pcCommandString, 3, &end_us))
    {
        snprintf(pcWriteBuffer,
                 xWriteBufferLen,
                 "Usage: servo_cal <pan|tilt> <start_us> <end_us> [step_us] [dwell_ms]\r\n");
        return pdFALSE;
    }

    if (parse_deg_param(pcCommandString, 4, &temp))
    {
        step_us = temp;
    }

    if (parse_deg_param(pcCommandString, 5, &temp))
    {
        dwell_ms = temp;
    }

    if ((start_us < (int32_t)SERVO_CAL_MIN_PULSE_US) || (start_us > (int32_t)SERVO_CAL_MAX_PULSE_US) ||
        (end_us < (int32_t)SERVO_CAL_MIN_PULSE_US) || (end_us > (int32_t)SERVO_CAL_MAX_PULSE_US))
    {
        snprintf(pcWriteBuffer,
                 xWriteBufferLen,
                 "Error: pulse range must stay within %u-%u us\r\n",
                 (unsigned int)SERVO_CAL_MIN_PULSE_US,
                 (unsigned int)SERVO_CAL_MAX_PULSE_US);
        return pdFALSE;
    }

    if (step_us <= 0)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: step_us must be > 0\r\n");
        return pdFALSE;
    }

    if (dwell_ms < 20)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: dwell_ms must be >= 20\r\n");
        return pdFALSE;
    }

    delta_us = start_us - end_us;
    if (delta_us < 0)
    {
        delta_us = -delta_us;
    }

    steps = ((uint32_t)delta_us / (uint32_t)step_us) + 1u;
    if (steps > SERVO_CAL_MAX_STEPS)
    {
        snprintf(pcWriteBuffer,
                 xWriteBufferLen,
                 "Error: sweep has %lu steps (max %u). Increase step_us.\r\n",
                 (unsigned long)steps,
                 (unsigned int)SERVO_CAL_MAX_STEPS);
        return pdFALSE;
    }

    msg.cmd = SERVO_CTRL_CMD_SWEEP;
    msg.set_pan = false;
    msg.set_tilt = false;
    msg.pan_deg = 0u;
    msg.tilt_deg = 0u;
    msg.sweep_start_us = (uint32_t)start_us;
    msg.sweep_end_us = (uint32_t)end_us;
    msg.sweep_step_us = (uint32_t)step_us;
    msg.sweep_dwell_ms = (uint32_t)dwell_ms;

    if (xQueueSendToBack(q_servo_ctrl, &msg, pdMS_TO_TICKS(100)) != pdPASS)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: failed to start servo_cal sweep\r\n");
        return pdFALSE;
    }

    axis_name = (msg.sweep_axis == SERVO_AXIS_PAN) ? "pan" : "tilt";
    snprintf(pcWriteBuffer,
             xWriteBufferLen,
             "Started servo_cal %s sweep (%ld -> %ld us, step=%ld, dwell=%ld ms)\r\n",
             axis_name,
             (long)start_us,
             (long)end_us,
             (long)step_us,
             (long)dwell_ms);

    return pdFALSE;
}

static BaseType_t cli_handler_servo_track(
    char *pcWriteBuffer,
    size_t xWriteBufferLen,
    const char *pcCommandString)
{
    const char *pcParameter;
    BaseType_t xParameterStringLength;
    char parameter_buffer[16];
    size_t copy_len;

    configASSERT(pcWriteBuffer);
    memset(pcWriteBuffer, 0, xWriteBufferLen);

    pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength);
    if ((pcParameter == NULL) || (xParameterStringLength <= 0))
    {
        snprintf(pcWriteBuffer,
                 xWriteBufferLen,
                 "Auto-tracking is %s (usage: servo_track [on|off|status])\r\n",
                 g_servo_track_enabled ? "ON" : "OFF");
        return pdFALSE;
    }

    copy_len = (size_t)xParameterStringLength;
    if (copy_len > (sizeof(parameter_buffer) - 1u))
    {
        copy_len = sizeof(parameter_buffer) - 1u;
    }

    memset(parameter_buffer, 0, sizeof(parameter_buffer));
    strncpy(parameter_buffer, pcParameter, copy_len);
    parameter_buffer[copy_len] = '\0';

    if ((strcmp(parameter_buffer, "on") == 0) || (strcmp(parameter_buffer, "1") == 0) ||
        (strcmp(parameter_buffer, "enable") == 0))
    {
        task_servo_ctrl_set_tracking(true);
        snprintf(pcWriteBuffer, xWriteBufferLen, "Auto-tracking enabled\r\n");
        return pdFALSE;
    }

    if ((strcmp(parameter_buffer, "off") == 0) || (strcmp(parameter_buffer, "0") == 0) ||
        (strcmp(parameter_buffer, "disable") == 0))
    {
        task_servo_ctrl_set_tracking(false);
        snprintf(pcWriteBuffer, xWriteBufferLen, "Auto-tracking disabled\r\n");
        return pdFALSE;
    }

    if (strcmp(parameter_buffer, "status") == 0)
    {
        snprintf(pcWriteBuffer,
                 xWriteBufferLen,
                 "Auto-tracking is %s\r\n",
                 g_servo_track_enabled ? "ON" : "OFF");
        return pdFALSE;
    }

    snprintf(pcWriteBuffer,
             xWriteBufferLen,
             "Error: expected on/off/status (got '%s')\r\n",
             parameter_buffer);
    return pdFALSE;
}

/*******************************************************************************/
/* Public Function Definitions                                                 */
/*******************************************************************************/

void task_servo_ctrl_init(void)
{
    cy_rslt_t rslt;

    rslt = cyhal_pwm_init(&g_pwm_pan, SERVO_PAN_PWM_PIN, NULL);
    if (rslt != CY_RSLT_SUCCESS)
    {
        task_print_error("Failed to initialize pan PWM (0x%08lX)", (unsigned long)rslt);
        return;
    }

    rslt = cyhal_pwm_init(&g_pwm_tilt, SERVO_TILT_PWM_PIN, NULL);
    if (rslt != CY_RSLT_SUCCESS)
    {
        task_print_error("Failed to initialize tilt PWM (0x%08lX)", (unsigned long)rslt);
        return;
    }

    if (!servo_set_angle(&g_pwm_pan, g_pan_deg, SERVO_PAN_MIN_PULSE_US, SERVO_PAN_MAX_PULSE_US, SERVO_PAN_INVERT) ||
        !servo_set_angle(&g_pwm_tilt, g_tilt_deg, SERVO_TILT_MIN_PULSE_US, SERVO_TILT_MAX_PULSE_US, SERVO_TILT_INVERT))
    {
        task_print_error("Failed to configure default servo angles");
        return;
    }

    rslt = cyhal_pwm_start(&g_pwm_pan);
    if (rslt != CY_RSLT_SUCCESS)
    {
        task_print_error("Failed to start pan PWM (0x%08lX)", (unsigned long)rslt);
        return;
    }

    rslt = cyhal_pwm_start(&g_pwm_tilt);
    if (rslt != CY_RSLT_SUCCESS)
    {
        task_print_error("Failed to start tilt PWM (0x%08lX)", (unsigned long)rslt);
        return;
    }

    q_servo_ctrl = xQueueCreate(SERVO_CTRL_QUEUE_LEN, sizeof(servo_ctrl_message_t));
    if (q_servo_ctrl == NULL)
    {
        task_print_error("Failed to create servo control queue");
        return;
    }

    FreeRTOS_CLIRegisterCommand(&xServoPan);
    FreeRTOS_CLIRegisterCommand(&xServoTilt);
    FreeRTOS_CLIRegisterCommand(&xServoSet);
    FreeRTOS_CLIRegisterCommand(&xServoCal);
    FreeRTOS_CLIRegisterCommand(&xServoTrack);

    xTaskCreate(
        task_servo_ctrl,
        "Task_Servo_Ctrl",
        configMINIMAL_STACK_SIZE * 2,
        NULL,
        configMAX_PRIORITIES - 6,
        NULL);

    task_print_info("Servo control initialized (pan=%u deg, tilt=%u deg)",
                    (unsigned int)g_pan_deg,
                    (unsigned int)g_tilt_deg);
    task_print_info("Pan pulse range: %lu-%lu us, invert=%u",
                    (unsigned long)SERVO_PAN_MIN_PULSE_US,
                    (unsigned long)SERVO_PAN_MAX_PULSE_US,
                    (unsigned int)SERVO_PAN_INVERT);
    task_print_info("Tilt pulse range: %lu-%lu us, invert=%u",
                    (unsigned long)SERVO_TILT_MIN_PULSE_US,
                    (unsigned long)SERVO_TILT_MAX_PULSE_US,
                    (unsigned int)SERVO_TILT_INVERT);
    task_print_info("Servo auto-tracking: %s", g_servo_track_enabled ? "ON" : "OFF");
}

bool task_servo_ctrl_get_tracking(void)
{
    return g_servo_track_enabled;
}

void task_servo_ctrl_set_tracking(bool enable)
{
    if (g_servo_track_enabled == enable)
    {
        return;
    }
    /* When turning tracking back ON, drain any queued manual servo
     * commands. Otherwise a still-pending drag cmd from BLE arrives
     * AFTER set_tracking(true), trips the manual-override path, and
     * flips tracking right back off — the exact race the desktop
     * "release sends track:1" path was meant to avoid. */
    if (enable && (q_servo_ctrl != NULL))
    {
        servo_ctrl_message_t drain;
        while (xQueueReceive(q_servo_ctrl, &drain, 0u) == pdPASS) { }
    }
    g_servo_track_enabled = enable;
    /* Any explicit user-driven mode change clears a stuck TOF
     * suppression so a leftover too-close trip doesn't keep the
     * tracker frozen when the user re-engages auto-track. */
    g_servo_track_suppressed = false;
    /* Parseable telemetry token for desktop GUI tracking dot. */
    task_print_info("track:%u", enable ? 1u : 0u);
}

void task_servo_ctrl_suppress_tracking(bool suppress)
{
    if (g_servo_track_suppressed == suppress)
    {
        return;
    }
    g_servo_track_suppressed = suppress;
    task_print_info("track-suppress:%u", suppress ? 1u : 0u);
}
