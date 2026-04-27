/**
 * @file task_fan.c
 * @brief Fan PWM control on P10.2 with CLI access.
 */

#include "task_fan.h"
#include "task_console.h"
#include "FreeRTOS_CLI.h"
#include "cyhal_pwm.h"
#include <stdlib.h>
#include <string.h>

#define FAN_PWM_PIN          P10_2
#define FAN_PWM_FREQUENCY_HZ (25000u)
#define FAN_DEFAULT_DUTY     (100u)

/* Intel-spec 4-pin PC fans treat "no PWM edges" as a signal-loss failsafe
 * and run at full speed. Drive a low-but-nonzero duty so `fan 0` still
 * means "as slow as possible" rather than "max speed". A true off needs
 * a hardware MOSFET on the V+ rail. */
#define FAN_MIN_DUTY         (1u)

static cyhal_pwm_t g_fan_pwm;
static volatile uint8_t g_fan_duty = 0u;
static bool g_fan_ready = false;

static BaseType_t cli_handler_fan(char *pcWriteBuffer,
                                  size_t xWriteBufferLen,
                                  const char *pcCommandString)
{
    const char *pcParameter;
    BaseType_t xParameterStringLength;

    configASSERT(pcWriteBuffer);
    memset(pcWriteBuffer, 0, xWriteBufferLen);

    pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength);

    if (pcParameter == NULL)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "fan: duty=%u%% pin=P10.2 freq=%lu Hz ready=%d\r\n",
                 (unsigned)g_fan_duty,
                 (unsigned long)FAN_PWM_FREQUENCY_HZ,
                 (int)g_fan_ready);
        return pdFALSE;
    }

    int v = atoi(pcParameter);
    if (v < 0)   v = 0;
    if (v > 100) v = 100;

    task_fan_set_duty((uint8_t)v);

    snprintf(pcWriteBuffer, xWriteBufferLen,
             "fan: duty set to %d%%\r\n", v);
    return pdFALSE;
}

static const CLI_Command_Definition_t xFan =
{
    "fan",
    "\r\nfan [0-100] : set fan duty cycle, no arg = status\r\n",
    cli_handler_fan,
    -1
};

void task_fan_set_duty(uint8_t duty_pct)
{
    if (!g_fan_ready)
    {
        return;
    }
    if (duty_pct > 100u)
    {
        duty_pct = 100u;
    }
    if (duty_pct < FAN_MIN_DUTY)
    {
        duty_pct = FAN_MIN_DUTY;
    }

    cy_rslt_t rslt = cyhal_pwm_set_duty_cycle(&g_fan_pwm,
                                              (float)duty_pct,
                                              FAN_PWM_FREQUENCY_HZ);
    if (rslt != CY_RSLT_SUCCESS)
    {
        task_print_error("fan: set_duty %u%% failed (0x%08lX)",
                         (unsigned)duty_pct, (unsigned long)rslt);
        return;
    }

    g_fan_duty = duty_pct;
}

uint8_t task_fan_get_duty(void)
{
    return g_fan_duty;
}

void task_fan_init(void)
{
    cy_rslt_t rslt = cyhal_pwm_init(&g_fan_pwm, FAN_PWM_PIN, NULL);
    if (rslt != CY_RSLT_SUCCESS)
    {
        task_print_error("fan: pwm_init failed (0x%08lX)", (unsigned long)rslt);
        return;
    }

    rslt = cyhal_pwm_set_duty_cycle(&g_fan_pwm,
                                    (float)FAN_DEFAULT_DUTY,
                                    FAN_PWM_FREQUENCY_HZ);
    if (rslt != CY_RSLT_SUCCESS)
    {
        task_print_error("fan: set_duty_cycle failed (0x%08lX)", (unsigned long)rslt);
        return;
    }

    rslt = cyhal_pwm_start(&g_fan_pwm);
    if (rslt != CY_RSLT_SUCCESS)
    {
        task_print_error("fan: pwm_start failed (0x%08lX)", (unsigned long)rslt);
        return;
    }

    g_fan_duty = FAN_DEFAULT_DUTY;
    g_fan_ready = true;

    FreeRTOS_CLIRegisterCommand(&xFan);

    task_print_info("fan: PWM up on P10.2 @ %lu Hz, duty %u%%",
                    (unsigned long)FAN_PWM_FREQUENCY_HZ,
                    (unsigned)FAN_DEFAULT_DUTY);
}
