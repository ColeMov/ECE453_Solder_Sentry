#ifndef __TASK_FAN_H__
#define __TASK_FAN_H__

#include "main.h"

void task_fan_init(void);

/* Set fan duty cycle, 0-100. Clamped to FAN_MIN_DUTY at the low end so a
 * 4-pin PWM fan honors "slow" instead of failing safe to full speed. */
void task_fan_set_duty(uint8_t duty_pct);

/* Read currently-commanded duty cycle, 0-100. */
uint8_t task_fan_get_duty(void);

#endif /* __TASK_FAN_H__ */
