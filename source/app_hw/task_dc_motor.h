#ifndef __TASK_DC_MOTOR_H__
#define __TASK_DC_MOTOR_H__

#include "main.h"
#include "cyhal.h"
#include "cyhal_pwm.h"
#include "task_console.h"

/**
 * H-Bridge Control Pins (DRV8251)
 * IN1 and IN2 control the direction via the following truth table:
 *   IN1=0, IN2=0: Coast (High-Z, disabled)
 *   IN1=0, IN2=1: Reverse (OUT2→OUT1)
 *   IN1=1, IN2=0: Forward (OUT1→OUT2)
 *   IN1=1, IN2=1: Brake (low-side slow decay)
 *
 * For speed control, PWM is applied to the active direction pin.
 */
#define DC_MOTOR_IN1_PIN       P5_4  /* H-bridge control input 1 */
#define DC_MOTOR_IN2_PIN       P5_5  /* H-bridge control input 2 */
#define DC_MOTOR_PWM_FREQUENCY 20000 /* 20 kHz PWM frequency for motor control */

/**
 * @brief DC Motor direction enumeration
 */
typedef enum
{
    DC_MOTOR_COAST = 0,
    DC_MOTOR_FORWARD = 1,
    DC_MOTOR_REVERSE = 2,
    DC_MOTOR_BRAKE = 3
} dc_motor_direction_t;

/**
 * @brief DC Motor command message structure
 */
typedef struct
{
    dc_motor_direction_t direction;  /* Motor direction (forward/reverse/stop) */
    uint8_t speed_percent;           /* Motor speed as percentage (0-100) */
} dc_motor_message_t;

extern QueueHandle_t q_dc_motor;
void task_dc_motor_init(void);

#endif