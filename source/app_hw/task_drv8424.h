/**
 * @file task_drv8424.h
 * @author Tanay Gupta (tgupta43@wisc.edu)
 * @brief
 * DRV8424 stepper motor driver interface header.
 *
 * Control pins used by this driver:
 *   STEP  - Rising edge advances one microstep
 *   DIR   - Sets rotation direction (high = CW, low = CCW)
 *   nSLEEP - Active-low sleep (low = sleep, high = active)
 *   EN/nFAULT - Enable input / fault output (active-high enable, driven low on fault)
 *
 * Microstepping mode is configured externally via M0/M1 hardware pins.
 *
 * @version 0.1
 * @date 2026-02-16
 *
 * @copyright Copyright (c) 2026
 *
 */
#ifndef __TASK_DRV8424_H__
#define __TASK_DRV8424_H__

#include "main.h"
#include "cyhal.h"
#include "task_console.h"

/*
 * DRV8424 control pins — adjust to match your board wiring.
 *
 * STEP and DIR must be on pins capable of fast GPIO toggling.
 * nSLEEP and nENBL are active-low control pins.
 */
#define DRV8424_STEP_PIN       P5_4   /* STEP input to DRV8424 */
#define DRV8424_DIR_PIN        P5_5   /* DIR input to DRV8424 */
#define DRV8424_NSLEEP_PIN     P10_2  /* nSLEEP — low = sleep, high = active */
#define DRV8424_NFAULT_PIN     P10_4  /* EN/nFAULT — input (active-low fault) */

/*
 * Timing parameters (from DRV8424 datasheet SLOSE54C).
 * tWAKE: time after nSLEEP goes high before device accepts STEP inputs.
 * Minimum STEP pulse width: ~1.7 us (typ), we use generous margin.
 */
#define DRV8424_TWAKE_MS           10u   /* Wake-up delay after nSLEEP asserted high */
#define DRV8424_STEP_PULSE_US      5u    /* Minimum STEP high pulse width in microseconds */

/*
 * Stepping speed limits.
 * Step delay is the time between consecutive rising edges on STEP.
 * Lower delay = faster stepping.
 */
#define DRV8424_STEP_DELAY_MIN_MS  1u    /* Fastest stepping (1 ms between steps) */
#define DRV8424_STEP_DELAY_MAX_MS  50u   /* Slowest stepping (50 ms between steps) */

/**
 * @brief DRV8424 motor direction / command enumeration
 */
typedef enum
{
    DRV8424_STOP  = 0,  /* Stop stepping, optionally enter sleep */
    DRV8424_CW    = 1,  /* Step clockwise (DIR = high) */
    DRV8424_CCW   = 2   /* Step counter-clockwise (DIR = low) */
} drv8424_direction_t;

/**
 * @brief DRV8424 command message sent via FreeRTOS queue
 */
typedef struct
{
    drv8424_direction_t direction;   /* Desired direction or stop */
    uint8_t speed_percent;           /* Speed 0-100: maps to step delay */
    uint16_t steps;                  /* Number of steps (0 = continuous) */
} drv8424_message_t;

/**
 * @brief Queue used by other tasks / CLI to send commands to the DRV8424 task
 */
extern QueueHandle_t q_drv8424;

/**
 * @brief Initialize the DRV8424 driver task, GPIO pins, and CLI command.
 *
 * Call this once from main() before vTaskStartScheduler().
 */
void task_drv8424_init(void);

#endif /* __TASK_DRV8424_H__ */
