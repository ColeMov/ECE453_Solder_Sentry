/**
 * @file task_captouch.h
 * @brief IQS228B capacitive touch sensor task - detects copper pad touch
 *
 * IQS228B/D is a single-channel capacitive proximity and touch controller.
 * The copper pad (sense electrode) connects to the Cx pin of the IQS228B.
 * The TOUT pin outputs touch state: HIGH when touched (Active High) or LOW (Active Low).
 *
 * @see IQS228B/D Datasheet - Azoteq ProxSense Series
 */
#ifndef __TASK_CAPTOUCH_H__
#define __TASK_CAPTOUCH_H__

#include "main.h"
#include "ece453_pins.h"

/* IQS228B TOUT pin (touch output). Set this to the MCU GPIO you wired to TOUT. */
#define IQS228B_TOUT_PIN    MOD_1_PIN_I2C_SCL  /* P9_0 */

/* Set to 1 if IQS228B is configured Active High (touch = logic 1), 0 for Active Low */
#define IQS228B_ACTIVE_HIGH (0)

void task_captouch_init(void);

#endif /* __TASK_CAPTOUCH_H__ */
