#ifndef __TASK_BLINK_H__
#define __TASK_BLINK_H__

#include "main.h"

#define PIN_LED		P7_1

/* Blink message type */
typedef enum
{
    BLINK_OFF = 0,
    BLINK_ON = 1
} blink_message_type_t;

extern QueueHandle_t q_blink;
void task_blink_init(void);

#endif
