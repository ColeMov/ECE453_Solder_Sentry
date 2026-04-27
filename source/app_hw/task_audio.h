#ifndef __TASK_AUDIO_H__
#define __TASK_AUDIO_H__

#include "main.h"

#define AUDIO_I2C_SCL_PIN     P9_0
#define AUDIO_I2C_SDA_PIN     P9_1
#define AUDIO_I2C_FREQ_HZ     (100000u)
#define AUDIO_DAC_PIN         P9_6

void task_audio_init(void);

#endif /* __TASK_AUDIO_H__ */
