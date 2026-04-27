#ifndef __TASK_AUDIO_H__
#define __TASK_AUDIO_H__

#include "main.h"

#define AUDIO_I2C_SCL_PIN     P9_0
#define AUDIO_I2C_SDA_PIN     P9_1
#define AUDIO_I2C_FREQ_HZ     (100000u)
#define AUDIO_DAC_PIN         P9_6

void task_audio_init(void);

/* Trigger playback of a stored voice prompt by name. Names match the
 * keys passed to wav_to_c.py (e.g. "pairing_on", "connected"). Safe to
 * call from any task; spawns a one-shot playback task. Returns true if
 * a clip was queued, false if name unknown or DAC not ready. */
bool task_audio_say(const char *clip_name);

#endif /* __TASK_AUDIO_H__ */
