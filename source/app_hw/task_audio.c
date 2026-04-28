/**
 * @file task_audio.c
 * @brief TAS54414 audio amp bring-up over I2C on Module Site 1.
 *
 * Step 1: probe the bus to find the amp's I2C address. All bring-up work
 * runs in a deferred FreeRTOS task so a stuck bus (no pullups, missing
 * power) cannot block boot — the HAL spins forever in cyhal_i2c_init when
 * SDA/SCL are stuck low at startup.
 *
 * Uses its own cyhal_i2c_t (separate SCB from i2c.c's MODULE_SITE_0) so it
 * does not interfere with the IR sensor on P10.0/P10.1.
 */

#include "task_audio.h"
#include "task_console.h"
#include "FreeRTOS_CLI.h"
#include "audio_clips.h"
#include "cyhal_i2c.h"
#include "cyhal_gpio.h"
#include "cyhal_dac.h"
#include "cyhal_system.h"
#include <stdlib.h>
#include <string.h>

#define TAS5441_I2C_ADDR    (0x6Cu)

static cyhal_i2c_t g_audio_i2c;
static bool g_audio_i2c_ready = false;

static cyhal_dac_t g_audio_dac;
static bool g_audio_dac_ready = false;
static volatile bool g_audio_play_busy = false;

/* Tone generator state. The CLI 'audio tone' starts a one-shot FreeRTOS task
 * that toggles the DAC at audio rate to produce a square wave for a fixed
 * duration. */
typedef struct {
    uint32_t freq_hz;
    uint32_t duration_ms;
} audio_tone_params_t;

/* PCM playback. Feeds 8-bit unsigned samples at AUDIO_CLIP_RATE_HZ (8 kHz) by
 * busy-waiting between writes — pinned to a dedicated FreeRTOS task so other
 * priority-> tasks (BLE link, IR streaming) can still preempt. */
typedef struct {
    const int16_t *pcm;
    uint32_t       len;
    const char    *name;
} audio_clip_play_t;

static void task_audio_play(void *param)
{
    audio_clip_play_t *p = (audio_clip_play_t *)param;
    const int16_t *pcm = p->pcm;
    uint32_t       len = p->len;
    const char    *name = p->name;
    vPortFree(p);

    if (!g_audio_dac_ready)
    {
        task_print_error("audio: DAC not ready");
        vTaskDelete(NULL);
        return;
    }

    /* cyhal_dac_write takes a few us; trim the busy-wait so total period
     * matches AUDIO_CLIP_RATE_HZ. Tune empirically. */
    #define AUDIO_PLAY_DELAY_TRIM_US (5u)
    uint16_t period_us = (uint16_t)(1000000u / AUDIO_CLIP_RATE_HZ);
    if (period_us > AUDIO_PLAY_DELAY_TRIM_US) period_us -= AUDIO_PLAY_DELAY_TRIM_US;

    /* 16-bit signed PCM (-32768..32767) -> DAC midpoint 32768. Half-swing
     * (sample >> 1) to give the amp some headroom and avoid clipping. */
    for (uint32_t i = 0u; i < len; i++)
    {
        int32_t v = 32768 + (pcm[i] >> 1);
        if (v < 0)     v = 0;
        if (v > 65535) v = 65535;
        cyhal_dac_write(&g_audio_dac, (uint16_t)v);
        cyhal_system_delay_us(period_us);
    }
    cyhal_dac_write(&g_audio_dac, 32768u);
    task_print_info("audio: clip '%s' done (%lu samples)",
                    name ? name : "?", (unsigned long)len);
    g_audio_play_busy = false;
    vTaskDelete(NULL);
}

/* Lookup table — single source of truth for clip name → PCM data. */
static const struct {
    const char     *name;
    const int16_t  *pcm;
    uint32_t        len;
} g_audio_clips[] = {
    { "pairing_on",     pairing_on_pcm,     PAIRING_ON_PCM_LEN     },
    { "pairing_off",    pairing_off_pcm,    PAIRING_OFF_PCM_LEN    },
    { "connected",      connected_pcm,      CONNECTED_PCM_LEN      },
    { "disconnected",   disconnected_pcm,   DISCONNECTED_PCM_LEN   },
    { "too_close",      too_close_pcm,      TOO_CLOSE_PCM_LEN      },
    { "pairing_chime",  pairing_chime_pcm,  PAIRING_CHIME_PCM_LEN  },
    { "success_chime",  success_chime_pcm,  SUCCESS_CHIME_PCM_LEN  },
};
#define AUDIO_CLIP_COUNT (sizeof(g_audio_clips) / sizeof(g_audio_clips[0]))

static cy_rslt_t audio_dispatch_clip(const int16_t *pcm, uint32_t len, const char *name)
{
    if (!g_audio_dac_ready)
    {
        return CY_RSLT_TYPE_ERROR;
    }
    /* Drop the request if a clip is already playing — prevents two
     * busy-wait playback tasks from running concurrently and stomping
     * on each other's DAC writes (which sounds like glitchy garbage). */
    if (g_audio_play_busy)
    {
        return CY_RSLT_TYPE_ERROR;
    }
    audio_clip_play_t *p = pvPortMalloc(sizeof(*p));
    if (p == NULL)
    {
        return CY_RSLT_TYPE_ERROR;
    }
    p->pcm = pcm;
    p->len = len;
    p->name = name;
    g_audio_play_busy = true;
    BaseType_t ok = xTaskCreate(task_audio_play, "AudPlay",
                                3 * configMINIMAL_STACK_SIZE,
                                p, tskIDLE_PRIORITY + 2, NULL);
    if (ok != pdPASS)
    {
        g_audio_play_busy = false;
        vPortFree(p);
        return CY_RSLT_TYPE_ERROR;
    }
    return CY_RSLT_SUCCESS;
}

static void task_audio_tone(void *param)
{
    audio_tone_params_t *p = (audio_tone_params_t *)param;
    uint32_t freq = p->freq_hz;
    uint32_t duration_ms = p->duration_ms;
    vPortFree(p);

    if (!g_audio_dac_ready || freq == 0u)
    {
        vTaskDelete(NULL);
        return;
    }

    /* Half period in microseconds. cyhal_system_delay_us busy-waits, but
     * this task runs at low priority so other tasks still preempt. */
    uint32_t half_us = 500000u / freq;
    if (half_us < 10u) half_us = 10u;

    TickType_t end_tick = xTaskGetTickCount() + pdMS_TO_TICKS(duration_ms);

    while (xTaskGetTickCount() < end_tick)
    {
        cyhal_dac_write(&g_audio_dac, 60000u);   /* near full scale */
        cyhal_system_delay_us((uint16_t)half_us);
        cyhal_dac_write(&g_audio_dac, 5000u);    /* near ground */
        cyhal_system_delay_us((uint16_t)half_us);
    }

    cyhal_dac_write(&g_audio_dac, 32768u);  /* park at mid-rail */
    task_print_info("audio: tone done");
    vTaskDelete(NULL);
}

static cy_rslt_t audio_read_reg(uint8_t addr_7bit, uint8_t reg, uint8_t *out)
{
    return cyhal_i2c_master_mem_read(&g_audio_i2c,
                                     (uint16_t)addr_7bit,
                                     (uint16_t)reg, 1u,
                                     out, 1u, 50u);
}

static cy_rslt_t audio_write_reg(uint8_t addr_7bit, uint8_t reg, uint8_t val)
{
    return cyhal_i2c_master_mem_write(&g_audio_i2c,
                                      (uint16_t)addr_7bit,
                                      (uint16_t)reg, 1u,
                                      &val, 1u, 50u);
}

static const cyhal_i2c_cfg_t g_audio_i2c_cfg =
{
    .is_slave        = false,
    .address         = 0,
    .frequencyhal_hz = AUDIO_I2C_FREQ_HZ,
};

/* Probe a 7-bit I2C address by attempting a zero-byte write. ACK = device
 * present. Standard probe trick. */
static bool audio_i2c_addr_acks(uint8_t addr_7bit)
{
    if (!g_audio_i2c_ready)
    {
        return false;
    }
    uint8_t dummy = 0u;
    cy_rslt_t rslt = cyhal_i2c_master_write(&g_audio_i2c,
                                            (uint16_t)addr_7bit,
                                            &dummy, 0u, 50u, true);
    return (rslt == CY_RSLT_SUCCESS);
}

static BaseType_t cli_handler_audio(char *pcWriteBuffer,
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
                 "audio: subcommands -> probe | status\r\n");
        return pdFALSE;
    }

    if (strncmp(pcParameter, "probe", 5) == 0)
    {
        if (!g_audio_i2c_ready)
        {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "audio: I2C not initialized\r\n");
            return pdFALSE;
        }

        size_t off = 0;
        off += snprintf(pcWriteBuffer + off, xWriteBufferLen - off,
                        "audio: probing 0x03-0x77 on P9.0/P9.1 @ %lu Hz\r\n",
                        (unsigned long)AUDIO_I2C_FREQ_HZ);

        unsigned found = 0u;
        for (uint8_t a = 0x03u; a <= 0x77u; a++)
        {
            if (audio_i2c_addr_acks(a))
            {
                if ((xWriteBufferLen - off) > 24u)
                {
                    off += snprintf(pcWriteBuffer + off, xWriteBufferLen - off,
                                    "audio: ACK at 0x%02X\r\n", a);
                }
                found++;
            }
        }

        if ((xWriteBufferLen - off) > 28u)
        {
            snprintf(pcWriteBuffer + off, xWriteBufferLen - off,
                     "audio: probe done, %u devices found\r\n", found);
        }
        return pdFALSE;
    }

    if (strncmp(pcParameter, "status", 6) == 0)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "audio: i2c_ready=%d pins=P9.0/P9.1 freq=%lu\r\n",
                 (int)g_audio_i2c_ready,
                 (unsigned long)AUDIO_I2C_FREQ_HZ);
        return pdFALSE;
    }

    if (strncmp(pcParameter, "say", 3) == 0)
    {
        const char *p_name = FreeRTOS_CLIGetParameter(pcCommandString, 2, &xParameterStringLength);
        if (p_name == NULL)
        {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "audio say <pairing_on|pairing_off|connected|disconnected|too_close>\r\n");
            return pdFALSE;
        }
        bool ok = task_audio_say(p_name);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "audio: say %s -> %s\r\n",
                 p_name, ok ? "queued" : "unknown clip or DAC not ready");
        return pdFALSE;
    }

    if (strncmp(pcParameter, "tone", 4) == 0)
    {
        if (!g_audio_dac_ready)
        {
            snprintf(pcWriteBuffer, xWriteBufferLen, "audio: DAC not initialized\r\n");
            return pdFALSE;
        }

        /* Optional second arg = frequency in Hz (default 1000). */
        const char *p_freq = FreeRTOS_CLIGetParameter(pcCommandString, 2, &xParameterStringLength);
        uint32_t freq_hz = 1000u;
        if (p_freq != NULL)
        {
            int v = atoi(p_freq);
            if (v > 50 && v < 20000) freq_hz = (uint32_t)v;
        }

        audio_tone_params_t *p = pvPortMalloc(sizeof(audio_tone_params_t));
        if (p == NULL)
        {
            snprintf(pcWriteBuffer, xWriteBufferLen, "audio: tone alloc failed\r\n");
            return pdFALSE;
        }
        p->freq_hz = freq_hz;
        p->duration_ms = 2000u;

        BaseType_t ok = xTaskCreate(task_audio_tone, "AudTone",
                                    3 * configMINIMAL_STACK_SIZE,
                                    p, tskIDLE_PRIORITY + 2, NULL);
        if (ok != pdPASS)
        {
            vPortFree(p);
            snprintf(pcWriteBuffer, xWriteBufferLen, "audio: tone task create failed\r\n");
            return pdFALSE;
        }
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "audio: tone %lu Hz for 2 s on P9.6\r\n", (unsigned long)freq_hz);
        return pdFALSE;
    }

    if (strncmp(pcParameter, "dc", 2) == 0)
    {
        if (!g_audio_dac_ready)
        {
            snprintf(pcWriteBuffer, xWriteBufferLen, "audio: DAC not initialized\r\n");
            return pdFALSE;
        }
        /* 'audio dc <0-65535>' parks DAC at a static value (handy for
         * checking the DAC -> amp -> speaker path with a multimeter). */
        const char *p_val = FreeRTOS_CLIGetParameter(pcCommandString, 2, &xParameterStringLength);
        uint16_t v = 32768u;
        if (p_val != NULL)
        {
            int n = atoi(p_val);
            if (n < 0)     n = 0;
            if (n > 65535) n = 65535;
            v = (uint16_t)n;
        }
        cyhal_dac_write(&g_audio_dac, v);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "audio: DAC P9.6 set to %u/65535\r\n", (unsigned)v);
        return pdFALSE;
    }

    if (strncmp(pcParameter, "dump", 4) == 0)
    {
        if (!g_audio_i2c_ready)
        {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "audio: I2C not initialized\r\n");
            return pdFALSE;
        }
        task_print_info("audio: dump regs 0x00-0x1F at TAS54414 (0x%02X)",
                        (unsigned)TAS5441_I2C_ADDR);
        for (uint8_t reg = 0u; reg <= 0x1Fu; reg++)
        {
            uint8_t val = 0u;
            cy_rslt_t r = audio_read_reg(TAS5441_I2C_ADDR, reg, &val);
            if (r == CY_RSLT_SUCCESS)
            {
                task_print_info("  reg[0x%02X] = 0x%02X", (unsigned)reg, (unsigned)val);
            }
            else
            {
                task_print_info("  reg[0x%02X] = ERR 0x%08lX", (unsigned)reg, (unsigned long)r);
            }
        }
        snprintf(pcWriteBuffer, xWriteBufferLen, "audio: dump complete\r\n");
        return pdFALSE;
    }

    snprintf(pcWriteBuffer, xWriteBufferLen,
             "audio: unknown subcommand '%s'\r\n", pcParameter);
    return pdFALSE;
}

static const CLI_Command_Definition_t xAudio =
{
    "audio",
    "\r\naudio < probe | status | dump | tone [hz] | dc [0-65535] | say <name> >\r\n",
    cli_handler_audio,
    -1
};

/* Refuse to call cyhal_i2c_init if SDA or SCL is stuck low — the HAL spins
 * forever waiting for the bus to release. Read each pin briefly as a
 * pull-up input first; if it doesn't go high, log and bail. */
static bool audio_bus_lines_idle_high(void)
{
    cy_rslt_t r;
    bool ok = true;

    r = cyhal_gpio_init(AUDIO_I2C_SCL_PIN, CYHAL_GPIO_DIR_INPUT,
                        CYHAL_GPIO_DRIVE_PULLUP, 1);
    if (r != CY_RSLT_SUCCESS)
    {
        return false;
    }
    r = cyhal_gpio_init(AUDIO_I2C_SDA_PIN, CYHAL_GPIO_DIR_INPUT,
                        CYHAL_GPIO_DRIVE_PULLUP, 1);
    if (r != CY_RSLT_SUCCESS)
    {
        cyhal_gpio_free(AUDIO_I2C_SCL_PIN);
        return false;
    }

    /* Give the lines a moment to settle on the internal pull-up. */
    for (volatile int i = 0; i < 1000; i++) { __asm__ volatile ("nop"); }

    if (cyhal_gpio_read(AUDIO_I2C_SCL_PIN) == 0u) ok = false;
    if (cyhal_gpio_read(AUDIO_I2C_SDA_PIN) == 0u) ok = false;

    cyhal_gpio_free(AUDIO_I2C_SCL_PIN);
    cyhal_gpio_free(AUDIO_I2C_SDA_PIN);
    return ok;
}

static void task_audio(void *param)
{
    (void)param;

    if (!audio_bus_lines_idle_high())
    {
        task_print_error("audio: SDA/SCL stuck low on P9.0/P9.1 — check pullups + amp power");
        vTaskDelete(NULL);
        return;
    }

    cy_rslt_t rslt = cyhal_i2c_init(&g_audio_i2c,
                                    AUDIO_I2C_SDA_PIN,
                                    AUDIO_I2C_SCL_PIN,
                                    NULL);
    if (rslt != CY_RSLT_SUCCESS)
    {
        task_print_error("audio: i2c_init failed (0x%08lX)", (unsigned long)rslt);
        vTaskDelete(NULL);
        return;
    }

    rslt = cyhal_i2c_configure(&g_audio_i2c, &g_audio_i2c_cfg);
    if (rslt != CY_RSLT_SUCCESS)
    {
        task_print_error("audio: i2c_configure failed (0x%08lX)", (unsigned long)rslt);
        cyhal_i2c_free(&g_audio_i2c);
        vTaskDelete(NULL);
        return;
    }

    g_audio_i2c_ready = true;
    task_print_info("audio: I2C up on P9.0/P9.1 @ %lu Hz, type 'audio probe'",
                    (unsigned long)AUDIO_I2C_FREQ_HZ);

    /* Bring up the CTDAC on P9.6 for amp input. Park at mid-rail so the
     * speaker doesn't pop on init. */
    cy_rslt_t dr = cyhal_dac_init(&g_audio_dac, AUDIO_DAC_PIN);
    if (dr != CY_RSLT_SUCCESS)
    {
        task_print_error("audio: dac_init failed (0x%08lX)", (unsigned long)dr);
        vTaskDelete(NULL);
        return;
    }
    cyhal_dac_write(&g_audio_dac, 32768u);
    g_audio_dac_ready = true;
    task_print_info("audio: DAC up on P9.6, parked mid-rail; type 'audio tone'");

    vTaskDelete(NULL);
}

bool task_audio_say(const char *clip_name)
{
    if (clip_name == NULL || !g_audio_dac_ready)
    {
        return false;
    }
    for (size_t k = 0u; k < AUDIO_CLIP_COUNT; k++)
    {
        if (strcmp(g_audio_clips[k].name, clip_name) == 0)
        {
            return audio_dispatch_clip(g_audio_clips[k].pcm,
                                       g_audio_clips[k].len,
                                       g_audio_clips[k].name) == CY_RSLT_SUCCESS;
        }
    }
    return false;
}

void task_audio_init(void)
{
    /* Register CLI immediately so `audio status` reports state even if the
     * deferred I2C init fails. */
    FreeRTOS_CLIRegisterCommand(&xAudio);

    BaseType_t ok = xTaskCreate(task_audio,
                                "Audio",
                                4 * configMINIMAL_STACK_SIZE,
                                NULL,
                                tskIDLE_PRIORITY + 1,
                                NULL);
    if (ok != pdPASS)
    {
        task_print_error("audio: task create failed");
    }
}
