/**
 * @file task_captouch.c
 * @brief IQS228B capacitive touch sensor - prints when copper pad is touched
 *
 * Based on IQS228B/D datasheet (Azoteq ProxSense):
 * - Single-channel self-capacitive sensor with DYCAL
 * - TOUT pin: touch output (connects to copper pad via Cx)
 * - Touch detected when Counts (CS) diverge from LTA by touch threshold
 *
 * This task polls the TOUT GPIO and prints a message on touch/release.
 */
#include "task_captouch.h"
#include "task_console.h"
#include "task_blink.h"
#include "task_ble.h"
#include "ece453_pins.h"

#include "cyhal_gpio.h"

#define CAPTOUCH_POLL_MS         (50u)
#define CAPTOUCH_DEBOUNCE_COUNT  (2u)  /* Consecutive same readings before accepting */
#define CAPTOUCH_RAW_LOG_MS      (2000u)

static void task_captouch(void *param);

void task_captouch_init(void)
{
    cy_rslt_t rslt;

    /* IQS228 commonly uses Active-Low open-drain output; use pull-up in that mode. */
#if IQS228B_ACTIVE_HIGH
    rslt = cyhal_gpio_init(IQS228B_TOUT_PIN, CYHAL_GPIO_DIR_INPUT,
                          CYHAL_GPIO_DRIVE_PULLDOWN, 0);
#else
    rslt = cyhal_gpio_init(IQS228B_TOUT_PIN, CYHAL_GPIO_DIR_INPUT,
                          CYHAL_GPIO_DRIVE_PULLUP, 1);
#endif
    if (rslt != CY_RSLT_SUCCESS)
    {
        task_print_error("CapTouch: GPIO init failed (0x%08lx)", (unsigned long)rslt);
        return;
    }

    /* On-board LED is active-low: 1 = OFF, 0 = ON. */
    rslt = cyhal_gpio_init(PIN_LED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 1);
    if (rslt != CY_RSLT_SUCCESS)
    {
        task_print_error("CapTouch: LED init failed (0x%08lx)", (unsigned long)rslt);
        return;
    }

    xTaskCreate(
        task_captouch,
        "CapTouch",
        4 * configMINIMAL_STACK_SIZE,
        NULL,
        configMAX_PRIORITIES - 7,
        NULL);

    task_print_info("CapTouch: IQS228B touch detection started (TOUT -> P9_0)");
    task_print_info("CapTouch: mode=%s", IQS228B_ACTIVE_HIGH ? "Active-High" : "Active-Low");
}

static void task_captouch(void *param)
{
    bool last_sample_touched = false;
    bool stable_touched = false;
    uint32_t stable_count = 0;
    uint32_t raw_log_ticks = 0;

    (void)param;

    while (1)
    {
        uint32_t level = cyhal_gpio_read(IQS228B_TOUT_PIN);
        raw_log_ticks += CAPTOUCH_POLL_MS;

#if IQS228B_ACTIVE_HIGH
        bool touched = (level != 0);
#else
        bool touched = (level == 0);
#endif
        if (raw_log_ticks >= CAPTOUCH_RAW_LOG_MS)
        {
            task_print_info("CapTouch[BLEDBG][BLEC=%u][BLES=%lu]: raw TOUT=%lu | touched_if_active_high=%u touched_if_active_low=%u",
#ifdef COMPONENT_BLESS
                            1u,
#else
                            0u,
#endif
                            (unsigned long)g_ble_diag_state,
                            (unsigned long)level,
                            (level != 0) ? 1u : 0u,
                            (level == 0) ? 1u : 0u);
            raw_log_ticks = 0;
        }

        /* Debounce by requiring N consecutive identical samples before accepting a state change. */
        if (touched != last_sample_touched)
        {
            last_sample_touched = touched;
            stable_count = 1;
        }
        else
        {
            if (stable_count < CAPTOUCH_DEBOUNCE_COUNT)
            {
                stable_count++;
            }
        }

        if ((stable_count >= CAPTOUCH_DEBOUNCE_COUNT) && (stable_touched != touched))
        {
            stable_touched = touched;
            if (stable_touched)
            {
                /* active-low LED: 0 = ON */
                cyhal_gpio_write(PIN_LED, 0u);
                task_print_info("CapTouch: Copper pad TOUCHED! LED ON");
            }
            else
            {
                /* active-low LED: 1 = OFF */
                cyhal_gpio_write(PIN_LED, 1u);
                task_print_info("CapTouch: Copper pad released");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(CAPTOUCH_POLL_MS));
    }
}
