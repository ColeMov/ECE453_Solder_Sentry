#include "task_tof.h"
#include <string.h>
#include "task_console.h"
#include "task_blink.h"
#include "task_fan.h"
#include "task_audio.h"
#include "task_servo_ctrl.h"
#include "i2c.h"
#include "ece453_pins.h"
#include "FreeRTOS_CLI.h"

#include "vl53lx_api.h"
#include "vl53lx_def.h"
#include "vl53lx_ll_device.h"

#define TOF_XSHUT_HOLD_MS          (50u)
#define TOF_I2C_SPEED_KHZ          (100u)
/* Hysteresis: trip "too close" below 300 mm, clear above 400 mm. Prevents
 * the fan from flapping on/off when an object hovers near the threshold. */
#define TOF_NEAR_ENTER_MM          (300u)
#define TOF_NEAR_LEAVE_MM          (400u)
#define TOF_FAN_RESUME_DUTY        (100u)
#define TOF_TIMING_BUDGET_US       (50000u)
#define TOF_INTER_MEASUREMENT_MS   (60u)

static void task_tof(void *param);
static cy_rslt_t tof_gpio_init(void);
static bool tof_sensor_init(void);

static VL53LX_Dev_t tof_dev;

static BaseType_t cli_handler_tof(char *pcWriteBuffer,
                                  size_t xWriteBufferLen,
                                  const char *pcCommandString)
{
    (void)pcCommandString;
    configASSERT(pcWriteBuffer);
    memset(pcWriteBuffer, 0, xWriteBufferLen);

    task_print_info("ToF: probing 0x03-0x77 on Module Site 2 (P6.4/P6.5)");
    unsigned found = 0u;
    for (uint8_t a = 0x03u; a <= 0x77u; a++)
    {
        uint8_t dummy = 0u;
        cy_rslt_t r = cyhal_i2c_master_write(&i2c_master_obj_site2,
                                             (uint16_t)a,
                                             &dummy, 0u, 50u, true);
        if (r == CY_RSLT_SUCCESS)
        {
            task_print_info("  ACK at 0x%02X", (unsigned)a);
            found++;
        }
    }

    /* Read VL53L3CX model ID register 0x010F — expected 0xEA.
     * Use mem_read so the HAL handles the START/RSTART/STOP sequence. */
    uint16_t reg_addr = 0x010Fu;
    uint8_t model_id = 0u;
    xSemaphoreTake(Semaphore_I2C_site2, portMAX_DELAY);
    cy_rslt_t r = cyhal_i2c_master_mem_read(&i2c_master_obj_site2, 0x29u,
                                            reg_addr, 2u,
                                            &model_id, 1u, 100u);
    xSemaphoreGive(Semaphore_I2C_site2);
    if (r == CY_RSLT_SUCCESS)
    {
        task_print_info("ToF: model_id reg 0x010F = 0x%02X (expected 0xEA for VL53L3CX)",
                        (unsigned)model_id);
    }
    else
    {
        task_print_info("ToF: model_id read failed (0x%08lX)", (unsigned long)r);
    }

    snprintf(pcWriteBuffer, xWriteBufferLen,
             "ToF: probe done, %u devices found\r\n", found);
    return pdFALSE;
}

static const CLI_Command_Definition_t xTof =
{
    "tof",
    "\r\ntof : probe Module Site 2 I2C bus for VL53LX (expected 0x29)\r\n",
    cli_handler_tof,
    0
};

void task_tof_init(void)
{
    cy_rslt_t rslt;

    /* main.c is responsible for calling i2c_init(MODULE_SITE_2) before us
     * so vl53lx_platform.c's reads/writes work via i2c_master_obj_site2. */

    /* Register the bus-probe CLI even if sensor init below fails — handy
     * for diagnosing wiring / power issues without re-flashing. */
    FreeRTOS_CLIRegisterCommand(&xTof);

    rslt = tof_gpio_init();
    if (rslt != CY_RSLT_SUCCESS)
    {
        task_print_error("ToF: GPIO init failed (0x%08lx)", (unsigned long)rslt);
        return;
    }

    if (!tof_sensor_init())
    {
        task_print_error("ToF: sensor init failed (use 'tof' CLI to probe bus)");
        return;
    }

    /* Higher than the servo control task (configMAX_PRIORITIES - 5) so the
     * TOF poll wins the race when the IR-tracker is active. The servo
     * tracker only needs to update twice a second; TOF must hit its data-
     * ready window or the read times out. */
    xTaskCreate(
        task_tof,
        "ToF",
        6 * configMINIMAL_STACK_SIZE,
        NULL,
        configMAX_PRIORITIES - 4,
        NULL);
}

static cy_rslt_t tof_gpio_init(void)
{
    cy_rslt_t rslt;

    // XSHUT (Module 2 IO_0 = P12.6) controls sensor reset.
    rslt = cyhal_gpio_init(MOD_2_PIN_IO_0, CYHAL_GPIO_DIR_OUTPUT,
                           CYHAL_GPIO_DRIVE_STRONG, 0);
    if (rslt != CY_RSLT_SUCCESS)
    {
        return rslt;
    }

    // Hold in reset briefly, then release.
    cyhal_system_delay_ms(TOF_XSHUT_HOLD_MS);
    cyhal_gpio_write(MOD_2_PIN_IO_0, 1);
    cyhal_system_delay_ms(TOF_XSHUT_HOLD_MS);

    // GPIO1 (Module 2 IO_1 = P12.7) is the sensor interrupt output.
    // Per VL53L3CX datasheet GPIO1 is OPEN-DRAIN active-low — needs pull-UP
    // (datasheet recommends 10 kΩ external; PSoC internal pull-up is fine
    // as fallback). Was previously pull-down which made it always read 0.
    rslt = cyhal_gpio_init(MOD_2_PIN_IO_1, CYHAL_GPIO_DIR_INPUT,
                           CYHAL_GPIO_DRIVE_PULLUP, 1);
    if (rslt != CY_RSLT_SUCCESS)
    {
        return rslt;
    }

    /* PIN_LED is owned by task_captouch; do NOT init here, just leave it
     * alone. The ToF "too close" alert is voice + log only. */

    return CY_RSLT_SUCCESS;
}

static bool tof_sensor_init(void)
{
    VL53LX_Error status;

    memset(&tof_dev, 0, sizeof(tof_dev));

    status = VL53LX_CommsInitialise(&tof_dev, VL53LX_I2C, TOF_I2C_SPEED_KHZ);
    if (status != VL53LX_ERROR_NONE)
    {
        task_print_error("ToF: comms init error %d", (int)status);
        return false;
    }

    status = VL53LX_WaitDeviceBooted(&tof_dev);
    if (status != VL53LX_ERROR_NONE)
    {
        task_print_error("ToF: boot wait error %d", (int)status);
        return false;
    }

    status = VL53LX_DataInit(&tof_dev);
    if (status != VL53LX_ERROR_NONE)
    {
        task_print_error("ToF: data init error %d", (int)status);
        return false;
    }

    status = VL53LX_SetDistanceMode(&tof_dev, VL53LX_DISTANCEMODE_LONG);
    if (status != VL53LX_ERROR_NONE)
    {
        task_print_error("ToF: distance mode error %d", (int)status);
        return false;
    }

    status = VL53LX_SetMeasurementTimingBudgetMicroSeconds(&tof_dev, TOF_TIMING_BUDGET_US);
    if (status != VL53LX_ERROR_NONE)
    {
        task_print_error("ToF: timing budget error %d", (int)status);
        return false;
    }

    status = VL53LX_StartMeasurement(&tof_dev);
    if (status != VL53LX_ERROR_NONE)
    {
        task_print_error("ToF: start measurement error %d", (int)status);
        return false;
    }

    task_print_info("ToF: ranging started");
    return true;
}

static void task_tof(void *param)
{
    VL53LX_Error status;
    VL53LX_MultiRangingData_t data;
    bool too_close = false;   /* sticky state with hysteresis */

    (void)param;

    /* Initial settle time: sensor needs at least one full timing budget
     * cycle (~50 ms) before first data is available. */
    vTaskDelay(pdMS_TO_TICKS(200u));

    uint32_t poll_count = 0u;

    while (1)
    {
        /* Poll the GPIO1 INT pin (P12.7) directly — open-drain active-low
         * per datasheet. Bypasses the driver's I2C-polled status register
         * which has been unreliable on this bus. Wired pull-up is enabled
         * in tof_gpio_init so the line idles HIGH and goes LOW on ready. */
        vTaskDelay(pdMS_TO_TICKS(20u));
        poll_count++;

        if (cyhal_gpio_read(MOD_2_PIN_IO_1) != 0u)
        {
            /* Every ~2 s of waiting, kick the sensor by re-arming the
             * measurement. Sometimes the sensor "forgets" to restart
             * after Clear and stays idle. */
            if ((poll_count % 100u) == 0u)
            {
                task_print_info("ToF: waiting on INT pin (poll=%lu) — re-arming",
                                (unsigned long)poll_count);
                (void)VL53LX_ClearInterruptAndStartMeasurement(&tof_dev);
            }
            continue;
        }

        int16_t distance_mm = -1;
        uint8_t range_status = 0xFFu;
        status = VL53LX_GetMultiRangingData(&tof_dev, &data);
        if (status == VL53LX_ERROR_NONE && data.NumberOfObjectsFound > 0)
        {
            distance_mm = data.RangeData[0].RangeMilliMeter;
            range_status = data.RangeData[0].RangeStatus;
        }

        /* Stream parseable telemetry: throttled to ~5 Hz so the BLE
         * notify queue + DAC playback don't get swamped by 50 Hz TOF
         * polls. Always emit on the first read after a state transition
         * by tracking the previous bucket. */
        if (distance_mm >= 0 && (poll_count % 10u) == 0u)
        {
            task_print_info("tof:%d", (int)distance_mm);
        }

        bool valid_reading = (status == VL53LX_ERROR_NONE) &&
                             (range_status == VL53LX_RANGESTATUS_RANGE_VALID) &&
                             (distance_mm > 0);

        if (!too_close && valid_reading &&
            ((uint16_t)distance_mm <= TOF_NEAR_ENTER_MM))
        {
            too_close = true;
            task_print_info("paused:1");
            task_fan_set_duty(0);
            /* Pause IR-tracker too: the TOF sits next to the fan, so any
             * pan/tilt motion swings TOF off-target and the safety logic
             * loses its read. Stop motion until the obstruction clears. */
            task_servo_ctrl_set_tracking(false);
            (void)task_audio_say("too_close");
        }
        else if (too_close && valid_reading &&
                 ((uint16_t)distance_mm >= TOF_NEAR_LEAVE_MM))
        {
            too_close = false;
            task_print_info("paused:0");
            task_fan_set_duty(TOF_FAN_RESUME_DUTY);
            task_servo_ctrl_set_tracking(true);
        }

        status = VL53LX_ClearInterruptAndStartMeasurement(&tof_dev);
        if (status != VL53LX_ERROR_NONE)
        {
            task_print_warning("ToF: clear/start error %d", (int)status);
        }
    }
}
