#include "task_tof.h"
#include <string.h>
#include "task_console.h"
#include "task_blink.h"
#include "i2c.h"
#include "ece453_pins.h"
#include "app_state.h"

#include "vl53lx_api.h"
#include "vl53lx_def.h"
#include "vl53lx_ll_device.h"

#define TOF_XSHUT_HOLD_MS          (5u)
#define TOF_I2C_SPEED_KHZ          (100u)
#define TOF_DISTANCE_THRESHOLD_MM  (200u)
#define TOF_TIMING_BUDGET_US       (50000u)
#define TOF_INTER_MEASUREMENT_MS   (60u)

static void task_tof(void *param);
static cy_rslt_t tof_gpio_init(void);
static bool tof_sensor_init(void);

static VL53LX_Dev_t tof_dev;

void task_tof_init(void)
{
    cy_rslt_t rslt;

    rslt = i2c_init(MODULE_SITE_0);
    if (rslt != CY_RSLT_SUCCESS)
    {
        task_print_error("ToF: I2C init failed (0x%08lx)", (unsigned long)rslt);
        return;
    }

    rslt = tof_gpio_init();
    if (rslt != CY_RSLT_SUCCESS)
    {
        task_print_error("ToF: GPIO init failed (0x%08lx)", (unsigned long)rslt);
        return;
    }

    if (!tof_sensor_init())
    {
        task_print_error("ToF: sensor init failed");
        return;
    }

    xTaskCreate(
        task_tof,
        "ToF",
        6 * configMINIMAL_STACK_SIZE,
        NULL,
        configMAX_PRIORITIES - 6,
        NULL);
}

static cy_rslt_t tof_gpio_init(void)
{
    cy_rslt_t rslt;

    // XSHUT (Module IO_0) controls sensor reset.
    rslt = cyhal_gpio_init(MOD_0_PIN_IO_0, CYHAL_GPIO_DIR_OUTPUT,
                           CYHAL_GPIO_DRIVE_STRONG, 0);
    if (rslt != CY_RSLT_SUCCESS)
    {
        return rslt;
    }

    // Hold in reset briefly, then release.
    cyhal_system_delay_ms(TOF_XSHUT_HOLD_MS);
    cyhal_gpio_write(MOD_0_PIN_IO_0, 1);
    cyhal_system_delay_ms(TOF_XSHUT_HOLD_MS);

    // GPIO1 (Module IO_1) is the sensor interrupt output.
    rslt = cyhal_gpio_init(MOD_0_PIN_IO_1, CYHAL_GPIO_DIR_INPUT,
                           CYHAL_GPIO_DRIVE_PULLDOWN, 0);
    if (rslt != CY_RSLT_SUCCESS)
    {
        return rslt;
    }

    // Initialize the status LED (active-low, start OFF).
    rslt = cyhal_gpio_init(PIN_LED, CYHAL_GPIO_DIR_OUTPUT,
                           CYHAL_GPIO_DRIVE_STRONG, 1);
    if (rslt != CY_RSLT_SUCCESS)
    {
        return rslt;
    }

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
    bool last_near = false;

    (void)param;

    while (1)
    {
        status = VL53LX_WaitMeasurementDataReady(&tof_dev);
        if (status != VL53LX_ERROR_NONE)
        {
            task_print_warning("ToF: wait data error %d", (int)status);
            vTaskDelay(pdMS_TO_TICKS(TOF_INTER_MEASUREMENT_MS));
            continue;
        }

        bool near = false;
        status = VL53LX_GetMultiRangingData(&tof_dev, &data);
        if (status == VL53LX_ERROR_NONE && data.NumberOfObjectsFound > 0)
        {
            int16_t distance_mm = data.RangeData[0].RangeMilliMeter;
            uint8_t range_status = data.RangeData[0].RangeStatus;
            near = (range_status == VL53LX_RANGESTATUS_RANGE_VALID) &&
                   (distance_mm > 0) &&
                   ((uint16_t)distance_mm <= TOF_DISTANCE_THRESHOLD_MM);
        }

        if (near && !last_near)
        {
            cyhal_gpio_write(PIN_LED, 0); /* active-low: 0 = ON */
            task_print_info("ToF: object detected (%d mm) — switching system OFF",
                            data.RangeData[0].RangeMilliMeter);
            app_state_set_active(false);
        }
        else if (!near && last_near)
        {
            cyhal_gpio_write(PIN_LED, 1); /* 1 = OFF */
            task_print_info("ToF: object gone");
        }

        last_near = near;

        status = VL53LX_ClearInterruptAndStartMeasurement(&tof_dev);
        if (status != VL53LX_ERROR_NONE)
        {
            task_print_warning("ToF: clear/start error %d", (int)status);
        }
    }
}
