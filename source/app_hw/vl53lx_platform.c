/**
 * @file vl53lx_platform.c
 * @brief Platform implementation for VL53LX API on PSoC 6.
 * @version 0.1
 * @date 2026-02-09
 */

#include <string.h>
#include "main.h"
#include "i2c.h"
#include "ece453_pins.h"

#include "vl53lx_platform.h"
#include "vl53lx_error_codes.h"
#include <string.h>

#define VL53LX_I2C_ADDR_7BIT   (0x29u)

static VL53LX_Error vl53lx_write(uint16_t index, const uint8_t *pdata, uint32_t count)
{
    cy_rslt_t rslt;
    uint8_t buffer[2 + 256];
    uint32_t total = 2 + count;

    if (count > 256u)
    {
        return VL53LX_ERROR_INVALID_PARAMS;
    }

    buffer[0] = (uint8_t)((index >> 8) & 0xFF);
    buffer[1] = (uint8_t)(index & 0xFF);
    if (count > 0)
    {
        memcpy(&buffer[2], pdata, count);
    }

    xSemaphoreTake(Semaphore_I2C, portMAX_DELAY);
    rslt = cyhal_i2c_master_write(
        &i2c_master_obj,
        VL53LX_I2C_ADDR_7BIT,
        buffer,
        (uint16_t)total,
        0,
        true);
    xSemaphoreGive(Semaphore_I2C);

    return (rslt == CY_RSLT_SUCCESS) ? VL53LX_ERROR_NONE : VL53LX_ERROR_CONTROL_INTERFACE;
}

static VL53LX_Error vl53lx_read(uint16_t index, uint8_t *pdata, uint32_t count)
{
    cy_rslt_t rslt;
    uint8_t index_buf[2];

    index_buf[0] = (uint8_t)((index >> 8) & 0xFF);
    index_buf[1] = (uint8_t)(index & 0xFF);

    xSemaphoreTake(Semaphore_I2C, portMAX_DELAY);
    rslt = cyhal_i2c_master_write(
        &i2c_master_obj,
        VL53LX_I2C_ADDR_7BIT,
        index_buf,
        2,
        0,
        false);

    if (rslt == CY_RSLT_SUCCESS)
    {
        rslt = cyhal_i2c_master_read(
            &i2c_master_obj,
            VL53LX_I2C_ADDR_7BIT,
            pdata,
            (uint16_t)count,
            0,
            true);
    }
    xSemaphoreGive(Semaphore_I2C);

    return (rslt == CY_RSLT_SUCCESS) ? VL53LX_ERROR_NONE : VL53LX_ERROR_CONTROL_INTERFACE;
}

VL53LX_Error VL53LX_CommsInitialise(VL53LX_Dev_t *pdev, uint8_t comms_type, uint16_t comms_speed_khz)
{
    (void)pdev;
    (void)comms_type;
    (void)comms_speed_khz;
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_CommsClose(VL53LX_Dev_t *pdev)
{
    (void)pdev;
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WriteMulti(VL53LX_Dev_t *pdev, uint16_t index, uint8_t *pdata, uint32_t count)
{
    (void)pdev;
    return vl53lx_write(index, pdata, count);
}

VL53LX_Error VL53LX_ReadMulti(VL53LX_Dev_t *pdev, uint16_t index, uint8_t *pdata, uint32_t count)
{
    (void)pdev;
    return vl53lx_read(index, pdata, count);
}

VL53LX_Error VL53LX_WrByte(VL53LX_Dev_t *pdev, uint16_t index, uint8_t data)
{
    (void)pdev;
    return vl53lx_write(index, &data, 1);
}

VL53LX_Error VL53LX_WrWord(VL53LX_Dev_t *pdev, uint16_t index, uint16_t data)
{
    (void)pdev;
    uint8_t buffer[2];
    buffer[0] = (uint8_t)((data >> 8) & 0xFF);
    buffer[1] = (uint8_t)(data & 0xFF);
    return vl53lx_write(index, buffer, 2);
}

VL53LX_Error VL53LX_WrDWord(VL53LX_Dev_t *pdev, uint16_t index, uint32_t data)
{
    (void)pdev;
    uint8_t buffer[4];
    buffer[0] = (uint8_t)((data >> 24) & 0xFF);
    buffer[1] = (uint8_t)((data >> 16) & 0xFF);
    buffer[2] = (uint8_t)((data >> 8) & 0xFF);
    buffer[3] = (uint8_t)(data & 0xFF);
    return vl53lx_write(index, buffer, 4);
}

VL53LX_Error VL53LX_RdByte(VL53LX_Dev_t *pdev, uint16_t index, uint8_t *pdata)
{
    (void)pdev;
    return vl53lx_read(index, pdata, 1);
}

VL53LX_Error VL53LX_RdWord(VL53LX_Dev_t *pdev, uint16_t index, uint16_t *pdata)
{
    uint8_t buffer[2];
    VL53LX_Error status;
    (void)pdev;
    status = vl53lx_read(index, buffer, 2);
    if (status == VL53LX_ERROR_NONE)
    {
        *pdata = (uint16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);
    }
    return status;
}

VL53LX_Error VL53LX_RdDWord(VL53LX_Dev_t *pdev, uint16_t index, uint32_t *pdata)
{
    uint8_t buffer[4];
    VL53LX_Error status;
    (void)pdev;
    status = vl53lx_read(index, buffer, 4);
    if (status == VL53LX_ERROR_NONE)
    {
        *pdata = ((uint32_t)buffer[0] << 24) |
                 ((uint32_t)buffer[1] << 16) |
                 ((uint32_t)buffer[2] << 8) |
                 (uint32_t)buffer[3];
    }
    return status;
}

VL53LX_Error VL53LX_WaitUs(VL53LX_Dev_t *pdev, int32_t wait_us)
{
    (void)pdev;
    if (wait_us > 0)
    {
        cyhal_system_delay_us((uint32_t)wait_us);
    }
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WaitMs(VL53LX_Dev_t *pdev, int32_t wait_ms)
{
    (void)pdev;
    if (wait_ms > 0)
    {
        cyhal_system_delay_ms((uint32_t)wait_ms);
    }
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GetTimerFrequency(int32_t *ptimer_freq_hz)
{
    if (ptimer_freq_hz == NULL)
    {
        return VL53LX_ERROR_INVALID_PARAMS;
    }
    *ptimer_freq_hz = (int32_t)configTICK_RATE_HZ;
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GetTimerValue(int32_t *ptimer_count)
{
    if (ptimer_count == NULL)
    {
        return VL53LX_ERROR_INVALID_PARAMS;
    }
    *ptimer_count = (int32_t)xTaskGetTickCount();
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GpioSetMode(uint8_t pin, uint8_t mode)
{
    (void)pin;
    (void)mode;
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GpioSetValue(uint8_t pin, uint8_t value)
{
    (void)pin;
    cyhal_gpio_write(MOD_2_PIN_IO_0, value ? 1 : 0);
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GpioGetValue(uint8_t pin, uint8_t *pvalue)
{
    (void)pin;
    if (pvalue == NULL)
    {
        return VL53LX_ERROR_INVALID_PARAMS;
    }
    *pvalue = cyhal_gpio_read(MOD_2_PIN_IO_1) ? 1 : 0;
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GpioXshutdown(uint8_t value)
{
    cyhal_gpio_write(MOD_2_PIN_IO_0, value ? 1 : 0);
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GpioCommsSelect(uint8_t value)
{
    (void)value;
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GpioPowerEnable(uint8_t value)
{
    (void)value;
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GpioInterruptEnable(void (*function)(void), uint8_t edge_type)
{
    (void)function;
    (void)edge_type;
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GpioInterruptDisable(void)
{
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GetTickCount(VL53LX_Dev_t *pdev, uint32_t *ptime_ms)
{
    (void)pdev;
    if (ptime_ms == NULL)
    {
        return VL53LX_ERROR_INVALID_PARAMS;
    }
    *ptime_ms = (uint32_t)(xTaskGetTickCount() * (1000u / configTICK_RATE_HZ));
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WaitValueMaskEx(VL53LX_Dev_t *pdev, uint32_t timeout_ms,
                                    uint16_t index, uint8_t value, uint8_t mask,
                                    uint32_t poll_delay_ms)
{
    uint8_t read_val = 0;
    uint32_t elapsed = 0;

    (void)pdev;

    while (elapsed <= timeout_ms)
    {
        if (vl53lx_read(index, &read_val, 1) == VL53LX_ERROR_NONE)
        {
            if ((read_val & mask) == value)
            {
                return VL53LX_ERROR_NONE;
            }
        }

        if (poll_delay_ms == 0)
        {
            poll_delay_ms = 1;
        }
        cyhal_system_delay_ms(poll_delay_ms);
        elapsed += poll_delay_ms;
    }

    return VL53LX_ERROR_TIME_OUT;
}
