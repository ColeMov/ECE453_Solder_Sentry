/**
 * @file ece453_pins.h
 * @author Joe Krachey (jkrachey@wisc.edu)
 * @brief 
 * @version 0.1
 * @date 2025-05-28
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __ECE453_PINS_H__
#define __ECE453_PINS_H__

/* Include Infineon BSP Libraries */
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"

/*****************************************************************************/
/* Module 0 */
/*****************************************************************************/
#define MOD_0_SCB_UART       SCB1
#define MOD_0_SCB_I2C        SCB1
#define MOD_0_SCB_SPI        SCB1

/* UART Pins */
#define MOD_0_PIN_UART_RX    P10_0
#define MOD_0_PIN_UART_TX    P10_1

/* I2C Pins */
#define MOD_0_PIN_I2C_SCL    P10_0
#define MOD_0_PIN_I2C_SDA    P10_1

/* SPI Pins */
#define MOD_0_PIN_SPI_MOSI   P10_0
#define MOD_0_PIN_SPI_MISO   P10_1
#define MOD_0_PIN_SPI_SCLK   P10_2
#define MOD_0_PIN_SPI_CS_N   P10_3

/* Generic IO Pins */
#define MOD_0_PIN_IO_0       P5_4
#define MOD_0_PIN_IO_1       P5_5

/* Analog Pins */
#define MOD_0_PIN_ADC_AIN    P10_4

/*****************************************************************************/

/*****************************************************************************/
/* Module 1 */
/*****************************************************************************/
#define MOD_1_SCB_UART       SCB2
#define MOD_1_SCB_I2C        SCB2
#define MOD_1_SCB_SPI        SCB2

/* UART Pins */
#define MOD_1_PIN_UART_RX    P9_0
#define MOD_1_PIN_UART_TX    P9_1

/* I2C Pins */
#define MOD_1_PIN_I2C_SCL    P9_0
#define MOD_1_PIN_I2C_SDA    P9_1

/* SPI Pins */
#define MOD_1_PIN_SPI_MOSI   P9_0
#define MOD_1_PIN_SPI_MISO   P9_1
#define MOD_1_PIN_SPI_SCLK   P9_2
#define MOD_1_PIN_SPI_CS_N   P9_3

/* Generic IO Pins */
#define MOD_1_PIN_IO_0       P9_4
#define MOD_1_PIN_IO_1       P9_5

/* Analog Pins */
#define MOD_1_PIN_ADC_AIN    P10_5
#define MOD_1_PIN_DAC        P9_6

/*****************************************************************************/

/*****************************************************************************/
/* Module 2 */
/*****************************************************************************/
#define MOD_2_SCB_UART       SCB6
#define MOD_2_SCB_I2C        SCB8

/* UART Pins */
#define MOD_2_PIN_UART_RX    P6_4
#define MOD_2_PIN_UART_TX    P6_5

/* I2C Pins */
#define MOD_2_PIN_I2C_SCL    P6_4
#define MOD_2_PIN_I2C_SDA    P6_5

/* Generic IO Pins */
#define MOD_2_PIN_IO_0       P12_6
#define MOD_2_PIN_IO_1       P12_7

/* Analog Pins */
#define MOD_2_PIN_ADC_AIN   P10_6

/*****************************************************************************/

/*****************************************************************************/
/* Module Site Enum */
/*****************************************************************************/
typedef enum
{
    MODULE_SITE_0 = 0,
    MODULE_SITE_1,
    MODULE_SITE_2,
    MODULE_SITE_INVALID
} module_site_t;


#endif
