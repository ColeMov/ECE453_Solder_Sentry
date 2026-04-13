/*
 * opt3001.h
 *
 *  Created on: Oct 20, 2020
 *      Author: Joe Krachey
 */

#ifndef __TASK_IO_EXPANDER_H__
#define __TASK_IO_EXPANDER_H__

#include "main.h"
#include "task_console.h"
#include "i2c.h"

#define TCA9534_SUBORDINATE_ADDR                 0x20

typedef enum 
{
    IOXP_OP_READ,
    IOXP_OP_WRITE,
    IOXP_OP_INVALID,
}io_expander_operation_t;

typedef enum 
{
    IOXP_ADDR_INPUT_PORT  = 0x00,
    IOXP_ADDR_OUTPUT_PORT = 0x01,
    IOXP_ADDR_POLARITY    = 0x02,
    IOXP_ADDR_CONFIG      = 0x03,
    IOXP_ADDR_INVALID     = 0xFF,
}io_expander_reg_addr_t;

typedef struct 
{
    io_expander_operation_t operation;
    io_expander_reg_addr_t  address;
    uint32_t value;
    QueueHandle_t return_queue;
} io_expander_packet_t;

extern QueueHandle_t q_io_expander_req;
extern QueueHandle_t q_io_expander_rsp;

void task_io_exp_init(void);


#endif
