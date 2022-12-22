/**
 * @file aht20_lib.h
 * @author Simone Ricciardi
 * @brief Header of file containing functions for handling humidithy/temperature sensor AHT20
 * @version 0.1
 * @date 2022-07-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _AHT_LIB_H
#define _AHT_LIB_H

/* Includes --------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

/* Defines ---------------------------------------------------------------*/
#define STOP_BIT                        false
#define NO_STOP_BIT                     true

// Errors
#define ACK_ERROR                       -1
#define NOT_INIT_ERR                    -2

#define AHT20_SOFT_RST_CMD_push(x)      { x[0]=0xBA; x[1]=0x00; x[2]=0x00; }
#define AHT20_INIT_CMD_push(x)          { x[0]=0xBE; x[1]=0x08; x[2]=0x00; }
#define AHT20_TRIG_CMD_push(x)          { x[0]=0xAC; x[1]=0x33; x[2]=0x00; }
#define AHT20_SLAVE_ADDR                0x38
#define BUSY_BIT                        ( 1 << 7 )

#define _is_busy(x)                     ( x & BUSY_BIT )


#define I2C_TIMEOUT                     20 * 1000   // 10 ms

/* Typedefs --------------------------------------------------------------*/
typedef struct humidity_sensor {
    char sensor_model[30];
    i2c_inst_t *i2c_;
    uint8_t slave_addr;
    uint8_t status_word;
    bool initialized;
    float humidity;
    float temperature;
}   humidity_sensor_t;


/* Exported functions ---------------------------------------------------*/
extern int AHT20_init ( humidity_sensor_t *sensor_instance, i2c_inst_t *i2c_port );
extern int aht20_read_trig ( humidity_sensor_t *sensor_instance );

#endif