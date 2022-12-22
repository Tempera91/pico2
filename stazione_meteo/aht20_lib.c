/**
 * @file aht20_lib.c
 * @author Simone Ricciardi
 * @brief Contain functions and definitions for handling AHT20 humidity/temperature sensor
 * @version 0.1
 * @date 2022-07-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "aht20_lib.h"

/**
 * @brief AHT20 humidity sensor initialize and handling structure population
 * 
 * @param sensor_instance pointer to sensor handling structure
 * @param i2c_port i2c port instance
 * @return 0 correctly initialized, ACK_ERROR instead
 */
int AHT20_init(humidity_sensor_t *sensor_instance, i2c_inst_t *i2c_port) {
    uint8_t cmd[3];
    sensor_instance->i2c_ = i2c_port;
    strcpy(sensor_instance->sensor_model, "ATH20");
    sensor_instance->slave_addr = AHT20_SLAVE_ADDR;
    sensor_instance->initialized = false;

    /* Soft reset                                                   */
    AHT20_SOFT_RST_CMD_push(cmd)
    if (i2c_write_blocking(i2c_port, sensor_instance->slave_addr, 
                        cmd, 1, STOP_BIT) == PICO_ERROR_GENERIC ) 
        return ACK_ERROR;
    do {
        sleep_ms(10);
        if (i2c_read_blocking(i2c_port, sensor_instance->slave_addr, 
                        &sensor_instance->status_word, 1, STOP_BIT) 
                                            == PICO_ERROR_GENERIC )
            return ACK_ERROR;
    } while (sensor_instance->status_word != 0x18);

    /* Initialization                                               */
    AHT20_INIT_CMD_push(cmd)
    if (i2c_write_blocking(i2c_port, sensor_instance->slave_addr, 
                        cmd, 3, STOP_BIT) == PICO_ERROR_GENERIC ) 
        return ACK_ERROR;
    do {
        sleep_ms(10);
        if (i2c_read_blocking(i2c_port, sensor_instance->slave_addr, 
                        &sensor_instance->status_word, 1, STOP_BIT) 
                                            == PICO_ERROR_GENERIC )
            return ACK_ERROR;
    } while (sensor_instance->status_word != 0x18);
    sleep_ms(100);
    sensor_instance->initialized = true;
    return 0;
}

/**
 * @fn int aht20_read_trig( humidity_sensor_t *sensor_instance )
 * @brief Trig a reading of humidity and temperature data
 * 
 * @param sensor_instance pointer to sensor handling structure
 * @return int 
 */
int aht20_read_trig( humidity_sensor_t *sensor_instance ) {
    uint8_t cmd[3];
    uint8_t rcvd_data[7];
    float humidity, temperature;
    uint32_t h, t;

    /* Control if sensor is previous initialized */
    if ( !sensor_instance->initialized )
        return NOT_INIT_ERR;
    /* Trigger measurement */
    AHT20_TRIG_CMD_push(cmd)
    if (i2c_write_timeout_us(sensor_instance->i2c_, sensor_instance->slave_addr, 
                                        cmd, 3, STOP_BIT, I2C_TIMEOUT) == PICO_ERROR_GENERIC)
        return ACK_ERROR;
    do {
        sleep_ms(10);
        if (i2c_read_timeout_us(sensor_instance->i2c_, sensor_instance->slave_addr, 
                                rcvd_data, 1, STOP_BIT, I2C_TIMEOUT) == PICO_ERROR_GENERIC)
            return ACK_ERROR;
    } while ( _is_busy(rcvd_data[0]) );

    if (i2c_read_timeout_us(sensor_instance->i2c_, sensor_instance->slave_addr, 
                                    rcvd_data, 6, STOP_BIT, I2C_TIMEOUT) == PICO_ERROR_GENERIC)
        return ACK_ERROR;

    h = rcvd_data[1];
    h <<= 8;
    h |= rcvd_data[2];
    h <<= 4;
    h |= rcvd_data[3] >> 4;
    sensor_instance->humidity = ((float)h * 100) / 0x100000;

    t = rcvd_data[3] & 0x0F;
    t <<= 8;
    t |= rcvd_data[4];
    t <<= 8;
    t |= rcvd_data[5];
    sensor_instance->temperature = ((float)t * 200 / 0x100000) - 50;

    return 0;
}