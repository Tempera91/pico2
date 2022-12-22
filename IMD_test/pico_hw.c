/**
 * @file pico_hw.c
 * @author Simone Ricciardi
 * @brief Contain functions for Pi Pico hardware management
 * @version 0.1
 * @date 2022-05-03
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "pico_hal.h"   // Contain all Pico definitions

/**
 * @brief Pico initialization
 * 
 */
void pico_init() {
    stdio_init_all();

    // SPI initialisation. This example will use SPI at 1MHz.
    spi_init(SPI_PORT, 1000*1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
    

    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400*1000);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
}

/**
 * @brief Alias for micro initialization
 * 
 */
void micro_init() __attribute__((alias("pico_init")));

/**
 * @brief 
 * 
 * @param addr 
 * @param dst 
 * @param len 
 * @return int 
 */
int i2c_read(uint8_t addr, uint8_t *dst, size_t len) {
    return i2c_read_blocking(I2C_PORT, addr, dst, len, false);
}
