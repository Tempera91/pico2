/**
 * @file aux_test.c
 * @author Simone Ricciardi
 * @brief Auxiliary functions for testing and developing
 * @version 0.1
 * @date 2022-05-03
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "pico_hal.h"


/**
 * @brief Blink the pico on-board default led.
 *          It's blocking function
 * 
 * @param time_ms blinking time in ms
 */
void blink_led(uint32_t time_ms) {

    // Pico built-in led GPIO configuration
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    while (1) {
        gpio_put(PICO_DEFAULT_LED_PIN,1);
        sleep_ms(time_ms);
        gpio_put(PICO_DEFAULT_LED_PIN,0);
        sleep_ms(time_ms);
    }
    
}