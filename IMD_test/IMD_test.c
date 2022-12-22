#include <stdio.h>
#include "pico_hal.h"

extern void blink_led(uint32_t time_ms);

int64_t alarm_callback(alarm_id_t id, void *user_data) {
    // Put your timeout handler code in here
    return 0;
}



int main()
{
    // Microcontroller initialization
    micro_init();

    // Timer example code - This example fires off the callback after 2000ms
    //add_alarm_in_ms(2000, alarm_callback, NULL, false);

    blink_led(100);
    
    return 0;
}
