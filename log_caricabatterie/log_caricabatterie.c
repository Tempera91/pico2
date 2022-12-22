#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

#define AVG_N_SAMPLE            32
#define SAMPLE_TIME_MS          5000

#define LEDG0                   3
#define LEDR0                   5
#define CHG_INPUT               0
#define PPR_INPUT               1

#define V_REF                   3.237f
#define R1                      33.98f
#define R2                      99.8f

int64_t alarm_callback(alarm_id_t id, void *user_data) {
    // Put your timeout handler code in here
    return 0;
}



int main()
{
    stdio_init_all();

    // Timer example code - This example fires off the callback after 2000ms
    //add_alarm_in_ms(2000, alarm_callback, NULL, false);
    adc_init();
    adc_gpio_init(26);
    
    adc_gpio_init(27);

    gpio_init(LEDG0);
    gpio_init(LEDR0);
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_init(CHG_INPUT);
    gpio_init(PPR_INPUT);
    gpio_set_dir(LEDG0, GPIO_OUT);
    gpio_set_dir(LEDR0, GPIO_OUT);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_set_dir(CHG_INPUT, GPIO_IN);
    gpio_pull_up(CHG_INPUT);
    gpio_set_dir(PPR_INPUT, GPIO_IN);
    gpio_pull_up(PPR_INPUT);


    //puts("Hello, world!");
    long int i = 0;
    const float k_part = R2 / (R1 + R2);
    const float conv_factor = V_REF / (1<<12);    //conversion factor
    const float conv_fac_vbat = (V_REF / (1<<12)) / k_part;    //conversion factor

    uint16_t result=0;
    uint32_t raw_result=0;
    uint16_t res_vbat=0;
    uint32_t raw_res_vbat=0;
    long int time=0;
    float V, Vbat;

    absolute_time_t timestamp;

    while (gpio_get(PPR_INPUT));
    gpio_put(LEDG0,1);
    printf("time[s] raw_val Vi  Vbat    CHG Pin\n");

    while (1) {
        if (!gpio_get(PPR_INPUT)) {
            gpio_put(PICO_DEFAULT_LED_PIN, !gpio_get(PICO_DEFAULT_LED_PIN));
            gpio_put(LEDR0, 0);
        }
        else {
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            gpio_put(LEDR0, 1);
        }
        
        timestamp = make_timeout_time_ms(SAMPLE_TIME_MS);
    
        // start sampling timer
        if (!gpio_get(PPR_INPUT)) {  
            
            raw_result = 0;
            raw_res_vbat = 0;
            for (int i=0; i<AVG_N_SAMPLE; i++) {
                adc_select_input(0);
                raw_result = raw_result + adc_read();
                adc_select_input(1);
                raw_res_vbat = raw_res_vbat + adc_read();
                sleep_ms(10);
            }
            result = raw_result / AVG_N_SAMPLE;
            res_vbat = raw_res_vbat / AVG_N_SAMPLE;
            V = result * conv_factor;
            Vbat = res_vbat * conv_fac_vbat;
            printf ("%d\t%u\t%.4f\t%.4f", time, result, V, Vbat);
            if (gpio_get(CHG_INPUT)) {
                printf("\t%d\n",1);
            }
            else {
                printf("\t%d\n",0);
            }
        }

        //wait sampling time
        while (!time_reached(timestamp));
        time = time + SAMPLE_TIME_MS / 1000;
    }

    return 0;
}
