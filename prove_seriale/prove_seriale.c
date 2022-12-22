#include <stdio.h>
#include "pico/stdlib.h"
#include "string.h"

#define N_INBUF         255

int main()
{
    char in_buff[N_INBUF];

    stdio_init_all();
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    while (1) {
        // echo sulla seriale
        memset(in_buff, 0, N_INBUF);
        //scanf("%s", in_buff);
        for (int i=0; i<N_INBUF; i++){
            in_buff[i] = getchar();
            if (in_buff[i] == '\n')
                break;
        }

        gpio_put(PICO_DEFAULT_LED_PIN, 1);

        //printf("%s\n", in_buff);
        for (int i=0; i<N_INBUF; i++){
            putchar(in_buff[i]);
            if (in_buff[i] == '\0') {
                putchar('\n');
                break;
            }
                
        }

        sleep_ms(500);

        gpio_put(PICO_DEFAULT_LED_PIN, 0);
    }
    
    return 0;
}
