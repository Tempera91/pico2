#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "pico/runtime.h"

#include "general_inc.h"



int64_t alarm_callback(alarm_id_t id, void *user_data) {
    // Put your timeout handler code in here
    return 0;
}

void uart_send(char* src) {
    gpio_put(UART0_REDE_PIN, 1);
    uart_puts(UART_PORT, src);
    //uart_set_break(UART_PORT, false);
    uart_tx_wait_blocking(UART_PORT);
    //uart_set_break(UART_PORT, true);
    gpio_put(UART0_REDE_PIN, 0);
}

int main()
{
    /* Variabili ausiliarie */
    char inchar;
    char inbuf[MAX_BUF_LEN];
    int inint;
    int i;
    uint8_t inuint8;
    bool esc;

    /*              Inizializzazioni                */
    stdio_init_all();

    /* Inizializzazione GPIO                        */
    gpio_init(UART0_REDE_PIN);
    gpio_set_dir(UART0_REDE_PIN, GPIO_OUT);
    gpio_put(UART0_REDE_PIN, 0);

    /*  I2C Initialisation. Using it at 400Khz.     */
    i2c_init(I2C_PORT, 400*1000); 
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    /*  Inizializzazione UART0 su GPIO0, GPIO1      */
    uart_init(UART_PORT, UART_BAUDRATE);
    gpio_set_function(UART0_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART0_RX_PIN, GPIO_FUNC_UART);
    //uart_set_break(UART_PORT, true);
    uart_set_hw_flow(UART_PORT, false, false);
    uart_set_format(UART_PORT, 8, 1, UART_PARITY_NONE);

    /*  Ciclo principale di comando                 */
    //scanf("%1s",inbuf);
    printf("* * * * * * * * * * * * Raspberry Pi-Pico * * * * * * * * * * * *\n");
    printf("*       Prove trasmissione / ricezione RS485 via UART           *\n");
    printf("* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *\n");

    do {
        esc = true;
        inint = 0;
        memset(inbuf, 0, MAX_BUF_LEN);

        printf(">> Inserire un comando:\n");
        printf(">> 1: trasmissione continua; 2: trasmetti caratteri; 3: ricevi caratteri\n");
        printf(">> 10: esci\n>");

        scanf("%3s", inbuf);
        inint = atoi(inbuf);
        switch (inint) {
            case 1:
                printf("******** Trasmissione Continua *********\n");
                break;
            case 2:
                printf("********* Trasmetti caratteri **********\n");
                memset(inbuf, 0, MAX_BUF_LEN);
                printf(">> Inserisci stringa:\n>");
                scanf("%64s", inbuf);
                strcat(inbuf, "\r\n");
                uart_send(inbuf);
                break;
            case 3:
                printf("*********** Ricevi caratteri ***********\n");
                //inuint8 = (uint8_t) uart_getc(UART_PORT);
                memset(inbuf, 0, MAX_BUF_LEN);
                //printf(">> In ricezione %u caratteri:\n", inuint8);
                //uart_read_blocking(UART_PORT, inbuf, MAX_BUF_LEN);
                i = 0;
                do {
                    uart_read_blocking(UART_PORT, inbuf + i, 1);
                    //inbuf[i] = uart_getc(UART_PORT);
                    i++;
                } while (uart_is_readable_within_us(UART_PORT, US_TO_WAIT));
                printf(">> %s\n",inbuf);
                break;
            case 10:
                printf(">> Uscita ... \n");
                esc = false;
                break;
            default:
                printf(">> Comando non valido \n");
                break;
        }
        printf("\n");
    } while(esc);
    

    return 0;
}
