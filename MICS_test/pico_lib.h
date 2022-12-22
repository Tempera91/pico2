/* This file contains definitions for interfacing with Pico Board

 */

#ifndef pico_lib_h
#define pico_lib_h
// SPI defines
#define SPI_PORT            spi0
#define PIN_CS              20
#define PIN_CS_CC           13  //CC2500 transceiver chip select
#define PIN_MISO            16
#define PIN_MOSI            19
#define PIN_SCK             18

#define GPIO2               2
#define GPIO3               3
#define GPIO4               4
#define GPIO5               5
#define GPIO15              15
#define GPIO22              22
#define GPIO26              26
#define GPIO27              27
#define GIPO28              28
#define GPIO14              14

#define ADC0                0
#define ADC1                1
#define ADC2                2

#define LEDR0               GPIO5
#define LEDR1               GPIO4
#define LEDG0               GPIO3
#define LEDG1               GPIO2

#define RSSI_EN_PIN         GPIO22
#define ZL_IRQ_PIN          GPIO15
#define TX_245_EN           GPIO14

#define wait_ms(p)          sleep_ms(p)
#define wait_us(p)          sleep_us(p)
#define ledR0_on()          gpio_put(LEDR0,1)
#define ledR0_off()         gpio_put(LEDR0,0)
#define ledR1_on()          gpio_put(LEDR1,1)
#define ledR1_off()         gpio_put(LEDR1,0)
#define ledG0_on()          gpio_put(LEDG0,1)
#define ledG0_off()         gpio_put(LEDG0,0)
#define ledG1_on()          gpio_put(LEDG1,1)
#define ledG1_off()         gpio_put(LEDG1,0)
#define gpio_set(p)         gpio_put(p,1)
#define gpio_clear(p)       gpio_put(p,0)
#define tx_245_en_set()     gpio_put(TX_245_EN,1)
#define tx_245_en_clear()   gpio_put(TX_245_EN,0)

#define adc_get(p)          adc_read(p)

extern void pico_init();
extern uint8_t read_registers (uint8_t addr, uint8_t *data, uint8_t n_bytes);
extern uint8_t write_registers (uint8_t addr, uint8_t data, uint8_t n_bytes);
extern uint8_t mult_write_registers (uint8_t addr, uint8_t *data, uint8_t n_bytes); 
extern uint8_t cc_read (uint8_t addr);
extern void cc_write (uint8_t addr, uint8_t data);
extern void cc_command (uint8_t comm);
extern void cccs_pulse();

#endif