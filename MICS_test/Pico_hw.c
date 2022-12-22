/*
    */
#include <stdio.h>
#include "pico/stdio.h"
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "hardware/adc.h"

#include "pico_lib.h"

#include "mics_hw.h"
#include "General.h"



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    Initialization function for all pico board peripherals.
    */
void pico_init() {
    stdio_init_all();
    setvbuf ( stdin, NULL , _IOFBF , 2048 );
    // SPI initialisation. This example will use SPI at 1MHz.
    spi_init(SPI_PORT, 250*1000);
    // ADC initialization
    adc_init();
    gpio_init(GPIO26);
    adc_select_input(ADC0); // connect ADC0 (GPIO26) to ADC input

    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SIO);
    gpio_set_function(PIN_CS_CC, GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI); 
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_set_dir(PIN_CS_CC, GPIO_OUT);
    gpio_put(PIN_CS, 1);
    gpio_put(PIN_CS_CC, 1);

    // 2.45GHz Power amplifier PADJ pin (3.3V->enabled, 0->disabled)
    gpio_init(TX_245_EN);
    gpio_set_dir(TX_245_EN, GPIO_OUT);

    //GPIO for LED initialization 
    gpio_init_mask((uint)(1<<PICO_DEFAULT_LED_PIN)|(1<<LEDR0)|(1<<LEDR1)|(1<<LEDG0)|(1<<LEDG1)|BIT(RSSI_EN_PIN));
    gpio_set_dir_out_masked((uint32_t)(1<<PICO_DEFAULT_LED_PIN)|(1<<LEDR0)|(1<<LEDR1)|(1<<LEDG0)|(1<<LEDG1)|BIT(RSSI_EN_PIN));
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * ** * * * * ** * * * */

inline static void cs_select() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}
inline static void cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 1);  // Active low
    asm volatile("nop \n nop \n nop");
}
//select deselect cc2500 chip select
inline static void cscc_select() {
    asm volatile("nop \n nop \n nop \n nop \n nop");
    gpio_put(PIN_CS_CC, 0);  // Active low
    asm volatile("nop \n nop \n nop \n nop \n nop");
}
inline static void cscc_deselect() {
    asm volatile("nop \n nop \n nop \n nop \n nop");
    gpio_put(PIN_CS_CC, 1);  // Active low
    asm volatile("nop \n nop \n nop \n nop \n nop");
}

/* * * * * * * * Read and write data on registers * * * * * * * *
    Read and write data on registers, using 7-bit addressing mode.
    DOES NOT SET a new page.
    Return number of readed/writed bytes.
    */
uint8_t read_registers (uint8_t addr, uint8_t *data, uint8_t n_bytes) {
    int n_bytes_loc;
    uint8_t loc_addr=addr|READ_REG_MASK;        //Setting Rd bit

    if (loc_addr==addr) {               // second page address 
        printf(">>> Warning in fn read_register(): incompatible register address\n");
        return ADDR_ERR;
    }

    cs_select();
    spi_write_blocking(SPI_PORT,&loc_addr,1);    //7-bit addressing
    n_bytes_loc=spi_read_blocking(SPI_PORT,0,data,n_bytes);
    cs_deselect();

    return n_bytes_loc;
}

uint8_t write_registers (uint8_t addr, uint8_t data, uint8_t n_bytes) {
    int n_bytes_loc;
    uint8_t loc_addr=addr&(0x7F);
    uint8_t loc_data=data;

    if (loc_addr!=addr) {               // second page address 
        printf(">>>x Error in fn write_register(): incompatible register address\n");
        return ADDR_ERR;
    }

    cs_select();
    spi_write_blocking(SPI_PORT,&loc_addr,1);    //7-bit addressing
    n_bytes_loc=spi_write_blocking(SPI_PORT,&loc_data,n_bytes);
    cs_deselect();

    return n_bytes_loc;
}
// Multiple byte write
uint8_t mult_write_registers (uint8_t addr, uint8_t *data, uint8_t n_bytes) {
    int n_bytes_loc;
    uint8_t loc_addr=addr&(0x7F);

    if (loc_addr!=addr) {               // second page address 
        printf(">>>x Error in fn write_register(): incompatible register address\n");
        return ADDR_ERR;
    }

    cs_select();
    spi_write_blocking(SPI_PORT,&loc_addr,1);    //7-bit addressing
    n_bytes_loc=spi_write_blocking(SPI_PORT,data,n_bytes);
    cs_deselect();

    return n_bytes_loc;
}
/** *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   **/

/***************** Read and write CC2500 registers ***********************
 *  These are the functions for configuring CC2500 transceiver registers 
 *  and sending command strobes.
 *  */
/*! \brief CC2500 Read register 
    \param addr register address
    \return register readed value */
uint8_t cc_read (uint8_t addr) {
    uint8_t data;
    uint8_t loc_addr=addr|READ_REG_MASK;        //Setting Rd bit

    cscc_select();
    spi_write_blocking(SPI_PORT,&loc_addr,1);    //7-bit addressing
    spi_read_blocking(SPI_PORT,0,&data,1);
    cscc_deselect();

    return data;
}
/** Write register on CC2500
 * @brief CC2500 write register
 * @param addr Register address
 * @param data Data to write
 * @return Nothing
 * 
 */
void cc_write (uint8_t addr, uint8_t data) {
    uint8_t loc_addr=addr&(0x7F);
    uint8_t loc_data=data;

    cscc_select();
    spi_write_blocking(SPI_PORT,&loc_addr,1);
    spi_write_blocking(SPI_PORT,&loc_data,1);
    cscc_deselect();

}
/** Send command strobe
 * \brief Send a command to CC2500
 * \param comm Command to send 
*/
void cc_command (uint8_t comm) {
    uint8_t loc_comm=comm&(0x7F);

    cscc_select();
    spi_write_blocking(SPI_PORT,&loc_comm,1);
    cscc_deselect();

}
/** *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   **/

void cccs_pulse() {
    cscc_select();
    cscc_deselect();
    wait_us(50);
}