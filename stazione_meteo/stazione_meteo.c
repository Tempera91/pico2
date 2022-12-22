#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"
#include "pico/multicore.h"

#include "aht20_lib.h"

// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9

/* Private defines-------------------------------------------------------*/
#define HUM_SENS_DLY_MS             1000

/* Global Variables -----------------------------------------------------*/
humidity_sensor_t hum_sens = {
    .initialized = false
};

/* Functions prototypes -------------------------------------------------*/
void error_handler(const char * err_cod);
int64_t alarm_callback(alarm_id_t id, void *user_data);
bool rep_timer_callback(repeating_timer_t *rtim);
void read_hum_sensor();
void core1_entry();


/* Functions definitions ------------------------------------------------*/
int64_t alarm_callback(alarm_id_t id, void *user_data) {
    // Put your timeout handler code in here
    return 0;
}

bool rep_timer_callback(repeating_timer_t *rtim) {

    return true;
}


/**
 * @fn error_handler(const char * err_cod)
 * @brief Handler for generic error
 * 
 * @param err_cod string describing error
 */
void error_handler(const char * err_cod) {
    bool o = false;
    printf(">> Errore irreversibile: %s\n", err_cod);
    while (1) {
        gpio_put(PICO_DEFAULT_LED_PIN, o);
            o = !o;
            sleep_ms(90);
    }
}


/*************************************************************************/
/*                             MAIN FUNCTION                             */
/*************************************************************************/
int main()
{
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
    

    // I2C Initialisation. Using it at 50Khz.
    i2c_init(I2C_PORT, 50*1000);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    // Timer example code - This example fires off the callback after 2000ms
    //add_alarm_in_ms(2000, alarm_callback, NULL, false); 

    bool o = false;
    uint8_t rcvd_data[7];
    int ret;
    char in[10];
    uint32_t h, t;

    repeating_timer_t tim;

    scanf("%1s");
    gpio_put(PICO_DEFAULT_LED_PIN, 1);

    

    /* Searching for AHT20 sensor on i2c bus                            */
    printf (">> Searching for AHT20 sensor on i2c bus");
    while (i2c_read_blocking(I2C_PORT, AHT20_SLAVE_ADDR, 
            rcvd_data, 1, STOP_BIT) == PICO_ERROR_GENERIC) {
        printf(".");
        gpio_put(PICO_DEFAULT_LED_PIN, o);
        o = !o;
        sleep_ms(200);
    }
    printf ("\n>> AHT20 Sensor found on i2c bus: slave address=%02x\n",
                AHT20_SLAVE_ADDR);

    printf (">> AHT20 Sensor: initializing ... \n");
    if (AHT20_init(&hum_sens, I2C_PORT) == ACK_ERROR)
        error_handler("AHT20_init return: ACK_ERROR");
    printf (">> AHT20 Sensor initialized! \n");

    multicore_launch_core1(core1_entry);

    /*add_repeating_timer_ms(HUM_SENS_DLY_MS, rep_timer_callback, NULL, &tim);*/

    while (1) {

        gpio_put(PICO_DEFAULT_LED_PIN, o);
        o = !o;
        sleep_ms(HUM_SENS_DLY_MS);

    }

    return 0;
}

void core1_entry() {

    while (1){
        read_hum_sensor();
        sleep_ms(HUM_SENS_DLY_MS);
    }

}


void read_hum_sensor() {
    int ret;

    /* printf(">>> core 1!\n"); */
    ret = aht20_read_trig(&hum_sens);
    if (ret == ACK_ERROR)
        error_handler("AHT20_read return: ACK_ERROR\n");
    if (ret == NOT_INIT_ERR)
        printf(">> Not initialized: waiting...\n");
    else
        printf(">> Humidity: %.2f, Temperature: %.2f\r", hum_sens.humidity, hum_sens.temperature); 
}