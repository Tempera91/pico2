/**
 * @file general_inc.h
 * @author Simone Ricciardi
 * @brief 
 * @version 0.1
 * @date 2022-05-30
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT            i2c0
#define I2C_SDA             8
#define I2C_SCL             9

#define UART_PORT           uart0
#define UART0_TX_PIN        0
#define UART0_RX_PIN        1
#define UART0_REDE_PIN      6
#define UART_BAUDRATE       115200

/*      Definizioni applicative     */
#define MAX_BUF_LEN         128
#define US_TO_WAIT          1000*MS_TO_WAIT
#define MS_TO_WAIT          50
