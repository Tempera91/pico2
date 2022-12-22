/*  This header contain declarations of functions for configure
    and manage the CC2500 transceiver
    */

#ifndef CC_LIB_H
#define CC_LIB_H

/* Register and related values defines*/

#define PATABLE0                0x3E    // Power code register
#define MDMCFG0                 0x14    // Modem configuration register 
#define MDMCFG1                 0x13    // Modem configuration register
#define MDMCFG2                 0x12    // Modem configuration register
#define FREQ2                   0x0D    // Frequency control word: byte2
#define FREQ1                   0x0E    // Frequency control word: byte1
#define FREQ0                   0x0F    // Frequency control word: byte0
#define CHANNR                  0x0A    // Channel number register
#define DEVIATN                 0x15    // Frequency deviaton register
#define MCSM0                   0x18    // Main radio control state machine configuration register
#define PKTCTRL0                0x08    // Packet control register
#define IOCFG0                  0x02    // Output pin configuration register

#define CC_TX_FREQ2400        1       // Use base frequency of 2400MHz
#define CC_TX_FREQ2450        2       // Use base frequency of 2450MHz

/*          Power code values            */
#define PATABLE0_DEF            0xBF    // Default power code value (approx 20dBm)
#define PATABLE0_11             0xCA    // Power code for approx 11dBm

/* Command defines */
#define CC_STX                     0x35    // Enable TX (perform autocalibration if from IDLE state)
#define CC_SRES                    0x30    // Reset Chip
#define CC_SIDLE                   0x36    // Goes in IDLE state
#define CC_SPWD                    0x39    // Enter in power down mode when CSn goes hight

extern int cc_reset(int freq, uint8_t pw_code);
extern void cc_transmit();
extern void cc_sleep();

/*      Types definitions       */
typedef struct {
    uint8_t channel;
    uint8_t freq0;
    uint8_t freq1;
    uint8_t freq2;
    uint8_t power_code;
    int base_frequency;
} CC_CONFIG;  

extern CC_CONFIG CCconfig;
#endif