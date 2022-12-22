/*  This file contain the functions for initialize and switch on/off the
    CC2500 transceiver.
    */
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"

#include "pico_lib.h"
#include "CC_lib.h"


// -- -- Values -- -- 

#define MDMCFG0_VAL             0xC7    // Channel spacing mantissa value
#define MDMCFG1_VAL             0x03    // Channel spacing exponential factor value
#define MDMCFG2_OOK_NOC_NOP     0x30    // OOK mode, no coding, no preamble
#define DEVIATN_NODEV           0x00    // No frequency deviation 
#define MCSM0_AUTOCAL           0x18    // Autocalibrate when going from IDLE to TX states   
#define PKTCTRL0_VAL            0x32    // No whitening, static asynchronous data, no CRC, infinite packet lenght   
#define IOCFG0_OH               0x6F    // Put configurable output to high 

// Channel frequency word for base frequency of 2400MHz
#define FREQ2_VAL2400           0x64    // Channel frequency byte2 value
#define FREQ1_VAL2400           0x00    // Channel frequency byte1 value
#define FREQ0_VAL2400           0x00    // Channel frequency byte0 value
// Channel frequency word for base frequency of 2450MHz
#define FREQ2_VAL2450           0x66    // Channel frequency byte2 value
#define FREQ1_VAL2450           0x15    // Channel frequency byte1 value
#define FREQ0_VAL2450           0x55    // Channel frequency byte0 value

#define CHANNR_DEF              0x00    // Default TX channel 



/*          Global variables        */
CC_CONFIG CCconfig;


/** Resets CC2500 transceiver and sets reccommended and default values
    \brief Reset CC2500 transceiver
    \param freq Base frequency of TX WU signal
    \param pw_code  Power code
    \return 0: successfull; -1: error in resetting
    */
int cc_reset(int freq, uint8_t pw_code) {
    // resetting and initialize configuration structure
    memset(&CCconfig, 0, sizeof(CCconfig));
    CCconfig.base_frequency = freq;

    switch (CCconfig.base_frequency) {  // populating frequency word 
        case CC_TX_FREQ2400:
            CCconfig.freq0 = FREQ0_VAL2400;
            CCconfig.freq1 = FREQ1_VAL2400;
            CCconfig.freq2 = FREQ2_VAL2400;
            break;
        case CC_TX_FREQ2450:
            CCconfig.freq0 = FREQ0_VAL2450;
            CCconfig.freq1 = FREQ1_VAL2450;
            CCconfig.freq2 = FREQ2_VAL2450;
            break;
        default:
            return -1;
    }

    CCconfig.channel = CHANNR_DEF;
    CCconfig.power_code = pw_code;
    // Reset CC2500 chip
    cc_command(CC_SRES);
    /* Configuration: CC2500 must be configured for a 2.45GHz continuous wave transmission */
    // Configuring power code 
    cc_write(PATABLE0, CCconfig.power_code);
    // Configuring channel spacing
    cc_write(MDMCFG0, MDMCFG0_VAL);     
    cc_write(MDMCFG1, MDMCFG1_VAL);
    //  Configuring base frequency and channel
    cc_write(FREQ2, CCconfig.freq2);
    cc_write(FREQ1, CCconfig.freq1);
    cc_write(FREQ0, CCconfig.freq0);
    cc_write(CHANNR, CCconfig.channel);
    // Configuring OOK mode, with no coding and no preamble
    cc_write(MDMCFG2, MDMCFG2_OOK_NOC_NOP); 
    // Cnfiguring no frequency deviation
    cc_write(DEVIATN, DEVIATN_NODEV);
    // Configuring autocalibrate when going from idle to TX states
    cc_write(MCSM0, MCSM0_AUTOCAL);
    // Configuring infinite packet length, static asyncronous data and no CRC
    cc_write(PKTCTRL0, PKTCTRL0_VAL);
    // Set configurable output high for prevent digital noise
    cc_write(IOCFG0, IOCFG0_OH);

    cc_command(CC_SIDLE);   // ensure that we are in idle state
    cc_command(CC_SPWD);    // switch off the transceiver

    return 0;
}

/** Wake up transceiver and start 2.45GHz carrier transmission
 *  \brief Start 2.45GHz carrier transmission
*/
inline void cc_transmit() {
    // Wake up transceiver and put that in idle state
    cc_command(CC_SIDLE);
    wait_ms(1);     // wait for stabilizing transceiver oscillator
    // Start TX
    cc_command(CC_STX);
}

/** Eventually stop 2.45GHz carrier transmission, and switch off transceiver
 *  \brief Stop 2.45GHz TX and Switch off transceiver
*/
inline void cc_sleep() {
    // Stop TX and put transceiver in idle state
    cc_command(CC_SIDLE);
    // Put transceiver in sleep state
    cc_command(CC_SPWD);
}


/** Eventually stop 2.45GHz carrier transmission, and switch off transceiver
 *  \brief Stop 2.45GHz TX and Switch off transceiver
*/
inline void cc_set_power() {
    // Stop TX and put transceiver in idle state
    cc_command(CC_SIDLE);
    // Put transceiver in sleep state
    cc_command(CC_SPWD);
}
