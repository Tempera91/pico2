/*
 */
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/adc.h"

#include "pico_lib.h"

#include "mics_hw.h"
#include "General.h"
#include "CC_lib.h"

/** Continuous wave transmission.
    \brief Program a 400MHz continuous wave transmission on a specified channel and with specified power.
    \param chnl Channel 
    \param pwr_code Power code for TXRFPWRDEFAULTSET register
    \return void
    */
void cw_transmission(uint8_t chnl, uint8_t pwr_code) {
    uint8_t pwr_code_bkup, chnl_bkup;
    uint8_t txiffreq_bkup, genenanles_bkup;
    char res=0;

    pwr_code_bkup = ChipData.power_code;
    chnl_bkup = ChipData.channel;
    zl_read_registers(REG_RF_GENENABLES, &genenanles_bkup);
    zl_read_registers(REG_MAC_TXIFFREQ, &txiffreq_bkup);
 
    set_channel(chnl);
    
    zl_write_registers(REG_MAC_TXIFFREQ, TXIFFREQ_CW_WORD);
    set_power(pwr_code);
    zl_write_registers(REG_RF_GENENABLES, TX_MODE_EN);

    wait_ms(5);

    printf(">> CW transmission on ch%d ...\t'q' for quit\n",ChipData.channel);
    while (res!='q') {
        printf("> 'q' for quit\n");
        scanf(" %c",&res);
    }

    zl_write_registers(REG_RF_GENENABLES, genenanles_bkup);
    wait_ms(5);
    zl_write_registers(REG_MAC_TXIFFREQ, txiffreq_bkup);
    set_channel(chnl_bkup);
    set_power(pwr_code_bkup);
}

/** 2.45GHz Continuous wave transmission
 * \brief Program a 2.45GHz continuous wave transmission with preselected power.
 * \param none
 * \return void 
 */
void cw_245_tx(bool enable) {
    if (enable) {
        // Enable CC2500 to transmitt 2.45GHz carrrier wave and enabling PA
        cc_transmit();     
        tx_245_en_set();
        // Set high PO0 pin                                         
        zl_write_registers(REG_PO0, GPO_ON_POx); 
        zl_write_registers(REG_GPO, BIT(0));
    }
    else {
        // Set low PO0 pin 
        zl_write_registers(REG_GPO, 0x00);
        zl_write_registers(REG_PO0, 0x00); 
        //switch off CC2500 to transmitt 2.45GHz carrrier wave and disabling PA
        cc_sleep(); 
        tx_245_en_clear();
    }    
}