/*  This file contain functions for MICS session communications
    */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/timer.h"

#include "mics_hw.h"
#include "pico_lib.h"
#include "mics_lib.h"
#include "CC_lib.h"
#include "General.h"

/*          Functions prototypes        */
int wait_link_ready(uint32_t timeout_ms);
int start_245_session();
int start_400_session();
int receive_IMD_transID(uint32_t *imd_transID, uint32_t timeout);
int abort_link(bool remote);
void tx_buff_data_push (uint8_t *data_buffer, uint16_t* start_index, uint16_t n_data);
void zl_send_data (uint8_t *data, uint16_t n_data);
void zl_receive_data ();


/*  Wait for asserting link_ready interrupt
    */
int wait_link_ready(uint32_t timeout_ms) {
    uint8_t irq_statt;
    absolute_time_t timeout_absolute;
    timeout_absolute = make_timeout_time_ms(timeout_ms);
    while (1) {
        zl_read_registers(REG_IRQ_RAWSTATUS2,&irq_statt);      // read interrupt status
        printf("--> irq_statt:%x\n",irq_statt);
        if (irq_statt & BIT(2)) {                                 // if link_ready interrupt is set
            zl_write_registers(REG_IRQ_RAWSTATUS2,~BIT(2));   // clear interrupt
            break; 
        }
        wait_us(1000);
        if (absolute_time_diff_us(timeout_absolute,get_absolute_time()) >= 0) {
            return TIMEOUT_ERR;
        }
    }
    return 0;
}
/**
 * \brief Link Start-Up with 2.45GHz Wake-Up
 * 
 */
int start_245_session() {
    int ret;
    cc_transmit();
    tx_245_en_set();
    set_initcom_245(true);
    ret = wait_link_ready(START_SESS_TIMEOUT);
    tx_245_en_clear();
    cc_sleep();
    set_initcom_245(false);
    if (ret == TIMEOUT_ERR) {
        ChipData.flags.on_session = false;
    }
    else {
        ChipData.flags.on_session = true;
    }
    return ret;
}

/**
 * \brief Link Start-Up with 400MHz Wake-Up
 * 
 * \return error code
 */
int start_400_session() {
    int ret;
    uint8_t irq_raw2, initcom_loc, resend_loc;
    zl_write_registers(REG_MAC_INITCOM, INIT_400_SESS);
    ret = wait_for_irq_timeout_ms(REG_IRQ_RAWSTATUS2, IRQ_LINK_READY, START_SESS_TIMEOUT);
    if (!ret) {
        ChipData.flags.on_session = true;
    }
    return ret;
}

/**
 * @brief Wait for irq_rxnotempty and read received IMD transceiver ID
 *        stops when timeout is reached
 * 
 * @param imd_transID pointer to variable in witch load received imd_transID 
 * @param timeout timeout for waiting IMD responses
 * @return Error code: TIMEOUT_ERR-> reached timeout in waiting interrupt
 */
int receive_IMD_transID(uint32_t *imd_transID, uint32_t timeout) {
    int ret = 0;
    uint8_t data[TRANSID_RX_BSIZE];
    memset((void*) data, 0, TRANSID_RX_BSIZE);
    *imd_transID = 0x00000000;
    ret = wait_for_irq_timeout_ms(REG_IRQ_RAWSTATUS1, IRQ_RXNOTEMPTY, timeout);
    if (!ret){
        read_registers(REG_TXRXBUFF, data, TRANSID_RX_BSIZE);
        for (int i=0; i<TRANSID_SIZE; i++) {
            *imd_transID |= (data[TRANSID_SIZE-1-i] << (i*BYTE_BIT_SPAN));
        }   
    }
    return ret;
}

/**
 * @brief Abort 400MHz communication link
 * @param remote true-> send hk abort message; false-> don't send hk abort message
 * 
 * @return error code: TIMEOU_ERR
 * 
 */
int abort_link(bool remote) {
    uint8_t irq_stat = 0;
    zl_read_registers(REG_MAC_TOPSTATE, &irq_stat);   
    if (irq_stat == BASE_IDLE){      // module in CHECK COMMAND IDLE STATE, no need aborting
        ChipData.flags.on_session = false;
        return 0;
    }
    if (remote) {
        hk_remote_abort();
    }
    /*  Ensure that only bit3 (IBS) is high for pevent to start new session 
        after link is aborted */
    zl_write_registers(REG_MAC_INITCOM, INITCOM_IBS_FL); 
    // Send abort link command
    zl_write_registers(REG_MAC_CTRL, ABORT_LINK);
    /*  Wait for moving to the CHECK COMMAND IDLE STATE */
    /*while (!(irq_stat & IRQ_RADIOREADY)) {
        zl_read_registers(REG_IRQ_RAWSTATUS2, &irq_stat);
    }*/
    if (wait_for_irq_timeout_ms(REG_IRQ_RAWSTATUS2,IRQ_RADIOREADY,ABORT_TIMEOUT) == TIMEOUT_ERR) {
        clear_bit_in_reg(REG_MAC_CTRL, 0xFF);
        ChipData.flags.on_session = false;
        return TIMEOUT_ERR;
    }
    /*  Clear the abort command link    */
    clear_bit_in_reg(REG_MAC_CTRL, 0xFF);
    /*  Flush TX buffer */
    zl_write_registers(REG_MAC_CTRL, TX_BUF_FLUSH);
    if (wait_for_irq_timeout_ms(REG_IRQ_RAWSTATUS2, IRQ_COMMANDONE,ABORT_TIMEOUT) == TIMEOUT_ERR) {
        clear_bit_in_reg(REG_MAC_CTRL, 0xFF);
        ChipData.flags.on_session = false;
        return TIMEOUT_ERR;
    }
    //wait_for_irq(REG_IRQ_RAWSTATUS2, IRQ_COMMANDONE);
    /*  Flush RX buffer */
    zl_write_registers(REG_MAC_CTRL, RX_BUF_FLUSH);
    if (wait_for_irq_timeout_ms(REG_IRQ_RAWSTATUS2, IRQ_COMMANDONE,ABORT_TIMEOUT) == TIMEOUT_ERR) {
        clear_bit_in_reg(REG_MAC_CTRL, 0xFF);
        ChipData.flags.on_session = false;
        return TIMEOUT_ERR;
    }
    //wait_for_irq(REG_IRQ_RAWSTATUS2, IRQ_COMMANDONE);
    /*  Clear irq_txnotempty    */
    clear_irq(REG_IRQ_RAWSTATUS1, IRQ_RXNOTEMPTY);
    ChipData.flags.on_session = false;
    emrg_env_off();
    return 0;
}

/* -    -   -   -   -   -   -   -    Data sending/receiving -   -   -   -   -   -   - */

/** \brief Insert data in tx data buffer until is full or all data are pushed
 *
 * \param data_buffer pointer to containing data buffer
 * \param start_index data_buffer index whence start to read data
 * \param n_data number of bytes in the data buffer
 * \return void
 *
 *  Try to push n_data bytes of data in the transmission buffer, reading from start_index in the data_buffer.
 * If the transmission buffer cant't contain all of data, it is completely filled and start_index is updated
 * at first byte not inserted.
 * The unused bytes of the last block in the transmission buffer are filled with 0.
 */
void tx_buff_data_push (uint8_t *data_buffer, uint16_t* start_index, uint16_t n_data) {
    uint16_t i = 0;
    uint8_t bsize_loc, tx_max_pack_size_loc;

    /* Choosing correct block size and tx max pack size, between normal and emergency environments */
    bsize_loc = (ChipData.flags.emergency_on) ? ChipData.data_tx.emrg_bsize : ChipData.data_tx.bsize;
    tx_max_pack_size_loc = (ChipData.flags.emergency_on) ? ChipData.emrg_tx_max_pack_size : ChipData.tx_max_pack_size;
    
    /* Inserting data in transmission buffer */
    while ((( *start_index + i) < n_data) && (i < tx_max_pack_size_loc*bsize_loc)) { // Stop when all data are pushed
        ChipData.data_tx.buff[ChipData.data_tx.used_blocks*bsize_loc + i] = data_buffer[*start_index + i];    // or transmission buffer is full
        i++;
    }

    *start_index += i;  // updating start_index
    /* Updating used_blocks  */
    ChipData.data_tx.used_blocks = (i % bsize_loc) ? i/bsize_loc + 1 : i/bsize_loc;
    /* If the last wrote block is not full, fill remaining bytes with 0 */
    if (((i) % bsize_loc)) {
        memset(&ChipData.data_tx.buff[(ChipData.data_tx.used_blocks-1)*bsize_loc + i%bsize_loc], 0, i%bsize_loc);
    }

}

/** \brief Send data to zl transmission buffer
 *
 * \param data buffer containing data to deliver
 * \param n_data number of bytes of data to deliver
 * \return void
 *
 */
void zl_send_data (uint8_t *data, uint16_t n_data) {
    uint16_t curr_index = 0;
    uint8_t irq_raw1 = 0;
    uint8_t irq_raw2 = IRQ_TXBUFFOVERFLOW;
    uint8_t bsize_loc, tx_max_pack_size_loc;

    /* Choosing correct block size and tx max pack size, between normal and emergency environments */
    bsize_loc = (ChipData.flags.emergency_on) ? ChipData.data_tx.emrg_bsize : ChipData.data_tx.bsize;
    tx_max_pack_size_loc = (ChipData.flags.emergency_on) ? ChipData.emrg_tx_max_pack_size : ChipData.tx_max_pack_size;
    
    memset(ChipData.data_tx.buff, 0, bsize_loc * tx_max_pack_size_loc * sizeof(uint8_t));
    /* Push data in the software transmission buffer and then send it to zl transmission buffer register.
        Cycle goes until all data are delivered.
    */
    while (curr_index < n_data) {
        tx_buff_data_push(data, &curr_index, n_data);
        while (!(irq_raw1&IRQ_TXEMPTY) || irq_raw2&IRQ_TXBUFFOVERFLOW) {
            irq_raw1 = IRQ_TXEMPTY;
            zl_write_registers(REG_IRQ_RAWSTATUS1,~irq_raw1);
            zl_write_registers(REG_IRQ_RAWSTATUS2,~IRQ_TXBUFFOVERFLOW);
            zl_read_registers(REG_IRQ_RAWSTATUS1, &irq_raw1);
            zl_read_registers(REG_IRQ_RAWSTATUS2, &irq_raw2);
        }
        
        mult_write_registers(REG_TXRXBUFF, ChipData.data_tx.buff, bsize_loc*ChipData.data_tx.used_blocks);
        ChipData.data_tx.used_blocks = 0;
    }
}

/** \brief receive data to zl transmission buffer
 *
 * \param data buffer containing data to deliver
 * \param n_data number of bytes of data to deliver
 * \return void
 *
 */
void zl_receive_data (){
    uint16_t n_data = 0;
    uint32_t i = 0;
    uint8_t bsize_loc;
    
    /* Choosing correct block size and tx max pack size, between normal and emergency environments */
    bsize_loc = (ChipData.flags.emergency_on) ? ChipData.data_rx.emrg_bsize : ChipData.data_rx.bsize;
    /* Wait for data */
    while (!n_data) {   
        zl_read_registers(REG_RXBUFF_USED, &ChipData.data_rx.used_blocks);
        n_data = bsize_loc * ChipData.data_rx.used_blocks;
    }
    /* Read data actually present in rx buffer */
    read_registers(REG_TXRXBUFF, ChipData.data_rx.buff, n_data);
    /* Clear irq */
    clear_irq(REG_IRQ_RAWSTATUS1, IRQ_RXNOTEMPTY);
}

