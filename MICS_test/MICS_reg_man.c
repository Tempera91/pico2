/*  *   *   *   *   *   *   *   Register manipulation   *   *   *   *   *   
    
    */
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/timer.h"

#include "mics_hw.h"
#include "mics_lib.h"
#include "pico_lib.h"
#include "General.h"

/* -    -   -   -    Function prototypes    -   -   -   -   -   -   -   */

uint8_t zl_write_registers(uint8_t addr, uint8_t data);
uint8_t zl_read_registers(uint8_t addr, uint8_t *data);
bool disable_wdog();
void disable_wdog_fast();
void enable_wdog();
void set_channel(uint8_t chnl);
void reset_core_cmd();
static inline void set_page(uint8_t page);
void set_bit_in_reg(uint8_t addr, uint8_t mask);
void clear_bit_in_reg(uint8_t addr, uint8_t mask);
void inline set_channel(uint8_t chnl);
void inline set_power(uint8_t pw_code);
void inline set_resend_time(uint8_t resnd_time);
void inline set_IMD_transID(uint32_t imd_ID);
void inline set_company_ID(uint8_t comp_ID);
void inline set_moduser(uint8_t tx, uint8_t rx);
int emrg_env_on();
int emrg_env_off();
int listen_IMD_wu_msg (bool enable);
int abort_cmd();
void inline clear_irq(uint8_t irq_reg, uint8_t irq_mask);
void inline set_initcom_245(bool wake);
void wait_for_irq(uint8_t irq_reg, uint8_t irq_mask);
int wait_for_irq_timeout_ms(uint8_t irq_reg, uint8_t irq_mask, uint32_t timeout_ms);
int hk_remote_write(uint8_t hk_addr, uint8_t hk_data, uint8_t *hk_reply);
int hk_write_remotereg(uint8_t hk_addr, uint8_t hk_data, uint8_t *hk_reply);
int hk_read_remotereg(uint8_t hk_addr, uint8_t *hk_reply);
void hk_remote_abort();
int align_TXRX_buff_dimensions(bool bs_to_imd);
int flush_rx();
/*  -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */

/**
 * @brief Send core reset command:
 *        reset MAC and RF subsystems, but not wake-up
 * 
 */
void reset_core_cmd() {
    write_registers(CORE_RESET_ADDR, CORE_RESET_VAL, 1);
}

/*  Set the current page
    */ 
static inline void set_page(uint8_t page){
    ChipData.page=page;
    read_registers(REG_INTERFACE_MODE, &ChipData.interf_mode, 1);
    ChipData.interf_mode=(ChipData.interf_mode&0xFE)|~(page|0xFE);
    write_registers(REG_INTERFACE_MODE, ChipData.interf_mode, 1);
}


/* * * * * * * * Read and write data on registers * * * * * * * *
    Read and write data on ZL70103 chip. 
    Sets correct page and adjusts 7-bit address.
    Return number of readed/writed bytes.
    */
uint8_t zl_read_registers(uint8_t addr, uint8_t *data){
    int n_bytes_loc;
    uint8_t loc_addr;
    uint8_t new_page;
    /* Checking register page */
    new_page = (addr&MSB_8) ? 2 : 1;       // if addr>127, page 2, else page 1
    /* If new page != currrent page, set new current page */
    if ( new_page != ChipData.page ) {
        set_page(new_page);
    }
    loc_addr = addr&0x7F;                   // 7-bit address adjusting

    n_bytes_loc = read_registers(loc_addr,data,1);

    return n_bytes_loc;
}

uint8_t zl_write_registers(uint8_t addr, uint8_t data){
    int n_bytes_loc;
    uint8_t loc_addr;   
    uint8_t new_page;
    /* Checking register page */
    new_page = (addr&MSB_8) ? 2 : 1;       // if addr>127, page 2, else page 1
    /* If new page != currrent page, set new current page */
    if ( new_page != ChipData.page ) {
        set_page(new_page);
    }
    loc_addr = addr&0x7F;                   // 7-bit address adjusting

    n_bytes_loc=write_registers(loc_addr,data,1);
    
    return n_bytes_loc;
}


/** 
 * @brief Sets the bit in register 
 * 
 * @param addr Register address
 * @param mask Mask of bits to set
 */
void set_bit_in_reg(uint8_t addr, uint8_t mask) {
    uint8_t old_val, new_val;

    zl_read_registers(addr, &old_val);
    
    new_val = old_val | mask;

    zl_write_registers(addr, new_val);
}
/** 
 * @brief Clear the bit in register 
 * 
 * @param addr Register address
 * @param mask Mask of bits to clear
 */
void clear_bit_in_reg(uint8_t addr, uint8_t mask) {
    uint8_t old_val, new_val;

    zl_read_registers(addr, &old_val);
    
    new_val = old_val & (~mask);

    zl_write_registers(addr, new_val);
}

/*  Disable the watchdog and attempt to write companyID register, after wait of 5s.
    This test if watchdog is correctly disabilited
    Return: True */
bool disable_wdog(){
    uint8_t r_try=0, w_try=AUXVAL1, r_try_old;
    uint8_t var=DISABLE_WDOG;
    bool return_value;
    printf(">>>> disabling watchdog ... \n");
    // Clearing and disabiliting watchdog 
    write_registers(REG_CLEARWDOG,DISABLE_WDOG,1);

    wait_ms(5000);
    // Try register write: exchange values between r_try and w_try via Company ID register
    write_registers(REG_MAC_COMPANYID,w_try,1);    // write w_try value on register
    wait_us(2);
    r_try_old=r_try;                                        // saving r_try value
    read_registers(REG_MAC_COMPANYID,&r_try,1);    // Read register: inserting in r_try w_try old value
    wait_us(2);
    write_registers(REG_MAC_COMPANYID,r_try_old,1);    // write old r_try value on register
    wait_us(2);
    read_registers(REG_MAC_COMPANYID,&w_try,1);    // Read register: inserting in r_try w_try old value

    return_value = (r_try==AUXVAL1) && (w_try==0) ? true : false;
    printf ("r_try=%d w,_try=%d return_value=%d\n",r_try,w_try,return_value);
    return return_value;

}

/**
 * @brief Disable main watchdog without control
 * 
 */
void disable_wdog_fast() {
    printf(">>>> disabling watchdog ... \n");
    // Clearing and disabiliting watchdog 
    write_registers(REG_CLEARWDOG,DISABLE_WDOG,1);
}

/**
 * @brief Enable main watchdog
 * 
 */
void enable_wdog() {
    // printf(">>>> enabling watchdog ... \n");
    // Enabling watchdog 
    write_registers(REG_CLEARWDOG,ENABLE_WDOG,1);
}

/*  Set current channel 
    */
void inline set_channel(uint8_t chnl) {
    zl_write_registers(REG_MAC_CHANNEL, chnl);
    ChipData.channel=chnl;
}

/*  Set default transmission power.
    pw_code=0  for default value
    */
void inline set_power(uint8_t pw_code) {
    if (pw_code == 0) {
        pw_code = TXRFPWRDEFAULTSET_DEF;
    }
    zl_write_registers(REG_RF_TXRFPWRDEFAULTSET, pw_code);
    ChipData.power_code=pw_code;
}

/*  Set current resend time.
    Set the resend time in multiple of 0.427ms (default 23).
    */
void inline set_resend_time(uint8_t resnd_time) {
    zl_write_registers(REG_MAC_RESENDTIME, resnd_time);
    ChipData.resend_time = resnd_time;
}

/*  Set IMD transID to communicate
    */
void inline set_IMD_transID(uint32_t imd_ID) {
    uint8_t byte_send;
    for (uint8_t i=0; i<3; i++) {
        byte_send = (imd_ID >> i*BYTE_BIT_SPAN) & 0xFF;
        zl_write_registers(REG_MAC_IMDTRANSID1+i, byte_send);
    }
    ChipData.IMD_transID = imd_ID;
}

/*  Set Company ID
    */
void inline set_company_ID(uint8_t comp_ID) {
    zl_write_registers(REG_MAC_COMPANYID, comp_ID);
    ChipData.company_ID = comp_ID;
}

/*  Set user modulation mode.
    */
void inline set_moduser(uint8_t tx, uint8_t rx) {
    zl_write_registers(REG_MAC_MODUSER, tx|rx);
    ChipData.moduser = tx|rx;
}

/**
 * @brief Configure registers for emergency listen and communication environment
 * 
 * @return Error code:  ALSO_IN_EMRG-> also in emergency mode
 *                      ALSO_IN_SESSION-> if in session, can't change environment 
 *                      EMRG_ENV_SET_ERR-> error in enviroment setting
 */
int emrg_env_on() {
    uint8_t ret = 0;
    uint8_t count = 0;
    /* Control if emergency environment is also loaded */
    if (ChipData.flags.emergency_on) {
        return ALSO_IN_EMRG;
    }
    /* Control if there is active session; in that case, can't change environment */
    if (ChipData.flags.emergency_on || ChipData.flags.on_session) {
        return ALSO_IN_SESSION;
    }
    ret += zl_write_registers(REG_MAC_MODUSER, ChipData.emrg_moduser);
    count++;
    ret += zl_write_registers(REG_MAC_CHANNEL, ChipData.emrg_channel);
    count++;
    ret += zl_write_registers(REG_TXBUFF_BSIZE, ChipData.data_tx.emrg_bsize);
    count++;
    ret += zl_write_registers(REG_RXBUFF_BSIZE, ChipData.data_rx.emrg_bsize);
    count++;
    ret += zl_write_registers(REG_TXBUFF_MAXPACKSIZE, ChipData.emrg_tx_max_pack_size);
    count++;
    ret += zl_write_registers(REG_MAC_RESENDTIME, ChipData.emrg_resend_time);
    count++;

    if (ret != count)
        return EMRG_ENV_SET_ERR;

    ChipData.flags.emergency_on = EMRG_ON;

    return 0;
}

/**
 * @brief Restore registers for normal communication 
 * 
 * @return Error code:  NOT_IN_EMRG-> not in emergency mode
 *                      ALSO_IN_SESSION-> if in session, can't change environment 
 *                      EMRG_ENV_SET_ERR-> error in enviroment setting
 */
int emrg_env_off() {
    uint8_t ret = 0;
    uint8_t count = 0;
    if (!ChipData.flags.emergency_on) {
        return NOT_IN_EMRG;
    }
    if (ChipData.flags.on_session) {
        return ALSO_IN_SESSION;
    }
    ret += zl_write_registers(REG_MAC_MODUSER, ChipData.moduser);
    count++;
    ret += zl_write_registers(REG_MAC_CHANNEL, ChipData.channel);
    count++;
    ret += zl_write_registers(REG_TXBUFF_BSIZE, ChipData.data_tx.bsize);
    count++;
    ret += zl_write_registers(REG_RXBUFF_BSIZE, ChipData.data_rx.bsize);
    count++;
    ret += zl_write_registers(REG_TXBUFF_MAXPACKSIZE, ChipData.tx_max_pack_size);
    count++;
    ret += zl_write_registers(REG_MAC_RESENDTIME, ChipData.resend_time);
    count++;
    ret += zl_write_registers(REG_IRQ_ENABLECLEAR1, IRQ_RXNOTEMPTY);
    count++;
    ret += zl_write_registers(REG_MAC_INITCOM, INITCOM_IBS_FL);
    count++;

    if (ret != count)
        return EMRG_ENV_SET_ERR;

    ChipData.flags.emergency_on = EMRG_OFF;

    return 0;
}

/**
 * @brief Starts and stops listen for IMD wakeup responses
 * 
 * @param enable true-> start listen; false-> stop listen
 * @return Error code: ALSO_IN_SESSION-> also in session; IMD_LISTEN_ERR-> error in writing registers
 */
int listen_IMD_wu_msg (bool enable) {
    uint8_t ret = 0, count = 0;
    if (ChipData.flags.on_session) 
        return ALSO_IN_SESSION;
    /*  if enable, starts listen wakeup message from IMD    */
    if (enable) {
        ret += zl_write_registers(REG_RXBUFF_BSIZE, IMPRVD_ERR_CORR(TRANSID_RX_BSIZE));
        count++;
        ret += zl_write_registers(REG_MAC_RESENDTIME, DISABLE_RESEND);
        count++;
        ret += zl_write_registers(REG_IRQ_ENABLESET1, IRQ_RXNOTEMPTY);  // enabling interrupt
        count++;
        ret += zl_write_registers(REG_MAC_INITCOM, RECEIVE_WU_MSG);
        count++;
    }
    /*  Stop listen and restore register values */
    else {
        ret += zl_write_registers(REG_MAC_INITCOM, INITCOM_IBS_FL); // stop receiving IMD message and set IBS
        count++;                                                    // flag, preparing to move base station 
                                                                    // in CHECK COMMAND IDLE after abort command
        if (ChipData.flags.emergency_on) {  // restore values for emergency session
            ret += zl_write_registers(REG_MAC_RESENDTIME, ChipData.emrg_resend_time);
            count++;
            ret += zl_write_registers(REG_RXBUFF_BSIZE, ChipData.data_rx.emrg_bsize);
            count++;
        }
        else {                              // restore value for normal session
            ret += zl_write_registers(REG_MAC_RESENDTIME, ChipData.resend_time);
            count++;
            ret += zl_write_registers(REG_RXBUFF_BSIZE, ChipData.data_rx.bsize);
            count++;
        }
        ret += zl_write_registers(REG_IRQ_ENABLECLEAR1, IRQ_RXNOTEMPTY);  // disabling interrupt
        count++;
    }
    if (ret != count)
        return IMD_LISTEN_ERR;

    return 0;
}

/**
 * @brief Send local abort command
 * 
 * @return Error code: TIMEOUT_ERR-> timeout in waiting for CHECK COMMAND IDLE state
 */
int abort_cmd() {
    int ret;
    uint8_t reg_mac_ctrl_loc;
    /*  Programming abort command for move to CHECK COMMAND IDLE state  */
    zl_write_registers(REG_MAC_CTRL,ABORT_LINK);
    ret = wait_for_irq_timeout_ms(REG_IRQ_RAWSTATUS2, IRQ_RADIOREADY, ABORT_TIMEOUT);
    if (!ret) {
        zl_write_registers(REG_MAC_CTRL, 0x00); // reset abort command 
    }
    return ret;
}

/**
 * @brief Send local rx buffer flush command
 * 
 * @return Error code: TIMEOUT_ERR-> timeout in waiting command done
 */
int flush_rx() {
    int ret=0;
    zl_write_registers(REG_MAC_CTRL,RX_BUF_FLUSH);
    ret = wait_for_irq_timeout_ms(REG_IRQ_RAWSTATUS2, IRQ_COMMANDONE, FLUSH_TIMEOUT);
    clear_irq(REG_IRQ_RAWSTATUS1, IRQ_RXNOTEMPTY);
    return ret;
}

/**
 * @brief Clear interrupts
 * 
 * @param irq_reg Address of interrupt register
 * @param irq_mask Bit mask of interrupt to clear
 */
void inline clear_irq(uint8_t irq_reg, uint8_t irq_mask) {
    zl_write_registers(irq_reg, ~irq_mask);
}

void inline set_initcom_245(bool wake) {
    if (wake) {
        zl_write_registers(REG_PO0, TX245);
        zl_write_registers(REG_MAC_INITCOM, INITCOM_245);
    }
    else {
        clear_bit_in_reg(REG_PO0, TX245);
        clear_bit_in_reg(REG_MAC_INITCOM, (BIT(0)|BIT(2)));
    }
    wait_ms(10);
    clear_irq(REG_IRQ_RAWSTATUS1, IRQ_ALL);
    clear_irq(REG_IRQ_RAWSTATUS2, IRQ_ALL);
    clear_irq(REG_IRQ_AUXSTATUS, IRQ_ALL);
}

void wait_for_irq(uint8_t irq_reg, uint8_t irq_mask) {
    uint8_t irq_stat = 0;
    while (!(irq_stat & irq_mask)) {
        zl_read_registers(irq_reg, &irq_stat);
    }
    clear_irq(irq_reg, irq_mask);
}

int wait_for_irq_timeout_ms(uint8_t irq_reg, uint8_t irq_mask, uint32_t timeout_ms) {
    uint8_t irq_stat = 0;
    absolute_time_t timeout_absolute;
    timeout_absolute = make_timeout_time_ms(timeout_ms);
    while (!(irq_stat & irq_mask)) {
        if (absolute_time_diff_us(timeout_absolute,get_absolute_time()) >= 0) {
            return TIMEOUT_ERR;
        }
        zl_read_registers(irq_reg, &irq_stat);
    }
    clear_irq(irq_reg, irq_mask);
    return 0;
}

/**
 * @brief Send housekeeping message for write remote register on PAGE1
 * 
 * @param hk_addr address of remote register to write
 * @param hk_data data to write on remote register
 * @param hk_reply reply of hk data write
 * @return 0 -> succesfull hk sent, -2 -> timeout reached: link fail, aborted link
 */
int hk_remote_write(uint8_t hk_addr, uint8_t hk_data, uint8_t *hk_reply) {
    int ret;
    /* Write data and address of register to write */
    zl_write_registers(REG_HK_TXDATA, hk_data);
    zl_write_registers(REG_HK_TXADDR, hk_addr&0x7F);   // ensure that MSB=0, for writing message
    /*  Wait for reply from remote device, until timeout.
        If timeout occour, so remote devices don't reply, means that link don't work.
        Then executes abort command, for stop session and return and TIMEOUT_ERROR.
        */
    ret = wait_for_irq_timeout_ms(REG_IRQ_RAWSTATUS1, IRQ_HKREMOTEDONE, HK_TIMEOUT);
    if (ret == TIMEOUT_ERR) {
        abort_link(LOCAL_ONLY);
    }
    else {
        zl_read_registers(REG_HK_TXREPLY,hk_reply);
    }
    return ret;
}

/**
 * @brief Send housekeeping message for write remote register
 * 
 * @param hk_addr address of remote register to write
 * @param hk_data data to write on remote register
 * @param hk_reply reply of hk data write; can be set NULL, if not required
 * @return 0 -> succesfull hk sent, -2 -> timeout reached: link fail, aborted link, -3 -> remote write disabled
 */
int hk_write_remotereg(uint8_t hk_addr, uint8_t hk_data, uint8_t *hk_reply) {
    int ret;
    uint8_t hk_reply_loc;
    
    /* Select page for remote register access: 
        if hk_addr > 127, select page 2; thus configure reg_hk_mode remote register for selecting second
        page; check also write succesfull 
        */
    if (hk_addr&MSB_8) {
        ret = hk_remote_write(REG_HK_MODE,REM_PAGE_SEL|HK_WRITE_EN,&hk_reply_loc);
        if (ret == TIMEOUT_ERR)
            return ret;
        if (hk_reply_loc != (REM_PAGE_SEL|HK_WRITE_EN)) 
            return HK_WR_NOT_PERM;
    }
    /*  Write data on register */
    ret = hk_remote_write(hk_addr&0x7F,hk_data,&hk_reply_loc);
    if (ret == TIMEOUT_ERR)
        return ret;
    if (hk_reply_loc != hk_data) 
        return HK_WR_NOT_PERM;
    if (hk_reply != NULL)
        *hk_reply = hk_reply_loc;
    /*  Reselect PAGE1, if it's changed
        */
    if (hk_addr&MSB_8) {
        ret = hk_remote_write(REG_HK_MODE,HK_WRITE_EN,&hk_reply_loc);
        if (ret == TIMEOUT_ERR)
            return ret;
        if (hk_reply_loc == ~HK_WRITE_EN) 
            return HK_WR_NOT_PERM;
    }
    return ret;
}

/**
 * @brief Send housekeeping message for read remote register
 * 
 * @param hk_addr address of remote register to write
 * @param hk_reply reply of hk message, contain readed data
 * @return 0 -> succesfull hk sent, -2 -> timeout reached: link fail, aborted link
 */
int hk_read_remotereg(uint8_t hk_addr, uint8_t *hk_reply) {
    int ret;
    uint8_t hk_reply_loc;
    /* Select page for remote register access: 
        if hk_addr > 127, select page 2; thus configure reg_hk_mode remote register for selecting second
        page; check also write succesfull 
        */
    if (hk_addr&MSB_8) {
        ret = hk_remote_write(REG_HK_MODE,REM_PAGE_SEL|HK_WRITE_EN,&hk_reply_loc);
        if (ret == TIMEOUT_ERR)
            return ret;
        if (hk_reply_loc != (REM_PAGE_SEL|HK_WRITE_EN)) 
            return HK_WR_NOT_PERM;
    }
    zl_write_registers(REG_HK_TXADDR, hk_addr|MSB_8);   // ensure that MSB=1, for reading message
    /*  Wait for reply from remote device, until timeout.
        If timeout occour, so remote devices don't reply, means that link don't work.
        Then executes abort command, for stop session and return and TIMEOUT_ERROR.
        */
    ret = wait_for_irq_timeout_ms(REG_IRQ_RAWSTATUS1, IRQ_HKREMOTEDONE, HK_TIMEOUT);
    if (ret == TIMEOUT_ERR) {
        abort_link(LOCAL_ONLY);
    }
    else {
        zl_read_registers(REG_HK_TXREPLY,hk_reply);
        printf("hk_reply=%02x\n", *hk_reply);
    }
    /*  Reselect PAGE1, if it's changed
        */
    if (hk_addr&MSB_8) {  
        ret = hk_remote_write(REG_HK_MODE,HK_WRITE_EN,&hk_reply_loc);   
        if (ret == TIMEOUT_ERR)
            return ret;
        if (hk_reply_loc == ~HK_WRITE_EN) 
            return HK_WR_NOT_PERM;
    }
    return ret;
}

/**
 * @brief Send abort link command with HK message
 * 
 */
void hk_remote_abort(){
    zl_write_registers(REG_HK_TXDATA, ABORT_LINK);
    zl_write_registers(REG_HK_TXADDR, REG_MAC_CTRL&(~BIT(7)));
    wait_ms(500);
}
/**
 * @brief Align base station and IMD buffers block dimensions.
 *  If HK registers write is disabled on IMD, return with no actions.
 * 
 * @param bs_to_imd True-> base station to IMD; False-> IMD to base station
 * @return int 0 if base station can set IMD parameters, HK_WR_NOT_PERM if hk remote write is disabled.
 */
int align_TXRX_buff_dimensions(bool bs_to_imd) {
    int ret;
    uint8_t tx_bsize_loc, rx_bsize_loc, max_pack_size_loc;
    /* Ensure that session is on */
    if (!ChipData.flags.on_session) {
            return NOT_IN_SESSION;
    }
    tx_bsize_loc = (ChipData.flags.emergency_on) ? ChipData.data_tx.emrg_bsize : ChipData.data_tx.bsize;
    rx_bsize_loc = (ChipData.flags.emergency_on) ? ChipData.data_rx.emrg_bsize : ChipData.data_rx.bsize;
    max_pack_size_loc = (ChipData.flags.emergency_on) ? ChipData.emrg_tx_max_pack_size : ChipData.tx_max_pack_size;
    /* if requested base station to imd synchronization, try to write imd registers */
    if (bs_to_imd) {
        ret = hk_remote_write(REG_RXBUFF_BSIZE, tx_bsize_loc, NULL);
        if (ret) 
            return ret;
        ret = hk_remote_write(REG_TXBUFF_BSIZE, rx_bsize_loc, NULL);
        if (ret) 
            return ret;
        ret = hk_remote_write(REG_TXBUFF_MAXPACKSIZE, max_pack_size_loc, NULL);
    }
    else {
        ret = hk_read_remotereg(REG_RXBUFF_BSIZE, &tx_bsize_loc);
        if (ret) 
            return ret;
        ret = hk_read_remotereg(REG_TXBUFF_BSIZE, &rx_bsize_loc);
        if (ret) 
            return ret;
        ret = hk_read_remotereg(REG_TXBUFF_MAXPACKSIZE, &max_pack_size_loc);
        /* Update base station registers */
        if (!ret) {
            zl_write_registers(REG_TXBUFF_BSIZE, tx_bsize_loc);
            zl_write_registers(REG_RXBUFF_BSIZE, rx_bsize_loc);
            zl_write_registers(REG_TXBUFF_MAXPACKSIZE, max_pack_size_loc);
            ChipData.data_tx.bsize = tx_bsize_loc;
            ChipData.data_rx.bsize = rx_bsize_loc;
            ChipData.tx_max_pack_size = max_pack_size_loc;
        }
    }
    return ret;
}