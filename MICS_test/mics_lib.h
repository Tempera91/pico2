/*  This file contain declarations of mics functions
    */

#ifndef mics_lib_h
#define mics_lib_h

/*- -   -   -   -   - Macro defines -   -   -   -   -   -   -   -   -
    */
/*      Specifier for abort command only locally or also remote     */
#define LOCAL_ONLY              false
#define ALSO_REMOTE             true

/*- -   -   -   -   - Functions declarations -   -   -   -   -   -   -
    */ 
extern void int_RSSI_meas(uint8_t ch, uint8_t n_s, uint32_t t_s, uint16_t *adc_out_ave, uint16_t *adc_out_max);
extern void ext_RSSI_meas(int n_sample, uint32_t t_s, uint16_t *adc_out_ave, uint16_t *adc_out_max);
extern uint8_t cca();
extern void cw_transmission(uint8_t chnl, uint8_t pwr_code);
extern void cw_245_tx(bool enable);
extern int start_245_session();
extern int abort_link(bool remote);
extern int wait_link_ready(uint32_t timeout_ms);
extern void tx_buff_data_push (uint8_t *data_buffer, uint16_t* start_index, uint16_t n_data);
extern void zl_send_data (uint8_t *data, uint16_t n_data);
extern void zl_receive_data();
extern int start_400_session();
extern int receive_IMD_transID(uint32_t *imd_transID, uint32_t timeout);


#endif