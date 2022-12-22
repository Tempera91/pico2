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

/*  Internal RSSI measurement using internal general pourpose ADC.
    Parameters: n_sample: number of consecutive samples; ts: sampling time;
                adc_out_ave: averaged RSSI result value; adc_out_max: max RSSI result
    */
void int_RSSI_meas(uint8_t ch, uint8_t n_s, uint32_t t_s, uint16_t *adc_out_ave, uint16_t *adc_out_max)
{
    uint8_t adc_output = 0;
    uint32_t adc_out_ave_loc = 0;
    *adc_out_ave = 0;
    *adc_out_max = 0;
    uint8_t genenables_bckup, rxgenctrl_bckup, adcctrl_bckup, channel_bkup;

    // saving value of used register
    zl_read_registers(REG_MAC_CHANNEL, &channel_bkup);
    zl_read_registers(REG_RF_GENENABLES, &genenables_bckup);
    zl_read_registers(REG_RF_RXGENCTRL, &rxgenctrl_bckup);
    zl_read_registers(REG_RF_ADCCTRL, &adcctrl_bckup);

    //  Selecting channel
    set_channel(ch & 0xF);
    // zl_write_registers(REG_MAC_CHANNEL, ch&0xF, 1);

    // Enable receiver and internal RSSI
    zl_write_registers(REG_RF_GENENABLES, RX_MODE_EN);
    zl_write_registers(REG_RF_RXGENCTRL, INT_RSSI_RXGENCTRL);
    wait_ms(5);

    // Enable ADC and select input to be converted
    zl_write_registers(REG_RF_ADCCTRL, INT_RSSI_ADCCTRL);
    wait_us(15);

    printf(" Internal RSSI measurement, results:\n");
    for (int i = 0; i < n_s; i++)
    {
        // Start conversion fo selected input and read result
        zl_write_registers(REG_RF_ADCCTRL, INT_RSSI_ADCCTRL | ADC_CONV_START); // START conversion
        wait_us(3);                                                               // wait conversion time

        while (!(adc_output & BIT(5)))
        { // check if converson is completed ([5] == 1)
            gpio_put(LEDG1, 1);
            zl_read_registers(REG_RF_ADCOUTPUT, &adc_output);
            wait_us(1);
        }
        adc_output &= (BIT(0) | BIT(1) | BIT(2) | BIT(3) | BIT(4)); // masking result unuseful bits
        printf("-> s%d: %d\n", i, adc_output);
        adc_out_ave_loc += adc_output;
        *adc_out_max = (adc_output > *adc_out_max) ? adc_output : *adc_out_max;
        zl_write_registers(REG_RF_ADCCTRL, INT_RSSI_ADCCTRL); // reset ADC and prepare for new conversion
        wait_ms(t_s);
    }
    *adc_out_ave = adc_out_ave_loc / n_s;
    printf("--> averaged RSSI=%d, RSSI max=%d", *adc_out_ave, *adc_out_max);

    // Restoring previous status of register
    zl_write_registers(REG_RF_ADCCTRL, adcctrl_bckup);
    wait_us(100);
    zl_write_registers(REG_RF_RXGENCTRL, rxgenctrl_bckup);
    wait_us(100);
    zl_write_registers(REG_RF_GENENABLES, genenables_bckup);
    wait_ms(5);
    set_channel(channel_bkup);
    // zl_write_registers(REG_MAC_CHANNEL, channel_bkup, 1);

    return;
}

/*  External RSSI measurement: connect IF signals to TESTIO5..6, that are connected to log amplifier.
    Takes multiple values from application u-processor's ADC.
    Parameters: n_sample: number of consecutive samples; t_s: sampling time;
                ave_res: averaged RSSI result value; max_res: max RSSI result
    */
void ext_RSSI_meas(int n_sample, uint32_t t_s, uint16_t *ave_res, uint16_t *max_res)
{
    uint8_t genenables_bckup, rxgenctrl_bckup, testioclamp_bckup, testiobufen_bckup;
    uint16_t adc_result, adc_max = 0;
    uint32_t adc_ave = 0;

    // - - - - - Saving values of used registers - - - - - -
    zl_read_registers(REG_RF_GENENABLES, &genenables_bckup);
    zl_read_registers(REG_RF_RXGENCTRL, &rxgenctrl_bckup);
    zl_read_registers(REG_WAKEUP_TESTIOCLAMP, &testioclamp_bckup);
    zl_read_registers(REG_WAKEUP_TESTIOBUFEN, &testiobufen_bckup);
    // - - - - - - - - - - - - - - - -- - - - - - - - - - -

    // Enable receiver and external RSSI
    zl_write_registers(REG_RF_GENENABLES, RX_MODE_EN);
    zl_write_registers(REG_RF_RXGENCTRL, EXT_RSSI_RXGENCTRL);
    // Configure RF subsystem signals on TESTIO5..6, disables clamp and enables buffers
    zl_write_registers(REG_WAKEUP_TESTIOCLAMP, TESTIOCLAMP_RSSI);
    zl_write_registers(REG_WAKEUP_TESTIOBUFEN, BUF_ON_TESTIO(6) | BUF_ON_TESTIO(5));

    wait_ms(5);

    gpio_set(RSSI_EN_PIN); // enable external log amplifier

    printf(" External RSSI measurement, results:\n");
    ledG0_on();

    for (int i = 0; i < n_sample; i++)              // take n_sample ADC values 
    {
        adc_result = adc_get();
        if (adc_result > adc_max)                   // if current value is the maximum
        {
            adc_max = adc_result;
        }
        adc_ave += adc_result;
        printf("-> s%d: %d\n", i, adc_result);
        wait_ms(t_s);                               // wait sampling time
    }
    *ave_res = adc_ave / n_sample;                  // averaged value
    *max_res = adc_max;                             // maximum value

    printf("--> averaged RSSI=%d, RSSI max=%d", *ave_res, *max_res);

    // - - - - - Restore values of used registers - - - - - -
    zl_write_registers(REG_RF_RXGENCTRL, rxgenctrl_bckup);
    zl_write_registers(REG_RF_GENENABLES, genenables_bckup);
    zl_write_registers(REG_WAKEUP_TESTIOBUFEN, testiobufen_bckup);
    zl_write_registers(REG_WAKEUP_TESTIOCLAMP, testioclamp_bckup);
    // - - - - - - - - - - - - - - - -- - - - - - - - - - -

    ledG0_off();
    gpio_clear(RSSI_EN_PIN); // disable external log amplifier
    printf(">>> Removed TESTIO5..6: Ext RSSI disabled\n");

    return;
}

/*  External RSSI measurement: connect IF signals to TESTIO5..6, that are connected to log amplifier.
    Takes multiple values in specified acquisition period (in ms), from application u-processor's ADC.
    Parameters: t_acq: acquisition time; t_s: sampling time (us);
                ave_res: averaged RSSI result value; max_res: max RSSI result
    */
void ext_RSSI_meas_ms(uint32_t t_acq, uint32_t t_s, uint16_t *ave_res, uint16_t *max_res)
{
    uint8_t genenables_bckup, rxgenctrl_bckup, testioclamp_bckup, testiobufen_bckup;
    uint16_t adc_result, adc_max = 0;
    uint32_t adc_ave = 0;
    absolute_time_t acqu_time;
    int i = 0;

    // - - - - - Saving values of used registers - - - - - -
    zl_read_registers(REG_RF_GENENABLES, &genenables_bckup);
    zl_read_registers(REG_RF_RXGENCTRL, &rxgenctrl_bckup);
    zl_read_registers(REG_WAKEUP_TESTIOCLAMP, &testioclamp_bckup);
    zl_read_registers(REG_WAKEUP_TESTIOBUFEN, &testiobufen_bckup);
    // - - - - - - - - - - - - - - - -- - - - - - - - - - -

    // Enable receiver and external RSSI
    zl_write_registers(REG_RF_GENENABLES, RX_MODE_EN);
    zl_write_registers(REG_RF_RXGENCTRL, EXT_RSSI_RXGENCTRL);
    // Configure RF subsystem signals on TESTIO5..6, disables clamp and enables buffers
    zl_write_registers(REG_WAKEUP_TESTIOCLAMP, TESTIOCLAMP_RSSI);
    zl_write_registers(REG_WAKEUP_TESTIOBUFEN, BUF_ON_TESTIO(6) | BUF_ON_TESTIO(5));

    wait_ms(5);

    gpio_set(RSSI_EN_PIN); // enable external log amplifier

    printf(" External RSSI measurement, results:\n");
    ledG0_on();

    acqu_time = make_timeout_time_ms(t_acq);    // generates timestamp of now + t_acqu [ms]
    while (!time_reached(acqu_time))            // take values for t_acq
    {
        adc_result = adc_get();
        if (adc_result > adc_max)               // if current value is the maximum
        {
            adc_max = adc_result;
        }
        adc_ave += adc_result;
        //printf("-> s%d: %d\n", i, adc_result);
        wait_us(t_s);

        i++;
    }
    *ave_res = adc_ave / i;     // averaged value
    *max_res = adc_max;         // max value

    printf("--> averaged RSSI=%d, RSSI max=%d", *ave_res, *max_res);

    // - - - - - Restore values of used registers - - - - - -
    zl_write_registers(REG_RF_RXGENCTRL, rxgenctrl_bckup);
    zl_write_registers(REG_RF_GENENABLES, genenables_bckup);
    zl_write_registers(REG_WAKEUP_TESTIOBUFEN, testiobufen_bckup);
    zl_write_registers(REG_WAKEUP_TESTIOCLAMP, testioclamp_bckup);
    // - - - - - - - - - - - - - - - -- - - - - - - - - - -

    ledG0_off();
    gpio_clear(RSSI_EN_PIN); // disable external log amplifier
    printf(">>> Removed TESTIO5..6: Ext RSSI disabled\n");

    return;
}

/*  Performs the clear channell assestment. 
    Takes several values of RSSI for each channel. Search the channel with lowest 
    RSSI value (maximum RSSI value, taken in CCA_ACQU_TIME).
    Return: selected channel 
    */
uint8_t cca()
{
    uint16_t ch_RSSI[MAX_CHNLS];
    uint16_t RSSI_min = 0xFFFF, RSSI_ave[MAX_CHNLS];
    int i_min = 0;

    for (int i = 0; i < MAX_CHNLS; i++)
    {
        set_channel((uint8_t)i);
        printf("---> Channel %d:", i);
        ext_RSSI_meas_ms(CCA_ACQ_TIME, 1000, &RSSI_ave[i], &ch_RSSI[i]);
        if (ch_RSSI[i] < RSSI_min)
        {
            RSSI_min = ch_RSSI[i];
            i_min = i;
        }
        printf("\tRSSI_min=%d;\tRSSI_ave=%d;\ti_min=%d\n", RSSI_min, RSSI_ave[i], i_min);
    }
    set_channel((uint8_t)i_min);
    printf(">>> CCA selected channel:%d\n", i_min);

    return (uint8_t)i_min;
}
