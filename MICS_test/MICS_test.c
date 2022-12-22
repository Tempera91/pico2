/* *    *   *   *   *   *   * Main function *   *   *   *   *   *   *   *

    This file contains main program routine: user interface, command scheduler etc.
    */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "pico/malloc.h"
#include "hardware/timer.h"

#include "pico_lib.h"

#include "mics_hw.h"
#include "General.h"
#include "mics_lib.h"
#include "CC_lib.h"

#define MAX_RETRY 3

/*      Global variables         */
CHIP_DATA ChipData;
char *input_buffer;

bool alarm_callback(struct repeating_timer *t)
{
    // Put your timeout handler code in here
    printf("Allarme!\n");
    return true;
}

/*  Module initializer:
    Disable watchdog, initializes chip data structures
    */
void module_init()
{
    memset(&ChipData, 0, sizeof(ChipData));
    /*  Initialize data buffers */
    ChipData.data_tx.bsize = TX_BUFF_BSIZE_DEF;
    ChipData.data_rx.bsize = RX_BUFF_BSIZE_DEF;
    ChipData.tx_max_pack_size = TX_BUFF_MAXPACK_SIZE;
    ChipData.data_tx.buff = malloc(ChipData.data_tx.bsize * ChipData.tx_max_pack_size * sizeof(uint8_t));
    ChipData.data_rx.buff = malloc(2 * ChipData.data_rx.bsize * ChipData.tx_max_pack_size * sizeof(uint8_t));
    memset(ChipData.data_tx.buff, 0, ChipData.data_tx.bsize * ChipData.tx_max_pack_size * sizeof(uint8_t));
    memset(ChipData.data_rx.buff, 0, 2 * ChipData.data_rx.bsize * ChipData.tx_max_pack_size * sizeof(uint8_t));
    zl_write_registers(REG_RXBUFF_BSIZE, ChipData.data_rx.bsize);
    zl_write_registers(REG_TXBUFF_BSIZE, ChipData.data_tx.bsize);
    zl_write_registers(REG_TXBUFF_MAXPACKSIZE, ChipData.tx_max_pack_size);

    /*  Initialize emergency chip data structure    */
    ChipData.emrg_moduser = MODUSER(RX_2FSK_BARK5, TX_2FSK_BARK5);
    ChipData.emrg_channel = EMRG_CHANNEL;
    ChipData.emrg_resend_time = EMRG_RESENDTIME;
    ChipData.emrg_tx_max_pack_size = EMRG_TX_MAXPACKSIZE;
    ChipData.data_tx.emrg_bsize = EMRG_TX_BSIZE;
    ChipData.data_rx.emrg_bsize = EMRG_RX_BSIZE;

    uint8_t reg_int_addr = REG_INTERFACE_MODE;
    if (disable_wdog())
    {
        ChipData.wdog_en = false;
        ChipData.err_stat = 0;
        printf(">>>> Watchdog disabled\n");
    }
    else
    {
        ChipData.err_stat = ERR_WRITEREG;
        printf(">>>x RegWriteError in attempting to disabilitate watchdog:   watchdog NOT disabled\n");
    }
    // initializing data module data structure
    if ((zl_read_registers(REG_INTERFACE_MODE, &ChipData.interf_mode) == 1) &&
        (zl_read_registers(REG_MAC_CHANNEL, &ChipData.channel) == 1) &&
        (zl_read_registers(REG_RF_TXRFPWRDEFAULTSET, &ChipData.power_code) == 1) &&
        (zl_read_registers(REG_MAC_RESENDTIME, &ChipData.resend_time) == 1) &&
        (zl_read_registers(REG_MAC_MODUSER, &ChipData.moduser) == 1))
    {
        // ChipData.page = (ChipData.interf_mode&PAGE_BIT_MASK) ? 2 : 1;
        printf(">>>> ChipData strucuture initialized\n");
    }
    else
    {
        ChipData.err_stat = ERR_READREG;
        printf(">>>x RegReadError in initializing ChipData structure:   ChipData structure NOT initialized\n");
    }
}

/*  -   -   -   -   -   -   -   - User interface functions  -   -   -   -   -   -
 */
// User can read and write any register
void user_registers_operations()
{
    int reg;
    int pag;
    char in;
    uint8_t dst[10];
    uint8_t read_byte, n_byte_to_read;
    uint8_t reg_add;

    printf("--->> Register operation command <<---\n");
    while (1)
    {
        read_byte = 0;

        memset(dst, 0, sizeof(dst));
        wait_ms(500);

        printf("> Registro?\n");
        scanf("%u", &reg);
        printf("> Pagina (1/2)?\n");
        scanf("%u", &pag);
        printf("--- reg: %x pag: %x\n", reg, pag);

        printf("> Leggere o scrivere? (r/w - 'q' per uscire)\n");
        scanf(" %c", &in);
        n_byte_to_read = 1;

        switch (in)
        {
        case 'r':
            reg_add = reg_pag_to_add(reg, pag);
            read_byte = zl_read_registers(reg_add, dst);

            if (read_byte == n_byte_to_read)
            {
                printf("Correttamente letto registro %x: %x\n", reg_add, dst[0]);
                wait_ms(1000);
            }
            else
            {
                printf("Registro %x non letto correttamente: dst=%x\n", reg_add, dst[0]);
            }
            break;
        case 'w':
            wait_ms(500);
            printf("Valore?\n");
            scanf("%x", dst);
            reg_add = reg_pag_to_add(reg, pag);
            read_byte = zl_write_registers(reg_add, dst[0]);
            if (read_byte == n_byte_to_read)
            {
                printf("Correttamente scritto registro %4x: %x\n", reg_add, dst[0]);
                wait_ms(1000);
            }
            else
            {
                printf("Registro %x non scritto correttamente: dst=%4x\n", reg_add, dst[0]);
            }
            break;
        case 'q':
            printf("--> Lettura/scrittura registri: uscita... \n");
            return;
        default:
            printf("-- no action --\n");
            break;
        }
    }
}
// user function for set modulatios modes
void set_moduser_user()
{
    uint8_t tx[5] = {TX_2FSK_FOLDB, TX_2FSK_BARK5, TX_2FSK_BARK11, TX_2FSK, TX_4FSK};
    uint8_t rx[5] = {RX_2FSK_FOLDB, RX_2FSK_BARK5, RX_2FSK_BARK11, RX_2FSK, RX_4FSK};
    int rest, resr;
    printf("> TX modulation? (0: 2FSK-Foldback; 1: 2FSK-Barker5; 2: 2FSK-Barker11; 3: 2FSK; 4: 4FSK\n");
    scanf("%d", &rest);
    printf("> RX modulation? (0: 2FSK-Foldback; 1: 2FSK-Barker5; 2: 2FSK-Barker11; 3: 2FSK; 4: 4FSK\n");
    scanf("%d", &resr);
    set_moduser(tx[rest], rx[resr]);
}

void set_transm_param()
{
    int res;
    uint8_t rt;
    uint32_t transID;
    printf("| Specify parameter to set:                                                              |\n");
    printf("|   1: set resend time; 2: set company ID; 3: set IMD transceiver ID; 4: set modulations |\n");
    printf("|   0: exit                                                                              |\n");
    scanf("%d", &res);

    while (res)
    {
        switch (res)
        {
        case 1:
            printf("> Resend time? (in multiple of 0.427ms) \n");
            scanf("%d", &rt);
            set_resend_time((uint8_t)rt);
            break;
        case 2:
            printf("> Company ID?\n");
            scanf("%s", input_buffer);
            rt = (uint8_t)strtoul(input_buffer, NULL, 16);
            set_company_ID(rt);
            break;
        case 3:
            printf("> IMD Transciver ID?\n");
            scanf("%s", input_buffer);
            transID = (uint32_t)strtoul(input_buffer, NULL, 16);
            set_IMD_transID(transID);
            break;
        case 4:
            set_moduser_user();
            break;
        default:
            printf(">>> Comando non valido\n");
            printf("|   1: set resend time; 2: set company ID; 3: set IMD transceiver ID; 4: set modulations |\n");
            printf("|   0: exit                                                                              |\n");
            break;
        }
        printf("setted. Other? (0: exit)\n");
        wait_ms(100);
        printf("> ");
        scanf("%d", &res);
    }
}

void start_session_user()
{
    char in;
    if (ChipData.flags.on_session)
    {
        if (ChipData.flags.emergency_on) {
            printf(">>> Warning: Emergency session in progress!\n");
            printf(">>> Do you want to restart? (y/n) (Warning: this will stop emergency session!)\n");

        }
        else {
            printf(">>> Warning: session already started!\n");
            printf(">>> Do you want to restart? (y/n)\n");
        }
        scanf(" %c", &in);
        if (in == 'y')
        {
            printf(">>> Aborting session, waiting for IMD watchdog...\n");
            abort_link(ALSO_REMOTE);
            wait_ms(5000);
            printf(">>> Session expired\n");
            wait_ms(100);
        }
        else
            return;
    }
    printf(">>> Do you want to set parameters? y/n\n");
    scanf(" %c", &in);
    if (in == 'y')
    {
        set_transm_param();
    }
    else
    {
        printf(">>> Using default or pre setted parameters...\n");
    }
    printf(">>> Clear Channel Assestment ...\n");
    cca();
    printf(">>> Starting Session with 2.45GHz wakeup ... \n");
    if (start_245_session() == TIMEOUT_ERR)
    {
        wait_ms(100);
        printf(">>>X Timeout reached, session not started \n");
    }
    else
    {
        //enable_wdog();
        printf("\t>>>> Session started <<<< \n");
        ledG0_on();
    }
}

/* Program 400Mz continuous wave transmission */
void cw_400()
{
    int chn, pw_code;
    printf(">> - CW 400MHz -\n");
    printf("> channel [0:11]?\n");
    scanf("%d", &chn);
    printf("> power code (0 - default)?\n");
    scanf("%d", &pw_code);
    cw_transmission((uint8_t)chn, (uint8_t)pw_code);
}
/* Program 2.44Gz continuous wave transmission */
void cw_2400()
{
    printf(">> - CW 2.4GHz -\n");
    ledG0_on();
    cw_245_tx(true);
    printf(">> Transmitting ... (0 for stop)\n");
    scanf("%d", NULL);
    cw_245_tx(false);
    ledG0_off();
}

/*

*/
void inline cc_regs_read()
{
    cc_command(CC_SIDLE);
    wait_ms(1);
    printf("\t- PATABLE0 = %x", cc_read(PATABLE0));
    printf("\t- MDMCFG0 = %x", cc_read(MDMCFG0));
    printf("\t- MDMCFG1 = %x", cc_read(MDMCFG1));
    printf("\t- MDMCFG2 = %x", cc_read(MDMCFG2));
    printf("\t- FREQ2 = %x", cc_read(FREQ2));
    printf("\t- FREQ1  = %x", cc_read(FREQ1));
    printf("\t- FREQ0 = %x", cc_read(FREQ0));
    printf("\t- DEVIATN = %x", cc_read(DEVIATN));
    printf("\t- MCSM0 = %x", cc_read(MCSM0));
    printf("\t- PKTCTRL0 = %x", cc_read(PKTCTRL0));
    printf("\t- IOCFG0 = %x", cc_read(IOCFG0));
    printf("\t- CHANNR = %x", cc_read(CHANNR));
    cc_sleep();
}

void hk_user()
{
    unsigned int user_input2, user_input3;
    uint8_t user_output;
    while (1)
    {
        memset(input_buffer, '\0', IN_BUFF_SIZE);
        printf(">> Read or write? (r/w; q-quit)\n");
        scanf("%s", input_buffer);
        int ret;
        if (input_buffer[0] == 'q')
        {
            return;
        }
        printf(">> Remote register address?\n");
        scanf("%u", &user_input2);
        printf("%02x\n", user_input2);
        switch (input_buffer[0])
        {
        case 'r':
            ret = hk_read_remotereg((uint8_t)user_input2, &user_output);
            if (ret == TIMEOUT_ERR)
            {
                printf(">>x Timeout reached: aborting session...\n");
                return;
            }
            if (ret == HK_WR_NOT_PERM)
            {
                printf(">>x Error in selecting page: maybe remote HK write disabled\n");
            }
            else
                printf(">> Succesfull readed remote %02x reg: %02x\n", (uint8_t)user_input2, user_output);
            //printf(" - - -  Under developement - - -\n");
            break;
        case 'w':
            printf(">> Data?\n");
            scanf("%x", &user_input3);
            ret = hk_write_remotereg((uint8_t)user_input2, (uint8_t)user_input3, &user_output);
            if (ret == TIMEOUT_ERR)
            {
                printf(">>x Timeout reached: session aborted...\n");
                return;
            }
            if (ret == HK_WR_NOT_PERM)
            {
                printf(">>x %02x register not writed: maybe remote HK write disabled\n", (uint8_t)user_input2);
            }
            else
                printf(">> Succesfull writed remote %02x reg: %02x\n", (uint8_t)user_input2, user_output);
            break;
        default:
            printf(">> No command\n");
            break;
        }
        wait_ms(500);
    }
}

/* Module initializig sequence */
int init_module_sequence()
{
    printf(">>> Initializing module...\n");
    int ret = 0;
    ChipData.err_stat = DUMMY_ERR;
    ledR0_on();
    while (ChipData.err_stat)
    {
        if (ret >= MAX_RETRY)
        {
            break;
        }
        module_init();
        if (ChipData.err_stat)
        {
            printf(">>x Error in initializing module: retry in 3s\n");
            for (int i = 0; i < 6; i++)
            {
                wait_ms(500);
                printf("*");
            }
            printf("\n");
        }
        ret++;
    }
    ledR0_off();
    if (ChipData.err_stat)
    {
        printf(">>x Error in initializing module: MAX_RETRY reached\n");
        printf(">>> Exiting...");
        ledR1_on();
        wait_ms(10);
        return INIT_ERR;
    }
    else
    {
        printf(">>> Module initialized \n");
        ledG1_on();
    }
    return 0;
}

void update_sess_led()
{
    if (ChipData.flags.on_session)
        ledG0_on();
    else
        ledG0_off();
}

int reset_module()
{
    printf(">>> Enabling watchdog, for sleep - wakeup sequence...\n");
    enable_wdog();
    wait_ms(WDOG_TIME_MS + START_UP_TIME_MS);  // wait for watchdog and start-up
    if (abort_link(LOCAL_ONLY) == TIMEOUT_ERR)
        return INIT_ERR;
    printf(">>> Resetting core..\n");
    reset_core_cmd();
    if (init_module_sequence() == INIT_ERR)
        return INIT_ERR;
    /* Initializing CC2500 Transceiver */
    cccs_pulse();
    if (!cc_reset(CC_TX_FREQ2400, PATABLE0_DEF))
    {
        printf(">>> CC2500 transceiver initialized\n\n");
    }
    else
    {
        printf(">>> Error in CC2500 transceiver initialization");
        return INIT_ERR;
    }
    return 0;
}

void send_data_user()
{
    uint16_t n_in_bytes = 0;
    printf("Inserisci:\n");
    memset(input_buffer, 0, IN_BUFF_SIZE);

    //while (getchar() != '\n'){;}
    while (getchar_timeout_us(50000) != PICO_ERROR_TIMEOUT)
    {
        ;
    }
    for (int i = 0; i < IN_BUFF_SIZE - 1; i++)
    { // Prelevamento caratteri dall'interfaccia e inserimento nel buffer
        input_buffer[i] = getchar();
        wait_us(100);
        //scanf("%s",input_buffer);
        input_buffer[IN_BUFF_SIZE - 1] = '\0';
        //printf("%x ",input_buffer[i]);
        if (input_buffer[i] == '\r')
        {
            input_buffer[i] = '\0';
            break;
        }
        n_in_bytes = strlen(input_buffer);
    }
    printf("- input_buffer: %s, n_in_bytes:%d\n", input_buffer, n_in_bytes);
    zl_send_data((uint8_t *)input_buffer, n_in_bytes);
}

void receive_data_user()
{
    printf(">> Receiving data...\n");
    while (getchar_timeout_us(50000) != PICO_ERROR_TIMEOUT)
    {
        ;
    }
    bool first = true;
    while (1)
    {
        memset(ChipData.data_rx.buff, 0, 2 * ChipData.data_rx.bsize * ChipData.tx_max_pack_size);
        if (!wait_for_irq_timeout_ms(REG_IRQ_RAWSTATUS1, IRQ_RXNOTEMPTY, 10))
        {
            zl_receive_data();
            if (first)
            {
                printf("Data received: %s", ChipData.data_rx.buff);
                first = false;
            }
            else
                printf("%s", ChipData.data_rx.buff);
        }
        if (getchar_timeout_us(10000) != PICO_ERROR_TIMEOUT)
            break;
    }
    printf("\n");
}

void emergency_listen_user(uint32_t timeout) {
    uint32_t recvd_imd_transID = 0;
    int ret = 0;
    uint8_t emrg_moduser_loc=0, emrg_channel_loc=0, emrg_max_pack_size_loc=0;
    uint8_t emrg_txbsize_loc=0, emrg_rxbsize_loc=0, emrg_resendtime_loc=0, emrg_initcom_loc=0;
    uint8_t irq_raw1=0;

    switch (emrg_env_on()) {
        case    ALSO_IN_EMRG:
            printf(">>> Also in listen for emergency...\n");
            return;
        case    ALSO_IN_SESSION:
            printf(">>> Also in session can't do anything: abort communication, first\n");
            return;
        case    EMRG_ENV_SET_ERR:
            printf(">>>x Error in changing environment: problem in communication with module, try reset\n");
            return;
        default:
            printf(">>> Entered in emergency context\n");
            break;    
    }
    if (listen_IMD_wu_msg(WU_MSG_ON)) {
        printf(">>>x Error in putting in listen mode: module in indefinite status\n");
        ledR1_on();
        return;
    }

    ret = receive_IMD_transID(&recvd_imd_transID, timeout);

    if (listen_IMD_wu_msg(WU_MSG_OFF)) {
        printf(">>>x Error stopping listen mode: module in indefinite status\n");
        ledR1_on();
        return;
    }
    if (abort_cmd()) {
        printf(">>>x Error in abort command: module in indefinite status\n");
        ledR1_on();
        return;
    }
    /*      Restart resend timer    */
    //zl_write_registers(REG_MAC_RESENDTIME, ChipData.emrg_resend_time);
    flush_rx();
    if (ret) {
        if (emrg_env_off()) {
            printf(">>>x Error in changing environment: module in indefinte status\n");
        }
        printf(">>> Emergency message not received\n");
        return;
    }
    if (recvd_imd_transID == ChipData.IMD_transID) {
        if (start_400_session()) {
            emrg_env_off();
            printf(">>>x Error in starting session\n");
        }
        else {
            printf(">>>> In emergency session with %06x <<<<\n", recvd_imd_transID);
            ledG0_on();
        }
    }
}

void main_program()
{
    char user_input;
    unsigned int user_input2, user_input3;
    uint8_t user_output, user_output2, user_output3;
    uint16_t n_in_bytes;
    uint8_t dst[10];
    uint8_t read_byte, n_byte_to_read;
    uint8_t reg_add;
    uint16_t ave_out = 0, max_out = 0;
    int chan = 0, n_s, t_s;
    int pow_code;
    int sel;

    input_buffer = (char *)malloc(IN_BUFF_SIZE);
    memset(input_buffer, 0, IN_BUFF_SIZE);

    while (1)
    {
        printf("*********************** Inserisci un comando *************************\n");
        printf("'r'-register operation; 'c'-CCA; 'e'-External RSSI Measurement setup;\n");
        printf("'i'-Internal RSSI measurement; 't'-Constant Wave Transmission\n");
        printf("'s'-Start session using 2.45GHz; 'p'-Set communication parameter\n");
        printf("'w'-read CC2500 registers; 'a'-Abort link; 'd'-Data send; h-hk message\n");
        printf("'l'-listen emergency msg; 'x'-reset module\n");
        printf("**********************************************************************\n");
        printf(">");
        update_sess_led();
        //user_input = getchar();
        scanf(" %c", input_buffer);
        user_input = input_buffer[0];
        while (PICO_ERROR_TIMEOUT != getchar_timeout_us(10000))
        {
            ;
        }

        switch (user_input)
        {
        case 'r':
            ledR0_on();
            user_registers_operations();
            ledR0_off();
            break;
        case 'w':
            printf("\n\t\t--->> CC2500 registers read <<---\n");
            ledR0_on();
            cc_regs_read();
            ledR0_off();
            break;
        case 'e':
            printf("\n\t\t--->> EXternal RSSI measurement <<---\n");
            ledR0_on();
            wait_ms(100);
            printf("> channel [0:11]?\n");
            scanf("%d", &chan);
            if (chan >= 0 && chan <= 11)
            {
                set_channel(chan);
            }
            else
                printf(">> Channel value not correct: ignored, taken default\n");
            printf("> number of sample?\n");
            scanf("%d", &n_s);
            printf("> sampling time [ms]?\n");
            scanf("%d", &t_s);
            ext_RSSI_meas(n_s, (uint32_t)t_s, &ave_out, &max_out);
            ledR0_off();
            break;
        case 'i':
            printf("\n\t\t--->> Internal RSSI measurement <<---\n");
            ledR0_on();
            wait_ms(100);
            printf("> channel [0:11]?\n");
            scanf("%d", &chan);
            printf("> number of sample?\n");
            scanf("%d", &n_s);
            printf("> sampling time [ms]?\n");
            scanf("%d", &t_s);
            int_RSSI_meas((uint8_t)chan, (uint8_t)n_s, (uint32_t)t_s, &ave_out, &max_out);
            ledR0_off();
            break;
        case 'c':
            printf("\n\t\t--->> Clear Channel Assestment <<---\n");
            ledR0_on();
            wait_ms(100);
            cca();
            ledR0_off();
            break;
        case 't':
            printf("\n\t\t--->> Constant Wave Transmission <<---\n");
            ledR0_on();
            printf(">> 400MHz or 2.4GHz? (0: 400Mz; 1: 2.4GHz)\n");
            scanf("%d", &sel);
            switch (sel)
            {
            case 0:
                cw_400(chan, pow_code);
                break;
            case 1:
                cw_2400();
                break;
            default:
                printf(">> Not valid command\n");
                break;
            }
            ledR0_off();
            break;
        case 's':
            printf("\n\t\t--->> Start session with 2.45GHz wakeup <<---\n");
            ledR0_on();
            start_session_user();
            ledR0_off();
            break;
        case 'p':
            printf("\n\t\t--->> Set Communication Parameters <<---\n");
            ledR0_on();
            set_transm_param();
            ledR0_off();
            break;
        case 'a':
            printf("\n\t\t--->> Aborting link... <<---\n");
            ledR0_on();
            if (!ChipData.flags.on_session)
            {
                printf(">>> Warning: Not in session, nothing to do\n");
                ledR0_off();
                break;
            }
            abort_link(ALSO_REMOTE);
            //disable_wdog_fast();
            switch (emrg_env_off()) {
                case    ALSO_IN_SESSION:
                    printf(">>>x Error in aborting link x<<< \n ");
                    break;
                case    EMRG_ENV_SET_ERR:
                    printf(">>>x Error in changing environment from emergency x<<< \n ");
                    break;
                default:
                    printf(">>> Link aborted; flushed TX, RX buffers <<< \n ");
                    break;
            }
            ledR0_off();
            ledG0_off();
            break;
        case 'd':
            printf("\n\t\t--->> Data send/receive <<---\n");
            ledR0_on();
            if (!ChipData.flags.on_session)
            {
                printf(">>> Warning: Not in session, nothing to do\n");
                ledR0_off();
                break;
            }
            if (align_TXRX_buff_dimensions(BS_TO_IMD))
            {
                if (align_TXRX_buff_dimensions(IMD_TO_BS))
                {
                    printf(">>> Error in link: session aborted ...\n");
                    ledR0_off();
                    break;
                }
            }
            input_buffer[0] = 0;
            while (1)
            {
                printf(">>> Send or receive data? (s/r; q- quit)\n");
                scanf("%s", input_buffer);
                if (input_buffer[0] == 'q')
                    break;
                switch (input_buffer[0])
                {
                case 's':
                    send_data_user();
                    break;
                case 'r':
                    receive_data_user();
                    break;
                default:
                    printf(">> No valid command <<\n");
                    break;
                }
            }
            ledR0_off();
            break;
        case 'h':
            printf("\n\t\t--->> Housekeeping message <<---\n");
            ledR0_on();
            if (!ChipData.flags.on_session)
            {
                printf(">>> Warning: Not in session, nothing to do\n");
                ledR0_off();
                break;
            }
            hk_user();
            ledR0_off();
            break;
        case 'x':
            printf("\n\t\t--->> RESETTING <<---\n");
            ledR0_on();
            if (reset_module() == INIT_ERR)
            {
                printf(">>> ERROR IN REINITIALIZING MODULE !!!\n");
                break;
            }
            ledR0_off();
            break;
        case 'l':
            printf("\n\t\t--->> Listening for emergency message... <<---\n");
            ledR0_on();
            emergency_listen_user(LISTEN_TIMEOUT);
            ledR0_off();
            break;
        default:
            printf("> No valid command <\n");
            break;
        }
        printf("\n\n");
        wait_ms(500);
    }
}

int main()
{
    pico_init();
    // wait for user action
    scanf("%u", NULL);
    wait_ms(500);

    /* Initializing module */
    if (init_module_sequence() == INIT_ERR)
        return 0;
    /* Initializing CC2500 Transceiver */
    cccs_pulse();
    if (!cc_reset(CC_TX_FREQ2400, PATABLE0_DEF))
    {
        printf(">>> CC2500 transceiver initialized\n\n");
    }
    else
    {
        printf(">>> Error in CC2500 transceiver initialization");
    }

    main_program();

    return 0;
}
