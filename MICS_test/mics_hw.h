
#ifndef mics_hw_h
#define mics_hw_h

// Macro definitions
#define BIT(p) (1<<p)
#define READ_REG_MASK           BIT(7)
#define MSB_8                   BIT(7)

//Register Definitions
//Converting register address/page to 16bit address
#define reg_pag_to_addr_read(r,p)((r<<8)|(1<<15)|((p-1)<<7)|(1<<6)) // 1|a6|a5|a4|a3|a2|a1|a0|p0|1|0|0|0|0|0|0 (Rd=1, M0=1)
#define reg_pag_to_addr_write(r,p)((r<<8)|((p-1)<<7)) // 0|a6|a5|a4|a3|a2|a1|a0|p0|0|0|0|0|0|0|0  (Rd=0, M0=0)
#define P1_8BIT_ADDR (1<<6)
#define P2_8BIT_ADDR ((1<<7)|(1<<6))
#define reg_pag_to_add(r,p)(r|(((p&0x01)-1)<<7))

/*  -   -   -   -   -   -   PAGE 1 registers definitions -  -   -   -   -   -
    Addresses and values definitions for the first page of memory map.
    Addresses are just the 7-bit register addresses 
*/
#define REG_MAC_TRAINNUM        0x07
#define REG_MAC_COMPANYID       0x16
// IMD transceiver ID registers
#define REG_MAC_IMDTRANSID1     0x13
#define REG_MAC_IMDTRANSID2     0x14
#define REG_MAC_IMDTRANSID3     0x15
// - - - - - - Interrupts, group 1 - - - - - -
// [7]-irq_txempty [6]-irq_txhalfempty [5]-irq_txfull [4]-irq_rxnotempty 
// [3]-irq_hkuserdata [2]-irq_hkuserstatus[1]-irq_clost [0]-irq_hkremotedone
#define REG_IRQ_RAWSTATUS1      0x2E    //interrupt souces before enable masking
#define REG_IRQ_ENABLE1         0x2F    //status of enable status (read only)
#define REG_IRQ_ENABLESET1      0x30    //enable set register     
#define REG_IRQ_ENABLECLEAR1    0x31    //enable clear register  
#define REG_IRQ_STATUS1         0x36    // masked interrup sources
// values
#define IRQ_HKREMOTEDONE        BIT(0)
#define IRQ_CLOST               BIT(1)
#define IRQ_HKUSERSTATUS        BIT(2)
#define IRQ_HKUSERDATA          BIT(3)
#define IRQ_RXNOTEMPTY          BIT(4)
#define IRQ_TXFULL              BIT(5)
#define IRQ_TXHALFEMPTY         BIT(6)
#define IRQ_TXEMPTY             BIT(7)
// - - - - - - Interrupts, group 2 - - - - - - 
// [6]-irq_txbuffoverflow [5]-irq_linkquality [4]-irq_radiofail 
// [3]-irq_radioready [2]-irq_link_ready [1]-irq_hkmessreg [0]-irq_commanddone
#define REG_IRQ_RAWSTATUS2      0x32    //interrupt souces before enable masking
#define REG_IRQ_ENABLE2         0x33    //status of enable status (read only)
#define REG_IRQ_ENABLESET2      0x34    //enable set register
#define REG_IRQ_ENABLECLEAR2    0x35    //enable clear register  
#define REG_IRQ_STATUS2         0x37    // masked interrup sources
// values
#define IRQ_COMMANDONE          BIT(0)
#define IRQ_HKMESSREG           BIT(1)
#define IRQ_LINK_READY          BIT(2)
#define IRQ_RADIOREADY          BIT(3)
#define IRQ_RADIOFAIL           BIT(4)
#define IRQ_LINKQUALITY         BIT(5)
#define IRQ_TXBUFFOVERFLOW      BIT(6)
// - - - - - - Auxiliary interrupts - - - - - - -
// [6]-irq_vregfail [5]-irq_maxretries [4]-irq_maxberr 
// [3]-irq_hkabortlink [2]-irq_wdog [1]-irq_crcerr[0]-irq_synthlockfail
#define REG_IRQ_AUXSTATUS       0x38    //auxiliary interrupts
// values
#define IRQ_SYNTHLOCKFAIL       BIT(0)
#define IRQ_CRCERR              BIT(1)
#define IRQ_WDOG                BIT(2)
#define IRQ_HKABORTLINK         BIT(3)
#define IRQ_MAXBERR             BIT(4)
#define IRQ_MAXRETRIES          BIT(5)
#define IRQ_VREGFAIL            BIT(6)

#define IRQ_ALL                 0xFF
// - - - - - - SPI bus interface control - - - - - - 
#define REG_INTERFACE_MODE      0x2B
#define PAGE_BIT_MASK           BIT(0)
// - - - - - - Main watchdog control - - - - - -
// writing: 0xFF-> clear watchdog; 0x0F-> disable watchdog; 0xF0-> enable watchdog
#define REG_CLEARWDOG           0x1A
#define CLEAR_WDOG              0xFF
#define DISABLE_WDOG            0x0F
#define ENABLE_WDOG             0xF0

// - - - - - - MAC control register - - - - - -
#define REG_MAC_CTRL            0x29
// ------- MAC command --------
#define RX_BUF_FLUSH            BIT(6)  // local RX buffer flush
#define TX_BUF_FLUSH            BIT(5)  // local TX buffer flush
#define SKIP_CRC_CHK            BIT(4)  // skip CRC check
#define CAL_REQ                 BIT(3)  // perform calibration check
#define REG_COPY                BIT(2)  // copy register requiring backup
#define ABORT_LINK              BIT(1)  // Abort link (IMB goes in SLEEP state)
#define LINK_FLUSH              BIT(0)  // Link flush: flushes buffers and sets flush bit in next packet header
// - - - - - - MAC Initiate communication register - - - - - - 
#define REG_MAC_INITCOM         0x2A
#define INITCOM_245             BIT(0)|BIT(2)|BIT(3)    // start session using 2.45GHz wakeup
#define INITCOM_IBS_FL          BIT(3)                  // IBS flag
#define RECEIVE_WU_MSG          BIT(7)|BIT(3)|BIT(1)    // start base station to being monitoring IMD wakeup responses
#define INIT_400_SESS           BIT(3)|BIT(2)   // start 400MHz session
// - - - - - - MAC Channel - - - - - - - - 
#define REG_MAC_CHANNEL         0x19    // MICS 400MHz Channel

//     -   -   - Modulation and user regiter -   -   -   -
#define REG_MAC_MODUSER         0x17    // Set TX and RX modulation
// Values
#define RX_2FSK_FOLDB           0x00                // Set RX listening in 2FSK Foldback
#define RX_2FSK_BARK5           (0x01<<2)|BIT(5)    // Set RX listening in 2FSK 
#define RX_2FSK_BARK11          (0x01<<2)           // Set RX listening in 2FSK Barker11
#define RX_2FSK                 (0x02<<2)           // Set RX listening in 2FSK mode
#define RX_4FSK                 (0x03<<2)           // Set RX listening in 4FSK mode
#define TX_2FSK_FOLDB           0x00                // Set TX listening in 2FSK Foldback
#define TX_2FSK_BARK5           0x01|BIT(4)         // Set TX listening in 2FSK Barker5
#define TX_2FSK_BARK11          0x01                // Set TX listening in 2FSK Barker11
#define TX_2FSK                 0x02                // Set TX listening in 2FSK mode
#define TX_4FSK                 0x03                // Set TX listening in 4FSK mode

#define MODUSER(tx,rx)          tx|rx

// -    -   -   - MAC resend time - -   -   -
#define REG_MAC_RESENDTIME      0x25    //Resend time register, expressed in multiple of BASE_RESENDTIME  
#define BASE_RESENDTIME         427     // [us]
#define RESEND_TIME_DEF         0x17    // default resend time register value
#define DISABLE_RESEND          BIT(7)  // disable resend timer (Base station only)

// -    -   -   - Data buffers registers -  -   -   --
#define REG_RXBUFF_USED         0x01
#define REG_RXBUFF_BSIZE        0x02
#define REG_TXBUFF_USED         0x03
#define REG_TXBUFF_BSIZE         0x04
#define REG_TXBUFF_MAXPACKSIZE  0x05
#define REG_TXRXBUFF            0x2D
/*  block size notable values */
#define TRANSID_RX_BSIZE        3           //block size for receiving transceiver ID
#define TRANSID_SIZE            3
#define IMPRVD_ERR_CORR(p)      p|BIT(4)    //sets also improved error correction

// - - - - - - - Chip and core reset - - - - - - -
#define CORE_RESET_ADDR         0x5E    // reset MAC and RF subsystems. (Wake up block not reset)
#define CORE_RESET_VAL          114     // writing 0b01110010 (decimal 114), reset syncronyzed with 25KHz clock
#define CHIP_RESET_ADDR         0x5F    // full chip reset
#define CHIP_RESET_VAL          82      // writing 0b01010010 (decimal 82), reset syncronyzed with 25KHz clock

/*- -   -   -   - RF general enable and control registers -  -   -   -   -
*/
// ------- RF general enable register -------
#define REG_RF_GENENABLES       0x6E    // RF general enable register
// values
#define RSSI_SN_EN              BIT(5)  // Enable 400MHz RSSI sniffing (for 400MHz wake up)
#define RX_RF_EN                BIT(4)  // Enable receiver RF block
#define RX_IF_EN                BIT(3)  // Enable receiver IF block
#define TX_RF_EN                BIT(2)  // Enable transmitter RF block
#define TX_IF_EN                BIT(1)  // Enable transmitter IF block
#define SYNTH_EN                BIT(0)  // Enable sinthesyzer block

#define RX_MODE_EN        RX_RF_EN|RX_IF_EN|SYNTH_EN    // enables word for RX mode 
#define TX_MODE_EN        TX_RF_EN|TX_IF_EN|SYNTH_EN    // enables word for TX mode
#define RSSI_SNIFF        RSSI_SN_EN|RX_RF_EN|SYNTH_EN  // enables RSSI sniff mode for 400MHz wake up



// ------- RF RX control registers -------
#define REG_RF_RXGENCTRL        0x6A    // RX block control register
// values
#define EXT_RSSI_EN             BIT(6)  // enables IF filter outputs in TESTIO6..3 for external RSSI measurement
#define INT_RSSI_EN             BIT(5)  // enables internal RSSI block
#define RX_IF_XO_EN             BIT(4)  // enables RX IF XO signal 
#define SEL_2_4FSK              BIT(3)  // 0->2FSK receive and 1-bit ADC resolution
                                        // 1->4FSK receive and 2-bits ADC resolution
#define LOOPBK_EN               BIT(2)  // enables loopback of IF I&Q, receive selection
#define NORM_MODE_EN            BIT(1)  // enables normal mode inputs (1-default)
#define DC_REMOV_EN             BIT(0)  // enables DC removal function (1 default)

#define INT_RSSI_RXGENCTRL      INT_RSSI_EN|NORM_MODE_EN|DC_REMOV_EN    // control word for internal RSSI mesurement
#define EXT_RSSI_RXGENCTRL      EXT_RSSI_EN|NORM_MODE_EN|DC_REMOV_EN    // control word for external RSSI mesurement

// - - - - - RF TX control registers - - - - - - - - -
#define REG_RF_TXRFPWRDEFAULTSET    0x63    // adjusts output power
// value
#define TXRFPWRDEFAULTSET_DEF       0x38    // default value


// ------- General pourpose ADC registers -------
#define REG_RF_ADCCTRL          0x74    //RF ADC control register
//values
#define SEL_TESTIO1             (0x00<<2)    // Source selector (bits [4:2])
#define SEL_TESTIO2             (0x01<<2)
#define SEL_TESTIO3             (0x02<<2)
#define SEL_TESTIO4             (0x03<<2)
#define SEL_VSUP                (0x04<<2)
#define SEL_400PEAK             (0x05<<2)
#define SEL_400RSSI             (0x06<<2)
#define SEL_245RSSI             (0x07<<2)

#define ADC_CONV_START          BIT(1)  // ADC conversion start. (enable signal [bit 0] must be set before)
#define ADC_EN                  BIT(0)  // ADC general pourpose enable:should be enabled before starting conversion

#define INT_RSSI_ADCCTRL        SEL_400RSSI|ADC_EN  // ADC control register word for internalRSSI measurement

//ADC output register
#define REG_RF_ADCOUTPUT        0x75    // [5] - conversion completed flag
                                        // [4:0] - Output of ADC corresponding to selected input


/*- -   -   -   - Wakeup block register -   -   -   -   -*/
#define REG_WAKEUP_TESTIOCLAMP  0x5D    // Test I/O clamp control register
// values
#define RF_ON_TESTIO56          (1<<6)  // Enables porting RF subsystem signal on TESTIO5..6
                                        // Requires enable use TESTIO5 and 6
#define EN_USE_TESTIO6          (1<<5)  // Enable use TESTIO6 (disable pull down)
#define EN_USE_TESTIO5          (1<<4)  // Enable use TESTIO5 (disable pull down)
#define EN_USE_TESTIO4          (1<<3)  // Enable use TESTIO4 (disable pull down)
#define EN_USE_TESTIO3          (1<<2)  // Enable use TESTIO3 (disable pull down)
#define EN_USE_TESTIO2          (1<<1)  // Enable use TESTIO2 (disable pull down)
#define EN_USE_TESTIO1          (1<<0)  // Enable use TESTIO1 (disable pull down)
#define TESTIOCLAMP_RSSI        RF_ON_TESTIO56|EN_USE_TESTIO6|EN_USE_TESTIO5    // configure TESTIO56 for external RSSI

/*  -   -   -   - PO registers -    -   -   -   -   -   */
#define REG_PO0                 0x3A    // select source of PO0 signal
#define REG_GPO                 0x3E    // General purpose output register
//values
#define TX245                   BIT(1)  // route 2.45GHz modulation sequence on po0 pin
#define GPO_ON_POx              BIT(0)  // route general pourpose output register on POx pin



#define REG_WAKEUP_TESTIOBUFEN  0x5C    // Buffers on TESTIO6..1 enable register
#define BUF_ON_TESTIO(p)        (1<<(p-1))  // enable buffer on TESTIO p

/* - - - - - Housekeeping message registers - - - - - */
#define REG_HK_TXADDR           0x1C    // housekeeping remote address register (MSB=0 -> write, MSB=1 -> read)
#define REG_HK_TXDATA           0x1D    // housekeeping remote data register
#define REG_HK_TXREPLY          0x1E    // housekeeping remote reply data register
#define REG_HK_USERDATA         0x1F    // housekeeping user data register
#define REG_HK_USERSTATUS       0x20    // housekeeping user status register
#define REG_HK_LASTADDRESS      0x21    // last incoming housekeeping address register (MSB=0->writed,MSB=1->readed)
#define REG_HK_MODE             0x22    // housekeeping mode control register 
                                        // [1]-write enable (enabling remote writing using hk)
                                        // [0]-select remote page to access
#define REM_PAGE_SEL            BIT(0)  // mask for select page of remote register in reg_hk_mode
#define HK_WRITE_EN             BIT(1)  // mask for enable hk write

#define HK_WR_NOT_PERM          -3      // HK remote write not permitted (HK write disabled in remote device)

/*  -   -   -   -   -   -   PAGE 2 registers definitions -  -   -   -   -   -
    Addresses and values definitions for the second page of memory map.
    Addresses are just the 7-bit register addresses + 128. (HEX 0x80).
    For detect the register page, we must check address MSB: 0-> page1, 1-> page2.
    */
#define PAGE2_OFFSET             0x80        // address offset for second page 

/* - - - - - State of the sequencer - - - - - */
#define REG_MAC_TOPSTATE        0x0c+0x80   // Register address
// Register values   
#define START                   0b00000
#define MAC_READY               0b00001
#define MODECHK                 0b00011
#define CRCCHECK                0b00010
#define POP                     0b00110
#define RADIOSETUP              0b00111
#define TRIM                    0b00101
#define TRIMDONE                0b00100
#define BASE_IDLE               0b01100
#define BICHK                   0b01101
#define TXWUSTART               0b01110
#define TXWUDONE                0b01010
#define RXWUSTART               0b01011
#define RXWUDONE                0b01111
#define WUCHK                   0b01001
#define TXSTART                 0b11100
#define TXDONE                  0b11101
#define RXSTART                 0b11111
#define RXDONE                  0b11110
#define TXRXCHK                 0b11000
#define ABORTSTART              0b10100
#define FINALSTATE              0b10000

/* -    -   -   - MAC registers -   -   -   -   -*/
#define REG_MAC_TXIFFREQ         0x3E + PAGE2_OFFSET //disconnect and set IF frequency from MAC
//value
#define TXIFFREQ_CW_WORD        0x1C    // value for constant wave transmission

/*          Useful numbers              */
#define MAX_CHNLS               10   // number of MICS band channels

/*          Auxiliary values            */ 
#define AUXVAL1                 0x1a

/*          Error codes                 */
#define DUMMY_ERR               0xFF
#define ERR_WRITEREG            1
#define ERR_READREG             2

/*          Acquisition time        */
#define CCA_ACQ_TIME            11  //[ms], acquisition time per channel, used in cca

/*          Timeouts                */
#define CMD_TIMEOUTE            5000            // timeout for executing command
#define ABORT_TIMEOUT           CMD_TIMEOUTE    // timeout for aborting
#define FLUSH_TIMEOUT           CMD_TIMEOUTE    // timeout for flush command

/*              Direction of syncrhonization                */
#define BS_TO_IMD               true
#define IMD_TO_BS               false

/*  Enables and disables synthesizer and Receiver RF block 
    */
#define rx_synthrf_en()         zl_write_registers(REG_RF_GENENABLES, RX_MODE_EN, 1)
#define rx_synthrf_dis()        zl_write_registers(REG_RF_GENENABLES, 0, 1)

/*              Error defines           */
#define EMRG_ENV_SET_ERR        -1
#define IMD_LISTEN_ERR          -2
#define NOT_IN_EMRG             -3
#define ALSO_IN_EMRG            -4

/*          Structs and typedef         */
typedef struct {
    bool on_session;
    bool emergency_on;
} FLAGS_T;

typedef struct {
    uint8_t *buff;          // TX, RX buffers
    uint8_t bsize;          // Transmission buffer block size
    uint8_t emrg_bsize;     // Transmission buffer block size in emergency session
    uint8_t used_blocks;    // Receive buffer block size
} DATA_BUFF_T;

typedef struct {
    uint8_t page;                   //current page
    uint8_t interf_mode;            //current interface mode register value
    bool wdog_en;                   //watchdog enabling status
    uint8_t err_stat;               //last error code
    uint8_t channel;                //current channel
    uint8_t emrg_channel;           //emergency predefined channel
    uint8_t power_code;             //power code for reg txrfpwrdefaultset
    uint8_t resend_time;            //value of resend time register
    uint8_t emrg_resend_time;       //value of resend time register in emergency session 
    uint32_t IMD_transID;           //current IMD transceiver ID
    uint8_t company_ID;             // Company ID
    uint8_t moduser;                // moduser register
    uint8_t emrg_moduser;           // moduser register in emergency session
    FLAGS_T flags;                  // General status flags
    uint8_t tx_max_pack_size;       // maximum package size in blocks (1-31)
    uint8_t emrg_tx_max_pack_size;  // maximum package size in blocks in emergency session (1-31)
    DATA_BUFF_T data_tx; 
    DATA_BUFF_T data_rx;
} CHIP_DATA;

extern CHIP_DATA ChipData;

//extern inline void set_page(uint8_t page);
extern uint8_t zl_read_registers(uint8_t addr, uint8_t *data);
extern uint8_t zl_write_registers(uint8_t addr, uint8_t data);
extern void set_bit_in_reg(uint8_t addr, uint8_t mask);
extern void clear_bit_in_reg(uint8_t addr, uint8_t mask);
extern void clear_irq(uint8_t irq_reg, uint8_t irq_mask);

extern bool disable_wdog();
extern void disable_wdog_fast();
extern void enable_wdog();
extern void set_channel(uint8_t chnl);
extern void set_power(uint8_t pw_code);
extern void set_moduser(uint8_t tx, uint8_t rx);
extern void set_IMD_transID(uint32_t imd_ID);
extern void set_company_ID(uint8_t comp_ID);
extern void set_resend_time(uint8_t resnd_time);
extern void set_initcom_245(bool wake);
extern void wait_for_irq(uint8_t irq_reg, uint8_t irq_mask);
extern int wait_for_irq_timeout_ms(uint8_t irq_reg, uint8_t irq_mask, uint32_t timeout_ms);
extern int hk_write_remotereg(uint8_t hk_addr, uint8_t hk_data, uint8_t *hk_reply);
extern int hk_read_remotereg(uint8_t hk_addr, uint8_t *hk_reply);
extern void hk_remote_abort();
extern int align_TXRX_buff_dimensions(bool bs_to_imd);
extern int emrg_env_on();
extern int emrg_env_off();
extern int listen_IMD_wu_msg (bool enable);
extern int abort_cmd();
extern int flush_rx();
extern void reset_core_cmd();

#endif