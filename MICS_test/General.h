/* */ 
#ifndef general_h
#define general_h

#define TX_BUFF_BSIZE_DEF       4
#define RX_BUFF_BSIZE_DEF       4
#define TX_BUFF_MAXPACK_SIZE    31

#define IN_BUFF_SIZE            100  //input buffer dimension

/*      Time defines            */
#define START_SESS_TIMEOUT      10000   // timeout (ms) for starting session
#define HK_TIMEOUT              10000   // timeout (ms) for hk message reply
#define WDOG_TIME_MS            5000    // global watchdog time (ms)
#define START_UP_TIME_MS        100     // start-up time (from SLEEP to CHECK COMMAND IDLE)

#define LISTEN_TIMEOUT          20000   // waiting imd responses timeout

/*      Error Value defines     */
#define ADDR_ERR                -1
#define TIMEOUT_ERR             -2
#define INIT_ERR                -1
#define NOT_IN_SESSION          -4
#define ALSO_IN_SESSION         -5

/*      Emergency default defines       */
#define EMRG_CHANNEL            1
#define EMRG_TX_BSIZE           5
#define EMRG_RX_BSIZE           5
#define EMRG_TX_MAXPACKSIZE     31
#define EMRG_RESENDTIME         0x17

#define EMRG_ON                 true
#define EMRG_OFF                false

#define WU_MSG_ON               true
#define WU_MSG_OFF              false

#define BYTE_BIT_SPAN                  8

#endif