#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <malloc.h>


#define IN_BUFF_SIZE         50

#define TX_BUFF_BSIZE_DEF       4
#define RX_BUFF_BSIZE_DEF       2
#define TX_BUFF_MAXPACK_SIZE    5

#define REG_TXRXBUFF        55

typedef struct {
    uint8_t *buff;     // TX, RX buffers
    uint8_t bsize;          // Transmission buffer block size
    uint8_t used_blocks;          // Receive buffer block size
} DATA_BUFF_T;

typedef struct {
    uint8_t page;           //current page
    uint8_t interf_mode;    //current interface mode register value
    bool wdog_en;           //watchdog enabling status
    uint8_t err_stat;       //last error code
    uint8_t channel;        //current channel
    uint8_t power_code;     //power code for reg txrfpwrdefaultset
    uint8_t resend_time;    //value of resend time register
    uint32_t IMD_transID;   //current IMD transceiver ID
    uint8_t company_ID;     // Company ID
    uint8_t moduser;        // moduser register.
    //FLAGS_T flags;            // General status flags
    uint8_t tx_max_pack_size;
    DATA_BUFF_T data_tx;
    DATA_BUFF_T data_rx;
} CHIP_DATA;

CHIP_DATA ChipData;
char *input_buffer;

void mult_write_registers (uint8_t reg, uint8_t *buffer, uint8_t n_bytes){
    char internal_buffer[TX_BUFF_BSIZE_DEF*TX_BUFF_MAXPACK_SIZE + 1];
    memset(internal_buffer,'\0', sizeof(internal_buffer));
    strncpy(internal_buffer, (char *)buffer, n_bytes);
    for (int i=0; i<n_bytes; i++) {
        printf("%02x ",internal_buffer[i]);
    }
    printf("\n");
}

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
    /* Inserting data in transmission buffer */
    while ((( *start_index + i) < n_data) && (i < ChipData.tx_max_pack_size*ChipData.data_tx.bsize)) { // Stop when all data are pushed
        ChipData.data_tx.buff[ChipData.data_tx.used_blocks*ChipData.data_tx.bsize + i] = data_buffer[*start_index + i];    // or transmission buffer is full
        i++;
    }

    *start_index += i;  // updating start_index
    /* Updating used_blocks  */
    ChipData.data_tx.used_blocks = (i % ChipData.data_tx.bsize) ? i/ChipData.data_tx.bsize + 1 : i/ChipData.data_tx.bsize;
    /* If the last wrote block is not full, fill remaining bytes with 0 */
    if (((i) % ChipData.data_tx.bsize)) {
        memset(&ChipData.data_tx.buff[(ChipData.data_tx.used_blocks-1)*ChipData.data_tx.bsize + i%ChipData.data_tx.bsize], 0, i%ChipData.data_tx.bsize);
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
    memset(ChipData.data_tx.buff, 0, sizeof(ChipData.data_tx.buff));
    /* Push data in the software transmission buffer and then send it to zl transmission buffer register.
        Cycle goes until all data are delivered.
    */
    while (curr_index < n_data) {
        tx_buff_data_push(data, &curr_index, n_data);
        mult_write_registers(REG_TXRXBUFF, ChipData.data_tx.buff, ChipData.data_tx.bsize*ChipData.data_tx.used_blocks);
        ChipData.data_tx.used_blocks = 0;
    }
}


int main() {
    uint16_t n_in_bytes;
    /* Initialize chip data and vector */
    memset(&ChipData,0,sizeof(ChipData));
    input_buffer = (char *) malloc(IN_BUFF_SIZE);
    /* Initialize ChipData transmission parameter */
    ChipData.tx_max_pack_size = TX_BUFF_MAXPACK_SIZE;
    ChipData.data_tx.bsize = TX_BUFF_BSIZE_DEF;
    ChipData.data_tx.buff = malloc(ChipData.data_tx.bsize * ChipData.tx_max_pack_size);


    while(1){
        n_in_bytes = 0;
        printf("Inserisci:\n");
        memset(input_buffer,0,IN_BUFF_SIZE);

        for (int i=0; i<IN_BUFF_SIZE-1; i++) { // Prelevamento caratteri dall'interfaccia e inserimento nel buffer
            //scanf("%c",&input_buffer[i]);
            input_buffer[i] = getchar();
            if (input_buffer[i] == '\n') {
                input_buffer[i] = '\0';
                break;
            }
            n_in_bytes++;
        }
        printf("- input_buffer: %s, n_in_bytes:%d\n",input_buffer,n_in_bytes);
        zl_send_data((uint8_t *) input_buffer, n_in_bytes );
    }
    return 0;
}







