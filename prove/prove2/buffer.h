#ifndef BUFFER_INCLUDED
#define BUFFER_INCLUDED

#define TX_BUFF_BSIZE_DEF       4
#define TX_BUFF_MAXBLOCKSIZE    31

#define FIFO_TX_SIZE            TX_BUFF_BSIZE_DEF*TX_BUFF_MAXBLOCKSIZE

typedef struct {
    uint8_t *buffer;
    int i_in, i_out;
    int used;
    int dim;
} FIFO_BUF_T;



#endif // BUFFER_INCLUDED
