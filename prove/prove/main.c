#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint-gcc.h>
#include <stdbool.h>
#include <malloc.h>

#define TX_BUFF_BSIZE_DEF   4

#define IN_BUFF_SIZE         50

char *buff;
char *input_buffer;

int main() {
    buff = (char *)malloc((TX_BUFF_BSIZE_DEF+1)*sizeof(char));
    input_buffer = malloc(IN_BUFF_SIZE*sizeof(char));
    memset(input_buffer,0,sizeof(input_buffer));
    memset(buff,0,sizeof(buff));
    volatile bool exit_=false;
    //volatile bool exit_2=false;
    char in[3];

    while(1){
        exit_ = false;
        printf("Inserisci:\n");
        memset(input_buffer,0,sizeof(input_buffer));

        for (int i=0; i<IN_BUFF_SIZE-1; i++) { // Prelevamento caratteri dall'interfaccia e inserimento nel buffer
            scanf("%c",&input_buffer[i]);
            if (input_buffer[i] == '\n') {
                input_buffer[i] == '\0';
                break;
            }
        }

        for(int i=0; i<IN_BUFF_SIZE; i+=TX_BUFF_BSIZE_DEF) {    // copia nel buffer di invio
            for (int j=0; j<TX_BUFF_BSIZE_DEF; j++) {
                buff[j] = input_buffer[i+j];
                if (input_buffer[i+j] == 0) {
                    exit_=true;
                    break;
                }
            }
            printf("%s\n", buff); // da sostituire con send
            if (exit_) {
                break;
            }
        }
    }


    /*while(1){
        memset(input_buffer,0,sizeof(input_buffer));
        scanf("%s",input_buffer);
        for(int i=0; i<IN_BUFF_SIZE; i++) {
            printf("%d\t", input_buffer[i]);
        }
        printf("\n");
    }*/

    return 0;
}





