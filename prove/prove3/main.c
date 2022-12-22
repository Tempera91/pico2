
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

//* *   *   *   *   *   *   *   Defines  *  *   *   *   *   *   *
#define BIG                     true
#define LITTLE                  false

uint16_t vect_to_wrd16(uint8_t *vect, bool endianess){
    uint16_t var = 0;
    if (endianess) {            // BIG_ENDIAN
        var = vect[0] << 8;     // var = |  vect[0] |00000000|
        var = var | vect[1];    // var = |  vect[0] |  vect[1] |
    }
    else {
        var = vect[1] << 8;     // var = |  vect[1] |00000000|
        var = var | vect[0];    // var = |  vect[1] |  vect[0] |
    }
    return var;
}

uint32_t vect_to_wrd32(uint8_t *vect, bool endianess){
    uint32_t var = 0;
    if (endianess) {                    // Big Endian
        var = vect[0] << 24;            // var = |  vect[0] |   0x00   |   0x00   |   0x00   |
        var = var | (vect[1] << 16);    // var = |  vect[0] |  vect[1] |   0x00   |   0x00   |
        var = var | (vect[2] << 8);     // var = |  vect[0] |  vect[1] |  vect[2] |   0x00   |
        var = var | vect[3];            // var = |  vect[0] |  vect[1] |  vect[2] |  vect[3] |
    }
    else {                              // Little Endian
        var = vect[3] << 24;            // var = |  vect[3] |   0x00   |   0x00   |   0x00   |
        var = var | (vect[2] << 16);    // var = |  vect[3] |  vect[2] |   0x00   |   0x00   |
        var = var | (vect[1] << 8);     // var = |  vect[3] |  vect[2] |  vect[1] |   0x00   |
        var = var | vect[0];            // var = |  vect[3] |  vect[2] |  vect[1] |  vect[0] |
    }
    return var;
}

void wrd16_to_vect(uint16_t wrd, uint8_t *vect, bool endianess){
    if (endianess) {                    // Big Endian
        vect[1] = wrd & 0x00FF;
        vect[0] = (wrd & 0xFF00) >> 8;
    }
    else {                              // Little Endian
        vect[0] = wrd & 0x00FF;
        vect[1] = (wrd & 0xFF00) >> 8;
    }
}

void wrd32_to_vect(uint32_t wrd, uint8_t *vect, bool endianess){
    if (endianess) {                    // Big Endian
        vect[3] = wrd & 0xFF;
        vect[2] = (wrd & (0xFF << 8)) >> 8;
        vect[1] = (wrd & (0xFF << 16)) >> 16;
        vect[0] = (wrd & (0xFF << 24)) >> 24;
    }
    else {                              // Little Endian
        vect[0] = wrd & 0xFF;
        vect[1] = (wrd & (0xFF << 8)) >> 8;
        vect[2] = (wrd & (0xFF << 16)) >> 16;
        vect[3] = (wrd & (0xFF << 24)) >> 24;
    }
}

int main(/*int argc, char *argv[]*/) {
    uint8_t data_vect[4];
    uint32_t word;
    float in;
    unsigned int out;

    while (1) {
        printf(">> Inserire numero float\n");
        scanf("%f",&in);
        out = (unsigned int) in;
        printf(">> float: %f; int: %d\n", out, (int)out);
    }

}
