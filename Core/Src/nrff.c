/*
 * nrff.c
 *
 *  Created on: Sep 12, 2021
 *      Author: makso
 */

#include "nrff.h"
#include "nrff_secure.h"

void xtea_encipher(uint32_t *v);
void xtea_decipher(uint32_t *v);

/**
 * encodes 2x32bit block
 */
void xtea_encipher(uint32_t *v) {
    uint32_t i;
    const uint32_t num_rounds = nrff_x.XTEA_ROUNDS;
    const uint32_t *k = nrff_x.XTEA_KEY;
    uint32_t v0=v[0], v1=v[1], sum=0, delta=0x9E3779B9;
    for (i=0; i < num_rounds; i++) {
        v0 += (((v1 << 4) ^ (v1 >> 5)) + v1) ^ (sum + k[sum & 3]);
        sum += delta;
        v1 += (((v0 << 4) ^ (v0 >> 5)) + v0) ^ (sum + k[(sum>>11) & 3]);
    }
    v[0]=v0; v[1]=v1;
}

/**
 * decodes 2x32bit block
 */
void xtea_decipher(uint32_t *v) {
    uint32_t i;
    const uint32_t num_rounds = nrff_x.XTEA_ROUNDS;
    const uint32_t *k = nrff_x.XTEA_KEY;
    uint32_t v0=v[0], v1=v[1], delta=0x9E3779B9, sum=delta*num_rounds;
    for (i=0; i < num_rounds; i++) {
        v1 -= (((v0 << 4) ^ (v0 >> 5)) + v0) ^ (sum + k[(sum>>11) & 3]);
        sum -= delta;
        v0 -= (((v1 << 4) ^ (v1 >> 5)) + v1) ^ (sum + k[sum & 3]);
    }
    v[0]=v0; v[1]=v1;
}
