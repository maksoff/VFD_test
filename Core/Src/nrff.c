/*
 * nrff.c
 *
 *  Created on: Sep 12, 2021
 *      Author: makso
 */

#include "nrff.h"
#include "nrff_secure.h"

#include "nrf24l01p.h"

void xxtea_encipher(uint32_t *v, uint8_t n);
void xxtea_decipher(uint32_t *v, uint8_t n);
const uint32_t DELTA = 0x9e3779b9;

/**
 * encodes 32bit blocks (min 2x)
 */
void xxtea_encipher(uint32_t *v, uint8_t n)
{
	uint32_t y, z, sum;
	uint32_t p, rounds, e;
	rounds = 6 + 52/n;
	sum = 0;
	z = v[n-1];
	do {
		sum += DELTA;
		e = (sum >> 2) & 3;
		for (p=0; p<n-1; p++) {
			y = v[p+1];
			z = v[p] += (((z>>5^y<<2) + (y>>3^z<<4)) ^ ((sum^y) + (key[(p&3)^e] ^ z)));
		}
		y = v[0];
		z = v[n-1] += (((z>>5^y<<2) + (y>>3^z<<4)) ^ ((sum^y) + (key[(p&3)^e] ^ z)));
	} while (--rounds);
}

/**
 * decodes 32bit blocks (min 2x)
 */
void xxtea_decipher(uint32_t *v, uint8_t n)
{
	uint32_t y, z, sum;
	uint32_t p, rounds, e;
	rounds = 6 + 52/n;
	sum = rounds*DELTA;
	y = v[0];
	do {
		e = (sum >> 2) & 3;
		for (p=n-1; p>0; p--) {
			z = v[p-1];
			y = v[p] -= (((z>>5^y<<2) + (y>>3^z<<4)) ^ ((sum^y) + (key[(p&3)^e] ^ z)));
		}
		z = v[n-1];
		y = v[0] -= (((z>>5^y<<2) + (y>>3^z<<4)) ^ ((sum^y) + (key[(p&3)^e] ^ z)));
		sum -= DELTA;
	} while (--rounds);
}


