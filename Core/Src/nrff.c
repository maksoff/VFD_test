/*
 * nrff.c
 *
 *  Created on: Sep 12, 2021
 *      Author: makso
 */

#include "nrff.h"
#include "nrff_secure.h"
#include "nrf24l01p.h"

const uint8_t version = 0x01; // first version

void xxtea_encipher(uint32_t *v, uint8_t n);
void xxtea_decipher(uint32_t *v, uint8_t n);
const uint32_t DELTA = 0x9e3779b9;

typedef struct  {
	uint8_t  version;	// protocol version
	uint8_t  dir_size;	// DONW | SERVICE | MULTIPACKET | DATA_SIZE[5]
	uint16_t msg_cnt_id;// COUNTER[4] | MESSAGE_ID[12]
	uint32_t nonce; 	// 32 bits
	uint8_t  data[24];	// 24 bytes max in one packet (total packet size 32 bytes)
} NRFF_PACKET;

#define DS_DOWN(x) 		  (x & (1<<7)) /*< Is packet going "down" the tree */
#define DS_SERVICE(x) 	  (x & (1<<6)) /*< This message used for library purpose */
#define DS_MULTIPACKET(x) (x & (1<<5)) /*< This message is part of multipacket message */
#define DS_SIZE(x) 		  (x & 0b11111) /*< Number of bytes in packet */


/********* nRFF user functions *******/

/**
 * sends multiple requests for the packet
 */
// is this really good idea? what if at this time server will transmit something?
int nrff_request_n(uint16_t* message_ids, uint16_t count)
{
	int error = 0;
	for (int i = 0; i < count; i++)
	{
		error = nrff_request(message_ids[count]);
		if (error != 0)
		{
			return error;
		}
	}
	return error;
}


/*********** SECURITY ***************/

/**
 * encodes 32bit blocks (min 2x)
 */
void xxtea_encipher(uint32_t *v, uint8_t n)
{
	const uint32_t *key = nrff_x.KEY;
	uint32_t y, z, sum;
	uint32_t p, rounds, e;
	rounds = 6 + 52/n; // if more than 212 bytes, increase rounds to 8
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
	const uint32_t *key = nrff_x.KEY;
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

/***** this functions should be redefined ******/

__weak void nrff_process(uint8_t * data, uint8_t size)
{
	// to avoid warnings
	(void*)data;
	(void)size;
}
