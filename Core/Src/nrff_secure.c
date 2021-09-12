/*
 * nrff_secure.c
 *
 *  Created on: Sep 12, 2021
 *      Author: makso
 */

#include "nrff_secure.h"

#warning never publish this file online! (e.g. git)
// this file is used only for example

const NRFF_X nrff_x = {
		.CH = 111, 	// Radio Channel number, 0...125
		.ADDRESS = { 0x31, 0x41, 0x59, 0x26, 0x56 },
		.XTEA_KEY = {0x00112233, 0x44556677, 0x8899AABB, 0xCCDDEEFF}, // keep in secret!
		.XTEA_ROUNDS = 32 // more than 32 recommended
};
