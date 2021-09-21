/*
 * nrff_secure.h
 *
 *  Created on: Sep 12, 2021
 *      Author: makso
 */

#ifndef INC_NRFF_SECURE_H_
#define INC_NRFF_SECURE_H_

#include "main.h"

/**
 * prototype for security constants
 */
typedef struct  {
    const uint8_t  CH;			// Radio Channel number, 0...125
    const uint8_t  ADDRESS[5];	// Default address
    const uint32_t KEY[4]; // key 4x32 bit  <- keep this really in secret!
} NRFF_X;

/**
 * define this in separate file, don't publish it online!
 * e.g. NRFF_X nrff_x = {.CH=111, ...
 */
extern const NRFF_X nrff_x;

#endif /* INC_NRFF_SECURE_H_ */
