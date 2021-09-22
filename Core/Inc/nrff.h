/*
 * nrff.h
 *
 *  Created on: Sep 12, 2021
 *      Author: makso
 */

#ifndef INC_NRFF_H_
#define INC_NRFF_H_

#include "main.h"

/**
 * user must define this function. Recommended to use sys timer
 */
uint32_t nrff_get_nonce(void);

#endif /* INC_NRFF_H_ */
