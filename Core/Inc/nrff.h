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

#define MSG_CNT(x) 	((x & 0xF000)>>12) /*< Macro to get the message count */
#define MSG_ID(x)	(x & 0x0FFF) 	   /*< Macro to get the message id99- */

#endif /* INC_NRFF_H_ */
