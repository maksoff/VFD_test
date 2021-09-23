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
uint32_t nrff_get_tick(void);

/**
 * user must define this function. used as callback for received packet
 */
void nrff_process(uint8_t * data, uint8_t size);

/* run this function periodically to test the data */
void nrff_update(void);

// nrf_init - address/type (server, always on, etc)/configure interrupts
int nrff_publish(uint16_t message_id, uint8_t * data, uint8_t size);
int nrff_request(uint16_t message_id);
int nrff_request_n(uint16_t* message_ids, uint16_t count);


#define MSG_CNT(x) 	((x & 0xF000)>>12) /*< Macro to get the message count */
#define MSG_ID(x)	(x & 0x0FFF) 	   /*< Macro to get the message id99- */

#endif /* INC_NRFF_H_ */
