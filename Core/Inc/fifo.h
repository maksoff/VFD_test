/*
 * fifo.h
 *
 *  Created on: Oct 27, 2018
 *      Author: makso
 */

#ifndef FIFO_H_
#define FIFO_H_

#include "main.h"

#define BUFF_SIZE (256)

typedef uint8_t buff_t;

bool fifo_is_empty(void);
uint32_t fifo_length(void);
bool fifo_push(buff_t data);
buff_t fifo_pop(void);



#endif /* FIFO_H_ */
