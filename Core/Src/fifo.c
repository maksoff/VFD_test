/*
 * fifo.c
 *
 *  Created on: Oct 27, 2018
 *      Author: makso
 */

#include "fifo.h"

buff_t fifo_buffer[BUFF_SIZE];
uint32_t head_index = 0;
uint32_t tail_index = 0;

bool fifo_push(buff_t data)
{
	if (fifo_length() >= BUFF_SIZE - 1)
		return 1; // too much data!
	if (++tail_index == BUFF_SIZE)
		tail_index = 0;
	fifo_buffer[tail_index] = data;
	return 0;
}

buff_t fifo_pop(void)
{
	if (fifo_is_empty())
		return fifo_buffer[head_index]; // buffer is empty, return the last value
	if (++head_index == BUFF_SIZE)
		head_index = 0;
	return fifo_buffer[head_index];
}

bool fifo_is_empty(void)
{
	return head_index == tail_index;
}

uint32_t fifo_length(void)
{
	if (tail_index >= head_index)
		return tail_index - head_index;
	else
		return tail_index + BUFF_SIZE - head_index;
}

