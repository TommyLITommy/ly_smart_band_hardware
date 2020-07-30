#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "nrf_error.h"
#include "sys_queue.h"
#include "sys_malloc.h"

//#define BUFFER_FULL_DISCARD_OLD
uint32_t sys_queue_put(sys_queue_t *p_sys_queue, uint8_t element_type, uint8_t *p_buffer, uint16_t buffer_length)
{
	uint8_t temp = p_sys_queue->element_queue_head;
	
	#ifndef BUFFER_FULL_DISCARD_OLD
	if(((temp + 1) & ELEMENT_QUEUE_SIZE_MASK) == p_sys_queue->element_queue_tail)
	{
		return NRF_ERROR_NO_MEM;
	}		
	#endif
	
	p_sys_queue->element_queue[temp].element_type   = element_type;
	p_sys_queue->element_queue[temp].p_buffer 	    = p_buffer;
	p_sys_queue->element_queue[temp].buffer_length  = buffer_length; 
	
	temp = (temp + 1) & ELEMENT_QUEUE_SIZE_MASK;
	p_sys_queue->element_queue_head = temp;
	
	return NRF_SUCCESS;
}

uint32_t sys_queue_get(sys_queue_t *p_sys_queue, uint8_t *p_element_type, uint8_t **p_p_buffer, uint16_t *p_buffer_length)
{
	if(p_sys_queue->element_queue_tail == p_sys_queue->element_queue_head)
	{
		return NRF_ERROR_NOT_FOUND;
	}
	
	*p_element_type  = p_sys_queue->element_queue[p_sys_queue->element_queue_tail].element_type;
	*p_p_buffer 	 = p_sys_queue->element_queue[p_sys_queue->element_queue_tail].p_buffer;
	*p_buffer_length = p_sys_queue->element_queue[p_sys_queue->element_queue_tail].buffer_length;
	
    p_sys_queue->element_queue_tail = (p_sys_queue->element_queue_tail + 1) & ELEMENT_QUEUE_SIZE_MASK;
    
	return NRF_SUCCESS;
}

//critical aera
uint32_t sys_queue_length_get(sys_queue_t *p_sys_queue)
{
	uint32_t length = 0;
	if(p_sys_queue->element_queue_head >= p_sys_queue->element_queue_tail)
	{
		length = p_sys_queue->element_queue_head - p_sys_queue->element_queue_tail;
	}
	else
	{
		length = ELEMENT_QUEUE_SIZE - (p_sys_queue->element_queue_tail - p_sys_queue->element_queue_head);
	}
	return length;
}

void sys_queue_clean(sys_queue_t *p_sys_queue)
{
	while(p_sys_queue->element_queue_tail != p_sys_queue->element_queue_head)
	{
		if(p_sys_queue->element_queue[p_sys_queue->element_queue_tail].p_buffer != NULL)
		{
			sys_malloc_free(p_sys_queue->element_queue[p_sys_queue->element_queue_tail].p_buffer);
			p_sys_queue->element_queue[p_sys_queue->element_queue_tail].p_buffer = NULL;
		}

		p_sys_queue->element_queue_tail  = (p_sys_queue->element_queue_tail + 1) & ELEMENT_QUEUE_SIZE_MASK;
	}
}

void sys_queue_init(sys_queue_t *p_sys_queue)
{
	p_sys_queue->element_queue_head = 0;
	p_sys_queue->element_queue_tail = 0;
}

