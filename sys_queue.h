#ifndef __SYS_QUEUE_H__
#define __SYS_QUEUE_H__

#define ELEMENT_QUEUE_SIZE 32//This define must be the 2's pow
#define ELEMENT_QUEUE_SIZE_MASK (ELEMENT_QUEUE_SIZE - 1)

typedef enum
{
	ELEMENT_TYPE_NONE = 0,
	ELEMENT_TYPE_APP,
	ELEMENT_TYPE_WECHAT,
	ELEMENT_TYPE_UART,
}element_type_t;

typedef struct
{
	uint8_t element_type;
	uint8_t *p_buffer;
	uint16_t buffer_length;
}element_t;

typedef struct
{
	element_t element_queue[ELEMENT_QUEUE_SIZE];
	uint8_t element_queue_head;
	uint8_t element_queue_tail;
}sys_queue_t;

extern uint32_t sys_queue_put(sys_queue_t *p_sys_queue, uint8_t element_type, uint8_t *p_buffer, uint16_t buffer_length);
extern uint32_t sys_queue_get(sys_queue_t *p_sys_queue, uint8_t *p_element_type, uint8_t **p_p_buffer, uint16_t *p_buffer_length);
extern uint32_t sys_queue_length_get(sys_queue_t *p_sys_queue);
extern void sys_queue_clean(sys_queue_t *p_sys_queue);
extern void sys_queue_init(sys_queue_t *p_sys_queue);

#endif

