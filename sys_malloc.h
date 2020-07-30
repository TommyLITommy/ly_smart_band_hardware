#ifndef __SYS_MALLOC_H__
#define __SYS_MALLOC_H__

#include "sys_info.h"

#define MEMORY_TYPE_00_BLOCK_COUNT 10
#define MEMORY_TYPE_01_BLOCK_COUNT 5
#define MEMORY_TYPE_02_BLOCK_COUNT 5
#define MEMORY_TYPE_03_BLOCK_COUNT 5
#define MEMORY_TYPE_04_BLOCK_COUNT 5
#define MEMORY_TYPE_05_BLOCK_COUNT 2

#define MEMORY_TYPE_00_BLOCK_SIZE 8
#define MEMORY_TYPE_01_BLOCK_SIZE 16
#define MEMORY_TYPE_02_BLOCK_SIZE 32
#define MEMORY_TYPE_03_BLOCK_SIZE 64
#define MEMORY_TYPE_04_BLOCK_SIZE 128
#define MEMORY_TYPE_05_BLOCK_SIZE 512

typedef enum
{
	MEMORY_USAGE_NONE = 0,
	MEMORY_USAGE_BLE_RX,
	MEMORY_USAGE_BLE_TX,
	MEMORY_USAGE_UART_RX,
	MEMORY_USAGE_UART_TX,
	MEMORY_USAGE_MAX,
}memory_usage_t;

typedef enum
{
	SYS_MALLOC_MEMORY_TYPE_00 = 0,
	SYS_MALLOC_MEMORY_TYPE_01,
	SYS_MALLOC_MEMORY_TYPE_02,
	SYS_MALLOC_MEMORY_TYPE_03,
	SYS_MALLOC_MEMORY_TYPE_04,
    SYS_MALLOC_MEMORY_TYPE_05,
	SYS_MALLOC_MEMORY_TYPE_MAX,
	SYS_MALLOC_MEMORY_TYPE_NONE = 0xFF
}memory_type_t;

struct memory_control_block
{
	uint8_t memory_type;
	uint8_t memory_usage;
	uint8_t *p_start_addr;
	uint16_t call_line_num;
	uint32_t create_timestamp;//Just For Test, 
	struct memory_control_block *prev;
	struct memory_control_block *next;
};

typedef struct memory_control_block memory_control_block_t;

typedef struct
{
	sys_info_t *p_sys_info;
}sys_malloc_control_block_t;

extern void sys_malloc_print(void);
extern void sys_malloc_debug(void);
extern uint8_t *sys_malloc_apply(uint16_t size, uint8_t usage, uint16_t line_num);
extern void sys_malloc_free(uint8_t *p_start_addr);
extern void sys_malloc_init(sys_info_t *p_sys_info);

#endif

