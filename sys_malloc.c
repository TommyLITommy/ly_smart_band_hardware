#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "nrf_error.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "sys_malloc.h"

static sys_malloc_control_block_t smcb;

const char *memory_usage_string[MEMORY_USAGE_MAX] =
{
	"[NO_USE]",
	"[BLE_RX]",
	"[BLE_TX]",
	"[UART_RX]",
	"[UART_TX]",
};

static uint8_t memory_type_01_memory_pool[MEMORY_TYPE_00_BLOCK_COUNT][MEMORY_TYPE_00_BLOCK_SIZE];
static uint8_t memory_type_02_memory_pool[MEMORY_TYPE_01_BLOCK_COUNT][MEMORY_TYPE_01_BLOCK_SIZE];
static uint8_t memory_type_03_memory_pool[MEMORY_TYPE_02_BLOCK_COUNT][MEMORY_TYPE_02_BLOCK_SIZE];
static uint8_t memory_type_04_memory_pool[MEMORY_TYPE_03_BLOCK_COUNT][MEMORY_TYPE_03_BLOCK_SIZE];
static uint8_t memory_type_05_memory_pool[MEMORY_TYPE_04_BLOCK_COUNT][MEMORY_TYPE_04_BLOCK_SIZE];
static uint8_t memory_type_06_memory_pool[MEMORY_TYPE_05_BLOCK_COUNT][MEMORY_TYPE_05_BLOCK_SIZE];

static memory_control_block_t memory_type_00_mcb_array[MEMORY_TYPE_00_BLOCK_COUNT];
static memory_control_block_t memory_type_01_mcb_array[MEMORY_TYPE_01_BLOCK_COUNT];
static memory_control_block_t memory_type_02_mcb_array[MEMORY_TYPE_02_BLOCK_COUNT];
static memory_control_block_t memory_type_03_mcb_array[MEMORY_TYPE_03_BLOCK_COUNT];
static memory_control_block_t memory_type_04_mcb_array[MEMORY_TYPE_04_BLOCK_COUNT];
static memory_control_block_t memory_type_05_mcb_array[MEMORY_TYPE_05_BLOCK_COUNT];

static memory_control_block_t *free_memory_control_block_head_array[SYS_MALLOC_MEMORY_TYPE_MAX];

static memory_control_block_t *busy_memory_control_block_list_head = NULL;

static uint16_t memory_size[SYS_MALLOC_MEMORY_TYPE_MAX] = 
{
	MEMORY_TYPE_00_BLOCK_SIZE,
	MEMORY_TYPE_01_BLOCK_SIZE,
	MEMORY_TYPE_02_BLOCK_SIZE,
	MEMORY_TYPE_03_BLOCK_SIZE,
	MEMORY_TYPE_04_BLOCK_SIZE,
    MEMORY_TYPE_05_BLOCK_SIZE
};

/*
static uint8_t sys_malloc_find_memory_type_by_size(uint16_t size)
{	
	for(uint8_t i = 0; i < SYS_MALLOC_MEMORY_TYPE_MAX; i++)
	{
		if(size <= memory_size[i])
		{
			return i;
		}
	}
	
	return SYS_MALLOC_MEMORY_TYPE_NONE;
}
*/

/*
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
*/

static char *memory_type_to_string(uint8_t memory_type)
{
	switch(memory_type)
	{
		case SYS_MALLOC_MEMORY_TYPE_00:
			return "TYPE_00";
		case SYS_MALLOC_MEMORY_TYPE_01:
			return "TYPE_01";
		case SYS_MALLOC_MEMORY_TYPE_02:
			return "TYPE_02";
		case SYS_MALLOC_MEMORY_TYPE_03:
			return "TYPE_03";
		case SYS_MALLOC_MEMORY_TYPE_04:
			return "TYPE_04";
        case SYS_MALLOC_MEMORY_TYPE_05:
            return "TYPE_05";
		default:
			return NULL;
	}
}

static memory_control_block_t *get_memory_control_block_from_list(memory_control_block_t **p_p_mcb_list_head)
{
	memory_control_block_t *p_mcb = NULL;
	if(*p_p_mcb_list_head != NULL)
	{
		p_mcb = *p_p_mcb_list_head;
		*p_p_mcb_list_head = (*p_p_mcb_list_head)->next;//Pay more attention here!
        (*p_p_mcb_list_head)->prev = NULL;//Pay More Attention!!!
	}
	
	return p_mcb;
}

static void add_memory_control_block_to_list(memory_control_block_t **p_p_mcb_list_head, memory_control_block_t *p_mcb)
{
	memory_control_block_t *p;
	
	if(p_mcb == NULL)
	{
		return;
	}
	
	p_mcb->prev = NULL;
	p_mcb->next = NULL;//This is very important!!!
	
	if(*p_p_mcb_list_head == NULL)
	{
		*p_p_mcb_list_head = p_mcb;
		return;
	}
	
	p = *p_p_mcb_list_head;
	
	while(p->next != NULL)
	{
		if(p->p_start_addr > p_mcb->p_start_addr)
		{
			/*
			p_mcb->prev = p->prev;
			p_mcb->next = p;
			if(p->prev != NULL)
			{
				p->prev->next = p_mcb;
			}
			p->prev = p_mcb;
			return;
			*/
			break;
		}
		p = p->next;
	}
	
	//p is either the end of the list or the bigger one 
    
	if(p->p_start_addr > p_mcb->p_start_addr)
	{
		p_mcb->prev = p->prev;
		p_mcb->next = p;
		if(p->prev != NULL)
		{
			p->prev->next = p_mcb;
		}
		p->prev = p_mcb;
        
        if(p == *p_p_mcb_list_head)
        {
            *p_p_mcb_list_head = p_mcb;
        }
	}
	else
	{
		p->next = p_mcb;
		p_mcb->prev = p;
	}
    
   
}

//void sys_malloc_print(memory_control_block_t *p)
void sys_malloc_print(void)
{
    memory_control_block_t *p = free_memory_control_block_head_array[0];
    while(p != NULL)
    {
        NRF_LOG_INFO("----memory_type:%d\r\n", p->memory_type);
		NRF_LOG_INFO("----memory_usage:%d\r\n", p->memory_usage);
		NRF_LOG_INFO("----p_start_address:0x%08x\r\n", (uint32_t)p->p_start_addr);
		NRF_LOG_INFO("----call_line_num:%d\r\n", p->call_line_num);
		NRF_LOG_INFO("----create_timestamp:%d\r\n", p->create_timestamp);
        NRF_LOG_INFO("----prev:0x%08x\r\n", (uint32_t)p->prev);
        NRF_LOG_INFO("----next:0x%08x\r\n\n", (uint32_t)p->next);
        
        p = p->next;
    }
}

void sys_malloc_debug(void)
{
	memory_control_block_t *p_mcb = busy_memory_control_block_list_head;
    
    NRF_LOG_INFO("---sys_malloc_debug\r\n");
    
	while(p_mcb != NULL)
	{
		NRF_LOG_INFO("----memory_type:%s\r\n", (uint32_t)memory_type_to_string(p_mcb->memory_type));
		NRF_LOG_INFO("----memory_usage:%s\r\n", (uint32_t)memory_usage_string[p_mcb->memory_usage]);
		NRF_LOG_INFO("----p_start_address:0x%08x\r\n", (uint32_t)p_mcb->p_start_addr);
		NRF_LOG_INFO("----call_line_num:%d\r\n", p_mcb->call_line_num);
		NRF_LOG_INFO("----create_timestamp:%d\r\n\n", p_mcb->create_timestamp);
        
        p_mcb = p_mcb->next;
	}
}

uint8_t *sys_malloc_apply(uint16_t size, uint8_t usage, uint16_t line_num)
{
	memory_control_block_t *p_mcb = NULL;
	uint8_t *p_start_addr = NULL;
	uint8_t memory_type;
		
	for(memory_type = SYS_MALLOC_MEMORY_TYPE_00; memory_type < SYS_MALLOC_MEMORY_TYPE_MAX; memory_type++)
	{
        NRF_LOG_INFO("memory_type:%d, size:%d\r\n", memory_type, memory_size[memory_type]);
		if(size <= memory_size[memory_type])
		{
			if((p_mcb = get_memory_control_block_from_list(&free_memory_control_block_head_array[memory_type])) != NULL)
			{
				p_mcb->memory_usage 	= usage;
				p_mcb->call_line_num 	= line_num;
				p_mcb->create_timestamp = smcb.p_sys_info->sys_running_time;
				add_memory_control_block_to_list(&busy_memory_control_block_list_head, p_mcb);
				p_start_addr = p_mcb->p_start_addr;
				
				break;
			}
		}
	}
	return p_start_addr;
}

void sys_malloc_free(uint8_t *p_start_addr)
{
	memory_control_block_t *p_mcb = busy_memory_control_block_list_head;
	while(p_mcb != NULL)
	{
		if(p_mcb->p_start_addr == p_start_addr)
		{
			if(p_mcb->prev != NULL)
			{
				p_mcb->prev->next = p_mcb->next;
			}
			
			if(p_mcb->next != NULL)
			{
				p_mcb->next->prev = p_mcb->prev;
			}
			
            if(p_mcb == busy_memory_control_block_list_head)//Pay more attention
            {
                busy_memory_control_block_list_head = p_mcb->next;
            }
            
			add_memory_control_block_to_list(&free_memory_control_block_head_array[p_mcb->memory_type], p_mcb);
            break;
		}
        
        p_mcb = p_mcb->next;
	}
    
   
}

void sys_malloc_init(sys_info_t *p_sys_info)
{
	uint8_t i;
	
	smcb.p_sys_info = p_sys_info; 
	
	for(i = SYS_MALLOC_MEMORY_TYPE_00; i < SYS_MALLOC_MEMORY_TYPE_MAX; i++)
	{
		free_memory_control_block_head_array[i] = NULL;
	}
	
	for(i = 0; i < MEMORY_TYPE_00_BLOCK_COUNT; i++)
	{
		memory_type_00_mcb_array[i].memory_type 	 = SYS_MALLOC_MEMORY_TYPE_00;
		memory_type_00_mcb_array[i].memory_usage 	 = MEMORY_USAGE_NONE;
		memory_type_00_mcb_array[i].p_start_addr 	 = &memory_type_01_memory_pool[i][0];
		memory_type_00_mcb_array[i].call_line_num    = 0;
		memory_type_00_mcb_array[i].create_timestamp = 0;
		
		add_memory_control_block_to_list(&free_memory_control_block_head_array[SYS_MALLOC_MEMORY_TYPE_00], &memory_type_01_mcb_array[i]);
	}
	
	for(i = 0; i < MEMORY_TYPE_01_BLOCK_COUNT; i++)
	{
		memory_type_01_mcb_array[i].memory_type 	 = SYS_MALLOC_MEMORY_TYPE_01;
		memory_type_01_mcb_array[i].memory_usage 	 = MEMORY_USAGE_NONE;
		memory_type_01_mcb_array[i].p_start_addr 	 = &memory_type_01_memory_pool[i][0];
		memory_type_01_mcb_array[i].call_line_num    = 0;
		memory_type_01_mcb_array[i].create_timestamp = 0;
		
	    add_memory_control_block_to_list(&free_memory_control_block_head_array[SYS_MALLOC_MEMORY_TYPE_01], &memory_type_01_mcb_array[i]);
	}
	
	for(i = 0; i < MEMORY_TYPE_02_BLOCK_COUNT; i++)
	{
		memory_type_02_mcb_array[i].memory_type 	 = SYS_MALLOC_MEMORY_TYPE_02;
		memory_type_02_mcb_array[i].memory_usage 	 = MEMORY_USAGE_NONE;
		memory_type_02_mcb_array[i].p_start_addr 	 = &memory_type_02_memory_pool[i][0];
		memory_type_02_mcb_array[i].call_line_num    = 0;
		memory_type_02_mcb_array[i].create_timestamp = 0;
		
		add_memory_control_block_to_list(&free_memory_control_block_head_array[SYS_MALLOC_MEMORY_TYPE_02], &memory_type_02_mcb_array[i]);
	}
		
	for(i = 0; i < MEMORY_TYPE_03_BLOCK_COUNT; i++)
	{
		memory_type_03_mcb_array[i].memory_type 	 = SYS_MALLOC_MEMORY_TYPE_03;
		memory_type_03_mcb_array[i].memory_usage 	 = MEMORY_USAGE_NONE;
		memory_type_03_mcb_array[i].p_start_addr 	 = &memory_type_03_memory_pool[i][0];
		memory_type_03_mcb_array[i].call_line_num    = 0;
		memory_type_03_mcb_array[i].create_timestamp = 0;
		
		add_memory_control_block_to_list(&free_memory_control_block_head_array[SYS_MALLOC_MEMORY_TYPE_03], &memory_type_03_mcb_array[i]);
	}
	
		
	for(i = 0; i < MEMORY_TYPE_04_BLOCK_COUNT; i++)
	{
		memory_type_04_mcb_array[i].memory_type 	 = SYS_MALLOC_MEMORY_TYPE_04;
		memory_type_04_mcb_array[i].memory_usage 	 = MEMORY_USAGE_NONE;
		memory_type_04_mcb_array[i].p_start_addr 	 = &memory_type_04_memory_pool[i][0];
		memory_type_04_mcb_array[i].call_line_num    = 0;
		memory_type_04_mcb_array[i].create_timestamp = 0;
		
		add_memory_control_block_to_list(&free_memory_control_block_head_array[SYS_MALLOC_MEMORY_TYPE_04], &memory_type_04_mcb_array[i]);
	}
    
    for(i = 0; i < MEMORY_TYPE_05_BLOCK_COUNT; i++)
	{
		memory_type_05_mcb_array[i].memory_type 	 = SYS_MALLOC_MEMORY_TYPE_05;
		memory_type_05_mcb_array[i].memory_usage 	 = MEMORY_USAGE_NONE;
		memory_type_05_mcb_array[i].p_start_addr 	 = &memory_type_05_memory_pool[i][0];
		memory_type_05_mcb_array[i].call_line_num    = 0;
		memory_type_05_mcb_array[i].create_timestamp = 0;
		
		add_memory_control_block_to_list(&free_memory_control_block_head_array[SYS_MALLOC_MEMORY_TYPE_05], &memory_type_05_mcb_array[i]);
	}
}

