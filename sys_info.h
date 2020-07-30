#ifndef __SYS_INFO_H__
#define __SYS_INFO_H__

#include "uart_protocol.h"
#include "hardware.h"
#include "sys_queue.h"

typedef struct
{
	uint32_t        sys_running_time;
    uart_protocol_t uart_protocol;
    hardware_t      hardware;
    sys_queue_t     rx_queue;
    sys_queue_t     tx_queue;
}sys_info_t;

extern sys_info_t sys_info;

extern void sys_info_init(sys_info_t *p_sys_info);

#endif
