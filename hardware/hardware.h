#ifndef __HARDWARE_H__
#define __HARDWARE_H__

#include "drv_lcd.h"
#include "drv_uart.h"

typedef struct
{
	drv_lcd_t drv_lcd;
	drv_uart_t drv_uart;
}hardware_t;

extern void hardware_init(hardware_t *p_hardeare);
#endif






