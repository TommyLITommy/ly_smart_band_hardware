#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_gpio.h"
#include "sys_info.h"
#include "lcd_init.h"
#include "drv_lcd.h"

static uint16_t color_array[] =
{
    BLUE,           	 
    BRED,             
    GRED, 			 
    GBLUE,			 
    RED,           	 
    MAGENTA,       	 
    GREEN,         	 
    CYAN,          	 
    YELLOW,        	 
    BROWN, 			 
    BRRED, 			 
    GRAY,  			 
    DARKBLUE,      	 
    LIGHTBLUE,      	 
    GRAYBLUE,       	 
    LIGHTGREEN,     	 
    LGRAY, 			 
    LGRAYBLUE,        
    LBBLUE,           
};

static void drv_lcd_fill(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    NRF_LOG_INFO("drv_lcd_fill\r\n");
    u16 i,j; 
	LCD_Address_Set(x1, y1, x2 - 1, y2 - 1);
	for(i = y1; i < y2; i++)
	{													   	 	
		for(j = x1; j < x2; j++)
		{
			LCD_WR_DATA(color);
		}
	} 
}

static void drv_lcd_draw_point(uint16_t x, uint16_t y, uint16_t color)
{
	LCD_Address_Set(x,y,x,y);
	LCD_WR_DATA(color);
}

static void drv_lcd_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
	u16 t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance;
	int incx,incy,uRow,uCol;
	delta_x=x2-x1; 
	delta_y=y2-y1;
	uRow=x1;
	uCol=y1;
	if(delta_x>0)incx=1; 
	else if (delta_x==0)incx=0;
	else {incx=-1;delta_x=-delta_x;}
	if(delta_y>0)incy=1;
	else if (delta_y==0)incy=0;
	else {incy=-1;delta_y=-delta_x;}
	if(delta_x>delta_y)distance=delta_x; 
	else distance=delta_y;
	for(t=0;t<distance+1;t++)
	{
		drv_lcd_draw_point(uRow,uCol,color);
		xerr+=delta_x;
		yerr+=delta_y;
		if(xerr>distance)
		{
			xerr-=distance;
			uRow+=incx;
		}
		if(yerr>distance)
		{
			yerr-=distance;
			uCol+=incy;
		}
	}
}

static void drv_lcd_draw_rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
	drv_lcd_draw_line(x1,y1,x2,y1,color);
	drv_lcd_draw_line(x1,y1,x1,y2,color);
	drv_lcd_draw_line(x1,y2,x2,y2,color);
	drv_lcd_draw_line(x2,y1,x2,y2,color);
}

static void drv_lcd_draw_picture(uint16_t x, uint16_t y, uint16_t length, uint16_t width, const uint8_t pic[])
{
	u16 i,j,k=0;
	LCD_Address_Set(x,y,x+length-1,y+width-1);
	for(i=0;i<length;i++)
	{
		for(j=0;j<width;j++)
		{
			LCD_WR_DATA8(pic[k*2]);
			LCD_WR_DATA8(pic[k*2+1]);
			k++;
		}
	}			
}

void drv_lcd_init(drv_lcd_t *p_drv_lcd)
{	
    uint8_t line_color[240 * 2];
    
    for(uint8_t m = 0; m < 240; m++)
    {
        line_color[2 * m + 0] = 0xF8;
        line_color[2 * m + 1] = 0x00;
    }
    
    nrf_gpio_cfg_output(24);//Chip Select Pin!!!
    nrf_gpio_pin_clear(24);
    
    nrf_gpio_cfg_output(31);//VLCDBK_EN
    nrf_gpio_cfg_output(8);//VLED_EN
    
    nrf_gpio_pin_set(31);
    nrf_gpio_pin_set(8);
    
    NRF_LOG_INFO("drv_lcd_init\r\n");
    
    LCD_Init();
    drv_lcd_fill(0, 0, LCD_W, LCD_H, WHITE);
    //drv_lcd_fill(1, 1, 15, 15, 0XF81F);
    //drv_lcd_draw_picture(0, 10, 240, 1, line_color);
    NRF_LOG_INFO("----\r\n");
    //while(1)
    while(0)
    {
       for(uint8_t i = 0; i < 5; i++)
       {
           drv_lcd_fill(0, 0, LCD_W, LCD_H, color_array[i]);
       }
    }
    
    p_drv_lcd->drv_lcd_fill 		    = drv_lcd_fill;
	p_drv_lcd->drv_lcd_draw_point 		= drv_lcd_draw_point;
	p_drv_lcd->drv_lcd_draw_line		= drv_lcd_draw_line;
	p_drv_lcd->drv_lcd_draw_rectangle 	= drv_lcd_draw_rectangle;
	p_drv_lcd->drv_lcd_draw_picture 	= drv_lcd_draw_picture;
}

