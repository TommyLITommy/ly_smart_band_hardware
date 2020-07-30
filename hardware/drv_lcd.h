#ifndef __DRV_LCD_H__
#define __DRV_LCD_H__

#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE           	 0x001F  
#define BRED             0XF81F
#define GRED 			 0XFFE0
#define GBLUE			 0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			 0XBC40
#define BRRED 			 0XFC07 
#define GRAY  			 0X8430 
#define DARKBLUE      	 0X01CF	
#define LIGHTBLUE      	 0X7D7C	
#define GRAYBLUE       	 0X5458
#define LIGHTGREEN     	 0X841F 
#define LGRAY 			 0XC618
#define LGRAYBLUE        0XA651
#define LBBLUE           0X2B12

typedef void(*drv_lcd_fill_t)(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
typedef void(*drv_lcd_draw_point_t)(uint16_t x, uint16_t y, uint16_t color);
typedef void(*drv_lcd_draw_line_t)(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
typedef void(*drv_lcd_draw_rectangle_t)(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
typedef void(*drv_lcd_draw_picture_t)(uint16_t x, uint16_t y, uint16_t length, uint16_t width, const uint8_t pic[]);

typedef struct
{
    drv_lcd_fill_t              drv_lcd_fill;
	drv_lcd_draw_point_t 		drv_lcd_draw_point;
	drv_lcd_draw_line_t  		drv_lcd_draw_line;
	drv_lcd_draw_rectangle_t 	drv_lcd_draw_rectangle;
	drv_lcd_draw_picture_t 		drv_lcd_draw_picture;
}drv_lcd_t;

extern void drv_lcd_init(drv_lcd_t *p_drv_lcd);

#endif
