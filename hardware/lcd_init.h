#ifndef __LCD_INIT_H
#define __LCD_INIT_H

#include "type.h"

#define USE_HORIZONTAL 0  //���ú�������������ʾ 0��1Ϊ���� 2��3Ϊ����

#define LCD_W 240
#define LCD_H 240

#define delay_ms    nrf_delay_ms 

//LCD relative gpio
//#define NEW_HARDWARE    1

#ifdef NEW_HARDWARE
#define LCD_MOSI_PIN_NUMBER							26
#define LCD_SCLK_PIN_NUMBER							25
#define LCD_DC_PIN_NUMBER							2
#define LCD_RES_PIN_NUMBER							23
#define LCD_BLK_PIN_NUMBER                          9
#else
#define LCD_MOSI_PIN_NUMBER							11
#define LCD_SCLK_PIN_NUMBER							12
#define LCD_DC_PIN_NUMBER							13
#define LCD_RES_PIN_NUMBER							14
#define LCD_BLK_PIN_NUMBER                          15
#endif

//-----------------LCD�˿ڶ���---------------- 

#define LCD_SCLK_Clr() nrf_gpio_pin_clear(LCD_SCLK_PIN_NUMBER); 
#define LCD_SCLK_Set() nrf_gpio_pin_set(LCD_SCLK_PIN_NUMBER);

#define LCD_MOSI_Clr() nrf_gpio_pin_clear(LCD_MOSI_PIN_NUMBER)//SDA=MOSI
#define LCD_MOSI_Set() nrf_gpio_pin_set(LCD_MOSI_PIN_NUMBER)

#define LCD_RES_Clr()  nrf_gpio_pin_clear(LCD_RES_PIN_NUMBER)//RES
#define LCD_RES_Set()  nrf_gpio_pin_set(LCD_RES_PIN_NUMBER)

#define LCD_DC_Clr()   nrf_gpio_pin_clear(LCD_DC_PIN_NUMBER)//DC
#define LCD_DC_Set()   nrf_gpio_pin_set(LCD_DC_PIN_NUMBER)

#define LCD_BLK_Clr()  nrf_gpio_pin_clear(LCD_BLK_PIN_NUMBER)//BLK
#define LCD_BLK_Set()  nrf_gpio_pin_set(LCD_BLK_PIN_NUMBER)

void LCD_GPIO_Init(void);//��ʼ��GPIO
void LCD_Writ_Bus(u8 dat);//ģ��SPIʱ��
void LCD_WR_DATA8(u8 dat);//д��һ���ֽ�
void LCD_WR_DATA(u16 dat);//д�������ֽ�
void LCD_WR_REG(u8 dat);//д��һ��ָ��
void LCD_Address_Set(u16 x1,u16 y1,u16 x2,u16 y2);//�������꺯��
extern void set_scroll_area(uint16_t tfa, uint16_t bta);
extern void set_scroll(uint16_t vsp);
void LCD_Init(void);//LCD��ʼ��
#endif




