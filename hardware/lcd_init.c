#include "nrf_gpio.h"
#include "lcd_init.h"
#include "nrf_delay.h"

void LCD_GPIO_Init(void)
{    
    nrf_gpio_cfg_output(LCD_DC_PIN_NUMBER);
	nrf_gpio_cfg_output(LCD_RES_PIN_NUMBER);
  	nrf_gpio_cfg_output(LCD_MOSI_PIN_NUMBER);
  	nrf_gpio_cfg_output(LCD_SCLK_PIN_NUMBER);
    nrf_gpio_cfg_output(LCD_BLK_PIN_NUMBER);
    
    //Why init those pin to high??? The screen will not turn on when don't add following statements!
    nrf_gpio_pin_set(LCD_DC_PIN_NUMBER);
	nrf_gpio_pin_set(LCD_RES_PIN_NUMBER);
  	nrf_gpio_pin_set(LCD_MOSI_PIN_NUMBER);
  	nrf_gpio_pin_set(LCD_SCLK_PIN_NUMBER);
    nrf_gpio_pin_set(LCD_BLK_PIN_NUMBER);
}


/******************************************************************************
      函数说明：LCD串行数据写入函数
      入口数据：dat  要写入的串行数据
      返回值：  无
******************************************************************************/
void LCD_Writ_Bus(u8 dat) 
{	
	u8 i;
	for(i=0;i<8;i++)
	{			  
		LCD_SCLK_Clr();
		if(dat&0x80)
		{
		   LCD_MOSI_Set();
		}
		else
		{
		   LCD_MOSI_Clr();
		}
		LCD_SCLK_Set();
		dat<<=1;
	}	
}


/******************************************************************************
      函数说明：LCD写入数据
      入口数据：dat 写入的数据
      返回值：  无
******************************************************************************/
void LCD_WR_DATA8(u8 dat)
{
	LCD_Writ_Bus(dat);
}


/******************************************************************************
      函数说明：LCD写入数据
      入口数据：dat 写入的数据
      返回值：  无
******************************************************************************/
void LCD_WR_DATA(u16 dat)
{
	LCD_Writ_Bus(dat>>8);
	LCD_Writ_Bus(dat);
}


/******************************************************************************
      函数说明：LCD写入命令
      入口数据：dat 写入的命令
      返回值：  无
******************************************************************************/
void LCD_WR_REG(u8 dat)
{
	LCD_DC_Clr();//写命令
	LCD_Writ_Bus(dat);
	LCD_DC_Set();//写数据
}


/******************************************************************************
      函数说明：设置起始和结束地址
      入口数据：x1,x2 设置列的起始和结束地址
                y1,y2 设置行的起始和结束地址
      返回值：  无
******************************************************************************/
void LCD_Address_Set(u16 x1,u16 y1,u16 x2,u16 y2)
{
	if(USE_HORIZONTAL==0)
	{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1);
		LCD_WR_DATA(x2);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1);
		LCD_WR_DATA(y2);
		LCD_WR_REG(0x2c);//储存器写
	}
	else if(USE_HORIZONTAL==1)
	{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1);
		LCD_WR_DATA(x2);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1+80);
		LCD_WR_DATA(y2+80);
		LCD_WR_REG(0x2c);//储存器写
	}
	else if(USE_HORIZONTAL==2)
	{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1);
		LCD_WR_DATA(x2);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1);
		LCD_WR_DATA(y2);
		LCD_WR_REG(0x2c);//储存器写
	}
	else
	{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1+80);
		LCD_WR_DATA(x2+80);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1);
		LCD_WR_DATA(y2);
		LCD_WR_REG(0x2c);//储存器写
	}
}

void set_scroll_area(uint16_t tfa, uint16_t bta)
{
    uint16_t vsa = 320 - tfa - bta;
    LCD_WR_REG(0x33);//STESCROLLAREA
    LCD_WR_DATA((uint8_t)(tfa >> 8));
    LCD_WR_DATA((uint8_t)(tfa >> 0));
    LCD_WR_DATA((uint8_t)(vsa >> 8));
    LCD_WR_DATA((uint8_t)(vsa >> 0));
    LCD_WR_DATA((uint8_t)(bta >> 8));
    LCD_WR_DATA((uint8_t)(bta >> 0));
}

void set_scroll(uint16_t vsp)
{
    LCD_WR_REG(0x37);//ST7789_VSCRSADD
    LCD_WR_DATA((uint8_t)(vsp >> 8));
    LCD_WR_DATA((uint8_t)(vsp >> 0));
}

#if 0
void LCD_Init(void)
{
    LCD_GPIO_Init();//初始化GPIO
    delay_ms(120);
    LCD_WR_REG(0x11); //Sleep out 
    delay_ms(120);
    LCD_WR_REG(0xB2);                                       //PORCTRL: Porch control
	LCD_WR_DATA8(0x0C);
	LCD_WR_DATA8(0x0C);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x33);
	LCD_WR_DATA8(0x33);
    
    LCD_WR_REG(0xB7);
    LCD_WR_DATA8(0x21);
    
    LCD_WR_REG(0xBB);
    LCD_WR_DATA8(0x3F);
    
    LCD_WR_REG(0xC2);
    LCD_WR_DATA8(0x01);
    
    LCD_WR_REG(0xC3);
    LCD_WR_DATA8(0x19);
    
    LCD_WR_REG(0xC4);
    LCD_WR_DATA8(0x20);
    
    LCD_WR_REG(0xC6);
    LCD_WR_DATA8(0x0F);
    
    LCD_WR_REG(0xD0);
    LCD_WR_DATA8(0xA4);
    LCD_WR_DATA8(0xA1);
    
    LCD_WR_REG(0xE0);
    LCD_WR_DATA8(0x70);
    LCD_WR_DATA8(0x04);
    LCD_WR_DATA8(0x08);
    LCD_WR_DATA8(0x09);
    LCD_WR_DATA8(0x09);
    LCD_WR_DATA8(0x05);
    LCD_WR_DATA8(0x2A);
    LCD_WR_DATA8(0x33);
    LCD_WR_DATA8(0x41);
    LCD_WR_DATA8(0x07);
    LCD_WR_DATA8(0x13);
    LCD_WR_DATA8(0x13);
    LCD_WR_DATA8(0x29);
    LCD_WR_DATA8(0x2F);
    
    LCD_WR_REG(0xE1);
    LCD_WR_DATA8(0x70);
    LCD_WR_DATA8(0x03);
    LCD_WR_DATA8(0x09);
    LCD_WR_DATA8(0x0A);
    LCD_WR_DATA8(0x09);
    LCD_WR_DATA8(0x06);
    LCD_WR_DATA8(0x2B);
    LCD_WR_DATA8(0x34);
    LCD_WR_DATA8(0x41);
    LCD_WR_DATA8(0x07);
    LCD_WR_DATA8(0x12);
    LCD_WR_DATA8(0x14);
    LCD_WR_DATA8(0x28);
    LCD_WR_DATA8(0x2E);
    
    LCD_WR_DATA8(0x29);
}
#else
void LCD_Init(void)
{
	LCD_GPIO_Init();//初始化GPIO
	
    //LCD_BLK_Clr();//close backlight
    
	LCD_RES_Clr();//复位
	delay_ms(100);
	LCD_RES_Set();
	delay_ms(100);
	
	LCD_BLK_Set();//打开背光
    delay_ms(100);
	
	//************* Start Initial Sequence **********//
	LCD_WR_REG(0x11); //Sleep out 
	delay_ms(120);              //Delay 120ms 
	//************* Start Initial Sequence **********// 
	LCD_WR_REG(0x36);                                       //MADCTL:Memory data access control
	if(USE_HORIZONTAL==0)LCD_WR_DATA8(0x00);
	else if(USE_HORIZONTAL==1)LCD_WR_DATA8(0xC0);
	else if(USE_HORIZONTAL==2)LCD_WR_DATA8(0x70);
	else LCD_WR_DATA8(0xA0);

    LCD_WR_REG(0x3A);                                       //COLMOD:Interface pixel format
	LCD_WR_DATA8(0x05);

    LCD_WR_REG(0xB2);                                       //PORCTRL: Porch control
	LCD_WR_DATA8(0x0C);
	LCD_WR_DATA8(0x0C);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x33);
	LCD_WR_DATA8(0x33); 

    LCD_WR_REG(0xB7);                                        //GCTRL: Gate Control
	LCD_WR_DATA8(0x35);  

    LCD_WR_REG(0xBB);                                        //VCOMS:VCOM Setting
	LCD_WR_DATA8(0x19);

    LCD_WR_REG(0xC0);                                        //LCMCTRL:LCM Control
	LCD_WR_DATA8(0x2C);

    LCD_WR_REG(0xC2);                                        //VDVVRHEN:VDV and VRH Command Enable
	LCD_WR_DATA8(0x01);

    LCD_WR_REG(0xC3);                                        //VRHS:VRH Set
	LCD_WR_DATA8(0x12);   

LCD_WR_REG(0xC4);                                        //VDVSET:VDV Setting
	LCD_WR_DATA8(0x20);  

LCD_WR_REG(0xC6);                                    //FRCTR2:FR Control 2
	LCD_WR_DATA8(0x0F);    

LCD_WR_REG(0xD0);                                    //PWCTRL1:Power Control 1
	LCD_WR_DATA8(0xA4);
	LCD_WR_DATA8(0xA1);

LCD_WR_REG(0xE0);                                //PVGAMCTRL:Positive Voltage Gamma Control
	LCD_WR_DATA8(0xD0);
	LCD_WR_DATA8(0x04);
	LCD_WR_DATA8(0x0D);
	LCD_WR_DATA8(0x11);
	LCD_WR_DATA8(0x13);
	LCD_WR_DATA8(0x2B);
	LCD_WR_DATA8(0x3F);
	LCD_WR_DATA8(0x54);
	LCD_WR_DATA8(0x4C);
	LCD_WR_DATA8(0x18);
	LCD_WR_DATA8(0x0D);
	LCD_WR_DATA8(0x0B);
	LCD_WR_DATA8(0x1F);
	LCD_WR_DATA8(0x23);

LCD_WR_REG(0xE1);                                                    //NVGAMCTRL:Negative Voltage Gamma Control
	LCD_WR_DATA8(0xD0);
	LCD_WR_DATA8(0x04);
	LCD_WR_DATA8(0x0C);
	LCD_WR_DATA8(0x11);
	LCD_WR_DATA8(0x13);
	LCD_WR_DATA8(0x2C);
	LCD_WR_DATA8(0x3F);
	LCD_WR_DATA8(0x44);
	LCD_WR_DATA8(0x51);
	LCD_WR_DATA8(0x2F);
	LCD_WR_DATA8(0x1F);
	LCD_WR_DATA8(0x1F);
	LCD_WR_DATA8(0x20);
	LCD_WR_DATA8(0x23);
	LCD_WR_REG(0x21);                        //INVON:Display inversion on

    LCD_WR_REG(0x29);                        //DISPON:Display on
} 
#endif






