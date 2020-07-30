/**
 * Copyright (c) 2015 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @file
 * @defgroup blinky_example_main main.c
 * @{
 * @ingroup blinky_example_freertos
 *
 * @brief Blinky FreeRTOS Example Application main file.
 *
 * This file contains the source code for a sample application using FreeRTOS to blink LEDs.
 *
 */

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "bsp.h"
#include "nordic_common.h"
#include "nrf_drv_clock.h"
#include "sdk_errors.h"
#include "app_error.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "sys_info.h"
#include "sys_malloc.h"
#include "sys_queue.h"
#include "drv_uart.h"
#include "lcd_init.h"
#include "drv_lcd.h"
#include "app_timer.h"
#include "nrf_delay.h"
#include "drv_flash.h"
#include "store_picture_to_flash.h"

#include "drv_lis3dh.h"
#include "drv_gt24l24a2y.h"
#include "GT24L24A2Y.h"
#include "nrf_drv_spi.h"
#include "drv_Ads129x.h"
#include "spi_driver.h"

sys_info_t sys_info;

#define HARDWARE_TEST_GT24L24A2Y            1
//#define HARDWARE_TEST_LIS3DH                1
//#define HAREWARE_TEST_LCD                   1
//#define HAREWARE_TEST_ADS129X               1
//#define HARDWARE_TEST_TOUCH_KEY              1
//#define HAREWARE_TEST_MOTOR                     1

/*-----------------------------------*/
#ifdef HARDWARE_TEST_GT24L24A2Y
uint8_t font_buffer[24][24] = {0};

void oled_draw_rectangle_new(uint8_t frame_array_row, uint8_t frame_array_column, uint8_t frame_actual_row, uint8_t frame_actual_column, const uint8_t *image)
{
	uint8_t i, j, k, m;
	uint8_t temp;
	for(i = 0; i < frame_array_row; i++)//2
	{
		for(j = 0; j < frame_array_column; j++)//16
		{
			temp = image[i * frame_array_column + j];
	
			for(k = 0; k < 8; k++)
			{
				if((8 * i + k) >= frame_actual_row)
				{
					break;//Here pay more attention
				}
				
				if(temp & 0x01)
				{
					font_buffer[8 * i + k][j] = 1;
				}
				else
				{
				}
				temp = temp >> 1;
			}
		}
	}
}

static void uart_task_function (void * pvParameter)
{
	uint8_t element_type;
	uint8_t *p_start_addr;
	uint16_t length;
    uint32_t err_code;
    uint32_t index = 0;
    
    nrf_delay_ms(2000);
    
    nrf_gpio_cfg_output(8);
    nrf_gpio_pin_set(8);
    
    nrf_gpio_cfg_output(4);
    nrf_gpio_pin_clear(4);
    //nrf_gpio_pin_set(4);
    
    NRF_LOG_RAW_INFO("\r\n");
        
    nrf_gpio_cfg_output(28);
    nrf_gpio_pin_set(28);
    
    nrf_gpio_cfg_output(30);
    nrf_gpio_pin_set(30);
    
    nrf_gpio_cfg_output(24);
    nrf_gpio_pin_set(24);
    
    drv_gt24l24a2y_init();
    uint8_t temp_buffer[128] = {0};
    #if 0
    ASCII_GetData('A', ASCII_12X24, temp_buffer);
    oled_draw_rectangle_new(3, 12, 24, 12, temp_buffer);
    for(int m = 0; m < 24; m++)
    {
    	for(int n = 0; n < 12; n++)
    	{
    		NRF_LOG_RAW_INFO("%d ", font_buffer[m][n]);
    	}

		NRF_LOG_RAW_INFO("\r\n");
    }
    #else
	uint32_t gb18030;
	gb18030 = U2G(0x5171);
    //hzbmp16(SEL_GB, gb18030, 0, 16, temp_buffer);
    hzbmp24(SEL_GB, gb18030, 0, 24, temp_buffer);

	NRF_LOG_HEXDUMP_INFO(temp_buffer, 128);
    
    oled_draw_rectangle_new(3, 24, 24, 24, temp_buffer);

    for(int m = 0; m < 24; m++)
    {
    	for(int n = 0; n < 24; n++)
    	{
    		NRF_LOG_RAW_INFO("%d ", font_buffer[m][n]);
    	}

		NRF_LOG_RAW_INFO("\r\n");
    }
    #endif
    
    extern const unsigned char gImage_clock_01[115200];
    store_picture_to_flash(2, 2 + 11, gImage_clock_01, 115200);
    
    uint8_t font_buffer[240 * 2];
    
    uint32_t address;
    address = FLASH_FIRST_SECTOR_START_ADDRESS + 2 * PICTURE_SIZE + 1;
    
    NRF_LOG_INFO("Pic Read Start\r\n");
    
    for(uint32_t m = 0; m < 240; m++)
    {
        drv_flash_read(address, font_buffer, 480);
        //NRF_LOG_HEXDUMP_INFO(font_buffer, 480);
        address += 480;
    }
    
    NRF_LOG_INFO("Pic Read End\r\n");
    
    while(1)
    {
    }
}
#endif

#ifdef HARDWARE_TEST_LIS3DH
static void uart_task_function (void * pvParameter)
{        
    uint8_t chip_id;
    nrf_gpio_cfg_output(8);
    nrf_gpio_pin_set(8);
    
    nrf_gpio_cfg_output(4);
    nrf_gpio_pin_set(4);
    
    NRF_LOG_RAW_INFO("\r\n");
   
    //Test Lis3dh    
    nrf_gpio_cfg_output(6);
    nrf_gpio_pin_set(6);
    
    nrf_gpio_cfg_output(28);
    nrf_gpio_pin_set(28);
    
    nrf_gpio_cfg_output(30);
    nrf_gpio_pin_set(30);
    
    nrf_gpio_cfg_output(24);
    nrf_gpio_pin_set(24);
    
    #if 1
    SPI_Init();
    while(1)
    {
        chip_id = SPI_ReadReg(0x0F, NULL);
        NRF_LOG_INFO("1-chip_id = %d\r\n", chip_id);
        nrf_delay_ms(1000);
    }
    #else
    drv_lis3dh_init();
    //vADS1X9X_Initialize();
    //Ads129x_Auto_Measurement_Set(ADS129X_AUTO_ON); 
    while(1)
    {
        who_am_i_check();
        nrf_delay_ms(500);
        //chip_id = u8ADS1x9x_Reg_Read(ADS1291_ID_CONFIG);
        //NRF_LOG_INFO("chip_id = %d\r\n", chip_id);
        nrf_delay_ms(500);
    }
    #endif
}
#endif

#ifdef HAREWARE_TEST_ADS129X

#if 0
typedef enum
{
	ADS129X_AUTO_ON = 0x01,
  ADS129X_AUTO_OFF = 0x00
}Ads129x_auto_t;

typedef enum
{
	ADS129X_POWER_ON = 0x01,
  ADS129X_POWER_OFF = 0x00
}Ads129x_power_t;

#define ADS1291_ID_CONFIG       0x00
#define ADS1291_REG_CONFIG1     0x01
#define ADS1291_REG_CONFIG2     0x02
#define ADS1291_REG_LOFF        0x03
#define ADS1291_REG_CH1SET      0x04
#define ADS1291_REG_CH2SET      0x05
#define ADS1291_REG_RLDSENS     0x06
#define ADS1291_REG_LOFFSENS    0x07
#define ADS1291_REG_LOFF_STAT   0x08
#define ADS1291_REG_RESP1       0x09
#define ADS1291_REG_RESP2       0x0A
#define ADS1291_REG_GPIO        0x0B

#define ADS1X9X_RESET_PIN                   21             
#define ADS1X9X_START_PIN                   20
#define ADS1X9X_DRDY_PIN                    19
#define ADS1X9X_SPI_SOMI_PIN                18
#define ADS1X9X_SPI_SCLK_PIN                17
#define ADS1X9X_SPI_SIMO_PIN                16
#define ADS1X9X_SPI_CS_PIN                  15
#define ADS1X9X_AVDD_PIN           		    14
#define ADS1X9X_DVDD_PIN                     9

//Register Read Commands
#define RREG		0x20			//Read n nnnn registers starting at address r rrrr
													//first byte 001r rrrr (2xh)(2) - second byte 000n nnnn(2)
#define WREG		0x40			//Write n nnnn registers starting at address r rrrr
													//first byte 010r rrrr (2xh)(2) - second byte 000n nnnn(2)

#define NRF_DRV_SPI_ADS1X9X_CONFIG                           \
{                                                            \
    .sck_pin      = ADS1X9X_SPI_SCLK_PIN,                \
    .mosi_pin     = ADS1X9X_SPI_SIMO_PIN,                \
    .miso_pin     = ADS1X9X_SPI_SOMI_PIN,                \
    .ss_pin       = NRF_DRV_SPI_PIN_NOT_USED,                \
    .irq_priority = SPI_DEFAULT_CONFIG_IRQ_PRIORITY,         \
    .orc          = 0xFF,                                    \
    .frequency    = NRF_DRV_SPI_FREQ_1M,                     \
    .mode         = NRF_DRV_SPI_MODE_1,                      \
    .bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,         \
}

#define ADS1291_DEVICE_ID       0x52 
#define ADS1291_ECG_DATA_NUM    6 
uint8_t  gu8Ads1x9x_Ecg_data_buf[ADS1291_ECG_DATA_NUM] = {0}; 

#define ADS1X9X_SPI_INSTANCE    1       /**< SPI instance index. */
static volatile bool Ads1x9x_spi_xfer_done; 
static const    nrf_drv_spi_t Ads1x9x_Spi = NRF_DRV_SPI_INSTANCE(ADS1X9X_SPI_INSTANCE);

static uint8_t    gu8Ads1x9x_spi_tx_buf[256];   /**< TX buffer. */
static uint8_t    gu8Ads1x9x_spi_rx_buf[256];   /**< RX buffer. */

void vADS1x9x_CS_HIGH(void)
{
	nrf_gpio_pin_set(ADS1X9X_SPI_CS_PIN);
}

void vADS129x_CS_LOW(void)
{
   nrf_gpio_pin_clear(ADS1X9X_SPI_CS_PIN); 
}

uint8_t u8ADS1x9x_Reg_Read(uint8_t Reg_address)
{
	uint8_t len = 2;
	uint8_t retVal = 0;
  
	gu8Ads1x9x_spi_tx_buf[0] = (Reg_address | RREG);
	gu8Ads1x9x_spi_tx_buf[1] = 0;                  
	
	vADS129x_CS_LOW();
	nrf_delay_us(1);
	Ads1x9x_spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&Ads1x9x_Spi, gu8Ads1x9x_spi_tx_buf, len, gu8Ads1x9x_spi_rx_buf, 3));
	while(!Ads1x9x_spi_xfer_done);
	nrf_delay_us(1);
	vADS1x9x_CS_HIGH();
	
	retVal = gu8Ads1x9x_spi_rx_buf[2]; 
	
	return retVal;
}

void vAds129x_PortInit(void)
{
    nrf_gpio_cfg_output(ADS1X9X_RESET_PIN);
    nrf_gpio_cfg_output(ADS1X9X_START_PIN);
    nrf_gpio_cfg_output(ADS1X9X_DRDY_PIN);
    nrf_gpio_cfg_output(ADS1X9X_SPI_SOMI_PIN);
    nrf_gpio_cfg_output(ADS1X9X_SPI_SCLK_PIN);
    nrf_gpio_cfg_output(ADS1X9X_SPI_SIMO_PIN);
    nrf_gpio_cfg_output(ADS1X9X_SPI_CS_PIN);

    nrf_gpio_cfg_output(ADS1X9X_DVDD_PIN);
    nrf_gpio_cfg_output(ADS1X9X_AVDD_PIN);

    nrf_gpio_pin_clear(ADS1X9X_RESET_PIN);
    nrf_gpio_pin_clear(ADS1X9X_START_PIN);
    nrf_gpio_pin_set(ADS1X9X_DRDY_PIN);
    nrf_gpio_pin_clear(ADS1X9X_SPI_SOMI_PIN);
    nrf_gpio_pin_clear(ADS1X9X_SPI_SCLK_PIN);
    nrf_gpio_pin_clear(ADS1X9X_SPI_SIMO_PIN);
    nrf_gpio_pin_set(ADS1X9X_SPI_CS_PIN);
    
    return;
}

void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    Ads1x9x_spi_xfer_done = true;
}

void vADS129X_Spi_Init(void)  
{
    nrf_drv_spi_config_t Ads1x9x_spi_config = NRF_DRV_SPI_ADS1X9X_CONFIG;
	APP_ERROR_CHECK(nrf_drv_spi_init(&Ads1x9x_Spi, &Ads1x9x_spi_config, spi_event_handler, NULL));
	return;
}


//ADS1291 - 5V
void vADS129X_Avdd_Power_On(void)
{
	nrf_gpio_pin_set(ADS1X9X_AVDD_PIN);
}

void vADS129X_Avdd_Power_Down(void)
{
	 nrf_gpio_pin_clear(ADS1X9X_AVDD_PIN);
}

//ADS1291 - 3.3V
void vADS129X_Dvdd_Power_On(void)
{
	nrf_gpio_pin_set(ADS1X9X_DVDD_PIN);
}

//ADS1291
void vADS129X_Dvdd_Power_Down(void)
{
	 nrf_gpio_pin_clear(ADS1X9X_DVDD_PIN);
}

//ADS129X
void vAds129x_PowerEnable(Ads129x_power_t ePowerSwitch)
{	  
	if(ePowerSwitch == ADS129X_POWER_ON)
    {
        vADS129X_Avdd_Power_On();//AVDD 5V
        vADS129X_Dvdd_Power_On();//DVDD 3.3V
    }
    else
    {
         vADS129X_Avdd_Power_Down();//AVDD 5V
         vADS129X_Dvdd_Power_Down();//DVDD 3.3V
    }
	
	return;
} 
#endif

static void uart_task_function (void * pvParameter)
{
    uint8_t chip_id;
    vAds129x_PortInit();
    vAds129x_PowerEnable(ADS129X_POWER_ON);
    vADS129X_Spi_Init();
   
    #if 1
    vInit_ADS1x9x_DRDY_Interrupt();
	vADS1x9x_Enable_Start();// Set START pin to High
	vADS1x9x_CS_HIGH();
	vADS1x9x_Reset();//diff 
	vADS1x9x_Disable_Start();// Set START pin to LOW
	vAds1x9x_read_data_stop_continue();
    #endif
    
    while(1)
    {
        chip_id = u8ADS1x9x_Reg_Read(ADS1291_ID_CONFIG);
        NRF_LOG_INFO("--chip_id = %d\r\n", chip_id);
        nrf_delay_ms(1000);
    }
}
#endif

#ifdef HARDWARE_TEST_TOUCH_KEY
static void uart_task_function (void * pvParameter)
{
    nrf_gpio_cfg_output(10);
    nrf_gpio_pin_set(10);
    
    nrf_gpio_cfg_output(1);	
	nrf_delay_ms(20);
	nrf_gpio_pin_clear(1);
	nrf_delay_ms(10);
    
    nrf_gpio_cfg_input(7, NRF_GPIO_PIN_PULLUP);	
    
    while(1)
    {
       nrf_delay_ms(1000);
       NRF_LOG_INFO("touck_key:%d\r\n", nrf_gpio_pin_read(7));
    }
}
#endif

#ifdef HAREWARE_TEST_MOTOR
static void uart_task_function (void * pvParameter)
{
    nrf_gpio_cfg_output(3);
    nrf_gpio_pin_clear(3);
    nrf_gpio_cfg_input(7, NRF_GPIO_PIN_PULLUP);	
    
    while(1)
    {
       NRF_LOG_INFO("motor on\r\n");
       nrf_gpio_pin_set(3);
       nrf_delay_ms(1000);
       
#if 0        
       NRF_LOG_INFO("motor off\r\n");
       nrf_gpio_pin_clear(3);
       nrf_delay_ms(1000);
#endif
    }
}
#endif

int main(void)
{
    ret_code_t err_code;

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    
    /* Initialize clock driver for better time accuracy in FREERTOS */
    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_gpio_cfg_output(4);
    nrf_gpio_pin_clear(4);
    
    uart_task_function(NULL);
}
