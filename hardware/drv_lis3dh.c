#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_delay.h"
#include "drv_lis3dh.h"

#define G_SENSOR_MOSI_PIN_NUMBER  	26
#define G_SENSOR_CS_PIN_NUMBER	    28
#define G_SENSOR_SCLK_PIN_NUMBER	25
#define G_SENSOR_MISO_PIN_NUMBER	27

#define	G_SENSOR_SPI_CS_SET			nrf_gpio_pin_set(G_SENSOR_CS_PIN_NUMBER);
#define	G_SENSOR_SPI_CS_CLR			nrf_gpio_pin_clear(G_SENSOR_CS_PIN_NUMBER);

#define	G_SENSOR_SPI_MOSI_SET		nrf_gpio_pin_set(G_SENSOR_MOSI_PIN_NUMBER);		
#define	G_SENSOR_SPI_MOSI_CLR		nrf_gpio_pin_clear(G_SENSOR_MOSI_PIN_NUMBER);

#define	G_SENSOR_SPI_SCLK_SET		nrf_gpio_pin_set(G_SENSOR_SCLK_PIN_NUMBER);	
#define	G_SENSOR_SPI_SCLK_CLR		nrf_gpio_pin_clear(G_SENSOR_SCLK_PIN_NUMBER);

uint8_t lis3dh_spi_send_byte(uint8_t data)
{
	uint8_t i;
	for(i = 0; i < 8; i++)
	{
		if(0x80 & data) //MSB bit is transfered first
		{
			G_SENSOR_SPI_MOSI_SET;
		}
		else 
		{
			G_SENSOR_SPI_MOSI_CLR;
		}
		G_SENSOR_SPI_SCLK_CLR;
		nrf_delay_us(10);
		//临时将delay的时间从100改成2
		//cpu_delay(100);
		G_SENSOR_SPI_SCLK_SET;
		nrf_delay_us(5);
		data <<= 1;
		if(nrf_gpio_pin_read(G_SENSOR_MISO_PIN_NUMBER))
		{
			data++;
		}
	}
	return data;
}

static uint8_t lis3dh_spi_read(uint8_t address)
{
	uint8_t data;
	G_SENSOR_SPI_CS_CLR
	lis3dh_spi_send_byte(0x80 | address);
	data = lis3dh_spi_send_byte(0x00);
	G_SENSOR_SPI_CS_SET
	return data;
}

static void lis3dh_spi_init(void)
{
	nrf_gpio_cfg_output(G_SENSOR_MOSI_PIN_NUMBER);
	nrf_gpio_cfg_output(G_SENSOR_CS_PIN_NUMBER);
	nrf_gpio_cfg_output(G_SENSOR_SCLK_PIN_NUMBER);
	nrf_gpio_cfg_input(G_SENSOR_MISO_PIN_NUMBER, NRF_GPIO_PIN_PULLUP);
    
    nrf_gpio_pin_set(G_SENSOR_MOSI_PIN_NUMBER);
    nrf_gpio_pin_set(G_SENSOR_CS_PIN_NUMBER);
    nrf_gpio_pin_set(G_SENSOR_SCLK_PIN_NUMBER);   
}

#define LIS3DH_WHO_AM_I				0x0F

void who_am_i_check(void)
{
	uint8_t i;
	uint8_t who_am_i;

	who_am_i = lis3dh_spi_read(LIS3DH_WHO_AM_I);	
    NRF_LOG_INFO("who_am_i:%d\r\n", who_am_i);
}

void drv_lis3dh_init(void)
{
	lis3dh_spi_init();
	who_am_i_check();
	
}
