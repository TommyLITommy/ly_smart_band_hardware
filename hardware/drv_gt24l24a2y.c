#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "drv_gt24l24a2y.h"

void font_spi_send_byte(uint32_t cmd)
{
	uint8_t i;
	cmd |= 0x03000000;
	for(i = 0; i < 32; i++)
	{
		for(i = 0; i < 32; i++)
		{
			GT24L24A2Y_SPI_SCLK_LOW;
			if(cmd & 0x80000000)
			{
				GT24L24A2Y_SPI_MOSI_HIGH;
			}
			else
			{
				GT24L24A2Y_SPI_MOSI_LOW;
			}
			GT24L24A2Y_SPI_SCLK_HIGH;
			cmd <<= 1;
		}
	}
}

uint8_t font_spi_read_byte(void)
{
	uint8_t i;
	uint8_t data = 0;
	GT24L24A2Y_SPI_SCLK_HIGH;
	for(i = 0; i < 8; i++)
	{
		GT24L24A2Y_SPI_SCLK_LOW;
		data <<= 1;
		if(nrf_gpio_pin_read(GT24L24A2Y_SPI_MISO_PIN_NUMBER))
		{
			data |= 0x01;
		}
		else
		{
			data &= 0xFE;
		}
		GT24L24A2Y_SPI_SCLK_HIGH;
	}
	return data;
}

void r_dat_bat(unsigned long address, unsigned int byte_long, unsigned char *p_arr)
{
	uint32_t i;
	GT24L24A2Y_SPI_CS_LOW;
	font_spi_send_byte(address);
	for(i = 0; i< byte_long; i++)
	{
		p_arr[i] = font_spi_read_byte();
	}
	GT24L24A2Y_SPI_CS_HIGH;
}

//Program relative function
static void drv_gt24l24a2y_send_data(uint8_t data)
{
    for(uint8_t i = 0; i < 8; i++)
    {
        GT24L24A2Y_SPI_SCLK_LOW;
        if(((data << i) & 0x80) == 0)
        {
            GT24L24A2Y_SPI_MOSI_LOW;
        }
        else
        {
            GT24L24A2Y_SPI_MOSI_HIGH;
        }
        GT24L24A2Y_SPI_SCLK_HIGH;
    }    
}

static uint8_t drv_gt24l24a2y_read_byte(void)
{
    uint8_t i;
    uint8_t read_data;
    
    GT24L24A2Y_SPI_SCLK_HIGH;
    for(i = 0; i < 8; i++)
    {
        GT24L24A2Y_SPI_SCLK_LOW;
        read_data <<= 1;
		if(nrf_gpio_pin_read(GT24L24A2Y_SPI_MISO_PIN_NUMBER))
		{
			read_data |= 0x01;
		}
		else
		{
			read_data &= 0xFE;
		}
		GT24L24A2Y_SPI_SCLK_HIGH;
    }
    
    return read_data;
}

static void drv_gt24l24a2y_check_status(void)
{
    uint8_t temp;
    GT24L24A2Y_SPI_CS_LOW;
    while(1)
    {
        drv_gt24l24a2y_send_data(0x05);
        temp = drv_gt24l24a2y_read_byte();
        if((temp & 0x01) == 0x0)
        {
            break;
        }
    }
    GT24L24A2Y_SPI_CS_HIGH;
}

static void drv_gt24l24a2y_write_enable(void)
{
    GT24L24A2Y_SPI_CS_LOW;
    drv_gt24l24a2y_send_data(0x06);
    GT24L24A2Y_SPI_CS_HIGH;
}

void drv_gt24l24a2y_write_page(unsigned long address,unsigned char *p,unsigned int number)
{
    uint32_t i;
    uint8_t addr0, addr1, addr2;
    
    if(address < 0x200000)
    {
        while(1)
        {}
        return;
    }
    
    addr0 = (uint8_t)(address >> 16);
    addr1 = (uint8_t)(address >> 8);
    addr2 = (uint8_t)(address >> 0);
    
    drv_gt24l24a2y_write_enable();
    GT24L24A2Y_SPI_CS_LOW;
    drv_gt24l24a2y_send_data(0x02);
    drv_gt24l24a2y_send_data(addr0);
    drv_gt24l24a2y_send_data(addr1);
    drv_gt24l24a2y_send_data(addr2);
    
    for(i = 0; i < number; i++)
    {
        drv_gt24l24a2y_send_data(*(p++));
    }
    GT24L24A2Y_SPI_CS_HIGH;
    
    drv_gt24l24a2y_check_status();
}

void drv_gt24l24a2y_sector_erase(unsigned long address)
{
    uint8_t addr0, addr1, addr2;
    
    if(address < 0x200000)
    {
        while(1)
        {}
        return;
    }
    
    addr0 = (uint8_t)(address >> 16);
    addr1 = (uint8_t)(address >> 8);
    addr2 = (uint8_t)(address >> 0);
    
    drv_gt24l24a2y_write_enable();
    GT24L24A2Y_SPI_CS_LOW;
    drv_gt24l24a2y_send_data(0x20);
    drv_gt24l24a2y_send_data(addr0);
    drv_gt24l24a2y_send_data(addr1);
    drv_gt24l24a2y_send_data(addr2);
    GT24L24A2Y_SPI_CS_HIGH;
    drv_gt24l24a2y_check_status();
}

//Change to inline function
void drv_gt24l24a2y_enter_low_power(void)
{
	
}

void drv_gt24l24a2y_exit_low_power(void)
{
	
}

void drv_gt24l24a2y_init(void)
{
	nrf_gpio_cfg_output(GT24L24A2Y_SPI_CS_PIN_NUMBER);
	nrf_gpio_cfg_output(GT24L24A2Y_SPI_MOSI_PIN_NUMBER);
	nrf_gpio_cfg_output(GT24L24A2Y_SPI_SCLK_PIN_NUMBER);
	nrf_gpio_cfg_input(GT24L24A2Y_SPI_MISO_PIN_NUMBER, NRF_GPIO_PIN_PULLUP);
    
    #if 1//This is very important! after init, we should set cs and sclk to high
    nrf_gpio_pin_set(GT24L24A2Y_SPI_CS_PIN_NUMBER);
    nrf_gpio_pin_set(GT24L24A2Y_SPI_MOSI_PIN_NUMBER);
    nrf_gpio_pin_set(GT24L24A2Y_SPI_SCLK_PIN_NUMBER);
    #endif
    
    nrf_delay_ms(50);
}
