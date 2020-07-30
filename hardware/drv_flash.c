#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "drv_gt24l24a2y.h"
#include "drv_flash.h"

void drv_flash_erase_all(void)
{
	for(uint16_t i = 0; i < FLASH_TOTAL_SECOTR_COUNT; i++)
	{
		drv_gt24l24a2y_sector_erase(FLASH_FIRST_SECTOR_START_ADDRESS + i * FLASH_SECTOR_SIZE);
	}
}

void drv_flash_erase_sector(uint32_t sector_start_address)
{
	//Address Check！！！
	drv_gt24l24a2y_sector_erase(sector_start_address);
}

void drv_flash_write(uint32_t address, uint8_t *p_buffer, uint32_t length)
{
	uint32_t write_length;
	uint32_t index = 0;
	
	while(length)
	{
		write_length = 256 - address % 256;
		if(length < write_length)
		{
			write_length = length;
		}
		
		//Call write function
		drv_gt24l24a2y_write_page(address, &p_buffer[index], write_length);
		length 	-= write_length;
		address += write_length;
		index   += write_length;
	}
}

void drv_flash_read(uint32_t address, uint8_t *p_buffer, uint32_t length)
{
    if(address < 0x200000)
    {
        return;
    }
    
    r_dat_bat(address, length, p_buffer);
}

void drv_flash_init(void)
{
	
}
