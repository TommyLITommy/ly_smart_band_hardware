#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_delay.h"

#include "drv_gt24l24a2y.h"
#include "drv_flash.h"
#include "store_picture_to_flash.h"

void store_picture_to_flash(uint8_t picture_id, uint8_t flag, const uint8_t *p_buffer, uint32_t length)
{
	uint8_t temp;
	uint32_t pic_start_address; 
	pic_start_address = FLASH_FIRST_SECTOR_START_ADDRESS + picture_id * PICTURE_SIZE;
	drv_flash_read(pic_start_address, &temp, 1);
    NRF_LOG_INFO("temp1 = %d, flag = %d\r\n", temp, flag);
	
    #if 0
    drv_flash_erase_sector(pic_start_address);
    drv_flash_write(pic_start_address, &flag, 1);//This flag must be written at the end!!
    drv_flash_read(pic_start_address, &temp, 1);
    NRF_LOG_INFO("temp2 = %d\r\n", temp);
    #else
    if(flag != temp)
	{
        NRF_LOG_INFO("flash pic start\r\n");
		for(uint8_t i = 0; i < 30; i++)
		{
			drv_flash_erase_sector(pic_start_address + i * FLASH_SECTOR_SIZE);
		}
		
        nrf_delay_ms(1000);
        
		drv_flash_write(pic_start_address + 1, (uint8_t *)p_buffer, length);
        //nrf_delay_ms(10);
        drv_flash_write(pic_start_address, &flag, 1);//This flag must be written at the end!!
        //nrf_delay_ms(10);
        drv_flash_read(pic_start_address, &temp, 1);
        //nrf_delay_ms(10);
        NRF_LOG_INFO("flash pic done\r\n");
        NRF_LOG_INFO("temp:%d\r\n", temp);
	}
    else
    {
        NRF_LOG_INFO("pic ok\r\n");
    }
    #endif
}

uint32_t get_pic_start_address(uint8_t picture_id)
{
	uint32_t pic_start_address;
	pic_start_address = FLASH_FIRST_SECTOR_START_ADDRESS + picture_id * PICTURE_SIZE;
	return pic_start_address;
}

