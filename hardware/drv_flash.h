#ifndef __DRV_FLASH_H__
#define __DRV_FLASH_H__

#define FLASH_SECTOR_SIZE	(4 * 1024)//4k
#define FLASH_FIRST_SECTOR_START_ADDRESS (0x200000 + FLASH_SECTOR_SIZE * 2)
#define FLASH_TOTAL_SECOTR_COUNT	510

extern void drv_flash_erase_all(void);
extern void drv_flash_erase_sector(uint32_t sector_start_address); 
extern void drv_flash_write(uint32_t address, uint8_t *p_buffer, uint32_t length); 
extern void drv_flash_read(uint32_t address, uint8_t *p_buffer, uint32_t length);
extern void drv_flash_init(void); 

#endif
