#ifndef __STORE_PICUTRE_TO_FLASH_H__
#define __STORE_PICUTRE_TO_FLASH_H__

#define PICTURE_SIZE	((4 * 1024) * 30)
extern void store_picture_to_flash(uint8_t picture_id, uint8_t flag, const uint8_t *p_buffer, uint32_t length);
extern uint32_t get_pic_start_address(uint8_t picutre_id);

#endif
