#ifndef __DRV_GT24L24A2Y_H__
#define __DRV_GT24L24A2Y_H__

#include <stdint.h>

#define GT24L24A2Y_SPI_CS_PIN_NUMBER			30
#define GT24L24A2Y_SPI_MOSI_PIN_NUMBER			26
#define GT24L24A2Y_SPI_MISO_PIN_NUMBER			27
#define GT24L24A2Y_SPI_SCLK_PIN_NUMBER			25

#define GT24L24A2Y_SPI_CS_HIGH		nrf_gpio_pin_set(GT24L24A2Y_SPI_CS_PIN_NUMBER)
#define GT24L24A2Y_SPI_CS_LOW		nrf_gpio_pin_clear(GT24L24A2Y_SPI_CS_PIN_NUMBER)
#define GT24L24A2Y_SPI_MOSI_HIGH	nrf_gpio_pin_set(GT24L24A2Y_SPI_MOSI_PIN_NUMBER)
#define GT24L24A2Y_SPI_MOSI_LOW		nrf_gpio_pin_clear(GT24L24A2Y_SPI_MOSI_PIN_NUMBER)
#define GT24L24A2Y_SPI_SCLK_HIGH	nrf_gpio_pin_set(GT24L24A2Y_SPI_SCLK_PIN_NUMBER)
#define GT24L24A2Y_SPI_SCLK_LOW		nrf_gpio_pin_clear(GT24L24A2Y_SPI_SCLK_PIN_NUMBER)

extern void r_dat_bat(unsigned long address, unsigned int byte_long, unsigned char *p_arr);
extern void drv_gt24l24a2y_write_page(unsigned long address,unsigned char *p,unsigned int number);
extern void drv_gt24l24a2y_sector_erase(unsigned long address);
extern void drv_gt24l24a2y_enter_low_power(void);
extern void drv_gt24l24a2y_exit_low_power(void);
extern void drv_gt24l24a2y_init(void);

#endif
