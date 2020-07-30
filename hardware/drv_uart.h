#ifndef __DRV_UART_H__
#define __DRV_UART_H__

#define UART_RX_PIN_NUMBER   8
#define UART_TX_PIN_NUMBER   6
#define UART_RTS_PIN_NUMBER  5
#define UART_CTS_PIN_NUMBER  7

#define UART_RX_BUF_SIZE 1024
#define UART_TX_BUF_SIZE 1024

#define UART_COMMAND_HEADER_OFFSET			0
#define UART_COMMAND_SEQUENCE_OFFSET		2
#define UART_COMMAND_GROUP_ID_OFFSET		3
#define UART_COMMAND_CMD_ID_OFFSET			4
#define UART_COMMAND_PAYLOAD_LENGTH_OFFSET 	5
#define UART_COMMAND_PAYLOAD_OFFSET			7

#define UART_COMMAND_MIN_SIZE				9

#define UART_RX_WORKING_BUFFER_SIZE 1014

enum
{
	UART_RX_STATE_WAITING_FOR_HEADER = 0,
	UART_RX_STATE_WAITING_FOR_LENGTH,
	UART_RX_STATE_WAITING_FOR_MORE_DATA,
};

typedef void(*drv_uart_rx_command_handler_t)(uint8_t *p_data, uint16_t length);
typedef void(*drv_uart_tx_command_handler_t)(uint8_t *p_data, uint16_t length);
typedef void(*drv_uart_rx_buffer_check_t)(void);

typedef struct
{
    drv_uart_rx_command_handler_t drv_uart_rx_command_handler;//assign by other module
	drv_uart_tx_command_handler_t drv_uart_tx_command_handler;
	drv_uart_rx_buffer_check_t    drv_uart_rx_buffer_check;
}drv_uart_t;

extern void drv_uart_init(drv_uart_t *p_drv_uart);

#endif
