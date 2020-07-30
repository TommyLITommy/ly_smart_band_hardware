#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "app_uart.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "sys_info.h"
#include "hardware.h"
#include "drv_uart.h"

static uint8_t uart_rx_current_state;
static uint8_t uart_rx_working_buffer[UART_RX_WORKING_BUFFER_SIZE];
static uint16_t uart_rx_working_buffer_index;
static uint16_t uart_rx_command_length;

static void uart_recieve(uint8_t data)
{
	if(uart_rx_working_buffer_index >= UART_RX_WORKING_BUFFER_SIZE)
	{
		uart_rx_working_buffer_index = 0;
        uart_rx_current_state = UART_RX_STATE_WAITING_FOR_HEADER;
	}
	
	uart_rx_working_buffer[uart_rx_working_buffer_index++] = data;
	
	switch(uart_rx_current_state)
	{
		case UART_RX_STATE_WAITING_FOR_HEADER:
			if(0x55 == uart_rx_working_buffer[0])
			{
				if(0x02 == uart_rx_working_buffer_index)
				{
					if(0xAA == uart_rx_working_buffer[1])
					{
						uart_rx_current_state = UART_RX_STATE_WAITING_FOR_LENGTH;
					}
					else
					{
						uart_rx_working_buffer_index = 0;
					}
				}
			}
			else
			{
				uart_rx_working_buffer_index = 0;
			}
			break;
		case UART_RX_STATE_WAITING_FOR_LENGTH:
			if(UART_COMMAND_PAYLOAD_OFFSET == uart_rx_working_buffer_index)
			{
				uart_rx_command_length = UART_COMMAND_MIN_SIZE + (uint16_t)(uart_rx_working_buffer[UART_COMMAND_PAYLOAD_LENGTH_OFFSET] << 0) + (uart_rx_working_buffer[UART_COMMAND_PAYLOAD_LENGTH_OFFSET + 1] << 8);
                NRF_LOG_INFO("length:%04X\r\n", uart_rx_command_length);
                if(uart_rx_command_length >= UART_RX_WORKING_BUFFER_SIZE)
                {
                    uart_rx_working_buffer_index = 0;
                    uart_rx_current_state = UART_RX_STATE_WAITING_FOR_HEADER;
                }
                else
                {
                    uart_rx_current_state = UART_RX_STATE_WAITING_FOR_MORE_DATA;
                }
            }
			break;
		case UART_RX_STATE_WAITING_FOR_MORE_DATA:
			if(uart_rx_working_buffer_index == uart_rx_command_length)
			{
                //Do crc16 check here?
				//Handle command
				//uart_rx_command_handler(uart_rx_working_buffer, uart_rx_command_length);
				
                NRF_LOG_INFO("handle rx data\r\n");
                
                sys_info.hardware.drv_uart.drv_uart_rx_command_handler(uart_rx_working_buffer, uart_rx_command_length);
                
				uart_rx_working_buffer_index = 0;
				uart_rx_command_length = 0;
				uart_rx_current_state = UART_RX_STATE_WAITING_FOR_HEADER;
			}
			break;
		default:
			break;
	}
}

static void uart_event_handle(app_uart_evt_t * p_event)
{
    uint32_t err_code;
	uint8_t uart_rx_data;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
			err_code = app_uart_get(&uart_rx_data);
            //NRF_LOG_INFO("uart_rx_data:%02X\r\n", uart_rx_data);
			
            if(err_code == NRF_SUCCESS)
			{
				uart_recieve(uart_rx_data);
			}
			
            break;

        case APP_UART_COMMUNICATION_ERROR:
			NRF_LOG_INFO("uart_error:0x%02X\r\n", p_event->data.error_communication);
            //APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}

static void drv_uart_rx_buffer_check(void)
{

}

static void drv_uart_tx_command_handler(uint8_t *p_data, uint16_t length)
{
    for(uint16_t i = 0; i < length; i++)
    {
        app_uart_put(p_data[i]);
    }
}

void drv_uart_init(drv_uart_t *p_drv_uart)
{
	uint32_t                     err_code;
    
    NRF_LOG_INFO("drv_uart_init\r\n");
    
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = UART_RX_PIN_NUMBER,
        .tx_pin_no    = UART_TX_PIN_NUMBER,
        .rts_pin_no   = UART_RTS_PIN_NUMBER,
        .cts_pin_no   = UART_CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
	
    p_drv_uart->drv_uart_tx_command_handler = drv_uart_tx_command_handler;
	p_drv_uart->drv_uart_rx_buffer_check    = drv_uart_rx_buffer_check;
    
	uart_rx_current_state = UART_RX_STATE_WAITING_FOR_HEADER;
    uart_rx_working_buffer_index = 0;
    uart_rx_command_length = 0;
}
