#ifndef __UART_PROTOCOL_H__
#define __UART_PROTOCOL_H__

#include <stdint.h>
#include <string.h>
#include <stdio.h>

#define COMMAND_HEADER_OFFSET           0
#define COMMAND_SEQUENCE_OFFSET         2
#define COMMAND_GROUP_ID_OFFSET         3
#define COMMAND_COMMAND_ID_OFFSET       4
#define COMMAND_PAYLOAD_LENGTH_OFFSET   5
#define COMMAND_PAYLOAD_OFFSET          7

typedef enum
{
    GROUP_ID_HARDWARE,//下位机硬件外设控制Group命令
    GROUP_ID_BLE,     //BLE相关命令
    GROUP_ID_BLE_P,   //BLE Peripheral
    GROUP_ID_BLE_C,   //BLE Central
    GROUP_ID_BLE_MESH,
    GROUP_ID_BLE_AUDIO,
    GROUP_ID_ACK,
    GROUP_ID_DEBUG,   //调试log输出
}group_id;

typedef enum
{
    GID_HARDWARE_CID_PTD_LCD_FILL,
    GID_HARDWARE_CID_PTD_LCD_DRAW_POINT,
    GID_HARDWARE_CID_PTD_LCD_DRAW_LINE,
    GID_HARDWARE_CID_PTD_LCD_DRAW_RECTANGLE,
    GID_HARDWARE_CID_PTD_LCD_DRAW_PICTURE,
    GID_HARDWARE_CID_PTD_MEMS_CALIBRATION
}gid_hardware_cid_ptd_t;//ptd-->pc to device

typedef enum
{
    GID_HARDWARE_CID_DTP_NONE,
    GID_HARDWARE_CID_DTP_3_AXIS_RAW_DATA_REPORT,
    GID_HARDWARE_CID_DTP_6_AXIS_RAW_DATA_REPORT,
    GID_HARDWARE_CID_DTP_9_AXIS_RAW_DATA_REPORT,
    GID_HARDWARE_CID_DTP_PITCH_YAW_ROLL_REPORT,
    GID_HARDWARE_CID_DTP_ECG_RAW_DATA_REPORT,
    GID_HARDWARE_CID_DTP_PPG_RAW_DATA_REPORT,
}gid_hardware_cid_dtp_t;//dtp-->device to pc

#define WORKING_BUFFER_LENGTH 1024

#define PC_TO_DEVICE_HEADER 0xAA55
#define DEVICE_TO_PC_HEADER 0x55AA

typedef void(*uart_protocol_send_ack_t)(uint8_t group_id, uint8_t command_id);
typedef void(*uart_protocol_rx_handler_t)(uint8_t *p_data, uint16_t length);
typedef void(*uart_protocol_assemble_command_and_send_t)(uint8_t group_id, uint8_t command_id, uint8_t *p_payload, uint16_t payload_length);

typedef struct 
{
	uart_protocol_send_ack_t uart_protocol_send_ack;
	uart_protocol_rx_handler_t uart_protocol_rx_handler;
    uart_protocol_assemble_command_and_send_t uart_protocol_assemble_command_and_send;
}uart_protocol_t;

extern void uart_protocol_init(uart_protocol_t *p_uart_protocol);

#endif
