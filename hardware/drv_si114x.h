/*************************************************************************//**
 * @file
 * @brief Si114x function prototypes, structure and bit definitions   
 * @version 13.2.9
 *****************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 ******************************************************************************
 * 
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *  
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *  
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgement in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 *
 *****************************************************************************/

#ifndef SI114X_FUNCTIONS_H
#define SI114X_FUNCTIONS_H

/***************************************************************************//**
 * @addtogroup Drivers
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup Si114x
 * @{
 ******************************************************************************/

#include <stdint.h>
#include "nrf_drv_twi.h"
#include "sdk_errors.h"
#include "app_util_platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 *******************************   STRUCTS   ***********************************
 ******************************************************************************/

#define Auto_On_Off_Test   0 //自动测试宏，打开后，自动启动心率传感器，运行10秒后，再隔一秒后停止运行心率传感器。循环。
#define Si114X_JUDGE_SAMP  0 // 1 - 验证采样率寄存器  0 - 不验证
#define COUNT_DATA_NUM     0 //调试数据是否有丢包宏 0 - 不统计数据   1 - 统计数据
#define PRINT_RAW_DATA     0 // 打印Si114x_HeartData_Buf里面的数据宏  0 - 不打印    1 - 打印
#define TIME_SAMPS_DEBUG   0 //打印相隔2次进入心率中断的时间戳,单位是毫秒。
	
/*********************************
* si114x的测量率，即在自发运行模式中，1s中可以采集多少个数据点
0xa4 every 40.0 ms  - 25HZ 
0xa0 every 30.0 ms  - 33HZ 
0x94 every 20.0 ms  - 50HZ
0x99 every 25.0 ms  - 40HZ
0x84 every 10.4 ms  - 100HZ
0x74 every  5.2 ms
0x70 every  4.2 ms
0x60 every  3.2 ms       
*********************************/
#define SI114X_MEASURE_RATE 0xA4

#if Auto_On_Off_Test
void si114x_On_Off_timer_start(void);
void si114x_On_Off_timer_stop(void);
void si114x_Auto_On_Off_timer_init(void);
#endif
	
/*******************************************************************************
 ************************** Si114x I2C Registers *******************************
 ******************************************************************************/
/// @cond DOXYGEN_SHOULD_SKIP_THIS
#define REG_PART_ID               0x00	
#define REG_REV_ID                0x01
#define REG_SEQ_ID                0x02
#define REG_INT_CFG               0x03
#define REG_IRQ_ENABLE            0x04
#define REG_IRQ_MODE1             0x05
#define REG_IRQ_MODE2             0x06
#define REG_HW_KEY                0x07
#define REG_MEAS_RATE             0x08
#define REG_ALS_RATE              0x09
#define REG_PS_RATE               0x0A
#define REG_PS_LED21              0x0F
#define REG_PS_LED3               0x10
#define REG_PARAM_WR              0x17
#define REG_COMMAND               0x18
#define REG_RESPONSE              0x20
#define REG_IRQ_STATUS            0x21
#define REG_IRQ_PS1_DATA0         0x26
#define REG_IRQ_PS1_DATA1         0x27
#define REG_CHIP_STAT             0x30
#define REG_PARAM_RD              0x2E

/*******************************************************************************
 ************************** Si114x I2C Parameter Offsets ***********************
 ******************************************************************************/
/// @cond DOXYGEN_SHOULD_SKIP_THIS
#define PARAM_I2C_ADDR            0x00
#define PARAM_CH_LIST             0x01              // used
#define PARAM_PSLED12_SELECT      0x02
#define PARAM_PSLED3_SELECT       0x03
#define PARAM_FILTER_EN           0x04
#define PARAM_PS_ENCODING         0x05
#define PARAM_ALS_ENCODING        0x06
#define PARAM_PS1_ADC_MUX         0x07
#define PARAM_PS2_ADC_MUX         0x08
#define PARAM_PS3_ADC_MUX         0x09
#define PARAM_PS_ADC_COUNTER      0x0A
#define PARAM_PS_ADC_CLKDIV       0x0B
#define PARAM_PS_ADC_GAIN         0x0B
#define PARAM_PS_ADC_MISC         0x0C
#define PARAM_VIS_ADC_MUX         0x0D
#define PARAM_IR_ADC_MUX          0x0E
#define PARAM_AUX_ADC_MUX         0x0F
#define PARAM_ALSVIS_ADC_COUNTER  0x10
#define PARAM_ALSVIS_ADC_CLKDIV   0x11
#define PARAM_ALSVIS_ADC_GAIN     0x11
#define PARAM_ALSVIS_ADC_MISC     0x12
#define PARAM_ALS_HYST            0x16
#define PARAM_PS_HYST             0x17
#define PARAM_PS_HISTORY          0x18
#define PARAM_ALS_HISTORY         0x19
#define PARAM_ADC_OFFSET          0x1A
#define PARAM_SLEEP_CTRL          0x1B
#define PARAM_LED_RECOVERY        0x1C
#define PARAM_ALSIR_ADC_COUNTER   0x1D
#define PARAM_ALSIR_ADC_CLKDIV    0x1E
#define PARAM_ALSIR_ADC_GAIN      0x1E
#define PARAM_ALSIR_ADC_MISC      0x1F
/// @endcond

// REG_IRQ_CFG 
#define  ICG_INTOE                0x01
#define  ICG_INTMODE              0x02

// REG_IRQ_ENABLE
// REG_IRQ_STATUS
#define IE_NONE                   0x00

#define PSLED12_SELECT            0x02
#define PSLED3_SELECT             0x03

#define IE_ALS_NONE               0x00
#define IE_ALS_EVRYSAMPLE         0x01
#define IE_ALS_EXIT_WIN           0x01
#define IE_ALS_ENTER_WIN          0x02

#define IE_PS1_NONE               0x00
#define IE_PS1_EVRYSAMPLE         0x04
#define IE_PS1_CROSS_TH           0x04
#define IE_PS1_EXCEED_TH          0x04
#define IE_PS1                    0x04

#define IE_PS2_NONE               0x00
#define IE_PS2_EVRYSAMPLE         0x08
#define IE_PS2_CROSS_TH           0x08
#define IE_PS2_EXCEEED_TH         0x08
#define IE_PS2                    0x08

#define IE_PS3_NONE               0x00
#define IE_PS3_EVRYSAMPLE         0x10
#define IE_PS3_CROSS_TH           0x10
#define IE_PS3_EXCEED_TH          0x10
#define IE_PS3                    0x10

// REG_IRQ_MODE1
#define IM1_NONE                  0x00
#define IM1_ALS_NONE              0x00
#define IM1_ALS_EVRYSAMPLE        0x00
#define IM1_ALS_VIS_EXIT          0x01
#define IM1_ALS_VIS_ENTER         0x05
#define IM1_ALS_IR_EXIT           0x03
#define IM1_ALS_IR_ENTER          0x06

#define IM1_PS1_NONE              0x00
#define IM1_PS1_EVRYSAMPLE        (0x0<<4)
#define IM1_PS1_CROSS_TH          (0x1<<4)
#define IM1_PS1_EXCEED_TH         (0x3<<4)

#define IM1_PS2_NONE              0x00
#define IM1_PS2_EVRYSAMPLE        (0x0<<6)
#define IM1_PS2_CROSS_TH          (0x1<<6)
#define IM1_PS2_EXCEED_TH         (0x3<<6)


// REG_IRQ_MODE1
#define IM2_PS3_NONE              0x00
#define IM2_PS3_EVRYSAMPLE        (0x0)
#define IM2_PS3_CROSS_TH          (0x1)
#define IM2_PS3_EXCEED_TH         (0x3)
	
// Hardware Key value
// REG_HW_KEY
#define HW_KEY_VAL0               0x17	
	

// PARAM_CH_LIST
#define PS1_TASK                  0x01
#define PS2_TASK                  0x02
#define PS3_TASK                  0x04
#define ALS_VIS_TASK              0x10
#define ALS_IR_TASK               0x20
#define AUX_TASK                  0x40

//
// REG_PS_LED21   LED2 Current is upper nibble
//                LED1 Current is lower nibble 
// REG_PS_LED3    LED3 Current is lower nibble

#if 0
	#define LEDI_000                  0x00
	#define LEDI_006                  0x01
	#define LEDI_011                  0x02
	#define LEDI_022                  0x03
	#define LEDI_045                  0x04
	#define LEDI_067                  0x05
	#define LEDI_090                  0x06
	#define LEDI_112                  0x07
	#define LEDI_135                  0x08
	#define LEDI_157                  0x09
	#define LEDI_180                  0x0A
	#define LEDI_202                  0x0B
	#define LEDI_224                  0x0C
	#define LEDI_269                  0x0D
	#define LEDI_314                  0x0E
	#define LEDI_359                  0x0F
#endif

typedef enum
{
	 GREEN_LED1 = 0x00,
	 GREEN_LED2,
	 GREEN_LED3
}Si114x_green_led_t;


typedef enum
{
	AUTO_ON = 0x01,
  AUTO_OFF = 0x00
}Si114x_auto_t;

/*
	REG_PS_LED21   LED2 Current is upper nibble
								 LED1 Current is lower nibble 
	REG_PS_LED3    LED3 Current is lower nibble
	LED驱动电流大小：LEDI_011 - 表示驱动电流大概为11mA.
*/
typedef enum
{
	 LEDI_000 = 0x00,
	 LEDI_006,
	 LEDI_011,
	 LEDI_022,
	 LEDI_045,
	 LEDI_067,
	 LEDI_090,
	 LEDI_112,
	 LEDI_135,
	 LEDI_157,
	 LEDI_180,
	 LEDI_202,
	 LEDI_224,
	 LEDI_269,
	 LEDI_314,
	 LEDI_359
}Si114x_led_current_t;

typedef enum
{
	 SI114X_STOP_INIT = 0x00,
	 SI114X_STOP_STAR,
	 SI114X_STOP_END,
}Si114x_STOP_t;


uint32_t twi_master_Si114x_init(void);
uint8_t findSensors(void);
uint8_t si114x_init(void);
uint32_t twi_master_Si114x_uinit(void);
void Si114x_INT_Init(void);
void Si114x_vLED_Init(void);
void Si114x_vLED_Enable(void);
void Si114x_vLED_Disable(void);
void Process_Si114x_INT(void);
uint8_t Si114x_Read(nrf_drv_twi_t si114x_handle, uint8_t *addr, uint8_t *outdata, uint8_t size);
uint8_t Si114x_SetLedCurrent(nrf_drv_twi_t si114x_handle, Si114x_green_led_t greenLed, Si114x_led_current_t ledCurrent);
uint8_t WriteAndRead(void);
uint8_t Auto_Measurement_Set(Si114x_auto_t autoFlg);
uint8_t SetSi114x_Current(uint8_t uCurrentData);
void Si114x_ClrRxFifo(void);

#ifdef __cplusplus
}
#endif

/** @} (end group Si114x) */
/** @} (end group Drivers) */

#endif // #define SI114X_FUNCTIONS_H

	
