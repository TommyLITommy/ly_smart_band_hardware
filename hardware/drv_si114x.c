#include "drv_si114x.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "heart_rate_data_handle.h"
#include "display_character.h"
#include "nrf_drv_gpiote.h"
#include "app_error.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include <stdio.h>

#include "app_timer.h"

#include "global_config.h"
#include "ble_nus.h"
#include "log_print.h"
#include "watch_event_queue.h"

#include "watch_mode_heart_rate.h"
#include "watch_mode_blood_pressure.h"
#include "whole_day_measure.h"
#include "pin_definition.h"

/************************************************************
* 版本：V1.3
* 作者: zyh
* 时间: 2018年4月2日
* 功能：si1142心率传感器驱动
************************************************************/

#ifdef ENABLE_COLLECTING_DATA_FOR_ANALYSIS
extern ble_nus_t		 	m_nus;
#endif

#if COUNT_DATA_NUM
uint32_t gu32Si114xIntNum = 0; //进入中断的次数
uint32_t gu32Si114xGetFifoNum = 0; //从FIFO里取数据的次数
uint32_t gu32Si114xAdjustNum = 0;//波形调整的次数
#endif

#define Si1142_PART_ID            0x42  //si1142的PART ID
#define Si114x_I2C_ADDRESS        0x5A  //slave address 

/*
#define Si114x_INT_PIN            11    //中断输出引脚  
#define Si114x_SDA_PIN            12    //I2C数据线
#define Si114x_SCL_PIN            13    //I2C时钟线
#define Si114x_vLED_EN_PIN        14    //Si114x LED供电使能引脚
*/

#define MASTER_TWI_INSTANCE       0    //!< TWI interface used as a master accessing Si114x

#define LOOP_TIMEOUT_MS           20   //200

static const nrf_drv_twi_t m_si114xTwi_master = NRF_DRV_TWI_INSTANCE(MASTER_TWI_INSTANCE);


#if TIME_SAMPS_DEBUG
	APP_TIMER_DEF(si114x_Timestamps_timer_id);//1ms时间戳定时器ID - 用于验证si1142的采样率
	static uint32_t gu32Counter_1ms_Count = 0;
	static uint32_t gu32Timestamp1 = 0;
	static uint32_t gu32Timestamp2 = 0;
	#define SI114X_TIMESTAMPS_INTERVAL_1ms		 APP_TIMER_TICKS(1) 
#endif


/***************************************************************************
* 心率传感器接接收数据的队列
*
***************************************************************************/

#define  SI114X_RXBUF_SIZE  250    //接收FIFO最大容量
static uint16_t Si114x_HeartData_Buf[SI114X_RXBUF_SIZE];//接收FIFIO缓冲区数组
static uint16_t Si114x_Rx_IndexR = 0;//接收FIFO的读指针
static uint16_t Si114x_Tx_IndexW = 0;//接收FIFO 的写指针


#define WAVE_A_GRADE   0x00  //A等级的波形
#define WAVE_B_GRADE   0x01  //B等级的波形
#define WAVE_C_GRADE   0x02  //C等级的波形
#define WAVE_D_GRADE   0x03  //D等级的波形

static uint8_t uwaveformLevel = WAVE_A_GRADE;
#define SI114X_LEVLE_NUM 4  //波形等级数
uint8_t guWave[SI114X_LEVLE_NUM][2] = {
	                                      //增益，驱动电流
	                                      {0x02, 0x04},  //A等级波形对应的参数  04  04
																				{0x03, 0x04},  //B等级波形对应的参数  02  04
																				{0x04, 0x04},  //C等级波形对应的参数  03  04
																				{0x01, 0x09},  //D等级波形对应的参数  01  09
                                      };

static uint8_t _sendCmd(nrf_drv_twi_t si114x_handle, uint8_t command);
static uint8_t Si114xParamRead(nrf_drv_twi_t si114x_handle, uint8_t address, uint8_t *outData);
																			
/*******************************************************************************
 * 心率数据接收FIFO初始化
 * 
 ******************************************************************************/
void Si114x_ClrRxFifo(void)
{
		Si114x_Rx_IndexR  = 0;
    Si114x_Tx_IndexW  = 0;
}

//接收缓冲区为空
// 1 - 空   0 - 不为空
uint8_t Si114x_Fifo_isempty(void)
{
    if(Si114x_Rx_IndexR == Si114x_Tx_IndexW ) return 1; 
 
    return 0;
}

/*******************************************************************************
* 功能：从缓冲队列内读取1字节已接收的数据
* 入口参数：读取数据所存放的地址指针
* 返回： 1 - 读取成功， 0 - 读取失败
*******************************************************************************/
uint8_t Si114x_ReadFifo_Data(uint16_t *udata)
{
	  if(Si114x_Fifo_isempty())
		{
				return  0;  //如果FIFO内无数据返回0
		}
		
		//涉及FIFO操作时不允许中断，以免指针错乱。
		*udata = Si114x_HeartData_Buf[Si114x_Rx_IndexR];
		if(++Si114x_Rx_IndexR >= SI114X_RXBUF_SIZE)
		{
				Si114x_Rx_IndexR = 0;
		}
		//FIFO操作完毕，恢复中断允许

		return 1;
}

/*******************************************************************************
* 功能：从缓冲队列内读取1字节已接收的数据
* 入口参数：读取数据所存放的地址指针
* 返回： 无
*******************************************************************************/
void Si114x_WriteFifo_Data(uint16_t udata)
{
		Si114x_HeartData_Buf[Si114x_Tx_IndexW] = udata;
		if(++Si114x_Tx_IndexW >= SI114X_RXBUF_SIZE)
		{
			 Si114x_Tx_IndexW = 0;	
		}
    
		return;
}


/******************************************************************************
* @brief 
*   功能：I2C主控制器初始化
*   return: 0 - OK   非0 - Failed
 ******************************************************************************/	
uint32_t twi_master_Si114x_init(void)
{
    ret_code_t ret;

	  const nrf_drv_twi_config_t Si114x_config =
		{
			 .scl                = Si114x_SCL_PIN,
			 .sda                = Si114x_SDA_PIN,
			 .frequency          = NRF_TWI_FREQ_400K,
			 .interrupt_priority = APP_IRQ_PRIORITY_LOW,
			 .clear_bus_init     = false
		};
    ret = nrf_drv_twi_init(&m_si114xTwi_master, &Si114x_config, NULL, NULL);

    if (NRF_SUCCESS == ret)
    {
        nrf_drv_twi_enable(&m_si114xTwi_master);
    }

    return ret;
}


/******************************************************************************
* @brief 
*   功能：si114x的TWI控制器使能
 ******************************************************************************/
void Twi_master_Si114x_Enable(void)
{
		nrf_drv_twi_enable(&m_si114xTwi_master);
}

/******************************************************************************
* @brief 
*   功能：si114x的TWI控制器去能
 ******************************************************************************/
void Twi_master_Si114x_Disable(void)
{
		nrf_drv_twi_disable(&m_si114xTwi_master);
}

/******************************************************************************
* @brief 
*   功能：I2C主控制器去初始化
*   return: 0 - OK   非0 - Failed
 ******************************************************************************/
uint32_t twi_master_Si114x_uinit(void)
{
    ret_code_t ret;
		
		Twi_master_Si114x_Disable();
    nrf_drv_twi_uninit(&m_si114xTwi_master);

    return ret;
}

/******************************************************************************
* @brief 
*   功能：读Si114x寄存器的数据 
*   参数：
*         si114x_handle   - I2C主控制器句柄
*         addr      			- 待读的寄存器地址
*         outdata   			- 输出数据的缓存首地址
*         size      			- 待读数据的大小
*   return: 0 - OK   非0 - Failed
 ******************************************************************************/
uint8_t Si114x_Read(nrf_drv_twi_t si114x_handle, uint8_t *addr, uint8_t *outdata, uint8_t size)
{
		uint32_t err_code = 0xFF;
	
		if(addr == NULL || outdata == NULL) return 1;
	
		err_code = nrf_drv_twi_tx(&si114x_handle, Si114x_I2C_ADDRESS, addr, 1, false);
		err_code = nrf_drv_twi_rx(&si114x_handle, Si114x_I2C_ADDRESS, outdata, size);
		if(err_code == NRF_SUCCESS)
		{
			  return 0;	
		}
	
		return 1;
}

/******************************************************************************
* @brief 
*   功能：给Si114x寄存器写数据
*   参数：
*         si114x_handle   - I2C主控制器句柄
*         txdata    		  - 待写数据缓存
*         size            - 待写数据的大小
*   return: 0 - OK   非0 - Failed
*   注：用于给Si114x某个寄存器写数据。只写一个寄存器。
 ******************************************************************************/
uint8_t Si114x_Write(nrf_drv_twi_t si114x_handle, uint8_t *txdata, uint8_t size)
{
		uint32_t err_code = 0xFF;
	
		if(txdata == NULL) return 1;
	
		err_code = nrf_drv_twi_tx(&si114x_handle, Si114x_I2C_ADDRESS, txdata, size, false); 
		if(err_code == NRF_SUCCESS)
		{
				return 0;
		}
	
		return 1;
}


/******************************************************************************
* @brief 
*   功能：给Si114x寄存器写多个数据
*   参数：
*         si114x_handle   - I2C主控制器句柄
*         address    			- 待写寄存器地址
*         length      	  - 待发送数据的大小
*   return: 0 - OK   非0 - Failed
*   注：此函数给si114x寄存器写多个数据时，寄存器的地址是连续的。
 ******************************************************************************/
uint8_t Si114xBlockWrite(nrf_drv_twi_t si114x_handle, uint8_t address, uint8_t length, uint8_t *values)
{
   uint8_t retval = 0, counter;
	 uint8_t tx_data[2] = {0}; 
	
   for (counter = 0; counter < length; counter++)
   {
		  tx_data[0] = address+counter;
			tx_data[1] = values[counter];
		  retval += Si114x_Write(si114x_handle, tx_data, sizeof(tx_data));
   }

   return retval;    
}


/***************************************************************************//**
 * @brief 
 *   Waits until the Si113x/4x is sleeping before proceeding
*    return: 0 - OK   1 - Failed
 ******************************************************************************/
static uint8_t _waitUntilSleep(nrf_drv_twi_t si114x_handle)
{
  uint8_t retval = 0;
  uint8_t count = 0;
	uint8_t regAddr;
	
  // This loops until the Si114x is known to be in its sleep state
  // or if an i2c error occurs
	regAddr = REG_CHIP_STAT;
  while(count < LOOP_TIMEOUT_MS)
  {
		Si114x_Read(si114x_handle, &regAddr, &retval, 1);
		if(retval == 1) return 0;
		count++;
		nrf_delay_ms(1); 
  }
	
  return 1;
}

/***************************************************************************//**
 * @brief 
 *   Writes a byte to an Si113x/4x Parameter(将1个字节写入到Si113x/4x 参数)
 * @param[in] si114x_handle 
 *   The programmer's toolkit handle
 * @param[in] address 
 *   The parameter address 
 * @param[in] value   
 *   The byte value to be written to the Si113x/4x parameter
 * @retval 0    
 *   Success
 * @retval 1  
 *   Error
 * @note This function ensures that the Si113x/4x is idle and ready to
 * receive a command before writing the parameter. Furthermore, 
 * command completion is checked. If setting parameter is not done
 * properly, no measurements will occur. This is the most common
 * error. It is highly recommended that host code make use of this
 * function.
 * 注意：此功能确保Si113x/4x在写入参数之前处于空闲状态并准备好接收命令，此外,
 * 命令完成后被检查，如果设置参数不正确，不会有测量发生，这是最常见的错误。
 * 强烈建议主机代码使用这个功能。
 ******************************************************************************/
uint8_t Si114xParamSet(nrf_drv_twi_t si114x_handle, uint8_t address, uint8_t value) //address = 0x01(PARAM_CH_LIST),  value = 0x31
{
	int16_t retval = 0;
  uint8_t buffer[2];
  uint8_t response_stored;
  uint8_t response;
	uint8_t regAddr;
	//uint8_t tx_data[2] = {0}; 
	uint8_t  count = 0; 
	
  if((retval = _waitUntilSleep(si114x_handle))!= 0) return 1;
	
	retval = 0;
	regAddr = REG_RESPONSE;
	retval += Si114x_Read(si114x_handle, &regAddr, &response_stored, 1);
	
	buffer[0]= value; //buffer[0]= 0x31;
  buffer[1]= 0xA0 + (address & 0x1F); //buffer[1] = 0xA0 + (0x01 & 0x1F) = 0xA0 + 0x01 = 0xA1
	retval += Si114xBlockWrite(si114x_handle, REG_PARAM_WR, 2, ( uint8_t* ) buffer);
	if(retval != 0)
	{
		 return 1;
	}
	
	// Wait for command to finish
	regAddr = REG_RESPONSE;
	retval += Si114x_Read(si114x_handle, &regAddr, &response, 1);
	while(response == response_stored)
	{
		 retval +=  Si114x_Read(si114x_handle, &regAddr, &response, 1);
		 if (response == response_stored)
     {
			  count++;
				nrf_delay_ms(1); 
     }
		 
		 if(count >= LOOP_TIMEOUT_MS) //验证一下
		 {
				break;
		 }
	}
	
	if(retval == 0)
    return 0;
  else
    return 1;
}


/***************************************************************************//**
 * @brief 
 *   Reads a Parameter from the Si113x/4x
 * @param[in] si114x_handle 
 *   The programmer's toolkit handle
 * @param[in] address   
 *   The address of the parameter. 
 * @retval <0       
 *   Error
 * @retval 0-255    
 *   Parameter contents
 si114x_handle - TWI句柄
 address       - 命令参数地址
 outData       - 保存数据的地址
返回          -  0 :成功    1:失败
 ******************************************************************************/
uint8_t Si114xParamRead(nrf_drv_twi_t si114x_handle, uint8_t address, uint8_t *outData)
{
		int16_t retval = 0;
	  uint8_t data = 0;
	  uint8_t regAddr;
	
	  uint8_t cmd = 0x80 + (address & 0x1F);
	
	  if((retval=_sendCmd(si114x_handle, cmd )) != 0) return 1;
	
	  retval = 0;
		regAddr = REG_PARAM_RD;
		retval = Si114x_Read(si114x_handle, &regAddr, &data, 1);
	  if(retval == 0)
		{
			*outData = data;
	  }
		
	  if(retval == 0)
			return 0;
		else
			return 1;
}

/******************************************************************************
 * @brief 
 *  Helper function to send a command to the Si113x/4x(发送一个命令到Si113x/Si114x)
 *  return  0 - OK  1 - failed
 ******************************************************************************/
static uint8_t _sendCmd(nrf_drv_twi_t si114x_handle, uint8_t command)
{
	uint8_t  response = 0x01; //
  uint8_t  retval = 0x02;
	uint8_t  ivalue = 0; 
	uint8_t regAddr;
  uint8_t  count = 0; 
	uint8_t tx_data[2] = {0}; 
	
	// Get the response register contents
	regAddr = REG_RESPONSE;
	ivalue = Si114x_Read(si114x_handle, &regAddr, &response, 1);
	if(ivalue)
	{
		  NRF_LOG_RAW_INFO("_sendCmd0\r\n");
			return 1;
	}
	
	// Double-check the response register is consistent
	while(count < LOOP_TIMEOUT_MS)
	{
			if((retval=_waitUntilSleep(si114x_handle)) != 0) 
			{
				NRF_LOG_RAW_INFO("_sendCmd1\r\n");
				return 1;
			}
			if(command==0) break; // Skip if the command is NOP 
		
			ivalue = Si114x_Read(si114x_handle, &regAddr, &retval, 1);
			if(ivalue)
			{
					NRF_LOG_RAW_INFO("_sendCmd2\r\n");
					return 1;
			}
			
			if(retval==response) 
			{
					break;
			}
			else
			{
					response = retval;
			}
			
			count++;
	}
	
	//NRF_LOG_RAW_INFO("C1:%d", count); //这个值一般为0或1
	
	// Send the Command  - Pauses PS and ALS  -  0x0B
	tx_data[0] = REG_COMMAND;
	tx_data[1] = command;
	ivalue = Si114x_Write(si114x_handle, tx_data, sizeof(tx_data));
	if(ivalue)
	{
		 NRF_LOG_RAW_INFO("_sendCmd3\r\n");
		 return 1;  
	}
	
	count = 0;
	// Expect a change in the response register
	while(count < LOOP_TIMEOUT_MS)
	{
			if(command==0) break; // Skip if the command is NOP
		
			ivalue = Si114x_Read(si114x_handle, &regAddr, &retval, 1);
			if(ivalue)
			{
					NRF_LOG_RAW_INFO("_sendCmd4\r\n");
					return 1;
			}
		
			if(retval != response) break;
			count++;
			nrf_delay_ms(1); 
	}
	//NRF_LOG_RAW_INFO("C2:%d", count);//这个值一般为0或1
	
	return 0;
}

/***************************************************************************//**
 * @brief 
 *   Pause measurement helper function(暂停测量帮助功能)
 ******************************************************************************/
static uint8_t _PsAlsPause (nrf_drv_twi_t si114x_handle) 
{
  return _sendCmd(si114x_handle, 0x0B);
}


/***************************************************************************//**
 * @brief 
 *   Pauses autonomous measurements(暂停自主测量功能)
 * @param[in] si114x_handle 
 *  The programmer's toolkit handle
 * @retval  0       
 *   Success
 * @retval  1     
 *   Error
 ******************************************************************************/
uint8_t Si114xPauseAll(nrf_drv_twi_t si114x_handle)
{
  uint8_t countA, countB;
  uint8_t retval;
	uint8_t regAddr;
	uint8_t tx_data[2] = {0}; 
	//uint8_t uvalue = 0;

  //  After a RESET, if the Si114x receives a command (including NOP) before the
  //  Si114x has gone to sleep, the chip hangs. This first while loop avoids 
  //  this.  The reading of the REG_CHIPSTAT does not disturb the internal MCU.
  //

   retval = 0; //initialize data so that we guarantee to enter the loop
	 regAddr = REG_CHIP_STAT;
	
 //while(retval != 0x01) // need modify
	 countA = 0;
	 while(countA < LOOP_TIMEOUT_MS)
   {    
			Si114x_Read(si114x_handle, &regAddr, &retval, 1);
			if (retval != 0x01)
      {
					nrf_delay_ms(1); 
      }
			else
			{
					break;
			}
			countA++;
   }
	 
	 if (retval != 0x01)
	 {
			return 1;
	 }
	
	//uvalue = 0;
  countA = 0;
  while(countA < LOOP_TIMEOUT_MS)
  {
    countB = 0;
    // Keep sending nops until the response is zero
    while(countB < LOOP_TIMEOUT_MS)
    {
			regAddr = REG_RESPONSE;
			Si114x_Read(si114x_handle, &regAddr, &retval, 1);
			if( retval == 0 )
			{
          break;
			}
			else
			{
					// Send the NOP Command to clear any error...we cannot use Si114xNop()
					// because it first checks if REG_RESPONSE < 0 and if so it does not
					// perform the cmd. Since we have a saturation REG_RESPONSE will be <0
					tx_data[0] = REG_COMMAND; //要写的寄存器
					tx_data[1] = 0x00;        //要写的寄存器的值
					Si114x_Write(si114x_handle, tx_data, sizeof(tx_data));
			}
			countB++;
			nrf_delay_ms(1); 
    }
		
    // Pause the device
#if 0
		_PsAlsPause(si114x_handle);
#else 
    if(_PsAlsPause(si114x_handle))
		{
				NRF_LOG_RAW_INFO("error_PsAlsPause\r\n");
		}
#endif

    countB = 0;
		regAddr = REG_RESPONSE;
    // Wait for response
    while(countB < LOOP_TIMEOUT_MS)
    {
			Si114x_Read(si114x_handle, &regAddr, &retval, 1);
			if( retval !=0 )
        break;
			countB++;
			nrf_delay_ms(1); 
    }

    // When the PsAlsPause() response is good, we expect it to be a '1'.
		regAddr = REG_RESPONSE;
		retval = 0;
		Si114x_Read(si114x_handle, &regAddr, &retval, 1);
		if(retval == 1)
		{
			  return 0;  // otherwise, start over.	
		}
		countA++;
  }
	
  return 1;
}

/***************************************************************************//**
 * @brief 
 *   Sends a PSALSAUTO command to the Si113x/4x（发送一个PSALSAUTO命令到Si113x/4x,开始自动测量）
 *   开始临近测量和环境光测量。
 * @param[in] si114x_handle
 *   The programmer's toolkit handle
 * @retval  0       
 *   Success
 * @retval  <0      
 *   Error
 ******************************************************************************/
uint8_t Si114xPsAlsAuto (nrf_drv_twi_t si114x_handle)
{
  return _sendCmd(si114x_handle, 0x0F);
}

//开始临近自发测量
uint8_t Si114xPsAuto (nrf_drv_twi_t si114x_handle)
{
  return _sendCmd(si114x_handle, 0x0D);
}

//停止临近自发测量 
uint8_t Si114xPsPause (nrf_drv_twi_t si114x_handle)
{
  return _sendCmd(si114x_handle, 0x09);
}

//停止可见光自发测量 
uint8_t Si114xAlsPause (nrf_drv_twi_t si114x_handle)
{
  return _sendCmd(si114x_handle, 0x0A);
}

//停止临近及可见光自发测量 
uint8_t Si114xPsAlsPause (nrf_drv_twi_t si114x_handle)
{
  return _sendCmd(si114x_handle, 0x0B);
}

/***************************************************************************//**
 * @brief 
* 功能: 复位Si113x/4x,清除任何中断并初始化HW_KEY.  
 * Resets the Si113x/4x, clears any interrupts and initializes the HW_KEY 
 * register.
 * @param[in] si114x_handle 
 *   The programmer's toolkit handle
 * @retval  0       
 *   Success
 * @retval  1     
 *   Error
 ******************************************************************************/
uint8_t Si114xReset(nrf_drv_twi_t si114x_handle)
{
  uint8_t retval = 0;
	uint8_t tx_Data[2] = {0,0};
  //
  // Do not access the Si114x earlier than 25 ms from power-up. 
  // Uncomment the following lines if Si114xReset() is the first
  // instruction encountered, and if your system MCU boots up too 
  // quickly. 
  //
	nrf_delay_ms(10); 
	nrf_delay_ms(10); 
	nrf_delay_ms(10); 
	
	tx_Data[0] = REG_MEAS_RATE;//要写的寄存器地址
	tx_Data[1] = 0x00;//要写的寄存器的值
	retval += Si114x_Write(si114x_handle, tx_Data, sizeof(tx_Data));
	
	tx_Data[0] = REG_ALS_RATE;
	tx_Data[1] = 0x00;
	retval += Si114x_Write(si114x_handle, tx_Data, sizeof(tx_Data));
	
	retval += Si114xPauseAll(si114x_handle);
	
	// The clearing of the registers could be redundant, but it is okay.
  // This is to make sure that these registers are cleared.
	tx_Data[0] = REG_MEAS_RATE;
	tx_Data[1] = 0x00;
	retval += Si114x_Write(si114x_handle, tx_Data, sizeof(tx_Data));
	
	tx_Data[0] = REG_IRQ_ENABLE;
	tx_Data[1] = 0x00;
	retval += Si114x_Write(si114x_handle, tx_Data, sizeof(tx_Data));
	
	tx_Data[0] = REG_IRQ_MODE1;
	tx_Data[1] = 0x00;
	retval += Si114x_Write(si114x_handle, tx_Data, sizeof(tx_Data));
	
	tx_Data[0] = REG_IRQ_MODE2;
	tx_Data[1] = 0x00;
	retval += Si114x_Write(si114x_handle, tx_Data, sizeof(tx_Data));
	
	tx_Data[0] = REG_INT_CFG;
	tx_Data[1] = 0x00;
	retval += Si114x_Write(si114x_handle, tx_Data, sizeof(tx_Data));
	
	tx_Data[0] = REG_IRQ_STATUS;
	tx_Data[1] = 0xFF;
	retval += Si114x_Write(si114x_handle, tx_Data, sizeof(tx_Data));
	
	// Perform the Reset Command
	tx_Data[0] = REG_COMMAND;
	tx_Data[1] = 1;
	retval += Si114x_Write(si114x_handle, tx_Data, sizeof(tx_Data));
	
	// Delay for 10 ms. This delay is needed to allow the Si114x
  // to perform internal reset sequence. 
	nrf_delay_ms(10); 
	
	// Write Hardware Key
	tx_Data[0] = REG_HW_KEY;
	tx_Data[1] = HW_KEY_VAL0;
	retval += Si114x_Write(si114x_handle, tx_Data, sizeof(tx_Data));
	
  return retval;
}

/********************************************************
* 功能：清除中断标记位
* 返回：0 - ok  1 - failed
注：如果清除心率传感器的中断标记位失败，不会进入中断，读取不到心率数据。
********************************************************/
uint8_t ClearInterruptStatusBit(void)
{
	uint8_t tx_Data[2] = {0,0};
	uint8_t retval = 0;
	
	tx_Data[0] = REG_IRQ_STATUS;
	tx_Data[1] = 0xFF;
	retval = Si114x_Write(m_si114xTwi_master, tx_Data, sizeof(tx_Data));
	
	return retval;
}


/***************************************************************************
 * @brief 
 *   
 * 功能：设置si114x中任意一个LED驱动电流
 *  
 * 参数：si114x_handle - TWI的句柄      
 *       greenLed      - 需要设置的LED; GREEN_LED1,GREEN_LED2,GREEN_LED3
 *       ledCurrent    - 设置的电流大小；
 * 返回：0 - ok  1 - failed
 * 
 * 注意：si1141 - 只能设置LED1  si1142 - 可设置LED1,LED2  si1143 - 可设置LED1,LED2，LED3
 ******************************************************************************/
uint8_t Si114x_SetLedCurrent(nrf_drv_twi_t si114x_handle, Si114x_green_led_t greenLed, Si114x_led_current_t ledCurrent)
{
	  uint8_t  retval = 0;
		uint8_t tx_Data[2] = {0,0};
		uint8_t regAddr;
		uint8_t readData = 0x00;
		
		
		if( (greenLed == GREEN_LED1) || (greenLed == GREEN_LED2))
		{
			  regAddr = REG_PS_LED21;
				retval += Si114x_Read(si114x_handle, &regAddr, &readData, sizeof(readData));
		}
		
		if(retval)
			return 1;
		
		if(greenLed == GREEN_LED1)
		{
			 tx_Data[0] = REG_PS_LED21;
			 tx_Data[1] = ledCurrent;
			 retval += Si114x_Write(si114x_handle, tx_Data, sizeof(tx_Data));
		}
		else if(greenLed == GREEN_LED2)
		{
			 tx_Data[0] = REG_PS_LED21;
			 tx_Data[1] = ledCurrent;
			 retval += Si114x_Write(si114x_handle, tx_Data, sizeof(tx_Data));
		}
		else if(greenLed == GREEN_LED3)
		{
			 tx_Data[0] = REG_PS_LED3;
			 tx_Data[1] = ledCurrent;
			 retval += Si114x_Write(si114x_handle, tx_Data, sizeof(tx_Data));
		}
	  
		return retval;
}

//验证连续读取的数据是否正确，正确返回1，错误返回0。连续写
#if 0
uint8_t VerifyWriteReadOK(void)
{
	  uint8_t error = 0;
	  uint8_t  writeBuf[4] = {0x03,0x1F,0x51,0x05};
		uint8_t  readBuf[4] = {0};
		uint8_t  VData_Buffer[2] = {0};
		uint8_t  Vsi114x_reg = REG_INT_CFG;
		uint8_t irightNum = 0;
		uint8_t iCalNum = 0;
		
		while(1)
		{
			iCalNum++;
			VData_Buffer[0] = REG_INT_CFG;
			VData_Buffer[1] = writeBuf[0];
			error += Si114x_Write(m_si114xTwi_master, VData_Buffer, sizeof(VData_Buffer));
			
			VData_Buffer[0] = REG_IRQ_ENABLE;
			VData_Buffer[1] = writeBuf[1];
			error += Si114x_Write(m_si114xTwi_master, VData_Buffer, sizeof(VData_Buffer));
			
			VData_Buffer[0] = REG_IRQ_MODE1;
			VData_Buffer[1] = writeBuf[2];
			error += Si114x_Write(m_si114xTwi_master, VData_Buffer, sizeof(VData_Buffer));
			
			VData_Buffer[0] = REG_IRQ_MODE2;
			VData_Buffer[1] = writeBuf[3];
			error += Si114x_Write(m_si114xTwi_master, VData_Buffer, sizeof(VData_Buffer));
			
			error += Si114x_Read(m_si114xTwi_master, &Vsi114x_reg, readBuf, 4);
			if(memcmp(writeBuf,readBuf, sizeof(writeBuf)) == 0)
			{
				 irightNum++;
			}

			memset(readBuf, 0x00, sizeof(readBuf));
			
			if(iCalNum >= 200) 
			{
					break;
			}
		}
		
		if(irightNum >= 200)
		{
			 return 1;
		}
		
		return 0;
}
#endif


/***************************************************************************
* @brief 
* 功能:si114x心率传感器自动测量模式的参数设置
* 参数：
*      si114x_handle  - 句柄
*      current        - LED灯驱动电流设置
*      tasklist       - 测量选项
*      measrate       - 测量间隔时间
*      adcgain        - LED灯的脉冲宽度
*      adcmisc        - 杂项设置，（第2位）设置用于原始ADC测量还是PS通道正常工作；PS测量时，设置工作在正常模式还是高灵敏度操作（第5位）。
                        设置为PS通道正常工作（第2位置1），正常模式（第5位置1）
*      adcmux         - 设置为大型红外光电二极管 还是 小型红外光电二极管 
* 返回：
*      0 - ok    1 - failed
******************************************************************************/
uint8_t Si114xConfigure(nrf_drv_twi_t si114x_handle)
{ 
	 int16_t  current[3]= {LEDI_045, 0x00, 0x00};//各LED的驱动电流 LED1 = 22mA, LED2 = 0mA, LED2 = 0mA Si1142只有LED1、LED2。本项目只用到LED1.
   int16_t  tasklist= 0x31;  //使能ALS PS1通道 
   int16_t  measrate= SI114X_MEASURE_RATE;  	                      
   int16_t  adcgain= 0x04;  // PS_ADC_GAIN(脉冲宽度) 0x00 - 25.6us, 0x01 - 51.2us  0x02 - 102.4us 0x03-204.8us  0x04-409.6us  0x05-819.2us.取值不要大于等于0x05(官方建议).
   int16_t  adcmisc= 0x04;  // PS_ADC_MISC to default Normal Signal Range
   int16_t  adcmux[3]= {0x03, 0x03, 0x03};// PS1_ADCMUX, PS2_ADCMUX, PS3_ADCMUX to default large photodiode
	
   uint8_t  retval=0;
	 uint8_t tx_Data[2] = {0,0};
	 
	 
	 adcgain = guWave[0][0];//波形A等级的增益参数设置
	 current[0] = guWave[0][1];//波形A等级的驱动电流参数设置
	 
   // Note that the Si114xReset() actually performs the following functions:
   // 1. Pauses all prior measurements
   // 2. Clear  i2c registers that need to be cleared
   // 3. Clears irq status to make sure INT* is negated
   // 4. Delays 10 ms
   // 5. Sends HW Key
   retval += Si114xReset(si114x_handle);
	 
   // Get the LED current passed from the caller
   // If the value passed is negative, just pick a current... 202 mA
	 //设置LED1，LED2，LED3的驱动电流
   {
       uint8_t i21, i3; 
		 
			 i21 = (current[1]<<4) + current[0];//current[0]为LED1的驱动电流，current[1]为LED2的驱动电流
       i3  = current[2];//LED3的驱动电流
			 
			 tx_Data[0] = REG_PS_LED21;
			 tx_Data[1] = i21;
			 retval += Si114x_Write(si114x_handle, tx_Data, sizeof(tx_Data));//设置LED1，LED2的驱动电流;LED2的驱动电流为0MA
			  
			 //LED3的驱动电流设置，Si1142没有用到
			 tx_Data[0] = REG_PS_LED3;
			 tx_Data[1] = i3;
			 retval += Si114x_Write(si114x_handle, tx_Data, sizeof(tx_Data));//设置LED3的驱动电流 - 0MA
   }
	 
	 //VerifyWriteReadOK();
	 //设置PS1、PS2、PS3通道的LED驱动。只用到PS1通道，PS1用LED1驱动，PS2，PS3设置为没有LED驱动。如果不设置，默认都使用LED1驱动。
	 retval += Si114xParamSet(si114x_handle, PSLED12_SELECT, 0x01);//指定PS1测量期间，LED驱动的引脚为LED1驱动。PS2测量没有驱动，因为没有打开PS2测量。
	 retval += Si114xParamSet(si114x_handle, PSLED3_SELECT, 0x00);//PS3测量的LED驱动设为无。

   // Initialize CHLIST Parameter from caller to enable measurement
   // Valid Tasks are: ALS_VIS_TASK, ALS_IR_TASK, PS1_TASK
   //                  PS2_TASK, PS3_TASK and AUX_TASK
   // However, if we are passed a 'negative' task, we will
   // turn on ALS_IR, ALS_VIS and PS1. Otherwise, we will use the 
   // task list specified by the caller.
   if (tasklist < 0 ) tasklist = PS1_TASK;
   retval += Si114xParamSet(si114x_handle, PARAM_CH_LIST, tasklist);//使能临近PS1通道，可见光环境测量，红外光环境测量  
   
	 //中断输出使能及中断模式设置。
   // Set IRQ Modes and INT CFG to interrupt on every sample
	 //只要IRQ_STATUS及相应的IRQ_ENABLE位匹配，INT引脚就被驱动位低电平
	 tx_Data[0] = REG_INT_CFG;
	 tx_Data[1] = ICG_INTOE;	 
	 retval += Si114x_Write(si114x_handle, tx_Data, sizeof(tx_Data));
	 
	 //PS1通道测量使能，辅助测量(ALS)使能
	 tx_Data[0] = REG_IRQ_ENABLE; 
   tx_Data[1] = (IE_ALS_EVRYSAMPLE + IE_PS1_EVRYSAMPLE);	
	 retval += Si114x_Write(si114x_handle, tx_Data, sizeof(tx_Data));
	 
	 //PS1、ALS中断模式选择
	 tx_Data[0] = REG_IRQ_MODE1;
   tx_Data[1] = (IM1_ALS_EVRYSAMPLE + IM1_PS1_EVRYSAMPLE);	
	 retval += Si114x_Write(si114x_handle, tx_Data, sizeof(tx_Data));

#if 0	 //0 - 1
   //PS3中断模式选择
	 tx_Data[0] = REG_IRQ_MODE2;
	 tx_Data[1] = IM2_PS3_EVRYSAMPLE;	 
	 retval += Si114x_Write(si114x_handle, tx_Data, sizeof(tx_Data)); 
#endif

	 //临近通道增益设置（即LED的脉冲宽度）,也称积分时间，数据手册要求设置值不要大于5.
	 retval += Si114xParamSet(si114x_handle, PARAM_PS_ADC_GAIN, adcgain);
	 //PS测量时，使用范围设置，1.可以设置为高信号情况下使用，即阳光直射，2.正常情况下   -  选择为正常情况
	 retval += Si114xParamSet(si114x_handle, PARAM_PS_ADC_MISC, adcmisc);
	 //为PS1测量选择ADC输入，大型红外光电二极管，小型红外光电二极管。选择大型
   retval += Si114xParamSet(si114x_handle, PARAM_PS1_ADC_MUX, adcmux[0]);

#if 0 //0  -  1
	 //PS2测量选择输入
	 retval += Si114xParamSet(si114x_handle, PARAM_PS2_ADC_MUX, adcmux[1]);
	 //PS3测量选择输入
   retval += Si114xParamSet(si114x_handle, PARAM_PS3_ADC_MUX, adcmux[2]);
#endif

	 // Configure the ALS IR channel for the same settings as PS
	 retval += Si114xParamSet(si114x_handle, PARAM_ALSIR_ADC_GAIN, adcgain);
	 retval += Si114xParamSet(si114x_handle, PARAM_ALSIR_ADC_MISC, adcmisc &0x20);
	 retval += Si114xParamSet(si114x_handle, PARAM_IR_ADC_MUX, adcmux[0]);

	 // If the caller passes a negative measrate, it means that it does not
   // want to start autonomous measurements.
   if(measrate > 0 )
   {
			 // Set up how often the device wakes up to make measurements
       // measrate, for example can be the following values:
		   // 0xa4 = Device Wakes up every ~40 ms
       // 0xa0 = Device Wakes up every ~30 ms
       // 0x94 = Device Wakes up every ~20 ms
       // 0x84 = Device Wakes up every ~10 ms
       // 0xB9 = Device Wakes up every ~100 ms
		   // 0xDF = Device Wakes up every ~496 ms
       // 0xFF = Device Wakes up every ~2 sec
			 tx_Data[0] = REG_MEAS_RATE;
			 tx_Data[1] = measrate;	 
			 retval += Si114x_Write(si114x_handle, tx_Data, sizeof(tx_Data)); 
		 
			 // if 0x08, PS1, PS2 and PS3 made every time device wakes up.
		   tx_Data[0] = REG_PS_RATE;
			 tx_Data[1] = 0x08;	 
			 retval += Si114x_Write(si114x_handle, tx_Data, sizeof(tx_Data)); 

		   // if 0x08, VIS, IR, AUX Measurements every time device wakes up.
		   tx_Data[0] = REG_ALS_RATE;
			 tx_Data[1] = 0x08;	 
			 retval += Si114x_Write(si114x_handle, tx_Data, sizeof(tx_Data)); 
 
			 // Enable Autonomous Operation
       retval += Si114xPsAlsAuto(si114x_handle);// Si114xPsAuto - > Si114xPsAlsAuto

			 // If nothing went wrong after all of this time, the value
       // returned will be 0. Otherwise, it will be some negative
       // number
	 }

	 if(retval == 0)
	 {
			retval = 0;
	 }
	 else
	 {
			retval = 1;
		  NRF_LOG_RAW_INFO("error_autoflg = %d\r\n", retval);
	 }
	 
   return retval;
}


/***************************************************
* 功能：写一个寄存器再读取该寄存器，验证读写是否成功。测试用
* return:1 - OK    0 - 失败
***************************************************/
#if 0
uint8_t WriteAndRead(void)
{
		uint8_t tx_data[2] = {0, 0};
		uint8_t retval=0;
		uint8_t regAddr;
		uint8_t readData = 0x00;
		
		tx_data[0] = 0x03;
		tx_data[1] = 0x02;
		
		retval += Si114x_Write(m_si114xTwi_master, tx_data, sizeof(tx_data)); 
		regAddr = 0x03;
		retval += Si114x_Read(m_si114xTwi_master, &regAddr, &readData, 1);
		
		if(readData == tx_data[1])
		{
				return 1;
		}
		
		return 0;
}
#endif 

/***************************************************************************************************
* 功能：从PS1通道读取心率数据
* 参数：保存心率数据的缓存
* 返回: 0 - ok  1 - failed
****************************************************************************************************/
uint16_t Si114x_GetHeartData(uint16_t *data)
{
	  uint8_t Data_Buffer[2] = {0};
		uint8_t Si114x_reg = 0;
	  uint8_t Si114x_Tx_Data[2] = {REG_IRQ_STATUS, 0xff};
		uint16_t tmpData = 0;	
		uint8_t error = 0;
		
#if Si114X_JUDGE_SAMP
		uint8_t u8SamData = 0;
#endif
		
		Si114x_reg = REG_IRQ_PS1_DATA0;
		error += Si114x_Read(m_si114xTwi_master, &Si114x_reg, &Data_Buffer[0], 1);//低8位
		if(error != 0 )
		{
				NRF_LOG_RAW_INFO("error1");
			  //return 1;
		}
		
		Si114x_reg = REG_IRQ_PS1_DATA1;
		error += Si114x_Read(m_si114xTwi_master, &Si114x_reg, &Data_Buffer[1], 1);//高八位
		if(error != 0)
		{
				NRF_LOG_RAW_INFO("error2");
			  //return 1;
		}
	
#if Si114X_JUDGE_SAMP
		Si114x_reg = REG_MEAS_RATE;
		error += Si114x_Read(m_si114xTwi_master, &Si114x_reg, &u8SamData, 1);
		if( (error != 0) || (u8SamData != 0xA4))
		{
			NRF_LOG_RAW_INFO("error4:%d-%d.", error, u8SamData);
		}
		
		Si114x_reg = REG_PS_RATE;
		error += Si114x_Read(m_si114xTwi_master, &Si114x_reg, &u8SamData, 1);
		if( (error != 0) || (u8SamData != 0x08))
		{
			NRF_LOG_RAW_INFO("error5:%d-%d.", error, u8SamData);
		}
#endif
		
		//error += Si114x_Write(m_si114xTwi_master, Si114x_Tx_Data, sizeof(Si114x_Tx_Data)); //清PS1通道中断标记位
		error += Si114x_Write(m_si114xTwi_master, Si114x_Tx_Data, sizeof(Si114x_Tx_Data)); //清PS1通道中断标记位
		tmpData = (uint16_t)Data_Buffer[1];
		tmpData = tmpData << 8;
		tmpData += (uint16_t)Data_Buffer[0];  
		if(error != 0)
		{
				NRF_LOG_RAW_INFO("error3.");
			  return 1;
		}
		*data = 0xFFFF - tmpData;
	
#if 0
		if(*data == 0)
			NRF_LOG_RAW_INFO("zz.");
#endif
		
		return 0;
}

/***************************************************************************************************
* 功能：写寄存器
* 参数：regAddr - 寄存器地址    data - 寄存器数据
* 返回: 0 - ok  1 - failed
****************************************************************************************************/
uint8_t Si114xWriteToRegister(uint8_t regAddr, uint8_t data)
{
	 uint8_t Si114x_Tx_Data[2] = {0, 0};
	 uint8_t error = 0;
	 
	 Si114x_Tx_Data[0] = regAddr;
	 Si114x_Tx_Data[1] = data;
	 error = Si114x_Write(m_si114xTwi_master, Si114x_Tx_Data, sizeof(Si114x_Tx_Data));
	 
	 return error;
}


//动态调整波形等级对应的参数 返回：0 - 成功  1 - 失败
uint8_t DynamicallyAdjustParmeters(uint8_t waveState)
{
		uint8_t ureturn = 0;
		uint8_t uData = 0xFF;
	  uint8_t Si114x_reg;
	
#if 0
		uint8_t LastPara[2] = {0};
		LastPara[0] = guWave[uwaveformLevel][0];//上一次等级的增益
		LastPara[1] = guWave[uwaveformLevel][1];//上一次等级的驱动电流
#endif
		
	  if(uwaveformLevel < 3)
		{
				uwaveformLevel++;
		}
		else
		{
				uwaveformLevel = WAVE_A_GRADE;
		}
		
#if 0
		//上一次波形等级的增益与本次的不一样，设置。
		if(LastPara[0] != guWave[uwaveformLevel][0])
		{
				ureturn += Si114xParamSet(m_si114xTwi_master, PARAM_PS_ADC_GAIN, guWave[uwaveformLevel][0]);
				if(ureturn)
				{
						NRF_LOG_RAW_INFO("err-A%d-0",uwaveformLevel);
				}
		}
		
		//上一次波形等级的驱动电流与本次的不一样，设置。
		if(LastPara[1] != guWave[uwaveformLevel][1])
		{
				ureturn += SetSi114x_Current(guWave[uwaveformLevel][1]);
				if(ureturn)
				{
					 NRF_LOG_RAW_INFO("err-A%d-1",uwaveformLevel);
				}
		}	
#else
		//在中断里调整增益和驱动电流有关系么？
		ureturn += Si114xParamSet(m_si114xTwi_master, PARAM_PS_ADC_GAIN, guWave[uwaveformLevel][0]);
		if(ureturn)
		{
				NRF_LOG_RAW_INFO("err-A%d-0\r\n",uwaveformLevel);
		}
		else
		{
#if 0
				Si114xParamRead(m_si114xTwi_master, PARAM_PS_ADC_GAIN, &uData);
			  if(uData != guWave[uwaveformLevel][0])
				{
					NRF_LOG_RAW_INFO("err-A:%d-%d-3\r\n", guWave[uwaveformLevel][0], uData);
				}
#endif
		}
		
		ureturn += SetSi114x_Current(guWave[uwaveformLevel][1]);
		if(ureturn)
		{
				NRF_LOG_RAW_INFO("err-A%d-1\r\n",uwaveformLevel);
		}
		else
		{
#if 0
			  uData = 0xFF;
			  Si114x_reg = REG_PS_LED21;
				ureturn += Si114x_Read(m_si114xTwi_master, &Si114x_reg, &uData, 1);
			  if(guWave[uwaveformLevel][1] != uData)
				{
					NRF_LOG_RAW_INFO("err-A:%d-%d-4\r\n", guWave[uwaveformLevel][1], uData);
				}
#endif				
		}
#endif
		
		return ureturn;
}	

//清空波形等级
void ClearWaveLevel(void)
{
		uwaveformLevel = 3;
}


/***************************************************************************************************
* 功能：心率传感器中断引脚，产生GPTE中断的事件函数
* 
****************************************************************************************************/
extern uint8_t u8GbWrsitIsWearing;
void si114x_Int_handle(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	  uint16_t uheartData = 0;
	  uint8_t  uWaveState = 0;//波形状态 0 - 可以，1 - 不好  2 - 饱和
	  watch_evt_t watch_evt;

	  //NRF_LOG_RAW_INFO("si114x_Int_handle\r\n");
#if TIME_SAMPS_DEBUG
	  uint32_t u32tmp = 0;
#endif
	  
		if(pin == Si114x_INT_PIN)
		{
#if TIME_SAMPS_DEBUG
			 gu32Timestamp1 = gu32Counter_1ms_Count;
			 u32tmp = gu32Timestamp1 - gu32Timestamp2;
			 if( u32tmp > 43 || u32tmp < 39)
				 NRF_LOG_RAW_INFO("%d-%d-%d-%d\r\n", gu32Timestamp2, gu32Timestamp1, u32tmp, gu32Si114xIntNum);
			 gu32Timestamp2 = gu32Timestamp1;
#endif
			 
#if COUNT_DATA_NUM
				gu32Si114xIntNum++;
#endif

			  //读取ADC数据及清中断标记位,写数据到FIFO.
				if(Si114x_GetHeartData(&uheartData) == 0)
				{
						Si114x_WriteFifo_Data(uheartData);
				}
         
				//检测并发送未佩戴事件及调整波形参数
				uWaveState = GetSignalsState();
			  if(uWaveState ==  NO_WEAR)
				{
					u8GbWrsitIsWearing = false;
					watch_evt.watch_evt_id = WATCH_EVT_NO_WEAR;
					u32WatchEventQueuePut(&structWatchEventQueueArray[eWATCH_EVENT_QUEUE_ID_2], &watch_evt);
					NRF_LOG_RAW_INFO("!!!si114x_Int_handler:WATCH_EVT_NO_WEAR\r\n");
				}
				else
				{
					u8GbWrsitIsWearing = true;
					if(uWaveState ==  NO_PULSE_WAVE || uWaveState ==  WAVE_SATURATION)
					{
#if COUNT_DATA_NUM
						gu32Si114xAdjustNum++;
						NRF_LOG_RAW_INFO("DD:%d\r\n", gu32Si114xIntNum);
#endif
						DynamicallyAdjustParmeters(uWaveState);			
					}					
				}
		}
		
		return;
}



/***************************************************************************************************
* 功能：心率传感器ADC中断引脚设置
*       
****************************************************************************************************/
#if 0
void Si114x_Int_Set(void)
{
		uint32_t err_code;
	
	  if(!nrf_drv_gpiote_is_init())
	  {
			err_code = nrf_drv_gpiote_init();
			APP_ERROR_CHECK(err_code);
		}
	
		nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
		//开启中断引脚的上拉电阻
		in_config.pull = NRF_GPIO_PIN_PULLUP; 
		//配置该引脚为GPIOTE输入
		err_code = nrf_drv_gpiote_in_init(Si114x_INT_PIN, &in_config, si114x_Int_handle);
		APP_ERROR_CHECK(err_code);
		//使能该引脚所在GPIOTE通道的事件模式
		nrf_drv_gpiote_in_event_enable(Si114x_INT_PIN, true);
}
#endif

/***************************************************************************************************
* 功能：心率传感器ADC中断引脚软件部分初始化
*       因为要用到GPIOTE中断，需要对其先初始化
****************************************************************************************************/
void Si114x_AdcPin_SoftInt(void)
{
		uint32_t err_code;
	
	  if(!nrf_drv_gpiote_is_init())
	  {
			err_code = nrf_drv_gpiote_init();
			APP_ERROR_CHECK(err_code);
		}
}

/***************************************************************************************************
* 功能：心率传感器ADC中断引脚硬件部分初始化  
*       
****************************************************************************************************/
void Si114x_AdcPin_HardInt_And_Enable(void)
{
		uint32_t err_code;
	
		nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
		//开启中断引脚的上拉电阻
		in_config.pull = NRF_GPIO_PIN_PULLUP; 
		//配置该引脚为GPIOTE输入
		err_code = nrf_drv_gpiote_in_init(Si114x_INT_PIN, &in_config, si114x_Int_handle);
		APP_ERROR_CHECK(err_code);
		//使能该引脚所在GPIOTE通道的事件模式
		nrf_drv_gpiote_in_event_enable(Si114x_INT_PIN, true);
}

/***************************************************************************************************
* 功能：心率传感器ADC中断引脚硬件去初始化和去使能
*       
****************************************************************************************************/
void Si114x_AdcPinHardUint_And_Disable(void)
{
		nrf_drv_gpiote_in_event_enable(Si114x_INT_PIN, false);//禁止中断
		nrf_drv_gpiote_in_uninit(Si114x_INT_PIN);
}

/***************************************************
* 功能：Si114x LED供电引脚初始化
* 
***************************************************/
void Si114x_vLED_Init(void)
{
	nrf_gpio_cfg_output(Si114x_vLED_EN_PIN);
	
  return;	
}

/***************************************************
* 功能：Si114x LED供电引脚使能
* 
***************************************************/
void Si114x_vLED_Enable(void)
{
	 nrf_gpio_pin_set(Si114x_vLED_EN_PIN);
	 
	 return;
}

/***************************************************
* 功能：Si114x LED供电引脚去能
* 
***************************************************/
void Si114x_vLED_Disable(void)
{
	  nrf_gpio_pin_clear(Si114x_vLED_EN_PIN);
	  
	  return;
}


/***************************************************
* 功能：初始化时，心率中断引脚，LED引脚设置为上拉输入 - 用于低功耗
* 
***************************************************/
void Si114x_Vled_Int_Init(void)
{
	  nrf_gpio_cfg_output(Si114x_vLED_EN_PIN);
	  nrf_gpio_pin_clear(Si114x_vLED_EN_PIN);
	  nrf_gpio_cfg_input(Si114x_INT_PIN, NRF_GPIO_PIN_PULLUP);
}

/***************************************************
* 功能：查找心率传感器的PART ID 
        si1141 - 0x41 si1142 - 0x42 si1143 - 0x43
* retrun:
*       0 - ok  1 - failed   
***************************************************/
uint8_t findSensors(void)
{
	 uint8_t regAddr = REG_PART_ID;
	 uint8_t readData = 0x00;
	
	 Si114x_Read(m_si114xTwi_master, &regAddr, &readData, 1);
	 if(readData == Si1142_PART_ID)
	 {
  		 return 0;
	 }
	 
	 return 1;
}

#if TIME_SAMPS_DEBUG
//开启时间戳定时器
void si114x_Timestamps_start_timer_start(void)
{
	 app_timer_start(si114x_Timestamps_timer_id, SI114X_TIMESTAMPS_INTERVAL_1ms, NULL);
}

//关闭时间戳定时器
void si114x_Timestamps_stop_timer_stop(void)
{
	 app_timer_stop(si114x_Timestamps_timer_id);
}

//时间戳定时器中断
void si114x_Timestamps_timeout_handler(void *p_context)
{
		gu32Counter_1ms_Count++;
}

//时间戳定时器初始化
void si114x_Timestamps_time_init(void)
{
	 app_timer_create(&si114x_Timestamps_timer_id, APP_TIMER_MODE_REPEATED, si114x_Timestamps_timeout_handler);
}
#endif


/***************************************************
* 功能：心率传感器初始化
*        
* retrun:
*       0 - ok  1 - failed   
***************************************************/
uint8_t si114x_init(void)
{
	uint8_t  uReturn = 0;
	
#if Auto_On_Off_Test
	si114x_Auto_On_Off_timer_init();
	si114x_On_Off_timer_start();
#endif
	
	Si114x_Vled_Int_Init();
	Si114x_AdcPin_SoftInt();

#if TIME_SAMPS_DEBUG
	si114x_Timestamps_time_init();
	si114x_Timestamps_start_timer_start();
#endif
	
	return uReturn;
}

/****************************************************************
* 功能：自动测量开/关
* 参数：autoFlg (AUTO_ON, AUTO_0FF) 
*               AUTO_ON  -  自动测量开    AUTO_0FF - 关    
* 返回：0 - ok  1 - failed      
* 注：初始化设置为自发运行模式的参数，通过往MEAS_RATE寄存器写0x00,心率传感器进入强制转换模式。关闭了内部振荡器。
*****************************************************************/
uint8_t  Auto_Measurement_Set(Si114x_auto_t autoFlg)
{
			NRF_LOG_INFO("Auto_Measurement_Set:%d\r\n", autoFlg);
			uint8_t uReturnValue = 0;
			
			if(autoFlg)
			{	   
				  uReturnValue = twi_master_Si114x_init();
				  if(uReturnValue)
					{	
						 twi_master_Si114x_uinit();
						 NRF_LOG_RAW_INFO("err-auto0.");
						 return 1;
					}
				  
					if(findSensors())
					{
						 twi_master_Si114x_uinit();
						 NRF_LOG_RAW_INFO("err-auto1");
						 return 1; //读取设备ID失败，返回1.心率传感器没有接好。
					} 

					/*Attention Please, When we start test, we will clear all old data*/
					structWatchModeHeartRate.u8HeartRate = 0;
                    structWatchModeBloodPressure.u8SystolicPressure = 0;
					structWatchModeBloodPressure.u8DiatolicPressure = 0;
					structGbWholeDayMeasure.u8BloodPressureDiatolicPressure = 0;
					structGbWholeDayMeasure.u8BloodPressureSystolicPressure = 0;
					structGbWholeDayMeasure.u8HeartRate = 0;
					
					Si114x_vLED_Init();
					Si114x_vLED_Enable();
					Si114x_AdcPin_HardInt_And_Enable();
					Si114x_ClrRxFifo();
					clear_heart_rate_data_handle_para();

					extern void Clear_Blood_Pressure(void);
					//娓呮琛�鍘嬬畻娉曠浉鍏冲彉閲?
					Clear_Blood_Pressure();
					
					//重新配置，解决长时间测心率时，算心率不准的问题。
					uReturnValue = Si114xConfigure(m_si114xTwi_master); 
					if(uReturnValue)
					{
						NRF_LOG_RAW_INFO("err-auto2.");
					}
			}
			else
			{
					Si114x_AdcPinHardUint_And_Disable();
					uReturnValue += Si114xPauseAll(m_si114xTwi_master); //Si114xPsPause -> Si114xPsAlsPause
				  if(uReturnValue)
					{
						NRF_LOG_RAW_INFO("err-auto50:%d", uReturnValue);
					}
				  uReturnValue += Si114xWriteToRegister(REG_IRQ_ENABLE, 0);	
					if(uReturnValue)
					{
						NRF_LOG_RAW_INFO("err-auto51:%d", uReturnValue);
					}
					Si114x_vLED_Disable();
					Si114x_Vled_Int_Init();//用于低功耗处理
					ClearWaveLevel();//清空波形等级
					uReturnValue += DynamicallyAdjustParmeters(0);//设置波形等级为A等级
					if(uReturnValue)
					{
						NRF_LOG_RAW_INFO("err-auto6:%d", uReturnValue); 
					}
					twi_master_Si114x_uinit();
                    
                    *(volatile uint32_t *)0x40003FFC = 0;
                    *(volatile uint32_t *)0x40003FFC;
                    *(volatile uint32_t *)0x40003FFC = 1;
			}
			
			return uReturnValue;
}


/******************************************************************
		设置LED0驱动电流
    LED0的驱动电流大小等级范围从1到15.
    返回：
        0 - ok    1  -  failed
******************************************************************/
uint8_t SetSi114x_Current(uint8_t uCurrentData)
{
		uint8_t ucdata[2] = {0, 0};
		
		if( (uCurrentData >=1) && (uCurrentData <= 15))
		{
				ucdata[0] = REG_PS_LED21;
				ucdata[1] = uCurrentData;
		
				return Si114x_Write(m_si114xTwi_master, ucdata, sizeof(ucdata)); 
		}
		
		return 1;
}

/***************************************************
* 功能：处理心率传感器中断
*        
* 注：在自主测量时，当测量间隔时间到了，产生中断，根据中断标记位可以去读取数据     
***************************************************/
extern void set_blood_pressure(void);
extern void vWholeDayHeartRateSet(uint8_t u8HeartRate);
extern void set_heart_rate(uint8_t heart_rate);
extern uint8_t real_time_ppg_upload_switch;
void command_id_0x02_key_id_0x11_upload_real_time_ppg_data_to_app(uint16_t ppg);
void Process_Si114x_INT(void)
{
	 uint16_t tmpData = 0;

	 if(Si114x_ReadFifo_Data(&tmpData))
	 {
#if COUNT_DATA_NUM
		  gu32Si114xGetFifoNum++;
#endif 
		 
#ifdef ENABLE_UPLOAD_REAL_TIME_PPG_DATA_TO_APP
			if(real_time_ppg_upload_switch == SWITCH_ON)
			{
				command_id_0x02_key_id_0x11_upload_real_time_ppg_data_to_app(tmpData);
			}
#endif
			
#ifdef ENABLE_COLLECTING_DATA_FOR_ANALYSIS
			uint8_t buffer[2] = {0};
			if(m_nus.is_notification_enabled)
			{
				buffer[0] = (uint8_t)((tmpData >> 0) & 0xFF);
				buffer[1] = (uint8_t)((tmpData >> 8) & 0xFF);
				ble_nus_string_send(&m_nus, buffer, 2);
			}
#endif
			
	if(CalHeartRateValue(tmpData) || CalBloodpressureValue())
			{
				vSetHeartRate(GetHeartRateValue());
				vWholeDayHeartRateSet(GetHeartRateValue());
				vSetBloodPressure();
			}

#if   PRINT_RAW_DATA
			NRF_LOG_RAW_INFO("%d\r\n", tmpData);
#endif
	 }		 
}	


/**********************************************************************
* 功能:测试函数，自动开启和关闭心率传感器，压力测试是否会失败。
* 测试时，打开此宏
**********************************************************************/
#if Auto_On_Off_Test
//定时器中隔一段时间自动打开或关闭心率传感器  
APP_TIMER_DEF(si114x_Auto_ON_OFF_timer_id);
uint8_t si114x_Auto_ON_OFF_count;
uint8_t si114x_Auto_ON_OFF_type;
uint16_t si114x_Auto_ON_OFF_handle;
uint16_t guAutoCount = 0;
#define AUTO_OFF_INTERVAL_1S		APP_TIMER_TICKS(1000)

typedef uint32_t(*timeout_callback_t1)(uint16_t handle);
timeout_callback_t1 Si1142_timeout_callback;

void si114x_On_Off_timer_start(void)
{
	app_timer_start(si114x_Auto_ON_OFF_timer_id, AUTO_OFF_INTERVAL_1S, NULL);
}

void si114x_On_Off_timer_stop(void)
{
	app_timer_stop(si114x_Auto_ON_OFF_timer_id);
}

void si114x_On_Off_timer_timeout_handler(void *p_context)
{
	  uint8_t ureturn  =  0;
	
		guAutoCount++;
	  if(guAutoCount == 1)
		{
				//extern void heart_rate_measure_start(void);
				//heart_rate_measure_start();
			  ureturn = Auto_Measurement_Set(AUTO_ON);
			  if(ureturn)
				{
						NRF_LOG_RAW_INFO("err-On_Off-0");
				}
				NRF_LOG_RAW_INFO("1\r\n");
		}
		else if(guAutoCount == 61)
		{
			  //void heart_rate_measure_stop(void);
			  //heart_rate_measure_stop();
			  ureturn = Auto_Measurement_Set(AUTO_OFF);
			  if(ureturn)
				{
						NRF_LOG_RAW_INFO("err-On_Off-1");
				}
				NRF_LOG_RAW_INFO("2\r\n");
		}
		else if(guAutoCount == 62)
		{
#if COUNT_DATA_NUM
			    if(gu32Si114xIntNum < 1250)
						NRF_LOG_RAW_INFO("LL:%d-%d-%d.\r\n", gu32Si114xIntNum, gu32Si114xGetFifoNum,gu32Si114xAdjustNum);

					if(gu32Si114xIntNum > 1600)
						NRF_LOG_RAW_INFO("HH:%d-%d-%d.\r\n", gu32Si114xIntNum, gu32Si114xGetFifoNum,gu32Si114xAdjustNum);
				  
					if( (gu32Si114xIntNum >= 1250) && (gu32Si114xIntNum <= 1600))
						NRF_LOG_RAW_INFO("g:%d-%d-%d.\r\n", gu32Si114xIntNum, gu32Si114xGetFifoNum,gu32Si114xAdjustNum);
				
			  gu32Si114xIntNum = 0;
			  gu32Si114xGetFifoNum = 0;
				gu32Si114xAdjustNum = 0;
#endif
			  guAutoCount=0;
		}
		
}

void si114x_Auto_On_Off_timer_init(void)
{
	  app_timer_create(&si114x_Auto_ON_OFF_timer_id, APP_TIMER_MODE_REPEATED, si114x_On_Off_timer_timeout_handler);
}

#endif


