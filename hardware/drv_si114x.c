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
* �汾��V1.3
* ����: zyh
* ʱ��: 2018��4��2��
* ���ܣ�si1142���ʴ���������
************************************************************/

#ifdef ENABLE_COLLECTING_DATA_FOR_ANALYSIS
extern ble_nus_t		 	m_nus;
#endif

#if COUNT_DATA_NUM
uint32_t gu32Si114xIntNum = 0; //�����жϵĴ���
uint32_t gu32Si114xGetFifoNum = 0; //��FIFO��ȡ���ݵĴ���
uint32_t gu32Si114xAdjustNum = 0;//���ε����Ĵ���
#endif

#define Si1142_PART_ID            0x42  //si1142��PART ID
#define Si114x_I2C_ADDRESS        0x5A  //slave address 

/*
#define Si114x_INT_PIN            11    //�ж��������  
#define Si114x_SDA_PIN            12    //I2C������
#define Si114x_SCL_PIN            13    //I2Cʱ����
#define Si114x_vLED_EN_PIN        14    //Si114x LED����ʹ������
*/

#define MASTER_TWI_INSTANCE       0    //!< TWI interface used as a master accessing Si114x

#define LOOP_TIMEOUT_MS           20   //200

static const nrf_drv_twi_t m_si114xTwi_master = NRF_DRV_TWI_INSTANCE(MASTER_TWI_INSTANCE);


#if TIME_SAMPS_DEBUG
	APP_TIMER_DEF(si114x_Timestamps_timer_id);//1msʱ�����ʱ��ID - ������֤si1142�Ĳ�����
	static uint32_t gu32Counter_1ms_Count = 0;
	static uint32_t gu32Timestamp1 = 0;
	static uint32_t gu32Timestamp2 = 0;
	#define SI114X_TIMESTAMPS_INTERVAL_1ms		 APP_TIMER_TICKS(1) 
#endif


/***************************************************************************
* ���ʴ������ӽ������ݵĶ���
*
***************************************************************************/

#define  SI114X_RXBUF_SIZE  250    //����FIFO�������
static uint16_t Si114x_HeartData_Buf[SI114X_RXBUF_SIZE];//����FIFIO����������
static uint16_t Si114x_Rx_IndexR = 0;//����FIFO�Ķ�ָ��
static uint16_t Si114x_Tx_IndexW = 0;//����FIFO ��дָ��


#define WAVE_A_GRADE   0x00  //A�ȼ��Ĳ���
#define WAVE_B_GRADE   0x01  //B�ȼ��Ĳ���
#define WAVE_C_GRADE   0x02  //C�ȼ��Ĳ���
#define WAVE_D_GRADE   0x03  //D�ȼ��Ĳ���

static uint8_t uwaveformLevel = WAVE_A_GRADE;
#define SI114X_LEVLE_NUM 4  //���εȼ���
uint8_t guWave[SI114X_LEVLE_NUM][2] = {
	                                      //���棬��������
	                                      {0x02, 0x04},  //A�ȼ����ζ�Ӧ�Ĳ���  04  04
																				{0x03, 0x04},  //B�ȼ����ζ�Ӧ�Ĳ���  02  04
																				{0x04, 0x04},  //C�ȼ����ζ�Ӧ�Ĳ���  03  04
																				{0x01, 0x09},  //D�ȼ����ζ�Ӧ�Ĳ���  01  09
                                      };

static uint8_t _sendCmd(nrf_drv_twi_t si114x_handle, uint8_t command);
static uint8_t Si114xParamRead(nrf_drv_twi_t si114x_handle, uint8_t address, uint8_t *outData);
																			
/*******************************************************************************
 * �������ݽ���FIFO��ʼ��
 * 
 ******************************************************************************/
void Si114x_ClrRxFifo(void)
{
		Si114x_Rx_IndexR  = 0;
    Si114x_Tx_IndexW  = 0;
}

//���ջ�����Ϊ��
// 1 - ��   0 - ��Ϊ��
uint8_t Si114x_Fifo_isempty(void)
{
    if(Si114x_Rx_IndexR == Si114x_Tx_IndexW ) return 1; 
 
    return 0;
}

/*******************************************************************************
* ���ܣ��ӻ�������ڶ�ȡ1�ֽ��ѽ��յ�����
* ��ڲ�������ȡ��������ŵĵ�ַָ��
* ���أ� 1 - ��ȡ�ɹ��� 0 - ��ȡʧ��
*******************************************************************************/
uint8_t Si114x_ReadFifo_Data(uint16_t *udata)
{
	  if(Si114x_Fifo_isempty())
		{
				return  0;  //���FIFO�������ݷ���0
		}
		
		//�漰FIFO����ʱ�������жϣ�����ָ����ҡ�
		*udata = Si114x_HeartData_Buf[Si114x_Rx_IndexR];
		if(++Si114x_Rx_IndexR >= SI114X_RXBUF_SIZE)
		{
				Si114x_Rx_IndexR = 0;
		}
		//FIFO������ϣ��ָ��ж�����

		return 1;
}

/*******************************************************************************
* ���ܣ��ӻ�������ڶ�ȡ1�ֽ��ѽ��յ�����
* ��ڲ�������ȡ��������ŵĵ�ַָ��
* ���أ� ��
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
*   ���ܣ�I2C����������ʼ��
*   return: 0 - OK   ��0 - Failed
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
*   ���ܣ�si114x��TWI������ʹ��
 ******************************************************************************/
void Twi_master_Si114x_Enable(void)
{
		nrf_drv_twi_enable(&m_si114xTwi_master);
}

/******************************************************************************
* @brief 
*   ���ܣ�si114x��TWI������ȥ��
 ******************************************************************************/
void Twi_master_Si114x_Disable(void)
{
		nrf_drv_twi_disable(&m_si114xTwi_master);
}

/******************************************************************************
* @brief 
*   ���ܣ�I2C��������ȥ��ʼ��
*   return: 0 - OK   ��0 - Failed
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
*   ���ܣ���Si114x�Ĵ��������� 
*   ������
*         si114x_handle   - I2C�����������
*         addr      			- �����ļĴ�����ַ
*         outdata   			- ������ݵĻ����׵�ַ
*         size      			- �������ݵĴ�С
*   return: 0 - OK   ��0 - Failed
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
*   ���ܣ���Si114x�Ĵ���д����
*   ������
*         si114x_handle   - I2C�����������
*         txdata    		  - ��д���ݻ���
*         size            - ��д���ݵĴ�С
*   return: 0 - OK   ��0 - Failed
*   ע�����ڸ�Si114xĳ���Ĵ���д���ݡ�ֻдһ���Ĵ�����
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
*   ���ܣ���Si114x�Ĵ���д�������
*   ������
*         si114x_handle   - I2C�����������
*         address    			- ��д�Ĵ�����ַ
*         length      	  - ���������ݵĴ�С
*   return: 0 - OK   ��0 - Failed
*   ע���˺�����si114x�Ĵ���д�������ʱ���Ĵ����ĵ�ַ�������ġ�
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
 *   Writes a byte to an Si113x/4x Parameter(��1���ֽ�д�뵽Si113x/4x ����)
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
 * ע�⣺�˹���ȷ��Si113x/4x��д�����֮ǰ���ڿ���״̬��׼���ý����������,
 * ������ɺ󱻼�飬������ò�������ȷ�������в�����������������Ĵ���
 * ǿ�ҽ�����������ʹ��������ܡ�
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
		 
		 if(count >= LOOP_TIMEOUT_MS) //��֤һ��
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
 si114x_handle - TWI���
 address       - ���������ַ
 outData       - �������ݵĵ�ַ
����          -  0 :�ɹ�    1:ʧ��
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
 *  Helper function to send a command to the Si113x/4x(����һ�����Si113x/Si114x)
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
	
	//NRF_LOG_RAW_INFO("C1:%d", count); //���ֵһ��Ϊ0��1
	
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
	//NRF_LOG_RAW_INFO("C2:%d", count);//���ֵһ��Ϊ0��1
	
	return 0;
}

/***************************************************************************//**
 * @brief 
 *   Pause measurement helper function(��ͣ������������)
 ******************************************************************************/
static uint8_t _PsAlsPause (nrf_drv_twi_t si114x_handle) 
{
  return _sendCmd(si114x_handle, 0x0B);
}


/***************************************************************************//**
 * @brief 
 *   Pauses autonomous measurements(��ͣ������������)
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
					tx_data[0] = REG_COMMAND; //Ҫд�ļĴ���
					tx_data[1] = 0x00;        //Ҫд�ļĴ�����ֵ
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
 *   Sends a PSALSAUTO command to the Si113x/4x������һ��PSALSAUTO���Si113x/4x,��ʼ�Զ�������
 *   ��ʼ�ٽ������ͻ����������
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

//��ʼ�ٽ��Է�����
uint8_t Si114xPsAuto (nrf_drv_twi_t si114x_handle)
{
  return _sendCmd(si114x_handle, 0x0D);
}

//ֹͣ�ٽ��Է����� 
uint8_t Si114xPsPause (nrf_drv_twi_t si114x_handle)
{
  return _sendCmd(si114x_handle, 0x09);
}

//ֹͣ�ɼ����Է����� 
uint8_t Si114xAlsPause (nrf_drv_twi_t si114x_handle)
{
  return _sendCmd(si114x_handle, 0x0A);
}

//ֹͣ�ٽ����ɼ����Է����� 
uint8_t Si114xPsAlsPause (nrf_drv_twi_t si114x_handle)
{
  return _sendCmd(si114x_handle, 0x0B);
}

/***************************************************************************//**
 * @brief 
* ����: ��λSi113x/4x,����κ��жϲ���ʼ��HW_KEY.  
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
	
	tx_Data[0] = REG_MEAS_RATE;//Ҫд�ļĴ�����ַ
	tx_Data[1] = 0x00;//Ҫд�ļĴ�����ֵ
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
* ���ܣ�����жϱ��λ
* ���أ�0 - ok  1 - failed
ע�����������ʴ��������жϱ��λʧ�ܣ���������жϣ���ȡ�����������ݡ�
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
 * ���ܣ�����si114x������һ��LED��������
 *  
 * ������si114x_handle - TWI�ľ��      
 *       greenLed      - ��Ҫ���õ�LED; GREEN_LED1,GREEN_LED2,GREEN_LED3
 *       ledCurrent    - ���õĵ�����С��
 * ���أ�0 - ok  1 - failed
 * 
 * ע�⣺si1141 - ֻ������LED1  si1142 - ������LED1,LED2  si1143 - ������LED1,LED2��LED3
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

//��֤������ȡ�������Ƿ���ȷ����ȷ����1�����󷵻�0������д
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
* ����:si114x���ʴ������Զ�����ģʽ�Ĳ�������
* ������
*      si114x_handle  - ���
*      current        - LED��������������
*      tasklist       - ����ѡ��
*      measrate       - �������ʱ��
*      adcgain        - LED�Ƶ�������
*      adcmisc        - �������ã�����2λ����������ԭʼADC��������PSͨ������������PS����ʱ�����ù���������ģʽ���Ǹ������Ȳ�������5λ����
                        ����ΪPSͨ��������������2λ��1��������ģʽ����5λ��1��
*      adcmux         - ����Ϊ���ͺ���������� ���� С�ͺ���������� 
* ���أ�
*      0 - ok    1 - failed
******************************************************************************/
uint8_t Si114xConfigure(nrf_drv_twi_t si114x_handle)
{ 
	 int16_t  current[3]= {LEDI_045, 0x00, 0x00};//��LED���������� LED1 = 22mA, LED2 = 0mA, LED2 = 0mA Si1142ֻ��LED1��LED2������Ŀֻ�õ�LED1.
   int16_t  tasklist= 0x31;  //ʹ��ALS PS1ͨ�� 
   int16_t  measrate= SI114X_MEASURE_RATE;  	                      
   int16_t  adcgain= 0x04;  // PS_ADC_GAIN(������) 0x00 - 25.6us, 0x01 - 51.2us  0x02 - 102.4us 0x03-204.8us  0x04-409.6us  0x05-819.2us.ȡֵ��Ҫ���ڵ���0x05(�ٷ�����).
   int16_t  adcmisc= 0x04;  // PS_ADC_MISC to default Normal Signal Range
   int16_t  adcmux[3]= {0x03, 0x03, 0x03};// PS1_ADCMUX, PS2_ADCMUX, PS3_ADCMUX to default large photodiode
	
   uint8_t  retval=0;
	 uint8_t tx_Data[2] = {0,0};
	 
	 
	 adcgain = guWave[0][0];//����A�ȼ��������������
	 current[0] = guWave[0][1];//����A�ȼ�������������������
	 
   // Note that the Si114xReset() actually performs the following functions:
   // 1. Pauses all prior measurements
   // 2. Clear  i2c registers that need to be cleared
   // 3. Clears irq status to make sure INT* is negated
   // 4. Delays 10 ms
   // 5. Sends HW Key
   retval += Si114xReset(si114x_handle);
	 
   // Get the LED current passed from the caller
   // If the value passed is negative, just pick a current... 202 mA
	 //����LED1��LED2��LED3����������
   {
       uint8_t i21, i3; 
		 
			 i21 = (current[1]<<4) + current[0];//current[0]ΪLED1������������current[1]ΪLED2����������
       i3  = current[2];//LED3����������
			 
			 tx_Data[0] = REG_PS_LED21;
			 tx_Data[1] = i21;
			 retval += Si114x_Write(si114x_handle, tx_Data, sizeof(tx_Data));//����LED1��LED2����������;LED2����������Ϊ0MA
			  
			 //LED3�������������ã�Si1142û���õ�
			 tx_Data[0] = REG_PS_LED3;
			 tx_Data[1] = i3;
			 retval += Si114x_Write(si114x_handle, tx_Data, sizeof(tx_Data));//����LED3���������� - 0MA
   }
	 
	 //VerifyWriteReadOK();
	 //����PS1��PS2��PS3ͨ����LED������ֻ�õ�PS1ͨ����PS1��LED1������PS2��PS3����Ϊû��LED��������������ã�Ĭ�϶�ʹ��LED1������
	 retval += Si114xParamSet(si114x_handle, PSLED12_SELECT, 0x01);//ָ��PS1�����ڼ䣬LED����������ΪLED1������PS2����û����������Ϊû�д�PS2������
	 retval += Si114xParamSet(si114x_handle, PSLED3_SELECT, 0x00);//PS3������LED������Ϊ�ޡ�

   // Initialize CHLIST Parameter from caller to enable measurement
   // Valid Tasks are: ALS_VIS_TASK, ALS_IR_TASK, PS1_TASK
   //                  PS2_TASK, PS3_TASK and AUX_TASK
   // However, if we are passed a 'negative' task, we will
   // turn on ALS_IR, ALS_VIS and PS1. Otherwise, we will use the 
   // task list specified by the caller.
   if (tasklist < 0 ) tasklist = PS1_TASK;
   retval += Si114xParamSet(si114x_handle, PARAM_CH_LIST, tasklist);//ʹ���ٽ�PS1ͨ�����ɼ��⻷������������⻷������  
   
	 //�ж����ʹ�ܼ��ж�ģʽ���á�
   // Set IRQ Modes and INT CFG to interrupt on every sample
	 //ֻҪIRQ_STATUS����Ӧ��IRQ_ENABLEλƥ�䣬INT���žͱ�����λ�͵�ƽ
	 tx_Data[0] = REG_INT_CFG;
	 tx_Data[1] = ICG_INTOE;	 
	 retval += Si114x_Write(si114x_handle, tx_Data, sizeof(tx_Data));
	 
	 //PS1ͨ������ʹ�ܣ���������(ALS)ʹ��
	 tx_Data[0] = REG_IRQ_ENABLE; 
   tx_Data[1] = (IE_ALS_EVRYSAMPLE + IE_PS1_EVRYSAMPLE);	
	 retval += Si114x_Write(si114x_handle, tx_Data, sizeof(tx_Data));
	 
	 //PS1��ALS�ж�ģʽѡ��
	 tx_Data[0] = REG_IRQ_MODE1;
   tx_Data[1] = (IM1_ALS_EVRYSAMPLE + IM1_PS1_EVRYSAMPLE);	
	 retval += Si114x_Write(si114x_handle, tx_Data, sizeof(tx_Data));

#if 0	 //0 - 1
   //PS3�ж�ģʽѡ��
	 tx_Data[0] = REG_IRQ_MODE2;
	 tx_Data[1] = IM2_PS3_EVRYSAMPLE;	 
	 retval += Si114x_Write(si114x_handle, tx_Data, sizeof(tx_Data)); 
#endif

	 //�ٽ�ͨ���������ã���LED�������ȣ�,Ҳ�ƻ���ʱ�䣬�����ֲ�Ҫ������ֵ��Ҫ����5.
	 retval += Si114xParamSet(si114x_handle, PARAM_PS_ADC_GAIN, adcgain);
	 //PS����ʱ��ʹ�÷�Χ���ã�1.��������Ϊ���ź������ʹ�ã�������ֱ�䣬2.���������   -  ѡ��Ϊ�������
	 retval += Si114xParamSet(si114x_handle, PARAM_PS_ADC_MISC, adcmisc);
	 //ΪPS1����ѡ��ADC���룬���ͺ���������ܣ�С�ͺ���������ܡ�ѡ�����
   retval += Si114xParamSet(si114x_handle, PARAM_PS1_ADC_MUX, adcmux[0]);

#if 0 //0  -  1
	 //PS2����ѡ������
	 retval += Si114xParamSet(si114x_handle, PARAM_PS2_ADC_MUX, adcmux[1]);
	 //PS3����ѡ������
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
* ���ܣ�дһ���Ĵ����ٶ�ȡ�üĴ�������֤��д�Ƿ�ɹ���������
* return:1 - OK    0 - ʧ��
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
* ���ܣ���PS1ͨ����ȡ��������
* �����������������ݵĻ���
* ����: 0 - ok  1 - failed
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
		error += Si114x_Read(m_si114xTwi_master, &Si114x_reg, &Data_Buffer[0], 1);//��8λ
		if(error != 0 )
		{
				NRF_LOG_RAW_INFO("error1");
			  //return 1;
		}
		
		Si114x_reg = REG_IRQ_PS1_DATA1;
		error += Si114x_Read(m_si114xTwi_master, &Si114x_reg, &Data_Buffer[1], 1);//�߰�λ
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
		
		//error += Si114x_Write(m_si114xTwi_master, Si114x_Tx_Data, sizeof(Si114x_Tx_Data)); //��PS1ͨ���жϱ��λ
		error += Si114x_Write(m_si114xTwi_master, Si114x_Tx_Data, sizeof(Si114x_Tx_Data)); //��PS1ͨ���жϱ��λ
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
* ���ܣ�д�Ĵ���
* ������regAddr - �Ĵ�����ַ    data - �Ĵ�������
* ����: 0 - ok  1 - failed
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


//��̬�������εȼ���Ӧ�Ĳ��� ���أ�0 - �ɹ�  1 - ʧ��
uint8_t DynamicallyAdjustParmeters(uint8_t waveState)
{
		uint8_t ureturn = 0;
		uint8_t uData = 0xFF;
	  uint8_t Si114x_reg;
	
#if 0
		uint8_t LastPara[2] = {0};
		LastPara[0] = guWave[uwaveformLevel][0];//��һ�εȼ�������
		LastPara[1] = guWave[uwaveformLevel][1];//��һ�εȼ�����������
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
		//��һ�β��εȼ��������뱾�εĲ�һ�������á�
		if(LastPara[0] != guWave[uwaveformLevel][0])
		{
				ureturn += Si114xParamSet(m_si114xTwi_master, PARAM_PS_ADC_GAIN, guWave[uwaveformLevel][0]);
				if(ureturn)
				{
						NRF_LOG_RAW_INFO("err-A%d-0",uwaveformLevel);
				}
		}
		
		//��һ�β��εȼ������������뱾�εĲ�һ�������á�
		if(LastPara[1] != guWave[uwaveformLevel][1])
		{
				ureturn += SetSi114x_Current(guWave[uwaveformLevel][1]);
				if(ureturn)
				{
					 NRF_LOG_RAW_INFO("err-A%d-1",uwaveformLevel);
				}
		}	
#else
		//���ж��������������������й�ϵô��
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

//��ղ��εȼ�
void ClearWaveLevel(void)
{
		uwaveformLevel = 3;
}


/***************************************************************************************************
* ���ܣ����ʴ������ж����ţ�����GPTE�жϵ��¼�����
* 
****************************************************************************************************/
extern uint8_t u8GbWrsitIsWearing;
void si114x_Int_handle(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	  uint16_t uheartData = 0;
	  uint8_t  uWaveState = 0;//����״̬ 0 - ���ԣ�1 - ����  2 - ����
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

			  //��ȡADC���ݼ����жϱ��λ,д���ݵ�FIFO.
				if(Si114x_GetHeartData(&uheartData) == 0)
				{
						Si114x_WriteFifo_Data(uheartData);
				}
         
				//��Ⲣ����δ����¼����������β���
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
* ���ܣ����ʴ�����ADC�ж���������
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
		//�����ж����ŵ���������
		in_config.pull = NRF_GPIO_PIN_PULLUP; 
		//���ø�����ΪGPIOTE����
		err_code = nrf_drv_gpiote_in_init(Si114x_INT_PIN, &in_config, si114x_Int_handle);
		APP_ERROR_CHECK(err_code);
		//ʹ�ܸ���������GPIOTEͨ�����¼�ģʽ
		nrf_drv_gpiote_in_event_enable(Si114x_INT_PIN, true);
}
#endif

/***************************************************************************************************
* ���ܣ����ʴ�����ADC�ж�����������ֳ�ʼ��
*       ��ΪҪ�õ�GPIOTE�жϣ���Ҫ�����ȳ�ʼ��
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
* ���ܣ����ʴ�����ADC�ж�����Ӳ�����ֳ�ʼ��  
*       
****************************************************************************************************/
void Si114x_AdcPin_HardInt_And_Enable(void)
{
		uint32_t err_code;
	
		nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
		//�����ж����ŵ���������
		in_config.pull = NRF_GPIO_PIN_PULLUP; 
		//���ø�����ΪGPIOTE����
		err_code = nrf_drv_gpiote_in_init(Si114x_INT_PIN, &in_config, si114x_Int_handle);
		APP_ERROR_CHECK(err_code);
		//ʹ�ܸ���������GPIOTEͨ�����¼�ģʽ
		nrf_drv_gpiote_in_event_enable(Si114x_INT_PIN, true);
}

/***************************************************************************************************
* ���ܣ����ʴ�����ADC�ж�����Ӳ��ȥ��ʼ����ȥʹ��
*       
****************************************************************************************************/
void Si114x_AdcPinHardUint_And_Disable(void)
{
		nrf_drv_gpiote_in_event_enable(Si114x_INT_PIN, false);//��ֹ�ж�
		nrf_drv_gpiote_in_uninit(Si114x_INT_PIN);
}

/***************************************************
* ���ܣ�Si114x LED�������ų�ʼ��
* 
***************************************************/
void Si114x_vLED_Init(void)
{
	nrf_gpio_cfg_output(Si114x_vLED_EN_PIN);
	
  return;	
}

/***************************************************
* ���ܣ�Si114x LED��������ʹ��
* 
***************************************************/
void Si114x_vLED_Enable(void)
{
	 nrf_gpio_pin_set(Si114x_vLED_EN_PIN);
	 
	 return;
}

/***************************************************
* ���ܣ�Si114x LED��������ȥ��
* 
***************************************************/
void Si114x_vLED_Disable(void)
{
	  nrf_gpio_pin_clear(Si114x_vLED_EN_PIN);
	  
	  return;
}


/***************************************************
* ���ܣ���ʼ��ʱ�������ж����ţ�LED��������Ϊ�������� - ���ڵ͹���
* 
***************************************************/
void Si114x_Vled_Int_Init(void)
{
	  nrf_gpio_cfg_output(Si114x_vLED_EN_PIN);
	  nrf_gpio_pin_clear(Si114x_vLED_EN_PIN);
	  nrf_gpio_cfg_input(Si114x_INT_PIN, NRF_GPIO_PIN_PULLUP);
}

/***************************************************
* ���ܣ��������ʴ�������PART ID 
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
//����ʱ�����ʱ��
void si114x_Timestamps_start_timer_start(void)
{
	 app_timer_start(si114x_Timestamps_timer_id, SI114X_TIMESTAMPS_INTERVAL_1ms, NULL);
}

//�ر�ʱ�����ʱ��
void si114x_Timestamps_stop_timer_stop(void)
{
	 app_timer_stop(si114x_Timestamps_timer_id);
}

//ʱ�����ʱ���ж�
void si114x_Timestamps_timeout_handler(void *p_context)
{
		gu32Counter_1ms_Count++;
}

//ʱ�����ʱ����ʼ��
void si114x_Timestamps_time_init(void)
{
	 app_timer_create(&si114x_Timestamps_timer_id, APP_TIMER_MODE_REPEATED, si114x_Timestamps_timeout_handler);
}
#endif


/***************************************************
* ���ܣ����ʴ�������ʼ��
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
* ���ܣ��Զ�������/��
* ������autoFlg (AUTO_ON, AUTO_0FF) 
*               AUTO_ON  -  �Զ�������    AUTO_0FF - ��    
* ���أ�0 - ok  1 - failed      
* ע����ʼ������Ϊ�Է�����ģʽ�Ĳ�����ͨ����MEAS_RATE�Ĵ���д0x00,���ʴ���������ǿ��ת��ģʽ���ر����ڲ�������
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
						 return 1; //��ȡ�豸IDʧ�ܣ�����1.���ʴ�����û�нӺá�
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
					//清楚血压算法相关变�?
					Clear_Blood_Pressure();
					
					//�������ã������ʱ�������ʱ�������ʲ�׼�����⡣
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
					Si114x_Vled_Int_Init();//���ڵ͹��Ĵ���
					ClearWaveLevel();//��ղ��εȼ�
					uReturnValue += DynamicallyAdjustParmeters(0);//���ò��εȼ�ΪA�ȼ�
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
		����LED0��������
    LED0������������С�ȼ���Χ��1��15.
    ���أ�
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
* ���ܣ��������ʴ������ж�
*        
* ע������������ʱ�����������ʱ�䵽�ˣ������жϣ������жϱ��λ����ȥ��ȡ����     
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
* ����:���Ժ������Զ������͹ر����ʴ�������ѹ�������Ƿ��ʧ�ܡ�
* ����ʱ���򿪴˺�
**********************************************************************/
#if Auto_On_Off_Test
//��ʱ���и�һ��ʱ���Զ��򿪻�ر����ʴ�����  
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


