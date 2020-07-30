#include "drv_Ads129x.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

void vADS1x9x_Disable_Start(void);
void vADS1x9x_Enable_Start(void);
void vHard_Stop_ADS1x9x (void);
void vADS1x9x_CS_HIGH(void);
void vADS129x_CS_LOW(void);
void vADS1x9x_PowerDown_Enable(void);
void vADS1x9x_PowerDown_Disable(void);

void vAds1x9x_read_data_stop_continue(void);
void vADS1x9x_SPI_Command_Data(uint8_t u8Ads129xCmd);
uint8_t u8ADS1x9x_Reg_Read(uint8_t Reg_address);
void vADS1x9x_Reg_Write(uint8_t Reg_address, uint8_t udata);

void vADS1x9x_Read_All_Regs(void);
void vADS1x9x_Start_Recording_Data(void);
void vAds1291_read_data_start_continue(void);
void vEnable_ADS1x9x_DRDY_Interrupt (void);
void vDisable_ADS1x9x_DRDY_Interrupt (void);

void vAds129x_PortInit(void);
void vAds129x_PowerEnable(Ads129x_power_t ePowerSwitch);
void vADS1x9x_Reset(void);
void vADS129X_Spi_Init(void);
void vADS1X9X_Initialize(void);

void vInit_ADS1x9x_DRDY_Interrupt (void);
	
void vADS129X_ClrRxFifo(void);
uint8_t u8ADS129X_Fifo_isempty(void);
uint8_t u8ADS129X_ReadFifo_Data(int32_t *udata);
void vADS129X_WriteFifo_Data(int32_t udata);


///////////////////////////////////////////////////////////////////////////////////////////////////

// 注： ADS1X9X的CLK_SEL引脚硬件上拉高了。该引脚用于Master clock 的选择，拉高选择

//ADS129X所用到的宏
#define NRF_DRV_SPI_ADS1X9X_CONFIG                           \
{                                                            \
    .sck_pin      = ADS1X9X_SPI_SCLK_PIN,                \
    .mosi_pin     = ADS1X9X_SPI_SIMO_PIN,                \
    .miso_pin     = ADS1X9X_SPI_SOMI_PIN,                \
    .ss_pin       = NRF_DRV_SPI_PIN_NOT_USED,                \
    .irq_priority = SPI_DEFAULT_CONFIG_IRQ_PRIORITY,         \
    .orc          = 0xFF,                                    \
    .frequency    = NRF_DRV_SPI_FREQ_1M,                     \
    .mode         = NRF_DRV_SPI_MODE_1,                      \
    .bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,         \
}

#define ADS1291_DEVICE_ID 0x52 
#define ADS1291_ECG_DATA_NUM 6 //心电数据个数-6个字节
uint8_t  gu8Ads1x9x_Ecg_data_buf[ADS1291_ECG_DATA_NUM] = {0}; 

#define ADS1X9X_SPI_INSTANCE  1      /**< SPI instance index. */
static volatile bool Ads1x9x_spi_xfer_done;  //SPI数据传输完成标志
static const    nrf_drv_spi_t Ads1x9x_Spi = NRF_DRV_SPI_INSTANCE(ADS1X9X_SPI_INSTANCE);  /**< SPI instance. */

static uint8_t    gu8Ads1x9x_spi_tx_buf[256];   /**< TX buffer. */
static uint8_t    gu8Ads1x9x_spi_rx_buf[256];   /**< RX buffer. */

#define  ADS129X_RXBUF_SIZE  500    //接收FIFO最大容量
static int32_t gi32ADS129X_EcgData_Buf[ADS129X_RXBUF_SIZE];//接收FIFIO缓冲区数组
static uint16_t ADS129X_Rx_IndexR = 0;//接收FIFO的读指针
static uint16_t ADS129X_Tx_IndexW = 0;//接收FIFO 的写指针

#define TEST_DEBUG  0  //调试宏 1- 打开  0 -关闭

#if TEST_DEBUG
	 static uint32_t gu32TestNum = 0;
#endif

uint32_t u32Ads129xStatus = 0;

void vAds1291xLeadOffStatusCheck(void)
{
	uint8_t u8LeadOffStatus = 0;
	u8LeadOffStatus = (uint8_t)((u32Ads129xStatus >> 15) & 0x0000001F);

	#if 0
	NRF_LOG_INFO("---RLD_STAT = %d\r\n", (u8LeadOffStatus & (1 << 4)) ? 1 : 0);
	NRF_LOG_INFO("---IN2N = %d\r\n", (u8LeadOffStatus & (1 << 3)) ? 1 : 0);
	NRF_LOG_INFO("---IN2P = %d\r\n", (u8LeadOffStatus & (1 << 2)) ? 1 : 0);
	NRF_LOG_INFO("---IN1N = %d\r\n", (u8LeadOffStatus & (1 << 1)) ? 1 : 0);
	NRF_LOG_INFO("---IN1P = %d\r\n", (u8LeadOffStatus & (1 << 0)) ? 1 : 0);
	#endif
}

//SPI 相关

/**
 * @brief SPI user event handler.
 * @param event
 */

void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    Ads1x9x_spi_xfer_done = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//ADS129X队列函数

//清ADS129X队列
void vADS129X_ClrRxFifo(void)
{
		ADS129X_Rx_IndexR  = 0;
    ADS129X_Tx_IndexW  = 0;
	
	  return;
}

//接收缓冲区为空
// 1 - 空   0 - 不为空
uint8_t u8ADS129X_Fifo_isempty(void)
{
    if(ADS129X_Rx_IndexR == ADS129X_Tx_IndexW ) return 1; 
 
    return 0;
}

/*******************************************************************************
* 功能：从缓冲队列内读取1字节已接收的数据
* 入口参数：读取数据所存放的地址指针
* 返回： 1 - 读取成功， 0 - 读取失败
*******************************************************************************/
uint8_t u8ADS129X_ReadFifo_Data(int32_t *udata)
{
	  if(u8ADS129X_Fifo_isempty())
		{
				return  0;  //如果FIFO内无数据返回0
		}
		
		
		*udata = gi32ADS129X_EcgData_Buf[ADS129X_Rx_IndexR];
		if(++ADS129X_Rx_IndexR >= ADS129X_RXBUF_SIZE)
		{
				ADS129X_Rx_IndexR = 0;
		}
		//FIFO操作完毕，恢复中断允许

		return 1;
}

//写数据到ADS129X队列
void vADS129X_WriteFifo_Data(int32_t udata)
{
		gi32ADS129X_EcgData_Buf[ADS129X_Tx_IndexW] = udata;
		if(++ADS129X_Tx_IndexW >= ADS129X_RXBUF_SIZE)
		{
			 ADS129X_Tx_IndexW = 0;	
		}
    
		return;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//ADS129X函数

//ADS129X上电前所有引脚初始化
void vAds129x_PortInit(void)
{
	 	nrf_gpio_cfg_output(ADS1X9X_RESET_PIN);
	  nrf_gpio_cfg_output(ADS1X9X_START_PIN);
	  nrf_gpio_cfg_output(ADS1X9X_DRDY_PIN);
	  nrf_gpio_cfg_output(ADS1X9X_SPI_SOMI_PIN);
	  nrf_gpio_cfg_output(ADS1X9X_SPI_SCLK_PIN);
	  nrf_gpio_cfg_output(ADS1X9X_SPI_SIMO_PIN);
	  nrf_gpio_cfg_output(ADS1X9X_SPI_CS_PIN);

#if ADS1291_DRIVE_RUN_IN_BLE
	  nrf_gpio_cfg_output(PERIPHERAL_VDD_EN_PIN_NUMBER);
#endif
		nrf_gpio_cfg_output(ADSX9X_DVDD_PIN);
	  nrf_gpio_cfg_output(ADSX9X_AVDD_PIN);
	
	  nrf_gpio_pin_clear(ADS1X9X_RESET_PIN);
	  nrf_gpio_pin_clear(ADS1X9X_START_PIN);
	  nrf_gpio_pin_set(ADS1X9X_DRDY_PIN);
    nrf_gpio_pin_clear(ADS1X9X_SPI_SOMI_PIN);
	  nrf_gpio_pin_clear(ADS1X9X_SPI_SCLK_PIN);
	  nrf_gpio_pin_clear(ADS1X9X_SPI_SIMO_PIN);
		nrf_gpio_pin_set(ADS1X9X_SPI_CS_PIN);
		
		return;
}

#if ADS1291_DRIVE_RUN_IN_BLE

void vperipheral_power_on_Vdd(void)
{
	 nrf_gpio_pin_clear(PERIPHERAL_VDD_EN_PIN_NUMBER);
}

void vperipheral_power_down_Vdd(void)
{
	 nrf_gpio_pin_set(PERIPHERAL_VDD_EN_PIN_NUMBER);
}

#endif


//ADS1291模拟电压供电 - 5V
void vADS129X_Avdd_Power_On(void)
{
	nrf_gpio_pin_set(ADSX9X_AVDD_PIN);
}

//ADS1291模拟电压掉电
void vADS129X_Avdd_Power_Down(void)
{
	 nrf_gpio_pin_clear(ADSX9X_AVDD_PIN);
}

//ADS1291数字电压供电 - 3.3V
void vADS129X_Dvdd_Power_On(void)
{
	nrf_gpio_pin_set(ADSX9X_DVDD_PIN);
}

//ADS1291数字电压掉电 
void vADS129X_Dvdd_Power_Down(void)
{
	 nrf_gpio_pin_clear(ADSX9X_DVDD_PIN);
}

//ADS129X电源引脚初始化
void vAds129x_PowerEnable(Ads129x_power_t ePowerSwitch)
{
#if ADS1291_DRIVE_RUN_IN_BLE
	  vperipheral_power_on_Vdd();//VDD 供电
	  nrf_delay_ms(10);
#endif
	  
	  if(ePowerSwitch == ADS129X_POWER_ON)
		{
			vADS129X_Avdd_Power_On();//AVDD 5V供电
			vADS129X_Dvdd_Power_On();//DVDD 3.3V供电
		}
		else
		{
			 vADS129X_Avdd_Power_Down();//AVDD 5V掉电
			 vADS129X_Dvdd_Power_Down();//DVDD 3.3V掉电
		}
	
	  return;
}  


void vADS1x9x_Disable_Start(void)
{
  nrf_gpio_pin_clear(ADS1X9X_START_PIN);  // Set Start pin to Low
	nrf_delay_ms(7);// Small Delay to settle 
	
	return;
}


void vADS1x9x_Enable_Start(void)
{
    nrf_gpio_pin_set(ADS1X9X_START_PIN);	// Set Start pin to High
	  nrf_delay_ms(10);// Small Delay to settle 
	
	  return;
}


void vHard_Stop_ADS1x9x (void)
{
		nrf_gpio_pin_clear(ADS1X9X_START_PIN);  // Set Start pin to Low
	  nrf_delay_ms(14);// Small Delay to settle  
	
	  return;
}


void vADS1x9x_CS_HIGH(void)
{
	nrf_gpio_pin_set(ADS1X9X_SPI_CS_PIN);
	
	return;
}


void vADS129x_CS_LOW(void)
{
   nrf_gpio_pin_clear(ADS1X9X_SPI_CS_PIN); 
	
	 return;
}


void vADS1x9x_PowerDown_Enable(void)
{
		nrf_gpio_pin_clear(ADS1X9X_START_PIN);
		nrf_delay_ms(5);
	
	  return;
}

void vADS1x9x_PowerDown_Disable(void)
{
	nrf_gpio_pin_set(ADS1X9X_RESET_PIN);
	nrf_delay_ms(14);
	
	return;
}



//ADS129X复位
void vADS1x9x_Reset(void)
{
	nrf_gpio_pin_set(ADS1X9X_RESET_PIN);
	nrf_delay_ms(1);
	nrf_gpio_pin_clear(ADS1X9X_RESET_PIN);
	nrf_delay_ms(1);
	nrf_gpio_pin_set(ADS1X9X_RESET_PIN);
	nrf_delay_ms(7);
	
	return;
}


void ADS1x9x_Read_Data1(uint8_t uLen, uint8_t *pbuf)
{
	Ads1x9x_spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&Ads1x9x_Spi, NULL, 0, pbuf, uLen));
	while(!Ads1x9x_spi_xfer_done);
	
	return ;
}

void vAds1291_read_data_continue(uint8_t* rx_buff)
{
	vADS129x_CS_LOW();
	nrf_delay_us(100);
	ADS1x9x_Read_Data1(6, rx_buff);
	nrf_delay_us(100);
	vADS1x9x_CS_HIGH();
}


//ADS129X的DRDY中断服务函数
void vADS1X9X_DRDY_Int_handle(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	static uint8_t uSdata[10]={0,};
	
	memset(uSdata, 0x00, sizeof(uSdata));
	vAds1291_read_data_continue(uSdata);
	vADS1x9x_Handle_EcgData(uSdata);
	
	return;
}

//ADS129X的DRDY中断使能
void vEnable_ADS1x9x_DRDY_Interrupt (void)
{
	  nrf_drv_gpiote_in_event_enable(ADS1X9X_DRDY_PIN, true);
}

//ADS129X的DRDY中断去能
void vDisable_ADS1x9x_DRDY_Interrupt (void)
{
	  nrf_drv_gpiote_in_event_disable(ADS1X9X_DRDY_PIN);
}

//初始化ADS129X的DRDY中断引脚
void vInit_ADS1x9x_DRDY_Interrupt (void)
{
	 ret_code_t err_code;
	
	 if(!nrf_drv_gpiote_is_init())
	 {
			err_code = nrf_drv_gpiote_init();
			APP_ERROR_CHECK(err_code);
	 }
	
	 nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
	 //开启中断引脚的上拉电阻
	 in_config.pull = GPIO_PIN_CNF_PULL_Pullup;
	 //配置该引脚为GPIOTE输入
	 //err_code = nrf_drv_gpiote_in_init(ADS1X9X_DRDY_PIN, &in_config, ADS1X9X_DRDY_Int_handle);
	 err_code = nrf_drv_gpiote_in_init(ADS1X9X_DRDY_PIN, &in_config, vADS1X9X_DRDY_Int_handle);
	 APP_ERROR_CHECK(err_code);
	 //使能该引脚所在GPIOTE通道的事件模式
	 nrf_drv_gpiote_in_event_disable(ADS1X9X_DRDY_PIN);
	 
	 return;
}

//去初始化ADS129X的DRDY中断引脚
void vUint_ADS1x9x_DRDY_Interrupt (void)
{
	 vDisable_ADS1x9x_DRDY_Interrupt();
	 nrf_drv_gpiote_in_uninit(ADS1X9X_DRDY_PIN);
}

////////////////////////////////////////////////////////////////////////////////////////////////
//发送命令
void vADS1x9x_SPI_Command_Data(uint8_t u8Ads129xCmd)
{
	 uint8_t len = 1;
	 ret_code_t error = 0;
	
	 gu8Ads1x9x_spi_tx_buf[0] = u8Ads129xCmd;
	 
   vADS129x_CS_LOW();
	 nrf_delay_us(1);
	 Ads1x9x_spi_xfer_done = false;
	 error = nrf_drv_spi_transfer(&Ads1x9x_Spi, gu8Ads1x9x_spi_tx_buf, len, NULL, 0);
	 //APP_ERROR_CHECK(nrf_drv_spi_transfer(&Ads1x9x_Spi, gu8Ads1x9x_spi_tx_buf, len, NULL, 0));
	 while(!Ads1x9x_spi_xfer_done);
	 nrf_delay_us(1);
	 vADS1x9x_CS_HIGH();
	
	 return;
}

void vStart_Read_Data_Continuous (void)
{
    vADS1x9x_SPI_Command_Data (RDATAC);					// Send 0x10 to the ADS1x9x
}

void vWake_Up_ADS1x9x (void)
{
    vADS1x9x_SPI_Command_Data (WAKEUP);         // Send 0x02 to the ADS1x9x                                                      
}

void vPut_ADS1x9x_In_Sleep (void)
{
    vADS1x9x_SPI_Command_Data (STANDBY);       // Send 0x04 to the ADS1x9x
}

void vSoft_Reset_ADS1x9x (void)
{
    vADS1x9x_SPI_Command_Data (RESET);         // Send 0x06 to the ADS1x9x
}

void vStart_Data_Conv_Command (void)
{
    vADS1x9x_SPI_Command_Data (START_);					// Send 0x08 to the ADS1x9x
}

void vSoft_Start_ADS1x9x (void)
{
    vADS1x9x_SPI_Command_Data (START_);          // Send 0x0A to the ADS1x9x
}

void vSoft_Stop_ADS1x9x (void)
{
    vADS1x9x_SPI_Command_Data (STOP);            // Send 0x0A to the ADS1x9x
}

void vStop_Read_Data_Continuous (void)
{
    vADS1x9x_SPI_Command_Data(SDATAC);					  // Send 0x11 to the ADS1x9x
}

//读取N个数据到pbuf中
void vADS1x9x_Read_Data(uint8_t uLen, uint8_t *pbuf)
{	
	vADS129x_CS_LOW();
	nrf_delay_us(1);
	Ads1x9x_spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&Ads1x9x_Spi, NULL, 0, pbuf, uLen));
	while(!Ads1x9x_spi_xfer_done);
	nrf_delay_us(1);
	vADS1x9x_CS_HIGH();
	
	return ;
}

//连续读取心电数据，读取6个字节的心电数据，前3个字节为心电状态数据，后3个字节为通道1的心电数据（24位）。
void vAds129x_read_data_continue(uint8_t* rx_buff)
{	
	vADS1x9x_Read_Data(6, rx_buff);//读取
}

//往ADS129X某个寄存器地址写数据
void vADS1x9x_Reg_Write(uint8_t Reg_address, uint8_t udata)
{
	 gu8Ads1x9x_spi_tx_buf[0] = (Reg_address | WREG);
	 gu8Ads1x9x_spi_tx_buf[1] = 0;
	 gu8Ads1x9x_spi_tx_buf[2] = udata;
	
	 vADS129x_CS_LOW();
	 nrf_delay_us(1);
	 Ads1x9x_spi_xfer_done = false;
	 //APP_ERROR_CHECK(nrf_drv_spi_transfer(&Ads1x9x_Spi, Ads1x9x_spi_tx_buf, 3, Ads1x9x_spi_rx_buf, 0));
	 APP_ERROR_CHECK(nrf_drv_spi_transfer(&Ads1x9x_Spi, gu8Ads1x9x_spi_tx_buf, 3, gu8Ads1x9x_spi_rx_buf, 3));
	 while(!Ads1x9x_spi_xfer_done);
	 nrf_delay_us(1);
	 vADS1x9x_CS_HIGH();
	
	 return;
}

//读取ADS129X的寄存器
uint8_t u8ADS1x9x_Reg_Read(uint8_t Reg_address)
{
	uint8_t len = 2;
	uint8_t retVal = 0;
  
	gu8Ads1x9x_spi_tx_buf[0] = (Reg_address | RREG);
	gu8Ads1x9x_spi_tx_buf[1] = 0;                  //要写的数据总数-1
	
	vADS129x_CS_LOW();
	nrf_delay_us(1);
	Ads1x9x_spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&Ads1x9x_Spi, gu8Ads1x9x_spi_tx_buf, len, gu8Ads1x9x_spi_rx_buf, 3));
	while(!Ads1x9x_spi_xfer_done);
	nrf_delay_us(1);
	vADS1x9x_CS_HIGH();
	
	retVal = gu8Ads1x9x_spi_rx_buf[2]; //查看DEMO数据是在第3个字节。
	
	return retVal;
}


//退出连续数据采集模式 - 发送停止连续数据转换命令，可以读取到寄存器表示发送退出了连续数据采集模式
void vAds1x9x_read_data_stop_continue(void)
{
		do
		{
			nrf_delay_ms(10);
			vADS1x9x_SPI_Command_Data(SDATAC);
			nrf_delay_ms(100);
		}while(u8ADS1x9x_Reg_Read(ADS1291_ID_CONFIG) != ADS1291_DEVICE_ID);   
}

//发送连续采集数据模式的命令  -  当读取不到设备ID表示命令发送成功
#if 1
void vAds1291_read_data_start_continue(void)
{
	do
	{
		nrf_delay_ms(10);
		vStart_Read_Data_Continuous(); 
		nrf_delay_ms(100);
	}while(u8ADS1x9x_Reg_Read(ADS1291_ID_CONFIG) == ADS1291_DEVICE_ID);
}
#else
void vAds1291_read_data_start_continue(void)
{
	do
	{
		nrf_delay_ms(10);
		vStart_Read_Data_Continuous(); 
		nrf_delay_ms(100);
	}while(u8ADS1x9x_Reg_Read(ADS1291_ID_CONFIG) == ADS1291_DEVICE_ID);
}
#endif

//读取ADS129X寄存器0~11的值
void vADS1x9x_Read_All_Regs(void)
{
	uint8_t Regs_i = 0;
	uint8_t uValue = 0;
	
	//printf("R0~R11:");

	for ( Regs_i = 0; Regs_i < 12; Regs_i++)
	{
		 uValue = u8ADS1x9x_Reg_Read(Regs_i);
		 nrf_delay_ms(10);
		 //printf("%02x-", uValue);
	}
	
	//printf("\r\n");
	
	return;
}

//处理得到的心电数据
void vADS1x9x_Handle_EcgData(uint8_t *buff)
{
	uint32_t u32_SatatusADS1x9xData = 0;
	uint32_t u32_EcgADS1x9xData = 0;
  int32_t tmpdata = 0;
	
#if TEST_DEBUG
	if(gu32TestNum == 5000)
	{
		 gu32TestNum = 5000;
		 return;
	}
	gu32TestNum++;
#endif

  //ECG status data
	u32_SatatusADS1x9xData = ((uint32_t)buff[0]) << 16;//16~23 bit
	u32_SatatusADS1x9xData += ((uint32_t)buff[1]) << 8;//8~15 bit
	u32_SatatusADS1x9xData += ((uint32_t)buff[2]);//0~7 bit

	u32Ads129xStatus = u32_SatatusADS1x9xData;
		
	//ECG data
	u32_EcgADS1x9xData = ((uint32_t)buff[3]) << 16;//16~23 bit
	u32_EcgADS1x9xData += ((uint32_t)buff[4]) << 8;//8~15 bit
	u32_EcgADS1x9xData += ((uint32_t)buff[5]);//0~7 bit 
	
	if( (u32_SatatusADS1x9xData & 0x00F00000) == 0x00C00000)
	{
		 if(u32_EcgADS1x9xData & 0x800000)
	   {
		    u32_EcgADS1x9xData ^= 0xFFFFFF;
		    u32_EcgADS1x9xData += 1;
		    tmpdata = u32_EcgADS1x9xData;
		    tmpdata = tmpdata * (-1);
	   }
		 else
		 {
			  tmpdata = u32_EcgADS1x9xData;
		 }
		 
		 vADS129X_WriteFifo_Data(tmpdata);
	}
	
}

//开始连续采集数据 - 设置为500SPS的采样率，即1秒钟采集500个数据。
#if 0
void vADS1x9x_Start_Recording_Data(void)
{
		vADS1x9x_Disable_Start(); //Set Start pin to Low
		nrf_delay_ms(5);
		vADS1x9x_Enable_Start(); // Set Start pin to High
		nrf_delay_ms(5);
		vADS1x9x_Disable_Start();// Set Start pin to Low
		nrf_delay_ms(5);
		vAds1291_read_data_start_continue(); //RDATAC command - Send 0x10 to the ADS1x9x
		nrf_delay_ms(5);
		vEnable_ADS1x9x_DRDY_Interrupt();// Enable DRDY interrupt
		nrf_delay_ms(5);
		vADS1x9x_Enable_Start();	// Set Start pin to High
}
#else
//不要太多的延时
void vADS1x9x_Start_Recording_Data(void)
{
		vADS1x9x_Disable_Start(); //Set Start pin to Low
		nrf_delay_us(5);
		vADS1x9x_Enable_Start(); // Set Start pin to High
		nrf_delay_us(5);
		vADS1x9x_Disable_Start();// Set Start pin to Low
		nrf_delay_us(5);
		vAds1291_read_data_start_continue(); //RDATAC command - Send 0x10 to the ADS1x9x
		nrf_delay_us(5);
		vEnable_ADS1x9x_DRDY_Interrupt();// Enable DRDY interrupt
		nrf_delay_us(5);
		vADS1x9x_Enable_Start();	// Set Start pin to High
}
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//寄存器设置

//CONFIG2的bit5设置 Reference buffer is enabled    -- >  ADS1291_REFRENCE_BUFF_DISABLE - 去能  ADS1291_REFRENCE_BUFF_ENABLE - 使能
void ads1291_set_refrence_buff(ADS1291_REFRENCE_BUFF_EN flags)
{
	uint8_t value = u8ADS1x9x_Reg_Read(ADS1291_REG_CONFIG2);
	value |= (0x01<<7);
	value &= ADS1291_REFRENCE_BUFF_MASK;
	value |= (flags << ADS1291_REFRENCE_BUFF_OFFSET);
	vADS1x9x_Reg_Write(ADS1291_REG_CONFIG2, value);
	
	return;
}

void ads1291_set_refrence_vol(ADS1291_REFRENCE_VOL flags)
{
	uint8_t value=	u8ADS1x9x_Reg_Read(ADS1291_REG_CONFIG2);
	
	value |= (0x01<<7);
	value &= ADS1291_REFRENCE_VOL_MASK;
	value |= (flags << ADS1291_REFRENCE_VOL_OFFSET);
	vADS1x9x_Reg_Write(ADS1291_REG_CONFIG2, value);
}

void ads1291_set_lead_off_comparator(ADS1291_LEAD_OFF_COMPARATOR flags)
{
	uint8_t value=	u8ADS1x9x_Reg_Read(ADS1291_REG_CONFIG2);
	value |= (0x01<<7);
	value &= ADS1291_LEAD_OFF_COMPARATOR_MASK;
	value |= (flags << ADS1291_LEAD_OFF_COMPARATOR_OFFSET);
	vADS1x9x_Reg_Write(ADS1291_REG_CONFIG2, value);
}

void ads1291_set_internal_clk_connect(ADS1291_INTERNAL_CLK_CONNECT_EN flags)
{
	uint8_t value = u8ADS1x9x_Reg_Read(ADS1291_REG_CONFIG2);
	value |= (0x01 << 7);
	value &= ADS1291_INTERNAL_CLK_CONNECT_MASK;
	value |= (flags << ADS1291_INTERNAL_CLK_CONNECT_OFFSET);
	vADS1x9x_Reg_Write(ADS1291_REG_CONFIG2, value);
}


void ads1291_test_signal_en(ADS1291_TEST_SIGNAL_EN flags)
{
	uint8_t value=	u8ADS1x9x_Reg_Read(ADS1291_REG_CONFIG2);
	value |= (0x01<<7);
	value &= ADS1291_TEST_SIGNAL_MASK;
	value |= (flags<<ADS1291_TEST_SIGNAL_OFFSET);
	vADS1x9x_Reg_Write(ADS1291_REG_CONFIG2, value);
}


void ads1291_set_mode(ADS1292R_MODE flags)
{
	uint8_t value=u8ADS1x9x_Reg_Read(ADS1291_REG_CONFIG1);	
	value&=ADS1292R_MODE_MASK;
	value|=(flags<<ADS1292R_MODE_FOFSET);
	vADS1x9x_Reg_Write(ADS1291_REG_CONFIG1,value);
}


void ads1291_set_sample_rate(ADS1292R_SAMPLE_RATE flags)
{
	uint8_t value=u8ADS1x9x_Reg_Read(ADS1291_REG_CONFIG1);	
	value&=ADS1292R_SAMPLE_RATE_MASK;
	value|=(flags<<ADS1292R_SAMPLE_RATE_OFFSET);
	vADS1x9x_Reg_Write(ADS1291_REG_CONFIG1,value);
}

void ads1291_set_lead_off_comp_th(ADS1292R_LEAD_OFF_COMP_TH flags)
{
	uint8_t value=u8ADS1x9x_Reg_Read(ADS1291_REG_LOFF);
	value&=ADS1292R_LEAD_OFF_COMP_TH_MASK;
	value|=(flags<<ADS1292R_LEAD_OFF_COMP_TH_OFFSET);
	vADS1x9x_Reg_Write(ADS1291_REG_LOFF,value);
}


void  ads1291_set_lead_off_current(ADS1292R_LEAD_OFF_CURRENT flags)
{
	uint8_t value=	u8ADS1x9x_Reg_Read(ADS1291_REG_LOFF);
	value&=ADS1292R_LEAD_OFF_CUR_MASK;
	value|=(flags<<ADS1292R_LEAD_OFF_CUE_OFFSET);
	vADS1x9x_Reg_Write(ADS1291_REG_LOFF,value);
}

void ads1291_set_lead_off_frq(ADS1292R_LEAD_OFF_FRQ flags)
{
	uint8_t value=	u8ADS1x9x_Reg_Read(ADS1291_REG_LOFF);
	value&=ADS1292R_LEAD_OFF_FRQ_MASK;
	value|=(flags<<ADS1292R_LEAD_OFF_FRQ_OFFSET);
	vADS1x9x_Reg_Write(ADS1291_REG_LOFF,value);
}

//0x04/0x05
void ads1291_set_channel1_pwdn(ADS1292R_CHANNEL_X_PWD flags)
{
	uint8_t value=	u8ADS1x9x_Reg_Read(ADS1291_REG_CH1SET);
	value&=ADS1292R_CHANNEL_X_PWD_MASK;
	value|=(flags<<ADS1292R_CHANNEL_X_PWD_OFFSET);
	vADS1x9x_Reg_Write(ADS1291_REG_CH1SET,value);
}


void ads1291_set_channel2_pwdn(ADS1292R_CHANNEL_X_PWD flags)
{
	uint8_t value=	u8ADS1x9x_Reg_Read(ADS1291_REG_CH2SET);
	value&=ADS1292R_CHANNEL_X_PWD_MASK;
	value|=(flags<<ADS1292R_CHANNEL_X_PWD_OFFSET);
	vADS1x9x_Reg_Write(ADS1291_REG_CH2SET,value);
}


void ads1291_channel1_pag(ADS1292R_CHANNEL_X_PAG flags)
{
	uint8_t value=	u8ADS1x9x_Reg_Read(ADS1291_REG_CH1SET);
	value&=ADS1292R_CHANNEL_PAG_MASK;
	value|=(flags<<ADS1292R_CHANNEL_PAG_OFFSET);
	vADS1x9x_Reg_Write(ADS1291_REG_CH1SET,value);
}

void ads1291_channel2_pag(ADS1292R_CHANNEL_X_PAG flags)
{
	uint8_t value=u8ADS1x9x_Reg_Read(ADS1291_REG_CH2SET);	
	value&=ADS1292R_CHANNEL_PAG_MASK;
	value|=(flags<<ADS1292R_CHANNEL_PAG_OFFSET);
	vADS1x9x_Reg_Write(ADS1291_REG_CH2SET,value);
}

void ads1291_set_channel1_input_select(ADS1292_CHANNEL_X_INPUT_SELECT flags)
{
	uint8_t value=	u8ADS1x9x_Reg_Read(ADS1291_REG_CH1SET);
	value&=ADS1292R_INPUT_SELECT_MASK;
	value|=(flags<<ADS1292R_INPUT_SELECT_OFFSET);
	vADS1x9x_Reg_Write(ADS1291_REG_CH1SET,value);
}


void ads1291_set_channel2_input_select(ADS1292_CHANNEL_X_INPUT_SELECT flags)
{
	uint8_t value=	u8ADS1x9x_Reg_Read(ADS1291_REG_CH2SET);
	value&=ADS1292R_INPUT_SELECT_MASK;
	value|=(flags<<ADS1292R_INPUT_SELECT_OFFSET);
	vADS1x9x_Reg_Write(ADS1291_REG_CH2SET,value);
}


//0x06
void ads1291_set_chop_frq(ADS1292R_PAG_CHOP_FRQ flags)
{
	uint8_t value=	u8ADS1x9x_Reg_Read(ADS1291_REG_RLDSENS);
	value&=ADS1292R_PAG_CHOP_FRQ_MASK;
	value|=(flags<<ADS1292R_PAG_CHOP_FRQ_OFFSET);
	vADS1x9x_Reg_Write(ADS1291_REG_RLDSENS,value);
}


void ads1291_set_rld_buff( ADS1292R_EN_OR_DIS flags)
{
	uint8_t value=	u8ADS1x9x_Reg_Read(ADS1291_REG_RLDSENS);
	value&=ADS1292R_RLD_BUFF_MASK;
	value|=(flags<<ADS1292R_RLD_BUFF_OFFSET);
	vADS1x9x_Reg_Write(ADS1291_REG_RLDSENS,value);
}

void ads1291_set_rld_lead_off_sense( ADS1292R_EN_OR_DIS flags)
{
	uint8_t value=	u8ADS1x9x_Reg_Read(ADS1291_REG_RLDSENS);
	value&=ADS1292R_RLD_LEAD_OFF_SENSE_MASK;
	value|=(flags<<ADS1292R_RLD_LEAD_OFF_SENSE_OFFSET);
	vADS1x9x_Reg_Write(ADS1291_REG_RLDSENS,value);
}

void ads1291_set_rld_connect_channel1_positive( ADS1292R_EN_OR_DIS flags)
{
	uint8_t value=u8ADS1x9x_Reg_Read(ADS1291_REG_RLDSENS);
	value&=ADS1292R_RLD_CONNECT_POSITIVE1_MASK;
	value|=(flags<<ADS1292R_RLD_CONNECT_POSITIVE1_OFFSET);
	vADS1x9x_Reg_Write(ADS1291_REG_RLDSENS,value);
}

void ads1291_set_rld_connect_channel2_positive( ADS1292R_EN_OR_DIS flags)
{
	uint8_t value=	u8ADS1x9x_Reg_Read(ADS1291_REG_RLDSENS);
	value&=ADS1292R_RLD_CONNECT_POSITIVE2_MASK;
	value|=(flags<<ADS1292R_RLD_CONNECT_POSITIVE2_OFFSET);
	vADS1x9x_Reg_Write(ADS1291_REG_RLDSENS,value);
}

void ads1291_set_rld_connect_channel1_negative( ADS1292R_EN_OR_DIS flags)
{
	uint8_t value=	u8ADS1x9x_Reg_Read(ADS1291_REG_RLDSENS);
	value&=ADS1292R_RLD_CONNECT_NEGATIVE1_MASK;
	value|=(flags<<ADS1292R_RLD_CONNECT_NEGATIVE1_OFFSET);
	vADS1x9x_Reg_Write(ADS1291_REG_RLDSENS,value);
}

void ads1291_set_rld_connect_channel2_negative( ADS1292R_EN_OR_DIS flags)
{
	uint8_t value=	u8ADS1x9x_Reg_Read(ADS1291_REG_RLDSENS);
	value&=ADS1292R_RLD_CONNECT_NEGATIVE2_MASK;
	value|=(flags<<ADS1292R_RLD_CONNECT_NEGATIVE2_OFFSET);
	vADS1x9x_Reg_Write(ADS1291_REG_RLDSENS,value);
}

void ads1291_set_loff_dir_current_sel_channel2( ADS1292R_EN_OR_DIS flags)
{
	uint8_t value=	u8ADS1x9x_Reg_Read(ADS1291_REG_LOFFSENS);
	value&=ADS1292R_CHANNEL2_CURRENT_DIR_SEL_MASK;
	value|=(flags<<ADS1292R_CHANNEL2_CRRENT_DIR_SEL_OFFSET);
	vADS1x9x_Reg_Write(ADS1291_REG_LOFFSENS,value);
}

void ads1291_set_loff_dir_current_sel_channel1( ADS1292R_EN_OR_DIS flags)
{
	uint8_t value=u8ADS1x9x_Reg_Read(ADS1291_REG_LOFFSENS);	
	value&=ADS1292R_CHANNEL1_CURRENT_DIR_SEL_MASK;
	value|=(flags<<ADS1292R_CHANNEL1_CRRENT_DIR_SEL_OFFSET);
	vADS1x9x_Reg_Write(ADS1291_REG_LOFFSENS,value);
}

void ads1291_set_channel2_loff_detection_neg_input( ADS1292R_EN_OR_DIS flags)
{
	uint8_t value=	u8ADS1x9x_Reg_Read(ADS1291_REG_LOFFSENS);
	value&=ADS1292R_CHANNEL2_LEAD_OFF_SEL_NEG_MASK;
	value|=(flags<<ADS1292R_CHANNEL2_LEAD_OFF_SEL_NEG_OFFSET);
	vADS1x9x_Reg_Write(ADS1291_REG_LOFFSENS,value);
}

void ads1291_set_channel2_loff_detection_pos_input( ADS1292R_EN_OR_DIS flags)
{
	uint8_t value=	u8ADS1x9x_Reg_Read(ADS1291_REG_LOFFSENS);
	value&=ADS1292R_CHANNEL2_LEAD_OFF_SEL_POS_MASK;
	value|=(flags<<ADS1292R_CHANNEL2_LEAD_OFF_SEL_POS_OFFSET);
	vADS1x9x_Reg_Write(ADS1291_REG_LOFFSENS,value);
}

void ads1291_set_channel1_loff_detection_neg_input( ADS1292R_EN_OR_DIS flags)
{
	uint8_t value=	u8ADS1x9x_Reg_Read(ADS1291_REG_LOFFSENS);
	value&=ADS1292R_CHANNEL1_LEAD_OFF_SEL_NEG_MASK;
	value|=(flags<<ADS1292R_CHANNEL1_LEAD_OFF_SEL_NEG_OFFSET);
	vADS1x9x_Reg_Write(ADS1291_REG_LOFFSENS,value);
}

void ads1291_set_channel1_loff_detection_pos_input( ADS1292R_EN_OR_DIS flags)
{
	uint8_t value=	u8ADS1x9x_Reg_Read(ADS1291_REG_LOFFSENS);
	value&=ADS1292R_CHANNEL1_LEAD_OFF_SEL_POS_MASK;
	value|=(flags<<ADS1292R_CHANNEL1_LEAD_OFF_SEL_POS_OFFSET);
	vADS1x9x_Reg_Write(ADS1291_REG_LOFFSENS,value);
}

void ads1291_set_modultor_clk_div( ADS1292R_CLK_DIV_SEL flags)
{
	uint8_t value=	u8ADS1x9x_Reg_Read(ADS1291_REG_LOFF_STAT);
	value&=ADS1292R_CLK_DIV_SEL_MASK;
	value|=(flags<<ADS1292R_CLK_DIV_SEL_OFFSET);
	vADS1x9x_Reg_Write(ADS1291_REG_LOFF_STAT,value);
}

void ads1291_get_rld_connect_status( uint8_t* data)
{
	uint8_t value=u8ADS1x9x_Reg_Read(ADS1291_REG_LOFF_STAT);
	data[0]=((value>>ADS1292R_RLD_IS_CONNECT_OFFSET)&&(0x01));
}

void ads1291_set_resp_demod(ADS1292R_RESP_DEMOD_EN1 flags)
{
	uint8_t value=	u8ADS1x9x_Reg_Read(ADS1291_REG_RESP1);
	value|=(0x01<<1);
	value&=ADS1292R_RESP_DEMOD_MASK;
	value|=(flags<<ADS1292R_RESP_DEMOD_OFFSET);
	vADS1x9x_Reg_Write(ADS1291_REG_RESP1,value);
}

void ads1291_set_resp_mod(ADS1292R_RESP_MOD_EN flags)
{
	uint8_t value=	u8ADS1x9x_Reg_Read(ADS1291_REG_RESP1);
	value|=(0x01<<1);
	value&=ADS1292R_RESP_MOD_MASK;
	value|=(flags<<ADS1292R_RESP_MOD_OFFSET);
	vADS1x9x_Reg_Write(ADS1291_REG_RESP1,value);
}

void ads1291_set_resp_phase(ADS1292R_RESP_PHASE flags)
{
	uint8_t value=	u8ADS1x9x_Reg_Read(ADS1291_REG_RESP1);
	value|=(0x01<<1);
	value&=ADS1292R_RESP_PHASE_MASK;
	value|=(flags<<ADS1292R_RESP_PHASE_OFFSET);
	vADS1x9x_Reg_Write(ADS1291_REG_RESP1,value);
}

void ads1291_set_resp_clk_mode(ADS1292R_RESP_CLK_MODE flags)
{
	uint8_t value=	u8ADS1x9x_Reg_Read(ADS1291_REG_RESP1);
	value|=(0x01<<1);
	value&=ADS1292R_RESP_CLK_MODE_MASK;
	value|=(flags<<ADS1292R_RESP_CLK_MODE_OFFSET);
	vADS1x9x_Reg_Write(ADS1291_REG_RESP1,value);
}

void ads1291_set_resp_calib(ADS1292_RESP_CALIB flags)
{
	uint8_t value=	u8ADS1x9x_Reg_Read(ADS1291_REG_RESP2);
	value|=(0x01<<0);
	value&=ADS1292R_RESP_CALIB_MASK;
	value|=(flags<<ADS1292R_RESP_CALIB_OFFSET);
	vADS1x9x_Reg_Write(ADS1291_REG_RESP2,value);
}


void ads1291_set_resp_frq(ADS1292R_RESP_FERQ flags)
{
	uint8_t value=	u8ADS1x9x_Reg_Read(ADS1291_REG_RESP2);
	value|=(0x01<<0);
	value&=ADS1292R_RESP_FREQ_MASK;
	value|=(flags<<ADS1292R_REFSP_FREQ_OFFSET);
	vADS1x9x_Reg_Write(ADS1291_REG_RESP2,value);
}

void ads1291_set_RLDREF_INT(ADS1292R_RLDREF_SOURCE flags)
{
	uint8_t value=	u8ADS1x9x_Reg_Read(ADS1291_REG_RESP2);
	value|=(0x01<<0);
	value&=ADS1292R_RLD_SOURCE_MASK;
	value|=(flags<<ADS1292R_RLD_SOURCE_OFFSET);
	vADS1x9x_Reg_Write(ADS1291_REG_RESP2,value);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
#define ADS1291_REG_CONFIG1     0x01
#define ADS1291_REG_CONFIG2     0x02
#define ADS1291_REG_LOFF        0x03
#define ADS1291_REG_CH1SET      0x04
#define ADS1291_REG_CH2SET      0x05
#define ADS1291_REG_RLDSENS     0x06
#define ADS1291_REG_LOFFSENS    0x07
#define ADS1291_REG_LOFF_STAT   0x08
#define ADS1291_REG_RESP1       0x09
#define ADS1291_REG_RESP2       0x0A
#define ADS1291_REG_GPIO        0x0B
52-02-e0-f0-00-81-23-03-00-02-07-0c-
*/

void vECGInterface_Init(void)
{

#if 1
		vInit_ADS1x9x_DRDY_Interrupt();
		vADS1x9x_Enable_Start();// Set START pin to High
	  vADS1x9x_CS_HIGH();
	  vADS1x9x_Reset();//diff 
	  vADS1x9x_Disable_Start();// Set START pin to LOW
	  vAds1x9x_read_data_stop_continue();
#else
	  nrf_delay_ms(24);
		vInit_ADS1x9x_DRDY_Interrupt();
	  nrf_delay_ms(12);
	  vADS1x9x_Disable_Start();// Set START pin to LOW
	  vADS1x9x_Enable_Start();// Set START pin to High
	  vHard_Stop_ADS1x9x();// Set START pin to Low
	  vStart_Data_Conv_Command();
	  vSoft_Stop_ADS1x9x();
	  nrf_delay_ms(8);
	  vAds1x9x_read_data_stop_continue();
	
#endif
	
#if 1

	  //vADS1x9x_Reg_Write(ADS1291_REG_CONFIG1, 0x02);//2ms
	  vADS1x9x_Reg_Write(ADS1291_REG_CONFIG1, 0x01);//4ms
	  vADS1x9x_Reg_Write(ADS1291_REG_CONFIG2,0xe0);
	  vADS1x9x_Reg_Write(ADS1291_REG_LOFF,0xf0);
	  vADS1x9x_Reg_Write(ADS1291_REG_CH1SET,0x00);
	  vADS1x9x_Reg_Write(ADS1291_REG_CH2SET, 0x81);
	  vADS1x9x_Reg_Write(ADS1291_REG_RLDSENS, 0x20);
	  vADS1x9x_Reg_Write(ADS1291_REG_LOFFSENS, 0x03);
	  vADS1x9x_Reg_Write(ADS1291_REG_LOFF_STAT, 0x00);
		vADS1x9x_Reg_Write(ADS1291_REG_RESP1, 0x02);
	  vADS1x9x_Reg_Write(ADS1291_REG_RESP2, 0x07);
	  vADS1x9x_Reg_Write(ADS1291_REG_GPIO, 0x30);
	
#else

	  //CONFIG2 - 0x02 - 1010 0000  0XA0  -  1010 000x 
		ads1291_set_refrence_buff(ADS1291_REFRENCE_BUFF_ENABLE);//PDB_REFBUF(bit5) 1
		ads1291_set_refrence_vol(ADS1291_REFRENCE_VOL_2420);//VREF_4V(bit4) 0(2.42-V) ADS1291_REFRENCE_VOL_2420 -> ADS1291_REFRENCE_VOL_4033 
		ads1291_set_lead_off_comparator(ADS1291_LEAD_OFF_COMPARATOR_ENABLE);//PDB_LOFF_COMP(bit6)  电极脱落检测
		nrf_delay_ms(10);
		ads1291_set_internal_clk_connect(ADS1291_INTERNAL_CLK_CONNECT_DISABLE);//CLK_EN bit3 0
		ads1291_test_signal_en(ADS1291_TEST_SIGNAL_OFF); //INT_TEST bit1 0

		//CONGFIG1 - 0x01 - 0000 0010 - 0x02
		ads1291_set_mode(ADS1292R_CONTINUE_MODE);//SINGLE_SHOT(bit7)  0(Continuous conversion mode)
		ads1291_set_sample_rate(ADS1292R_SAMPLE_RATE_500);// Channel oversampling ratio Bits[2:0]  000(125SPS)  ADS1292R_SAMPLE_RATE_125 -> ADS1292R_SAMPLE_RATE_500

		//LOFF   -  0x03 - 1111 0000 - 0xF0
		ads1291_set_lead_off_comp_th(ADS1292R_LEAD_OFF_COMP_TH_70); //Bits[7:5] 70%
		ads1291_set_lead_off_current(ADS1292R_LEAD_OFF_CUR_06_NA);//ILEAD_OFF[1:0] 6 nA 
		ads1291_set_lead_off_frq(ADS1292R_LEAD_OFF_DC);//FLEAD_OFF 0 = At dc lead-off detect

		//CH1SET  - 0x04 - 0110 0000 - 0x00   修改为0x00
		//CH2SET  - 0x05 - 0110 0000 - 0x60   修改为0x81
		ads1291_set_channel1_pwdn(ADS1292R_NOMAL_OPERATION);
		ads1291_set_channel2_pwdn(ADS1292R_POWER_DOWN);//ADS1292R_NOMAL_OPERATION -> ADS1292R_POWER_DOWN
		ads1291_channel1_pag(ADS1292R_PAG_6);
		ads1291_channel2_pag(ADS1292R_PAG_6);
		ads1291_set_channel1_input_select(ADS1292R_NOMAL_INPUT);
		ads1291_set_channel2_input_select(ADS1292R_INPUT_SHORT);//ADS1292R_NOMAL_INPUT -> ADS1292R_INPUT_SHORT

		//RLD_SENS - 0x06 - 0010 0000 - 0x20  
		ads1291_set_chop_frq(ADS1292R_CHOP_FRQ_16);
		ads1291_set_rld_buff(ADS1292R_ENABLE);
		ads1291_set_rld_lead_off_sense(ADS1292R_DISABLE);
		ads1291_set_rld_connect_channel1_positive(ADS1292R_DISABLE);//ADS1292R_ENABLE -> ADS1292R_DISABLE
		ads1291_set_rld_connect_channel2_positive(ADS1292R_DISABLE);//ADS1292R_ENABLE -> ADS1292R_DISABLE
		ads1291_set_rld_connect_channel1_negative(ADS1292R_DISABLE);//ADS1292R_ENABLE -> ADS1292R_DISABLE
		ads1291_set_rld_connect_channel2_negative(ADS1292R_DISABLE);//ADS1292R_ENABLE -> ADS1292R_DISABLE


		//LOFF_SENS - 0x07 - 0000 0011 - 0x0F  修改为0x03
		ads1291_set_loff_dir_current_sel_channel2(ADS1292R_DISABLE);
		ads1291_set_loff_dir_current_sel_channel1(ADS1292R_DISABLE);
		ads1291_set_channel2_loff_detection_neg_input(ADS1292R_DISABLE);//ADS1292R_ENABLE -> ADS1292R_DISABLE
		ads1291_set_channel2_loff_detection_pos_input(ADS1292R_DISABLE);//ADS1292R_ENABLE -> ADS1292R_DISABLE
		ads1291_set_channel1_loff_detection_neg_input(ADS1292R_ENABLE);
		ads1291_set_channel1_loff_detection_pos_input(ADS1292R_ENABLE);

		//LOFF_STAT - 0x08 - 0000 00000 - 0x00  
		ads1291_set_modultor_clk_div(ADS1292R_CLK_DIV_4);	


		//RESP1 - 0x09 - 0000 0010  - 0x02  - ADS1291必须设置为0x02
		ads1291_set_resp_demod(ADS1292R_RESP_DEMOD_OFF);
		ads1291_set_resp_mod( ADS1292R_RESP_MOD_OFF);
		//ADS1292R_RESP_PHASE
		ads1291_set_resp_phase( ADS1292R_RESP_PHASE_0);
		//ADS1292R_RESP_CLK_MODE
		ads1291_set_resp_clk_mode( ADS1292R_RESP_CLK_INTERNAL);

		//RESP2 - 0x0A - 0000 0111 - 0x07
		ads1291_set_resp_calib(ADS1292R_RESP_CALIB_OFF);
		ads1291_set_resp_frq(ADS1292R_RESP_FREQ_64);
		ads1291_set_RLDREF_INT(ADS1292R_RLDREF_INTERNAL);	
	
#endif

#if ADS1291_DRIVE_RUN_IN_BLE
	  vADS1x9x_Read_All_Regs();
		nrf_delay_ms(20);
		vADS1x9x_Start_Recording_Data();
#endif	
		
    
}


////////////////////////////////////////////////////////////////////////////////////////////////
//SPI 相关

/**
 * @brief SPI user event handler.
 * @param event
 */
/*
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    Ads1x9x_spi_xfer_done = true;
}
*/

//SPI控制器初始化
void vADS129X_Spi_Init(void)  
{
		nrf_drv_spi_config_t Ads1x9x_spi_config = NRF_DRV_SPI_ADS1X9X_CONFIG;
	  APP_ERROR_CHECK(nrf_drv_spi_init(&Ads1x9x_Spi, &Ads1x9x_spi_config, spi_event_handler, NULL));
	
	  return;
}

//SPI控制器去初始化
void vADS129X_Spi_Uinit(void)  
{
	 nrf_drv_spi_uninit(&Ads1x9x_Spi);
}

/////////////////////////////////////////////////////////////////////////////////////////////////

//ADS129X初始化
void vADS1X9X_Initialize(void)
{ 
	 vAds129x_PortInit();
	 vAds129x_PowerEnable(ADS129X_POWER_ON);
	 vADS129X_Spi_Init();
	 vADS129X_ClrRxFifo();
	 vECGInterface_Init();
}

/****************************************************************
* 功能：自动测量开/关
* 参数：autoFlg (AUTO_ON, AUTO_0FF) 
*               ADS129X_POWER_ON  -  自动测量开    ADS129X_POWER_OFF - 关    
* 返回：0 - ok  1 - failed     
*****************************************************************/
uint8_t  Ads129x_Auto_Measurement_Set(Ads129x_auto_t autoFlg)
{
		if(autoFlg == ADS129X_POWER_ON)
		{
              //high
              //nrf_gpio_pin_set(CHOSE_ECG_OR_TOUCH_PIN);
			  vAds129x_PortInit();
			  vAds129x_PowerEnable(ADS129X_POWER_ON);
			  vADS129X_Spi_Init();
			  vADS129X_ClrRxFifo();
			  vECGInterface_Init();
			  vADS1x9x_Start_Recording_Data();
		}
		else
		{
              //low
              //nrf_gpio_pin_clear(CHOSE_ECG_OR_TOUCH_PIN);
			  vAds1x9x_read_data_stop_continue();
			  vADS1x9x_PowerDown_Disable();//RESET引脚拉低5毫秒ADS1291即掉电
			  vAds129x_PowerEnable(ADS129X_POWER_OFF);//DVDD AVDD断电
			  vADS129X_Spi_Uinit();
			  vUint_ADS1x9x_DRDY_Interrupt();
		}
		
		return 0;
}

//处理获取到的心电数据
void vProcess_ADS1x9x_Data(void)
{
    uint32_t i = 0;
    static uint32_t gu32TestNum = 0;

    if(gu32TestNum == 5000)
    {
        //printf("start.\r\n");
        for(i = 0; i<5000; i++)
        {
            //printf("%d\r\n", gi32ADS129X_EcgData_Buf[i]);	
        }
    }
    nrf_delay_ms(2000);
}

