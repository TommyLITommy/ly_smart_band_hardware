#ifndef DRIVE_ADS129X_H__
#define DRIVE_ADS129X_H__

#include <stdint.h>
#include <stdio.h>


//#define ADS1291_NO_RUN_IN_BLE  0x01  //0x01 - ADS1291运行在裸机

/****************************************************************/
/* ADS1x9x COMMAND DESCRIPTION and definations */
/****************************************************************/
//System Commands
#define WAKEUP		0x02		//Wake-up from standby mode
#define STANDBY	0x04			//Enter standby mode
#define RESET		0x06			//Reset the device
#define START_		0x08		//Start/restart (synchronize) conversions
#define STOP		0x0A			//Stop conversion
 
//Data Read Commands
#define RDATAC		0x10		//Enable Read Data Continuous mode.
													//This mode is the default mode at power-up.
#define SDATAC		0x11		//Stop Read Data Continuously mode
#define RDATA		  0x12		//Read data by command; supports multiple read back.
 
//Register Read Commands
#define RREG		0x20			//Read n nnnn registers starting at address r rrrr
													//first byte 001r rrrr (2xh)(2) - second byte 000n nnnn(2)
#define WREG		0x40			//Write n nnnn registers starting at address r rrrr
													//first byte 010r rrrr (2xh)(2) - second byte 000n nnnn(2)


//#if ADS1291_DRIVE_RUN_IN_BLE
#if 1

#define PERIPHERAL_VDD_EN_PIN_NUMBER         4

#define ADS1X9X_RESET_PIN                   21             
#define ADS1X9X_START_PIN                   20
#define ADS1X9X_DRDY_PIN                    19
#define ADS1X9X_SPI_SOMI_PIN                18
#define ADS1X9X_SPI_SCLK_PIN                17
#define ADS1X9X_SPI_SIMO_PIN                16
#define ADS1X9X_SPI_CS_PIN                  15
//#define PERIPHERAL_ECG_PWR_EN_PIN           14
#define ADSX9X_AVDD_PIN           				  14
#define ADSX9X_DVDD_PIN                     9

#endif

#define ADS1291_ID_CONFIG       0x00
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

typedef enum
{
	ADS129X_AUTO_ON = 0x01,
  ADS129X_AUTO_OFF = 0x00
}Ads129x_auto_t;

typedef enum
{
	ADS129X_POWER_ON = 0x01,
  ADS129X_POWER_OFF = 0x00
}Ads129x_power_t;

enum ADS129x
{
		ADS1191_16BIT	 = 0,
		ADS1192_16BIT,
		ADS1291_24BIT,
		ADS1292_24BIT
};

#define CON_SIG_FLAG 1 //1-CON  0-SIG
#define V_REFRENCE   0  //0-2.42  1-4.03


//SET SAMPLE RATE   config1
typedef enum{
ADS1292R_SAMPLE_RATE_125=0X00,
ADS1292R_SAMPLE_RATE_250=0X01,
ADS1292R_SAMPLE_RATE_500=0X02,
ADS1292R_SAMOLE_RATE_1000=0X03,
ADS1292R_SAMPLE_RATE_2000=0X04,
ADS1292R_SAMPLE_RATE_4000=0X05,
ADS1292R_SAMPLE_RATE_8000=0X06,	
ADS1292R_SAMPLE_RATE_NOT_USE=0X07,
}ADS1292R_SAMPLE_RATE;
#define ADS1292R_SAMPLE_RATE_MASK 0X80
#define ADS1292R_SAMPLE_RATE_OFFSET 0


//sets the conversion mode  config1
typedef enum{
	ADS1292R_CONTINUE_MODE=0X00,
	ADS1292R_SINGLE_SHOT  =0X01,
}ADS1292R_MODE;
#define ADS1292R_MODE_MASK 	0X07
#define ADS1292R_MODE_FOFSET 0X07


 
//This bit determines whether the test signal is turned on or off.  config2
typedef enum
{
	ADS1291_TEST_SIGNAL_ON=0X01,
	ADS1291_TEST_SIGNAL_OFF=0X00,
}ADS1291_TEST_SIGNAL_EN;
#define ADS1291_TEST_SIGNAL_MASK 0XF9
#define ADS1291_TEST_SIGNAL_OFFSET 0X01

//This bit determines if the internal oscillator signal is connected to the CLK pin when an internal oscillator is used  config2
typedef enum
{
ADS1291_INTERNAL_CLK_CONNECT_DISABLE=0X00,
ADS1291_INTERNAL_CLK_CONNECT_ENABLE =0X01
}ADS1291_INTERNAL_CLK_CONNECT_EN;
#define ADS1291_INTERNAL_CLK_CONNECT_MASK 0xF3
#define ADS1291_INTERNAL_CLK_CONNECT_OFFSET 0X03

//This bit chooses between 2.42-V and 4.033-V reference
typedef enum
{
	ADS1291_REFRENCE_VOL_2420 = 0X00,
	ADS1291_REFRENCE_VOL_4033 = 0X01,
}ADS1291_REFRENCE_VOL;
#define ADS1291_REFRENCE_VOL_MASK   0xEB
#define ADS1291_REFRENCE_VOL_OFFSET 04

//This bit powers down the internal reference buffer so that the external reference can be used.  config2
typedef enum
{
ADS1291_REFRENCE_BUFF_DISABLE  = 0X00,
ADS1291_REFRENCE_BUFF_ENABLE   = 0X01,
}ADS1291_REFRENCE_BUFF_EN;
#define ADS1291_REFRENCE_BUFF_MASK 0XDB
#define ADS1291_REFRENCE_BUFF_OFFSET 5

//This bit powers down the lead-off comparators   config2
typedef enum
{
	ADS1291_LEAD_OFF_COMPARATOR_ENABLE=0X01,
	ADS1291_LEAD_OFF_COMPARATOR_DISANLE=0X00,
}ADS1291_LEAD_OFF_COMPARATOR;
#define ADS1291_LEAD_OFF_COMPARATOR_MASK 0XBB
#define ADS1291_LEAD_OFF_COMPARATOR_OFFSET 6


//These bits determine the lead-off comparator threshold.  config3
typedef enum{
ADS1292R_LEAD_OFF_COMP_TH_95	=0X00,
ADS1292R_LEAD_OFF_COMP_TH_92_5=0X01,
ADS1292R_LEAD_OFF_COMP_TH_90	=0X02,
ADS1292R_LEAD_OFF_COMP_TH_97_5=0X03,
ADS1292R_LEAD_OFF_COMP_TH_85	=0X04,
ADS1292R_LEAD_OFF_COMP_TH_80	=0X05,
ADS1292R_LEAD_OFF_COMP_TH_75	=0X06,
ADS1292R_LEAD_OFF_COMP_TH_70	=0X07,	
}ADS1292R_LEAD_OFF_COMP_TH;
#define ADS1292R_LEAD_OFF_COMP_TH_MASK 0X1D
#define ADS1292R_LEAD_OFF_COMP_TH_OFFSET 5

//These bits determine the magnitude of current for the current lead-off mode   config3
typedef enum{
ADS1292R_LEAD_OFF_CUR_06_NA=0X00,
ADS1292R_LEAD_OFF_CUR_22_NA=0X01,
ADS1292R_LEAD_OFF_CUR_06_UA=0X02,
ADS1292R_LEAD_OFF_CUR_22_UA=0X03,
}ADS1292R_LEAD_OFF_CURRENT;
#define ADS1292R_LEAD_OFF_CUR_MASK 0XF1
#define ADS1292R_LEAD_OFF_CUE_OFFSET 2

//This bit selects ac or dc lead-off   config3
typedef enum{
ADS1292R_LEAD_OFF_DC=0X00,
ADS1292R_LEAD_OFF_AC=0X01,
}ADS1292R_LEAD_OFF_FRQ;
#define ADS1292R_LEAD_OFF_FRQ_MASK 0XFC
#define ADS1292R_LEAD_OFF_FRQ_OFFSET 0


//0X04/0X05
//Channel X power-down
typedef	enum{
ADS1292R_NOMAL_OPERATION=0X00,
ADS1292R_POWER_DOWN=0X01,
}ADS1292R_CHANNEL_X_PWD;
#define ADS1292R_CHANNEL_X_PWD_MASK 0X7F
#define ADS1292R_CHANNEL_X_PWD_OFFSET 7

//Channel X PGA gain setting
typedef enum{
ADS1292R_PAG_6=0X00,
ADS1292R_PAG_1=0X01,
ADS1292R_PAG_2=0X02,
ADS1292R_PAG_3=0X03,
ADS1292R_PAG_4=0X04,
ADS1292R_PAG_8=0X05,
ADS1292R_PAG_12=0X06
}ADS1292R_CHANNEL_X_PAG;
#define ADS1292R_CHANNEL_PAG_MASK 0X8F
#define ADS1292R_CHANNEL_PAG_OFFSET 4

//These bits determine the channel 1 input selection.
typedef enum{
ADS1292R_NOMAL_INPUT=0X00,
ADS1292R_INPUT_SHORT=0X01,
ADS1292R_RLD_MEASURE=0X02,
ADS1292R_MVDD_MEASURE=0X03,
ADS1292R_TEMP_SENOR=0X04,
ADS1292R_TEST_SIGNAL=0X05,
ADS1292R_RLD_DRP=0X06,
ADS1292R_RLD_DRM=0X07,
ADS1292R_RLD_DRMP=0X08,
ADS1292R_IN3P_IN3N=0X09,
}ADS1292_CHANNEL_X_INPUT_SELECT;
#define ADS1292R_INPUT_SELECT_MASK 0XF0
#define ADS1292R_INPUT_SELECT_OFFSET 0

//0X06
//These bits determine PGA chop frequency
typedef enum{
ADS1292R_CHOP_FRQ_16=0X00,
ADS1292R_CHOP_FRQ_2=0X02,
ADS1292R_CHOP_FRQ_4=0X03,
}ADS1292R_PAG_CHOP_FRQ;
#define ADS1292R_PAG_CHOP_FRQ_MASK 0X3F
#define ADS1292R_PAG_CHOP_FRQ_OFFSET 6
typedef enum{
ADS1292R_ENABLE =0X01,
ADS1292R_DISABLE=0X00,
}ADS1292R_EN_OR_DIS;
#define ADS1292R_RLD_BUFF_MASK 0XDF
#define ADS1292R_RLD_BUFF_OFFSET 5

#define ADS1292R_RLD_LEAD_OFF_SENSE_MASK 0XEF
#define ADS1292R_RLD_LEAD_OFF_SENSE_OFFSET 4

#define ADS1292R_RLD_CONNECT_POSITIVE2_MASK 0XFB
#define ADS1292R_RLD_CONNECT_POSITIVE2_OFFSET 2
#define ADS1292R_RLD_CONNECT_NEGATIVE2_MASK 0XF7
#define ADS1292R_RLD_CONNECT_NEGATIVE2_OFFSET 3

#define ADS1292R_RLD_CONNECT_POSITIVE1_MASK 0XFE
#define ADS1292R_RLD_CONNECT_POSITIVE1_OFFSET 0
#define ADS1292R_RLD_CONNECT_NEGATIVE1_MASK 0XFD
#define ADS1292R_RLD_CONNECT_NEGATIVE1_OFFSET 1

//0X07
//This bit controls the direction of the current used for lead-off derivation for channel 2.
#define ADS1292R_CHANNEL2_CURRENT_DIR_SEL_MASK 0X1F
#define ADS1292R_CHANNEL2_CRRENT_DIR_SEL_OFFSET 5
//This bit controls the direction of the current used for lead-off derivation for channel 2.
#define ADS1292R_CHANNEL1_CURRENT_DIR_SEL_MASK 0X2F
#define ADS1292R_CHANNEL1_CRRENT_DIR_SEL_OFFSET 4
//This bit controls the selection of negative input from channel 2 for lead-off detection.//
#define ADS1292R_CHANNEL2_LEAD_OFF_SEL_NEG_MASK 0X37
#define ADS1292R_CHANNEL2_LEAD_OFF_SEL_NEG_OFFSET 3
//This bit controls the selection of POSITIVE input from channel 2 for lead-off detection.
#define ADS1292R_CHANNEL2_LEAD_OFF_SEL_POS_MASK 0X3B
#define ADS1292R_CHANNEL2_LEAD_OFF_SEL_POS_OFFSET 2
//This bit controls the selection of negative input from channel 1 for lead-off detection.
#define ADS1292R_CHANNEL1_LEAD_OFF_SEL_NEG_MASK 0X3D
#define ADS1292R_CHANNEL1_LEAD_OFF_SEL_NEG_OFFSET 1
//This bit controls the selection of PSODITIVE input from channel 1 for lead-off detection.
#define ADS1292R_CHANNEL1_LEAD_OFF_SEL_POS_MASK 0X3E
#define ADS1292R_CHANNEL1_LEAD_OFF_SEL_POS_OFFSET 0

//0X08
//This bit sets the modultar divider ratio between fCLK and fMOD. Two external clock values are supported: 512 kHz and
//2.048 MHz.
typedef enum{
ADS1292R_CLK_DIV_4=0X00,
ADS1292R_CLK_DIV_16=0X01,
}ADS1292R_CLK_DIV_SEL;
#define ADS1292R_CLK_DIV_SEL_MASK 0X1F
#define ADS1292R_CLK_DIV_SEL_OFFSET 6
//This bit determines the status of RLD.
#define ADS1292R_RLD_IS_CONNECT_OFFSET 4
//This bit determines if the channel 2 negative electrode is connected or not
#define ADS1292R_CHANNEL2_NEG_IS_CONNECT_OFFSET 3
//This bit determines if the channel 2 positive electrode is connected or not
#define ADS1292R_CHANNEL2_POS_IS_CONNECT_OFFSET 2
//This bit determines if the channel 1 negative electrode is connected or not
#define ADS1292R_CHANNEL1_NEG_IS_CONNECT_OFFSET 1
//This bit determines if the channel 2 positive electrode is connected or not
#define ADS1292R_CHANNEL1_POS_IS_CONNECT_OFFSET 0

//RESPIRATION CONTROL REG1
//0X09
//This bit enables and disables the demodulation circuitry on channel 1
typedef enum{
ADS1292R_RESP_DEMOD_OFF=0X00,
ADS1292R_RESP_DEMOD_ON=0X01,
}ADS1292R_RESP_DEMOD_EN1;
#define ADS1292R_RESP_DEMOD_MASK 0X7F
#define ADS1292R_RESP_DEMOD_OFFSET 7
//This bit enables and disables the modulation circuitry on channel 1
typedef enum{
ADS1292R_RESP_MOD_OFF=0X00,
ADS1292R_RESP_MOD_ON=0X01,
}ADS1292R_RESP_MOD_EN;
#define ADS1292R_RESP_MOD_MASK 0XBF
#define ADS1292R_RESP_MOD_OFFSET 6

//These bits control the phase of the respiration demodulation control signal.
typedef enum{//32khz is this 64khz is x2
ADS1292R_RESP_PHASE_0=0X00,
ADS1292R_RESP_PHASE_11_25=0X01,
ADS1292R_RESP_PHASE_22_5=0X02,
ADS1292R_RESP_PHASE_33_75=0X03,
ADS1292R_RESP_PHASE_45=0X04,
ADS1292R_RESP_PHASE_56_25=0X05,
ADS1292R_RESP_PHASE_67_5=0X06,
ADS1292R_RESP_PHASE_78_75=0X07,
ADS1292R_RESP_PHASE_90=0X08,
ADS1292R_RESP_PHASE_101_25=0X09,
ADS1292R_RESP_PHASE_112_5=0X0A,
ADS1292R_RESP_PHASE_123_75=0X0B,
ADS1292R_RESP_PHASE_135=0X0C,
ADS1292R_RESP_PHASE_146_25=0X0D,
ADS1292R_RESP_PHASE_157_5=0X0E,
ADS1292R_RESP_PHASE_168_75=0X0F,
}ADS1292R_RESP_PHASE;
#define ADS1292R_RESP_PHASE_MASK 0XC3
#define ADS1292R_RESP_PHASE_OFFSET 2
//This bit sets the mode of the respiration circuitry
typedef enum{
ADS1292R_RESP_CLK_INTERNAL=0X00,
ADS1292_RESP_CLK_EXTERN =0X01,
}ADS1292R_RESP_CLK_MODE;
#define ADS1292R_RESP_CLK_MODE_MASK 0XFE
#define ADS1292R_RESP_CLK_MODE_OFFSET 0

//RESPIRATION REG2
//0X0A
//This bit is used to enable offset calibration
typedef enum{
ADS1292R_RESP_CALIB_ON=0X01,
ADS1292R_RESP_CALIB_OFF=0X00,
}ADS1292_RESP_CALIB;
#define ADS1292R_RESP_CALIB_MASK 0X07
#define ADS1292R_RESP_CALIB_OFFSET 7
//This bit controls the respiration control frequency when RESP_CTRL = 0. This bit must be written with '1' for the ADS1291
//and ADS1292.
typedef enum{
ADS1292R_RESP_FREQ_32=0X00,
ADS1292R_RESP_FREQ_64=0X01,
}ADS1292R_RESP_FERQ;
#define ADS1292R_RESP_FREQ_MASK 0X83
#define ADS1292R_REFSP_FREQ_OFFSET 2
//This bit determines the RLDREF signal source
typedef  enum{
ADS1292R_RLDREF_EXTERN =0X00,
ADS1292R_RLDREF_INTERNAL=0X01,
}ADS1292R_RLDREF_SOURCE;
#define ADS1292R_RLD_SOURCE_MASK 0X85
#define ADS1292R_RLD_SOURCE_OFFSET 1

//0X0B
//General-Purpose I/O Register
typedef enum {
ADS1292R_GPIOX_INPUT =0X03,
ADS1292R_GPIOX_OUTPUT=0X00,
}ADS1292R_GPIOX_CTL;
#define ADS1292R_GPIOX_CTL_MASK 0X03
#define ADS1292R_GPIOX_CTL_OFFSET 2


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
void vAds129x_read_data_continue(uint8_t* rx_buff);
void vADS1x9x_Handle_EcgData(uint8_t *buff);

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
void vProcess_ADS1x9x_Data(void);	
uint8_t  Ads129x_Auto_Measurement_Set(Ads129x_auto_t autoFlg);
	
#endif
