/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) Nuvoton Technology Corp. All rights reserved.                                              */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/		 

// ---------------------------------------------------------------------------------------------------------
//	Functions:
//		- System clock configuration.
//		- Keypad configuration.
//		- SPI Flash configuration.
//		- Speaker configuration.
//		- MIC configuration.
//		- Output pin configuration.
//		- UltraIO configuration.
//		- Application Initiation.
//		- Processing loop:
//			* Codec processing(use functions in "AppFunctions.c").
//			* Voice effect processing(use functions in "AppFunctions.c").
//			* Keypad check and execution actions(use functions in "InputKeyActions.c").
//			* Etc.
//	
//	Reference "Readme.txt" for more information.
// ---------------------------------------------------------------------------------------------------------


#include "App.h"
#include "Framework.h"
#include "Keypad.h"
#include "SPIFlash.h"
#include "ConfigSysClk.h"
#include "MicSpk.h"
#include "config.h"

#ifdef OLED_ENABLE
#include "./OLED/LY096BG30.h"
#endif

#ifdef HDC1000_ENABLE
#include "./HDC1000/HDC1000.h"
uint16_t HUMITURESENSOR_Data[2]={0x4000,0x2000};//humidity tempreture;
#ifdef HDC1000_ENABLE
void  ConvertHTValue(void)
{
	uint32_t u32temp=0;
	uint8_t u8tmp0 = 0;
	uint8_t u8tmp1 = 0;
//	tmp0 = Read_HDC1000_Manufacturer_ID();
	u32temp = Read_HDC1000_Temperature();
	u32temp = (( u32temp*16500)>>16)-4000;
	u8tmp0 = u32temp/100;
	u8tmp1 = u32temp%100;
	HUMITURESENSOR_Data[1] = u8tmp0<<8|u8tmp1;
	u32temp = Read_HDC1000_Humidity();
	u32temp = (u32temp*10000)>>16;
	u8tmp0 = u32temp/100;
	u8tmp1 = u32temp%100;
	HUMITURESENSOR_Data[0] = u8tmp0<<8|u8tmp1;
#ifdef DEBUG_ENABLE
	printf("tempriture is:%x  humidity is:%x \n",HUMITURESENSOR_Data[1],HUMITURESENSOR_Data[0]);
#endif
}

#endif

#endif
#ifdef MPU6050_ENABLE
#include "./MPU6050/MPU6050_api.h"
int16_t iEuler[3];
int16_t R, P, Y;
uint16_t iGyroX,iGyroY,iGyroZ;

uint16_t transfer(uint16_t input)
{
	if(input > 0x7FFF)
		return ~(input - 0x8000) + 1;
	else
		return	input;
}
void ConvertMPU6050Value(void)
{
	float Euler[3] ;
	uint8_t i=0;

	rawACC[0] = Read_MPU6050_AccX();
	rawACC[1] = Read_MPU6050_AccY();
	rawACC[2] = Read_MPU6050_AccZ();

	rawGYRO[0] = Read_MPU6050_GyroX();
	rawGYRO[1] = Read_MPU6050_GyroY();
	rawGYRO[2] = Read_MPU6050_GyroZ();
	nvtInputSensorRawACC(rawACC);
	nvtInputSensorRawGYRO(rawGYRO);
		
	nvtUpdateAHRS(SENSOR_ACC|SENSOR_GYRO);
	nvtGetEulerRPY(Euler);
	for (i=0 ; i<3 ; i++)
		iEuler[i] = (int) (Euler[i]);	


	R = transfer(iEuler[0]);
	P = transfer(iEuler[1]);
	Y = transfer(iEuler[2]);

	// Transfer Data formate for Android APP
	iGyroX = transfer(rawGYRO[0]);
	iGyroY = transfer(rawGYRO[1]);
	iGyroZ = transfer(rawGYRO[2]);
			
	//printf("iEuler[]=%x %x %x %x \n",iEuler[0],iEuler[1],iEuler[2],iEuler[3]);s				


}

#endif



#if( !defined(__CHIP_SERIES__) )
#error "Please update SDS version >= v5.0."
#endif

// SPI flash handler.
S_SPIFLASH_HANDLER g_sSpiFlash;
// Application control.
volatile UINT8 g_u8AppCtrl;
// Application handler.
S_APP g_sApp;

extern void App_Initiate(void);
extern BOOL App_StopPlay(void);
extern BOOL App_ProcessPlay(void);

UINT8 SPIFlash_Initiate(void)
{ 
	UINT16 ui16Temp;
	UINT32 ui32Temp;
	UINT32 u32Count;

	// SPI0: GPA1=SSB00, GPA2=SCLK0, GPA3=MISO0, GPA4=MOSI0 
	SYS->GPA_MFP  = 
		(SYS->GPA_MFP & (~(SYS_GPA_MFP_PA0MFP_Msk|SYS_GPA_MFP_PA1MFP_Msk|SYS_GPA_MFP_PA2MFP_Msk|SYS_GPA_MFP_PA3MFP_Msk)) )
		| (SYS_GPA_MFP_PA0MFP_SPI_MOSI0|SYS_GPA_MFP_PA1MFP_SPI_SCLK|SYS_GPA_MFP_PA2MFP_SPI_SSB0|SYS_GPA_MFP_PA3MFP_SPI_MISO0);	
	
	// Reset IP module
	CLK_EnableModuleClock(SPI0_MODULE);
	SYS_ResetModule(SPI0_RST);
	SPIFlash_Open(SPI0, SPI_SS0, SPI0_CLOCK, &g_sSpiFlash );

	// Make SPI flash leave power down mode if some where or some time had made it entring power down mode
	SPIFlash_PowerDown(&g_sSpiFlash, FALSE);
	
	// Check SPI flash is ready for accessing
	u32Count = ui32Temp = 0;
	while(u32Count!=100)
	{
		SPIFlash_Read(&g_sSpiFlash, 0, (PUINT8) &ui16Temp, 2);
		if ( ui32Temp != (UINT32)ui16Temp )
		{
			ui32Temp = (UINT32)ui16Temp;
			u32Count = 0;
		}
		else
			u32Count++;
	}

	// The following code can be remove to save code if the flash size is not necessary for this application
	SPIFlash_GetChipInfo(&g_sSpiFlash);
	if (g_sSpiFlash.u32FlashSize == 0)
		return 0;
	
	// The above code can be remove to save code if the flash size is not necessary for this application
	return 1;
}
void i2c_Init(void)  //ericyang 20151120 add for oled i2c driver
{
	CLK_EnableModuleClock(I2C0_MODULE);//ericyang 20151120
	SYS_ResetModule(I2C0_RST);	
	SYS->GPB_MFP  = (SYS->GPB_MFP & (~SYS_GPB_MFP_PB2MFP_Msk) ) | SYS_GPB_MFP_PB2MFP_I2C_SCL;
	SYS->GPB_MFP  = (SYS->GPB_MFP & (~SYS_GPB_MFP_PB3MFP_Msk) ) | SYS_GPB_MFP_PB3MFP_I2C_SDA;
	I2C_Open(I2C0, 400000);

}
#ifdef UART_TRANS_ENABLE
uint8_t GLOBAL_BUF[256];//ericyang 20151118
uint8_t real_rx_count=0;

/**
 *    @brief    The function is to write data into TX buffer to transmit data by UART.
 *
 *    @param[in]    uart            The base address of UART module.
 *    @param[in]    pu8TxBuf        The buffer to send the data to UART transmission FIFO.
 *    @param[in]    u32WriteBytes    The byte number of data.
 *
 *  @return u32Count: transfer byte count
 */
uint32_t UART_Write_Wecan(UART_T* uart,char *pu8TxBuf, uint32_t u32WriteBytes)
{
	uint32_t  u32Count, u32delayno;

	for(u32Count=0; u32Count != u32WriteBytes; u32Count++) 
	{
		u32delayno = 0;
		while((uart->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk) == 0) 
		{ /* Wait Tx empty and Time-out manner */
			u32delayno++;
			if( u32delayno >= 0x40000000 )
	    			return FALSE;
		}
		uart->DAT = pu8TxBuf[u32Count];    /* Send UART Data from buffer */
	}
	return u32Count;
}															

/**
 *    @brief    Set Rx FIFO interrupt Trigger Level
 *
 *    @param    uart        The base address of UART module
 *    @param    u32TriggerLevel   RX FIFO interrupt Trigger Level. ( \ref UART_FIFO_RFITL_1BYTE / \ref UART_FIFO_RFITL_4BYTE / 
 *                                                                   \ref UART_FIFO_RFITL_8BYTE / \ref UART_TLCTL_RFITL_14BYTES )
 *    @return    None
 */
#define UART_SET_RX_FIFO_INTTRGLV(uart, u32TriggerLevel)   ((uart)->FIFO  = (((uart)->FIFO  &  ~UART_FIFO_RFITL_Msk) | (u32TriggerLevel))

void UART0_IRQHandler(void)
{
	uint32_t u32IntSts= UART0->INTSTS;
	#ifdef NUVOTON_BLE_WIFI_ENABLE
	uint8_t i;
	#else
	uint8_t u8count = 0;
	#endif
	if(u32IntSts & UART_IS_RX_READY(UART0)) 
	{	
	#ifdef NUVOTON_BLE_WIFI_ENABLE
		UART_Read(UART0, RX_buffer, RXBUFSIZE);
		for(i=0;i<4;i++)
			command[i]=RX_buffer[i];
		Flag_report=1;
		//print_Line(3,RX_buffer);
		//uartno = 0;
	#else
		u8count = UART_Read(UART0, GLOBAL_BUF+real_rx_count, 256);
		real_rx_count += u8count;
	#endif

	}
}
void UART_Init(void)
{

	CLK_EnableModuleClock(UART_MODULE);//ericyang 20160105
	SYS_ResetModule(UART0_RST);//ericyang 20160105	

	/* Set GPG multi-function pins for UART0 RXD and TXD */
	SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA8MFP_Msk) ) | SYS_GPA_MFP_PA8MFP_UART_TX;
	SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA9MFP_Msk) ) | SYS_GPA_MFP_PA9MFP_UART_RX;

	/* Configure UART0 and set UART0 Baudrate(115200) */
	UART_Open( UART0,115200 );

	#ifdef NUVOTON_BLE_WIFI_ENABLE//ericyang 20151118 test polling not use interrupt
	UART_SET_RX_FIFO_INTTRGLV(UART0, UART_FIFO_RFITL_4BYTES));
	#else
	UART_SET_RX_FIFO_INTTRGLV(UART0, UART_FIFO_RFITL_1BYTE));
	#endif
	//UART_ENABLE_INT(UART0, UART_INTEN_RDAIEN_Msk);
	UART0->INTEN |= UART_INTEN_RDAIEN_Msk;
	UART0->FIFO |= UART_FIFO_RXRST_Msk;
	NVIC_EnableIRQ(UART0_IRQn);		
}

#endif

void System_Init(void)  //ericyang 20160105
{
										
	SYSCLK_INITIATE();				// Configure CPU clock source and operation clock frequency.
									// The configuration functions are in "SysClkConfig.h"
	
	CLK_EnableLDO(CLK_LDOSEL_3_3V);	// Enable interl 3.3 LDO.
	
	if (! SPIFlash_Initiate())		// Initiate SPI interface and checking flows for accessing SPI flash.
		while(1); 					// loop here for easy debug

	OUTPUTPIN_INITIATE();			// Initiate output pin configuration.
									// The output pins configurations are defined in "ConfigIO.h".
	
	ULTRAIO_INITIATE();				// Initiate ultraio output configurations.
									// The ultraio output pin configurations are defined in "ConfigUltraIO.h"
	
	KEYPAD_INITIATE();				// Initiate keypad configurations including direct trigger key and matrix key
									// The keypad configurations are defined in "ConfigIO.h".
		
	PDMA_INITIATE();				// Initiate PDMA.
									// After initiation, the PDMA engine clock NVIC are enabled.
									// Use PdmaCtrl_Open() to set PDMA service channel for desired IP.
									// Use PdmaCtrl_Start() to trigger PDMA operation.
									// Reference "PdmaCtrl.h" for PDMA related APIs.
									// PDMA_INITIATE() must be call before SPK_INITIATE() and MIC_INITIATE(), if open MIC or speaker.
	
	SPK_INITIATE();					// Initiate speaker including pop-sound canceling.
									// After initiation, the APU is paused.
									// Use SPK_Resume(0) to start APU operation.
									// Reference "MicSpk.h" for speaker related APIs.

	MIC_INITIATE();					// Initiate MIC.
									// After initiation, the ADC is paused.
									// Use ADC_Resume() to start ADC operation.
									// Reference "MicSpk.h" for MIC related APIs.
	
																	
	App_Initiate();					// Initiate application for audio decode.

}
//---------------------------------------------------------------------------------------------------------
// Main Function                                                           
//---------------------------------------------------------------------------------------------------------
INT32 main()
{
	uint32_t u32temp=0;
	char tmpString[16];

	System_Init();
	i2c_Init(); //need to excute  before #define I2C_IRQ
#ifdef UART_TRANS_ENABLE
	UART_Init();
#endif
#ifdef I2C_IRQ
	I2C_EnableInt(I2C0);
	NVIC_EnableIRQ(I2C0_IRQn);
	NVIC_SetPriority(I2C0_IRQn, 0);
#endif
#ifdef MPU6050_ENABLE
	Init_MPU6050();
#endif

#ifdef OLED_ENABLE
	Init_LCD();
	clear_LCD();

	print_Line(0, "OscarNuLiteEx");
//	print_Line(1, "2015.11.12      ");
//	print_Line(2, "Eric Yang      ");
//	print_Line(3, "0.96 OLED 128x64");
#endif
#ifdef HDC1000_ENABLE
	Init_HDC1000();
#endif
#ifdef MPU6050_ENABLE
	Init_AHRS();
	//Calibrate Gyro
	GyroCalibrate();
	//Calibrate Accel
	AccCalibrationZ();
#endif

	while (1)
	{

		if (( g_u8AppCtrl&APPCTRL_PLAY_STOP == 1)
				|| ( g_u8AppCtrl&APPCTRL_RECORD_END == 1)
				||(g_u8AppCtrl == APPCTRL_NO_ACTION))
		{
#ifdef HDC1000_ENABLE
			ConvertHTValue();
#ifdef OLED_ENABLE
			memset(tmpString,0,sizeof(tmpString));
			sprintf(tmpString,"T:%d C   H:%d%%  ",HUMITURESENSOR_Data[1]>>8,HUMITURESENSOR_Data[0]>>8);
			print_Line(2, tmpString);			
#endif
#endif	
#ifdef MPU6050_ENABLE
			ConvertMPU6050Value();
#ifdef OLED_ENABLE
		//	memset(tmpString,0,sizeof(tmpString));
		//	sprintf(tmpString,"R%xP%xY%x",R,P,Y);
		//	print_Line(0, tmpString);
			memset(tmpString,0,sizeof(tmpString));
			sprintf(tmpString,"X%x Y%x Z%x",iGyroX,iGyroY,iGyroZ);
			print_Line(3, tmpString);
#endif
#endif
#ifdef UART_TRANS_ENABLE
			if(real_rx_count)
			{
				real_rx_count = 0;
				if (strstr((const char*)GLOBAL_BUF,"DAT"))
				{
					memset(GLOBAL_BUF,0,sizeof(GLOBAL_BUF));
			
					sprintf(GLOBAL_BUF,"L0+\nT:%d C  H:%d%% \nR: %x P: %x Y: %x \nGyroX: %x\nGyroY: %x\nGyroZ: %x\n",
					HUMITURESENSOR_Data[1]>>8,HUMITURESENSOR_Data[0]>>8,R,P,Y,iGyroX,iGyroY,iGyroZ);
					UART_Write_Wecan(UART0, GLOBAL_BUF, strlen(GLOBAL_BUF));
				}	
				else
				{
					sprintf(tmpString,"L0+\nInvalid Cmd\n");	
					UART_Write_Wecan(UART0,tmpString, strlen(tmpString));
				}
				memset(GLOBAL_BUF,0x0,sizeof(GLOBAL_BUF));
			}
#endif
		}
	
		if ( g_u8AppCtrl&APPCTRL_PLAY )
		{
			if ( App_ProcessPlay() == FALSE )
			{
				App_StopPlay();
				#ifdef OLED_ENABLE
				print_Line(1, "   PLAY  Stop  ");	
				#endif	
			}
		}

		TRIGGER_KEY_CHECK();		// Check and execute direct trigger key actions defined in "InputKeyActions.c"
									// Default trigger key handler is "Default_KeyHandler()"
									// The trigger key configurations are defined in "ConfigIO.h".
		
//		MATRIX_KEY_CHECK();			// Check and execute matrix key actions defined in "InputKeyActions.c"
									// Default matrix key handler is "Default_KeyHandler()"
									// The matrix key configurations are defined in "ConfigIO.h".

//		TOUCH_KEY_CHECK();			// Check and execute touch key actions defined in "InputKeyActions.c"
									// Default touch key handler is "Default_KeyHandler()"
									// The touch key configurations are defined in "ConfigIO.h".
	}
}

