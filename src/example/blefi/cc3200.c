//*****************************************************************************
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

//*****************************************************************************
// cc3200.c
//
// Contains common functions related to CC3200 wireless MCU
//

//*****************************************************************************
//
//! \addtogroup get_weather
//! @{
//
//*****************************************************************************
#include <hw_types.h>
#include <uart.h>
#include <datatypes.h>
#include <simplelink.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <prcm.h>
#include <rom.h>
#include <rom_map.h>
#include <pin.h>
#include <utils.h>
#include <hw_memmap.h>
#include <hw_common_reg.h>
#include <gpio.h>
#include <protocol.h>
#include "cc3200.h"
#include "interrupt.h"
#include "hw_ints.h"
#include "gpio_if.h"
#include "uart_logger.h"
#include "stdbool.h"

//*****************************************************************************
// Variable related to Connection status
//*****************************************************************************
volatile char g_cc3101state = CC3101_UNINIT;
extern bool is_blefiCfgReady;

//*****************************************************************************
// Defining Timer Load Value. Corresponds to 1 second.
//*****************************************************************************
#define PERIODIC_TEST_CYCLES    80000000

#define UART_PRINT              Report


//*****************************************************************************
// Variables to store TIMER Port,Pin values
//*****************************************************************************
extern unsigned int g_uiLED1Port;
extern unsigned char g_ucLED1Pin;
OsiSyncObj_t g_GWGenSyncObj;

#define OSI_STACK_SIZE	2048

typedef enum
{
  GW_CALL_NONBLOCKING = 0,
  GW_CALL_BLOCKING
} GW_CALL;

typedef enum
{
  INT_BUTTON = 0,
  INT_TIMER
} INT_TYPE;

extern void GwButtonHandler();
extern int GC_Scan(GW_CALL calltype);
extern void Do_SBL_Dnld(void);
extern void CC26xx_Init(void);
extern void UpdateBlefiCfg(void);
extern unsigned char CC26xx_SBL_CheckIfUpdateFwNeeded(void);

/*
 * FUNCTION DEFINITIONS
 */

void Button2IntClear(void);
void Button2IntDisable(void);
void Button2IntEnable(void);
signed char  FlashWriteFile(unsigned char* pUserFileName, unsigned char * pBuffer , unsigned int BufLen);

static volatile unsigned long g_ulBase;
INT_TYPE g_ucIntFlag;

//*****************************************************************************
//
//!
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
signed char  FlashWriteFile(unsigned char* pUserFileName, unsigned char * pBuffer , unsigned int BufLen)
{
    int lRetVal;
    long lFileHandle;
    unsigned long ulToken;
    SlFsFileInfo_t  	pFsFileInfo;

	// IF BOND file is not present in flash create it
    lRetVal = sl_FsGetInfo((unsigned char *)pUserFileName,0,&pFsFileInfo);
   if(lRetVal < 0)
    {
    	 UART_PRINT("\n\rFile not present, Creating file  - %s", pUserFileName);

    	//
    	// create a user file
    	//

    	lRetVal = sl_FsOpen((unsigned char *)pUserFileName,
    						FS_MODE_OPEN_CREATE(BufLen, \
    		                           _FS_FILE_OPEN_FLAG_COMMIT|_FS_FILE_PUBLIC_WRITE),
    						 &ulToken,
    						 &lFileHandle);

		lRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
		 if(lRetVal < 0)
		 {
			 UART_PRINT("\n\rERROR - Could not open the file - %s", pUserFileName);
			 //while(1);
			 return(lRetVal);
		 }
    }

    // Open the file in  write mode for updating the file

	//
	// create a user file
	//
	lRetVal = sl_FsOpen((unsigned char *)pUserFileName,
						FS_MODE_OPEN_WRITE,
						 &ulToken,
						 &lFileHandle);

	 if(lRetVal < 0)
	 {
		 lRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
		 UART_PRINT("\n\rERROR - Could not open the file for writing - %s", pUserFileName);
		 return(lRetVal);
	 }

	// Update the BOND structure to the file in Flash
	lRetVal = sl_FsWrite(lFileHandle,0x0,(uint8*)pBuffer,BufLen);
	if(lRetVal < 0)
	{
		 lRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
		 UART_PRINT("\n\rERROR - Could not open the file - %s", pUserFileName);
		 return(lRetVal);
	}

    lRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
	if (SL_RET_CODE_OK != lRetVal)
	{
		//ASSERT_ON_ERROR(FILE_CLOSE_ERROR);
		//while(1);
		return(lRetVal);
	}

	return 0;

}


//*****************************************************************************
//
//!
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
signed char  FlashReadFile(unsigned char* pUserFileName, unsigned char * pBuffer , unsigned int BufLen)
{
    int lRetVal;
    long lFileHandle;
    unsigned long ulToken;
    SlFsFileInfo_t  	pFsFileInfo;

	// IF BOND file is not present in flash create it
    lRetVal = sl_FsGetInfo((unsigned char *)pUserFileName,0,&pFsFileInfo);
   if(lRetVal < 0)
    {
    	 UART_PRINT("\n\r Error Opening the file  %s", pUserFileName);
    	ASSERT_ON_ERROR(lRetVal);
    }

	//
	// create a user file
	//
	lRetVal = sl_FsOpen((unsigned char *)pUserFileName,
						FS_MODE_OPEN_READ,
						 &ulToken,
						 &lFileHandle);

	 if(lRetVal < 0)
	 {
		 lRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
		 UART_PRINT("\n\rERROR - Could not open the file %s", pUserFileName);
		 ASSERT_ON_ERROR(lRetVal);
	 }


	// Update the BOND structure to the file in Flash
    //lRetVal = sl_FsWrite(lFileHandle,0x0,(uint8*)(&bonds[0]),sizeof(BondRec_t));
    lRetVal = sl_FsRead(lFileHandle,0x0,(uint8*)pBuffer,BufLen);
	 if(lRetVal < 0)
	 {
		 lRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
		 UART_PRINT("\n\rERROR - Could not open the file for reading  %s", pUserFileName);
		 ASSERT_ON_ERROR(lRetVal);
	 }

    lRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
	if (SL_RET_CODE_OK != lRetVal)
	{
		//ASSERT_ON_ERROR(FILE_CLOSE_ERROR);
		ASSERT_ON_ERROR(lRetVal);
	}

	return 0;

}


/***************************************************************************
 *
 *
 *
 *
 ***************************************************************************/
signed char  FlashIsFilePresent(unsigned char* pUserFileName)
{
    int lRetVal;
    SlFsFileInfo_t  	pFsFileInfo;

	// IF BOND file is not present in flash create it
    lRetVal = sl_FsGetInfo((unsigned char *)pUserFileName,0,&pFsFileInfo);
   if(lRetVal < 0)
    {
	   UART_PRINT("\n\r[GW] Device bonding information file not present");
		return(-1);
    }
   else
   {
		//Message("\n\r File Present");
		UART_PRINT("\n\r[GW] Device bonding information file <%s> Present",pUserFileName);
		return(0);
   }
}




/************************************
 *
 * *  LED  and BUTTON FUNCTIONS  *
 *
 ************************************/

//****************************************************************************
//
//!	\brief    TurnOnLed1
//!
//! \return	                	None
//
//****************************************************************************
void TurnOnLed(LedNum LedInstance)
{

	switch(LedInstance)
	{
		case LED_1:
		{
			#ifdef BLEFI_BOARD_V1
				// For BLEFIBOARD1
				GPIOPinWrite(GPIOA1_BASE, 0x10, 0x10);
			#else
				// For BLEFIBOARD2
				GPIOPinWrite(GPIOA1_BASE, 0x40, 0x40);
			#endif
			break;
		}
		case LED_2:
		{
			#ifdef BLEFI_BOARD_V1
				// For BLEFIBOARD1
				// No LED2 connected to CC3200
			#else
				// For BLEFIBOARD2
				GPIOPinWrite(GPIOA3_BASE, 0x40, 0x40);
			#endif
			break;
		}
	}
}


//****************************************************************************
//
//!	\brief    TurnOffLed1
//!
//! \return	                	None
//
//****************************************************************************
void TurnOffLed(unsigned char LedInstance)
{

	switch(LedInstance)
	{
		case LED_1:
		{
			#ifdef BLEFI_BOARD_V1
				// For BLEFIBOARD1
				GPIOPinWrite(GPIOA1_BASE, 0x10, 0x0);
			#else
				// For BLEFIBOARD2
				GPIOPinWrite(GPIOA1_BASE, 0x40, 0x0);
			#endif
			break;
		}
		case LED_2:
		{
			#ifdef BLEFI_BOARD_V1
				// For BLEFIBOARD1
				// No LED2 connected to CC3200
			#else
				// For BLEFIBOARD2
				GPIOPinWrite(GPIOA3_BASE, 0x40, 0x0);
			#endif
			break;
		}
	}
}



//****************************************************************************
//
//!	\brief  BlinkLed
//!
//! \return	                	None
//
//****************************************************************************
void BlinkLed(LedNum LedInstance,unsigned short iBlinkCount, LedBlinkrate iBlinkRate)
{
	char i =0;
	unsigned short DelayCount = 0;
	//volatile int temp =1000000 , j=0;

	switch(iBlinkRate)
	{
		case SLOW:
		{
			DelayCount = 1000;
			break;
		}
		case MEDIUM:
		{
			DelayCount = 500;
			break;
		}
		case FAST1:
		{
			DelayCount = 200;
			break;
		}
		case FAST2:
		{
			DelayCount = 50;
			break;
		}
	}


    //Blink LED iCount times
    for(i=0; i<iBlinkCount; i++)
    {
		//Turn Wifi LED On
		// GPIO_IF_LedOn(MCU_IP_ALLOC_IND);
		//GPIOPinWrite(GPIOA1_BASE, 0x10, 0x10);
    	TurnOnLed(LedInstance);
		osi_Sleep(DelayCount);

		//Turn Wifi LED Off
		//  GPIO_IF_LedOff(MCU_IP_ALLOC_IND);
		//GPIOPinWrite(GPIOA1_BASE,  0x10, 0x0);
		TurnOffLed(LedInstance);
		osi_Sleep(DelayCount);
    }
}


/*Initialize Gateway Button*/
void Button2Init()
{

	/*
	 * BUTTON2
	 * */

	//
	// Set Interrupt Type for GPIO
	//
	MAP_GPIOIntTypeSet(GPIOA2_BASE,GPIO_PIN_1,GPIO_FALLING_EDGE);

	//
	// Enable Interrupt
	//
	MAP_GPIOIntClear(GPIOA2_BASE,GPIO_PIN_1);
	MAP_GPIOIntEnable(GPIOA2_BASE,GPIO_INT_PIN_1);

	//
	// Register Interrupt handler
	//
	osi_InterruptRegister(INT_GPIOA2,(P_OSI_INTR_ENTRY)GwButtonHandler, \
							INT_PRIORITY_LVL_1);

}

void Button2IntClear()
{
	MAP_GPIOIntClear(GPIOA2_BASE,GPIO_PIN_1);
}

void Button2IntDisable()
{
	MAP_GPIOIntDisable(GPIOA2_BASE,GPIO_PIN_1);
}

void Button2IntEnable()
{
	MAP_GPIOIntEnable(GPIOA2_BASE,GPIO_PIN_1);
}


void HandleError(unsigned char * filename, unsigned int line_num, unsigned char error)
{
  volatile unsigned char loopvar = 0xFF;


  switch (error)
  {
      // non-fatal error
  	  case 0:
  	  {

  		  break;
  	  }

  	  //fatal error
  	  case 1:
  	  {
  		   while(loopvar)
  		   {
  	 		   UART_PRINT("\n\r[GW] Fatal error %s, %d- Reset system\n\n",filename,line_num);

  	 		    //osi_Sleep(5000);

				BlinkLed(LED_1,10, FAST2);
				BlinkLed(LED_2,10, FAST2);
  		   }
  		  break;
  	  }
  	  case 2:
  		   UART_PRINT("\n\r[GW] Error %s, %d\n\n",filename,line_num);
  		 break;
  	  default:
  		  break;
  }
}


#define SBL_BAUD 115200
void sblUARTinit()
{
// Initialising the UART0.
//
UARTConfigSetExpClk(UARTA0_BASE, PRCMPeripheralClockGet(PRCM_UARTA0),
						SBL_BAUD,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                        UART_CONFIG_PAR_NONE));
}

void CC26xx_SBL_Dnld_Init(void)
{
	volatile unsigned char LoopVar = 0xFF;
	while (LoopVar)
	{
		if (is_blefiCfgReady == true)
		{
			break;
		}
		osi_Sleep(250);
	}
	if (CC26xx_SBL_CheckIfUpdateFwNeeded() == 0)
	{
		Message("\n\r[SBL] Firmware Up to date");
		return;
	}
	else
	{
		Message("\n\r[SBL] Firmware needs to be updated");
	}

	sblUARTinit();
	//GPIO_7 -> SBL Select
	//GPIO_22 -> Reset
	GPIOPinWrite(GPIOA2_BASE, GPIO_PIN_6, GPIO_PIN_6); //GPIO_22 -> CC26xx Reset
	GPIOPinWrite(GPIOA0_BASE, GPIO_PIN_7, GPIO_PIN_7); //GPIO_7
	UtilsDelay(10 * DELAY_1_MS);
 	GPIOPinWrite(GPIOA2_BASE, GPIO_PIN_6, 0); //GPIO_22 -> CC26xx Reset
 	UtilsDelay(10 * DELAY_1_MS);
 	//GPIOPinWrite(GPIOA0_BASE, GPIO_PIN_7, GPIO_PIN_7); //GPIO_7
 	GPIOPinWrite(GPIOA0_BASE, GPIO_PIN_7, 0); //GPIO_7  -> For SBL Select
 	UtilsDelay(10 * DELAY_1_MS);
 	GPIOPinWrite(GPIOA2_BASE, GPIO_PIN_6, 0xFF);

 	UtilsDelay(2 * DELAY_1_SEC); //Need to find the correct value.
 	//GPIOPinWrite(GPIOA2_BASE, GPIO_PIN_1, GPIO_PIN_1); //GPIO_17

 	Do_SBL_Dnld();
 	GPIOPinWrite(GPIOA0_BASE, GPIO_PIN_7, GPIO_PIN_7); //GPIO_7
 	UtilsDelay(10 * DELAY_1_MS);
 	CC26xx_Init();
 	UpdateBlefiCfg();
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
