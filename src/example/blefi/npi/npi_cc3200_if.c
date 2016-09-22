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


/*
 * npi_if.c
 *
 *  Created on: Jul 18, 2014
 *      Author: a0393958
 */


#include "npi_if.h"

#include "hw_types.h"
#include "hw_ints.h"
#include "hw_gpio.h"
#include "hw_memmap.h"
#include "pin.h"
#include "prcm.h"
#include "gpio.h"
#include "interrupt.h"
#include "spi.h"
#include "uart.h"
#include "uart_def.h"

#define SPI_IF_BIT_RATE  2000000
#ifdef BLEFI_BOARD_V1
#define UARTA1_BAUD_RATE 115200
#else
#define UARTA0_BAUD_RATE 115200
#endif

#define UART_READ_EVENT_HDR     0
#define UART_READ_EVENT_PYLD    1
#define UART_PACKET_TYPE        0
#define UART_PACKET_EVENT       1
#define UART_PACKET_LEN         2


void (*gIntHdlr)(void);

unsigned char      gNpiReady = FALSE;


#ifdef NPI_RTOS

#include "osi.h"

#define OSI_INTERRUPT_PRI  0x20

NPI_SyncObj gNPI_SyncObj;
extern unsigned char g_ucStartUARTSend;
extern unsigned char g_ucUartRxBuffer[CC2650_UART_RX_BUFFER_SIZE];
unsigned int  g_uiUartRxBufHead = 0;
unsigned char g_ucUartReadLen = 3;
unsigned char g_ucUartByteCount = 0;
unsigned char g_ucUartReadState = UART_READ_EVENT_HDR;
unsigned char g_ucUartPacketState = UART_PACKET_TYPE;

void NPI_TaskCreate(void *NpiEvent_taskFxn, const signed char* taskName, unsigned short usStackDepth,void *pvParameters,unsigned long uxPriority, NpiTaskHandle *pNpiEvent_TaskHandle)
{
	osi_TaskCreate((P_OSI_TASK_ENTRY)NpiEvent_taskFxn, (const signed char*) taskName, usStackDepth, pvParameters, uxPriority, (OsiTaskHandle*)pNpiEvent_TaskHandle);
}

int NPI_CreateSyncObj(NPI_SyncObj* pSyncObj)
{
	return osi_SyncObjCreate(pSyncObj);
}

int NPI_SyncObjWait(NPI_SyncObj* pSyncObj, int _timeout_)
{
	return osi_SyncObjWait(pSyncObj, _timeout_);
}

void NPI_SyncObjSignal(NPI_SyncObj* pSyncObj)
{
	osi_SyncObjSignal(pSyncObj);
}

void NPI_SyncObjSignalFromISR(NPI_SyncObj* pSyncObj)
{
	osi_SyncObjSignalFromISR(pSyncObj);
}

void NPI_Sleep(int duration)
{
	osi_Sleep(duration);
}

#endif /*NPI_RTOS*/



void NPI_SpiInit(void)
{

	  //
	  // Default state of MREADY - High
	  //
	  NPI_GpioPinWrite(NPI_MASTERSIGNAL, 0xFF);

	  //
	  // Enable the SPI module clock
	  //
	  PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);

	  //
	  // Reset the peripheral
	  //
	  PRCMPeripheralReset(PRCM_GSPI);

	  //
	  // Reset SPI
	  //
	  SPIReset(GSPI_BASE);

	  //
	  // Configure SPI interface
	  //
	  SPIConfigSetExpClk(GSPI_BASE, PRCMPeripheralClockGet(PRCM_GSPI),
	                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_3,
	                     (SPI_SW_CTRL_CS |
	                     SPI_4PIN_MODE |
	                     SPI_TURBO_OFF |
	                     SPI_CS_ACTIVELOW |
	                     SPI_WL_8));


	  //
	  // Enable SPI for communication
	  //
	  SPIEnable(GSPI_BASE);

}

static void gpioIntFuncHandler(void)
{
   unsigned long ulPinState = GPIOIntStatus(GPIOA0_BASE,1);

   if(ulPinState & GPIO_PIN_7)
   {
	   gIntHdlr();
   }
}

void NPI_RegisterGpioInterrupt(int GPIO_NUM, int edge, void* funcPtr)
{

	if(GPIO_NUM == NPI_SLAVESIGNAL)
	{
		gIntHdlr = (void(*)(void) )funcPtr;
		if(edge == FALLING_EDGE)
		{
			//
			// Set Interrupt Type for GPIO
			//
			GPIOIntTypeSet(GPIOA0_BASE, GPIO_PIN_7, GPIO_FALLING_EDGE);
		}

#ifdef NPI_RTOS
		osi_InterruptRegister(INT_GPIOA0, (P_OSI_INTR_ENTRY)gpioIntFuncHandler, OSI_INTERRUPT_PRI);
#else
		//
		// Register Interrupt handler
		//
		GPIOIntRegister(GPIOA2_BASE, gpioIntFuncHandler);
#endif
		//
		// Enable Interrupt
		//

		GPIOIntClear(GPIOA0_BASE, GPIO_PIN_7);
		GPIOIntEnable(GPIOA0_BASE, GPIO_PIN_7);
	}
}


void NPI_EnableGPIOInt(int GPIO_NUM)
{

	if(GPIO_NUM==NPI_SLAVESIGNAL)
	{
		IntEnable(INT_GPIOA0);
		GPIOIntEnable(GPIOA0_BASE, GPIO_PIN_7);
	}
}

void NPI_DisableGpioInt(int GPIO_NUM)
{
	if(GPIO_NUM==NPI_SLAVESIGNAL)
	{
		//Clear and Disable GPIO Interrupt
		GPIOIntDisable(GPIOA0_BASE, GPIO_PIN_7);
		GPIOIntClear(GPIOA0_BASE, GPIO_PIN_7);
		IntDisable(INT_GPIOA0);
	}
}



void NPI_GpioPinWrite(int gpio_num, int value)
{
	if( gpio_num == NPI_MASTERSIGNAL )
	{
		GPIOPinWrite(GPIOA1_BASE, GPIO_PIN_0, value);
	}
}

unsigned long NPI_GpioPinRead(int GPIO_NUM)
{
	if(GPIO_NUM == NPI_SLAVESIGNAL)
	{
		return (unsigned long) GPIOPinRead(GPIOA0_BASE, GPIO_PIN_7);
	}

	return 0xFFFF;
}


void NPI_SpiReadPacket(unsigned char *pPacket, char len)
{
	unsigned long nullPutPacket = 0x00;
	unsigned long nullGetPacket = 0x00;
	unsigned char i = 0;


	SPICSEnable(GSPI_BASE);							//Enable Chip Select
	for(i=0;i<len;i++)
	{
		SPIDataPut(GSPI_BASE, nullPutPacket);
		SPIDataGet(GSPI_BASE, &nullGetPacket);
		*(pPacket+i) = (unsigned char)nullGetPacket;
	}
	SPICSDisable(GSPI_BASE);                        //Disable Chip Select
}

void NPI_SpiWritePacket(unsigned char *pPacket, char len)
{
	int i = 0;
	unsigned long nullPutPacket = 0x00;
	unsigned long nullGetPacket = 0x00;

	SPICSEnable(GSPI_BASE);							//Enable Chip Select
	for(i=0;i<len;i++)
	{
		nullPutPacket = *(pPacket+i);
		SPIDataPut(GSPI_BASE, nullPutPacket);
		SPIDataGet(GSPI_BASE, &nullGetPacket);
	}
	SPICSDisable(GSPI_BASE);                        //Disable Chip Select
}


//*****************************************************************************
//
//! Interrupt Handler for Uart  finished Interrupt
//!
//! \param None
//!
//! \return None
//
//*****************************************************************************
static void UartIntHandler(void)
{

#ifdef BLEFI_BOARD_V1
	g_ucUartRxBuffer[g_uiUartRxBufHead++] = UARTCharGetNonBlocking(UARTA1_BASE);

	UARTIntClear(UARTA1_BASE, UART_INT_RT);
#else
	g_ucUartRxBuffer[g_uiUartRxBufHead] = UARTCharGetNonBlocking(UARTA0_BASE);
	UARTIntClear(UARTA0_BASE, UART_INT_RT);
#endif


	if(g_ucStartUARTSend == 1)
	{
		if(g_ucUartReadState == UART_READ_EVENT_HDR)            //Packet Header Read State
		{

			switch (g_ucUartPacketState)
			{
				case UART_PACKET_TYPE:
				{
					if(g_ucUartRxBuffer[(g_uiUartRxBufHead)] == 0x04)  //HCI Event Packet
					{
						g_ucUartPacketState = UART_PACKET_EVENT;
					}
				}
				break;

				case UART_PACKET_EVENT:
				{
					if(g_ucUartRxBuffer[(g_uiUartRxBufHead)] == 0xFF)   //Vendor Specific HCI Event code
					{
						g_ucUartPacketState = UART_PACKET_LEN;
					}
					else
					{
						g_ucUartPacketState = UART_PACKET_TYPE;
					}
				}
				break;

				case UART_PACKET_LEN:
				{
					g_ucUartReadLen = g_ucUartRxBuffer[(g_uiUartRxBufHead)];
					g_ucUartReadState  = UART_READ_EVENT_PYLD;
					g_ucUartPacketState = UART_PACKET_TYPE;
				}
				break;

				default:
				{
					g_ucUartPacketState = UART_PACKET_TYPE;
				}
				break;
			}
			g_uiUartRxBufHead++;
			if(g_uiUartRxBufHead >= CC2650_UART_RX_BUFFER_SIZE)
			{
				g_uiUartRxBufHead = 0;   //Reseting the Head
			}
		}
		else											// Packet Payload Read
		{
			g_ucUartByteCount++;
			g_uiUartRxBufHead++;
			if(g_uiUartRxBufHead >= CC2650_UART_RX_BUFFER_SIZE)
			{
				g_uiUartRxBufHead = 0;   //Reseting the Head
			}
			if(g_ucUartByteCount == g_ucUartReadLen)
			{

				//Payload & Header Copied. Signal the Uart Event Thread.
				g_ucUartByteCount = 0;

				// post the semaphore
				NPI_SyncObjSignal(&gNPI_SyncObj);
				g_ucUartReadState  = UART_READ_EVENT_HDR;

			}
		}
		return;
	}
	else
	{
		g_uiUartRxBufHead = 0;   //Reseting the head during Startup event.
		return;
	}

}


void NPI_UartInit(void)
{



	//
	// Initialising the UART0.
	//
	UARTConfigSetExpClk(UARTA0_BASE, PRCMPeripheralClockGet(PRCM_UARTA0),
	                        UARTA0_BAUD_RATE,
	                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
	                        UART_CONFIG_PAR_NONE));

	//
	// Configure the UART Tx and Rx FIFO level to 1/8 i.e 2 characters
	//
	UARTFIFOLevelSet(UARTA0_BASE,UART_FIFO_TX1_8,UART_FIFO_RX7_8);


#ifdef NPI_RTOS
		osi_InterruptRegister(INT_UARTA0, (P_OSI_INTR_ENTRY)UartIntHandler, OSI_INTERRUPT_PRI);
#else
		//
		// Register interrupt handler for UART0
		//
		UARTIntRegister(UARTA0_BASE, UartIntHandler);
#endif

	//
	// Clear interrupts for uart0
	//
	UARTIntClear(UARTA0_BASE,  UART_INT_RX);

	// Clear UARTA0 Recieve Timeout Interrupt.
	UARTIntClear(UARTA0_BASE,  UART_INT_RT);

	// Enable UARTA0 Recieve Timeout Interrupt.
	UARTIntEnable(UARTA0_BASE, UART_INT_RT);
	//
	// Enable interrupts for uart0
	//
	UARTIntEnable(UARTA0_BASE, UART_INT_RX);

	// Enable UARTA0
	UARTEnable(UARTA0_BASE);



}


void NPI_UartReadPacket(unsigned char *pPacket, char len)
{
	int i = 0;
	long nullGetPacket = 0x00;


	for(i=0;i<len;i++)
	{
#ifdef BLEFI_BOARD_V1
		nullGetPacket = UARTCharGet(UARTA1_BASE);
#else
		nullGetPacket = UARTCharGet(UARTA0_BASE);
#endif
		*(pPacket+i) = (unsigned char)nullGetPacket;
	}

}

void NPI_UartWritePacket(unsigned char *pPacket, char len)
{
	int i = 0;

	for(i=0;i<len;i++)
	{
#ifdef BLEFI_BOARD_V1
		UARTCharPut(UARTA1_BASE, *(pPacket+i));
#else
		UARTCharPut(UARTA0_BASE, *(pPacket+i));
#endif
	}

}


void NPI_UartDisableInt(void)
{
#ifdef BLEFI_BOARD_V1
	//
	// Disable interrupts for uart1
	//
	UARTIntDisable(UARTA1_BASE, UART_INT_RX);
#else
	//
	// Disable interrupts for uart1
	//
	UARTIntDisable(UARTA0_BASE, UART_INT_RX);
#endif
}

void NPI_UartEnableInt(void)
{
#ifdef BLEFI_BOARD_V1
	//
	// Enable interrupts for uart1
	//
	UARTIntEnable(UARTA1_BASE, UART_INT_RX);
#else
	//
	// Enable interrupts for uart1
	//
	UARTIntEnable(UARTA0_BASE, UART_INT_RX);
#endif
}

/*********************************************************************
 * @fn      NPI_IsReady
 *
 * @brief   Returns the Ready state of NPI layer.
 *
 * @param   none
 *
 * @return  none
 */
unsigned char NPI_IsReady()
{
    return(gNpiReady);
}
