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


/** ****************************************************************************
 *	@file uart_def.c
 *
 *  ***************************************************************************/

// Include Library Header Files
#include <string.h>
#include <stdlib.h>

#include "uart_def.h"
#include "ble_events.h"
#include "npi_if.h"
#include "osi.h"


// Global Variables **********************************************************//
/**
 * @brief This variable to used to store the UART receiving data from CC26XX
 *
 * Maximal length of receiving data is 128 UInt8s.
 */
unsigned char g_ucUartRxBuffer[CC2650_UART_RX_BUFFER_SIZE];
unsigned char g_ucUartAckRxBuffer[CC2650_UART_RX_BUFFER_SIZE];
unsigned int g_uiUartRxBufTail = 0;
extern unsigned int g_uiUartRxBufHead;

extern unsigned char   gOutGoingPacket;
extern unsigned char   gCC2650Packet;
extern unsigned char   gAckEvent;

unsigned char    	g_ucPacketType;
unsigned char       g_ucPacketEvent;
unsigned char    	g_ucPacketLen;
extern NpiEvtCB_t pNpiEvtCb;
extern unsigned char      gNpiReady;

extern unsigned char SBL_IsDone(void);
extern void HandleError(unsigned char *, unsigned int, unsigned char error);

#ifdef NPI_RTOS

extern NPI_SyncObj gNPI_SyncObj;

static int8 NPI_UartCheckNewPacket(void)
{
	int8 ret_val = -1;

	g_ucPacketType  = g_ucUartRxBuffer[g_uiUartRxBufTail];    	// Header

	g_ucPacketEvent = g_ucUartRxBuffer[((g_uiUartRxBufTail+1)%CC2650_UART_RX_BUFFER_SIZE)];       // Event Code

	g_ucPacketLen   = g_ucUartRxBuffer[((g_uiUartRxBufTail+2)%CC2650_UART_RX_BUFFER_SIZE)] + UART_EVENT_HDR_LEN; // Header + Payload Length

	if((g_ucPacketType == 0x04) && (g_ucPacketEvent == 0xFF))   //Check if Event Packet(0x04) and Event Code(0xFF)
	{
		if(g_uiUartRxBufHead >= (g_uiUartRxBufTail + g_ucPacketLen))
		{
			ret_val = 0;  // True
		}
	}

	return ret_val;

}


/*********************************************************************
 * @fn      NPI_UartEventTaskFxn
 *
 * @brief   Application task entry point for the NPI Event.
 *
 * @param   none
 *
 * @return  events not processed
 */
void NPI_UartEventTaskFxn(void)
{
	signed int  npi_SyncObjStatus;
	unsigned char* pIncomingPacket;
	volatile unsigned char LoopVar = 0xFF;

	if(SBL_IsDone() !=TRUE)
	{
		UART_PRINT("\n\r[NPI] Waiting for SBL to be complete ... ");
	}

	while(LoopVar)
	{
		if(SBL_IsDone()==TRUE)
		{
			break;
		}
		osi_Sleep(250);
	}

	NPI_UartInit();

   // Event Message loop
   while(LoopVar)
   {

	   gNpiReady = TRUE;

		npi_SyncObjStatus = (signed int )NPI_SyncObjWait(&gNPI_SyncObj, NPI_WAITFOREVER);
		LOOP_ON_ERROR(npi_SyncObjStatus);

		//while(g_uiUartRxBufTail < CC2650_UART_RX_BUFFER_SIZE)
		while(g_uiUartRxBufTail != g_uiUartRxBufHead)
		{
//			Report("[%d-%d]",g_uiUartRxBufHead,g_uiUartRxBufTail);
			g_ucPacketType  = g_ucUartRxBuffer[g_uiUartRxBufTail];    	// Header
			g_ucPacketEvent = g_ucUartRxBuffer[((g_uiUartRxBufTail+1)%CC2650_UART_RX_BUFFER_SIZE)];       // Event Code
			g_ucPacketLen   = g_ucUartRxBuffer[((g_uiUartRxBufTail+2)%CC2650_UART_RX_BUFFER_SIZE)] + UART_EVENT_HDR_LEN; // Header + Payload Length

			if((g_ucPacketType == 0x04) && (g_ucPacketEvent == 0xFF))   //Check if Event Packet(0x04) and Event Code(0xFF)
			{
				if(gAckEvent ==  FALSE)
				{
					//Event handling
					pIncomingPacket = malloc(g_ucPacketLen);
					if(pIncomingPacket == NULL)
					{
						UART_PRINT("\n\r[NPI] Error - not enough memory for pIncomingPacket ...");
						HandleError(__FILE__,__LINE__,1);
						return ;
					}
					if( (g_uiUartRxBufTail + g_ucPacketLen) > CC2650_UART_RX_BUFFER_SIZE)
					{
						//Find the excess bytes that needs to be written from the top
						unsigned char iPacketLenDiff = g_ucPacketLen - (CC2650_UART_RX_BUFFER_SIZE - g_uiUartRxBufTail);

						/*
						 *
						 * Copy permitted number of bytes till the end of buffer
						 * prev_tail to 2047
						 *
						 */
						memcpy(pIncomingPacket, &(g_ucUartRxBuffer[g_uiUartRxBufTail]), (CC2650_UART_RX_BUFFER_SIZE - g_uiUartRxBufTail));

						/*
						 *
						 * Copy the remaining number of bytes from beginning
						 * 0 to iPacketLenDiff
						 *
						 */
						memcpy((pIncomingPacket+(CC2650_UART_RX_BUFFER_SIZE - g_uiUartRxBufTail)), &(g_ucUartRxBuffer[0]), iPacketLenDiff);
						g_uiUartRxBufTail = iPacketLenDiff;
					}
					else
					{
						memcpy(pIncomingPacket, &(g_ucUartRxBuffer[g_uiUartRxBufTail]), g_ucPacketLen);
						g_uiUartRxBufTail += g_ucPacketLen;
						if(g_uiUartRxBufTail==CC2650_UART_RX_BUFFER_SIZE)
						{
							g_uiUartRxBufTail = 0;
						}
					}

					gCC2650Packet = TRUE;

					pNpiEvtCb(pIncomingPacket);
				}
				else
				{

					//Ack Handling, the global variable is updated in the receive function
					gAckEvent     = FALSE;
					gCC2650Packet = TRUE;

					if( (g_uiUartRxBufTail + g_ucPacketLen) > CC2650_UART_RX_BUFFER_SIZE)
					{
						unsigned char iPacketLenDiff = g_ucPacketLen - (CC2650_UART_RX_BUFFER_SIZE - g_uiUartRxBufTail);
						memcpy(g_ucUartAckRxBuffer, &(g_ucUartRxBuffer[g_uiUartRxBufTail]), (CC2650_UART_RX_BUFFER_SIZE - g_uiUartRxBufTail));
						memcpy((g_ucUartAckRxBuffer+(CC2650_UART_RX_BUFFER_SIZE - g_uiUartRxBufTail)), &(g_ucUartRxBuffer[0]), iPacketLenDiff);
						g_uiUartRxBufTail = iPacketLenDiff;
					}
					else
					{
						memcpy(g_ucUartAckRxBuffer, &(g_ucUartRxBuffer[g_uiUartRxBufTail]), g_ucPacketLen);
						g_uiUartRxBufTail += g_ucPacketLen;
						if(g_uiUartRxBufTail==CC2650_UART_RX_BUFFER_SIZE)
						{
							g_uiUartRxBufTail = 0;
						}
					}
				}

			}

			if(NPI_UartCheckNewPacket() != 0)
			{
				break;
			}

		}

   }// main task loop

}

/*********************************************************************
 * @fn      UARTEvent_createTask
 *
 * @brief   Task creation function for the NPI Event(UART).
 *
 * @param   none
 *
 * @return  none
 */
void NPI_UartEventCreateTask(void)
{

	int retvalue_SyncObj;

	retvalue_SyncObj = NPI_CreateSyncObj(&gNPI_SyncObj);
	LOOP_ON_ERROR(retvalue_SyncObj);


	NPI_TaskCreate(NPI_UartEventTaskFxn, (const signed char *)"NPI Event Task",
			       NPIEVENT_STACK_SIZE,NULL, NPIEVENT_TASK_PRIORITY, NULL);

}

#endif   //ifdef NPI_RTOS



void UartWritePacket(unsigned char *pPacket, char len)
{
	NPI_UartWritePacket(pPacket, len);
	gAckEvent = TRUE;
	gOutGoingPacket = FALSE;
}
