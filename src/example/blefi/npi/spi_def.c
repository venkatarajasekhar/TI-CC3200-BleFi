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
 *	@file spi.c
 *
 *  ***************************************************************************/

// Include Library Header Files
#include <string.h>
#include <stdlib.h>

#include "spi_def.h"
#include "ble_events.h"
#include "npi_if.h"


// Global Variables **********************************************************//
/**
 * @brief This variable to used to store the SPI receiving data from CC26XX
 *
 * Maximal length of receiving data is 128 UInt8s.
 */
unsigned char g_ucSpiRxBuffer[CC2650_SPI_RX_BUFFER_SIZE];
unsigned char g_ucSpiTxBuffer[CC2650_SPI_TX_BUFFER_SIZE];

unsigned char*   	gSPIpacket;
unsigned char    	gSPIpacketSize;

extern unsigned char   gOutGoingPacket;
extern unsigned char   gCC2650Packet;
extern unsigned char   gAckEvent;

unsigned char    	gPacketType;
unsigned char    	gPacketLen;
unsigned short int   	gOpCode;
unsigned char    	gDataLength;

typedef void* NpiEvent_TaskHandle;

extern NpiEvtCB_t pNpiEvtCb;

#if NPI_RTOS

#define SPITRANSMIT 0
#define SPIRECEIVE  1


extern NPI_SyncObj gNPI_SyncObj;

unsigned char      gSpiTransfer = TRUE;

extern unsigned char      gNpiReady;

extern void HandleError(unsigned char error);
/*********************************************************************
 * @fn      NpiEvent_taskFxn
 *
 * @brief   Application task entry point for the NPI Event.
 *
 * @param   none
 *
 * @return  events not processed
 */
static void NPI_SpiEventTaskFxn(void)
{
   signed long npi_SyncObjStatus;
   unsigned char* pIncomingPacket;

   // Event Message loop
   for (;;)
   {
	    gNpiReady = TRUE;

		npi_SyncObjStatus = NPI_SyncObjWait(&gNPI_SyncObj, NPI_WAITFOREVER);
		LOOP_ON_ERROR(npi_SyncObjStatus);

		if(gSpiTransfer == SPIRECEIVE)
		{
				SPI_readPacket(&gPacketType, 1);      // Check Header.

				//Read Again.
				SPI_readPacket(&gPacketType, 1);      // Check Header.

				if(gPacketType == 0xFE)               // Header matches
				{
					SPI_readPacket(&gPacketLen, 1);   // Read Packet Len.

					//Clear Recieve Buffer
					memset(g_ucSpiRxBuffer, 0, CC2650_SPI_RX_BUFFER_SIZE);

					if((gPacketLen+1) > CC2650_SPI_RX_BUFFER_SIZE)
					{
						HandleError(1); // fatal error
						return;
					}

					SPI_readPacket(&g_ucSpiRxBuffer[0], gPacketLen+1);  //Including FCS byte

					//
					// Default state of MREADY - High
					//
					SignalMasterReadyHigh();

					if(gAckEvent ==  FALSE)
					{
						// post the semaphore
						//Event handling
						pIncomingPacket = malloc(gPacketLen);
						memcpy(pIncomingPacket, g_ucSpiRxBuffer, gPacketLen);

						gCC2650Packet = TRUE;

						pNpiEvtCb(pIncomingPacket);
					}
				else
				{
					//Ack Handling, the global variable is updated in the receive function
					gAckEvent = FALSE;
					gCC2650Packet = TRUE;
				}
			}
		}
		else if(gSpiTransfer == SPITRANSMIT)
		{
			// Send the message
			SPI_writePacket(gSPIpacket, gSPIpacketSize);

			// Default state of MREADY - High
			SignalMasterReadyHigh();

			gAckEvent = TRUE;
			gOutGoingPacket = FALSE;
		}
   }// main task loop

}




/*********************************************************************
 * @fn      NPIEvent_createTask
 *
 * @brief   Task creation function for the NPI Event.
 *
 * @param   none
 *
 * @return  none
 */
void NPI_SpiEventCreateTask(void)
{

	int retvalue_SyncObj;

	NPI_SpiInit();
	NPI_RegisterGpioInterrupt(NPI_SLAVESIGNAL, FALLING_EDGE, SReadyIntHandler);

	retvalue_SyncObj = NPI_CreateSyncObj(&gNPI_SyncObj);
	LOOP_ON_ERROR(retvalue_SyncObj);

	NPI_TaskCreate(NPI_SpiEventTaskFxn, (const signed char *)"NPI Event Task",
			       NPIEVENT_STACK_SIZE,NULL, NPIEVENT_TASK_PRIORITY, NULL);

}



//*****************************************************************************
//
//! Interrupt Handler for S Ready Interrupt
//!
//! \param None
//!
//! \return None
//
//*****************************************************************************
void SReadyIntHandler(void)
{
	NPI_DisableGpioInt(NPI_SLAVESIGNAL);

	if( gOutGoingPacket == FALSE)           // Read Requested by Slave
	{
		gSpiTransfer = SPIRECEIVE;
	}
	else
	{
		gSpiTransfer = SPITRANSMIT;
	}
	// post the semaphore
	NPI_SyncObjSignal(&gNPI_SyncObj);

	NPI_EnableGPIOInt(NPI_SLAVESIGNAL);
}

#else

//*****************************************************************************
//
//! Interrupt Handler for S Ready Interrupt
//!
//! \param None
//!
//! \return None
//
//*****************************************************************************
void SReadyIntHandler(void)
{
	NPI_DisableGpioInt(NPI_SLAVESIGNAL);

	if( gOutGoingPacket == FALSE)           // Read Requested by Slave
	{
		SPI_readPacket(&gPacketType, 1);   // Check Header.

		//Read Again.
		SPI_readPacket(&gPacketType, 1);   // Check Header.

		if(gPacketType == 0xFE)              // Header matches
		{
			SPI_readPacket(&gPacketLen, 1);   // Read Packet Len.


			//Clear Recieve Buffer
			memset(g_ucSpiRxBuffer, 0, CC2650_SPI_RX_BUFFER_SIZE);

			SPI_readPacket(&g_ucSpiRxBuffer[0], gPacketLen+1);  //Including FCS byte


			//
			// Default state of MREADY - High
			//
			SignalMasterReadyHigh();

			if(gAckEvent ==  FALSE)
			{
				gCC2650Packet = TRUE;
				pNpiEvtCb(g_ucSpiRxBuffer);
			}
			else
			{
				gCC2650Packet = TRUE;
			}
		}
	}
	else
	{
		// Send the message
		SPI_writePacket(gSPIpacket, gSPIpacketSize);

		//
		// Default state of MREADY - High
		//
		SignalMasterReadyHigh();

		gAckEvent = TRUE;
		gOutGoingPacket = FALSE;
	}

	NPI_EnableGPIOInt(NPI_SLAVESIGNAL);

}
#endif

void SignalMasterReadyLow(void)
{
	NPI_GpioPinWrite(NPI_MASTERSIGNAL, 0x0);
}

void SignalMasterReadyHigh(void)
{
	NPI_GpioPinWrite(NPI_MASTERSIGNAL, 0xFF);
}

/**
 * @brief <b>Fucntion Name </b> SPI_writePacket
 * @detail Description: Sends a packet (not including the SOF, Checksum,
 * len parameters) to the CC2650. It can receive packets meanwhile sending the
 * packet_buf_ptr to the CC2650 and store the result inside g_ucSpiRxBuffer[],
 * and the total number of packets received is stored in g_cc2541s_packets_received.
 * @param Input value:
 * <BR> unsigned char* packet_buf_ptr is the address of the first unsigned char of the packet to
 * be sent to the CC2650.
 * <BR> GAP/GATT Command Packet Format:
 * <BR> packet_buf_ptr[0] = CMD;
 * <BR> packet_buf_ptr[1:2] = Op Code;
 * <BR>	packet_buf_ptr[3] = Parameter total length;
 * <BR>	packet_buf_ptr[4 ... Parameter total length + 4] = Parameters of payload
 *
 * <BR> unsigned char len is the length of the packet to be sent to the CC2650 (not
 * including the SOF, length, or checksum parameters).
 * @return Return value: None
 */
void SPI_writePacket(unsigned char* packet_buf_ptr, unsigned char len)
{
	gCC2650Packet = FALSE;
	gAckEvent = FALSE;
	NPI_SpiWritePacket(packet_buf_ptr, len);
}

/**
 * @brief <b>Fucntion Name </b> SPI_readPacket
 * @detail Description: Receives packets from the CC26XX
 *
 * @return Return value: None
 */
void SPI_readPacket(unsigned char* pBuffer, unsigned char len)
{
	NPI_SpiReadPacket(pBuffer, len);
}

