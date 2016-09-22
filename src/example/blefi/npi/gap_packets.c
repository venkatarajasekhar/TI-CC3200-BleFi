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
 *	@file gap.c
 *
 * 	@detail	Description:
 *	Encode and Decode For GAP packets. For GAP Command, fill GAP related packet
 *	and then send the packet to CC2650 through SPI; For GAP Event, now just
 *	return the Status when receive the packet from CC2650 through SPI.
 *
 *
 *  ***************************************************************************/

// Include Library Header Files
#include "gap.h"
#include "hci.h"

#ifndef NPI_UART
#define NPI_UART 1
#endif

#ifdef NPI_SPI

#include "spi_def.h"

// Local/Global Variables
extern unsigned char*   gSPIpacket;
extern unsigned char    gSPIpacketSize;

#endif  //ifdef NPI_SPI

#ifdef NPI_UART

#include "uart_def.h"
unsigned char g_ucStartUARTSend = FALSE;

#endif  //ifdef NPI_UART

extern unsigned char  gOutGoingPacket;
#define GAP_PACKET_SIZE 45
unsigned char gGapPacket[GAP_PACKET_SIZE];

extern void HandleError(unsigned char error);

static void GAP_sendPacket(unsigned char* pGapPacket, unsigned char packetSize)
{

#ifdef NPI_SPI
	gSPIpacket  	= pGapPacket;
	gSPIpacketSize  = packetSize;

	gOutGoingPacket = TRUE;

	SignalMasterReadyLow();
#endif

#ifdef NPI_UART



	if(g_ucStartUARTSend == FALSE)
	{
		g_ucStartUARTSend = 1;  //TRUE from first UART Write Operation (GAP_DeviceInit)
	}
	UartWritePacket(pGapPacket, packetSize);

#endif

}

/**
 * @brief <b>Fucntion Name </b> GAP_DeviceInit
 * @brief <b>Description</b>:
 * Assume that PacketType(1 octet), OpCode(2), Parameter Length(1) is already stored
 * in the packet. Fill GAP Device Init Command packet and send the packet to CC2650
 * through SPI.
 * @param Input Parameters:
 * <BR> GapDeviceInit_t* para is the parameter we need to send to CC2650 through SPI.
 * @return Return Values: None
 */
void GAP_deviceInitPacketize(gapDeviceInit_t* para)
{
	unsigned char j,i = 0;
#ifdef NPI_SPI
	// Fill Header and Packet Length First. Checked by CC26XX
	gGapPacket[i++] = 0xFE;        //Packet Header
	gGapPacket[i++] = 0x2A;        //Packet Length
#endif
	//
	// Command Byte
	//
	gGapPacket[i++] = COMMAND;
	//
	// Opcode Bytes (2 Bytes)
	//
	gGapPacket[i++] = (unsigned char)GAP_DEVICEINIT;
	gGapPacket[i++] = (unsigned char)(GAP_DEVICEINIT >> 8);
	//
	// Length byte
	//
	gGapPacket[i++] = 0x26;

	// Profile Role (1 octet)
	gGapPacket[i++] = para->ProfileRole;

	// Max Scan Responses (1 octet)
	gGapPacket[i++] = para->MaxScanResponses;

	// IRK & CSRK () (each 16 octets)

	for (j = 0; j < 16; j++)
		gGapPacket[i++] = *(para->pIRK++);
	for (j = 0; j < 16; j++)
		gGapPacket[i++] = *(para->pSRK++);
	// Sign Counter (4 octets)
	for (j = 0; j < 4; j++)
		gGapPacket[i++] = *(para->pSignCounter++);

#ifdef NPI_SPI
	gGapPacket[i++] = 0x0C; //CRC Check.-Footer
#endif


	if(i > GAP_PACKET_SIZE)
	{
        // error - GapPacket buffer overflow
		HandleError(1);
		return; // will never reach here
	}

	GAP_sendPacket(gGapPacket, i);

}


void GAP_configDeviceAddrPacketize(gapConfigDevAddr_t *para)
{

	unsigned char j, i = 0;

#ifdef NPI_SPI
	// Fill Header and Packet Length First. Checked by CC26XX
	gGapPacket[i++] = 0xFE;        //Packet Header
	gGapPacket[i++] = 0x0B;        //Packet Length
#endif

	//
	// Command Byte
	//
	gGapPacket[i++] = COMMAND;
	//
	// Opcode Bytes (2 Bytes)
	//
	gGapPacket[i++] = (unsigned char)GAP_CONFIGUREDEVICEADDR;
	gGapPacket[i++] = (unsigned char)(GAP_CONFIGUREDEVICEADDR >> 8);

	// Length byte
	//
	gGapPacket[i++] = 0x07;

	// Addr Type (1 octet)
	gGapPacket[i++] = para->addrType;

	// InitiatorAddr
	for (j = 0; j < 6; j++)
		gGapPacket[i++] = para->devAddr[j];

#ifdef NPI_SPI
	gGapPacket[i++] = 0x0C; //CRC Check.-Footer
#endif

	if(i > GAP_PACKET_SIZE)
	{
        // error - GapPacket buffer overflow
		HandleError(1);
		return; // will never reach here
	}

	GAP_sendPacket(gGapPacket, i);

}

void GAP_deviceDiscoveryRequestPacketize(gapDevDiscReq_t *para)
{
	unsigned char i = 0;
#ifdef NPI_SPI
	// Fill Header and Packet Length First. Checked by CC26XX
	gGapPacket[i++] = 0xFE;        //Packet Header
	gGapPacket[i++] = 0x07;        //Packet Length
#endif

	//
	// Command Byte
	//
	gGapPacket[i++] = COMMAND;
	//
	// Opcode Bytes
	//
	gGapPacket[i++] = (unsigned char)GAP_DEVICEDISCOVERYREQUEST;
	gGapPacket[i++] = (unsigned char)(GAP_DEVICEDISCOVERYREQUEST >> 8);

	// Length byte
	//
	gGapPacket[i++] = 0x03;

	// Discovery mode (1 octet)
	gGapPacket[i++] = para->mode;

	// Avtive Scan ON/OFF (1 octet)
	gGapPacket[i++] = para->activeScan;

	// WhiteList (1 octet)
	gGapPacket[i++] = para->whiteList;

#ifdef NPI_SPI
	gGapPacket[i++] = 0x0C; //CRC Check.-Footer
#endif

	if(i > GAP_PACKET_SIZE)
	{
        // error - GapPacket buffer overflow
		HandleError(1);
		return; // will never reach here
	}

	GAP_sendPacket(gGapPacket, i);

}

void GAP_deviceDiscoveryCancelPacketize(void)
{
	unsigned char i = 0;
#ifdef NPI_SPI
	// Fill Header and Packet Length First. Checked by CC26XX
	gGapPacket[i++] = 0xFE;        //Packet Header
	gGapPacket[i++] = 0x04;        //Packet Length
#endif
	//
	// Command Byte
	//
	gGapPacket[i++] = COMMAND;
	//
	// Opcode Bytes
	//
	gGapPacket[i++] = (unsigned char)GAP_DEVICEDISCOVERYCANCEL;
	gGapPacket[i++] = (unsigned char)(GAP_DEVICEDISCOVERYCANCEL >> 8);

	// Length byte
	//
	gGapPacket[i++] = 0x00;

#ifdef NPI_SPI
	gGapPacket[i++] = 0x0C; //CRC Check.-Footer
#endif

	if(i > GAP_PACKET_SIZE)
	{
        // error - GapPacket buffer overflow
		HandleError(1);
		return; // will never reach here
	}

	GAP_sendPacket(gGapPacket, i);

}


/**
 * @brief <b>Fucntion Name </b> GAP_MakeDiscoverable
 * @brief <b>Description</b>:
 * Assume that PacketType(1 octet), OpCode(2), Parameter Length(1) is already stored
 * in the packet.Fill GAP Make Discoverable Command packet and send the packet to
 * CC2650 through SPI.
 * @param Input Parameters:
 * <BR> GapMakeDiscoverable_t* para is the parameter we need to send to CC2650 through
 * SPI.
 * @return Return Values: None
 */
void GAP_makeDiscoverablePacketize(gapMakeDiscoverable_t* para)
{
	unsigned char i = 0;
#ifdef NPI_SPI
	// Fill Header and Packet Length First. Checked by CC26XX
	gGapPacket[i++] = 0xFE;        //Packet Header
	gGapPacket[i++] = 0x0E;        //Packet Length
#endif
	//
	// Command Byte
	//
	gGapPacket[i++] = COMMAND;
	//
	// Opcode Bytes ( 2 Bytes)
	//
	gGapPacket[i++] = (unsigned char)GAP_MAKEDISCOVERABLE;
	gGapPacket[i++] = (unsigned char)(GAP_MAKEDISCOVERABLE >> 8);
	//
	// Length byte
	//
	gGapPacket[i++] = 0x0a;
	// EventType
	gGapPacket[i++] = para->EventType;

	// InitiatorAddrType
	gGapPacket[i++] = para->InitiatorAddrType;

	// InitiatorAddr
	unsigned short int j;
	for (j = 0; j < B_ADDR_LEN; j++)
		gGapPacket[i++] = para->InitiatorAddr[j];

	// ChannelMap
	gGapPacket[i++] = para->ChannelMap;

	// FilterPolicy
	gGapPacket[i++] = para->FilterPolicy;

#ifdef NPI_SPI
	gGapPacket[i++] = 0x0C; //CRC Check.-Footer
#endif

	if(i > GAP_PACKET_SIZE)
	{
        // error - GapPacket buffer overflow
		HandleError(1);
		return; // will never reach here
	}

	GAP_sendPacket(gGapPacket, i);

}

/**
 * @brief <b>Fucntion Name </b> GAP_UpdateAdvertisingData
 * @brief <b>Description</b>:
 * Assume that PacketType(1 octet), OpCode(2), Parameter Length(1) is already stored
 * in the packet.Fill GAP Update Advertising Command packet and send the packet to
 * CC2650 through SPI.
 * @param Input Parameters:
 * <BR> GapUpdateAdvertData_t* para is the parameter we need to send to CC2650 through SPI.
 * @return Return Values: None
 */
void GAP_updateAdvertisingDataPacketize(gapUpdateAdvertData_t* para)
{
	unsigned char j,i = 0;
#ifdef NPI_SPI
	// Fill Header and Packet Length First. Checked by CC26XX
	gGapPacket[i++] = 0xFE;        //Packet Header
	gGapPacket[i++] = 6+(para->DataLen);        //Packet Length
#endif

	//
	// Command Byte
	//
	gGapPacket[i++] = COMMAND;
	//
	// Opcode Bytes
	//
	gGapPacket[i++] = (unsigned char)GAP_UPDATEADVERTISINGDATA;
	gGapPacket[i++] = (unsigned char)(GAP_UPDATEADVERTISINGDATA >> 8);
	//
	// Length Byte
	//
	gGapPacket[i++] = para->DataLen + 2;
	gGapPacket[i++] = para->AdType;
	gGapPacket[i++] = para->DataLen;

	for (j = 0; j < para->DataLen; j++)
		gGapPacket[i++] = para->AdvertData[j];

#ifdef NPI_SPI
	gGapPacket[i++] = 0x0C; //CRC Check.-Footer
#endif

	if(i > GAP_PACKET_SIZE)
	{
        // error - GapPacket buffer overflow
		HandleError(1);
		return; // will never reach here
	}

	GAP_sendPacket(gGapPacket, i);

}

void GAP_endDiscoverablePacketize()
{
	unsigned char i = 0;
#ifdef NPI_SPI
	// Fill Header and Packet Length First. Checked by CC26XX
	gGapPacket[i++] = 0xFE;        //Packet Header
	gGapPacket[i++] = 0x04;        //Packet Length
#endif
	//
	// Command Byte
	//
	gGapPacket[i++] = COMMAND;

	//
	// Opcode Bytes
	//
	gGapPacket[i++] = (unsigned char)GAP_ENDDISCOVERABLE;
	gGapPacket[i++] = (unsigned char)(GAP_ENDDISCOVERABLE >> 8);

	// Length byte
	//
	gGapPacket[i++] = 0x00;

#ifdef NPI_SPI
	gGapPacket[i++] = 0x0C; //CRC Check.-Footer
#endif

	if(i > GAP_PACKET_SIZE)
	{
        // error - GapPacket buffer overflow
		HandleError(1);
		return; // will never reach here
	}

	GAP_sendPacket(gGapPacket, i);
}

void GAP_establishLinkRequestPacketize(gapEstLinkReq_t *para)
{
	unsigned char j, i = 0;
#ifdef NPI_SPI
	// Fill Header and Packet Length First. Checked by CC26XX
	gGapPacket[i++] = 0xFE;        //Packet Header
	gGapPacket[i++] = 0x0D;        //Packet Length
#endif
	//
	// Command Byte
	//
	gGapPacket[i++] = COMMAND;
	//
	// Opcode Byte
	//
	gGapPacket[i++] = (unsigned char)GAP_ESTABLISHLINKREQUEST;
	gGapPacket[i++] = (unsigned char)(GAP_ESTABLISHLINKREQUEST >> 8);

	// Length byte
	//
	gGapPacket[i++] = 0x09;


	gGapPacket[i++] = para->highDutyCycle;

	gGapPacket[i++] = para->whiteList;

	gGapPacket[i++] = para->addrTypePeer;

	for(j=0;j<B_ADDR_LEN;j++)
		gGapPacket[i++] = para->peerAddr[j];

#ifdef NPI_SPI
	gGapPacket[i++] = 0x0C; //CRC Check.-Footer
#endif

	if(i > GAP_PACKET_SIZE)
	{
        // error - GapPacket buffer overflow
		HandleError(1);
		return; // will never reach here
	}

	GAP_sendPacket(gGapPacket, i);
}



void GAP_terminateLinkRequestPackeize(gapTerminateLink_t *para)
{
	unsigned char i = 0;
#ifdef NPI_SPI
	// Fill Header and Packet Length First. Checked by CC26XX
	gGapPacket[i++] = 0xFE;        //Packet Header
	gGapPacket[i++] = 0x07;        //Packet Length
#endif
	//
	// Command Byte
	//
	gGapPacket[i++] = COMMAND;
	//
	// Opcode Bytes
	//
	gGapPacket[i++] = (unsigned char)GAP_TERMINATELINKREQUEST;
	gGapPacket[i++] = (unsigned char)(GAP_TERMINATELINKREQUEST >> 8);

	// Length byte
	//
	gGapPacket[i++] = 0x03;


	gGapPacket[i++] = (unsigned char)para->connHandle;

	gGapPacket[i++] = (unsigned char)(para->connHandle >> 8);

	gGapPacket[i++] = para->reason;

#ifdef NPI_SPI
	gGapPacket[i++] = 0x0C; //CRC Check.-Footer
#endif

	if(i > GAP_PACKET_SIZE)
	{
        // error - GapPacket buffer overflow
		HandleError(1);
		return; // will never reach here
	}

	GAP_sendPacket(gGapPacket, i);
}

void GAP_authenticatePacketize(gapAuthenticateParams_t *para)
{
	uint8 j, i = 0;
	uint8 tmp = 0;
#ifdef NPI_SPI
	// Fill Header and Packet Length First. Checked by CC26XX
	gGapPacket[i++] = 0xFE;        //Packet Header
	gGapPacket[i++] = 0x20;        //Packet Length
#endif
	//
	// Command Byte
	//
	gGapPacket[i++] = COMMAND;
	//
	// Opcode Bytes (2 Bytes)
	//
	gGapPacket[i++] = (uint8)GAP_AUTHENTICATE;
	gGapPacket[i++] = (uint8)(GAP_AUTHENTICATE >> 8);

	// Length Byte
	//
	gGapPacket[i++] = 0x1D;


	gGapPacket[i++] = (uint8)(para->pParams->connectionHandle);

	gGapPacket[i++] = (uint8)(para->pParams->connectionHandle >> 8);

	gGapPacket[i++] = para->pParams->secReqs.ioCaps;

	gGapPacket[i++] = para->pParams->secReqs.oobAvailable;

	for(j=0;j<16;j++)
		gGapPacket[i++] = para->pParams->secReqs.oob[j];

	gGapPacket[i++] = para->pParams->secReqs.authReq;

	gGapPacket[i++] = para->pParams->secReqs.maxEncKeySize;

    tmp |= ( para->pParams->secReqs.keyDist.sEncKey ) ? 0x01 : 0;
    tmp |= ( para->pParams->secReqs.keyDist.sIdKey ) ? 0x02 : 0;
    tmp |= ( para->pParams->secReqs.keyDist.sSign ) ? 0x04 : 0;
    tmp |= ( para->pParams->secReqs.keyDist.mEncKey ) ? 0x10 : 0;
    tmp |= ( para->pParams->secReqs.keyDist.mIdKey ) ? 0x20 : 0;
    tmp |= ( para->pParams->secReqs.keyDist.mSign ) ? 0x40 : 0;

	gGapPacket[i++] = tmp;

	gGapPacket[i++] = para->pPairReq->enable;

	gGapPacket[i++] = para->pPairReq->ioCap;

	gGapPacket[i++] = para->pPairReq->oobDataFlag;

	gGapPacket[i++] = para->pPairReq->authReq;

	gGapPacket[i++] = para->pPairReq->maxEncKeySize;

	gGapPacket[i++] = tmp;

#ifdef NPI_SPI
	gGapPacket[i++] = 0x0C; //CRC Check.-Footer
#endif

	if(i > GAP_PACKET_SIZE)
	{
        // error - GapPacket buffer overflow
		HandleError(1);
		return; // will never reach here
	}

	GAP_sendPacket(gGapPacket, i);
}

void GAP_updateLinkParamterRequestPacketize(gapUpdateLinkParamReq_t *para)
{
	unsigned char i = 0;
#ifdef NPI_SPI
	// Fill Header and Packet Length First. Checked by CC26XX
	gGapPacket[i++] = 0xFE;        //Packet Header
	gGapPacket[i++] = 0x0E;        //Packet Length
#endif
	//
	// Command Byte
	//
	gGapPacket[i++] = COMMAND;
	//
	// Opcode Bytes (2 Bytes)
	//
	gGapPacket[i++] = (unsigned char)GAP_UPDATELINKPARAMREQUEST;
	gGapPacket[i++] = (unsigned char)(GAP_UPDATELINKPARAMREQUEST >> 8);

	// Length Byte
	//
	gGapPacket[i++] = 0x0A;


	gGapPacket[i++] = (unsigned char)para->connectionHandle;

	gGapPacket[i++] = (unsigned char)(para->connectionHandle >> 8);

	gGapPacket[i++] = (unsigned char)(para->intervalMin);

	gGapPacket[i++] = (unsigned char)(para->intervalMin >> 8);

	gGapPacket[i++] = (unsigned char)para->intervalMax;

	gGapPacket[i++] = (unsigned char)(para->intervalMax >> 8);

	gGapPacket[i++] = (unsigned char)(para->connLatency);

	gGapPacket[i++] = (unsigned char)(para->connLatency >> 8);

	gGapPacket[i++] = (unsigned char)(para->connTimeout);

	gGapPacket[i++] = (unsigned char)(para->connTimeout >> 8);

#ifdef NPI_SPI
	gGapPacket[i++] = 0x0C; //CRC Check.-Footer
#endif

	if(i > GAP_PACKET_SIZE)
	{
        // error - GapPacket buffer overflow
		HandleError(1);
		return; // will never reach here
	}

	GAP_sendPacket(gGapPacket, i);

}

void GAP_BondPacketize(gapBond_t *para)
{

	unsigned char j, i = 0;
#ifdef NPI_SPI
	// Fill Header and Packet Length First. Checked by CC26XX
	gGapPacket[i++] = 0xFE;        //Packet Header
	gGapPacket[i++] = 0x22;        //Packet Length
#endif
	//
	// Command Byte
	//
	gGapPacket[i++] = COMMAND;
	//
	// Opcode Bytes (2 Bytes)
	//
	gGapPacket[i++] = (unsigned char)GAP_BOND;
	gGapPacket[i++] = (unsigned char)(GAP_BOND >> 8);

	// Length Byte
	//
	gGapPacket[i++] = 0x1E;


	gGapPacket[i++] = (unsigned char)para->connHandle;

	gGapPacket[i++] = (unsigned char)(para->connHandle >> 8);

	gGapPacket[i++] = para->authenticated;

	for(j=0;j<16;j++)
		gGapPacket[i++] = para->pParams->ltk[j];

	gGapPacket[i++] = (unsigned char)(para->pParams->div);

	gGapPacket[i++] = (unsigned char)(para->pParams->div >> 8);

	for(j=0;j<8;j++)
		gGapPacket[i++] = para->pParams->rand[j];

	gGapPacket[i++] = para->pParams->keySize;

#ifdef NPI_SPI
	gGapPacket[i++] = 0x0C; //CRC Check.-Footer
#endif

	if(i > GAP_PACKET_SIZE)
	{
        // error - GapPacket buffer overflow
		HandleError(1);
		return; // will never reach here
	}

	GAP_sendPacket(gGapPacket, i);
}

void GAP_TerminateAuthPacketize(gapTerminateAuth_t* para)
{
	unsigned char i = 0;
#ifdef NPI_SPI
	// Fill Header and Packet Length First. Checked by CC26XX
	gGapPacket[i++] = 0xFE;        //Packet Header
	gGapPacket[i++] = 0x07;        //Packet Length
#endif
	//
	// Command Byte
	//
	gGapPacket[i++] = COMMAND;
	//
	// Opcode Bytes (2 Bytes)
	//
	gGapPacket[i++] = (unsigned char) GAP_TERMINATEAUTH;
	gGapPacket[i++] = (unsigned char) (GAP_TERMINATEAUTH >> 8);

	// Length Byte
	//
	gGapPacket[i++] = 0x03;


	gGapPacket[i++] = (unsigned char)para->connHandle;

	gGapPacket[i++] = (unsigned char)(para->connHandle >> 8);

	gGapPacket[i++] = para->reason;


#ifdef NPI_SPI
	gGapPacket[i++] = 0x0C; //CRC Check.-Footer
#endif

	if(i > GAP_PACKET_SIZE)
	{
        // error - GapPacket buffer overflow
		HandleError(1);
		return; // will never reach here
	}

	GAP_sendPacket(gGapPacket, i);
}

void GAP_SignablePacketize(gapSignable_t *para)
{
	uint8 j, i = 0;
#ifdef NPI_SPI
	// Fill Header and Packet Length First. Checked by CC26XX
	gGapPacket[i++] = 0xFE;        //Packet Header
	gGapPacket[i++] = 0x1A;        //Packet Length
#endif
	//
	// Command Byte
	//
	gGapPacket[i++] = COMMAND;
	//
	// Opcode Bytes (2 Bytes)
	//
	gGapPacket[i++] = (uint8)GAP_SIGNABLE;
	gGapPacket[i++] = (uint8)(GAP_SIGNABLE >> 8);

	// Length Byte
	//
	gGapPacket[i++] = 0x17;


	gGapPacket[i++] = (uint8)para->connHandle;

	gGapPacket[i++] = (uint8)(para->connHandle >> 8);

	gGapPacket[i++] = para->authenticated;

	for(j=0;j<16;j++)
		gGapPacket[i++] = para->pParams->srk[j];

	gGapPacket[i++] = (uint8)(para->pParams->signCounter);

	gGapPacket[i++] = (uint8)(para->pParams->signCounter >> 8);

	gGapPacket[i++] = (uint8)(para->pParams->signCounter >> 16);

	gGapPacket[i++] = (uint8)(para->pParams->signCounter >> 24);


#ifdef NPI_SPI
	gGapPacket[i++] = 0x0C; //CRC Check.-Footer
#endif

	if(i > GAP_PACKET_SIZE)
	{
        // error - GapPacket buffer overflow
		HandleError(1);
		return; // will never reach here
	}

	GAP_sendPacket(gGapPacket, i);
}

void GAP_PasskeyUpdatePacketize(gapPassKeyUpdateParam_t *para)
{
	//to be Filled
}

void GAP_setParameterPacketize(gapSetParam_t *para)
{
	unsigned char i = 0;
#ifdef NPI_SPI
	// Fill Header and Packet Length First. Checked by CC26XX
	gGapPacket[i++] = 0xFE;        //Packet Header
	gGapPacket[i++] = 0x07;        //Packet Length
#endif
	//
	// Command Byte
	//
	gGapPacket[i++] = COMMAND;
	//
	// Opcode Bytes (2 Bytes)
	//
	gGapPacket[i++] = (unsigned char)GAP_SETPARAMETER;
	gGapPacket[i++] = (unsigned char)(GAP_SETPARAMETER >> 8);

	// Length Byte
	//
	gGapPacket[i++] = 0x03;


	gGapPacket[i++] = para->paramID;

	gGapPacket[i++] = (unsigned char)(para->paramValue);

	gGapPacket[i++] = (unsigned char)(para->paramValue >> 8);


#ifdef NPI_SPI
	gGapPacket[i++] = 0x0C; //CRC Check.-Footer
#endif

	if(i > GAP_PACKET_SIZE)
	{
        // error - GapPacket buffer overflow
		HandleError(1);
		return; // will never reach here
	}

	GAP_sendPacket(gGapPacket, i);
}

void GAP_getParameterPacketize(gapGetParam_t *para)
{
	unsigned char i = 0;
#ifdef NPI_SPI
	// Fill Header and Packet Length First. Checked by CC26XX
	gGapPacket[i++] = 0xFE;        //Packet Header
	gGapPacket[i++] = 0x05;        //Packet Length
#endif
	//
	// Command Byte
	//
	gGapPacket[i++] = COMMAND;
	//
	// Opcode Bytes (2 Bytes)
	//
	gGapPacket[i++] = (unsigned char)GAP_GETPARAMETER;
	gGapPacket[i++] = (unsigned char)(GAP_GETPARAMETER >> 8);

	// Length Byte
	//
	gGapPacket[i++] = 0x01;


	gGapPacket[i++] = para->paramID;

#ifdef NPI_SPI
	gGapPacket[i++] = 0x0C; //CRC Check.-Footer
#endif

	if(i > GAP_PACKET_SIZE)
	{
        // error - GapPacket buffer overflow
		HandleError(1);
		return; // will never reach here
	}

	GAP_sendPacket(gGapPacket, i);
}

void GAP_ResolvePrivateAddrPacketize(gapResolvePrivateAddr_t *para)
{
	uint8 j, i = 0;
#ifdef NPI_SPI
	// Fill Header and Packet Length First. Checked by CC26XX
	gGapPacket[i++] = 0xFE;        //Packet Header
	gGapPacket[i++] = 0x19;        //Packet Length
#endif
	//
	// Command Byte
	//
	gGapPacket[i++] = COMMAND;
	//
	// Opcode Bytes (2 Bytes)
	//
	gGapPacket[i++] = (uint8)GAP_RESOLVE_PRIVATE_ADDR;
	gGapPacket[i++] = (uint8)(GAP_RESOLVE_PRIVATE_ADDR >> 8);

	// Length Byte
	//
	gGapPacket[i++] = 0x16;    //22 Bytes for IRK(16 bytes) and Addr(6 Bytes)


	for(j=0;j<16;j++)
		gGapPacket[i++] = *((para->pIRK)+j);

	for(j=0;j<6;j++)
		gGapPacket[i++] = *((para->pAddr)+j);

#ifdef NPI_SPI
	gGapPacket[i++] = 0x0C; //CRC Check.-Footer
#endif

	if(i > GAP_PACKET_SIZE)
	{
        // error - GapPacket buffer overflow
		HandleError(1);
		return;
	}

	GAP_sendPacket(gGapPacket, i);
}

void GAP_SendSlaveSecurityRequestPacketize(gapSendSlaveSecReq_t *para)
{
	//To be Filled
}

void GAPBondMgr_setParameterPacketize(gapBondMgrSetParams_t *para)
{

	unsigned char j, i = 0;
#ifdef NPI_SPI
	// Fill Header and Packet Length First. Checked by CC26XX
	gGapPacket[i++] = 0xFE;        //Packet Header
	gGapPacket[i++] = 0x07 + para->paramDatalen;        //Packet Length
#endif
	//
	// Command Byte
	//
	gGapPacket[i++] = COMMAND;
	//
	// Opcode Bytes (2 Bytes)
	//
	gGapPacket[i++] = (unsigned char)GAPBONDMGR_SETPARAMETER;
	gGapPacket[i++] = (unsigned char)(GAPBONDMGR_SETPARAMETER >> 8);

	// Length Byte
	//
	gGapPacket[i++] = 0x03 + para->paramDatalen;


	gGapPacket[i++] = (unsigned char)(para->paramId);

	gGapPacket[i++] = (unsigned char)(para->paramId >> 8);

	gGapPacket[i++] = para->paramDatalen;

	for(j=0;j<para->paramDatalen;j++)
		gGapPacket[i++] = *(para->pValue++);


#ifdef NPI_SPI
	gGapPacket[i++] = 0x0C; //CRC Check.-Footer
#endif

	if(i > GAP_PACKET_SIZE)
	{
        // error - GapPacket buffer overflow
		HandleError(1);
		return;
	}

	GAP_sendPacket(gGapPacket, i);

}

void GAPBondMgr_getParameterPacketize(gapBondMgrGetParams_t *para)
{
	unsigned char i = 0;
#ifdef NPI_SPI
	// Fill Header and Packet Length First. Checked by CC26XX
	gGapPacket[i++] = 0xFE;        //Packet Header
	gGapPacket[i++] = 0x06;        //Packet Length
#endif
	//
	// Command Byte
	//
	gGapPacket[i++] = COMMAND;
	//
	// Opcode Bytes (2 Bytes)
	//
	gGapPacket[i++] = (unsigned char)GAPBONDMGR_GETPARAMETER;
	gGapPacket[i++] = (unsigned char)(GAPBONDMGR_GETPARAMETER >> 8);

	// Length Byte
	//
	gGapPacket[i++] = 0x02;

	gGapPacket[i++] = (unsigned char)(para->paramId);

	gGapPacket[i++] = (unsigned char)(para->paramId >> 8);


#ifdef NPI_SPI
	gGapPacket[i++] = 0x0C; //CRC Check.-Footer
#endif

	if(i > GAP_PACKET_SIZE)
	{
        // error - GapPacket buffer overflow
		HandleError(1);
		return;
	}

	GAP_sendPacket(gGapPacket, i);
}

