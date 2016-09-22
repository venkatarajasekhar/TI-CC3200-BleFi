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
 *	@file hci.c
 *
 * 	@detail	Description:
 *	To decode packet header, when MCU SPI received packet from CC2541S. Now we
 *	just handle with two kinds of packets, COMMEND and EVENT.
 *
 * 	The file Architecture for blelib is,
 * 				blelib.lib
 * 	|	gap.h/.c	|	gatt.h/.c	|
 * 	|				|	att.h/.c	|
 * 	|			hci.h/.c			|
 * 	|			spi.h/.c			|
 * 	|	msp430.h	|	def.h		|
 *
 *  ***************************************************************************/

// Include Library Header Files
#include "hci.h"

/**
 * @brief <b>Fucntion Name </b> commandStatus
 * @brief <b>Description</b>:
 * It is already decoded before this function, that
 * packet[0] = Total length of payload
 * packet[1] = PacketType(which is 0x04, Event)
 * packet[2] = 0xFF (Vendor Specific Events)
 * packet[3] = Length
 * packet[4:5] = EventOpcode (GAP_HCI_ENTIONCOMMANDSTATUS 0x067F)
 * Using EventOpcode, Event functions will be selected in the Switch() case:
 * In this function, rest of the packet will be decoded and assigned to the struct
 * @param Input Parameters:
 * <BR> unsigned char* packet is the packets we receive through SPI.
 * @param:
 * <BR> unsigned short int* op_code_received is a pointer to store the received op_code.
 * @return Return Values:
 * <BR> bStatus_t is the status of this event.
 */
bStatus_t HCI_commandStatus(unsigned char* packet, unsigned short int* op_code_received)
{
	CommandStatus_t event;
	event.Status = (bStatus_t) packet[5];
	event.OpCode = packet[7];
	event.OpCode = event.OpCode << 8;
	event.OpCode += packet[6];

	*op_code_received = event.OpCode;

	return event.Status;
}

/**
 * @brief <b>Fucntion Name </b> decodeHeader
 * @brief <b>Description</b>:
 * Decode the header part of a packet
 *
 * 1.Received Event Packet Format:
 * 	packet[0] = packet total length;
 * 	packet[1] = packet type;
 * 	packet[2] = 0xff (Vendor Specific event code)
 * 	packet[3] = Parameter total length;
 * 	packet[4:5]  = Op Code;
 *
 * 2.Received Command Packet Format:
 * 	packet[0] = packet total length;
 * 	packet[1] = packet type;
 * 	packet[2:3] = Op Code;
 * 	packet[4] = Parameter total length;
 * 	packet_buf_ptr[5 ... Parameter total length + 4] = Parameters of payload.
 *
 * @param Input Parameters:
 * <BR> unsigned char* packet is the packets we receive through SPI.
 * @return Return value:
 * <BR> struct CommandHeader is the result we decode for the packet.
 *
 */
bStatus_t HCI_decodeHeader(unsigned char* packet, CommandHeader_t* header)
{
	header->PacketType = packet[0];
	if (header->PacketType == EVENT)
	{
		header->OpCode = packet[4];
		header->OpCode = packet[3] + (header->OpCode << 8);

		header->DataLength = packet[2];
	}
	else if (header->PacketType == COMMAND)
	{
		header->OpCode = packet[2];
		header->OpCode = packet[1] + (header->OpCode << 8);

		header->DataLength = packet[3];
	}
	else
	{
		return 1;
	}

	return 0;
}

