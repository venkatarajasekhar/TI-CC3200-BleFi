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

#include "datatypes.h"
#include "ble_events.h"
#include "hci.h"

#include "spi_def.h"
#include "uart_def.h"

unsigned char   gOutGoingPacket = TRUE;
unsigned char   gCC2650Packet   = FALSE;
unsigned char   gAckEvent       = FALSE;

extern unsigned char g_ucSpiRxBuffer[CC2650_SPI_RX_BUFFER_SIZE];
extern unsigned char g_ucUartAckRxBuffer[CC2650_UART_RX_BUFFER_SIZE];


/*********************************************************************
 * LOCAL VARIABLES
 */
NpiEvtCB_t *pNpiEvtCb;

/*******************************************************************************
 * @fn          NPI_RegisterTask
 *
 * @brief       This routine registers the Event call back from NPI layer
 *
 * input parameters
 *
 * @param       Event Callback pointer
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void NPI_RegisterTask(NpiEvtCB_t *pCBs)
{
	pNpiEvtCb = pCBs;
}

/**
 * @brief <b>Fucntion Name </b> BLE_decodeIncomingPacket
 * @brief <b>Description</b>: Decode the packet
 * @param Input Parameters:
 * <BR> unsigned char* packet is the packets we receive through SPI.
 * @param:
 * <BR> unsigned short int* received_opcode is a pointer to store the received op_code.
 * @param:
 * <BR> bStatus_t* status_param is a pointer to store status of received packet.
 * @return Return value: None
 */
void BLEAPI_decodeIncomingPacket(unsigned char* packet, unsigned short int* op_code_received, bStatus_t* status_param)
{

	bStatus_t status;
	CommandHeader_t header;

	status = HCI_decodeHeader(packet, &header);

	if(status == SUCCESS)
	{
		op_code_received[0] = header.OpCode;

		if (header.PacketType == EVENT)
		{
			switch(header.OpCode)
			{
			case GAP_HCI_EXTENTIONCOMMANDSTATUS:
				//
				// expected_op_code[1] is used for the case of COMMAND STATUS
				// error_flag will be set to TRUE when the expected_op_code is not same as the expected_op_code[1]
				//
				*status_param = HCI_commandStatus(packet, &op_code_received[1]);
				break;
			default:
				break;
				}
		}
		else if (header.PacketType == ASYNCDATA)
		{

		}
		else if (header.PacketType == SYNCDATA)
		{

		}
	}
}


/**
 * @brief <b>Fucntion Name </b> BLE_checkReceivedEvent
 * @brief <b>Description</b>:
 * Receive the packet from CC2650 and decode it.
 * @param Input Parameters:
 * <BR> unsigned short int* received_opcode is a pointer to store the received op_code.
 * @param:
 * <BR> unsigned char wait_until_cmd_received is the blocking flag for receiving packets.
 * @return Return value:
 * <BR> bStatus_t is the status of received packet.
 */
bStatus_t BLEAPI_checkReceivedEvent(unsigned short int* received_opcode, unsigned char wait_until_cmd_received)
{
	bStatus_t status_flag = FAILURE;

	//
	// receive packets from the CC2650
	//
	if(gCC2650Packet == TRUE)
	{
#if NPI_SPI
		BLEAPI_decodeIncomingPacket(g_ucSpiRxBuffer, received_opcode, &status_flag);
#endif
#if NPI_UART
		BLEAPI_decodeIncomingPacket(g_ucUartAckRxBuffer, received_opcode, &status_flag);
#endif
	}

	return status_flag;
}


/**
 * @brief <b>Fucntion Name </b> Wait_forSpecficEvent
 * @brief <b>Description</b>:
 * Wait for receiving specific event from CC2650 and decode it.
 * @param Input Parameters:
 * <BR> unsigned short int expected_op_code_1 is the first opcode that CC3200 expects to receive.
 * @param:
 * <BR> unsigned short int expected_op_code_2 is the second opcode that CC3200 expects to receive.
 * (This one is used for Command Status event.)
 * @param:
 * <BR> unsigned char wait_until_cmd_received is the blocking flag for receiving packets.
 * @return Return value:
 * <BR> bStatus_t is the status of received packet.
 */
bStatus_t BLEAPI_waitForSpecficEvent(unsigned short int expected_op_code_1, unsigned short int expected_op_code_2, unsigned char wait_until_cmd_received)
{
	bStatus_t status_flag = FAILURE;
	unsigned short int received_op_code[2]= {0x00};
	unsigned int loop_counter = 0;

	while( received_op_code[0] != expected_op_code_1)
	{
		status_flag = BLEAPI_checkReceivedEvent(received_op_code, wait_until_cmd_received);

		loop_counter++;

		if(loop_counter >=1000)
			return FAILURE;
	}

	if( received_op_code[0] == GAP_HCI_EXTENTIONCOMMANDSTATUS && \
			received_op_code[1] != expected_op_code_2)
		return FAILURE;
	else
		return status_flag;
}

