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

#ifndef __BLEEVENTS_H__
#define __BLEEVENTS_H__




// NPI event callback
typedef void (*npiDevEvtCB_t)(void* evt);
typedef npiDevEvtCB_t NpiEvtCB_t;

// ******** Definitions ******************************************************//

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
void NPI_RegisterTask(NpiEvtCB_t *pCBs);
void BLEAPI_decodeIncomingPacket(unsigned char* packet, unsigned short int* op_code_received, bStatus_t* status_param);
bStatus_t BLEAPI_checkReceivedEvent(unsigned short int* received_opcode, unsigned char wait_until_cmd_received);
bStatus_t BLEAPI_waitForSpecficEvent(unsigned short int expected_op_code_1, unsigned short int expected_op_code_2, unsigned char wait_until_cmd_received);


#endif /* BLEEVENTS_H_ */
