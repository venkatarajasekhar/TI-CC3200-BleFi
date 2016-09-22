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
 *	@file spi.h
 *
 * 	@detail	Description:
 * 	The header file of spi_def.c.
.
 *  ***************************************************************************/

// Include Library Header Files

#include "datatypes.h"
#include "npi_if.h"

//Maximal length of receiving data is 128
#define	CC2650_SPI_RX_BUFFER_SIZE	128
#define CC2650_SPI_TX_BUFFER_SIZE   128

// Function Prototypes *******************************************************//
#ifdef NPI_RTOS
void NPI_SpiEventCreateTask(void);
void NPI_SpiEventTaskFxn(void);
#endif

void SReadyIntHandler(void);
void SignalMasterReadyLow(void);
void SignalMasterReadyHigh(void);
void SPI_writePacket(unsigned char* packet_buf_ptr, unsigned char len);
void SPI_readPacket(unsigned char* pBuffer, unsigned char len);
void SPI_readSinglePacket(unsigned char* pPacket);
