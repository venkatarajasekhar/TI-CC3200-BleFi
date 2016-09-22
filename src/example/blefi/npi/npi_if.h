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
 * npi_if.h
 *
 *  Created on: Jul 18, 2014
 *      Author: a0393958
 */

#ifndef NPI_IF_H_
#define NPI_IF_H_


#include <string.h>
#include <stdlib.h>

#define NPI_RTOS 1
#define NPI_UART 1

#define UART_EVENT_HDR_LEN     3

#define NPI_WAITFOREVER        (0xFFFFFFFF)
#define NPIEVENT_TASK_PRIORITY  3
#define NPIEVENT_STACK_SIZE	    2048


#define NPI_MASTERSIGNAL		    1
#define NPI_SLAVESIGNAL             2


#ifdef NPI_RTOS

typedef void* NPI_SyncObj;
typedef void* NpiTaskHandle;

typedef enum
{
	FALLING_EDGE = 0,
	RAISING_EDGE = 1,
	BOTH_EDGES   = 2,
	NO_EDGE      = 3

}Npi_GpioEdge;

//void NPI_TaskCreate(void *NpiEvent_taskFxn, unsigned short usStackDepth,void *pvParameters,unsigned long uxPriority, NpiTaskHandle *pNpiEvent_TaskHandle);
void NPI_TaskCreate(void *NpiEvent_taskFxn, const signed char*  taskName, unsigned short usStackDepth,void *pvParameters,unsigned long uxPriority, NpiTaskHandle *pNpiEvent_TaskHandle);
int  NPI_CreateSyncObj(NPI_SyncObj* pSyncObj);
int  NPI_SyncObjWait(NPI_SyncObj* pSyncObj, int _timeout_);
void NPI_SyncObjSignal(NPI_SyncObj* pSyncObj);
void NPI_SyncObjSignalFromISR(NPI_SyncObj* pSyncObj);
void NPI_Sleep(int duration);
#endif

void NPI_SpiInit(void);
void NPI_SpiReadPacket(unsigned char *pPacket, char len);
void NPI_SpiWritePacket(unsigned char *pPacket, char len);

void NPI_UartInit(void);
void NPI_UartReadPacket(unsigned char *pPacket, char len);
void NPI_UartWritePacket(unsigned char *pPacket, char len);
void NPI_UartDisableInt(void);
void NPI_UartEnableInt(void);

void NPI_RegisterGpioInterrupt(int GPIO_NUM, int edge, void *funcPtr);
void NPI_EnableGPIOInt(int GPIO_NUM);
void NPI_DisableGpioInt(int GPIO_NUM);

void NPI_GpioPinWrite(int GPIO_NUM, int value);
unsigned long NPI_GpioPinRead(int GPIO_NUM);


#endif /* NPI_IF_H_ */
