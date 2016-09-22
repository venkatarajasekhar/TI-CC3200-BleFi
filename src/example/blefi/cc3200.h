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
//
// cc3200.h - Defines and Macros for CC3200
//
//*****************************************************************************

#ifndef __CC3200_H__
#define __CC3200_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************

#ifdef __cplusplus
extern "C"
{
#endif

#include "osi.h"
#include "datatypes.h"



#define UNUSED(x) ((x) = (x))
  
//*****************************************************************************
// GPIO Values
//*****************************************************************************
#ifdef PLATFORM_DVP
#define LED1_GPIO              14
#else
#define LED1_GPIO              9
#endif

typedef enum LedNum
{
  LED_1 = 0,
  LED_2
}LedNum;



typedef enum eLEDStatus
{
  LED_OFF = 0,
  LED_ON,
  LED_BLINK
}eLEDStatus;


typedef enum LedBlinkrate
{
SLOW = 0,
MEDIUM,
FAST1,
FAST2,
}LedBlinkrate;
  
//*****************************************************************************
// State Machine values 
//*****************************************************************************
#define NUM_STATES 6
#define FIRST_STATE_LED_NUM 1
#define MAX_SSID_LEN        32

//*****************************************************************************
// CC3000 State Machine Definitions
//*****************************************************************************
enum cc3101StateEnum
{
    CC3101_UNINIT           = 0x01, // CC3101 Driver Uninitialized
    CC3101_INIT             = 0x02, // CC3101 Driver Initialized
    CC3101_ASSOC            = 0x04, // CC3101 Associated to AP
    CC3101_IP_ALLOC         = 0x08, // CC3101 has IP Address
    CC3101_SERVER_INIT      = 0x10, // CC3101 Server Initialized
    CC3101_CLIENT_CONNECTED = 0x20  // CC3101 Client Connected to Server
};

  
//*****************************************************************************
//
// API Function prototypes
//
//*****************************************************************************
void MCUInit(void);
extern void TimerInit(void);
extern void TimerPeriodicIntHandler(void);
extern void AppTimerConfigure();
extern void GPIOConfigure();
extern void Delay(unsigned long ulCount);
extern void InitCallBack(UINT32 Status);

extern void InitDriver(void);
extern void DeInitDriver(void);
extern void UnsetCC3101MachineState(char stat);
extern void SetCC3101MachineState(char stat);
extern void ResetCC3101StateMachine();

void TurnOffLed(LedNum LedInstance);
void TurnOnLed(LedNum LedInstance);
void BlinkLed(LedNum LedInstance,unsigned short iBlinkCount, LedBlinkrate iBlinkRate);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif
#endif //  __CC3101_H__
//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************


