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

#include <xdc/std.h>
#include <hw_memmap.h>
#include <stdio.h>
#include "datatypes.h"
#include "osi.h"
#include "hw_ints.h"
#include "hw_types.h"
#include "interrupt.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "pinmux.h"
#include "cc3200.h"
#include "uart_if.h"
#include "cli.h"
#include "gpio.h"
#include "utils.h"

//****************************************************************************
//                          LOCAL DEFINES
//****************************************************************************

//****************************************************************************
//                     GLOBAL VARIABLES
//****************************************************************************

extern void Gateway_Init(void);
extern void CLI_Init(void);
extern void MQTT_Init(void);
extern void SimpleLink_Init(void);
extern void OTA_Init(void);
extern void GW_GenericTask_Init(void);



#define BLEFI_VERSION 1
//****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS
//****************************************************************************


unsigned char BlefiVersion()
{
	return(BLEFI_VERSION);
}

void CC26xx_Init(void);
/*
 * CC26xx Initialization
 *
 * */
void CC26xx_Init(void)
{
	/*
	 *	 CC26xx Reset
	 */

	// GPIO_22  - Pin 15
	GPIOPinWrite(GPIOA0_BASE, GPIO_PIN_7, GPIO_PIN_7); //GPIO_7
	UtilsDelay(10 * DELAY_1_MS);
 	GPIOPinWrite(GPIOA2_BASE, GPIO_PIN_6, 0);
 	UtilsDelay(10 * DELAY_1_MS);
 	GPIOPinWrite(GPIOA2_BASE, GPIO_PIN_6, 0xFF);
 	UtilsDelay(100 * DELAY_1_MS);
}

void BlefiDisplayBanner()
{
        UART_PRINT("\n\n\n\r");
        UART_PRINT("*********************************************************\n\r");
        UART_PRINT("       BleFi - BLE-WiFi gateway (Texas Instruments)\n\r");
        UART_PRINT("       Version : %lu   \n\r",BlefiVersion());
        UART_PRINT("*********************************************************\n\r");
        UART_PRINT("\n\n\r");
}

void Blefi_HW_Init()
{

	PRCMCC3200MCUInit();
    IntMasterEnable();
    //
    // Configure the pinmux settings for the peripherals exercised
    //
    PinMuxConfig();
    //
    // Configuring UART
    //
    InitTerm();
    //
    // Enable the SYSTICK interrupt
    //
    IntEnable(FAULT_SYSTICK);
	//Trigger nReset of 26xx
    CC26xx_Init();
	TurnOnLed(LED_2);
    BlefiDisplayBanner();
}


/*
 *  ======== main ========
 */
int main()
{
	volatile unsigned char LoopVar = 0xFF;


	Blefi_HW_Init();
	// Wifi Configuration
	SimpleLink_Init();

	/* Kick off Gateway Central application - Priority 2 */
    Gateway_Init();

    GW_GenericTask_Init();

    // Kick off  CLI task - Priority 1 */
	CLI_Init();

	// Initialize the MQTT App, always after Wifi_Init
	MQTT_Init();

     // Start the task scheduler
 	OTA_Init();

    // Start the task scheduler
    osi_start();
    while(LoopVar);

    // we will never reach here
    return 0;
}


