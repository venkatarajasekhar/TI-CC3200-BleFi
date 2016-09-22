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
// generic_task.c - Implementation of a generic time based task.
//
//*****************************************************************************
//*****************************************************************************
//
//! \addtogroup get_weather
//! @{
//
//*****************************************************************************
#include <hw_types.h>
#include <uart.h>
#include <datatypes.h>
#include <simplelink.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "osi.h"
#include "gateway_api.h"
#define BLEFI_CFG_FILE "blefi.cfg"

blefiCfg_t blefiCfgRec;
void * blefiCfgData;
bool is_blefiCfgReady = false;
bool is_blefiCfgupdate = false;
OsiSyncObj_t g_GWGenSyncObj;

#define GENERIC_TASK_STACK_SIZE	2048
#define GENERIC_TASK_FREQ_MS 250
#define AUTO_SCAN_DEFAULT 0 //in secs
#define AUTO_SCAN_MAX_VALUE 300
#define AUTO_SCAN_MIN_VALUE 30
#define AUTO_CON_ENABLE 1
#define AUTO_CON_DISABLE 0
unsigned int cfgSize = 0;
unsigned int autoScanTimeout = 0;


/*
 * EXTERN FUNCTIONS
 * */
extern void Button2Init(void);
extern void Button2IntClear(void);
extern void Button2IntDisable(void);
extern void Button2IntEnable(void);

extern void LoadBleFiCfg(void);
extern void writeBleCfg(void);
extern unsigned int getGenericTaskTicks(void);
extern void HandleError(unsigned char * filename, unsigned int line_num, unsigned char error);

extern signed char  FlashWriteFile(unsigned char* pUserFileName, unsigned char * pBuffer , unsigned int BufLen);
extern signed char  FlashReadFile(unsigned char* pUserFileName, unsigned char * pBuffer , unsigned int BufLen);
extern signed char  FlashIsFilePresent(unsigned char* pUserFileName);

extern void mqttPublishData(void);
extern unsigned int getSizeofMqttCfg(void);
extern unsigned int getSizeofBondCfg(void);
extern void fillMqttDefaultCfg(void * mqttRec);
extern void fillBondDefaultCfg(void * bondRec);
extern void fillOtaMetaDefaultCfg(void * otaMeta);
extern int getSizeofOtaMeta();
extern void DB_FlushSyncObjects();




void doHandleCmdtimeout();


/*
 *  FUNCTIONS
 * */

unsigned int getGenericTaskTicksPerSec()
{
	return(1000/GENERIC_TASK_FREQ_MS);
}


void doPeriodicScan()
{
	int devices_found;
	if (GC_IsBLEReady() != true)
	{
		return;
	}
	if(*(blefiCfgRec.autoScan) == 0)
	{
		if(autoScanTimeout!=0xFFFF)
		{
			/*
			 * This is the condition where the autoscan is zero
			 * and the trigger is not from interrupt
			 */

			return;
		}

	}
	Button2IntDisable();
	autoScanTimeout++;
	if(autoScanTimeout>=(getGenericTaskTicksPerSec()* (*(blefiCfgRec.autoScan))))
	{
		devices_found = GC_Scan(GW_CALL_BLOCKING);
		if (devices_found < 0) {
			UART_PRINT("\n\r[GW] ERROR - Discovery already in progress");
		}
		autoScanTimeout = 0;
	}
	Button2IntEnable();
}



void fillDefaultCfgVal()
{
	 *(unsigned int *) (((unsigned char *)blefiCfgData)) = 0x0; //CC26XX CRC
	 //Mqtt Default
	fillMqttDefaultCfg((void*)(((unsigned char *)blefiCfgData) + (0+sizeof(unsigned int))));
	//Bond Manager Default
	fillBondDefaultCfg((void *)(((unsigned char *)blefiCfgData) + (0+sizeof(unsigned int)+getSizeofMqttCfg())));
	//AutoScan default Value
	*(unsigned short *)(((unsigned char *)blefiCfgData) + (0+sizeof(unsigned int)+getSizeofMqttCfg()+getSizeofBondCfg())) = AUTO_SCAN_DEFAULT;

	*(unsigned short *)(((unsigned char *)blefiCfgData) + (0+sizeof(unsigned int)+getSizeofMqttCfg()+getSizeofBondCfg()+sizeof(unsigned short))) = AUTO_CON_DISABLE;

	fillOtaMetaDefaultCfg((void *)(((unsigned char *)blefiCfgData) + (0+sizeof(unsigned int)+getSizeofMqttCfg()+sizeof(unsigned int))));
}


//****************************************************************************
//
//! Task function implementing the GW generic  functions
//!
//! \param none
//!
//! \return None.
//
//****************************************************************************
void GwGenericTask(void *pvParameters)
{
    volatile unsigned char LoopVar = 0xFF;
    osi_SyncObjWait(&g_GWGenSyncObj,OSI_WAIT_FOREVER); //Wait until the network is ON.
    LoadBleFiCfg();
    while(LoopVar)
    {
    	osi_Sleep(GENERIC_TASK_FREQ_MS);
    	mqttPublishData();
    	writeBleCfg();
    	doPeriodicScan();
    	//doHandleCmdtimeout();
    }
}

//*****************************************************************************
//
//*****************************************************************************
void GW_GenericTask_Init()
{

  //
  // Create sync object to signal Sl_Start and Wlan Connect complete
  //
  osi_SyncObjCreate(&g_GWGenSyncObj);

  /*Initialize Gateway Button*/
  Button2Init();

  //
  // GW Generic Task
  //
  osi_TaskCreate(GwGenericTask,
                  (const signed char *)"GW generic",
                  GENERIC_TASK_STACK_SIZE,
                  NULL,
                  1,
                  NULL );

}

void KickGenericTask()
{
	osi_SyncObjSignal(&g_GWGenSyncObj);
}


/*********************************************************************
 * @fn      blefiCfgInit
 *
 * @brief   Initiate ble Cfg Read
 *
 * @param
 *
 * @return  none
 */
static signed char blefiCfgInit()
{
	signed char fileStatus;

	/*
	 *
	 * Blefi CFG file contains
	 * 1. CC26XX CRC - 4 bytes
	 * 2. Mqtt Cfg
	 * 3. Bond information
	 * The bond and Mqtt structure are very specific.
	 * */
	cfgSize = sizeof(unsigned int)+getSizeofMqttCfg()+getSizeofBondCfg()+sizeof(unsigned int)+getSizeofOtaMeta();
	blefiCfgData = malloc(cfgSize);
	if(blefiCfgData == NULL)
	{
		HandleError(__FILE__,__LINE__,1);
		return(-1);
	}
	blefiCfgRec.cc26xxCRC = (unsigned int *) (((unsigned char *)blefiCfgData));
	blefiCfgRec.mqttCfg = (void *) (((unsigned char *)blefiCfgData) + (0+sizeof(unsigned int)));
	blefiCfgRec.bonds =  (void *) (((unsigned char *)blefiCfgData) + (0+sizeof(unsigned int)+getSizeofMqttCfg()));
	blefiCfgRec.autoScan = (unsigned short *)(((unsigned char *)blefiCfgData) + (0+sizeof(unsigned int)+getSizeofMqttCfg()+getSizeofBondCfg()));
	blefiCfgRec.autocon = (unsigned short *)(((unsigned char *)blefiCfgData) + (0+sizeof(unsigned int)+getSizeofMqttCfg()+getSizeofBondCfg()+sizeof(unsigned short)));
	blefiCfgRec.otaMeta = (char *)(((unsigned char *)blefiCfgData) + (0+sizeof(unsigned int)+getSizeofMqttCfg()+getSizeofBondCfg()+sizeof(unsigned int)));

	// If file is present
	if(FlashIsFilePresent(BLEFI_CFG_FILE) == 0)
	{

		fileStatus=  FlashReadFile(BLEFI_CFG_FILE, (unsigned char *)blefiCfgData,cfgSize);
		if(fileStatus <0)
		{
			fillDefaultCfgVal();
			return(-1);
		}
	}
	else  // file is absent
	{
		fillDefaultCfgVal();
		fileStatus=  FlashWriteFile(BLEFI_CFG_FILE, (unsigned char *)blefiCfgData,cfgSize);
		if(fileStatus <0)
		{
			return(-1);
		}
	}
	if(*(blefiCfgRec.autoScan)>AUTO_SCAN_MAX_VALUE)
	{
		*(blefiCfgRec.autoScan) = AUTO_SCAN_MAX_VALUE;
	}
	else if(*(blefiCfgRec.autoScan) < AUTO_SCAN_MIN_VALUE)
	{
		*(blefiCfgRec.autoScan) = 0;
	}
	Report("\n\rAutoscan %d seconds",*(blefiCfgRec.autoScan));
	return 0;
}

unsigned int * cc26xxCRC;
void *  mqttCfg;
void *  bonds;


void LoadBleFiCfg()
{
	blefiCfgInit();
	is_blefiCfgReady = true;
}

void writeBleCfg()
{
	unsigned char * tempblefiCfgRec;
	signed char fileStatus;
	if((is_blefiCfgupdate == true) && (is_blefiCfgReady == true))
	{
		tempblefiCfgRec = malloc(cfgSize);
		if(tempblefiCfgRec==NULL)
		{
			HandleError(__FILE__,__LINE__,1);
			return;
		}
		memcpy(tempblefiCfgRec,blefiCfgData,cfgSize);
		fileStatus=  FlashWriteFile(BLEFI_CFG_FILE, (unsigned char *)tempblefiCfgRec,cfgSize);
		if(fileStatus <0)
		{
			is_blefiCfgupdate = false;
			free(tempblefiCfgRec);
			return;
		}
		free(tempblefiCfgRec);
		is_blefiCfgupdate = false;
	}

}


void UpdateBlefiCfg()
{
	is_blefiCfgupdate = true;
}

/*Gateway Button handler*/
void GwButtonHandler()
{

	Message("\n\r[GW] Scan Button Pressed. discovering devices...");
	Button2IntDisable();
	Button2IntClear();
	//Signal for the scan
	autoScanTimeout = 0xFFFF;
}


/*Initialize Gateway Button*/
void Gateway_ButtonInit()
{
	Button2Init();
}

int configureAutoScan(unsigned int timeout)
{
	if(timeout>AUTO_SCAN_MAX_VALUE)
	{
		return -1;
	}
	else if(timeout < AUTO_SCAN_MIN_VALUE)
	{
		*(blefiCfgRec.autoScan) = 0;
	}
	else
	{
		*(blefiCfgRec.autoScan) = timeout;
	}

	UpdateBlefiCfg();
	return((int)*(blefiCfgRec.autoScan));
}

int readAutoScan()
{
	return((int)*(blefiCfgRec.autoScan));
}

void enforceCC26XXFwChecsum(unsigned int checksum)
{
	*(blefiCfgRec.cc26xxCRC) = 0x0;
	UpdateBlefiCfg();

}

void doHandleCmdtimeout()
{
	if (GC_IsBLEReady() != true)
	{
		return;
	}
	DB_FlushSyncObjects();
}

void disableAutoconnect()
{
	*(blefiCfgRec.autocon) = AUTO_CON_DISABLE;
	UpdateBlefiCfg();
}

void enableAutoconnect()
{
	*(blefiCfgRec.autocon) = AUTO_CON_ENABLE;
	UpdateBlefiCfg();
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
