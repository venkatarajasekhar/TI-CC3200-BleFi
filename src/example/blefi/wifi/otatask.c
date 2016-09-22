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
// Application Name     -
// Application Overview -
//
// Application Details  -
//
//*****************************************************************************

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <datatypes.h>
// simplelink includes
#include "simplelink.h"
#include "wlan.h"
#include "osi.h"

// driverlib includes
#include "hw_ints.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "prcm.h"
#include "utils.h"
#include "uart.h"

// common interface includes
#include "common.h"
#include "uart_if.h"
#include "gpio.h"
#include "ota_api.h"
#include "otaconfig.h"
#include "OtaApp.h"





//*****************************************************************************
// System state Macros
//*****************************************************************************
#define SYS_STATE_WAIT          0
#define SYS_STATE_RUN           1
#define SYS_STATE_REBOOT        2
#define SYS_STATE_TEST_REBOOT   3
#define OTA_MAX_RETRY 			10
#define OTA_META_BUF_LEN 		64

//*****************************************************************************
// Global static variables
//*****************************************************************************
static OsiSyncObj_t g_OTAStatSyncObj;
static OsiSyncObj_t g_NetStatSyncObj;
//static OsiSyncObj_t g_FactResetSyncObj;
static OtaOptServerInfo_t g_otaOptServerInfo;
static void *pvOtaApp;
extern blefiCfg_t blefiCfgRec;

extern int Wifi_GetMAC(unsigned char * macAddressVal);
extern unsigned char BlefiVersion();
extern signed char  FlashIsFilePresent(unsigned char* pUserFileName);
extern signed char  FlashReadFile(unsigned char* pUserFileName, unsigned char * pBuffer , unsigned int BufLen);
extern signed char  FlashWriteFile(unsigned char* pUserFileName, unsigned char * pBuffer , unsigned int BufLen);
extern int OtaAddOtaMeta(signed char *pOtaMeta,unsigned char ucOtaMetaLen );
extern void UpdateBlefiCfg();
//****************************************************************************
//
//! Reboot the MCU by requesting hibernate for a short duration
//!
//! \return None
//
//****************************************************************************
void RebootMCU()
{

  volatile unsigned char LoopVar = 0xFF;

  //
  // Configure hibernate RTC wakeup
  //
  PRCMHibernateWakeupSourceEnable(PRCM_HIB_SLOW_CLK_CTR);

  //
  // Delay loop
  //
  MAP_UtilsDelay(8000000);

  //
  // Set wake up time
  //
  PRCMHibernateIntervalSet(330);

  //
  // Request hibernate
  //
  PRCMHibernateEnter();

  //
  // Control should never reach here
  //
  while(LoopVar)
  {

  }
}


int OtaAddOtaMeta(signed char *pOtaMeta,unsigned char ucOtaMetaLen )
{
	if(ucOtaMetaLen != (OTA_META_BUF_LEN))
	{
	// error
      return -1;
	}
	memcpy(blefiCfgRec.otaMeta,(char *)pOtaMeta,(unsigned char) OTA_META_BUF_LEN);
	Report("\n\rAdded OTA Meta \n\r%s",pOtaMeta);
	UpdateBlefiCfg();
	return 0;
}

//****************************************************************************
//
//! Sets the OTA server info and vendor ID
//!
//! \param pvOtaApp pointer to OtaApp handler
//! \param ucVendorStr vendor string
//! \param pfnOTACallBack is  pointer to callback function
//!
//! This function sets the OTA server info and vendor ID.
//!
//! \return None.
//
//****************************************************************************
int OTAServerInfoSet(void **pvOtaApp, char *vendorStr)
{

    //
    // Set OTA server info
    //
    g_otaOptServerInfo.ip_address = OTA_SERVER_IP_ADDRESS;
    g_otaOptServerInfo.secured_connection = OTA_SERVER_SECURED;
    strcpy((char *)g_otaOptServerInfo.server_domain, OTA_SERVER_NAME);
    strcpy((char *)g_otaOptServerInfo.rest_update_chk, OTA_SERVER_REST_UPDATE_CHK);
    strcpy((char *)g_otaOptServerInfo.rest_rsrc_metadata, OTA_SERVER_REST_RSRC_METADATA);
    strcpy((char *)g_otaOptServerInfo.rest_hdr, OTA_SERVER_REST_HDR);
    memcpy((char *)g_otaOptServerInfo.rest_hdr_val, (const char *)(blefiCfgRec.otaMeta),OTA_META_BUF_LEN);
    g_otaOptServerInfo.rest_hdr_val[OTA_META_BUF_LEN]='\0';
    Report("\n\rOTA Meta <%s>",g_otaOptServerInfo.rest_hdr_val);
    strcpy((char *)g_otaOptServerInfo.log_server_name, LOG_SERVER_NAME);
    strcpy((char *)g_otaOptServerInfo.rest_files_put, OTA_SERVER_REST_FILES_PUT);
    Wifi_GetMAC(g_otaOptServerInfo.log_mac_address);

    //
    // Set OTA server Info
    //
    sl_extLib_OtaSet(*pvOtaApp, EXTLIB_OTA_SET_OPT_SERVER_INFO,
                     sizeof(g_otaOptServerInfo), (_u8 *)&g_otaOptServerInfo);

    //
    // Set vendor ID.
    //
    sl_extLib_OtaSet(*pvOtaApp, EXTLIB_OTA_SET_OPT_VENDOR_ID, strlen(vendorStr),
                     (_u8 *)vendorStr);

    //
    // Return ok status
    //
    return RUN_STAT_OK;
}


int OtaUpdateLoop()
{
	int iRet = -1;
	int SetCommitInt = 1;
	int iRetry = OTA_MAX_RETRY;
	OtaApp_t* tempOtaApp = (OtaApp_t*)pvOtaApp;
	volatile unsigned char LoopVar = 0xFF;

	while(LoopVar)
	{
		iRet = sl_extLib_OtaRun(pvOtaApp);
		Report("\n\r[OTA] <State : %d> <iRet : %d>\n\r",tempOtaApp->state,iRet);

		if ( iRet < 0 )
		{
			if( RUN_STAT_ERROR_CONTINUOUS_ACCESS_FAILURES == iRet )
			{
			  return(iRet);
			}
			else if(RUN_STAT_ERROR_CONNECT_CDN_SERVER == iRet)
			{
				return(iRet);
			}
			else if(RUN_STAT_ERROR_RESOURCE_LIST == iRet)
			{
				return(iRet);
			}
			else if(RUN_STAT_ERROR_DOWNLOAD_SAVE == iRet)
			{
				return(iRet);
			}
			else
			{
			  iRetry--;
			  if(iRetry<=0)
			  {
				  return(iRet);
			  }
			}
		}
		else if( iRet == RUN_STAT_NO_UPDATES )
		{
			return (iRet);
		}
		else if ((iRet & RUN_STAT_DOWNLOAD_DONE))
		{

			iRet = sl_extLib_OtaSet(pvOtaApp, EXTLIB_OTA_SET_OPT_IMAGE_TEST,
									sizeof(int), (_u8 *)&SetCommitInt);
			if (iRet & (OTA_ACTION_RESET_MCU| OTA_ACTION_RESET_NWP) )
			{
				RebootMCU();
				while(LoopVar)
				{
					osi_Sleep(10*1000);
				}
			}
			else
			{
				return(iRet);
			}

		}
	}

	return 0;
}




void OtaCommit()
{

   int SetCommitInt;
   long OptionLen;
   char OptionVal;
   //
   // Check if this image is booted in test mode
   //
   sl_extLib_OtaGet(pvOtaApp,EXTLIB_OTA_GET_OPT_IS_PENDING_COMMIT,&OptionLen,
					 (_u8 *)&OptionVal);

   if(OptionVal == true)
   {
		SetCommitInt = OTA_ACTION_IMAGE_COMMITED;
		sl_extLib_OtaSet(pvOtaApp, EXTLIB_OTA_SET_OPT_IMAGE_COMMIT,
						 sizeof(int), (_u8 *)&SetCommitInt);

	}
}





//****************************************************************************
//
//! Task function implementing the OTA update functionality
//!
//! \param none
//!
//! \return None.
//
//****************************************************************************
void OTAUpdateTask(void *pvParameters)
{
    int iRet;
   // int SetCommitInt = 1;
    unsigned char ucVendorStr[50];
    unsigned long ulVendorStrLen;
    volatile unsigned char LoopVar = 0xFF;

    //Report("\n\r Waiting for WLAN Connect signal");
    //
    // Wait for sl_Start and wlan connect to complete
    //
    osi_SyncObjWait(&g_NetStatSyncObj, OSI_WAIT_FOREVER);

    Report("\n\r[OTA] WLAN Connected");


    while(LoopVar)
    {
    	Report("\n\r[OTA] Waiting for OTA trigger");
		osi_SyncObjWait(&g_OTAStatSyncObj, OSI_WAIT_FOREVER);
		Report("\n\r[OTA] Start");

        strcpy((char *)ucVendorStr,OTA_VENDOR_STRING);

        ulVendorStrLen = strlen(OTA_VENDOR_STRING);

        sprintf((char *)&ucVendorStr[ulVendorStrLen],"%02u",
        		BlefiVersion());

        Report("\n\r[OTA] Vendor String %s ",ucVendorStr );
        //
        // Initialize OTA
        //
        pvOtaApp = sl_extLib_OtaInit(RUN_MODE_NONE_OS | RUN_MODE_BLOCKING,0);

        //If its a Test code, then Commit.
        OtaCommit();

        //
        // Initializa OTA service
        //
        OTAServerInfoSet(&pvOtaApp,(char *)ucVendorStr);

        //
        // OTA update loop
        //
        iRet = OtaUpdateLoop();
        if(iRet == RUN_STAT_NO_UPDATES || iRet == RUN_STAT_ERROR_RESOURCE_LIST)
        {
        	Report("\n\r[OTA] No Updates Available");
        }
        else if(iRet < 0)
        {
        	Report("\n\r[OTA] Error - OTA Failure");
        }
        else
        {
        	Report("\n\r[OTA] Update Successful");
        }
    }
}


//****************************************************************************
//
//! \internal
//!
//! Switch 3 Interuupt handler
//!
//! \return None
//
//****************************************************************************
void OtaButtonHandler()
{

		//
		// Signal the button Press
		//
		MAP_GPIOIntClear(GPIOA1_BASE,GPIO_PIN_7);
		osi_SyncObjSignal(&g_OTAStatSyncObj);


}

//*****************************************************************************
//
//!  \brief Initialize Push Button GPIO
//!
//! \param[in] S2InterruptHdl          GPIO Interrupt Handler for SW2 LP button
//! \param[in] S3InterruptHdl          GPIO Interrupt Handler for SW3 LP button

//!
//! \return none
//!
//! \brief  Initializes Push Button Ports and Pins
//
//*****************************************************************************
void OtaButtonInit()
{

	/*
	 * BUTTON1
	 * */

	//
	// Set Interrupt Type for GPIO
	//
	MAP_GPIOIntTypeSet(GPIOA1_BASE,GPIO_PIN_7,GPIO_FALLING_EDGE);

	//
	// Enable Interrupt
	//
	MAP_GPIOIntClear(GPIOA1_BASE,GPIO_PIN_7);
	MAP_GPIOIntEnable(GPIOA1_BASE,GPIO_INT_PIN_7);
	//
	// Register Interrupt handler
	//

	osi_InterruptRegister(INT_GPIOA1,(P_OSI_INTR_ENTRY)OtaButtonHandler, \
							INT_PRIORITY_LVL_1);


}


//*****************************************************************************
//
//*****************************************************************************
void OTA_Init()
{

  //
  // Create sync object to signal Sl_Start and Wlan Connect complete
  //
  osi_SyncObjCreate(&g_NetStatSyncObj);

  //
  // Create sync object to signal OTA start
  //
  osi_SyncObjCreate(&g_OTAStatSyncObj);

  OtaButtonInit();

  //
  // OTA Update Task
  //
  osi_TaskCreate(OTAUpdateTask,
                  (const signed char *)"OTA Update",
                  OSI_STACK_SIZE,
                  NULL,
                  1,
                  NULL );

}

void otaSignalWLANConnect()
{
	//
	// Signal Wlan COnnect
	//
	osi_SyncObjSignal(&g_NetStatSyncObj);
}

int OtaDownload()
{
	  osi_SyncObjSignal(&g_OTAStatSyncObj);
	  return 0;
}

int getSizeofOtaMeta()
{
	return(OTA_META_BUF_LEN);
}

void fillOtaMetaDefaultCfg(void * otaMeta)
{
	memset(otaMeta,0x0,OTA_META_BUF_LEN);
}
