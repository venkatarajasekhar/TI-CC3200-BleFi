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


#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "datatypes.h"
#include "osi.h"
#include "gateway_cmds.h"
#include "gateway_task.h"
#include "gateway_api.h"
#include "gapbondmgr.h"

extern blefiCfg_t blefiCfgRec;
/** GLOBAL VARIABLES **/

Msg_t sMsg ;
bool is_bleStackReady = false;
extern gw_context_t gw_context;
extern device_db_t * DeviceRec[MAX_DEVICE_DB_SUPPORTED];
extern OsiSyncObj_t GC_ApiLockObj;

// Globals used for saving bond record and CCC values in NV
extern blefiCfg_t blefiCfgRec;

/**** extern functions*****/
extern bStatus_t gapBondMgrEraseAllBondings( void );
extern void HandleError(unsigned char *, unsigned int, unsigned char error);
extern signed int DB_IsConnIdValid(unsigned int uConnIdx);
extern void gapBondMgrFlashBondRecUpdate(void);

/***********************************************
 *
 * FUNCTIONS
 *
 * ********************************************/
void IfBondingEnabled(void)
{
	unsigned int ucScanIdx = 0xFFFF , idx ,i;
	GW_CALL calltype = GW_CALL_BLOCKING;
	BondRec_t * bonds;
	bonds = (BondRec_t *)blefiCfgRec.bonds;

	for(idx=0;idx<GAP_BONDINGS_MAX;idx++)
	{
		if(bonds[idx].link_connect == TRUE)
		{

		    for (i=0; i<get_gwDevCnt(); i++)
		    {
		    	//if the device gets connected, deviceRec gets malloced
		    	if(gw_context.scanRespArray[i]->deviceRec == NULL &&
		    			(memcmp(gw_context.scanRespArray[i]->addr ,bonds[idx].bond_rec.publicAddr, B_ADDR_LEN ) == 0))
		    	{
		    		ucScanIdx = i;
		    	}
		    }

			bonds[idx].link_connect = FALSE;
			gapBondMgrFlashBondRecUpdate();

			GC_LinkEstablish(ucScanIdx , calltype, bonds[idx].bond_rec.publicAddr);

	        UART_PRINT("\n\r Connecting...");
		}
	}
}



/*********************************************************************
 * @fn      GC_Scan
 *
 * @brief
 *
 * @param
 *
 * @return  none
 */
//Returns number of devices discovered
int GC_Scan(GW_CALL calltype)
{
	param_Scan_t *pScan ;
	OsiReturnVal_e eRetVal;

	eRetVal = osi_LockObjLock(&GC_ApiLockObj, OSI_NO_WAIT);
	{
		if(eRetVal != OSI_OK)
		{
			/*
			 * Connection APIs are being used			 *
			 * */
			return(-1);
		}
	}

	//Check if the device discovery is already in progress,
	//Return -1 if device discovery is in progress
	if(get_gwState() == GW_DISC_IN_PROGRESS)
	{
		osi_LockObjUnlock(&GC_ApiLockObj);
		return (-1);
	}

	set_gwCallType(DEVICE_SCAN,calltype);

	pScan = malloc(sizeof(param_Scan_t));
	if(pScan == NULL)
	{
		UART_PRINT("\n\r[GW] Error - not enough memory for pScan ...");
		osi_LockObjUnlock(&GC_ApiLockObj);
		HandleError(__FILE__,__LINE__,1); // Fatal error

		// Control should never reach here
		return (1);
	}

	pScan->InputCmd.Cmd = DEVICE_SCAN;

	//post App message
	sMsg.pData = pScan ;

	//post in App message Queue
    osi_MsgQWrite(&sGC_AppMsgQueue,&sMsg,OSI_NO_WAIT);

    if(calltype == GW_CALL_BLOCKING)
    {
    	//Wait until Device discovery is done.
    	osi_SyncObjWait(&gw_context.gw_GapSyncObj,OSI_WAIT_FOREVER);
    }
    osi_LockObjUnlock(&GC_ApiLockObj);
	if(*(blefiCfgRec.autocon) ==1)
	{
		IfBondingEnabled();
	}

    return(get_gwDevCnt());
}



void GC_DeviceScanList(fptrDevlist_t fptr)
{
	int i;

    for (i=0; i<get_gwDevCnt(); i++)
    {
    	//if the device gets connected, deviceRec gets malloced
    	if(gw_context.scanRespArray[i]->deviceRec == NULL)
    	{
			if(gw_context.scanRespArray[i]->deviceRec->dev_state != DEV_READY)
			{
				(*fptr)(i,gw_context.scanRespArray[i]->deviceName,
						  gw_context.scanRespArray[i]->addr,
						  gw_context.scanRespArray[i]->addrType);
			}
    	}
    }
}


void GC_DeviceConnList(fptrDevlist_t fptr)
{
	int i;

    for (i=0; i<MAX_DEVICE_DB_SUPPORTED; i++)
    {
    	if(DeviceRec[i]->dev_state == DEV_READY)
    	{
    		(*fptr)(i,DeviceRec[i]->devicename, DeviceRec[i]->addr, 0);
    	}
    }
}



unsigned char* GC_DeviceScanListBdAddr(unsigned long scanId)
{
	return ((unsigned char*)gw_context.scanRespArray[scanId]->addr);
}


short int GC_LinkEstablish(unsigned int ucScanIdx ,GW_CALL calltype,unsigned char* BdAddr)
{
	param_LinkEstablish_t *pLinkEstablish;
	OsiSyncObj_t * gc_LESemaphore;
	unsigned char* scanlistBdAddr;
	OsiReturnVal_e eRetVal;

	eRetVal = osi_LockObjLock(&GC_ApiLockObj, OSI_NO_WAIT);
	{
		if(eRetVal != OSI_OK)
		{
			/*
			 * Connection APIs are being used			 *
			 * */
			return (-1);
		}
	}

	//Return -1 if device discovery alreay in progress
	if(get_gwState() != GW_DISC_DONE)
	{
		osi_LockObjUnlock(&GC_ApiLockObj);
		return (-1);
	}
	if((get_gwDevCnt()==0)||(ucScanIdx > get_gwDevCnt()))
	{
		osi_LockObjUnlock(&GC_ApiLockObj);
		return (-1);
	}

	// Check if the Scan Id is  valid
	if(gwcontext_IsScanIdValid(ucScanIdx) < 0)
	{
		UART_PRINT("\n\r[GW] Error - Invalid Scan ID");
		osi_LockObjUnlock(&GC_ApiLockObj);
		return (-1);
	}

	set_gwCallType(LINK_ESTABLISH , calltype);
	gc_LESemaphore = gw_createDeviceRec(ucScanIdx);

	if(BdAddr !=NULL)
	{
		scanlistBdAddr = GC_DeviceScanListBdAddr(ucScanIdx);
		memcpy(BdAddr,scanlistBdAddr, B_ADDR_LEN);
	}

	pLinkEstablish = (param_LinkEstablish_t *)malloc(sizeof(param_LinkEstablish_t));
	if(pLinkEstablish == NULL)
	{
		UART_PRINT("\n\r[GW] Error - not enough memory for pLinkEstablish ...");
		osi_LockObjUnlock(&GC_ApiLockObj);
		HandleError(__FILE__,__LINE__,1); // Fatal error;

		// Control should never reach here
		return (-1);
	}
	pLinkEstablish->InputCmd.Cmd = LINK_ESTABLISH;
	pLinkEstablish->DeviceId = ucScanIdx;

	//post App message
	sMsg.pData = pLinkEstablish ;

	//post in App message Queue
    osi_MsgQWrite(&sGC_AppMsgQueue,&sMsg,OSI_NO_WAIT);

    if(get_gwCallType(LINK_ESTABLISH) == GW_CALL_BLOCKING)
    {

        //Wait on the device semaphore
        osi_SyncObjWait(gc_LESemaphore,OSI_WAIT_FOREVER);

        //Check the state of the device for successful connection or not

    }
   osi_LockObjUnlock(&GC_ApiLockObj);
   return(get_gw_devConnectionid(ucScanIdx));
}



short int GC_LinkTerminate(unsigned int ucDeviceId, GW_CALL calltype )
{
	param_LinkTerminate_t *pLinkTerminate;
	OsiReturnVal_e eRetVal;

		eRetVal = osi_LockObjLock(&GC_ApiLockObj, OSI_NO_WAIT);
		{
			if(eRetVal != OSI_OK)
			{
				/*
				 * Connection APIs are being used			 *
				 * */
				return(-1);
			}
		}

	set_gwCallType(LINK_TERMINATE , calltype);


	pLinkTerminate = malloc(sizeof(param_LinkTerminate_t));
	if(pLinkTerminate == NULL)
	{
		UART_PRINT("\n\r[GW] Error - not enough memory for pLinkTerminate ...");
		osi_LockObjUnlock(&GC_ApiLockObj);
		HandleError(__FILE__,__LINE__,1); // Fatal error;

		// Control should never reach here
		return(-1) ;
	}

	// Check if the Scan Id is  valid
	if(DB_IsConnIdValid(ucDeviceId) < 0)
	{
		UART_PRINT("\n\r[GW] Error - Invalid Connection ID");
		free(pLinkTerminate);
		osi_LockObjUnlock(&GC_ApiLockObj);
		return (-1);
	}

	pLinkTerminate->InputCmd.Cmd = LINK_TERMINATE;
	pLinkTerminate->DeviceId = ucDeviceId;

	//post App message
	sMsg.pData = pLinkTerminate ;

	//post in App message Queue
    osi_MsgQWrite(&sGC_AppMsgQueue,&sMsg,OSI_NO_WAIT);


#if 0
    if(get_gwCallType(LINK_TERMINATE) == GW_CALL_BLOCKING)
    {

    }
#endif
    osi_LockObjUnlock(&GC_ApiLockObj);
    return 0;
}


void GC_GATT_DiscAllPrimaryServices(unsigned short int connhandle)
{
	param_GattDiscAllPrimaryServices_t *pGattDiscAllPrimaryServices;

	pGattDiscAllPrimaryServices = malloc(sizeof(param_GattDiscAllPrimaryServices_t));
	if(pGattDiscAllPrimaryServices == NULL)
	{
		UART_PRINT("\n\r[GW] Error - not enough memory for pGattDiscAllPrimaryServices ...");
		HandleError(__FILE__,__LINE__,1); // Fatal error;

        // Control should never reach here
        return ;
	}

	pGattDiscAllPrimaryServices->InputCmd.Cmd = GATT_DISC_ALLPRIMARYSERVICES;
	pGattDiscAllPrimaryServices->connHandle   = connhandle;

	//post App message
	sMsg.pData = pGattDiscAllPrimaryServices ;

	//post in App message Queue
    osi_MsgQWrite(&sGC_AppMsgQueue,&sMsg,OSI_NO_WAIT);

 }




void GC_GATT_DiscPrimaryServiceByUUID(unsigned short int connhandle,unsigned char *pServUuid, unsigned short int* pServUuidlen , handle_range_t* pServHndlRng)
{
	param_GattDiscPrimaryServiceByUUID_t *pGattDiscPrimaryServiceByUUID;

	pGattDiscPrimaryServiceByUUID = malloc(sizeof(param_GattDiscPrimaryServiceByUUID_t));
	if(pGattDiscPrimaryServiceByUUID == NULL)
	{
		UART_PRINT("\n\r[GW] Error - not enough memory for pGattDiscPrimaryServiceByUUID ...");
		HandleError(__FILE__,__LINE__,1); // Fatal error;

		// Control should never reach here
		return ;
	}
	pGattDiscPrimaryServiceByUUID->InputCmd.Cmd = GATT_DISC_PRIMARYSERVICEBYUUID;
	pGattDiscPrimaryServiceByUUID->connHandle   = connhandle;
	pGattDiscPrimaryServiceByUUID->pServUuid    = pServUuid;
	pGattDiscPrimaryServiceByUUID->pServUuidlen = pServUuidlen;
	pGattDiscPrimaryServiceByUUID->pServHndlRng = pServHndlRng;

	//post App message
	sMsg.pData = pGattDiscPrimaryServiceByUUID ;

	//post in App message Queue
    osi_MsgQWrite(&sGC_AppMsgQueue,&sMsg,OSI_NO_WAIT);

}

void GC_GATT_DiscAllChars(unsigned short int connhandle, unsigned short int StartHndl, unsigned short int EndHndl)
{
	param_GATT_DiscAllChars_t *pGattDiscAllChars;

	pGattDiscAllChars = malloc(sizeof(param_GATT_DiscAllChars_t));
	if(pGattDiscAllChars == NULL)
	{
		UART_PRINT("\n\r[GW] Error - not enough memory for pGattDiscAllChars ...");
		HandleError(__FILE__,__LINE__,1); // Fatal error;

        // Control should never reach here
        return ;
	}

	pGattDiscAllChars->InputCmd.Cmd 	= GATT_DISC_ALLCHARS;
	pGattDiscAllChars->connHandle   	= connhandle;
	pGattDiscAllChars->startHandle  	= StartHndl;
	pGattDiscAllChars->endHandle   		= EndHndl;

	//post App message
	sMsg.pData = pGattDiscAllChars ;

	//post in App message Queue
    osi_MsgQWrite(&sGC_AppMsgQueue,&sMsg,OSI_NO_WAIT);

}


ErrorFlag_t GC_GetSet(unsigned short int connhandle ,char* pInputString , GC_GETSETFLAG get_set_flag ,unsigned char WriteValueLen ,unsigned char* pWriteValueBuf)
{
	return (DB_GetSetChar(connhandle, pInputString,  get_set_flag, WriteValueLen, pWriteValueBuf));
}

extern void DB_GetDevCharList(unsigned short int connhandle , fptrDevCharlist_t fptr);
void GC_GetDevCharList(unsigned short int connhandle , fptrDevCharlist_t fptr)
{
	if(connhandle>=MAX_DEVICE_DB_SUPPORTED)
	{
		return;
	}
	DB_GetDevCharList(connhandle, fptr);
}


void GC_EraseAllBondings(void)
{
	gapBondMgrEraseAllBondings();
}

bool GC_IsBLEReady()
{
	return(is_bleStackReady);
}
