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

/************
 * INCLUDES
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "datatypes.h"
#include "osi.h"
#include "gatt.h"
#include "hci.h"
#include "gateway_api.h"
#include "database.h"
#include "gateway_task.h"
#include "gateway_cmds.h"
#include "gapbondmgr.h"

extern blefiCfg_t blefiCfgRec;
/************
 * GLOBAL VARIABLES
 */

extern  gw_context_t gw_context;
extern  uint8  gapCentralRoleIRK[KEYLEN];
extern  uint8  gapCentralRoleSRK[KEYLEN];
extern  uint32 gapCentralRoleSignCounter;
extern  uint8  gapCentralRoleBdAddr[B_ADDR_LEN];
extern  uint8  gapCentralRoleMaxScanRes;

extern void TurnOffLed(unsigned char LedInstance);
extern void UpdateBlefiCfg(void);
extern uint8 gapBondMgrFindEmpty( void );

//#define DEBUG_ENABLED
/*********************************************************************
 * @fn      copydevicename
 *
 * @brief
 *
 * @param
 *
 * @return  none
 */
char* CopyDeviceName(unsigned char* pEvtData,unsigned char datalen)
{
	int i, flag, length;
	char* deviceName;
	for(i=0;i<datalen-1;)
	{
		//i will point to the length
		//i+1 will point to the FLAG
		flag = *(pEvtData+i+1);
		length = *(pEvtData+i); // equals strlen plus 1(flaglen)
		if(flag==GAP_ADTYPE_LOCAL_NAME_COMPLETE)
		{
			deviceName = (char *)malloc(length);
			if(deviceName==NULL)
			{
				LOOP_ON_ERROR(-1);
			}
			memcpy(deviceName,(pEvtData+i+2),(length-1));
			deviceName[length-1]='\0';
			return deviceName;
		}
		i=i+(*(pEvtData+i));

	}
	return NULL;
}



/*********************************************************************
 * @fn      GatewayCentral_processRoleEvent
 *
 * @brief   Gateway Central role event processing function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
unsigned char GatewayCentral_processRoleEvent(gapCentralRoleEvent_t **pEventPtr)
{
	gapCentralRoleEvent_t *pEvent  = *pEventPtr;
	BondRec_t * bonds;
	bonds = (BondRec_t *)blefiCfgRec.bonds;

	switch (pEvent->gap.opcode)
	{
		case GAP_DEVICE_INIT_DONE_EVENT:
		  {

			if (pEvent->initDone.status == SUCCESS)
			{
				memcpy(gapCentralRoleBdAddr, pEvent->initDone.devAddr, B_ADDR_LEN);
				memcpy(gapCentralRoleIRK, pEvent->initDone.IRK, 16 );
				memcpy(gapCentralRoleSRK, pEvent->initDone.CSRK, 16);
				Message("\n\r[GW] BLE Stack Initialized \n\r");

			}
			else
			{
				Message("\n\r[GW] ERROR - BLE Stack Not Initialized \n\r");
				return FAILURE;
			}
		  }
		  break;

		case GAP_DEVICE_INFO_EVENT:
		  {
			  if ((pEvent->deviceInfo.status == SUCCESS)&&(pEvent->deviceInfo.eventType==GAP_ADRPT_SCAN_RSP))
			  {
				  if(gw_context.scanCnt < MAX_SCAN_RESPONSE)
				  {
					  uint8 idx;                          // NV Index
					  uint8 publicAddr[B_ADDR_LEN] = {0, 0, 0, 0, 0, 0};      // Place to put the public address
					  scanResponse_t* pScanRespArray ;
					  pScanRespArray =(scanResponse_t*) malloc(sizeof(scanResponse_t));

					  if( pScanRespArray == NULL)
					  {
						LOOP_ON_ERROR(-1);
					  }

					  //Initialising the scanresponse Array
					  pScanRespArray->addrType = pEvent->deviceInfo.addrType;
					  memcpy(pScanRespArray->addr,pEvent->deviceInfo.addr,B_ADDR_LEN);
					  pScanRespArray->rssi = pEvent->deviceInfo.rssi;
					  pScanRespArray->deviceName = CopyDeviceName((unsigned char*)(&(pEvent->deviceInfo.pEvtData)),pEvent->deviceInfo.dataLen);
					  pScanRespArray->deviceRec = NULL;

					  idx = GAPBondMgr_ResolveAddr(  pScanRespArray->addrType, pScanRespArray->addr, publicAddr );


					  if(idx < GAP_BONDINGS_MAX )
					  {
						  bonds[idx].link_connect = TRUE;
						  UpdateBlefiCfg();
					  }

					  gw_context.scanRespArray[gw_context.scanCnt] = pScanRespArray;

					  // Increment the count  - No of devices scanned
					  gw_context.scanCnt++;

					  // Prepare the Call back for App. GW_DEVICE_FOUND
					  gwScanRspEvent_t gwScanRspEvent;
					  gwScanRspEvent.scanIdx = gw_context.scanCnt;
					  gwScanRspEvent.addrType = pEvent->deviceInfo.addrType;
					  memcpy(gwScanRspEvent.addr,pEvent->deviceInfo.addr,B_ADDR_LEN);
					  gwScanRspEvent.scanIdx = gw_context.scanCnt;
					  gwScanRspEvent.rssi = pEvent->deviceInfo.rssi;
					  gwScanRspEvent.deviceName = CopyDeviceName((unsigned char *)(&(pEvent->deviceInfo.pEvtData)),pEvent->deviceInfo.dataLen);
					  CallGwEventCB(GW_EVENT_SCANRSP , &gwScanRspEvent );
				  }
			 }
		  }
		 break;

		case GAP_DEVICE_DISCOVERY_EVENT:
		  {
			  if (pEvent->discCmpl.status == SUCCESS)
			  {
				   // uint8 idx = 0;
					// discovery complete
					set_gwState(GW_DISC_DONE);

					//Callback here GW_DISC_DONE
					gwScanComplEvent_t gwScanComplEvent;
					gwScanComplEvent.status = SUCCESS;

					CallGwEventCB(GW_EVENT_SCANCOMPLETE , &gwScanComplEvent );

					if(get_gwCallType(DEVICE_SCAN) == GW_CALL_BLOCKING)
					{
						//Signal Semaphore - Do we need this?
						osi_SyncObjSignal(&gw_context.gw_GapSyncObj);
					}

                    TurnOffLed(LED_2);

			  }
			  else if (pEvent->discCmpl.status == bleGAPUserCanceled)
			  {
					/*
					 * Discovery cancelled - Go to clean state
					 */
					set_gwState(GW_INIT_DONE);
			  }
		  }
		  break;

		case GAP_LINK_ESTABLISHED_EVENT:
		  {
			if (pEvent->gap.status == SUCCESS)
			{
				unsigned char ScanId;
				device_db_t * ptrDevRecMatch  =  NULL;
				unsigned short int connhandle;

				connhandle = pEvent->linkCmpl.connectionHandle;
#if AUTH_ENABLE
				// Notify the Bond Manager of the connection
				GAPBondMgr_LinkEst(pEvent->linkCmpl.devAddrType, pEvent->linkCmpl.devAddr,
						pEvent->linkCmpl.connectionHandle, GAP_PROFILE_CENTRAL);
#else
				uint8 idx = 0;
				idx = gapBondMgrFindBondEntry(pEvent->linkCmpl.devAddr);
				if(idx>=GAP_BONDINGS_MAX)
				{

					idx = gapBondMgrFindEmpty();
					Report("\n\rSaving device info for auto connection %d-[0x%02x%02x%02x%02x%02x%02x]",idx,
						pEvent->linkCmpl.devAddr[5],pEvent->linkCmpl.devAddr[4],pEvent->linkCmpl.devAddr[3],
						pEvent->linkCmpl.devAddr[2],pEvent->linkCmpl.devAddr[1],pEvent->linkCmpl.devAddr[0]);
					if(idx<GAP_BONDINGS_MAX)
					{
						memcpy( &(bonds[idx].bond_rec.publicAddr[0]), &(pEvent->linkCmpl.devAddr), B_ADDR_LEN );
						UpdateBlefiCfg();
					}
				}
#endif

				//Check pEvent->linkCmpl.devAddr in scanList
				//Get the appropriate DevRec and hook it to the DB
				//Dont give the semaphore, its only done after building the database
				for(ScanId =0 ; ScanId <  gw_context.scanCnt   ; ScanId++)
				{
				  if(memcmp(gw_context.scanRespArray[ScanId]->addr , pEvent->linkCmpl.devAddr , B_ADDR_LEN) == 0)
				  {
					  ptrDevRecMatch = gw_context.scanRespArray[ScanId]->deviceRec;
					  break;
				  }
				}

				//Link the record to the Device database
				DB_LinkDevRec(connhandle , ptrDevRecMatch);

				// Needs to be set once the record to the Device database is linked
				set_devState(connhandle,DEV_CONNECTED);

				DB_Build(connhandle);
			}
			else
			{
				DBG_PRINT("\n\r Connect Failed \n\r");
				DBG_PRINT("Reason: %d", pEvent->gap.status);
			}
		  }
		  break;

		case GAP_LINK_TERMINATED_EVENT:
		{
			set_devState(pEvent->linkTerminate.connectionHandle, DEV_DISCONNECTED);

			// Prepare the Call back for App. GW_DEVICE_FOUND
			gwLinkTermEvent_t gwLinkTermEvent;
			gwLinkTermEvent.connhandle = pEvent->linkTerminate.connectionHandle;
			gwLinkTermEvent.reason = pEvent->linkTerminate.reason;

			memcpy(gwLinkTermEvent.bdAddr, (const void*)DB_getDevAddrPtr(gwLinkTermEvent.connhandle), B_ADDR_LEN );

			gwLinkTermEvent.deviceName = (char*) malloc(strlen((const char*)(DB_getDevNamePtr(gwLinkTermEvent.connhandle))) + 1);
			if(gwLinkTermEvent.deviceName == NULL)
			{
				DBG_PRINT("\r\n[GW] Error - Not enough memory for gwLinkTermEvent.deviceName");
				HandleError(__FILE__,__LINE__,1); // fatal error
				break;
			}
			strcpy((char *)gwLinkTermEvent.deviceName ,(const char *) DB_getDevNamePtr(gwLinkTermEvent.connhandle));

			DB_deleteDevRec(pEvent->linkTerminate.connectionHandle);
			CallGwEventCB(GW_EVENT_LINKTERMINATE , &gwLinkTermEvent );
			free(gwLinkTermEvent.deviceName);

			GAPBondMgr_LinkTerm(pEvent->linkTerminate.connectionHandle);
		}
		  break;

		case GAP_LINK_PARAM_UPDATE_EVENT:
		  {


		  }
		  break;

		case GAP_PASSKEY_NEEDED_EVENT:
		{
#if AUTH_ENABLE
		  volatile gapPasskeyNeededEvent_t *pPkt = (gapPasskeyNeededEvent_t *)(&(pEvent->passKeyNeeded));
#endif

		}
		break;

	  case GAP_AUTHENTICATION_COMPLETE_EVENT:
		{
#if AUTH_ENABLE
		  gapAuthCompleteEvent_t *pPkt = (gapAuthCompleteEvent_t *)(pEvent);

		  // Should we save bonding information (one save at a time)
		  if ( (pPkt->status == SUCCESS) && (pPkt->authState & SM_AUTH_STATE_BONDING))
		  {
			uint8 idx = 0;

			idx = gapBondMgrFindBondEntry(pPkt->connectionHandle);


			DBG_PRINT("\r\n Authentication successful");
			DBG_PRINT("\r\n id = %d",idx);

			if(idx < GAP_BONDINGS_MAX)
			{
				// Do we have a public address in the data?
				if ( pPkt->pIdentityInfo.idInfoEnable )
				{
				  memcpy( &(bonds[idx].bond_rec.publicAddr[0]), &(pPkt->pIdentityInfo.idInfo.bd_addr[0]), B_ADDR_LEN );
				  UpdateBlefiCfg();
				}
				else
				{
					// We don't have an address, so ignore the message.
					break;
				}

				// Save off of the authentication state
				bonds[idx].bond_rec.stateFlags |= (pPkt->authState & SM_AUTH_STATE_AUTHENTICATED) ? GAP_BONDED_STATE_AUTHENTICATED : 0;
				gapBondMgrAddBond((void *) &(bonds[idx]), pPkt );
				UpdateBlefiCfg();
			}
		  }
#endif
		}
		break;

	  case GAP_BOND_COMPLETE_EVENT:

		{
#if AUTH_ENABLE
		// This message is received when the bonding is complete.  If hdr.status is SUCCESS
		// then call app state callback.  If hdr.status is NOT SUCCESS, the connection will be
		// dropped at the LL because of a MIC failure, so again nothing to do.
		  volatile gapBondCompleteEvent_t *pPkt = (gapBondCompleteEvent_t *)(&(pEvent->bondComplete));
#endif

    	}
		break;

		default:
		  break;
	  }

	return 0;
}


/*********************************************************************
 * @fn      GatewayCentral_processGattEvent
 *
 * @brief   Gateway Central role ATT event processing function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */

void GatewayCentral_processGattEvent(gattMsg_t  **pEventPtr)
{

	gattMsg_t *pEvent  = *pEventPtr;


	  switch (pEvent->gatt.opcode)
	  {

	    case ATT_READ_BLOB_RSP:
	    {

	    	if(pEvent->readBlobRsp.status == SUCCESS)
	    	{
				DBG_PRINT("\r\n");

	    	}
	    	else if(pEvent->readBlobRsp.status == bleProcedureComplete) // bleProcedureComplete = 0x1A
	    	{
                     // Procedure complete
	    	}
	    	else
	    	{
		        DBG_PRINT("\n\r error rsp");
	            DBG_PRINT("\r\n");
	    	}

	    }
	    break;

	    case ATT_HANDLE_VALUE_NOTI:
	    {
		      if (pEvent->handleValueNoti.status == SUCCESS)
		      {
		    	 unsigned char* pValue = pEvent->handleValueNoti.value ;
		    	  unsigned short int charValHandle = pEvent->handleValueNoti.handle;

		    		// Call back to App
					gwGetData_t gwGetData;
					gwGetData.connhandle = pEvent->handleValueNoti.connhandle;
					gwGetData.ValueLen = pEvent->handleValueNoti.pdulen - 2;
					gwGetData.pValueBuffer = (unsigned char *)malloc(pEvent->readRsp.pdulen - 2 );
					if(gwGetData.pValueBuffer==NULL)
					{
						HandleError(__FILE__,__LINE__,1);
						return;
					}
					memcpy(gwGetData.pValueBuffer , pValue , (pEvent->readRsp.pdulen - 2));
					gwGetData.pCurrentGetCharName = NULL ; // initialise it to NULL
					DB_FindCharNameString(gwGetData.connhandle ,charValHandle , &gwGetData.pCurrentGetCharName);
		  		    CallGwDataCB(GW_DATA_GET , NO_ERROR,  &gwGetData );
		  		    if(gwGetData.pCurrentGetCharName != NULL)
		  		    {
		  		    	free(gwGetData.pCurrentGetCharName);
		  		    	gwGetData.pCurrentGetCharName = NULL;
		  		    }
				    free(gwGetData.pValueBuffer);
				    gwGetData.pValueBuffer = NULL;
		      }

	    }
	    break;


	    case ATT_FIND_BY_TYPE_VALUE_RSP:
	    {
	      if (pEvent->findByTypeValueRsp.status == SUCCESS)
	      {

	      }
	      else if (pEvent->findByTypeValueRsp.status == bleProcedureComplete)
	      {

	      }
	    }
	    break;

	    case ATT_READ_BY_GRP_TYPE_RSP:
	    {
	      if (pEvent->readByGrpTypeRsp.status == SUCCESS)
	      {
	    	  	unsigned char i;
	    	   unsigned char dataLen  = pEvent->readByGrpTypeRsp.pdulen - 1;
	    	    unsigned short int connhandle = pEvent->readByGrpTypeRsp.connhandle;
	    	   unsigned char numgrps  = dataLen /  pEvent->readByGrpTypeRsp.len;
	    	   unsigned char ValueLen = pEvent->readByGrpTypeRsp.len - 4;

	    	    for ( i=0 ; i<numgrps; i++)
	    	    {
	    	    	// copy values inside , msg q will be freed
	    	    	unsigned short int *pStartHndl = (unsigned short int*) ((unsigned char*)pEvent->readByGrpTypeRsp.dataList + (pEvent->readByGrpTypeRsp.len * i));
	    	    	unsigned short int *pEndHndl   = (unsigned short int*) ((unsigned char*)pEvent->readByGrpTypeRsp.dataList + (pEvent->readByGrpTypeRsp.len * i) + 2);
	    	    	unsigned char  *pValue     = (unsigned char*)  ((unsigned char*)pEvent->readByGrpTypeRsp.dataList + (pEvent->readByGrpTypeRsp.len * i) + 4);
	    	    	DB_BuildPrimServTable(connhandle, pStartHndl, pEndHndl , ValueLen , pValue );
	    	    }
	      }
	      else if (pEvent->readByGrpTypeRsp.status == bleProcedureComplete )
	      {
			unsigned char connhandle = pEvent->readByGrpTypeRsp.connhandle;
#ifdef DEBUG_ENABLED
			DB_PrintPrimServTable(connhandle);
#endif
			set_devState(connhandle,DEV_SVC_DONE);
			DB_DiscAllCharinService(connhandle);
	      }

	    }
	    break;

	    case ATT_READ_BY_TYPE_RSP:
	    {
	      if (pEvent->readByTypeRsp.status == SUCCESS)
	      {
				// Find Char response
				unsigned char i;
				unsigned char dataLen  = pEvent->readByTypeRsp.pdulen - 1;
				unsigned char connhandle = pEvent->readByTypeRsp.connhandle;
				unsigned char numgrps  = dataLen /  pEvent->readByTypeRsp.len;
				unsigned char ValueLen = pEvent->readByTypeRsp.len - 2;

				for ( i=0 ; i<numgrps; i++)
				{
					// copy values inside , msg q will be freed
					unsigned short int *pHndl = (unsigned short int*) ((unsigned char*)pEvent->readByTypeRsp.dataList + (pEvent->readByTypeRsp.len * i));
					unsigned char  *pValue     = (unsigned char*)  ((unsigned char*)pEvent->readByTypeRsp.dataList + (pEvent->readByTypeRsp.len * i) + 2);
					DB_BuildCharTable(connhandle, pHndl, ValueLen , pValue );
				}
	      }
	      else if (pEvent->readByTypeRsp.status == bleProcedureComplete )
	      {
		    	  unsigned char connhandle = pEvent->readByTypeRsp.connhandle;

		    	   DB_DiscAllCharinService(connhandle);  // continue until  char in every service is discovered

             	  //set gw_context.deviceRec??
	      }

	    }
	    break;


	    case ATT_READ_RSP:
	    {
	    	if(pEvent->readRsp.status == SUCCESS)
	    	{
				unsigned char i;
				unsigned char* pValue = ((unsigned char*)(pEvent->readRsp.value));

	    		// Call back to App
				gwGetData_t gwGetData;
				gwGetData.connhandle = pEvent->readRsp.connhandle;
				gwGetData.ValueLen = pEvent->readRsp.pdulen;
				gwGetData.pValueBuffer =(unsigned char *) malloc(pEvent->readRsp.pdulen);
				if(gwGetData.pValueBuffer==NULL)
				{
					HandleError(__FILE__,__LINE__,1);
					return;
				}
				memcpy(gwGetData.pValueBuffer , pValue ,pEvent->readRsp.pdulen  );
				DB_FillCurrentGetCharName(gwGetData.connhandle,&gwGetData.pCurrentGetCharName);
				CallGwDataCB(GW_DATA_GET , NO_ERROR,  &gwGetData );
				free(gwGetData.pValueBuffer);

				// Print the received value
				DBG_PRINT("\n\r The value =");
				for(i =0 ; i<pEvent->readRsp.pdulen ;i++)
				{
					DBG_PRINT(" 0x%x ",	*(pValue + i));
				}
				DBG_PRINT("\r\n");
	    	}
	    	else if(pEvent->readRsp.status == bleProcedureComplete) // bleProcedureComplete = 0x1A
	    	{
                // Procedure complete
	    	}
	    	else
	    	{
		        DBG_PRINT("\n\r error rsp");
	            DBG_PRINT("\r\n");
	    	}
	    }
	    break;

	    case ATT_FIND_INFO_RSP:
	    {

	    	if(pEvent->findInfoRsp.status == SUCCESS)
	    	{
				unsigned char i , len = 0xFF;
				unsigned char format =  pEvent->findInfoRsp.format ;
				if(format == 1)
				{
					len = 4;
				}
				else if (format == 2)
				{
					len = 18;
				}
				unsigned char dataLen  = pEvent->findInfoRsp.pdulen - 1;
				unsigned char connhandle = pEvent->findInfoRsp.connhandle;
				unsigned char numgrps  = dataLen /  len;
				unsigned char UuidLen = len - 2;

				for ( i=0 ; i<numgrps; i++)
				{
					// pdb_engine_build();
					// copy values inside , msg q will be freed
					unsigned short int *pHndl = (unsigned short int*) ((unsigned char*)pEvent->findInfoRsp.dataList + (len * i));
					unsigned char  *pUuid     = (unsigned char*)  ((unsigned char*)pEvent->findInfoRsp.dataList + (len * i) + 2);
					DB_BuildCharDescsTable(connhandle, pHndl, UuidLen , pUuid );
				}

	    	}
	    	else if(pEvent->findInfoRsp.status == bleProcedureComplete) // bleProcedureComplete = 0x1A
	    	{
				unsigned char connhandle = pEvent->findInfoRsp.connhandle;
				set_devState(connhandle,DEV_READY);

#ifdef DEBUG_ENABLED
				DB_PrintCharDescsTable(connhandle);
#endif
	    	}
	    	else
	    	{
		        DBG_PRINT("\n\r error rsp");
	            DBG_PRINT("\r\n");
	    	}
	    }
	    break;

	    case ATT_WRITE_RSP:
	    {
	    	if(pEvent->writeRsp.status == SUCCESS)
	    	{
	    		gwSetData_t gwSetData;
	    		gwSetData.connhandle = pEvent->writeRsp.connhandle;
				CallGwDataCB(GW_DATA_SET , NO_ERROR,  &gwSetData );
				Message("\n\r Write Success");
				Message("\r\n");

	    	}
	    	else if(pEvent->writeRsp.status == bleProcedureComplete) // bleProcedureComplete = 0x1A
	    	{
                     // Procedure complete
	    	}
	    	else
	    	{
		        DBG_PRINT("\n\r error rsp");
	            DBG_PRINT("\r\n");
	    	}
	    }
	    break;

	    case ATT_ERROR_RSP:
	    {
	        DBG_PRINT("\n\r Error Received  \n\r");
            DBG_PRINT("\r\n");
	    }
	    break;

	    default:
	    {
	      // Do nothing.
	    }
	    break;
	  }
}


/*********************************************************************
 * @fn      printScanResponse
 *
 * @brief
 *
 * @param   numDevices
 *
 * @return  none
 */
void printScanResponse(unsigned char numDevices)
{
	unsigned char count;
	for(count=0; count<numDevices; count++)
	{
		DBG_PRINT("\r\n Device ID : %d", count);
		DBG_PRINT("\r\n addr : ");
		DBG_PRINT(Util_convertBdAddr2Str(gw_context.scanRespArray[count]->addr));
		DBG_PRINT("\r\n");
	}
}

