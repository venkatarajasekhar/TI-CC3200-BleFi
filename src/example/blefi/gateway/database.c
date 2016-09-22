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

/*********************************************************************
 * INCLUDES
 */
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "datatypes.h"

#include "schema.h"
#include "gateway_api.h"
#include "database.h"
#include "uart_if.h"
#include "gateway_task.h"
#include "gateway_cmds.h"
#include "gatt_uuid.h"
#include "osi.h"
#include "string.h"

/*********************************************************************
* MACROS
*/

//#define DEBUG_ENABLED

/*********************************************************************
* CONSTANTS
*/

/*********************************************************************
* DEFINES
*/
#define DB_PRINT Report


/*********************************************************************
* GLOBAL VARIABLES
*/

//Array of DeviceDB addresses
//Need to Malloc it to MAX_DEVICE_DB_SUPPORTED
device_db_t * DeviceRec[MAX_DEVICE_DB_SUPPORTED];

#define CHAR_STRING "char"

extern gw_context_t gw_context;


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn       DB_IsConnhandleValid
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */
unsigned char DB_IsConnhandleValid(unsigned short int connhandle)
{
	if(DeviceRec[connhandle] != NULL)
	{
		return CONNHNDL_VALID;
	}
	else
	{
		return CONNHNDL_INVALID;
	}
}


/*********************************************************************
 * @fn      set_gwState
 *
 * @brief Return updated state, or the previous state if the 'state' is wrong.
 *
 * @return  none
 */
unsigned char  set_devState(unsigned short int connhandle,DEV_STATE state)
{
	if(state<=DEV_DISCONNECTED)
	{
		DeviceRec[connhandle]->dev_state=state;
	}
	return(DeviceRec[connhandle]->dev_state);
}


/*********************************************************************
 * @fn      get_devState
 *
 * @brief
 *
 * @return  none
 */
unsigned char  get_devState(unsigned short int connhandle)
{
	return(DeviceRec[connhandle]->dev_state);
}


/*********************************************************************
 * @fn       DB_DevRecInit
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */
void DB_DevRecInit(device_db_t * ptrtempDevRec,unsigned char ScanIdx )
{
	int retvalue_SyncObj, i;

	ptrtempDevRec->connHandle 		= 0xFFFF;
	ptrtempDevRec->dev_state 		= DEV_DISCOVERED;
	ptrtempDevRec->pProfileHandle 	= NULL;

	memcpy(ptrtempDevRec->addr,gw_context.scanRespArray[ScanIdx]->addr,sizeof(ptrtempDevRec->addr));
	ptrtempDevRec->devicename = malloc(strlen((const char*)gw_context.scanRespArray[ScanIdx]->deviceName)+1);
	if(ptrtempDevRec->devicename == NULL)
	{
		DB_PRINT("\n\r [DB] Error - Not enough memory for device name \n");
		return;
	}
	strcpy((char*)ptrtempDevRec->devicename,(const char*)gw_context.scanRespArray[ScanIdx]->deviceName);

	ptrtempDevRec->char_idx = 0;
	ptrtempDevRec->pCurrentGetCharName = NULL;
	ptrtempDevRec->pDevPrimServiceTable = malloc(sizeof(DevPrimServiceTable_t));
	if(ptrtempDevRec->pDevPrimServiceTable == NULL)
	{
		DB_PRINT("\n\r [DB] Error - Not enough memory for Primary Service table \n");
		return;
	}
	ptrtempDevRec->pDevPrimServiceTable->prim_serv_count = 0;
	ptrtempDevRec->pDevPrimServiceTable->prim_serv_idx 	 = 0;

	ptrtempDevRec->CharDescsTable = NULL;

	ptrtempDevRec->db_uuid_info = malloc(sizeof(db_uuid_info_t));
	if(ptrtempDevRec->db_uuid_info == NULL)
	{
		DB_PRINT("\n\r [DB] Error - Not enough memory for uuid_info \n");
		return;
	}
	ptrtempDevRec->db_uuid_info->serv_uuid_len 	= 0;
	ptrtempDevRec->db_uuid_info->serv_uuid 		= NULL;
	ptrtempDevRec->db_uuid_info->char_uuid_len 	= 0;
	ptrtempDevRec->db_uuid_info->char_uuid 		= NULL;

	for(i = 0; i< MAX_CHAR_IN_DEV_SUPPORTED; i++)
	{
		ptrtempDevRec->char_db[i] = NULL;
	}

	if(get_gwCallType(LINK_ESTABLISH) == GW_CALL_BLOCKING)
	{
		//Create semaphore
		ptrtempDevRec->devRecSyncTimeout = 0xFFFFFFFF;
		retvalue_SyncObj = osi_SyncObjCreate(&(ptrtempDevRec->gw_DevRecSyncObj));
		LOOP_ON_ERROR(retvalue_SyncObj);
	}
	else
	{
		// Is this correct?
		ptrtempDevRec->devRecSyncTimeout = 0;
		ptrtempDevRec->gw_DevRecSyncObj = NULL;
	}
}


/*********************************************************************
 * @fn       DB_DevRecDeInit
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */
void DB_DevRecDeInit(device_db_t * ptrtempDevRec)
{
	unsigned char i;
	free(ptrtempDevRec->devicename); // Re-initialise to Null?
	ptrtempDevRec->devicename = NULL;

	if(ptrtempDevRec->pCurrentGetCharName != NULL)
	{
		free(ptrtempDevRec->pCurrentGetCharName); // Re-initialise to Null?
		ptrtempDevRec->pCurrentGetCharName = NULL;
	}

	for(i=0;i<ptrtempDevRec->char_idx;i++)
	{
		free(ptrtempDevRec->char_db[i]);
		ptrtempDevRec->char_db[i] = NULL;
	}

	if(get_gwCallType(LINK_ESTABLISH) == GW_CALL_BLOCKING)
	{
		osi_SyncObjDelete(&(ptrtempDevRec->gw_DevRecSyncObj));
	}

	free(ptrtempDevRec);
	ptrtempDevRec = NULL;
}

/*********************************************************************
 * @fn       DB_deleteDevRec
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */
void DB_deleteDevRec(unsigned short int connhandle)
{
	DB_DevRecDeInit(DeviceRec[connhandle]);
	DeviceRec[connhandle] =NULL;
}

/*********************************************************************
 * @fn       DB_FreePrimserv
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */
void DB_FreePrimserv(device_db_t * ptrtempDevRec)
{
	free(ptrtempDevRec->pDevPrimServiceTable);
	ptrtempDevRec->pDevPrimServiceTable = NULL;
}


/*********************************************************************
 * @fn       DB_GATT_DiscAllPrimaryServices
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */
void DB_GATT_DiscAllPrimaryServices(unsigned short int connhandle)
{
	param_GattDiscAllPrimaryServices_t *pGattDiscAllPrimaryServices;

	pGattDiscAllPrimaryServices = malloc(sizeof(param_GattDiscAllPrimaryServices_t));
	if(pGattDiscAllPrimaryServices == NULL)
	{
		DB_PRINT("\n\r [DB] Error - Not enough memory for pGattDiscAllPrimaryServices \n");
		return;
	}
	pGattDiscAllPrimaryServices->InputCmd.Cmd = GATT_DISC_ALLPRIMARYSERVICES;
	pGattDiscAllPrimaryServices->connHandle   = connhandle;

	process_GattDiscAllPrimaryServices((void **)&pGattDiscAllPrimaryServices);

	free(pGattDiscAllPrimaryServices);
}


/*********************************************************************
 * @fn       DB_FreeCharDescsTable
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */
void DB_FreeCharDescsTable(device_db_t * ptrtempDevRec)
{
	unsigned char i;

	for(i =0 ; i< ptrtempDevRec->CharDescsTable->chardescs_count ;i++)
	{
		if(ptrtempDevRec->CharDescsTable->chardescs_table[i]->uuid !=NULL)
		{
			free(ptrtempDevRec->CharDescsTable->chardescs_table[i]->uuid);
			ptrtempDevRec->CharDescsTable->chardescs_table[i]->uuid = NULL;
		}
       free(ptrtempDevRec->CharDescsTable->chardescs_table[i]);
       ptrtempDevRec->CharDescsTable->chardescs_table[i] = NULL;
	}

	free(ptrtempDevRec->CharDescsTable);
	ptrtempDevRec->CharDescsTable = NULL;
}



/*********************************************************************
 * @fn       DB_FreeDbUuidInfo
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */
void DB_FreeDbUuidInfo(device_db_t * ptrtempDevRec)
{
	free(ptrtempDevRec->db_uuid_info);
	ptrtempDevRec->db_uuid_info = NULL;
}


/*********************************************************************
 * @fn       gw_createDeviceRec
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */
OsiSyncObj_t* gw_createDeviceRec(unsigned char ScanId )
{
	device_db_t * ptrtempDevRec;

	ptrtempDevRec = (device_db_t*)malloc(sizeof(device_db_t));
	if(ptrtempDevRec == NULL)
	{
		DB_PRINT("\n\r [DB] Error - Not enough memory for ptrtempDevRec \n");
		HandleError(__FILE__,__LINE__,1);

		// Control should never reach here
		return 0;
	}
	gw_context.scanRespArray[ScanId]->deviceRec = ptrtempDevRec;

	DB_DevRecInit(ptrtempDevRec,ScanId );

	return (&(ptrtempDevRec->gw_DevRecSyncObj));
}


/*********************************************************************
 * @fn       DB_SyncObjSignal
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */
void DB_SyncObjSignal(unsigned short int connhandle)
{
	GW_CALL call_type = get_gwCallType(LINK_ESTABLISH );

	if(call_type == GW_CALL_BLOCKING)
	{
		DeviceRec[connhandle]->devRecSyncTimeout = 0;
		osi_SyncObjSignal(&DeviceRec[connhandle]->gw_DevRecSyncObj);
	}
}



/*********************************************************************
 * @fn       DB_DevRecReady
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */
void DB_DevRecReady(unsigned short int connhandle )
{
	// Create LinkEvnet to be sent to the App
	gwLinkEstEvent_t gwLinkEstEvent;

	set_devState(connhandle, DEV_READY);
#ifdef DEBUG_ENABLED
	DB_PrintCharTable(connhandle);
#endif
	// Free the Primary Service table of the device . Dev Record is ready
	DB_FreePrimserv(DeviceRec[connhandle]);

	DB_FreeDbUuidInfo(DeviceRec[connhandle]);

	// DB build Complete. Callback to App
	gwLinkEstEvent.connhandle = DeviceRec[connhandle]->connHandle;
	memcpy(gwLinkEstEvent.bdAddr,DeviceRec[connhandle]->addr , B_ADDR_LEN);
	gwLinkEstEvent.deviceName = malloc(strlen((const char*)DeviceRec[connhandle]->devicename) + 1);
	if(gwLinkEstEvent.deviceName == NULL)
	{
		DB_PRINT("\n\r [DB] Error - Not enough memory for gwLinkEstEvent.deviceName \n");
		return;
	}
	strcpy((char*)gwLinkEstEvent.deviceName , (const char *)DeviceRec[connhandle]->devicename );
	CallGwEventCB(GW_EVENT_LINKESTABLISH, &gwLinkEstEvent );
	free(gwLinkEstEvent.deviceName);
	gwLinkEstEvent.deviceName = NULL;

	//signal the Semaphore
	DB_SyncObjSignal(connhandle);
}


/*********************************************************************
 * @fn       DB_DevRecDbInit
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */
void DB_DevRecDbInit( void)
{
	int i;
	for(i=0;i< MAX_DEVICE_DB_SUPPORTED;i++)
	{
		DeviceRec[i] = NULL;
	}
}


/*********************************************************************
 * @fn       DB_GATT_ReadUsingCharUuid
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */
void DB_GATT_ReadUsingCharUuid(unsigned short int connhandle, unsigned short int starthandle , unsigned short int endhandle,  unsigned char char_uuid_len ,unsigned char *char_uuid)
{
	param_GattReadUsingCharUuid_t *pGattReadUsingCharUuid;

	pGattReadUsingCharUuid = malloc(sizeof(param_GattReadUsingCharUuid_t));
	if(pGattReadUsingCharUuid == NULL)
	{
		DB_PRINT("\n\r [DB] Error - Not enough memory for pGattReadUsingCharUuid \n");
		return;
	}
	pGattReadUsingCharUuid->InputCmd.Cmd = GATT_READ_USINGCHARUUID;
	pGattReadUsingCharUuid->connHandle   = connhandle;
	pGattReadUsingCharUuid->starthandle  = starthandle ;
	pGattReadUsingCharUuid->endhandle    = endhandle;
	pGattReadUsingCharUuid->CharUuidlen = char_uuid_len;
	pGattReadUsingCharUuid->pCharUuid = malloc(sizeof(char_uuid_len));
	if(pGattReadUsingCharUuid->pCharUuid == NULL)
	{
		DB_PRINT("\n\r [DB] Error - Not enough memory for pCharUuid \n");
		free(pGattReadUsingCharUuid);
		return;
	}
	memcpy(pGattReadUsingCharUuid->pCharUuid , char_uuid , char_uuid_len);

	process_GattReadUsingCharUuid((void **)&pGattReadUsingCharUuid);

	free(pGattReadUsingCharUuid);
}

/*********************************************************************
 * @fn       DB_GetDeviceName
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */
void  DB_LinkDevRec(unsigned short int connhandle , device_db_t* ptrDeviceRec )
{
	DeviceRec[connhandle] = ptrDeviceRec;
	DeviceRec[connhandle]->connHandle = connhandle;
}


/*********************************************************************
 * @fn       DB_Build
 *
 * @brief   build the UUID-handle database from schema
 *
 * @param   none
 *
 * @return  none
 */
void  DB_Build(unsigned short int connhandle)
{
    volatile unsigned char ProfileFound;

     ProfileFound = Schema_FindProfile(DeviceRec[connhandle]->devicename, DeviceRec[connhandle]->devicename_len,&DeviceRec[connhandle]->pProfileHandle);
	set_devState(connhandle,DEV_SVC_PENDING);
	DB_GATT_DiscAllPrimaryServices(connhandle);
}


/*********************************************************************
 * @fn       DB_BuildPrimServTable
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */
void  DB_BuildPrimServTable(unsigned short int connhandle, unsigned short int* pStartHndl, unsigned short int* pEndHndl ,unsigned char ValueLen ,unsigned char* pValue )
{
	unsigned short int servindex;
	DevPrimServiceTable_t* pDevPrimServiceTable = DeviceRec[connhandle]->pDevPrimServiceTable;

	servindex = DeviceRec[connhandle]->pDevPrimServiceTable->prim_serv_count;

	if(servindex < MAX_PRIM_SERV_SUPPORTED)
	{
		pDevPrimServiceTable->prim_serv_table[servindex].serv_hndl.start_handle = *(pStartHndl);
		pDevPrimServiceTable->prim_serv_table[servindex].serv_hndl.end_handle   = *(pEndHndl);

		pDevPrimServiceTable->prim_serv_table[servindex].uuid_len = ValueLen;
		pDevPrimServiceTable->prim_serv_table[servindex].uuid = malloc(ValueLen);
		if(pDevPrimServiceTable->prim_serv_table[servindex].uuid == NULL)
		{
			DB_PRINT("\n\r [DB] Error - Not enough memory for prim_serv_table[servindex].uuid \n");
			return;
		}

		// copy the uuid
		memcpy((void*)pDevPrimServiceTable->prim_serv_table[servindex].uuid ,
			   (const void*)pValue , ValueLen);

		pDevPrimServiceTable->prim_serv_count++;
	}
	else
	{
		DB_PRINT("\n\r EXCEEDED THE NO OF SERVICES SUPPORTED \n\r");
	}
}


/*********************************************************************
 * @fn       DB_PrintPrimServTable
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */
void  DB_PrintPrimServTable(unsigned short int connhandle)
{
	unsigned char i;

	DevPrimServiceTable_t* pDevPrimServiceTable = DeviceRec[connhandle]->pDevPrimServiceTable;
	DBG_PRINT("\n\r Primary Services of device Id are \n\r " );
	for(i = 0 ; i< pDevPrimServiceTable->prim_serv_count ; i++)
	{
		DBG_PRINT("PS no : %d , start_handle = %2d",i,pDevPrimServiceTable->prim_serv_table[i].serv_hndl.start_handle);
		DBG_PRINT("\n\r");
		DBG_PRINT("PS no : %d , end_handle = %2d",i,pDevPrimServiceTable->prim_serv_table[i].serv_hndl.end_handle);
		DBG_PRINT("\n\n\r");
	}
}


/*********************************************************************
 * @fn       DB_GATT_DiscAllChars
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */
void DB_GATT_DiscAllChars(unsigned short int connhandle, unsigned short int StartHndl, unsigned short int EndHndl)
{
	param_GATT_DiscAllChars_t *pGattDiscAllChars;

	pGattDiscAllChars = malloc(sizeof(param_GATT_DiscAllChars_t));
	if(pGattDiscAllChars == NULL)
	{
		DB_PRINT("\n\r [DB] Error - Not enough memory for pGattDiscAllChars \n");
		return;
	}
	pGattDiscAllChars->InputCmd.Cmd = GATT_DISC_ALLCHARS;
	pGattDiscAllChars->connHandle   = connhandle;
	pGattDiscAllChars->startHandle  = StartHndl;
	pGattDiscAllChars->endHandle    = EndHndl;

	process_GattDiscAllChars((void **)&pGattDiscAllChars);

	free(pGattDiscAllChars);
}

/*********************************************************************
 * @fn       DB_GATT_FindCharDescs(connhandle ,start_handle , end_handle )
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */

void DB_GATT_FindCharDescs(unsigned short int connhandle ,unsigned short int starthandle , unsigned short int endhandle )
{
	param_GATTFindCharDescs_t *pGATTFindCharDescs;

	pGATTFindCharDescs = malloc(sizeof(param_GATTFindCharDescs_t));
	if(pGATTFindCharDescs == NULL)
	{
		DB_PRINT("\n\r [DB] Error - Not enough memory for pGATTFindCharDescs \n");
		return;
	}
	pGATTFindCharDescs->InputCmd.Cmd = GATT_DISC_ALLCHARDESCS;
	pGATTFindCharDescs->connHandle   = connhandle;
	pGATTFindCharDescs->starthandle = starthandle ;
	pGATTFindCharDescs->endhandle = endhandle;

	process_GATTFindCharDescs((void**)&pGATTFindCharDescs);

	free(pGATTFindCharDescs);
}

/*********************************************************************
 * @fn       DB_DiscAllCharinService
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */

void  DB_DiscAllCharinService( unsigned short int connhandle)
{
	 DevPrimServiceTable_t* pDevPrimServiceTable = DeviceRec[connhandle]->pDevPrimServiceTable;

    unsigned char serv_idx = pDevPrimServiceTable->prim_serv_idx;
    unsigned char serv_count = pDevPrimServiceTable->prim_serv_count;

	if( serv_idx < pDevPrimServiceTable->prim_serv_count )
	{
		DeviceRec[connhandle]->db_uuid_info->serv_uuid_len = pDevPrimServiceTable->prim_serv_table[serv_idx].uuid_len;
		DeviceRec[connhandle]->db_uuid_info->serv_uuid     = pDevPrimServiceTable->prim_serv_table[serv_idx].uuid;

		DB_GATT_DiscAllChars(connhandle,
		pDevPrimServiceTable->prim_serv_table[serv_idx].serv_hndl.start_handle,
		pDevPrimServiceTable->prim_serv_table[serv_idx].serv_hndl.end_handle  );
		pDevPrimServiceTable->prim_serv_idx++;
	}
	else if (serv_idx == serv_count)
	{
		DB_DevRecReady(connhandle);
	}
}


/*********************************************************************
 * @fn       DB_BuildCharTable
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */
void  DB_BuildCharTable(unsigned short int connhandle, unsigned short int* pHndl,unsigned char ValueLen ,unsigned char* pValue )
{

  if(DeviceRec[connhandle]->char_idx < MAX_CHAR_IN_DEV_SUPPORTED)
  {
	DeviceRec[connhandle]->connHandle = connhandle;

	// One more char found. Malloc for the char structure in DB
	DeviceRec[connhandle]->char_db[DeviceRec[connhandle]->char_idx] = malloc(sizeof(char_db_t));
	if(DeviceRec[connhandle]->char_db[DeviceRec[connhandle]->char_idx] == NULL)
	{
		DB_PRINT("\n\r [DB] Error - Not enough memory for char_db[DeviceRec[connhandle]->char_idx] \n");
		return;
	}

	DeviceRec[connhandle]->char_db[DeviceRec[connhandle]->char_idx]->char_handle = *(pHndl);
	DeviceRec[connhandle]->char_db[DeviceRec[connhandle]->char_idx]->char_value_handle = (((char_decl_t*)pValue)->char_val_hndl);
	DeviceRec[connhandle]->char_db[DeviceRec[connhandle]->char_idx]->properties  = (((char_decl_t*)pValue)->char_prop);

	DeviceRec[connhandle]->db_uuid_info->char_uuid_len =  ValueLen - 3;  // char decl hdr is of 3 bytes . Ref core4.1 Part G - 3.3.1
	DeviceRec[connhandle]->db_uuid_info->char_uuid     = (((char_decl_t*)pValue)->uuid);  // copy the UUID if needed for later use. It will be freed from Stack Q

	DeviceRec[connhandle]->char_db[DeviceRec[connhandle]->char_idx]->pCharName = NULL;

	DB_UpdateCharName(DeviceRec[connhandle]->connHandle , DeviceRec[connhandle]->char_idx , DeviceRec[connhandle]->db_uuid_info);

	DeviceRec[connhandle]->char_idx++;
  }
  else
  {
	Message("\n\r EXCEEDED THE NO OF CHARS IN DEVICE SUPPORTED \n\r");
  }
}

/*********************************************************************
 * @fn       DB_UpdateCharName
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */
void  DB_UpdateCharName(unsigned short int connhandle,unsigned char char_db_index,db_uuid_info_t* pdb_uuid_info  )
{
	char* defaultstring;

	Schema_FindString( DeviceRec[connhandle]->pProfileHandle , pdb_uuid_info , &(DeviceRec[connhandle]->char_db[char_db_index]->pCharName)  );

	if(DeviceRec[connhandle]->char_db[char_db_index]->pCharName == NULL)
	{
#if 0
		// Add dynamic string - sHs
		if(char_db_index <10)
		{
			defaultstring = malloc(strlen((CHAR_STRING)+5) + 1);
			if(defaultstring == NULL)
			{
				DB_PRINT("\n\r [DB] Error - Not enough memory for defaultstring \n");
				return;
			}
			sprintf((char *)defaultstring,"%s%d",CHAR_STRING,char_db_index);
			defaultstring[strlen((CHAR_STRING)+5)] = '\0';
		}
		else
		{
			defaultstring = malloc(strlen((CHAR_STRING)+6) + 1);
			if(defaultstring == NULL)
			{
				DB_PRINT("\n\r [DB] Error - Not enough memory for defaultstring \n");
				return;
			}
			sprintf((char *)defaultstring,"%s%d",CHAR_STRING,char_db_index);
			defaultstring[strlen((CHAR_STRING)+6)] = '\0';
		}
#endif


		defaultstring = malloc((strlen(CHAR_STRING)+6) + 1);
		if(defaultstring == NULL)
		{
			DB_PRINT("\n\r[DB] Error - Not enough memory for defaultstring");
			return;
		}
		sprintf((char *)defaultstring,"%s%#02d",CHAR_STRING,char_db_index);
		defaultstring[(strlen(CHAR_STRING))+6] = '\0';



		DeviceRec[connhandle]->char_db[char_db_index]->pCharName = defaultstring;
	}
}


/*********************************************************************
 * @fn       DB_PrintCharTable
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */
void  DB_PrintCharTable(unsigned short int connhandle)
{
	unsigned short int i;
	DBG_PRINT("\n\r Chars of device Id are \n\r" );
	DBG_PRINT("\n\r------------------------\n\r" );
	for(i = 0 ; i<DeviceRec[connhandle]->char_idx ; i++)
	{
		DBG_PRINT("CharNo:%d ,hndl = %2d",i,DeviceRec[connhandle]->char_db[i]->char_handle);
		DBG_PRINT(" , prop  = 0x%x",DeviceRec[connhandle]->char_db[i]->properties);
		DBG_PRINT(" , value_hndl  = %2d",DeviceRec[connhandle]->char_db[i]->char_value_handle);
		DBG_PRINT(" , string  = %s",DeviceRec[connhandle]->char_db[i]->pCharName);
		DBG_PRINT("\n\r");
	}

	DBG_PRINT("\n\r");
}


/*********************************************************************
 * @fn DB_CharDescsTableInit
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */
void DB_CharDescsTableInit(unsigned short int connhandle)
{
	unsigned char i;
	device_db_t* ptrtempDevRec =  DeviceRec[connhandle];

	ptrtempDevRec->CharDescsTable	 = malloc(sizeof(CharDescsTable_t));
	if(ptrtempDevRec->CharDescsTable == NULL)
	{
		DBG_PRINT("\n\r[DB] Error - not enough memory for ptrtempDevRec->CharDescsTable ...");
		HandleError(__FILE__,__LINE__,1);

		// Control should never reach here
		return ;
	}
	ptrtempDevRec->CharDescsTable->char_handle	   = 0;
	ptrtempDevRec->CharDescsTable->chardescs_count = 0;
	for(i = 0; i< MAX_CHARDESCS_IN_DEV_SUPPORTED; i++)
	{
		ptrtempDevRec->CharDescsTable->chardescs_table[i] = NULL;
	}
}


/*********************************************************************
 * @fn DB_CharDescsTableDeInit
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */
void DB_CharDescsTableDeInit(unsigned short int connhandle)
{
	device_db_t* ptrtempDevRec =  DeviceRec[connhandle];

	DB_FreeCharDescsTable(ptrtempDevRec);
}


/*********************************************************************
 * @fn       DB_ParseString
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */
char_attribute_type  DB_ParseString( char* pInputString )
{
	const char desc_str[5] = "desc";
	const char notif_str[5] = "noti";

	// Read Start flag
   if(strstr((const char *)pInputString,desc_str) != 0)
   {
		DBG_PRINT("\n\r  Detected Description Request ");
		DBG_PRINT("\n\r");
	    return  CHAR_DESCRIPTION;
   }
   else if(strstr((const char *)pInputString,notif_str) != 0)
   {
		DBG_PRINT("\n\r  Detected notification Request ");
		DBG_PRINT("\n\r");
	    return CHAR_NOTIFICATION;
   }
   else
   {
		DBG_PRINT("\n\r  Detected Real Value Request ");
		DBG_PRINT("\n\r");
		return  CHAR_VALUE;
   }
}


/*********************************************************************
 * @fn       DB_FindCharHandle
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */
unsigned char  DB_FindCharHandle(unsigned short int connhandle, char* pInputString , unsigned short int* pCharIndexMatch)
{
	unsigned short int i;
	unsigned char StringFoundFlag = FALSE;

	for(i = 0 ; i<DeviceRec[connhandle]->char_idx ; i++)
	{
		if (strncmp((const char *)pInputString, (const char *)DeviceRec[connhandle]->char_db[i]->pCharName, strlen((const char *)DeviceRec[connhandle]->char_db[i]->pCharName)) == 0 )
		{
			StringFoundFlag = TRUE;

			DBG_PRINT("\n\r  Found Char String  in DB  = %2d", DeviceRec[connhandle]->char_db[i]->char_handle);
			DBG_PRINT("\n\r");
			*pCharIndexMatch = i;
			break ;
		}
	}

	if (StringFoundFlag != TRUE)
	{
		DBG_PRINT("\n\r  TBD to find handle ");
		DBG_PRINT("\n\r");
	}

	return 0;
}


/*********************************************************************
 * @fn       DB_GetSetCharValue
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */
void  DB_GetSetCharValue(unsigned short int connhandle , unsigned short int char_index_match ,
						GC_GETSETFLAG get_set_flag  ,unsigned char WriteValueLen ,unsigned char* pWriteValueBuf )
{

	switch(get_set_flag)
	{
		case CHAR_GET:
		{
			process_ReadHandle(connhandle, DeviceRec[connhandle]->char_db[char_index_match]->char_value_handle);
		}
		break;

		case CHAR_SET:
		{
			process_WriteHandle(connhandle, DeviceRec[connhandle]->char_db[char_index_match]->char_value_handle ,
								pWriteValueBuf , WriteValueLen);
		}
		break;

		default:
			break;
	}
}


/*********************************************************************
 * @fn       DB_GetChar
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */
ErrorFlag_t DB_GetSetChar(unsigned short int connhandle , char* pInputString , GC_GETSETFLAG get_set_flag ,unsigned char ValueLen ,unsigned char* pValue )
{
	unsigned short int   char_index_match = 0xFFFF;

	char_attribute_type char_type = DB_ParseString(pInputString);
	unsigned char connhndlvalid   = DB_IsConnhandleValid(connhandle);

	if(connhndlvalid != CONNHNDL_VALID)
	{
//		CallGwDataCB(get_set_flag , ERROR_INVALIDCONNHNDL,  NULL);

		return ERROR_INVALIDCONNHNDL; // return -1 ?
	}

	DB_FindCharHandle(connhandle,pInputString , &char_index_match);
	
	if(char_index_match == 0xFFFF)
	{
//		CallGwDataCB(get_set_flag , ERROR_INVALIDSTRING,  NULL);
		return ERROR_INVALIDSTRING; // return -1 ?
	}

    /*
     * Store the current Get char string name to return to the Apps
     * */
	if(get_set_flag == CHAR_GET)
	{
		if(DeviceRec[connhandle]->pCurrentGetCharName !=NULL )
		{
			free(DeviceRec[connhandle]->pCurrentGetCharName);
			DeviceRec[connhandle]->pCurrentGetCharName = NULL;
		}

		DeviceRec[connhandle]->pCurrentGetCharName = malloc(strlen((const char*)pInputString));
		if(DeviceRec[connhandle]->pCurrentGetCharName == NULL)
		{
			HandleError(__FILE__,__LINE__,1);
		}
		strcpy((char*)DeviceRec[connhandle]->pCurrentGetCharName,(const char*) pInputString );
	}

	switch(char_type)
	{
		case CHAR_VALUE:
		{
			DB_GetSetCharValue(connhandle , char_index_match , get_set_flag , ValueLen , pValue);
		}
		break;

		case CHAR_DESCRIPTION:
		{
			DB_GetCharDescription(connhandle , char_index_match , GATT_CHAR_USER_DESC_UUID , get_set_flag , ValueLen , pValue);
		}
		break;

		case CHAR_NOTIFICATION:
		{
			DB_GetCharDescription(connhandle , char_index_match , GATT_CLIENT_CHAR_CFG_UUID , get_set_flag , ValueLen , pValue);
		}
		break;

		default:
			break;
	}

	return NO_ERROR;
}

/*********************************************************************
 * @fn       DB_FillCurrentGetCharName
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */
void DB_FillCurrentGetCharName(unsigned short int connhandle,unsigned char** pPtrCurrentGetCharName)
{
	*pPtrCurrentGetCharName = DeviceRec[connhandle]->pCurrentGetCharName;
	return;
}


/*********************************************************************
 * @fn       DB_FindCharNameString
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */
void DB_FindCharNameString(unsigned short int connhandle,unsigned short int charValHandle ,unsigned char** pPtrCharName)
{
	unsigned char i;

	for(i=0 ; i< MAX_CHAR_IN_DEV_SUPPORTED; i++)
	{
		if(DeviceRec[connhandle]->char_db[i]->char_value_handle == charValHandle)
		{
			*pPtrCharName = malloc(strlen((const char*)DeviceRec[connhandle]->char_db[i]->pCharName)) ;
			if(*pPtrCharName == NULL)
			{
				HandleError(__FILE__,__LINE__,1);
				return;
			}
			strcpy((char*)*pPtrCharName, (const char*)DeviceRec[connhandle]->char_db[i]->pCharName);
			return;
		}
	}

	return;
}


/*********************************************************************
 * @fn       DB_GATT_FindCharDescriptions
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */

void  DB_GATT_FindCharDescriptions(unsigned short int connhandle, unsigned short int start_handle , unsigned short int end_handle , unsigned short int* p_chardeschandle , unsigned char* p_chardescfound , unsigned short int ATTR_TYPE )
{

	*p_chardescfound = FALSE;

	unsigned short int CHAR_USER_DESC_UUID = GATT_CHAR_USER_DESC_UUID ;
	unsigned short int CLIENT_CHAR_CFG_UUID = GATT_CLIENT_CHAR_CFG_UUID;
	unsigned short int NO_UUID = 0x0;
	unsigned short int* pUuidRequested = &NO_UUID;

	switch(ATTR_TYPE)
	{
		case GATT_CHAR_USER_DESC_UUID:
		{
			pUuidRequested = &CHAR_USER_DESC_UUID;
		}
		break;

		case GATT_CLIENT_CHAR_CFG_UUID :
		{
			pUuidRequested = &CLIENT_CHAR_CFG_UUID;
		}
		break;

		default:
			// return Failure??
		break;
	}

	// Initialise the CharDescsTable element in DeviceRec
	DB_CharDescsTableInit(connhandle);

	DeviceRec[connhandle]->dev_state = DEV_FIND_CHAR_DESCS_PENDING;

	// Query the Device for the list of attributes in the given characteristic
	DB_GATT_FindCharDescs(connhandle ,start_handle , end_handle );

    // Waiting until all the attributes of this chararcteristic is collected
	//osi_Sleep(5000);
	while(DeviceRec[connhandle]->dev_state != DEV_READY)
	{
		osi_Sleep(500);
	}

	unsigned short int i ;
	DBG_PRINT("\n\r Chars  decscs of this char  are \n\r" );
	DBG_PRINT("\n\r---------------------------------\n\r" );
	for(i = 0 ; i<DeviceRec[connhandle]->CharDescsTable->chardescs_count ; i++)
	{
		if((DeviceRec[connhandle]->CharDescsTable->chardescs_table[i]->uuid_len == ATT_BT_UUID_SIZE)
				&&
		   (memcmp(DeviceRec[connhandle]->CharDescsTable->chardescs_table[i]->uuid , pUuidRequested , ATT_BT_UUID_SIZE )==0))
		{
			*p_chardescfound  = TRUE;

			*p_chardeschandle = DeviceRec[connhandle]->CharDescsTable->chardescs_table[i]->chardescs_hndl;
			break;

		}
	}

	if (*p_chardescfound  == TRUE)
	{
	  DBG_PRINT("\n\r Chars  decscription found \n\r" );
	  DBG_PRINT("\n\r------------------------\n\r" );
	}
	else
	{
	  DBG_PRINT("\n\r !!!  No Chars  decscription found  !!! \n\r" );
	  DBG_PRINT("\n\r------------------------\n\r" );
	}

	//return SUCCESS;

}


/*********************************************************************
 * @fn       DB_GetCharDescription
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */

void  DB_GetCharDescription(unsigned short int connhandle , unsigned short int char_index_match , unsigned short int ATTR_TYPE ,
		GC_GETSETFLAG get_set_flag  ,unsigned char WriteValueLen ,unsigned char* pWriteValueBuf)
{
	unsigned char chardescfound ;
	unsigned short int chardeschandle;
	unsigned short int start_handle,end_handle;

	start_handle = DeviceRec[connhandle]->char_db[char_index_match]->char_handle + 1;
	
	if(char_index_match < DeviceRec[connhandle]->char_idx)
	{
		 end_handle   = DeviceRec[connhandle]->char_db[char_index_match + 1]->char_handle - 1;
	}
	else
	{
		 end_handle   = 0xFFFF;
	}

	DB_GATT_FindCharDescriptions(connhandle ,start_handle , end_handle ,
								&chardeschandle , &chardescfound , ATTR_TYPE);

	switch(get_set_flag)
	{
		case CHAR_GET:
		{
			process_ReadHandle(connhandle, chardeschandle);
		}
		break;

		case CHAR_SET:
		{
			process_WriteHandle(connhandle, chardeschandle, pWriteValueBuf , WriteValueLen);
		}
		break;

		default:
			break;
	}

	DB_CharDescsTableDeInit(connhandle);

}


/*********************************************************************
 * @fn       DB_BuildCharDescsTable
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */

void DB_BuildCharDescsTable(unsigned short int connhandle, unsigned short int* pHndl,unsigned char UuidLen ,unsigned char* pUuid )
{
 unsigned char CharDescCount = DeviceRec[connhandle]->CharDescsTable->chardescs_count;


  if(CharDescCount < MAX_CHARDESCS_IN_DEV_SUPPORTED)
  {
	//CharDescsTable.chardescs_count = chardescsindex;

	// One more char found. Malloc for the char structure in DB
	DeviceRec[connhandle]->CharDescsTable->chardescs_table[CharDescCount] = malloc(sizeof(chardescs_table_t));
	if(DeviceRec[connhandle]->CharDescsTable->chardescs_table[CharDescCount] == NULL)
	{
		Message("\n\r[DB] Malloc Error");
	}

	DeviceRec[connhandle]->CharDescsTable->chardescs_table[CharDescCount]->chardescs_hndl  = *(pHndl);
	DeviceRec[connhandle]->CharDescsTable->chardescs_table[CharDescCount]->uuid_len = UuidLen;
	DeviceRec[connhandle]->CharDescsTable->chardescs_table[CharDescCount]->uuid = malloc(UuidLen);
	if(DeviceRec[connhandle]->CharDescsTable->chardescs_table[CharDescCount]->uuid == NULL)
	{
		Message("\n\r[DB] Malloc Error");
	}
	memcpy(DeviceRec[connhandle]->CharDescsTable->chardescs_table[CharDescCount]->uuid, pUuid ,UuidLen );
	DeviceRec[connhandle]->CharDescsTable->chardescs_count++;
  }
  else
  {
	Message("\n\r EXCEEDED THE NO OF CHARS DESCS IN DEVICE SUPPORTED \n\r");
  }
}

/*********************************************************************
 * @fn       DB_PrintCharDescsTable
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */

void DB_PrintCharDescsTable(unsigned short int connhandle)
{
	  unsigned short int i , j;

	  unsigned short int chardescscount = DeviceRec[connhandle]->CharDescsTable->chardescs_count;

	  DBG_PRINT("\n\r Chars  decscs of this char  are \n\r" );
	  DBG_PRINT("\n\r------------------------\n\r" );
		for(i = 0 ; i< chardescscount  ; i++)
		{
			DBG_PRINT("CharDescs Handle :%2d",DeviceRec[connhandle]->CharDescsTable->chardescs_table[i]->chardescs_hndl);
			DBG_PRINT(" , uuid  :");

			for(j=0; j< DeviceRec[connhandle]->CharDescsTable->chardescs_table[i]->uuid_len ; j++)
			{
				 DBG_PRINT("   0x%x" , *((unsigned char*)DeviceRec[connhandle]->CharDescsTable->chardescs_table[i]->uuid + j) );
			}
			DBG_PRINT("\n\r");
		}
		DBG_PRINT("\n\r");
}

/*********************************************************************
 * @fn       DB_UpdateDeviceName
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */
void  DB_UpdateDeviceName(unsigned short int connhandle ,unsigned char  devicename_len,char* devicename )
{
    DeviceRec[connhandle]->devicename = malloc(devicename_len+1);
    strncpy((char *)DeviceRec[connhandle]->devicename,(const char *)devicename,devicename_len);
    DeviceRec[connhandle]->devicename[devicename_len]='\0';
    DeviceRec[connhandle]->devicename_len = devicename_len;
}


/*********************************************************************
 * @fn       DB_getDevAddrPtr
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */

unsigned char* DB_getDevAddrPtr(unsigned short int connhandle)
{
  return(DeviceRec[connhandle]->addr);
}


/*********************************************************************
 * @fn       DB_getDevNamePtr
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */
char * DB_getDevNamePtr(unsigned short int connhandle)
{
  return(DeviceRec[connhandle]->devicename);
}


/*********************************************************************
 * @fn       DB_GetDevCharList
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */
void DB_GetDevCharList(unsigned short int connhandle , fptrDevCharlist_t fptr)
{
	int i;
	for(i=0;i< DeviceRec[connhandle]->char_idx ;i++)
	{
		(*fptr)(connhandle, i,DeviceRec[connhandle]->char_db[i]->pCharName , DeviceRec[connhandle]->char_db[i]->properties);
	}
}


/*******************************************************************************
 * @fn          DB_IsConnIdValid
 *
 * @brief
 *
 * @param
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
signed int DB_IsConnIdValid(unsigned int uConnIdx)
{

	if(uConnIdx >= MAX_DEVICE_DB_SUPPORTED )
		return (-1);

	if(DeviceRec[uConnIdx] != NULL)
	{
		// Device record found . Device is already linked and can be terminated
		return(0);
	}
	else
	{
		// no Device record found . Device is not linked yet and so no trmination possible
		return(-1);
	}
}

//1 sec = 4 counts
//1 minute = 240 counts
#define LINK_TIMEOUT 240
DB_FlushSyncObjects()
{
	unsigned char count;
	for(count=0;count<MAX_SCAN_RESPONSE;count++)
	{
		if(gw_context.scanRespArray[count]!=NULL)
		{
			if(gw_context.scanRespArray[count]->deviceRec->devRecSyncTimeout > LINK_TIMEOUT)
			{
				gw_context.scanRespArray[count]->deviceRec->devRecSyncTimeout = LINK_TIMEOUT;
			}
			else if(gw_context.scanRespArray[count]->deviceRec->devRecSyncTimeout == 0)
			{
				continue;
			}
			else
			{
				gw_context.scanRespArray[count]->deviceRec->devRecSyncTimeout--;
				if(gw_context.scanRespArray[count]->deviceRec->devRecSyncTimeout==0)
				{
					if(gw_context.scanRespArray[count]->deviceRec->gw_DevRecSyncObj!=NULL)
					{
						Message("\n\rLink Establish Timed out");
						osi_SyncObjSignal(&(gw_context.scanRespArray[count]->deviceRec->gw_DevRecSyncObj));
					}
				}
			}

		}
	}
}
