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

#ifndef __DATABASE_H__
#define __DATABASE_H__

/*********************************************************************
 * INCLUDES
 */

#include "att.h"
#include "osi.h"
#include "hci.h"
#include "gateway_api.h"


/*********************************************************************
 * DEFINES
 */
#define MAX_DEVICE_DB_SUPPORTED 		10
#define MAX_PRIM_SERV_SUPPORTED 		30
#define MAX_CHAR_IN_SERV_SUPPORTED 		50
#define MAX_CHAR_IN_DEV_SUPPORTED 		50
#define MAX_CHARDESCS_IN_DEV_SUPPORTED 	10

// Discovery states

typedef enum DEV_STATE
{
  DEV_DISCOVERED,                // Idle
  DEV_CONNECTING,
  DEV_CONNECTED,
  DEV_SVC_PENDING,                 // Service discovery
  DEV_SVC_DONE,
  DEV_CHAR_PENDING,        // Characteristic discovery
  DEV_CHAR_DONE,			  // Characteristic discovery
  DEV_ATTR_PENDING,        // // is this used?
  DEV_ATTR_DONE,			  // // is this used?
  DEV_FIND_CHAR_DESCS_PENDING,
  DEV_FIND_CHAR_DESCS_DONE, // is this used?
  DEV_READY,
  DEV_DISCONNECTING,
  DEV_DISCONNECTED,
}DEV_STATE;

typedef enum char_attribute_type
{
  CHAR_VALUE,
  CHAR_NOTIFICATION,
  CHAR_DESCRIPTION
}char_attribute_type;


typedef struct char_db_t
{
	unsigned short int char_handle ;
	char*  pCharName;
	unsigned char properties;
	unsigned short int char_value_handle;
	unsigned char char_value_len;
	void* char_value;
}char_db_t;

typedef struct prim_serv_table_t
{
	handle_range_t serv_hndl;
	unsigned char uuid_len;         //!< Length of UUID
	const unsigned char *uuid; //!< Pointer to char UUID
}prim_serv_table_t;


typedef struct __attribute__((__packed__))
{
	unsigned char prim_serv_idx;
	unsigned char prim_serv_count;
	prim_serv_table_t prim_serv_table[MAX_PRIM_SERV_SUPPORTED];
}DevPrimServiceTable_t;

typedef struct __attribute__((__packed__))
{
	unsigned char char_prop;
	unsigned short int char_val_hndl;
	unsigned char uuid[ATT_UUID_SIZE]; //!< 16 or 128 bit UUID
}char_decl_t;

typedef struct char_table_t
{
	unsigned short int char_hndl;
	unsigned char uuid_len;         //!< Length of UUID
	char_decl_t char_decl;
}char_table_t;

typedef struct chardescs_table_t
{
	unsigned short int chardescs_hndl;
	unsigned char uuid_len;         //!< Length of UUID
	unsigned char* uuid;
}chardescs_table_t;

typedef struct CharDescsTable_t
{
	unsigned short int char_handle;
	unsigned char chardescs_count;
	chardescs_table_t* chardescs_table[MAX_CHARDESCS_IN_DEV_SUPPORTED];
}CharDescsTable_t;

typedef struct db_uuid_info_t
{
	unsigned char serv_uuid_len;         //!< Length of UUID
	const unsigned char *serv_uuid; //!< Pointer to service UUID
	unsigned char char_uuid_len;         //!< Length of UUID
	const unsigned char *char_uuid; //!< Pointer to char UUID
}db_uuid_info_t;


typedef enum CONNHNDLVALID
{
  CONNHNDL_INVALID = 0,
  CONNHNDL_VALID
}CONNHNDLVALID;


typedef struct device_db_t
{
	unsigned short int connHandle;
	DEV_STATE dev_state;
	void* pProfileHandle;
	unsigned char addr[B_ADDR_LEN]; //!< Device's Address
	unsigned char devicename_len; // Do we use this anymore?
	char* devicename;
	unsigned char char_idx;
	unsigned char*  pCurrentGetCharName;
	DevPrimServiceTable_t* pDevPrimServiceTable;
	db_uuid_info_t* db_uuid_info;
	CharDescsTable_t* CharDescsTable;
	OsiSyncObj_t gw_DevRecSyncObj;
	unsigned int devRecSyncTimeout;
	char_db_t* char_db[MAX_CHAR_IN_DEV_SUPPORTED];  // pointer to the char array
}device_db_t;



/*********************************************************************
 * FUNCTION DEFINITIONS
 */
unsigned char  set_devState(unsigned short int connhandle,DEV_STATE state);
unsigned char  get_devState(unsigned short int connhandle);
void  DB_DevRecDbInit( void);
void  DB_GetDeviceName(unsigned short int connhandle , device_db_t* ptrDeviceRec );
void  DB_Build(unsigned short int connhandle );
void  DB_BuildPrimServTable(unsigned short int connhandle, unsigned short int* pStartHndl, unsigned short int* pEndHndl ,unsigned char ValueLen ,unsigned char* pValue );
void  DB_DiscAllChars(unsigned short int connhandle, unsigned short int* StartHndl, unsigned short int* EndHndl );
void  DB_DiscAllCharinService(unsigned short int connhandle);
void  DB_PrintPrimServTable(unsigned short int connhandle);
void  DB_BuildCharTable(unsigned short int connhandle, unsigned short int* pHndl,unsigned char ValueLen ,unsigned char* pValue );
void  DB_PrintCharTable(unsigned short int connhandle);
void DB_CharDescsTableInit(unsigned short int connhandle);
unsigned char DB_FindCharHandle(unsigned short int connhandle, char* pInputString , unsigned short int* pChar_handle);
void  DB_UpdateCharName(unsigned short int connHandle,unsigned char char_db_count,db_uuid_info_t* pdb_uuid_info  );
void  DB_GetCharDescription(unsigned short int connhandle , unsigned short int char_handle , unsigned short int ATTR_TYPE, GC_GETSETFLAG get_set_flag  ,unsigned char WriteValueLen ,unsigned char* pWriteValueBuf);
void  DB_UpdateDeviceName(unsigned short int connhandle ,unsigned char  devicename_len, char* devicename );
char_attribute_type  DB_ParseString(char* pInputString );
ErrorFlag_t DB_GetSetChar(unsigned short int connhandle , char* pInputString , GC_GETSETFLAG get_set_flag ,unsigned char ValueLen ,unsigned char* pValue );
unsigned char* DB_getDevAddrPtr(unsigned short int connhandle);
void DB_BuildCharDescsTable(unsigned short int connhandle, unsigned short int* pHndl,unsigned char UuidLen ,unsigned char* pUuid );
void DB_deleteDevRec(unsigned short int connhandle);
void DB_FillCurrentGetCharName(unsigned short int connhandle,unsigned char** pPtrCurrentGetCharName);
void DB_FindCharNameString(unsigned short int connhandle,unsigned short int charValHandle ,unsigned char** pPtrCharName);
char * DB_getDevNamePtr(unsigned short int connhandle);
void  DB_LinkDevRec(unsigned short int connhandle , device_db_t* ptrDeviceRec );
OsiSyncObj_t* gw_createDeviceRec(unsigned char ScanId );
void DB_DevRecInit(device_db_t * ptrtempDevRec,unsigned char ScanIdx );
void DB_GATT_FindCharDescriptions(unsigned short int connhandle, unsigned short int start_handle ,
								   unsigned short int end_handle , unsigned short int* p_chardeschandle ,
								   unsigned char* p_chardescfound , unsigned short int ATTR_TYPE );
void DB_PrintCharDescsTable(unsigned short int connhandle);
signed int DB_IsConnIdValid(unsigned int uConnIdx);

#endif /* DATABASE_H */

