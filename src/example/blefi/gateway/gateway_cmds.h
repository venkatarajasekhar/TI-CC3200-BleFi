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

#ifndef __GATEWAY_CMDS_H_
#define __GATEWAY_CMDS_H_

#include "ble_central.h"
#include "gatt.h"
#include "gateway_api.h"
#include "database.h"

#define N_HELP_ENTRIES 15
extern char g_cmd_help[N_HELP_ENTRIES][20];

typedef enum
{
 HELP = 0,
 DEVICE_SCAN ,
 DEVICE_STOPSCAN ,
 DEVICE_LIST,
 LINK_ESTABLISH ,
 LINK_TERMINATE ,
 GET_TEMPERATURE,
 GET_TEMPERATURE_READING,
 GATT_DISC_ALLCHARDESCS ,
 GATT_DISC_ALLPRIMARYSERVICES,
 GATT_DISC_PRIMARYSERVICEBYUUID ,
 GATT_DISC_ALLCHARS,
 GATT_DISC_CHARBYUUID ,
 GATT_READ_USINGCHARUUID,
 cmd14,
 cmd15,
}cmd_no_e;

/**************************************************************/
typedef enum
{
  DEVICE_DISCOVERY_START = 0,
  DEVICE_DISCOVERY_STOP
}DeviceDiscovery_StartFlag_e;



/**************************************************************/


typedef struct param_Help_t
{
	// help
	cmd_hdr InputCmd;
}param_Help_t;

typedef struct param_Scan_t
{
  // Device Scan
  cmd_hdr InputCmd;
}param_Scan_t;

typedef struct param_StopScan_t
{
	// Stop Device Scan
	cmd_hdr InputCmd;
}param_StopScan_t;


typedef struct param_DeviceList_t
{
	// Device List
	cmd_hdr InputCmd;
}param_DeviceList_t;


typedef struct param_Get_t
{
	// GET
	cmd_hdr InputCmd;
	unsigned short connHandle;
}param_Get_t;

typedef struct param_Set_t
{
	// SET
	cmd_hdr InputCmd;
	unsigned short connHandle;
}param_Set_t;

typedef struct param_GattDiscAllPrimaryServices_t
{
	// GattDiscAllPrimaryServices
	cmd_hdr InputCmd;
	unsigned short connHandle;
}param_GattDiscAllPrimaryServices_t;

typedef struct param_GattDiscPrimaryServiceByUUID_t
{
	// GattDiscPrimaryServiceByUUID
	cmd_hdr InputCmd;
	unsigned short connHandle;
	unsigned char *pServUuid;
	unsigned short* pServUuidlen;
	handle_range_t* pServHndlRng;
}param_GattDiscPrimaryServiceByUUID_t;

typedef struct param_GATT_DiscAllChars_t
{
	// GATT_DiscAllChars
	cmd_hdr InputCmd;
	unsigned short connHandle;
	unsigned short startHandle;
	unsigned short endHandle;
}param_GATT_DiscAllChars_t;

typedef struct param_GATT_DiscCharByUUID_t
{
  // GATT_DiscCharByUUID
   cmd_hdr InputCmd;
   unsigned short connHandle;
  unsigned char *pCharUuid;
   unsigned short* pCharUuidlen;
   handle_range_t* pCharHndlRng;
}param_GATT_DiscCharByUUID_t;


typedef struct param_GATTFindCharDescs_t
{
  // GATT_DiscCharByUUID
   cmd_hdr InputCmd;
   unsigned short connHandle;
   unsigned short starthandle;
   unsigned short endhandle;
}param_GATTFindCharDescs_t;

typedef struct param_GattReadUsingCharUuid_t
{
  // GattReadUsingCharUuid
   cmd_hdr InputCmd;
   unsigned short connHandle;
   unsigned short starthandle;
   unsigned short endhandle;
  unsigned char CharUuidlen;
  unsigned char *pCharUuid;
}param_GattReadUsingCharUuid_t;

/***********************
 * FUNCTION DECLARATIONS
 *
 * *********************/


void GatewayCentral_handleinput(cmd_hdr** inParams);
// functions to process the commands
void process_DeviceDiscoveryStart(void** inParams );
void process_DeviceDiscoveryStop(void** inParams );
void process_DeviceList(void** inParams );
void process_LinkEstablish(void** inParams );
void process_LinkTerminate(void** inParams );
void process_GattReadUsingCharUuid(void** inParams );
void process_ReadHandle(unsigned short int connhandle, unsigned short int char_handle);
void process_WriteHandle(unsigned short int connhandle, unsigned short int char_handle ,unsigned char* pValue , unsigned short int valuelen);
void process_GATTFindCharDescs(void** inParams );
void process_GattDiscAllPrimaryServices(void** inParams );
void process_GattDiscAllChars(void** inParams );
void process_ScanStop(void** inParams );

unsigned char GatewayCentral_processRoleEvent(gapCentralRoleEvent_t **pEventPtr);
void GatewayCentral_processGattEvent(gattMsg_t  **pEventPtr);

#endif //GATEWAY_CMDS_H_
