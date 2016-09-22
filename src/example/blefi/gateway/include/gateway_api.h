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

#ifndef __GATEWAY_API_H_
#define __GATEWAY_API_H_

#include <stdbool.h>
#undef B_ADDR_LEN
#define B_ADDR_LEN 0x6
enum
{
  GW_EVENT_CB =0,
  GW_DATA_CB
}GW_APP_CB;


#define DEFAULT_MAX_CONN_RES 3
#define DEFAULT_MAX_SCAN_RES 5

enum
{
  GW_EVENT_SCANRSP =0,
  GW_EVENT_SCANCOMPLETE,
  GW_EVENT_LINKESTABLISH,
  GW_EVENT_LINKTERMINATE
}GW_EVENT_TYPE;

enum
{
  GW_DATA_GET =0,
  GW_DATA_SET
}GW_DATA_TYPE;

typedef enum
{
  GW_CALL_NONBLOCKING = 0,
  GW_CALL_BLOCKING
} GW_CALL;

typedef enum GC_GETSETFLAG
{
  CHAR_GET =0,
  CHAR_SET
}GC_GETSETFLAG;

// App event passed from profiles.
typedef struct
{
 unsigned char event;  // event type
 unsigned char status; // event status
 unsigned char *pData; // event data pointer
} sbcEvt_t;

typedef enum
{
NO_ERROR = 0,
ERROR_INVALIDCONNHNDL = 0x40, // setting an error offset to avoid confusion with other Data Events in CLI_handler.c
ERROR_INVALIDSTRING,
}ErrorFlag_t;

typedef struct cmd_hdr
{
  // help
	unsigned char Cmd;
}cmd_hdr;

typedef struct param_LinkEstablish_t
{
  // LINK ESTABLISH
   cmd_hdr InputCmd;
  unsigned short DeviceId;
}param_LinkEstablish_t;

typedef struct param_LinkTerminate_t
{
  // LINK TERMINATE
   cmd_hdr InputCmd;
   unsigned short int DeviceId;
}param_LinkTerminate_t;

typedef struct param_GetDevCharList_t
{
  // LINK ESTABLISH
  cmd_hdr InputCmd;
  unsigned short int DeviceId;
}param_GetDevCharList_t;


typedef struct handle_range_t
{
	unsigned short int start_handle;
	unsigned short int end_handle;
}handle_range_t;


/**
 *Get Data header format.
 */
typedef struct __attribute__((__packed__))
{
    unsigned short int connhandle;
   unsigned char* pCurrentGetCharName;
   unsigned char  ValueLen;
   unsigned char* pValueBuffer;
} gwGetData_t;



/**
 *Set Data header format.
 */
typedef struct __attribute__((__packed__))
{
    unsigned short int connhandle;
   unsigned char* pCurrentSetCharName;
} gwSetData_t;

/**
 *Scan Response event header format.
 */
typedef struct __attribute__((__packed__))
{
   unsigned char scanIdx;
	unsigned char addrType;           //!< address type: @ref GAP_ADDR_TYPE_DEFINES
	unsigned char addr[B_ADDR_LEN];   //!< Address of the advertisement or SCAN_RSP
	unsigned char rssi;                //!< Advertisement or SCAN_RSP RSSI
	char *deviceName;
} gwScanRspEvent_t;


/**
 *Scan Complete event header format.
 */
typedef struct __attribute__((__packed__))
{
   unsigned char status;
} gwScanComplEvent_t;

/**
 *Link Establish event header format.
 */
typedef struct __attribute__((__packed__))
{
    unsigned short int connhandle;
	unsigned char  bdAddr[B_ADDR_LEN];   //!< Address of the advertisement or SCAN_RSP
	char  *deviceName;
} gwLinkEstEvent_t;

/**
 *Link Terminate event header format.
 */
typedef struct __attribute__((__packed__))
{
    unsigned short int connhandle;
   unsigned char  reason;
	unsigned char  bdAddr[B_ADDR_LEN];   //!< Address of the advertisement or SCAN_RSP
	char *deviceName;
} gwLinkTermEvent_t;



/**
 * GW Event Structure
 */
typedef union __attribute__((__packed__))
{
  gwScanRspEvent_t            scanRsp;          //!< .
  gwLinkEstEvent_t 			  linkEst;          //!<
  gwLinkTermEvent_t 		  linkTerm;         //!<
} gwEvent_t;


/**
 * GW Data Structure
 */
typedef union __attribute__((__packed__))
{
  gwGetData_t            getRsp;          //!< .
  gwSetData_t            setRsp;
} gwData_t;

typedef void (*fptrDevlist_t)(unsigned char a, char *b, unsigned char *c, unsigned char d);
typedef void (*fptrDevCharlist_t)(unsigned short int connhandle, unsigned short int i,char* a,unsigned char b);
typedef void (*fptrPairlist_t)(unsigned char a, unsigned char *b);

int GC_Scan(GW_CALL calltype);
void GC_DeviceList(fptrDevlist_t fptr);
void GC_DeviceScanList(fptrDevlist_t fptr);
void GC_DeviceConnList(fptrDevlist_t fptr);
short int GC_LinkEstablish(unsigned int ucScanIdx ,GW_CALL calltype,unsigned char* BdAddr);
short int GC_LinkTerminate(unsigned int ucDeviceId, GW_CALL calltype );
void GC_GetTemperature(void);
void GC_GetTemperature_Reading(void);
void GC_GATT_DiscAllPrimaryServices(unsigned short int connHandle);
void GC_GATT_DiscPrimaryServiceByUUID(unsigned short int connHandle,unsigned char* pServUuid, unsigned short int* pServUuidlen , handle_range_t* pServHndlRng);
void GC_GATT_DiscAllChars(unsigned short int connhandle, unsigned short int StartHndl, unsigned short int EndHndl);
ErrorFlag_t GC_GetSet(unsigned short int connhandle ,char* pInputString , GC_GETSETFLAG get_set_flag ,unsigned char WriteValueLen ,unsigned char* pWriteValueBuf);
void GC_GATT_DiscCharByUUID(unsigned short int connHandle,unsigned char* pCharUuid,	unsigned short int* pCharUuidlen , handle_range_t* pCharHndlRng);
int GC_StopScan(GW_CALL calltype);
void GC_GetDevCharList(unsigned short int connhandle , fptrDevCharlist_t fptr);
void GC_EraseAllBondings(void);
bool GC_IsBLEReady();

char *Util_convertBdAddr2Str(unsigned char *pAddr);

typedef void (*fptrCB_t)(unsigned char Type, ErrorFlag_t ErrorFlag,void* pData);
int RegisterGwCB(unsigned char NotificationType, fptrCB_t fptr);



#endif /* GATEWAY_API_H_ */
