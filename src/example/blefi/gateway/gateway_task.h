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

#ifndef __GATEWAY_TASK_H
#define __GATEWAY_TASK_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

#include <stdbool.h>
#include "database.h"
#include "datatypes.h"
#include "gateway_cmds.h"
#include "gateway_api.h"
#include "gapbondmgr.h"

/*********************************************************************
*  EXTERNAL VARIABLES
*/


extern void HandleError(unsigned char *, unsigned int, unsigned char error);

/*********************************************************************
 * MACROS
 */

#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)

// TI Base 128-bit UUID: F000XXXX-0451-4000-B000-000000000000
#define TI_BASE_UUID_128( uuid )  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB0, \
                                  0x00, 0x40, 0x51, 0x04, LO_UINT16( uuid ), HI_UINT16( uuid ), 0x00, 0xF0
// TI Base 128-bit UUID: F000XXXX-0451-4000-B000-000000000000
#define TI_UUID_SIZE        ATT_UUID_SIZE
#define TI_UUID(uuid)       TI_BASE_UUID_128(uuid)




//*****************************************************************************
//                      GLOBAL VARIABLES
//*****************************************************************************

// Gateway Central Task Events
#define GC_START_DISCOVERY_EVT               0x0001
#define GC_PAIRING_STATE_EVT                 0x0002
#define GC_PASSCODE_NEEDED_EVT               0x0004
#define GC_RSSI_READ_EVT                     0x0008
#define GC_KEY_CHANGE_EVT                    0x0010
#define GC_STATE_CHANGE_EVT                  0x0020

#define GC_INPUT_FROM_APP                     0x0040

// Maximum number of scan responses
//#define DEFAULT_MAX_SCAN_RES                  8
#define DEFAULT_MAX_SCAN_RES                  5

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 40000

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// TRUE to use high scan duty cycle when creating link
#define DEFAULT_LINK_HIGH_DUTY_CYCLE          FALSE

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST               FALSE

// Default RSSI polling period in ms
#define DEFAULT_RSSI_PERIOD                   1000

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Minimum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL      400

// Maximum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL      800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_UPDATE_SLAVE_LATENCY          0

// Supervision timeout value (units of 10ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_CONN_TIMEOUT           600

// Default passcode
#define DEFAULT_PASSCODE                      19655

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_DISPLAY_ONLY

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           1000

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          TRUE

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

// Task configuration
#define GC_TASK_PRIORITY                     1
#define GC_TASK_STACK_SIZE                   764

#define GAP_DEVICE_NAME_LEN                     (20+1)


// Simple Profile Service UUID
//#define SIMPLEPROFILE_SERV_UUID               0xFFF0


// GAP - Messages IDs (0xD0 - 0xDF)
#define GAP_MSG_EVENT                         0x06 //!< Incoming GAP message

#define GATT_MSG_EVENT                        0x05 //!< Incoming GATT message


#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)

// Application states

typedef enum GW_STATE
{
	GW_NO_INIT, //0
	GW_INIT_DONE, //1
	GW_DISC_IN_PROGRESS, //2
	GW_DISC_DONE //3
}GW_STATE;


typedef enum LedNum
{
  LED_1 = 0,
  LED_2
}LedNum;

// GAP GATT Attributes
static const unsigned char attDeviceName[GAP_DEVICE_NAME_LEN] = "Gateway Central";

#define MAX_GATEWAY_MSGS 40 //16
#define MAX_APP_MSGS 16
#define NO_MSG 0xFF
#define STACK_MSG 0x1
#define APP_MSG 0x2

#define MAX_SCAN_RESPONSE 10
#define MAX_APPCB_SUPPORTED 5
#ifndef B_ADDR_LEN
#define B_ADDR_LEN 0x6
#endif
//Semaphore

OsiLockObj_t GC_ApiLockObj;

// Message Qs
OsiMsgQ_t sGC_StackMsgQueue;
OsiMsgQ_t sGC_AppMsgQueue;


// Stack Queue Structure
typedef struct Msg_t
{
	void * pData;
}Msg_t;

typedef struct MsgHdr_t
{
	unsigned char event;
	unsigned char status;
}MsgHdr_t;

typedef struct gwCallType_t
{
	GW_CALL gwScanCallType;
	GW_CALL gwStopScanCallType;
	GW_CALL gwLinkEstCallType;
	GW_CALL gwLinkTermCallType;
}gwCallType_t;

// GC state callback
typedef void (*gc_StateCB_t)(unsigned char state);

typedef gc_StateCB_t GC_StateCB_t;


typedef struct scanResponse_t
{
	unsigned char addrType;           //!< address type: @ref GAP_ADDR_TYPE_DEFINES
	unsigned char addr[B_ADDR_LEN];   //!< Address of the advertisement or SCAN_RSP
	unsigned char rssi;                //!< Advertisement or SCAN_RSP RSSI
	char *deviceName;
	device_db_t* deviceRec; //Points to the device rec once its malloced
}scanResponse_t;

typedef struct gw_context_t
{
	 GW_STATE state  ;
	 unsigned char scanCnt ;
	 gwCallType_t gwCallType;
	 fptrCB_t fptrEvent[MAX_APPCB_SUPPORTED];
	 fptrCB_t fptrData[MAX_APPCB_SUPPORTED];
	 scanResponse_t* scanRespArray[MAX_SCAN_RESPONSE]; //indexing the scan response
	 OsiSyncObj_t gw_GapSyncObj; //used to block the commands from the user.
}gw_context_t;

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */
/*
 * Task creation function for the Simple BLE Central.
 */
unsigned char get_central_state(void);
 void GatewayCentral_createTask(void);
 void GatewayCentral_UartInput_createTask(void);
bool GatewayCentral_findSvcUuid(unsigned short uuid,unsigned char *pData,
                                        unsigned char dataLen);
void GatewayCentral_addDeviceInfo(unsigned char *pAddr,unsigned char addrType);
void GatewayCentral_RegisterGC_CB(GC_StateCB_t pCBs);
void GC_StackEventHandler_CB(void *inPtr_event);
GW_CALL  get_gwCallType(cmd_no_e gwCmd);
GW_CALL  set_gwCallType(cmd_no_e gwCmd, GW_CALL CallType);
unsigned char  set_gwState(GW_STATE state);
unsigned char  get_gwState(void);
unsigned char  get_gwDevCnt(void);
int get_gw_devConnectionid(unsigned char index);
void gwcontext_Init(void);
signed int gwcontext_IsScanIdValid(unsigned int uScanIdx);
void CallGwEventCB(unsigned char EventType, void* EventParams);
void CallGwDataCB(unsigned char DataType, ErrorFlag_t ErrorFlag, void* DataParams);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* GATEWAY_TASK_H */
