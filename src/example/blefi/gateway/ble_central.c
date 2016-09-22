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
#include <stdlib.h>
#include <stdio.h>
#include "datatypes.h"
#include "gap.h"
#include "gatt.h"
#include "hci.h"
#include "osi.h"
#include "gateway_api.h"
#include "database.h"
#include "gateway_task.h"
#include "ble_central.h"



//




//#include "datatypes.h"
//#include "uart_if.h"
//


extern unsigned short int selfEntity;



/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Profile Events
#define START_ADVERTISING_EVT            0x0001
#define RSSI_READ_EVT                    0x0002
#define UPDATE_PARAMS_TIMEOUT_EVT        0x0004

// Profile OSAL Message IDs
#define GAPCENTRALROLE_RSSI_MSG_EVT      0xE0

// Task configuration
#define GAPCENTRALROLE_TASK_PRIORITY     3
#define GAPCENTRALROLE_TASK_STACK_SIZE   400

extern bool is_bleStackReady;

/*********************************************************************
 * TYPEDEFS
 */

// App event structure
typedef struct
{
 unsigned char  event;  // event type
 unsigned char  status; // event status
} gapCentralRoleEvt_t;



/*********************************************************************
 * GLOBAL VARIABLES
 */

// Link DB maximum number of connections
unsigned char linkDBNumConns;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */

bStatus_t WaitForEvent(unsigned long event);


/*********************************************************************
 * Profile Parameters - reference GAPCENTRALROLE_PROFILE_PARAMETERS for
 * descriptions
 */

 unsigned char  gapCentralRoleIRK[KEYLEN];
 unsigned char  gapCentralRoleSRK[KEYLEN];
unsigned long gapCentralRoleSignCounter = 0x1;
 unsigned char  gapCentralRoleBdAddr[B_ADDR_LEN];
 unsigned char  gapCentralRoleMaxScanRes = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * CALLBACKS
 */


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/**
 * @brief   Start the device in Central role.  This function is typically
 *          called once during system startup.
 *
 * Public function defined in ble_central.h.
 */
bStatus_t GAPCentralRole_StartDevice(void)
{
	bStatus_t ret_status;

  ret_status =  GAP_DeviceInit(selfEntity, GAP_PROFILE_CENTRAL,
                        gapCentralRoleMaxScanRes, gapCentralRoleIRK,
                        gapCentralRoleSRK, (unsigned char*)&gapCentralRoleSignCounter);

  WaitForEvent(GAP_DEVICEINITDONE);
  is_bleStackReady = true;

  return ret_status;


}

/**
 * @brief   Set a parameter in the Central Profile.
 *
 * Public function defined in ble_central.h.
 */
bStatus_t GAPCentralRole_SetParameter(unsigned short int param,unsigned char len, void *pValue)
{
  bStatus_t ret = SUCCESS;

  switch (param)
  {
    case GAPCENTRALROLE_IRK:
      if (len == KEYLEN)
      {
         memcpy(gapCentralRoleIRK, pValue, KEYLEN) ;
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPCENTRALROLE_SRK:
      if (len == KEYLEN)
      {
         memcpy(gapCentralRoleSRK, pValue, KEYLEN) ;
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPCENTRALROLE_SIGNCOUNTER:
      if (len == sizeof (unsigned long))
      {
        gapCentralRoleSignCounter = *((unsigned long*)pValue);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPCENTRALROLE_MAX_SCAN_RES:
      if (len == sizeof (unsigned char))
      {
        gapCentralRoleMaxScanRes = *((unsigned char*)pValue);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return ret;
}

/**
 * @brief   Get a parameter in the Central Profile.
 *
 * Public function defined in ble_central.h.
 */
bStatus_t GAPCentralRole_GetParameter(unsigned short int param, void *pValue)
{
  bStatus_t ret = SUCCESS;

  switch (param)
  {
    case GAPCENTRALROLE_IRK:
       memcpy(pValue, gapCentralRoleIRK, KEYLEN) ;
      break;

    case GAPCENTRALROLE_SRK:
       memcpy(pValue, gapCentralRoleSRK, KEYLEN) ;
      break;

    case GAPCENTRALROLE_SIGNCOUNTER:
      *((unsigned long*)pValue) = gapCentralRoleSignCounter;
      break;

    case GAPCENTRALROLE_BD_ADDR:
       memcpy(pValue, gapCentralRoleBdAddr, B_ADDR_LEN) ;
      break;

    case GAPCENTRALROLE_MAX_SCAN_RES:
      *((unsigned char*)pValue) = gapCentralRoleMaxScanRes;
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return ret;
}

/**
 * @brief   Terminate a link.
 *
 * Public function defined in ble_central.h.
 */
bStatus_t GAPCentralRole_TerminateLink(unsigned short int connHandle)
{
  return GAP_TerminateLinkReq(selfEntity, connHandle, HCI_DISCONNECT_REMOTE_USER_TERM) ;
}

/**
 * @brief   Establish a link to a peer device.
 *
 * Public function defined in ble_central.h.
 */
bStatus_t GAPCentralRole_EstablishLink(unsigned char highDutyCycle,unsigned char whiteList,
                                       unsigned char addrTypePeer,unsigned char *peerAddr)
{
  gapEstLinkReq_t params;


  params.highDutyCycle = highDutyCycle;
  params.whiteList = whiteList;
  params.addrTypePeer = addrTypePeer;

  memcpy(params.peerAddr, peerAddr, B_ADDR_LEN);

  return GAP_EstablishLinkReq(&params);
}

/**
 * @brief   Update the link connection parameters.
 *
 * Public function defined in ble_central.h.
 */
bStatus_t GAPCentralRole_UpdateLink(unsigned short int connHandle, unsigned short int connIntervalMin,
                                    unsigned short int connIntervalMax, unsigned short int connLatency,
                                    unsigned short int connTimeout)
{
  gapUpdateLinkParamReq_t params;

  params.connectionHandle = connHandle;
  params.intervalMin = connIntervalMin;
  params.intervalMax = connIntervalMax;
  params.connLatency = connLatency;
  params.connTimeout = connTimeout;
  
  return GAP_UpdateLinkParamReq(&params);
}

/**
 * @brief   Start a device discovery scan.
 *
 * Public function defined in ble_central.h.
 */
bStatus_t GAPCentralRole_StartDiscovery(unsigned char mode,unsigned char activeScan,unsigned char whiteList)
{
  gapDevDiscReq_t params;
  bStatus_t ret_status;

  params.mode = mode;
  params.activeScan = activeScan;
  params.whiteList = whiteList;

  ret_status = GAP_DeviceDiscoveryRequest(&params);


   return ret_status;
}

/**
 * @brief   Cancel a device discovery scan.
 *
 * Public function defined in ble_central.h.
 */
bStatus_t GAPCentralRole_CancelDiscovery(void)
{
  return GAP_DeviceDiscoveryCancel(selfEntity);
}


/**
 * @brief   WaitForEvent.
 *
 * local function
 */
bStatus_t WaitForEvent(unsigned long event)
{
   Msg_t GC_StackMsg_t;
   OsiReturnVal_e eRetVal;

    eRetVal = osi_MsgQRead( &sGC_StackMsgQueue, &GC_StackMsg_t, OSI_WAIT_FOREVER);
	ASSERT_ON_ERROR(eRetVal);
    if(OSI_OK==eRetVal )
    {
		// Message present, process stack message
    	GatewayCentral_processRoleEvent((gapCentralRoleEvent_t **)(&GC_StackMsg_t.pData));
	   if(GC_StackMsg_t.pData)
	   {
		   free(GC_StackMsg_t.pData);
	   }
    }

    return 0;
}


/*********************************************************************
*********************************************************************/
