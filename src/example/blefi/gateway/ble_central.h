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

#ifndef BLE_CENTRAL_H
#define BLE_CENTRAL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

#include "gap.h"

/*********************************************************************
 * CONSTANTS
 */

/** @defgroup GAPCENTRALROLE_PROFILE_PARAMETERS GAP Central Role Parameters
 * @{
 */
#define GAPCENTRALROLE_IRK                 0x400  //!< Identity Resolving Key. Read/Write. Size isunsigned char[KEYLEN]. Default is all 0, which means that the IRK will be randomly generated.
#define GAPCENTRALROLE_SRK                 0x401  //!< Signature Resolving Key. Read/Write. Size isunsigned char[KEYLEN]. Default is all 0, which means that the SRK will be randomly generated.
#define GAPCENTRALROLE_SIGNCOUNTER         0x402  //!< Sign Counter. Read/Write. Size is unsigned long. Default is 0.
#define GAPCENTRALROLE_BD_ADDR             0x403  //!< Device's Address. Read Only. Size isunsigned char[B_ADDR_LEN]. This item is read from the controller.
#define GAPCENTRALROLE_MAX_SCAN_RES        0x404  //!< Maximum number of discover scan results to receive. Default is 0 = unlimited.
/** @} End GAPCENTRALROLE_PROFILE_PARAMETERS */

/**
 * Number of simultaneous links with periodic RSSI reads
 */
#ifndef GAPCENTRALROLE_NUM_RSSI_LINKS
#define GAPCENTRALROLE_NUM_RSSI_LINKS     4
#endif

/*********************************************************************
 * VARIABLES
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * TYPEDEFS
 */

/**
 * Central Event Structure
 */
typedef union __attribute__((__packed__))
{
  gapEventHdr_t             gap;                //!< GAP_MSG_EVENT and status.
  gapDeviceInitDoneEvent_t  initDone;           //!< GAP initialization done.
  gapDeviceInfoEvent_t      deviceInfo;         //!< Discovery device information event structure.
  gapDevDiscEvent_t         discCmpl;           //!< Discovery complete event structure.
  gapEstLinkReqEvent_t      linkCmpl;           //!< Link complete event structure.
  gapLinkUpdateEvent_t      linkUpdate;         //!< Link update event structure.
  gapTerminateLinkEvent_t   linkTerminate;      //!< Link terminated event structure.
  gapPasskeyNeededEvent_t   passKeyNeeded;      //!< Pass key needed info structure
  gapAuthCompleteEvent_t    authComplete;       //!< Authentication complete event structure
  gapBondCompleteEvent_t    bondComplete;       //!< Bond complete event structure
  gapSignUpdateEvent_t      signUpdate;         //!< Sign update event structure
  gapSlaveSecurityReqEvent_t slaveSecReq;       //!< Slave Security Request Event
} gapCentralRoleEvent_t;



/*** Generic Status Return Values ***/
//#define SUCCESS                   0x00
//#define FAILURE                   0x01
#define INVALIDPARAMETER          0x02
#define INVALID_TASK              0x03
#define MSG_BUFFER_NOT_AVAIL      0x04
#define INVALID_MSG_POINTER       0x05
#define INVALID_EVENT_ID          0x06
#define INVALID_INTERRUPT_ID      0x07
#define NO_TIMER_AVAIL            0x08
#define NV_ITEM_UNINIT            0x09
#define NV_OPER_FAILED            0x0A
#define INVALID_MEM_SIZE          0x0B
#define NV_BAD_ITEM_LEN           0x0C
/*********************************************************************
 * VARIABLES
 */

/*********************************************************************
 * API FUNCTIONS
 */

/*-------------------------------------------------------------------
 * Central Profile Public APIs
 */

/**
 * @defgroup CENTRAL_PROFILE_API Central Profile API Functions
 *
 * @{
 */

/**
 * @brief   Start the device in Central role.  This function is typically
 *          called once during system startup.
 *
 * @param   pAppCallbacks - pointer to application callbacks
 *
 * @return  SUCCESS: Operation successful.<BR>
 *          bleAlreadyInRequestedMode: Device already started.<BR>
 */
extern bStatus_t GAPCentralRole_StartDevice(void);

/**
 * @brief   Set a parameter in the Central Profile.
 *
 * @param   param - profile parameter ID: @ref GAPCENTRALROLE_PROFILE_PARAMETERS
 * @param   len - length of data to write
 * @param   pValue - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type.
 *
 * @return  SUCCESS: Operation successful.<BR>
 *          INVALIDPARAMETER: Invalid parameter ID.<BR>
 */
extern bStatus_t GAPCentralRole_SetParameter(unsigned short int param,unsigned char len, void *pValue);

/**
 * @brief   Get a parameter in the Central Profile.
 *
 * @param   param - profile parameter ID: @ref GAPCENTRALROLE_PROFILE_PARAMETERS
 * @param   pValue - pointer to buffer to contain the read data
 *
 * @return  SUCCESS: Operation successful.<BR>
 *          INVALIDPARAMETER: Invalid parameter ID.<BR>
 */
extern bStatus_t GAPCentralRole_GetParameter(unsigned short int param, void *pValue);

/**
 * @brief   Terminate a link.
 *
 * @param   connHandle - connection handle of link to terminate
 *          or @ref GAP_CONN_HANDLE_DEFINES
 *
 * @return  SUCCESS: Terminate started.<BR>
 *          bleIncorrectMode: No link to terminate.<BR>
 */
extern bStatus_t GAPCentralRole_TerminateLink(unsigned short int connHandle);

/**
 * @brief   Establish a link to a peer device.
 *
 * @param   highDutyCycle -  TRUE to high duty cycle scan, FALSE if not
 * @param   whiteList - determines use of the white list: @ref GAP_WHITELIST_DEFINES
 * @param   addrTypePeer - address type of the peer device: @ref GAP_ADDR_TYPE_DEFINES
 * @param   peerAddr - peer device address
 *
 * @return  SUCCESS: started establish link process.<BR>
 *          bleIncorrectMode: invalid profile role.<BR>
 *          bleNotReady: a scan is in progress.<BR>
 *          bleAlreadyInRequestedMode: can�t process now.<BR>
 *          bleNoResources: too many links.<BR>
 */
extern bStatus_t GAPCentralRole_EstablishLink(unsigned char highDutyCycle,unsigned char whiteList,
                                             unsigned char addrTypePeer,unsigned char *peerAddr);

/**
 * @brief   Update the link connection parameters.
 *
 * @param   connHandle - connection handle
 * @param   connIntervalMin - minimum connection interval in 1.25ms units
 * @param   connIntervalMax - maximum connection interval in 1.25ms units
 * @param   connLatency - number of LL latency connection events
 * @param   connTimeout - connection timeout in 10ms units
 *
 * @return  SUCCESS: Connection update started started.<BR>
 *          bleIncorrectMode: No connection to update.<BR>
 */
extern bStatus_t GAPCentralRole_UpdateLink(unsigned short int connHandle, unsigned short int connIntervalMin,
                                           unsigned short int connIntervalMax, unsigned short int connLatency,
                                           unsigned short int connTimeout);
/**
 * @brief   Start a device discovery scan.
 *
 * @param   mode - discovery mode: @ref GAP_DEVDISC_MODE_DEFINES
 * @param   activeScan - TRUE to perform active scan
 * @param   whiteList - TRUE to only scan for devices in the white list
 *
 * @return  SUCCESS: Discovery scan started.<BR>
 *          bleIncorrectMode: Invalid profile role.<BR>
 *          bleAlreadyInRequestedMode: Not available.<BR>
 */
extern bStatus_t GAPCentralRole_StartDiscovery(unsigned char mode,unsigned char activeScan,unsigned char whiteList);

/**
 * @brief   Cancel a device discovery scan.
 *
 * @return  SUCCESS: Cancel started.<BR>
 *          bleInvalidTaskID: Not the task that started discovery.<BR>
 *          bleIncorrectMode: Not in discovery mode.<BR>
 */
extern bStatus_t GAPCentralRole_CancelDiscovery(void);

/**
 * @brief   Start periodic RSSI reads on a link.
 *
 * @param   connHandle - connection handle of link
 * @param   period - RSSI read period in ms
 *
 * @return  SUCCESS: Terminate started.<BR>
 *          bleIncorrectMode: No link.<BR>
 *          bleNoResources: No resources.<BR>
 */
extern bStatus_t GAPCentralRole_StartRssi(unsigned short int connHandle, unsigned short int period);

/**
 * @brief   Cancel periodic RSSI reads on a link.
 *
 * @param   connHandle - connection handle of link
 *
 * @return  SUCCESS: Operation successful.<BR>
 *          bleIncorrectMode: No link.<BR>
 */
extern bStatus_t GAPCentralRole_CancelRssi(unsigned short int connHandle);

/**
 * @}
 */

/*-------------------------------------------------------------------
 * TASK FUNCTIONS - Don't call these. These are system functions.
 */
   
/**
 * @brief       Task creation function for the GAP Central Role.
 *
 * @param       none
 *
 * @return      none
 */
extern void GAPCentralRole_createTask(void);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* BLE_CENTRAL_H */