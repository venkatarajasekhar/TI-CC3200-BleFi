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


/*******************************************************************************
  Filename:       hci.h
  Description:    This file contains the Host Controller Interface (HCI) API.
                  It provides the defines, types, and functions for all
                  supported Bluetooth Low Energy (BLE) commands.

                  All Bluetooth and BLE commands are based on:
                  Bluetooth Core Specification, V4.0.0, Vol. 2, Part E.

*******************************************************************************/

#ifndef __HCI_H__
#define __HCI_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "datatypes.h"

/*******************************************************************************
 * INCLUDES
 */

// ******** Definitions ******************************************************//
//
// Command opcodes
//
// GAP
#define GAP_DEVICEINIT             		0xFE00
#define GAP_CONFIGUREDEVICEADDR         0xFE03
#define GAP_DEVICEDISCOVERYREQUEST      0xFE04
#define GAP_DEVICEDISCOVERYCANCEL       0xFE05
#define GAP_MAKEDISCOVERABLE       		0xFE06
#define GAP_UPDATEADVERTISINGDATA  		0xFE07
#define GAP_ENDDISCOVERABLE             0xFE08
#define GAP_ESTABLISHLINKREQUEST        0xFE09
#define GAP_TERMINATELINKREQUEST        0xFE0A
#define GAP_AUTHENTICATE                0xFE0B
#define GAP_PASSKEY_UPDATE              0xFE0C
#define GAP_SLAVE_SECURITY_REQ_UPDATE   0xFE0D
#define GAP_SIGNABLE                    0xFE0E
#define GAP_BOND                        0xFE0F
#define GAP_TERMINATEAUTH               0xFE10
#define GAP_UPDATELINKPARAMREQUEST      0xFE11
#define GAP_SETPARAMETER                0xFE30
#define GAP_GETPARAMETER                0xFE31
#define GAP_RESOLVE_PRIVATE_ADDR        0xFE32
#define GAPBONDMGR_SETPARAMETER         0xFE36
#define GAPBONDMGR_GETPARAMETER         0xFE37

//GATT
#define GATT_DISCAPRIMARYSERVICEBYUUID  0xFD86
#define GATT_DISCALLPRIMARYSERVICES     0xFD90
#define GATT_FINDINCLUDEDSERVICES       0XFDB0
#define GATT_DISALLCHARS                0XFDB2
#define GATT_DISCCHARSBYUUID            0XFD88
#define GATT_DISCALLCHARDESCS           0XFD84
#define GATT_READCHARVALUE              0XFD8A
#define GATT_READUSINGCHARUUID          0XFDB4
#define GATT_READLONGCHARVALUE          0XFD8C
#define GATT_READMULTICHARVALUES        0XFD8E
#define GATT_WRITENORSP                 0XFDB6
#define GATT_SIGNEDWRITENORSP           0XFDB8
#define GATT_WRITECHARVALUE             0XFD92
#define GATT_WRITELONGCHARVALUE         0XFD96
#define GATT_NOTIFICATION               0XFD9B
#define GATT_INDICATION                 0XFD9D
#define GATT_ADDSERVICE              	0xFDFC
#define GATT_ADDATTRIBUTE            	0xFDFE
//ATT
#define ATT_CMD_ERRORRSP				0xFD01
#define ATT_CMD_READREQ					0xFD0A
#define ATT_CMD_READRSP					0xFD0B
#define ATT_CMD_WRITEREQ				0xFD12
#define ATT_CMD_WRITERSP				0xFD13
#define ATT_CMD_FINDBYTYPEVALUERSP		0xFD07
#define ATT_CMD_READBYTYPERSP			0xFD09
#define ATT_CMD_FINDINFOREQ             0xFD04
#define ATT_CMD_FINDINFORSP				0xFD05
#define ATT_CMD_READBYGRPTYPERSP		0xFD11
#define ATT_CMD_HANDLEVALUENOTI			0xFD1B
#define ATT_CMD_HANDLEVALUEIND			0xFD1D

//
// Event opcodes
//
// GAP
#define GAP_DEVICEINITDONE            	0x0600
#define GAP_ADVERTDATAUPDATEDONE    	0x0602
#define GAP_MAKEDISCOVERABLEDONE       	0x0603
#define GAP_ESTABLISHLINK              	0x0605
#define GAP_LINKTERMINATED				0x0606
#define GAP_HCI_EXTENTIONCOMMANDSTATUS 	0x067F

//ATT
#define ATT_EVENT_READREQ				0x050A
#define ATT_EVENT_WRITEREQ				0x0512
#define ATT_EVENT_FINDBYTYPEVALUEREQ	0x0506
#define ATT_EVENT_READBYTYPEREQ			0x0508
#define ATT_EVENT_FINDINFOREQ			0x0504
#define ATT_EVENT_READBYGRPTYPEREQ		0x0510
#define ATT_EVENT_READRSP				0x050B
#define ATT_EVENT_READBYGRPTYPERSP		0x0511

#ifndef B_ADDR_LEN
#define B_ADDR_LEN 0X06
#endif
/*
** HCI Status
**
** Per the Bluetooth Core Specification, V4.0.0, Vol. 2, Part D.
*/
#define HCI_SUCCESS                                                         0x00
#define HCI_ERROR_CODE_UNKNOWN_HCI_CMD                                      0x01
#define HCI_ERROR_CODE_UNKNOWN_CONN_ID                                      0x02
#define HCI_ERROR_CODE_HW_FAILURE                                           0x03
#define HCI_ERROR_CODE_PAGE_TIMEOUT                                         0x04
#define HCI_ERROR_CODE_AUTH_FAILURE                                         0x05
#define HCI_ERROR_CODE_PIN_KEY_MISSING                                      0x06
#define HCI_ERROR_CODE_MEM_CAP_EXCEEDED                                     0x07
#define HCI_ERROR_CODE_CONN_TIMEOUT                                         0x08
#define HCI_ERROR_CODE_CONN_LIMIT_EXCEEDED                                  0x09
#define HCI_ERROR_CODE_SYNCH_CONN_LIMIT_EXCEEDED                            0x0A
#define HCI_ERROR_CODE_ACL_CONN_ALREADY_EXISTS                              0x0B
#define HCI_ERROR_CODE_CMD_DISALLOWED                                       0x0C
#define HCI_ERROR_CODE_CONN_REJ_LIMITED_RESOURCES                           0x0D
#define HCI_ERROR_CODE_CONN_REJECTED_SECURITY_REASONS                       0x0E
#define HCI_ERROR_CODE_CONN_REJECTED_UNACCEPTABLE_BDADDR                    0x0F
#define HCI_ERROR_CODE_CONN_ACCEPT_TIMEOUT_EXCEEDED                         0x10
#define HCI_ERROR_CODE_UNSUPPORTED_FEATURE_PARAM_VALUE                      0x11
#define HCI_ERROR_CODE_INVALID_HCI_CMD_PARAMS                               0x12
#define HCI_ERROR_CODE_REMOTE_USER_TERM_CONN                                0x13
#define HCI_ERROR_CODE_REMOTE_DEVICE_TERM_CONN_LOW_RESOURCES                0x14
#define HCI_ERROR_CODE_REMOTE_DEVICE_TERM_CONN_POWER_OFF                    0x15
#define HCI_ERROR_CODE_CONN_TERM_BY_LOCAL_HOST                              0x16
#define HCI_ERROR_CODE_REPEATED_ATTEMPTS                                    0x17
#define HCI_ERROR_CODE_PAIRING_NOT_ALLOWED                                  0x18
#define HCI_ERROR_CODE_UNKNOWN_LMP_PDU                                      0x19
#define HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE                           0x1A
#define HCI_ERROR_CODE_SCO_OFFSET_REJ                                       0x1B
#define HCI_ERROR_CODE_SCO_INTERVAL_REJ                                     0x1C
#define HCI_ERROR_CODE_SCO_AIR_MODE_REJ                                     0x1D
#define HCI_ERROR_CODE_INVALID_LMP_PARAMS                                   0x1E
#define HCI_ERROR_CODE_UNSPECIFIED_ERROR                                    0x1F
#define HCI_ERROR_CODE_UNSUPPORTED_LMP_PARAM_VAL                            0x20
#define HCI_ERROR_CODE_ROLE_CHANGE_NOT_ALLOWED                              0x21
#define HCI_ERROR_CODE_LMP_LL_RESP_TIMEOUT                                  0x22
#define HCI_ERROR_CODE_LMP_ERR_TRANSACTION_COLLISION                        0x23
#define HCI_ERROR_CODE_LMP_PDU_NOT_ALLOWED                                  0x24
#define HCI_ERROR_CODE_ENCRYPT_MODE_NOT_ACCEPTABLE                          0x25
#define HCI_ERROR_CODE_LINK_KEY_CAN_NOT_BE_CHANGED                          0x26
#define HCI_ERROR_CODE_REQ_QOS_NOT_SUPPORTED                                0x27
#define HCI_ERROR_CODE_INSTANT_PASSED                                       0x28
#define HCI_ERROR_CODE_PAIRING_WITH_UNIT_KEY_NOT_SUPPORTED                  0x29
#define HCI_ERROR_CODE_DIFFERENT_TRANSACTION_COLLISION                      0x2A
#define HCI_ERROR_CODE_RESERVED1                                            0x2B
#define HCI_ERROR_CODE_QOS_UNACCEPTABLE_PARAM                               0x2C
#define HCI_ERROR_CODE_QOS_REJ                                              0x2D
#define HCI_ERROR_CODE_CHAN_ASSESSMENT_NOT_SUPPORTED                        0x2E
#define HCI_ERROR_CODE_INSUFFICIENT_SECURITY                                0x2F
#define HCI_ERROR_CODE_PARAM_OUT_OF_MANDATORY_RANGE                         0x30
#define HCI_ERROR_CODE_RESERVED2                                            0x31
#define HCI_ERROR_CODE_ROLE_SWITCH_PENDING                                  0x32
#define HCI_ERROR_CODE_RESERVED3                                            0x33
#define HCI_ERROR_CODE_RESERVED_SLOT_VIOLATION                              0x34
#define HCI_ERROR_CODE_ROLE_SWITCH_FAILED                                   0x35
#define HCI_ERROR_CODE_EXTENDED_INQUIRY_RESP_TOO_LARGE                      0x36
#define HCI_ERROR_CODE_SIMPLE_PAIRING_NOT_SUPPORTED_BY_HOST                 0x37
#define HCI_ERROR_CODE_HOST_BUSY_PAIRING                                    0x38
#define HCI_ERROR_CODE_CONN_REJ_NO_SUITABLE_CHAN_FOUND                      0x39
#define HCI_ERROR_CODE_CONTROLLER_BUSY                                      0x3A
#define HCI_ERROR_CODE_UNACCEPTABLE_CONN_INTERVAL                           0x3B
#define HCI_ERROR_CODE_DIRECTED_ADV_TIMEOUT                                 0x3C
#define HCI_ERROR_CODE_CONN_TERM_MIC_FAILURE                                0x3D
#define HCI_ERROR_CODE_CONN_FAILED_TO_ESTABLISH                             0x3E
#define HCI_ERROR_CODE_MAC_CONN_FAILED                                      0x3F

// Disconnect Reasons
#define HCI_DISCONNECT_AUTH_FAILURE                    HCI_ERROR_CODE_AUTH_FAILURE
#define HCI_DISCONNECT_REMOTE_USER_TERM                HCI_ERROR_CODE_REMOTE_USER_TERM_CONN
#define HCI_DISCONNECT_REMOTE_DEV_LOW_RESOURCES        HCI_ERROR_CODE_REMOTE_DEVICE_TERM_CONN_LOW_RESOURCES
#define HCI_DISCONNECT_REMOTE_DEV_POWER_OFF            HCI_ERROR_CODE_REMOTE_DEVICE_TERM_CONN_POWER_OFF
#define HCI_DISCONNECT_UNSUPPORTED_REMOTE_FEATURE      HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE
#define HCI_DISCONNECT_KEY_PAIRING_NOT_SUPPORTED       HCI_ERROR_CODE_PAIRING_WITH_UNIT_KEY_NOT_SUPPORTED
#define HCI_DISCONNECT_UNACCEPTABLE_CONN_INTERVAL      HCI_ERROR_CODE_UNACCEPTABLE_CONN_INTERVAL


// Device Address Type
#define HCI_PUBLIC_DEVICE_ADDRESS                      0
#define HCI_RANDOM_DEVICE_ADDRESS                      1

// Host Flow Control
#define HCI_CTRL_TO_HOST_FLOW_CTRL_OFF                 0
#define HCI_CTRL_TO_HOST_FLOW_CTRL_ACL_ON_SYNCH_OFF    1
#define HCI_CTRL_TO_HOST_FLOW_CTRL_ACL_OFF_SYNCH_ON    2
#define HCI_CTRL_TO_HOST_FLOW_CTRL_ACL_ON_SYNCH_ON     3
#define 	CONNECTION_HANDLE_0				0x0000
#define		LOOPBACK_CONNECTION				0xFFFE

//
/*******************************************************************************
 * TYPEDEFS
 */
// Packet Type
//
typedef unsigned char hciStatus_t;

// HCI Status type success
//
typedef enum HCI_StatusCodes_t
{
	SUCCESS                       = 0x00,
	FAILURE                       = 0x01,
	InvalidParameter              = 0x02,
	InvalidTask                   = 0x03,
	MsgBufferNotAvailable         = 0x04,
	InvalidMsgPointer             = 0x05,
	InvalidEventId                = 0x06,
	InvalidInteruptId             = 0x07,
	NoTimerAvail                  = 0x08,
	NVItemUnInit                  = 0x09,
	NVOpFailed                    = 0x0A,
	InvalidMemSize                = 0x0B,
	ErrorCommandDisallowed        = 0x0C,

	bleNotReady                   = 0x10,   // Not ready to perform task
	bleAlreadyInRequestedMode     = 0x11,   // Already performing that task
	bleIncorrectMode              = 0x12,   // Not setup properly to perform that task
	bleMemAllocError              = 0x13,   // Memory allocation error occurred
	bleNotConnected               = 0x14,   // Can't perform function when not in a connection
	bleNoResources                = 0x15,   // There are no resource available
	blePending                    = 0x16,   // Waiting
	bleTimeout                    = 0x17,   // Timed out performing function
	bleInvalidRange               = 0x18,   // A parameter is out of range
	bleLinkEncrypted              = 0x19,   // The link is already encrypted
	bleProcedureComplete          = 0x1A,   // The Procedure is completed

	// GAP Status Return Values - returned as bStatus_t
	bleGAPUserCanceled            = 0x30,   // The user canceled the task
	bleGAPConnNotAcceptable       = 0x31,   // The connection was not accepted
	bleGAPBondRejected            = 0x32,   // The bound information was rejected.

	// ATT Status Return Values - returned as bStatus_t
	bleInvalidPDU                 = 0x40,   // The attribute PDU is invalid
	bleInsufficientAuthen         = 0x41,   // The attribute has insufficient authentication
	bleInsufficientEncrypt        = 0x42,   // The attribute has insufficient encryption
	bleInsufficientKeySize        = 0x43,   // The attribute has insufficient encryption key size

	// L2CAP Status Return Values - returned as bStatus_t
	INVALID_TASK_ID               = 0xFF    // Task ID isn't setup properly
}HCI_StatusCodes_t;

//
// Packet Type
//
typedef enum
{
  COMMAND     = 0x01,
  ASYNCDATA   = 0x02,
  SYNCDATA    = 0x03,
  EVENT       = 0x04
}PacketType_t;

/*
** LE Events
*/

// ******** Struct ****************************************************************** //
//
// Command Header
//
typedef struct
{
  unsigned char PacketType;
  unsigned short int OpCode;
  unsigned char DataLength;
}CommandHeader_t;

typedef struct
{
	unsigned char  packet_type;
	unsigned char  event_code;
	unsigned char  data_length;
} EventHeader_t;


typedef struct
{
  bStatus_t Status;
  unsigned short int OpCode;
  unsigned char DataLen;
  unsigned char* PayLoad;
}CommandStatus_t;


// LE Connection Complete Event
typedef struct
{
  unsigned char  BLEEventCode;
  unsigned char  status;
  unsigned short int connectionHandle;
  unsigned char  role;
  unsigned char  peerAddrType;
  unsigned char  peerAddr[B_ADDR_LEN];
  unsigned short int connInterval;
  unsigned short int connLatency;
  unsigned short int connTimeout;
  unsigned char  clockAccuracy;
} hciEvt_BLEConnComplete_t;

// LE Advertising Report Event
typedef struct
{
  unsigned char  eventType;                       // advertisment or scan response event type
  unsigned char  addrType;                        // public or random address type
  unsigned char  addr[B_ADDR_LEN];                // device address
  unsigned char  dataLen;                         // length of report data
  unsigned char  rspData[B_MAX_ADV_LEN];          // report data given by dataLen
  unsigned char  rssi;                            // report RSSI
} hciEvt_DevInfo_t;

typedef struct
{
  unsigned char  BLEEventCode;
  unsigned char  numDevices;
  hciEvt_DevInfo_t* devInfo;              // pointer to the array of devInfo
} hciEvt_BLEAdvPktReport_t;

// LE Connection Update Complete Event
typedef struct
{
  unsigned char  BLEEventCode;
  unsigned char  status;
  unsigned short int connectionHandle;
  unsigned short int connInterval;
  unsigned short int connLatency;
  unsigned short int connTimeout;
} hciEvt_BLEConnUpdateComplete_t;

// LE Read Remote Used Features Complete Event
typedef struct
{
  unsigned char  BLEEventCode;
  unsigned char  status;
  unsigned short int connectionHandle;
  unsigned char  features[8];
} hciEvt_BLEReadRemoteFeatureComplete_t;

// LE Encryption Change Event
typedef struct
{
  unsigned char  BLEEventCode;
  unsigned short int connHandle;
  unsigned char  reason;
  unsigned char  encEnable;
} hciEvt_EncryptChange_t;

// LE Long Term Key Requested Event
typedef struct
{
  unsigned char  BLEEventCode;
  unsigned short int connHandle;
  unsigned char  random[8];
  unsigned short int encryptedDiversifier;
} hciEvt_BLELTKReq_t;

// Number of Completed Packets Event
typedef struct
{
  unsigned char  numHandles;
  unsigned short int *pConnectionHandle;              // pointer to the connection handle array
  unsigned short int *pNumCompletedPackets;           // pointer to the number of completed packets array
} hciEvt_NumCompletedPkt_t;

// Command Complete Event
typedef struct
{
  unsigned char  numHciCmdPkt;                    // number of HCI Command Packet
  unsigned short int cmdOpcode;
  unsigned char  *pReturnParam;                    // pointer to the return parameter
} hciEvt_CmdComplete_t;

// Command Status Event
typedef struct
{
  unsigned char  cmdStatus;
  unsigned char  numHciCmdPkt;
  unsigned short int cmdOpcode;
} hciEvt_CommandStatus_t;

// Hardware Error Event
typedef struct
{
  unsigned char hardwareCode;
} hciEvt_HardwareError_t;

// Disconnection Complete Event
typedef struct
{
  unsigned char  status;
  unsigned short int connHandle;                      // connection handle
  unsigned char  reason;
} hciEvt_DisconnComplete_t;

// Data Buffer Overflow Event
typedef struct
{
  unsigned char linkType;                         // synchronous or asynchronous buffer overflow
} hciEvt_BufferOverflow_t;

// Data structure for HCI Command Complete Event Return Parameter
typedef struct
{
  unsigned char  status;
  unsigned short int dataPktLen;
  unsigned char  numDataPkts;
} hciRetParam_LeReadBufSize_t;

typedef struct
{
  unsigned char            *pData;
} hciPacket_t;

typedef struct
{
  unsigned char  pktType;
  unsigned short int connHandle;
  unsigned char  pbFlag;
  unsigned short int pktLen;
  unsigned char  *pData;
} hciDataPacket_t;

// OSAL HCI_DATA_EVENT message format. This message is used to forward incoming
// data messages up to an application
typedef struct
{
  unsigned short int connHandle;                      // connection handle
  unsigned char  pbFlag;                          // data packet boundary flag
  unsigned short int len;                             // length of data packet
  unsigned char  *pData;                          // data packet given by len
} hciDataEvent_t;



// Function Prototypes *******************************************************//

bStatus_t HCI_commandStatus(unsigned char* packet, unsigned short int* op_code_received);

bStatus_t HCI_decodeHeader(unsigned char* packet, CommandHeader_t* header);

/*******************************************************************************
 * @fn          HCI_bm_alloc API
 *
 * @brief       This API is used to allocate memory using buffer management.
 *
 *              Note: This function should never be called by the application.
 *                    It is only used by HCI and L2CAP_bm_alloc.
 *
 * input parameters
 *
 * @param       size - Number of uint8s to allocate from the heap.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      Pointer to buffer, or NULL.
 */
extern void *HCI_bm_alloc( unsigned short int size );


/*******************************************************************************
 * @fn          HCI_ValidConnTimeParams API
 *
 * @brief       This API is used to check that the connection time parameter
 *              ranges are valid, and that the connection time parameter
 *              combination is valid.
 *
 *              Note: Only connIntervalMax is used as part of the time parameter
 *                    combination check.
 *
 * input parameters
 *
 * @param       connIntervalMin - Minimum connection interval.
 * @param       connIntervalMax - Maximum connection interval.
 * @param       connLatency     - Connection slave latency.
 * @param       connTimeout     - Connection supervision timeout.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      TRUE:  Connection time parameter check is valid.
 *              FALSE: Connection time parameter check is invalid.
 */
extern unsigned char HCI_ValidConnTimeParams( unsigned short int connIntervalMin,
                                      unsigned short int connIntervalMax,
                                      unsigned short int connLatency,
                                      unsigned short int connTimeout );


/*******************************************************************************
 * @fn          HCI_TestAppTaskRegister
 *
 * @brief       HCI vendor specific registration for HCI Test Application.
 *
 * input parameters
 *
 * @param       taskID - The HCI Test Application OSAL task identifer.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void HCI_TestAppTaskRegister( unsigned char taskID );


/*******************************************************************************
 * @fn          HCI_GAPTaskRegister
 *
 * @brief       HCI vendor specific registration for Host GAP.
 *
 * input parameters
 *
 * @param       taskID - The Host GAP OSAL task identifer.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void HCI_GAPTaskRegister( unsigned char taskID );


/*******************************************************************************
 *
 * @fn          HCI_L2CAPTaskRegister
 *
 * @brief       HCI vendor specific registration for Host L2CAP.
 *
 * input parameters
 *
 * @param       taskID - The Host L2CAP OSAL task identifer.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 *
 */
extern void HCI_L2CAPTaskRegister( unsigned char taskID );


/*******************************************************************************
 * @fn          HCI_SMPTaskRegister
 *
 * @brief       HCI vendor specific registration for Host SMP.
 *
 * input parameters
 *
 * @param       taskID - The Host SMP OSAL task identifer.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void HCI_SMPTaskRegister( unsigned char taskID );


/*******************************************************************************
 * @fn          HCI_ExtTaskRegister
 *
 * @brief       HCI vendor specific registration for Host extended commands.
 *
 * input parameters
 *
 * @param       taskID - The Host Extended Command OSAL task identifer.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void HCI_ExtTaskRegister( unsigned char taskID );


/*******************************************************************************
 * @fn          HCI_SendDataPkt API
 *
 * @brief       This API is used to send a ACL data packet over a connection.
 *
 *              Note: Empty packets are not sent.
 *
 *              Related Events: HCI_NumOfCompletedPacketsEvent
 *
 * input parameters
 *
 * @param       connHandle - Connection ID (handle).
 * @param       pbFlag     - Packet Boundary Flag.
 * @param       pktLen     - Number of uint8s of data to transmit.
 * @param       *pData     - Pointer to data buffer to transmit.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_SendDataPkt( unsigned short int connHandle,
                                    unsigned char  pbFlag,
                                    unsigned short int pktLen,
                                    unsigned char  *pData );


/*
** HCI API
*/

/*******************************************************************************
 * @fn          HCI_DisconnectCmd API
 *
 * @brief       This BT API is used to terminate a connection.
 *
 *              Related Events: HCI_CommandStatusEvent,
 *                              DisconnectEvent
 *
 * input parameters
 *
 * @param       connHandle - Connection handle.
 * @param       reason     - Reason for disconnection:
 *                           HCI_DISCONNECT_AUTH_FAILURE,
 *                           HCI_DISCONNECT_REMOTE_USER_TERM,
 *                           HCI_DISCONNECT_REMOTE_DEV_POWER_OFF,
 *                           HCI_DISCONNECT_UNSUPPORTED_REMOTE_FEATURE,
 *                           HCI_DISCONNECT_KEY_PAIRING_NOT_SUPPORTED
 *                           HCI_DISCONNECT_UNACCEPTABLE_CONN_INTERVAL
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_DisconnectCmd( unsigned short int connHandle,
                                      unsigned char  reason );


/*******************************************************************************
 * @fn          HCI_ReadRemoteVersionInfoCmd API
 *
 * @brief       This BT API is used to request version information from the
 *              the remote device in a connection.
 *
 *              Related Events: HCI_CommandStatusEvent,
 *                              ReadRemoteVersionInfoEvent
 *
 * input parameters
 *
 * @param       connHandle - Connection handle.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_ReadRemoteVersionInfoCmd( unsigned short int connHandle );


/*******************************************************************************
 * @fn          HCI_SetEventMaskCmd API
 *
 * @brief       This BT API is used to set the HCI event mask, which is used to
 *              determine which events are supported.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       pMask - Pointer to an eight unsigned char event mask.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_SetEventMaskCmd( unsigned char *pMask );


/*******************************************************************************
 * @fn          HCI_Reset API
 *
 * @brief       This BT API is used to reset the Link Layer.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_ResetCmd( void );


/*******************************************************************************
 * @fn          HCI_ReadTransmitPowerLevelCmd API
 *
 * @brief       This BT API is used to read the transmit power level.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       connHandle - Connection handle.
 * @param       txPwrType  - HCI_READ_CURRENT_TX_POWER_LEVEL,
 *                           HCI_READ_MAXIMUM_TX_POWER_LEVEL
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_ReadTransmitPowerLevelCmd( unsigned short int connHandle,
                                                  unsigned char  txPwrType );


/*******************************************************************************
 * @fn          HCI_SetControllerToHostFlowCtrlCmd API
 *
 * @brief       This BT API is used by the Host to turn flow control on or off
 *              for data sent from the Controller to Host.
 *
 *              Note: This command is currently not supported.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       flowControlEnable - HCI_CTRL_TO_HOST_FLOW_CTRL_OFF,
 *                                  HCI_CTRL_TO_HOST_FLOW_CTRL_ACL_ON_SYNCH_OFF,
 *                                  HCI_CTRL_TO_HOST_FLOW_CTRL_ACL_OFF_SYNCH_ON,
 *                                  HCI_CTRL_TO_HOST_FLOW_CTRL_ACL_ON_SYNCH_ON
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_SetControllerToHostFlowCtrlCmd( unsigned char flowControlEnable );


/*******************************************************************************
 * @fn          HCI_HostBufferSizeCmd API
 *
 * @brief       This BT API is used by the Host to notify the Controller of the
 *              maximum size ACL buffer size the Controller can send to the
 *              Host.
 *
 *              Note: This command is currently ignored by the Controller. It
 *                    is assumed that the Host can always handle the maximum
 *                    BLE data packet size.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       hostAclPktLen        - Host ACL data packet length.
 * @param       hostSyncPktLen       - Host SCO data packet length .
 * @param       hostTotalNumAclPkts  - Host total number of ACL data packets.
 * @param       hostTotalNumSyncPkts - Host total number of SCO data packets.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_HostBufferSizeCmd( unsigned short int hostAclPktLen,
                                          unsigned char  hostSyncPktLen,
                                          unsigned short int hostTotalNumAclPkts,
                                          unsigned short int hostTotalNumSyncPkts );


/*******************************************************************************
 * @fn          HCI_HostNumCompletedPktCmd API
 *
 * @brief       This BT API is used by the Host to notify the Controller of the
 *              number of HCI data packets that have been completed for each
 *              connection handle since this command was previously sent to the
 *              controller.
 *
 *              The Host_Number_Of_Conpleted_Packets command is a special
 *              command. No event is normally generated after the command
 *              has completed. The command should only be issued by the
 *              Host if flow control in the direction from controller to
 *              the host is on and there is at least one connection, or
 *              if the controller is in local loopback mode.
 *
 *              Note: It is assumed that there will be at most only one handle.
 *                    Even if more than one handle is provided, the Controller
 *                    does not track Host buffers as a function of connection
 *                    handles (and isn't required to do so).
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       numHandles       - Number of connection handles.
 * @param       connHandles      - Array of connection handles.
 * @param       numCompletedPkts - Array of number of completed packets.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_HostNumCompletedPktCmd( unsigned char  numHandles,
                                               unsigned short int *connHandles,
                                               unsigned short int *numCompletedPkts );


/*******************************************************************************
 * @fn          HCI_ReadLocalVersionInfoCmd API
 *
 * @brief       This BT API is used to read the local version information.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_ReadLocalVersionInfoCmd( void );


/*******************************************************************************
 * @fn          HCI_ReadLocalSupportedCommandsCmd API
 *
 * @brief       This BT API is used to read the locally supported commands.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_ReadLocalSupportedCommandsCmd( void );


/*******************************************************************************
 * @fn          HCI_ReadLocalSupportedFeaturesCmd API
 *
 * @brief       This BT API is used to read the locally supported features.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_ReadLocalSupportedFeaturesCmd( void );


/*******************************************************************************
 * @fn          HCI_ReadBDADDRCmd API
 *
 * @brief       This BT API is used to read this device's BLE address (BDADDR).
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_ReadBDADDRCmd( void );


/*******************************************************************************
 * @fn          HCI_ReadRssiCmd API
 *
 * @brief       This BT API is used to read the RSSI of the last packet
 *              received on a connection given by the connection handle. If
 *              the Receiver Modem test is running (HCI_EXT_ModemTestRx), then
 *              the RF RSSI for the last received data will be returned. If
 *              there is no RSSI value, then HCI_RSSI_NOT_AVAILABLE will be
 *              returned.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       connHandle - Connection handle.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_ReadRssiCmd( unsigned short int connHandle );

/*
** HCI Low Energy Commands
*/

/*******************************************************************************
 * @fn          HCI_LE_SetEventMaskCmd API
 *
 * @brief       This LE API is used to set the HCI LE event mask, which is used
 *              to determine which LE events are supported.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param        pEventMask - Pointer to LE event mask of 8 uint8s.

 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_SetEventMaskCmd( unsigned char *pEventMask );


/*******************************************************************************
 * @fn          HCI_LE_ReadBufSizeCmd API
 *
 * @brief       This LE API is used by the Host to determine the maximum ACL
 *              data packet size allowed by the Controller.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_ReadBufSizeCmd( void );


/*******************************************************************************
 * @fn          HCI_LE_ReadLocalSupportedFeaturesCmd API
 *
 * @brief       This LE API is used to read the LE locally supported features.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_ReadLocalSupportedFeaturesCmd( void );


/*******************************************************************************
 * @fn          HCI_LE_SetRandomAddressCmd API
 *
 * @brief       This LE API is used to set this device's Random address.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       pRandAddr - Pointer to random address.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_SetRandomAddressCmd( unsigned char *pRandAddr );


/*******************************************************************************
 * @fn          HCI_LE_SetAdvParamCmd API
 *
 * @brief       This LE API is used to set the Advertising parameters.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       advIntervalMin  - Minimum allowed advertising interval.
 * @param       advIntervalMax  - Maximum allowed advertising interval.
 * @param       advType         - HCI_CONNECTABLE_UNDIRECTED_ADV,
 *                                HCI_CONNECTABLE_DIRECTED_HDC_ADV,
 *                                HCI_SCANNABLE_UNDIRECTED,
 *                                HCI_NONCONNECTABLE_UNDIRECTED_ADV
 *                                HCI_CONNECTABLE_DIRECTED_LDC_ADV
 * @param       ownAddrType     - HCI_PUBLIC_DEVICE_ADDRESS,
 *                                HCI_RANDOM_DEVICE_ADDRESS
 * @param       directAddrType  - HCI_PUBLIC_DEVICE_ADDRESS,
 *                                HCI_RANDOM_DEVICE_ADDRESS
 * @param       directAddr      - Pointer to address of device when using
 *                                directed advertising.
 * @param       advChannelMap   - HCI_ADV_CHAN_37,
 *                                HCI_ADV_CHAN_38,
 *                                HCI_ADV_CHAN_39,
 *                                HCI_ADV_CHAN_37 | HCI_ADV_CHAN_38,
 *                                HCI_ADV_CHAN_37 | HCI_ADV_CHAN_39,
 *                                HCI_ADV_CHAN_38 | HCI_ADV_CHAN_39,
 *                                HCI_ADV_CHAN_ALL
 * @param       advFilterPolicy - HCI_ADV_WL_POLICY_ANY_REQ,
 *                                HCI_ADV_WL_POLICY_WL_SCAN_REQ,
 *                                HCI_ADV_WL_POLICY_WL_CONNECT_REQ,
 *                                HCI_ADV_WL_POLICY_WL_ALL_REQ
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_SetAdvParamCmd( unsigned short int advIntervalMin,
                                          unsigned short int advIntervalMax,
                                          unsigned char  advType,
                                          unsigned char  ownAddrType,
                                          unsigned char  directAddrType,
                                          unsigned char  *directAddr,
                                          unsigned char  advChannelMap,
                                          unsigned char  advFilterPolicy );


/*******************************************************************************
 * @fn          HCI_LE_SetAdvDataCmd API
 *
 * @brief       This LE API is used to set the Advertising data.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       dataLen - Length of Advertising data.
 * @param       pData   - Pointer to Advertising data.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_SetAdvDataCmd( unsigned char dataLen,
                                         unsigned char *pData );


/*******************************************************************************
 * @fn          HCI_LE_SetScanRspDataCmd API
 *
 * @brief       This LE API is used to set the Advertising Scan Response data.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       dataLen - Length of Scan Response data.
 * @param       pData   - Pointer to Scan Response data.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_SetScanRspDataCmd( unsigned char dataLen,
                                             unsigned char *pData );


/*******************************************************************************
 * @fn          HCI_LE_SetAdvEnableCmd API
 *
 * @brief       This LE API is used to turn Advertising on or off.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       advEnable - HCI_ENABLE_ADV, HCI_DISABLE_ADV
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_SetAdvEnableCmd( unsigned char advEnable );


/*******************************************************************************
 * @fn          HCI_LE_ReadAdvChanTxPowerCmd API
 *
 * @brief       This LE API is used to read transmit power when Advertising.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_ReadAdvChanTxPowerCmd( void );


/*******************************************************************************
 * @fn          HCI_LE_SetScanParamCmd API
 *
 * @brief       This LE API is used to set the Scan parameters.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       scanType     - HCI_SCAN_PASSIVE, HCI_SCAN_ACTIVE
 * @param       scanInterval - Time between scan events.
 * @param       scanWindow   - Time of scan before scan event ends.
 *                             Note: When the scanWindow equals the scanInterval
 *                                   then scanning is continuous.
 * @param       ownAddrType  - This device's address.
 * @param       filterPolicy - HCI_SCAN_PASSIVE, HCI_SCAN_ACTIVE
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_SetScanParamCmd( unsigned char  scanType,
                                           unsigned short int scanInterval,
                                           unsigned short int scanWindow,
                                           unsigned char  ownAddrType,
                                           unsigned char  filterPolicy );


/*******************************************************************************
 * @fn          HCI_LE_SetScanEnableCmd API
 *
 * @brief       This LE API is used to turn Scanning on or off.
 *
 *              Related Events: HCI_CommandCompleteEvent,
 *                              AdvReportEvent
 *
 * input parameters
 *
 * @param       scanEnable       - HCI_SCAN_START, HCI_SCAN_STOP
 * @param       filterDuplicates - HCI_FILTER_REPORTS_ENABLE,
 *                                 HCI_FILTER_REPORTS_DISABLE
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_SetScanEnableCmd( unsigned char scanEnable,
                                            unsigned char filterDuplicates );


/*******************************************************************************
 * @fn          HCI_LE_CreateConnCmd API
 *
 * @brief       This LE API is used to create a connection.
 *
 *              Related Events: HCI_CommandStatusEvent,
 *                              ConnectionCompleteEvent
 *
 * input parameters
 *
 * @param       scanInterval     - Time between Init scan events.
 * @param       scanWindow       - Time of scan before Init scan event ends.
 *                                 Note: When the scanWindow equals the
 *                                       scanInterval then scanning is
 *                                       continuous.
 * @param       initFilterPolicy - HCI_INIT_WL_POLICY_USE_PEER_ADDR,
 *                                 HCI_INIT_WL_POLICY_USE_WHITE_LIST
 * @param       addrTypePeer     - HCI_PUBLIC_DEVICE_ADDRESS,
 *                                 HCI_RANDOM_DEVICE_ADDRESS
 * @param       peerAddr         - Pointer to peer device's address.
 * @param       ownAddrType      - HCI_PUBLIC_DEVICE_ADDRESS,
 *                                 HCI_RANDOM_DEVICE_ADDRESS
 * @param       connIntervalMin  - Minimum allowed connection interval.
 * @param       connIntervalMax  - Maximum allowed connection interval.
 * @param       connLatency      - Number of skipped events (slave latency).
 * @param       connTimeout      - Connection supervision timeout.
 * @param       minLen           - Info parameter about min length of conn.
 * @param       maxLen           - Info parameter about max length of conn.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_CreateConnCmd( unsigned short int scanInterval,
                                         unsigned short int scanWindow,
                                         unsigned char  initFilterPolicy,
                                         unsigned char  addrTypePeer,
                                         unsigned char  *peerAddr,
                                         unsigned char  ownAddrType,
                                         unsigned short int connIntervalMin,
                                         unsigned short int connIntervalMax,
                                         unsigned short int connLatency,
                                         unsigned short int connTimeout,
                                         unsigned short int minLen,
                                         unsigned short int maxLen );


/*******************************************************************************
 * @fn          HCI_LE_CreateConnCancelCmd API
 *
 * @brief       This LE API is used to cancel a create connection.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_CreateConnCancelCmd( void );


/*******************************************************************************
 * @fn          HCI_LE_ReadWhiteListSizeCmd API
 *
 * @brief       This LE API is used to read the white list.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_ReadWhiteListSizeCmd( void );


/*******************************************************************************
 * @fn          HCI_LE_ClearWhiteListCmd API
 *
 * @brief       This LE API is used to clear the white list.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_ClearWhiteListCmd( void );


/*******************************************************************************
 * @fn          HCI_LE_AddWhiteListCmd API
 *
 * @brief       This LE API is used to add a white list entry.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       addrType - HCI_PUBLIC_DEVICE_ADDRESS, HCI_RANDOM_DEVICE_ADDRESS
 * @param       devAddr  - Pointer to address of device to put in white list.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_AddWhiteListCmd( unsigned char addrType,
                                           unsigned char *devAddr );


/*******************************************************************************
 * @fn          HCI_LE_RemoveWhiteListCmd API
 *
 * @brief       This LE API is used to remove a white list entry.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       addrType - HCI_PUBLIC_DEVICE_ADDRESS, HCI_RANDOM_DEVICE_ADDRESS
 * @param       devAddr  - Pointer to address of device to remove from the
 *                         white list.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_RemoveWhiteListCmd( unsigned char addrType,
                                              unsigned char *devAddr );


/*******************************************************************************
 * @fn          HCI_LE_ConnUpdateCmd API
 *
 * @brief       This LE API is used to update the connection parameters.
 *
 *              Related Events: HCI_CommandStatusEvent,
 *                              ConnectionUpdateCompleteEvent
 *
 * input parameters
 *
 * @param       connHandle       - Time between Init scan events.
 * @param       connIntervalMin  - Minimum allowed connection interval.
 * @param       connIntervalMax  - Maximum allowed connection interval.
 * @param       connLatency      - Number of skipped events (slave latency).
 * @param       connTimeout      - Connection supervision timeout.
 * @param       minLen           - Info parameter about min length of conn.
 * @param       maxLen           - Info parameter about max length of conn.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_ConnUpdateCmd( unsigned short int connHandle,
                                         unsigned short int connIntervalMin,
                                         unsigned short int connIntervalMax,
                                         unsigned short int connLatency,
                                         unsigned short int connTimeout,
                                         unsigned short int minLen,
                                         unsigned short int maxLen );


/*******************************************************************************
 * @fn          HCI_LE_SetHostChanClassificationCmd API
 *
 * @brief       This LE API is used to update the current data channel map.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       chanMap - Pointer to the new channel map.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_SetHostChanClassificationCmd( unsigned char *chanMap );


/*******************************************************************************
 * @fn          HCI_LE_ReadChannelMapCmd API
 *
 * @brief       This LE API is used to read a connection's data channel map.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       connHandle - Connection handle.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_ReadChannelMapCmd( unsigned short int connHandle );


/*******************************************************************************
 * @fn          HCI_LE_ReadRemoteUsedFeaturesCmd API
 *
 * @brief       This LE API is used to read the remote device's used features.
 *
 *              Related Events: HCI_CommandStatusEvent,
 *                              ReadRemoteUsedFeaturesCompleteEvent
 *
 * input parameters
 *
 * @param       connHandle - Connection handle.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_ReadRemoteUsedFeaturesCmd( unsigned short int connHandle );


/*******************************************************************************
 * @fn          HCI_LE_EncryptCmd API
 *
 * @brief       This LE API is used to perform an encryption using AES128.
 *
 *              Note: Input parameters are ordered MSB..LSB.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       key       - Pointer to 16 unsigned char encryption key.
 * @param       plainText - Pointer to 16 unsigned char plaintext data.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_EncryptCmd( unsigned char *key,
                                      unsigned char *plainText );


/*******************************************************************************
 * @fn          HCI_LE_RandCmd API
 *
 * @brief       This LE API is used to generate a random number.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_RandCmd( void );


/*******************************************************************************
 * @fn          HCI_LE_StartEncyptCmd API
 *
 * @brief       This LE API is used to start encryption in a connection.
 *
 *              Related Events: HCI_CommandStatusEvent,
 *                              EncChangeEvent or
 *                              EncKeyRefreshEvent
 *
 * input parameters
 *
 * @param       connHandle - Connection handle.
 * @param       random     - Pointer to eight unsigned char Random number.
 * @param       encDiv     - Pointer to two unsigned char Encrypted Diversifier.
 * @param       ltk        - Pointer to 16 unsigned char Long Term Key.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_StartEncyptCmd( unsigned short int connHandle,
                                          unsigned char  *random,
                                          unsigned char  *encDiv,
                                          unsigned char  *ltk );


/*******************************************************************************
 * @fn          HCI_LE_LtkReqReplyCmd API
 *
 * @brief       This LE API is used by the Host to send to the Controller a
 *              positive LTK reply.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       connHandle - Connection handle.
 * @param       ltk        - Pointer to 16 unsigned char Long Term Key.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_LtkReqReplyCmd( unsigned short int connHandle,
                                          unsigned char  *ltk );


/*******************************************************************************
 * @fn          HCI_LE_LtkReqNegReplyCmd API
 *
 * @brief       This LE API is used by the Host to send to the Controller a
 *              negative LTK reply.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       connHandle - Connectin handle.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_LtkReqNegReplyCmd( unsigned short int connHandle );


/*******************************************************************************
 * @fn          HCI_LE_ReadSupportedStatesCmd API
 *
 * @brief       This LE API is used to read the Controller's supported states.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_ReadSupportedStatesCmd( void );


/*******************************************************************************
 * @fn          HCI_LE_ReceiverTestCmd API
 *
 * @brief       This LE API is used to start the receiver Direct Test Mode test.
 *
 *              Note: A HCI reset should be issued when done using DTM!
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       rxFreq - Rx RF frequency:
 *                       k=0..HCI_DTM_NUMBER_RF_CHANS-1, where: F=2402+(k*2MHz)
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_ReceiverTestCmd( unsigned char rxFreq );


/*******************************************************************************
 * @fn          HCI_LE_TransmitterTestCmd API
 *
 * @brief       This LE API is used to start the transmit Direct Test Mode test.
 *
 *              Note: The BLE device is to transmit at maximum power!
 *
 *              Note: A HCI reset should be issued when done using DTM!
 *
 * input parameters
 *
 * @param       txFreq      - Tx RF frequency:
 *                            k=0..HCI_DTM_NUMBER_RF_CHANS-1, where:
 *                            F=2402+(k*2MHz)
 * @param       dataLen     - Test data length in uint8s:
 *                            0..HCI_DIRECT_TEST_MAX_PAYLOAD_LEN
 * @param       payloadType - Type of packet payload, per Direct Test Mode spec:
 *                            HCI_DIRECT_TEST_PAYLOAD_PRBS9,
 *                            HCI_DIRECT_TEST_PAYLOAD_0x0F,
 *                            HCI_DIRECT_TEST_PAYLOAD_0x55,
 *                            HCI_DIRECT_TEST_PAYLOAD_PRBS15,
 *                            HCI_DIRECT_TEST_PAYLOAD_0xFF,
 *                            HCI_DIRECT_TEST_PAYLOAD_0x00,
 *                            HCI_DIRECT_TEST_PAYLOAD_0xF0,
 *                            HCI_DIRECT_TEST_PAYLOAD_0xAA
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_TransmitterTestCmd( unsigned char txFreq,
                                              unsigned char dataLen,
                                              unsigned char pktPayload );


/*******************************************************************************
 * @fn          HCI_LE_TestEndCmd API
 *
 * @brief       This LE API is used to end the Direct Test Mode test.
 *
 *              Note: A HCI reset should be issued when done using DTM!
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_TestEndCmd( void );

/*
** HCI Vendor Specific Comamnds: Link Layer Extensions
*/

/*******************************************************************************
 * @fn          HCI_EXT_SetRxGainCmd API
 *
 * @brief       This HCI Extension API is used to set the receiver gain.
 *
 *              Related Events: HCI_VendorSpecifcCommandCompleteEvent
 *
 * input parameters
 *
 * @param       rxGain - HCI_EXT_RX_GAIN_STD, HCI_EXT_RX_GAIN_HIGH
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_EXT_SetRxGainCmd( unsigned char rxGain );


/*******************************************************************************
 * @fn          HCI_EXT_SetTxPowerCmd API
 *
 * @brief       This HCI Extension API is used to set the transmit power.
 *
 *              Related Events: HCI_VendorSpecifcCommandCompleteEvent
 *
 * input parameters
 *
 * @param       txPower - For CC254x: LL_EXT_TX_POWER_MINUS_23_DBM,
 *                                    LL_EXT_TX_POWER_MINUS_6_DBM,
 *                                    LL_EXT_TX_POWER_0_DBM,
 *                                    LL_EXT_TX_POWER_4_DBM
 *
 *                        For CC26xx: HCI_EXT_TX_POWER_MINUS_21_DBM,
 *                                    HCI_EXT_TX_POWER_MINUS_18_DBM,
 *                                    HCI_EXT_TX_POWER_MINUS_15_DBM,
 *                                    HCI_EXT_TX_POWER_MINUS_12_DBM,
 *                                    HCI_EXT_TX_POWER_MINUS_9_DBM,
 *                                    HCI_EXT_TX_POWER_MINUS_6_DBM,
 *                                    HCI_EXT_TX_POWER_MINUS_3_DBM,
 *                                    HCI_EXT_TX_POWER_0_DBM,
 *                                    HCI_EXT_TX_POWER_1_DBM,
 *                                    HCI_EXT_TX_POWER_2_DBM,
 *                                    HCI_EXT_TX_POWER_3_DBM,
 *                                    HCI_EXT_TX_POWER_4_DBM,
 *                                    HCI_EXT_TX_POWER_5_DBM
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_EXT_SetTxPowerCmd( unsigned char txPower );


/*******************************************************************************
 * @fn          HCI_EXT_OnePktPerEvtCmd API
 *
 * @brief       This HCI Extension API is used to set whether a connection will
 *              be limited to one packet per event.
 *
 *              Related Events: HCI_VendorSpecifcCommandCompleteEvent
 *
 * input parameters
 *
 * @param       control - HCI_EXT_ENABLE_ONE_PKT_PER_EVT,
 *                        HCI_EXT_DISABLE_ONE_PKT_PER_EVT
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_EXT_OnePktPerEvtCmd( unsigned char control );


/*******************************************************************************
 * @fn          HCI_EXT_ClkDivOnHaltCmd API
 *
 * @brief       This HCI Extension API is used to set whether the system clock
 *              will be divided when the MCU is halted.
 *
 *              Related Events: HCI_VendorSpecifcCommandCompleteEvent
 *
 * input parameters
 *
 * @param       control - HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT,
 *                        HCI_EXT_DISABLE_CLK_DIVIDE_ON_HALT
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_EXT_ClkDivOnHaltCmd( unsigned char control );


/*******************************************************************************
 * @fn          HCI_EXT_DeclareNvUsageCmd API
 *
 * @brief       This HCI Extension API is used to indicate to the Controller
 *              whether or not the Host will be using the NV memory during BLE
 *              operations.
 *
 *              Related Events: HCI_VendorSpecifcCommandCompleteEvent
 *
 * input parameters
 *
 * @param       mode - HCI_EXT_NV_IN_USE, HCI_EXT_NV_NOT_IN_USE
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_EXT_DeclareNvUsageCmd( unsigned char mode );


/*******************************************************************************
 * @fn          HCI_EXT_DecryptCmd API
 *
 * @brief       This HCI Extension API is used to decrypt encrypted data using
 *              AES128.
 *
 *              Related Events: HCI_VendorSpecifcCommandCompleteEvent
 *
 * input parameters
 *
 * @param       key     - Pointer to 16 unsigned char encryption key.
 * @param       encText - Pointer to 16 unsigned char encrypted data.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_EXT_DecryptCmd( unsigned char *key,
                                       unsigned char *encText );


/*******************************************************************************
 * @fn          HCI_EXT_SetLocalSupportedFeaturesCmd API
 *
 * @brief       This HCI Extension API is used to write this devie's supported
 *              features.
 *
 *              Related Events: HCI_VendorSpecifcCommandCompleteEvent
 *
 * input parameters
 *
 * @param       localFeatures - Pointer to eight uint8s of local features.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_EXT_SetLocalSupportedFeaturesCmd( unsigned char *localFeatures );


/*******************************************************************************
 * @fn          HCI_EXT_SetFastTxResponseTimeCmd API
 *
 * @brief       This HCI Extension API is used to set whether transmit data is
 *              sent as soon as possible even when slave latency is used.
 *
 *              Related Events: HCI_VendorSpecifcCommandCompleteEvent
 *
 * input parameters
 *
 * @param       control - HCI_EXT_ENABLE_FAST_TX_RESP_TIME,
 *                        HCI_EXT_DISABLE_FAST_TX_RESP_TIME
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_EXT_SetFastTxResponseTimeCmd( unsigned char control );


/*******************************************************************************
 * @fn          HCI_EXT_SetSlaveLatencyOverrideCmd API
 *
 * @brief       This HCI Extension API is used to to enable or disable
 *              suspending slave latency.
 *
 *              Related Events: HCI_VendorSpecifcCommandCompleteEvent
 *
 * input parameters
 *
 * @param       control - HCI_EXT_ENABLE_SL_OVERRIDE,
 *                        HCI_EXT_DISABLE_SL_OVERRIDE
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_EXT_SetSlaveLatencyOverrideCmd( unsigned char control );


/*******************************************************************************
 * @fn          HCI_EXT_ModemTestTxCmd
 *
 * @brief       This API is used start a continuous transmitter modem test,
 *              using either a modulated or unmodulated carrier wave tone, at
 *              the frequency that corresponds to the specified RF channel. Use
 *              HCI_EXT_EndModemTest command to end the test.
 *
 *              Note: A Controller reset will be issued by HCI_EXT_EndModemTest!
 *              Note: The BLE device will transmit at maximum power.
 *              Note: This API can be used to verify this device meets Japan's
 *                    TELEC regulations.
 *
 *              Related Events: HCI_VendorSpecifcCommandCompleteEvent
 *
 * input parameters
 *
 * @param       cwMode - HCI_EXT_TX_MODULATED_CARRIER,
 *                       HCI_EXT_TX_UNMODULATED_CARRIER
 *              txFreq - Transmit RF channel k=0..39, where BLE F=2402+(k*2MHz).
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_EXT_ModemTestTxCmd( unsigned char cwMode,
                                           unsigned char txFreq );


/*******************************************************************************
 * @fn          HCI_EXT_ModemHopTestTxCmd
 *
 * @brief       This API is used to start a continuous transmitter direct test
 *              mode test using a modulated carrier wave and transmitting a
 *              37 unsigned char packet of Pseudo-Random 9-bit data. A packet is
 *              transmitted on a different frequency (linearly stepping through
 *              all RF channels 0..39) every 625us. Use HCI_EXT_EndModemTest
 *              command to end the test.
 *
 *              Note: A Controller reset will be issued by HCI_EXT_EndModemTest!
 *              Note: The BLE device will transmit at maximum power.
 *              Note: This API can be used to verify this device meets Japan's
 *                    TELEC regulations.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_EXT_ModemHopTestTxCmd( void );


/*******************************************************************************
 * @fn          HCI_EXT_ModemTestRxCmd
 *
 * @brief       This API is used to start a continuous receiver modem test
 *              using a modulated carrier wave tone, at the frequency that
 *              corresponds to the specific RF channel. Any received data is
 *              discarded. Receiver gain may be adjusted using the
 *              HCI_EXT_SetRxGain command. RSSI may be read during this test
 *              by using the HCI_ReadRssi command. Use HCI_EXT_EndModemTest
 *              command to end the test.
 *
 *              Note: A Controller reset will be issued by HCI_EXT_EndModemTest!
 *              Note: The BLE device will transmit at maximum power.
 *
 * input parameters
 *
 * @param       rxFreq - Receiver RF channel k=0..39, where BLE F=2402+(k*2MHz).
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_EXT_ModemTestRxCmd( unsigned char rxFreq );


/*******************************************************************************
 * @fn          HCI_EXT_EndModemTestCmd
 *
 * @brief       This API is used to shutdown a modem test. A complete Controller
 *              reset will take place.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_EXT_EndModemTestCmd( void );


/*******************************************************************************
 * @fn          HCI_EXT_SetBDADDRCmd
 *
 * @brief       This API is used to set this device's BLE address (BDADDR).
 *
 *              Note: This command is only allowed when the device's state is
 *                    Standby.
 *
 *              Related Events: HCI_VendorSpecifcCommandCompleteEvent
 *
 * input parameters
 *
 * @param       bdAddr  - A pointer to a buffer to hold this device's address.
 *                        An invalid address (i.e. all FF's) will restore this
 *                        device's address to the address set at initialization.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_EXT_SetBDADDRCmd( unsigned char *bdAddr );


/*******************************************************************************
 * @fn          HCI_EXT_SetSCACmd
 *
 * @brief       This API is used to set this device's Sleep Clock Accuracy.
 *
 *              Note: For a slave device, this value is directly used, but only
 *                    if power management is enabled. For a master device, this
 *                    value is converted into one of eight ordinal values
 *                    representing a SCA range, as specified in Table 2.2,
 *                    Vol. 6, Part B, Section 2.3.3.1 of the Core specification.
 *
 *              Note: This command is only allowed when the device is not in a
 *                    connection.
 *
 *              Note: The device's SCA value remains unaffected by a HCI_Reset.
 *
 * input parameters
 *
 * @param       scaInPPM - A SCA value in PPM from 0..500.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_EXT_SetSCACmd( unsigned short int scaInPPM );


/*******************************************************************************
 * @fn          HCI_EXT_EnablePTMCmd
 *
 * @brief       This HCI Extension API is used to enable Production Test Mode.
 *
 *              Note: This function can only be directly called from the
 *                    application and is not available via an external transport
 *                    interface such as RS232. Also, no vendor specific
 *                    command complete will be returned.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_EXT_EnablePTMCmd( void );


/*******************************************************************************
 * @fn          HCI_EXT_SetFreqTuneCmd
 *
 * @brief       This HCI Extension API is used to set the frequency tuning up
 *              or down. Setting the mode up/down decreases/increases the amount
 *              of capacitance on the external crystal oscillator.
 *
 *              Note: This is a Production Test Mode only command!
 *
 * input parameters
 *
 * @param       step - HCI_PTM_SET_FREQ_TUNE_UP, HCI_PTM_SET_FREQ_TUNE_DOWN
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_EXT_SetFreqTuneCmd( unsigned char step );


/*******************************************************************************
 * @fn          HCI_EXT_SaveFreqTuneCmd
 *
 * @brief       This HCI Extension API is used to save the frequency tuning
 *              value to flash.
 *
 *              Note: This is a Production Test Mode only command!
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_EXT_SaveFreqTuneCmd( void );


/*******************************************************************************
 * @fn          HCI_EXT_SetMaxDtmTxPowerCmd API
 *
 * @brief       This HCI Extension API is used to set the maximum transmit
 *              output power for Direct Test Mode.
 *
 *              Related Events: HCI_VendorSpecifcCommandCompleteEvent
 *
 * input parameters
 *
 * @param       txPower - For CC254x: LL_EXT_TX_POWER_MINUS_23_DBM,
 *                                    LL_EXT_TX_POWER_MINUS_6_DBM,
 *                                    LL_EXT_TX_POWER_0_DBM,
 *                                    LL_EXT_TX_POWER_4_DBM
 *
 *                        For CC26xx: HCI_EXT_TX_POWER_MINUS_21_DBM,
 *                                    HCI_EXT_TX_POWER_MINUS_18_DBM,
 *                                    HCI_EXT_TX_POWER_MINUS_15_DBM,
 *                                    HCI_EXT_TX_POWER_MINUS_12_DBM,
 *                                    HCI_EXT_TX_POWER_MINUS_9_DBM,
 *                                    HCI_EXT_TX_POWER_MINUS_6_DBM,
 *                                    HCI_EXT_TX_POWER_MINUS_3_DBM,
 *                                    HCI_EXT_TX_POWER_0_DBM,
 *                                    HCI_EXT_TX_POWER_1_DBM,
 *                                    HCI_EXT_TX_POWER_2_DBM,
 *                                    HCI_EXT_TX_POWER_3_DBM,
 *                                    HCI_EXT_TX_POWER_4_DBM,
 *                                    HCI_EXT_TX_POWER_5_DBM
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_EXT_SetMaxDtmTxPowerCmd( unsigned char txPower );


/*******************************************************************************
 * @fn          HCI_EXT_MapPmIoPortCmd Vendor Specific API
 *
 * @brief       This HCI Extension API is used to configure and map a CC254x I/O
 *              Port as a General Purpose I/O (GPIO) output signal that reflects
 *              the Power Management (PM) state of the CC254x device. The GPIO
 *              output will be High on Wake, and Low upon entering Sleep. This
 *              feature can be disabled by specifying HCI_EXT_PM_IO_PORT_NONE
 *              for the ioPort (ioPin is then ignored). The system default value
 *              upon hardware reset is disabled. This command can be used to
 *              control an external DC-DC Converter (its actual intent) such has
 *              the TI TPS62730 (or any similar converter that works the same
 *              way). This command should be used with extreme care as it will
 *              override how the Port/Pin was previously configured! This
 *              includes the mapping of Port 0 pins to 32kHz clock output,
 *              Analog I/O, UART, Timers; Port 1 pins to Observables, Digital
 *              Regulator status, UART, Timers; Port 2 pins to an external 32kHz
 *              XOSC. The selected Port/Pin will be configured as an output GPIO
 *              with interrupts masked. Careless use can result in a
 *              reconfiguration that could disrupt the system. It is therefore
 *              the user's responsibility to ensure the selected Port/Pin does
 *              not cause any conflicts in the system.
 *
 *              Note: Only Pins 0, 3 and 4 are valid for Port 2 since Pins 1
 *                    and 2 are mapped to debugger signals DD and DC.
 *
 *              Note: Port/Pin signal change will only occur when Power Savings
 *                    is enabled.
 *
 * input parameters
 *
 * @param       ioPort - HCI_EXT_PM_IO_PORT_P0,
 *                       HCI_EXT_PM_IO_PORT_P1,
 *                       HCI_EXT_PM_IO_PORT_P2,
 *                       HCI_EXT_PM_IO_PORT_NONE
 *
 * @param       ioPin  - HCI_EXT_PM_IO_PORT_PIN0,
 *                       HCI_EXT_PM_IO_PORT_PIN1,
 *                       HCI_EXT_PM_IO_PORT_PIN2,
 *                       HCI_EXT_PM_IO_PORT_PIN3,
 *                       HCI_EXT_PM_IO_PORT_PIN4,
 *                       HCI_EXT_PM_IO_PORT_PIN5,
 *                       HCI_EXT_PM_IO_PORT_PIN6,
 *                       HCI_EXT_PM_IO_PORT_PIN7
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_EXT_MapPmIoPortCmd( unsigned char ioPort, unsigned char ioPin );


/*******************************************************************************
 * @fn          HCI_EXT_DisconnectImmedCmd API
 *
 * @brief       This HCI Extension API is used to disconnect the connection
 *              immediately.
 *
 *              Note: The connection (if valid) is immediately terminated
 *                    without notifying the remote device. The Host is still
 *                    notified.
 *
 * input parameters
 *
 * @param       connHandle - Connection handle.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_EXT_DisconnectImmedCmd( unsigned short int connHandle );


/*******************************************************************************
 * @fn          HCI_EXT_PacketErrorRate Vendor Specific API
 *
 * @brief       This function is used to Reset or Read the Packet Error Rate
 *              counters for a connection.
 *
 *              Note: The counters are only 16 bits. At the shortest connection
 *                    interval, this provides a bit over 8 minutes of data.
 *
 * input parameters
 *
 * @param       connHandle - The LL connection ID on which to send this data.
 * @param       command    - HCI_EXT_PER_RESET, HCI_EXT_PER_READ
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_EXT_PacketErrorRateCmd( unsigned short int connHandle, unsigned char command );


/*******************************************************************************
 * @fn          HCI_EXT_PERbyChanCmd Vendor Specific API
 *
 * @brief       This HCI Extension API is used to start or end Packet Error Rate
 *              by Channel counter accumulation for a connection. If the
 *              pointer is not NULL, it is assumed there is sufficient memory
 *              for the PER data, per the type perByChan_t. If NULL, then
 *              the operation is considered disabled.
 *
 *              Note: It is the user's responsibility to make sure there is
 *                    sufficient memory for the data, and that the counters
 *                    are cleared prior to first use.
 *
 *              Note: The counters are only 16 bits. At the shortest connection
 *                    interval, this provides a bit over 8 minutes of data.
 *
 * input parameters
 *
 * @param       connHandle - The LL connection ID on which to send this data.
 * @param       perByChan  - Pointer to PER by Channel data, or NULL.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_EXT_PERbyChanCmd( unsigned short int connHandle, unsigned short int *perByChan );


/*******************************************************************************
 * @fn          HCI_EXT_ExtendRfRangeCmd API
 *
 * @brief       This HCI Extension API is used to Extend Rf Range using the TI
 *              CC2590 2.4 GHz RF Front End device.
 *
 *              Related Events: HCI_VendorSpecifcCommandCompleteEvent
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_EXT_ExtendRfRangeCmd( void );


/*******************************************************************************
 * @fn          HCI_EXT_HaltDuringRfCmd API
 *
 * @brief       This HCI Extension API is used to enable or disable halting the
 *              CPU during RF. The system defaults to enabled.
 *
 *              Related Events: HCI_VendorSpecifcCommandCompleteEvent
 *
 * input parameters
 *
 * @param       mode - HCI_EXT_HALT_DURING_RF_ENABLE,
 *                     HCI_EXT_HALT_DURING_RF_DISABLE
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_EXT_HaltDuringRfCmd( unsigned char mode );


/*******************************************************************************
 * @fn          HCI_EXT_AdvEventNoticeCmd Vendor Specific API
 *
 * @brief       This HCI Extension API is used to enable or disable a
 *              notification to the specified task using the specified task
 *              event whenever a Adv event ends. A non-zero taskEvent value is
 *              taken to be "enable", while a zero valued taskEvent is taken
 *              to be "disable".
 *
 * input parameters
 *
 * @param       taskID    - User's task ID.
 * @param       taskEvent - User's task event.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_EXT_AdvEventNoticeCmd( unsigned char taskID, unsigned short int taskEvent );


/*******************************************************************************
 * @fn          HCI_EXT_ConnEventNoticeCmd Vendor Specific API
 *
 * @brief       This HCI Extension API is used to enable or disable a
 *              notification to the specified task using the specified task
 *              event whenever a Connection event ends. A non-zero taskEvent
 *              value is taken to be "enable", while a zero valued taskEvent
 *              taken to be "disable".
 *
 *              Note: Currently, only a Slave connection is supported.
 *
 * input parameters
 *
 * @param       taskID    - User's task ID.
 * @param       taskEvent - User's task event.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_EXT_ConnEventNoticeCmd( unsigned char taskID, unsigned short int taskEvent );


/*******************************************************************************
 * @fn          HCI_EXT_BuildRevisionCmd Vendor Specific API
 *
 * @brief       This HCI Extension API is used set a user revision number or
 *              read the build revision number.
 *
 * input parameters
 *
 * @param       mode - HCI_EXT_SET_USER_REVISION | HCI_EXT_READ_BUILD_REVISION
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_EXT_BuildRevisionCmd( unsigned char mode, unsigned short int userRevNum );


/*******************************************************************************
 * @fn          HCI_EXT_DelaySleepCmd Vendor Specific API
 *
 * @brief       This HCI Extension API is used set the sleep delay.
 *
 * input parameters
 *
 * @param       delay - 0..1000, in milliseconds.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_EXT_DelaySleepCmd( unsigned short int delay );


/*******************************************************************************
 * @fn          HCI_EXT_ResetSystemCmd Vendor Specific API
 *
 * @brief       This HCI Extension API is used to issue a soft or hard
 *              system reset.
 *
 * input parameters
 *
 * @param       mode - HCI_EXT_RESET_SYSTEM_HARD | HCI_EXT_RESET_SYSTEM_SOFT
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_EXT_ResetSystemCmd( unsigned char mode );


/*******************************************************************************
 * @fn          HCI_EXT_OverlappedProcessingCmd Vendor Specific API
 *
 * @brief       This HCI Extension API is used to enable or disable overlapped
 *              processing.
 *
 * input parameters
 *
 * @param       mode - HCI_EXT_ENABLE_OVERLAPPED_PROCESSING |
 *                     HCI_EXT_DISABLE_OVERLAPPED_PROCESSING
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_EXT_OverlappedProcessingCmd( unsigned char mode );


/*******************************************************************************
 * @fn          HCI_EXT_NumComplPktsLimitCmd Vendor Specific API
 *
 * @brief       This HCI Extension API is used to set the minimum number of
 *              completed packets which must be met before a Number of
 *              Completed Packets event is returned. If the limit is not
 *              reach by the end of the connection event, then a Number of
 *              Completed Packets event will be returned (if non-zero) based
 *              on the flushOnEvt flag.
 *
 * input parameters
 *
 * @param       limit      - From 1 to HCI_MAX_NUM_DATA_BUFFERS.
 * @param       flushOnEvt - HCI_EXT_DISABLE_NUM_COMPL_PKTS_ON_EVENT |
 *                           HCI_EXT_ENABLE_NUM_COMPL_PKTS_ON_EVENT
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_EXT_NumComplPktsLimitCmd( unsigned char limit,
                                                 unsigned char flushOnEvt );

/*******************************************************************************
 * @fn          HCI_EXT_NumComplPktsLimitCmd Vendor Specific API
 *
 * @brief       This HCI Extension API is used to get the number of allocated
 *              connections, and the number of active connections. The number
 *              of allocated connections is based on a default build value that
 *              can be changed at run time by the user (please see
 *              ll_userConfig.h). The number of active connections refers to
 *              active BLE connections.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_EXT_GetNumConns( void );


#ifdef __cplusplus
}
#endif

#endif /* HCI_H */
