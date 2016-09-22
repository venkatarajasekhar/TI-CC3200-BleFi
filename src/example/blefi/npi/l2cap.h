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


/**************************************************************************************************
  Filename:       l2cap.h
  Description:    This file contains the L2CAP definitions.
**************************************************************************************************/

#ifndef L2CAP_H
#define L2CAP_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "def.h"

// declared in def.h
//typedef unsigned char bStatus_t;
/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

// Minimum supported information payload for the Basic information frame (B-frame)
#define L2CAP_MTU_SIZE                   23

// Minimum supported information payload for the Control frame (C-frame)
#define L2CAP_SIG_MTU_SIZE               23

// Basic L2CAP header: Length (2 uint8s) + Channel ID (2 uint8s)
#define L2CAP_HDR_SIZE                   4

// Minimum size of PDU received from lower layer protocol (incoming
// packet), or delivered to lower layer protocol (outgoing packet).
#define L2CAP_PDU_SIZE                   ( L2CAP_HDR_SIZE + L2CAP_MTU_SIZE )

// Maximum SDU size (PSM's MTU) that can be received on a Connection Oriented
// Channel.
#define L2CAP_SDU_SIZE                   512

// SDU Length field size
#define L2CAP_LEN_FIELD_SIZE             2

// LE Protocol/Service Multiplexer values. LE PSM values are separated into
// two ranges. Values in the first range are assigned by the Bluetooth SIG
// and indicate protocols. Values in the second range are dynamically
// allocated and used in conjunction with services defined in the GATT server.
#define L2CAP_INVALID_PSM                0x0000

// Fixed LE PSM values (SIG assigned)
#define L2CAP_FIXED_PSM_MIN              0x0001
#define L2CAP_FIXED_PSM_MAX              0x007F

// Dynamic LE PSM values
#define L2CAP_DYNAMIC_PSM_MIN            0x0080
#define L2CAP_DYNAMIC_PSM_MAX            0x00FF

// L2CAP Channel Identifiers. Identifiers from 0x0001 to 0x003F are
// reserved for specific L2CAP functions. Identifiers 0x0001-0x0003
// are reserved by BR/EDR.
#define L2CAP_CID_NULL                   0x0000 // Illegal Identifier

// L2CAP Fixed Channel Identifiers
#define L2CAP_CID_ATT                    0x0004 // Attribute Protocol
#define L2CAP_CID_SIG                    0x0005 // L2CAP Signaling
#define L2CAP_CID_SMP                    0x0006 // Security Management Protocol
#define L2CAP_CID_GENERIC                0x0007 // Generic (proprietary channel)

// L2CAP Dynamic Channel Identifiers
#define L2CAP_DYNAMIC_CID_MIN            0x0040
#define L2CAP_DYNAMIC_CID_MAX            0x007F

// Number of Fixed channels: one for each of ATT, Signaling, SMP and Generic channels
#define L2CAP_NUM_FIXED_CHANNELS         4

// Number of Signaling Commands: one for Connection Parameter Update Request
#define L2CAP_NUM_SIG_COMMANDS           1

// Default maximum number of L2CAP Protocol/Service Multiplexers (PSM)
#define L2CAP_NUM_PSM_DEFAULT            3

// Default maximum number of L2CAP Connection Oriented Channels
#define L2CAP_NUM_CO_CHANNELS_DEFAULT    3

// L2CAP Response Timeout expired (RTX) value for Signaling commands (in seconds).
// The RTX timer is used for response timeout or to terminate a dynamic channel
// when the remote device is unresponsive to signaling requests. Its value may
// range from 1 to 60 seconds.
#define L2CAP_RTX_TIMEOUT                30

// L2CAP Signaling Codes (type of commands)
#define L2CAP_CMD_REJECT                 0x01
#define L2CAP_DISCONNECT_REQ             0x06
#define L2CAP_DISCONNECT_RSP             0x07
#define L2CAP_INFO_REQ                   0x0a // No longer supported
#define L2CAP_INFO_RSP                   0x0b // No longer supported
#define L2CAP_PARAM_UPDATE_REQ           0x12
#define L2CAP_PARAM_UPDATE_RSP           0x13
#define L2CAP_CONNECT_REQ                0x14
#define L2CAP_CONNECT_RSP                0x15
#define L2CAP_FLOW_CTRL_CREDIT           0x16

// L2CAP Proprietry Event Codes (type of events): 0x60-0x6F
#define L2CAP_CONNECT_EVT                0x60 // Channel connected
#define L2CAP_DISCONNECT_EVT             0x61 // Channel disconnected
#define L2CAP_CREDIT_EVT                 0x62 // Local credit
#define L2CAP_PEER_CREDIT_EVT            0x63 // Peer credit
#define L2CAP_TX_SDU_EVT                 0x64 // Transmit SDU status event

/*********************************************************************
 * Command Reject: Reason Codes
 */
  // Command not understood
#define L2CAP_REJECT_CMD_NOT_UNDERSTOOD  0x0000

  // Signaling MTU exceeded
#define L2CAP_REJECT_SIGNAL_MTU_EXCEED   0x0001

  // Invalid CID in request
#define L2CAP_REJECT_INVALID_CID         0x0002

/*********************************************************************
 * Connection Response: Result values
 */
  // Connection successful
#define L2CAP_CONN_SUCCESS               0x0000

  // Connection refused – LE_PSM not supported
#define L2CAP_CONN_PSM_NOT_SUPPORTED     0x0002

  // Connection refused – no resources available
#define L2CAP_CONN_NO_RESOURCES          0x0004

  // Connection refused – insufficient authentication
#define L2CAP_CONN_INSUFFICIENT_AUTHEN   0x0005

  // Connection refused – insufficient authorization
#define L2CAP_CONN_INSUFFICIENT_AUTHOR   0x0006

  // Connection refused – insufficient encryption key size
#define L2CAP_CONN_INSUFFICIENT_KEY_SIZE 0x0007

  // Connection refused – insufficient encryption
#define L2CAP_CONN_INSUFFICIENT_ENCRYPT  0x0008

  // Connection security varification pending (used internally)
  //
  // NOTE - PSM must send back a Connection Response
  //
#define L2CAP_CONN_PENDING_SEC_VERIFY    0xFFFF

/*********************************************************************
 * Information Request/Response: Info Type
 */
  // Connectionless MTU
#define L2CAP_INFO_CONNLESS_MTU          0x0001

  // Extended features supported
#define L2CAP_INFO_EXTENDED_FEATURES     0x0002

  // Fixed channels supported
#define L2CAP_INFO_FIXED_CHANNELS        0x0003

/*********************************************************************
 * Information Response: Extended Features Mask Values
 */
  // Fixed channels are supported
#define L2CAP_FIXED_CHANNELS             0x00000080

  // Length of Extended Features bit mask
#define L2CAP_EXTENDED_FEATURES_SIZE     4

/*********************************************************************
 * Information Response: Fixed Channels Mask Values
 */
  // Fixed Channel ATT is supported
#define L2CAP_FIXED_CHANNELS_ATT         0x10

  // Fixed Channel L2CAP Signaling is supported
#define L2CAP_FIXED_CHANNELS_SIG         0x20

  // Fixed Channel SMP is supported
#define L2CAP_FIXED_CHANNELS_SMP         0x40

  // Length of Fixed Channels bit mask
#define L2CAP_FIXED_CHANNELS_SIZE        8

/*********************************************************************
 * Information Response: Result Values
 */
  // Success
#define L2CAP_INFO_SUCCESS               0x0000

  // Not supported
#define L2CAP_INFO_NOT_SUPPORTED         0x0001

/*********************************************************************
 * Connection Parameter Update Response: Result values
 */
  // Connection Parameters accepted
#define L2CAP_CONN_PARAMS_ACCEPTED       0x0000

  // Connection Parameters rejected
#define L2CAP_CONN_PARAMS_REJECTED       0x0001

/*********************************************************************
 * Disconnect Event: Reason values
 */
  // Physical link went down
#define L2CAP_DISCONN_LINK_DOWN          0x0000

  // Channel disconnected by local PSM
#define L2CAP_DISCONN_BY_PSM             0x0001

  // Channel disconnected by remote PSM
#define L2CAP_DISCONN_BY_PEER_PSM        0x0002

  // Credit count exceeded 65535
#define L2CAP_DISCONN_MAX_CREDIT_EXCEED  0x0003

  // Total length of payload received exceeded local PSM's MTU
#define L2CAP_DISCONN_SDU_LEN_EXCEED     0x0004

  // Total length of payload received exceeded SDU length specified in
  // first LE-frame
#define L2CAP_DISCONN_PSM_MTU_EXCEED     0x0005

  // Packet was received from peer device that has a credit of count zero
#define L2CAP_DISCONN_RX_PKT_NO_CREDIT   0x0006

  // Packet was received with error
#define L2CAP_DISCONN_RX_ERROR           0x0007

  // Error happend while trying to send packet
#define L2CAP_DISCONN_TX_ERROR           0x0008

  // Memory allocation error occurred
#define L2CAP_DISCONN_MEM_ALLOC_ERROR    0x0009

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * TYPEDEFS
 */

// User configurable variables
typedef struct
{
  unsigned char maxNumPSM;        // Max number of Protocol/Service Multiplexers (PSM)
  unsigned char maxNumCoChannels; // Max number of Connection Oriented Channels
} l2capUserCfg_t;

// Invalid CID in Request format
typedef struct
{
  unsigned short int localCID;  // Destination CID from the rejected command
  unsigned short int remoteCID; // Source CID from the rejected command
} l2capInvalidCID_t;

// Command Reject Reason Data format
typedef union
{
  unsigned short int signalMTU;             // Maximum Signaling MTU
  l2capInvalidCID_t invalidCID; // Invalid CID in Request
} l2capReasonData_t;

// Command Reject format
typedef struct
{
  unsigned short int reason;                // Reason
  l2capReasonData_t reasonData; // Reason Data

  // Shorthand access for union members
  #define maxSignalMTU     reasonData.signalMTU
  #define invalidLocalCID  reasonData.invalidCID.localCID
  #define invalidRemoteCID reasonData.invalidCID.remoteCID
} l2capCmdReject_t;

// Connection Parameter Update Request format
typedef struct
{
  unsigned short int intervalMin;       // Minimum Interval
  unsigned short int intervalMax;       // Maximum Interval
  unsigned short int slaveLatency;      // Slave Latency
  unsigned short int timeoutMultiplier; // Timeout Multiplier
} l2capParamUpdateReq_t;

// Connection Parameter Update Response format
typedef struct
{
  unsigned short int result; // Result
} l2capParamUpdateRsp_t;

// Information Request format
typedef struct
{
  unsigned short int infoType; // Information type
} l2capInfoReq_t;

// Information Response Data field
typedef union
{
  unsigned short int connectionlessMTU;                       // Connectionless MTU
  unsigned long extendedFeatures;                        // Extended features supported
  unsigned char fixedChannels[L2CAP_FIXED_CHANNELS_SIZE]; // Fixed channels supported
} l2capInfo_t;

// Information Response format
typedef struct
{
  unsigned short int result;    // Result
  unsigned short int infoType;  // Information type
  l2capInfo_t info; // Content of Info field depends on infoType
} l2capInfoRsp_t;

// Connection Request format
typedef struct
{
  unsigned short int psm;         // LE PSM
  unsigned short int srcCID;      // Represents CID on device sending request and receiving response
  unsigned short int mtu;         // Specifies maximum SDU size that can be received on this channel
  unsigned short int mps;         // Specifies maximum payload size that can be received on this channel
  unsigned short int initCredits; // Indicates number of LE-frames that peer device can send
} l2capConnectReq_t;

// Connection Response format
typedef struct
{
  unsigned short int dstCID;      // Represents CID on device receiving request and sending response
  unsigned short int mtu;         // Specifies maximum SDU size that can be received on this channel
  unsigned short int mps;         // Specifies maximum payload size that can be received on this channel
  unsigned short int initCredits; // Indicates number of LE-frames that peer device can send
  unsigned short int result;      // Indicates outcome of connection request
} l2capConnectRsp_t;

// Disconnection Request format (src/dst CIDs are relative to sender of request)
typedef struct
{
  unsigned short int dstCID; // Specifies CID to be disconnected on device receiving request
  unsigned short int srcCID; // Specifies CID to be disconnected on device sending request
} l2capDisconnectReq_t;

// Disconnection Response format (src/dst CIDs are relative to sender of request)
typedef struct
{
  unsigned short int dstCID; // Identifies CID on device sending response
  unsigned short int srcCID; // Identifies CID on device receiving response
} l2capDisconnectRsp_t;

// Flow Control Credit format
typedef struct
{
  unsigned short int CID;     // Represents Source CID of device sending credit packet
  unsigned short int credits; // Number of LE-frames that can be sent to local device
} l2capFlowCtrlCredit_t;

// PSM info
typedef struct
{
  unsigned short int mtu;             // Maximum SDU size that can be received by local device
  unsigned short int mps;             // Maximum payload size that can be received by local device
  unsigned short int initPeerCredits; // Number of LE-frames that peer device can send
  unsigned char  numChannels;     // Number of active connection oriented channels
} l2capPsmInfo_t;

// Connection Oriented Channel info
typedef struct
{
  unsigned short int psm;         // PSM that channel belongs to
  unsigned short int mtu;         // Maximum SDU size that can be received by local device
  unsigned short int mps;         // Maximum payload size that can be received by local device
  unsigned short int credits;     // Number of LE-frames that local device can send
  unsigned short int peerCID;     // Remote channel id
  unsigned short int peerMtu;     // Maximum SDU size that can be received by peer device
  unsigned short int peerMps;     // Maximum payload size that can be received by peer device
  unsigned short int peerCredits; // Number of LE-frames that peer device can send
} l2capCoCInfo_t;

// Local channel info
typedef struct
{
  unsigned char state;         // Channel connection state
  l2capCoCInfo_t info; // Channel info
} l2capChannelInfo_t;

// L2CAP_CONNECT_EVT message format.  This message is sent to the
// app when an L2CAP Connection Oriented Channel is established.
typedef struct
{
  unsigned short int result;       // Indicates outcome of connection request
  unsigned short int CID;          // Local channel id
  l2capCoCInfo_t info; // Channel info
} l2capConnectEvt_t;

// L2CAP_DISCONNECT_EVT message format.  This message is sent to the
// app when an L2CAP Connection Oriented Channel is disconnected.
typedef struct
{
  unsigned short int psm;     // PSM channel belongs to
  unsigned short int CID;     // Local channel id
  unsigned short int peerCID; // Peer channel id
  unsigned short int reason;  // Indicates reason for disconnection
} l2capDisconnectEvt_t;

// L2CAP_CREDIT_EVT message format.  This message is sent to the
// app to indicate the number of available credits on a channel.
typedef struct
{
  unsigned short int psm;           // PSM channel belongs to
  unsigned short int CID;           // Local channel id
  unsigned short int peerCID;       // Peer channel id
  unsigned short int creditsNeeded; // Credits needed to send or receive remaining segments
} l2capCreditEvt_t;

// L2CAP_TX_SDU_EVT message format.  This message is sent to the
// app when an SDU transmission is completed or aborted.
typedef struct
{
  unsigned short int psm;         // PSM channel belongs to
  unsigned short int CID;         // Local channel id
  unsigned short int credits;     // Local credits
  unsigned short int peerCID;     // Peer channel id
  unsigned short int peerCredits; // Peer credits
  unsigned short int totalLen;    // Total length of SDU to be transmitted
  unsigned short int txLen;       // Total number of octets transmitted
} l2capTxSduEvt_t;

// Union of all L2CAP Signaling commands
typedef union
{
  // Requests
  l2capParamUpdateReq_t updateReq;
  l2capInfoReq_t infoReq;
  l2capConnectReq_t connectReq;
  l2capDisconnectReq_t disconnectReq;
  l2capFlowCtrlCredit_t credit;

  // Responses
  l2capParamUpdateRsp_t updateRsp;
  l2capInfoRsp_t infoRsp;
  l2capCmdReject_t cmdReject;
  l2capConnectRsp_t connectRsp;
  l2capDisconnectRsp_t disconnectRsp;

  // Proprietry local events
  l2capConnectEvt_t connectEvt;
  l2capDisconnectEvt_t disconnectEvt;
  l2capCreditEvt_t creditEvt;
  l2capTxSduEvt_t txSduEvt;
} l2capSignalCmd_t;

// OSAL L2CAP_SIGNAL_EVENT message format. This message is used to deliver an
// incoming Signaling command up to an upper layer application.
typedef struct
{
  unsigned short int connHandle;    // connection message was received on
  unsigned char id;             // identifier to match responses with requests
  unsigned char opcode;         // type of command
  l2capSignalCmd_t cmd; // command data
} l2capSignalEvent_t;

// L2CAP packet structure
typedef struct
{
  unsigned short int CID;      // local channel id
  unsigned char *pPayload; // pointer to information payload. This contains the payload
                   // received from the upper layer protocol (outgoing packet),
                   // or delivered to the upper layer protocol (incoming packet).
  unsigned short int len;      // length of information payload
} l2capPacket_t;

// OSAL L2CAP_DATA_EVENT message format. This message is used to forward an
// incoming data packet up to an upper layer application.
typedef struct
{
  unsigned short int connHandle;    // connection packet was received on
  l2capPacket_t pkt;    // received packet
} l2capDataEvent_t;

/**
 * @brief   Callback function prototype to verify security when a
 *          Connection Request is received.
 *
 * @param   connHandle - connection handle request was received on
 * @param   id - identifier matches responses with requests
 * @param   pReq - received connection request
 *
 * @return  See L2CAP Connection Response: Result values
 */
typedef unsigned short int (*pfnVerifySecCB_t)( unsigned short int connHandle, unsigned char id, l2capConnectReq_t *pReq );

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 *  Initialize L2CAP layer.
 */
extern void L2CAP_Init( unsigned char taskId );

/*
 *  L2CAP Task event processing function.
 */
extern unsigned short int L2CAP_ProcessEvent( unsigned char taskId, unsigned short int events );

/*
 * Register a protocol/application with an L2CAP channel.
 */
extern bStatus_t L2CAP_RegisterApp( unsigned char taskId, unsigned short int CID );

/*
 * Register a Protocol/Service Multiplexer with L2CAP.
 */
extern bStatus_t L2CAP_RegisterPsm( unsigned char taskId, unsigned short int psm, unsigned short int mtu, unsigned short int initCredits,
                                    pfnVerifySecCB_t pfnVerifySecCB );
/*
 * Deregister a Protocol/Service Multiplexer with L2CAP.
 */
extern bStatus_t L2CAP_DeregisterPsm( unsigned char taskId, unsigned short int psm );

/*
 * Get information about a given registered PSM.
 */
extern bStatus_t L2CAP_PsmInfo( unsigned short int psm, l2capPsmInfo_t *pInfo );

/*
 * Get all active channels for a given registered PSM.
 */
extern bStatus_t L2CAP_PsmChannels( unsigned short int psm, unsigned char numCIDs, unsigned short int *pCIDs );

/*
 * Get information about a given active Connection Oriented Channnel.
 */
extern bStatus_t L2CAP_ChannelInfo( unsigned short int CID, l2capChannelInfo_t *pInfo );

/*
 *  Send L2CAP Data Packet over an L2CAP connection oriented channel.
 */
extern bStatus_t L2CAP_SendSDU( unsigned short int CID, unsigned char *pSDU, unsigned short int len );

/*
 *  Send L2CAP Connection Request.
 */
extern bStatus_t L2CAP_ConnectReq( unsigned short int connHandle, unsigned short int psm, unsigned short int peerPsm,
                                   unsigned short int initPeerCredits );
/*
 * Build Connection Request.
 */
extern unsigned short int L2CAP_BuildConnectReq( unsigned char *pBuf, unsigned char *pCmd );

/*
 * Parse Connection Request.
 */
extern bStatus_t L2CAP_ParseConnectReq( l2capSignalCmd_t *pCmd, unsigned char *pData, unsigned short int len );

/*
 *  Send L2CAP Connection Response.
 */
extern bStatus_t L2CAP_ConnectRsp( unsigned short int psm, unsigned char id, unsigned short int result );

/*
 * Parse Connection Response.
 */
extern bStatus_t L2CAP_ParseConnectRsp( l2capSignalCmd_t *pCmd, unsigned char *pData, unsigned short int len );

/*
 *  Send L2CAP Disconnection Request.
 */
extern bStatus_t L2CAP_DisconnectReq( unsigned short int srcCID );

/*
 * Parse Disconnection Request.
 */
extern bStatus_t L2CAP_ParseDisconnectReq( l2capSignalCmd_t *pCmd, unsigned char *pData, unsigned short int len );

/*
 * Build Connection Response.
 */
extern unsigned short int L2CAP_BuildConnectRsp( unsigned char *pBuf, unsigned char *pCmd );

/*
 * Build Disconnection Response.
 */
extern unsigned short int L2CAP_BuildDisconnectRsp( unsigned char *pBuf, unsigned char *pCmd );

/*
 *  Send L2CAP Flow Control Credit message.
 */
extern bStatus_t L2CAP_FlowCtrlCredit( unsigned short int CID, unsigned short int peerCredits );

/*
 * Parse Flow Control Credit message.
 */
extern bStatus_t L2CAP_ParseFlowCtrlCredit( l2capSignalCmd_t *pCmd, unsigned char *pData, unsigned short int len );

/*
 *  Send L2CAP Data Packet over an L2CAP fixed channel.
 */
extern bStatus_t L2CAP_SendData( unsigned short int connHandle, l2capPacket_t *pPkt );

/*
 * Send Command Reject.
 */
extern bStatus_t L2CAP_CmdReject( unsigned short int connHandle, unsigned char id, l2capCmdReject_t *pCmdReject );

/*
 * Build Command Reject.
 */
extern unsigned short int L2CAP_BuildCmdReject( unsigned char *pBuf, unsigned char *pCmd );

/*
 *  Send L2CAP Information Request.
 */
extern bStatus_t L2CAP_InfoReq( unsigned short int connHandle, l2capInfoReq_t *pInfoReq, unsigned char taskId );

/*
 * Build Information Response.
 */
extern unsigned short int L2CAP_BuildInfoRsp( unsigned char *pBuf, unsigned char *pCmd );

/*
 * Parse Information Request.
 */
extern bStatus_t L2CAP_ParseInfoReq( l2capSignalCmd_t *pCmd, unsigned char *pData, unsigned short int len );

/*
 *  Send L2CAP Connection Parameter Update Request.
 */
extern bStatus_t L2CAP_ConnParamUpdateReq( unsigned short int connHandle, l2capParamUpdateReq_t *pUpdateReq, unsigned char taskId );

/*
 * Parse Connection Parameter Update Request.
 */
extern bStatus_t L2CAP_ParseParamUpdateReq( l2capSignalCmd_t *pCmd, unsigned char *pData, unsigned short int len );

/*
 *  Send L2CAP Connection Parameter Update Response.
 */
extern bStatus_t L2CAP_ConnParamUpdateRsp( unsigned short int connHandle, unsigned char id, l2capParamUpdateRsp_t *pUpdateRsp );

/*
 * Build Connection Parameter Update Response.
 */
extern unsigned short int L2CAP_BuildParamUpdateRsp( unsigned char *pBuf, unsigned char *pData );

/*
 * Allocate a block of memory at the L2CAP layer.
 */
extern void *L2CAP_bm_alloc( unsigned short int size );

/*
 * This API is used by the upper layer to turn flow control on
 * or off for data packets sent from the Controller to the Host.
 */
extern void L2CAP_SetControllerToHostFlowCtrl( unsigned short int hostBuffSize, unsigned char flowCtrlMode );

/*
 * This API is used by the upper layer to notify L2CAP of the
 * number of data packets that have been completed for connection
 * handle since this API was previously called.
 */
extern void L2CAP_HostNumCompletedPkts( unsigned short int connHandle, unsigned short int numCompletedPkts );

/*
 * This API is used by the upper layer to set the maximum data
 * packet size and the number of data packets allowed by the
 * Controller.
 */
extern void L2CAP_SetBufSize( unsigned short int dataPktLen, unsigned char numDataPkts );

/*
 * Set the user configurable variables for the L2CAP layer.
 *
 * Note: This function should be called BEFORE osal_init_system.
 */
extern void L2CAP_SetUserConfig( l2capUserCfg_t *pUserCfg );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif


#endif /* L2CAP_H_ */
