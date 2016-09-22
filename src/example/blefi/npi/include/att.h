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


/**
  @headerfile:    att.h

  Description:    This file contains Attribute Protocol (ATT) definitions
                  and prototypes.

**************************************************************************************************/

#ifndef ATT_H
#define ATT_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "hci.h"
#include "l2cap.h"
#include "ble_events.h"

/*********************************************************************
 * CONSTANTS
 */

// The Exchanging MTU Size is defined as the maximum size of any packet 
// transmitted between a client and a server. A higher layer specification
// defines the default ATT MTU value. The ATT MTU value should be within
// the range 23 to 517 inclusive.
#define ATT_MTU_SIZE                     L2CAP_MTU_SIZE //!< Minimum ATT MTU size
#define ATT_MAX_MTU_SIZE                 517            //!< Maximum ATT MTU size

/** @defgroup ATT_METHOD_DEFINES ATT Methods 
 * @{
 */

#define ATT_ERROR_RSP                    0x01 //!< ATT Error Response
#define ATT_EXCHANGE_MTU_REQ             0x02 //!< ATT Exchange MTU Request
#define ATT_EXCHANGE_MTU_RSP             0x03 //!< ATT Exchange MTU Response
#define ATT_FIND_INFO_REQ                0x04 //!< ATT Find Information Request
#define ATT_FIND_INFO_RSP                0x05 //!< ATT Find Information Response
#define ATT_FIND_BY_TYPE_VALUE_REQ       0x06 //!< ATT Find By Type Vaue Request
#define ATT_FIND_BY_TYPE_VALUE_RSP       0x07 //!< ATT Find By Type Vaue Response
#define ATT_READ_BY_TYPE_REQ             0x08 //!< ATT Read By Type Request
#define ATT_READ_BY_TYPE_RSP             0x09 //!< ATT Read By Type Response
#define ATT_READ_REQ                     0x0a //!< ATT Read Request
#define ATT_READ_RSP                     0x0b //!< ATT Read Response
#define ATT_READ_BLOB_REQ                0x0c //!< ATT Read Blob Request
#define ATT_READ_BLOB_RSP                0x0d //!< ATT Read Blob Response
#define ATT_READ_MULTI_REQ               0x0e //!< ATT Read Multiple Request
#define ATT_READ_MULTI_RSP               0x0f //!< ATT Read Multiple Response
#define ATT_READ_BY_GRP_TYPE_REQ         0x10 //!< ATT Read By Group Type Request
#define ATT_READ_BY_GRP_TYPE_RSP         0x11 //!< ATT Read By Group Type Response
#define ATT_WRITE_REQ                    0x12 //!< ATT Write Request
#define ATT_WRITE_RSP                    0x13 //!< ATT Write Response
#define ATT_PREPARE_WRITE_REQ            0x16 //!< ATT Prepare Write Request
#define ATT_PREPARE_WRITE_RSP            0x17 //!< ATT Prepare Write Response
#define ATT_EXECUTE_WRITE_REQ            0x18 //!< ATT Execute Write Request
#define ATT_EXECUTE_WRITE_RSP            0x19 //!< ATT Execute Write Response
#define ATT_HANDLE_VALUE_NOTI            0x1b //!< ATT Handle Value Notification
#define ATT_HANDLE_VALUE_IND             0x1d //!< ATT Handle Value Indication
#define ATT_HANDLE_VALUE_CFM             0x1e //!< ATT Handle Value Confirmation

#define ATT_WRITE_CMD                    0x52 //!< ATT Write Command
#define ATT_SIGNED_WRITE_CMD             0xD2 //!< ATT Signed Write Command

/** @} End ATT_METHOD_DEFINES */

/*** Opcode fields: bitmasks ***/
// Method (bits 5-0)
#define ATT_METHOD_BITS                  0x3f

// Command Flag (bit 6)
#define ATT_CMD_FLAG_BIT                 0x40

// Authentication Signature Flag (bit 7)
#define ATT_AUTHEN_SIG_FLAG_BIT          0x80

// Size of 16-bit Bluetooth UUID
#define ATT_BT_UUID_SIZE                 2
  
// Size of 128-bit UUID
#define ATT_UUID_SIZE                    16
  
// ATT Response or Confirmation timeout
#define ATT_MSG_TIMEOUT                  30

// Authentication Signature status for received PDU; it's TRUE or FALSE for PDU to be sent
#define ATT_SIG_NOT_INCLUDED             0x00 // Signature not included
#define ATT_SIG_VALID                    0x01 // Included signature valid
#define ATT_SIG_INVALID                  0x02 // Included signature not valid

/*********************************************************************
 * Error Response: Error Code
 */

/** @defgroup ATT_ERR_CODE_DEFINES ATT Error Codes
 * @{
 */

#define ATT_ERR_INVALID_HANDLE           0x01 //!< Attribute handle value given was not valid on this attribute server
#define ATT_ERR_READ_NOT_PERMITTED       0x02 //!< Attribute cannot be read
#define ATT_ERR_WRITE_NOT_PERMITTED      0x03 //!< Attribute cannot be written
#define ATT_ERR_INVALID_PDU              0x04 //!< The attribute PDU was invalid
#define ATT_ERR_INSUFFICIENT_AUTHEN      0x05 //!< The attribute requires authentication before it can be read or written
#define ATT_ERR_UNSUPPORTED_REQ          0x06 //!< Attribute server doesn't support the request received from the attribute client
#define ATT_ERR_INVALID_OFFSET           0x07 //!< Offset specified was past the end of the attribute
#define ATT_ERR_INSUFFICIENT_AUTHOR      0x08 //!< The attribute requires an authorization before it can be read or written
#define ATT_ERR_PREPARE_QUEUE_FULL       0x09 //!< Too many prepare writes have been queued
#define ATT_ERR_ATTR_NOT_FOUND           0x0a //!< No attribute found within the given attribute handle range
#define ATT_ERR_ATTR_NOT_LONG            0x0b //!< Attribute cannot be read or written using the Read Blob Request or Prepare Write Request
#define ATT_ERR_INSUFFICIENT_KEY_SIZE    0x0c //!< The Encryption Key Size used for encrypting this link is insufficient
#define ATT_ERR_INVALID_VALUE_SIZE       0x0d //!< The attribute value length is invalid for the operation
#define ATT_ERR_UNLIKELY                 0x0e //!< The attribute request that was requested has encountered an error that was very unlikely, and therefore could not be completed as requested
#define ATT_ERR_INSUFFICIENT_ENCRYPT     0x0f //!< The attribute requires encryption before it can be read or written
#define ATT_ERR_UNSUPPORTED_GRP_TYPE     0x10 //!< The attribute type is not a supported grouping attribute as defined by a higher layer specification
#define ATT_ERR_INSUFFICIENT_RESOURCES   0x11 //!< Insufficient Resources to complete the request

/*** Reserved for future use: 0x12 - 0x7F ***/

/*** Application error code defined by a higher layer specification: 0x80-0x9F ***/

#define ATT_ERR_INVALID_VALUE            0x80 //!< The attribute value is invalid for the operation
  
/** @} End ATT_ERR_CODE_DEFINES */

/*********************************************************************
 * Find Information Response: UUID Format
 */
  // Handle and 16-bit Bluetooth UUID
#define ATT_HANDLE_BT_UUID_TYPE          0x01
  
  // Handle and 128-bit UUID
#define ATT_HANDLE_UUID_TYPE             0x02
  
// Maximum number of handle and 16-bit UUID pairs in a single Find Info Response
#define ATT_MAX_NUM_HANDLE_BT_UUID       ( ( ATT_MTU_SIZE - 2 ) / ( 2 + ATT_BT_UUID_SIZE ) )

// Maximum number of handle and 128-bit UUID pairs in a single Find Info Response
#define ATT_MAX_NUM_HANDLE_UUID          ( ( ATT_MTU_SIZE - 2 ) / ( 2 + ATT_UUID_SIZE ) )

/*********************************************************************
 * Find By Type Value Response: Handles Infomation (Found Attribute Handle and Group End Handle)
 */
  // Maximum number of handles info in a single Find By Type Value Response
#define ATT_MAX_NUM_HANDLES_INFO         ( ( ATT_MTU_SIZE - 1 ) / 4 )

/*********************************************************************
 * Read Multiple Request: Handles
 */
  // Maximum number of handles in a single Read Multiple Request
#define ATT_MAX_NUM_HANDLES              ( ( ATT_MTU_SIZE - 1 ) / 2 )

  // Minimum number of handles in a single Read Multiple Request
#define ATT_MIN_NUM_HANDLES              2

/*********************************************************************
 * Execute Write Request: Flags
 */
  // Cancel all prepared writes
#define ATT_CANCEL_PREPARED_WRITES       0x00
  
  // Immediately write all pending prepared values
#define ATT_WRITE_PREPARED_VALUES        0x01

#if defined ( TESTMODES )
  // ATT Test Modes
  #define ATT_TESTMODE_OFF               0 // Test mode off
  #define ATT_TESTMODE_UNAUTHEN_SIG      1 // Do not authenticate incoming signature
#endif

/*********************************************************************
 * Size of mandatory fields of ATT requests
 */
// Length of Read By Type Request's fixed fields: First handle number (2) + Last handle number (2)
#define READ_BY_TYPE_REQ_FIXED_SIZE        4

// Length of Prepare Write Request's fixed size: Attribute Handle (2) + Value Offset (2)
#define PREPARE_WRITE_REQ_FIXED_SIZE       4

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
 * ATT event header format.
 */
typedef struct __attribute__((__packed__))
{
	EventHeader_t  hdr;           	//!< ATT_MSG_EVENT
	unsigned char opcode;                    //!< ATT type of command. Ref: @ref GATT_MSG_EVENT_DEFINES
	unsigned char event;
	bStatus_t status;
} attEventHdr_t;

/**
 * Attribute Protocol PDU format.
 */
typedef struct  __attribute__((__packed__))
{
  unsigned char sig;      //!< Authentication Signature status (not included (0), valid (1), invalid (2))
  unsigned char cmd;      //!< Command Flag
  unsigned char method;   //!< Method
  unsigned short int len;     //!< Length of Attribute Parameters
  unsigned char *pParams; //!< Attribute Parameters
} attPacket_t;

/**
 * Attribute Type format (2 or 16 octet UUID).
 */
typedef struct  __attribute__((__packed__))
{
  unsigned char len;                 //!< Length of UUID
  unsigned char uuid[ATT_UUID_SIZE]; //!< 16 or 128 bit UUID
} attAttrType_t;

/**
 * Attribute Type format (2-octet Bluetooth UUID).
 */
typedef struct  __attribute__((__packed__))
{
  unsigned char len;                    //!< Length of UUID
  unsigned char uuid[ATT_BT_UUID_SIZE]; //!< 16 bit UUID
} attAttrBtType_t;

/**
 * Error Response format.
 */
typedef struct  __attribute__((__packed__))
{
  unsigned char reqOpcode; //!< Request that generated this error response
  unsigned short int handle;   //!< Attribute handle that generated error response
  unsigned char errCode;   //!< Reason why the request has generated error response
} attErrorRsp_t;

/**
 * Exchange MTU Request format.
 */
typedef struct  __attribute__((__packed__))
{
  unsigned short int clientRxMTU; //!< Client receive MTU size
} attExchangeMTUReq_t;

/**
 * Exchange MTU Response format.
 */
typedef struct  __attribute__((__packed__))
{
  unsigned short int serverRxMTU; //!< Server receive MTU size
} attExchangeMTURsp_t;

/**
 * Find Information Request format.
 */
typedef struct  __attribute__((__packed__))
{
  unsigned short int startHandle;       //!< First requested handle number (must be first field)
  unsigned short int endHandle;         //!< Last requested handle number
} attFindInfoReq_t;

/**
 * Handle and its 16-bit Bluetooth UUIDs.
 */
typedef struct  __attribute__((__packed__))
{
  unsigned short int handle;                //!< Handle
  unsigned char uuid[ATT_BT_UUID_SIZE]; //!< 2-octet Bluetooth UUID
} attHandleBtUUID_t;

/**
 * Handle and its 128-bit UUID.
 */
typedef struct  __attribute__((__packed__))
{
  unsigned short int handle;             //!< Handle
  unsigned char uuid[ATT_UUID_SIZE]; //!< 16-octect UUID
} attHandleUUID_t;

/**
 * Info data format for Find Information Response (handle-UUID pair).
 */
typedef union  __attribute__((__packed__))
{
  attHandleBtUUID_t btPair[ATT_MAX_NUM_HANDLE_BT_UUID]; //!< A list of 1 or more handles with their 16-bit Bluetooth UUIDs
  attHandleUUID_t   pair[ATT_MAX_NUM_HANDLE_UUID];      //!< A list of 1 or more handles with their 128-bit UUIDs
} attFindInfo_t;

/**
 * Find Information Response format.
 */
typedef struct  __attribute__((__packed__))
{
	  EventHeader_t hdr;
	  unsigned char opcode;                   //!< Att type of command. Ref: @ref GAP_MSG_EVENT_DEFINES
	  unsigned char event;
	  unsigned char status;
	  unsigned short int connhandle;
	  //unsigned char numGrps;                  //!< Number of attribute handle, end group handle and value sets found
	  unsigned char pdulen;                  //!<
  unsigned char format;       //!< Format of information data
 // attFindInfo_t info; //!< Information data whose format is determined by format field

  unsigned char dataList[ATT_MTU_SIZE-2];
} attFindInfoRsp_t;

/**
 * Find By Type Value Request format.
 */
typedef struct  __attribute__((__packed__))
{
  unsigned short int startHandle;          //!< First requested handle number (must be first field)
  unsigned short int endHandle;            //!< Last requested handle number
  attAttrBtType_t type;        //!< 2-octet UUID to find
  unsigned char len;                   //!< Length of value
  unsigned char value[ATT_MTU_SIZE-7]; //!< Attribute value to find
} attFindByTypeValueReq_t;

/**
 * Handles Infomation format.
 */
typedef struct  __attribute__((__packed__))
{
  unsigned short int handle;       //!< Found attribute handle
  unsigned short int grpEndHandle; //!< Group end handle
} attHandlesInfo_t;

/**
 * Find By Type Value Response format.
 */
typedef struct  __attribute__((__packed__))
{
  EventHeader_t hdr;
  unsigned char opcode;                   //!< Att type of command. Ref: @ref GAP_MSG_EVENT_DEFINES
  unsigned char event;
  unsigned char status;
  unsigned short int connhandle;
  unsigned char len;                   //!< Length of value
  unsigned char numInfo;                                          //!< Number of handles information found
  attHandlesInfo_t handlesInfo[ATT_MAX_NUM_HANDLES_INFO]; //!< List of 1 or more handles information
} attFindByTypeValueRsp_t;

/**
 * Read By Type Request format.
 */
typedef struct  __attribute__((__packed__))
{
  unsigned short int startHandle; //!< First requested handle number (must be first field)
  unsigned short int endHandle;   //!< Last requested handle number
  attAttrType_t type; //!< Requested type (2 or 16 octet UUID)
} attReadByTypeReq_t;

/**
 * Read By Type Response format.
 */
typedef struct  __attribute__((__packed__))
{
	  EventHeader_t hdr;
	  unsigned char opcode;                   //!< Att type of command. Ref: @ref GAP_MSG_EVENT_DEFINES
	  unsigned char event;
	  unsigned char status;
	  unsigned short int connhandle;
	  //unsigned char numGrps;                  //!< Number of attribute handle, end group handle and value sets found
	  unsigned char pdulen;                  //!<
	  unsigned char len;                      //!< Length of each attribute handle, end group handle and value set
	  unsigned char dataList[ATT_MTU_SIZE-2];
} attReadByTypeRsp_t;

/**
 * Read Request format.
 */
typedef struct  __attribute__((__packed__))
{
  unsigned short int handle; //!< Handle of the attribute to be read (must be first field)
} attReadReq_t;

/**
 * Read Response format.
 */
typedef struct  __attribute__((__packed__))
{
  EventHeader_t hdr;
  unsigned char opcode;                   //!< Att type of command. Ref: @ref GAP_MSG_EVENT_DEFINES
  unsigned char event;
  unsigned char status;
  unsigned short int connhandle;
  unsigned char pdulen;
  unsigned char value[ATT_MTU_SIZE-1]; //!< Value of the attribute with the handle given
} attReadRsp_t;

/**
 * Read Blob Req format.
 */
typedef struct  __attribute__((__packed__))
{
  unsigned short int handle; //!< Handle of the attribute to be read (must be first field)
  unsigned short int offset; //!< Offset of the first octet to be read
} attReadBlobReq_t;

/**
 * Read Blob Response format.
 */
typedef struct  __attribute__((__packed__))
{
  EventHeader_t hdr;
  unsigned char opcode;                   //!< Att type of command. Ref: @ref GAP_MSG_EVENT_DEFINES
  unsigned char event;
  unsigned char status;
  unsigned short int connhandle;
  unsigned char len;                   //!< Length of value
  unsigned char value[4]; //!< Part of the value of the attribute with the handle given
} attReadBlobRsp_t;

/**
 * Read Multiple Request format.
 */
typedef struct  __attribute__((__packed__))
{
  unsigned short int handle[ATT_MAX_NUM_HANDLES]; //!< Set of two or more attribute handles (must be first field)
  unsigned char numHandles;                   //!< Number of attribute handles
} attReadMultiReq_t;

/**
 * Read Multiple Response format.
 */
typedef struct  __attribute__((__packed__))
{
  unsigned char len;                    //!< Length of values
  unsigned char values[ATT_MTU_SIZE-1]; //!< Set of two or more values
} attReadMultiRsp_t;

/**
 * Read By Group Type Request format.
 */
typedef struct  __attribute__((__packed__))
{
  unsigned char Status;
  unsigned short int startHandle; //!< First requested handle number (must be first field)
  unsigned short int endHandle;   //!< Last requested handle number
  attAttrType_t type; //!< Requested group type (2 or 16 octet UUID)
} attReadByGrpTypeReq_t;

/**
 * Read By Group Type Response format.
 */
typedef struct  __attribute__((__packed__))
{
  EventHeader_t hdr;
  unsigned char opcode;                   //!< Att type of command. Ref: @ref GAP_MSG_EVENT_DEFINES
  unsigned char event;
  unsigned char status;
  unsigned short int connhandle;
  //unsigned char numGrps;                  //!< Number of attribute handle, end group handle and value sets found
  unsigned char pdulen;                  //!<
  unsigned char len;                      //!< Length of each attribute handle, end group handle and value set
  unsigned char dataList[ATT_MTU_SIZE-2]; //!< List of 1 or more attribute handle, end group handle and value
} attReadByGrpTypeRsp_t;

/**
 * Write Request format.
 */
typedef struct  __attribute__((__packed__))
{
  unsigned short int handle;               //!< Handle of the attribute to be written (must be first field)
  unsigned char len;                   //!< Length of value
  unsigned char value[ATT_MTU_SIZE-2];              //!< Value of the attribute to be written
  unsigned char sig;                   //!< Authentication Signature status (not included (0), valid (1), invalid (2))
  unsigned char cmd;                   //!< Command Flag
} attWriteReq_t;



/**
 * Write Response format.
 */
typedef struct __attribute__((__packed__))
{
  EventHeader_t hdr;
  unsigned char opcode;                   //!< Att type of command. Ref: @ref GAP_MSG_EVENT_DEFINES
  unsigned char event;
  unsigned char status;                   //!< status of write request
  unsigned short int connhandle;              //!< conn handle of the connection
  unsigned char pdulen;                   //!<
} attWriteRsp_t;

/**
 * Prepare Write Request format.
 */
typedef struct  __attribute__((__packed__))
{
  unsigned short int handle;               //!< Handle of the attribute to be written (must be first field)
  unsigned short int offset;               //!< Offset of the first octet to be written
  unsigned char len;                   //!< Length of value
  unsigned char value[ATT_MTU_SIZE-5]; //!< Part of the value of the attribute to be written
} attPrepareWriteReq_t;

/**
 * Prepare Write Response format.
 */
typedef struct  __attribute__((__packed__))
{
  unsigned short int handle;               //!< Handle of the attribute that has been read
  unsigned short int offset;               //!< Offset of the first octet to be written
  unsigned char len;                   //!< Length of value
  unsigned char value[ATT_MTU_SIZE-5]; //!< Part of the value of the attribute to be written
} attPrepareWriteRsp_t;

/**
 * Execute Write Request format.
 */
typedef struct  __attribute__((__packed__))
{
  unsigned char flags; //!< 0x00 - cancel all prepared writes.
               //!< 0x01 - immediately write all pending prepared values.
} attExecuteWriteReq_t;

/**
 * Handle Value Notification format.
 */
typedef struct  __attribute__((__packed__))
{
  EventHeader_t hdr;
  unsigned char  opcode;                   //!< Att type of command. Ref: @ref GAP_MSG_EVENT_DEFINES
  unsigned char  event;
  unsigned char  status;                   //!< status of write request
  unsigned short int connhandle;              //!< conn handle of the connection
  unsigned char  pdulen;                   //!<
  unsigned short int handle;               //!< Handle of the attribute that has been changed (must be first field)
  unsigned char value[ATT_MTU_SIZE-3]; //!< New value of the attribute
} attHandleValueNoti_t;

/**
 * Handle Value Indication format.
 */
typedef struct  __attribute__((__packed__))
{
  unsigned short int handle;               //!< Handle of the attribute that has been changed (must be first field)
  unsigned char len;                   //!< Length of value
  unsigned char value[ATT_MTU_SIZE-3]; //!< New value of the attribute
} attHandleValueInd_t;

/**
 * ATT Message format. It's a union of all attribute protocol messages used 
 * between the attribute protocol and upper layer profile/application.
 */
typedef union
{
  // Request messages
  attExchangeMTUReq_t exchangeMTUReq;         //!< ATT Exchange MTU Request
  attFindInfoReq_t findInfoReq;               //!< ATT Find Information Request
  attFindByTypeValueReq_t findByTypeValueReq; //!< ATT Find By Type Vaue Request
  attReadByTypeReq_t readByTypeReq;           //!< ATT Read By Type Request
  attReadReq_t readReq;                       //!< ATT Read Request
  attReadBlobReq_t readBlobReq;               //!< ATT Read Blob Request
  attReadMultiReq_t readMultiReq;             //!< ATT Read Multiple Request
  attReadByGrpTypeReq_t readByGrpTypeReq;     //!< ATT Read By Group Type Request
  attWriteReq_t writeReq;                     //!< ATT Write Request
  attPrepareWriteReq_t prepareWriteReq;       //!< ATT Prepare Write Request
  attExecuteWriteReq_t executeWriteReq;       //!< ATT Execute Write Request

  // Response messages
  attErrorRsp_t errorRsp;                     //!< ATT Error Response
  attExchangeMTURsp_t exchangeMTURsp;         //!< ATT Exchange MTU Response
  attFindInfoRsp_t findInfoRsp;               //!< ATT Find Information Response
  attFindByTypeValueRsp_t findByTypeValueRsp; //!< ATT Find By Type Vaue Response
  attReadByTypeRsp_t readByTypeRsp;           //!< ATT Read By Type Response
  attReadRsp_t readRsp;                       //!< ATT Read Response
  attReadBlobRsp_t readBlobRsp;               //!< ATT Read Blob Response
  attReadMultiRsp_t readMultiRsp;             //!< ATT Read Multiple Response
  attReadByGrpTypeRsp_t readByGrpTypeRsp;     //!< ATT Read By Group Type Response
  attPrepareWriteRsp_t prepareWriteRsp;       //!< ATT Prepare Write Response

  // Indication and Notification messages
  attHandleValueNoti_t handleValueNoti;       //!< ATT Handle Value Notification
  attHandleValueInd_t handleValueInd;         //!< ATT Handle Value Indication
} attMsg_t;

/*********************************************************************
 * VARIABLES
 */

/*********************************************************************
 * API FUNCTIONS
 */

/*-------------------------------------------------------------------
 *  General Utility APIs
 */

/*
 * Parse an attribute protocol message.
 */
 unsigned char ATT_ParsePacket( l2capDataEvent_t *pL2capMsg, attPacket_t *pPkt );

/*
 * Compare two UUIDs. The UUIDs are converted if necessary.
 */
 unsigned char ATT_CompareUUID( const unsigned char *pUUID1, unsigned short int len1,
                              const unsigned char *pUUID2, unsigned short int len2 );
/*
 * Convert a 16-bit UUID to 128-bit UUID.
 */
 unsigned char ATT_ConvertUUIDto128( const unsigned char *pUUID16, unsigned char *pUUID128 );

/*
 * Convert a 128-bit UUID to 16-bit UUID.
 */
 unsigned char ATT_ConvertUUIDto16( const unsigned char *pUUID128, unsigned char *pUUID16 );


/*-------------------------------------------------------------------
 *  Attribute Client Utility APIs
 */

/*
 * Build Error Response.
 */
 unsigned short int ATT_BuildErrorRsp( unsigned char *pBuf, unsigned char *pMsg );

/*
 * Parse Error Response.
 */
 bStatus_t ATT_ParseErrorRsp( unsigned char *pParams, unsigned short int len, attMsg_t *pMsg );

/*
 * Build Exchange MTU Request.
 */
 unsigned short int ATT_BuildExchangeMTUReq( unsigned char *pBuf, unsigned char *pMsg );

/*
 * Build Exchange MTU Respnose.
 */
 unsigned short int ATT_BuildExchangeMTURsp( unsigned char *pBuf, unsigned char *pMsg );

/*
 * Parse Exchange MTU Response.
 */
 bStatus_t ATT_ParseExchangeMTURsp( unsigned char *pParams, unsigned short int len, attMsg_t *pMsg );

/*
 * Build Find Information Request.
 */
 unsigned short int ATT_BuildFindInfoReq( unsigned char *pBuf, unsigned char *pMsg );

/*
 * Parse Find Information Response.
 */
 bStatus_t ATT_ParseFindInfoRsp( unsigned char *pParams, unsigned short int len, attMsg_t *pMsg );

/*
 * Build Find Information Response.
 */
 unsigned short int ATT_BuildFindInfoRsp( unsigned char *pBuf, unsigned char *pMsg );

/*
 * Build Find By Type Value Request.
 */
 unsigned short int ATT_BuildFindByTypeValueReq( unsigned char *pBuf, unsigned char *pMsg );

/*
 * Build Find By Type Value Response.
 */
 unsigned short int ATT_BuildFindByTypeValueRsp( unsigned char *pBuf, unsigned char *pMsg );

/*
 * Parse Find By Type Value Response.
 */
 bStatus_t ATT_ParseFindByTypeValueRsp( unsigned char *pParams, unsigned short int len, attMsg_t *pMsg );

/*
 * Build Read By Type Request.
 */
 unsigned short int ATT_BuildReadByTypeReq( unsigned char *pBuf, unsigned char *pMsg );

/*
 * Build Read By Type Response.
 */
 unsigned short int ATT_BuildReadByTypeRsp( unsigned char *pBuf, unsigned char *pMsg );

/*
 * Parse Read By Type Response.
 */
 bStatus_t ATT_ParseReadByTypeRsp( unsigned char *pParams, unsigned short int len, attMsg_t *pMsg );

/*
 * Build Read Request.
 */
 unsigned short int ATT_BuildReadReq( unsigned char *pBuf, unsigned char *pMsg );

/*
 * Build Read Response.
 */
 unsigned short int ATT_BuildReadRsp( unsigned char *pBuf, unsigned char *pMsg );

/*
 * Parse Read Response.
 */
 bStatus_t ATT_ParseReadRsp( unsigned char *pParams, unsigned short int len, attMsg_t *pMsg );

/*
 * Build Read Blob Request.
 */
 unsigned short int ATT_BuildReadBlobReq( unsigned char *pBuf, unsigned char *pMsg );

/*
 * Build Read Blob Response.
 */
 unsigned short int ATT_BuildReadBlobRsp( unsigned char *pBuf, unsigned char *pMsg );

/*
 * Parse Read Blob Response.
 */
 bStatus_t ATT_ParseReadBlobRsp( unsigned char *pParams, unsigned short int len, attMsg_t *pMsg );

/*
 * Build Read Multiple Request.
 */
 unsigned short int ATT_BuildReadMultiReq( unsigned char *pBuf, unsigned char *pMsg );

/*
 * Build Read Multiple Response.
 */
 unsigned short int ATT_BuildReadMultiRsp( unsigned char *pBuf, unsigned char *pMsg );

/*
 * Parse Read Multiple Response.
 */
 bStatus_t ATT_ParseReadMultiRsp( unsigned char *pParams, unsigned short int len, attMsg_t *pMsg );

/*
 * Build Read By Group Type Response.
 */
 unsigned short int ATT_BuildReadByGrpTypeRsp( unsigned char *pBuf, unsigned char *pMsg );

/*
 * Parse Read By Group Type Response.
 */
 bStatus_t ATT_ParseReadByGrpTypeRsp( unsigned char *pParams, unsigned short int len, attMsg_t *pMsg );

/*
 * Build Write Request.
 */
 unsigned short int ATT_BuildWriteReq( unsigned char *pBuf, unsigned char *pMsg );

/*
 * Parse Write Response.
 */
 bStatus_t ATT_ParseWriteRsp( unsigned char *pParams, unsigned short int len, attMsg_t *pMsg );

/*
 * Build Prepare Write Request.
 */
 unsigned short int ATT_BuildPrepareWriteReq( unsigned char *pBuf, unsigned char *pMsg );

/*
 * Build Prepare Write Response.
 */
 unsigned short int ATT_BuildPrepareWriteRsp( unsigned char *pBuf, unsigned char *pMsg );

/*
 * Parse Prepare Write Response.
 */
 bStatus_t ATT_ParsePrepareWriteRsp( unsigned char *pParams, unsigned short int len, attMsg_t *pMsg );

/*
 * Build Execute Write Request.
 */
 unsigned short int ATT_BuildExecuteWriteReq( unsigned char *pBuf, unsigned char *pMsg );

/*
 * Parse Execute Write Response.
 */
 bStatus_t ATT_ParseExecuteWriteRsp( unsigned char *pParams, unsigned short int len, attMsg_t *pMsg );

/*
 * Build Handle Value Indication.
 */
 unsigned short int ATT_BuildHandleValueInd( unsigned char *pBuf, unsigned char *pMsg );

/*
 * Parse Handle Value Indication.
 */
 bStatus_t ATT_ParseHandleValueInd( unsigned char sig, unsigned char cmd, unsigned char *pParams, unsigned short int len, attMsg_t *pMsg );


/*-------------------------------------------------------------------
 *  Attribute Server Utility APIs
 */

/*
 * Parse Exchange MTU Request.
 */
 bStatus_t ATT_ParseExchangeMTUReq( unsigned char sig, unsigned char cmd, unsigned char *pParams, unsigned short int len, attMsg_t *pMsg );

/*
 * Parse Find Information Request.
 */
 bStatus_t ATT_ParseFindInfoReq( unsigned char sig, unsigned char cmd, unsigned char *pParams, unsigned short int len, attMsg_t *pMsg );

/*
 * Parse Find By Type Value Request.
 */
 bStatus_t ATT_ParseFindByTypeValueReq( unsigned char sig, unsigned char cmd, unsigned char *pParams, unsigned short int len, attMsg_t *pMsg );

/*
 * Parse Read By Type Request.
 */
 bStatus_t ATT_ParseReadByTypeReq( unsigned char sig, unsigned char cmd, unsigned char *pParams, unsigned short int len, attMsg_t *pMsg );

/*
 * Parse Read Request.
 */
 bStatus_t ATT_ParseReadReq( unsigned char sig, unsigned char cmd, unsigned char *pParams, unsigned short int len, attMsg_t *pMsg );

/*
 * Parse Write Blob Request.
 */
 bStatus_t ATT_ParseReadBlobReq( unsigned char sig, unsigned char cmd, unsigned char *pParams, unsigned short int len, attMsg_t *pMsg );

/*
 * Parse Read Multiple Request.
 */
 bStatus_t ATT_ParseReadMultiReq( unsigned char sig, unsigned char cmd, unsigned char *pParams, unsigned short int len, attMsg_t *pMsg );

/*
 * Parse Write Request.
 */
 bStatus_t ATT_ParseWriteReq( unsigned char sig, unsigned char cmd, unsigned char *pParams, unsigned short int len, attMsg_t *pMsg );

/*
 * Parse Execute Write Request.
 */
 bStatus_t ATT_ParseExecuteWriteReq( unsigned char sig, unsigned char cmd, unsigned char *pParams, unsigned short int len, attMsg_t *pMsg );

/*
 * Parse Prepare Write Request.
 */
 bStatus_t ATT_ParsePrepareWriteReq( unsigned char sig, unsigned char cmd, unsigned char *pParams, unsigned short int len, attMsg_t *pMsg );

/*
 * Parse Handle Value Confirmation.
 */
 bStatus_t ATT_ParseHandleValueCfm( unsigned char *pParams, unsigned short int len, attMsg_t *pMsg );


/*-------------------------------------------------------------------
 *  Attribute Client Public APIs
 */

/**
 * @defgroup ATT_CLIENT_API ATT Client API Functions
 * 
 * @{
 */

/**
 * @brief   Send Exchange MTU Request.
 *
 * @param   connHandle - connection to use
 * @param   pReq - pointer to request to be sent
 *
 * @return  SUCCESS: Request was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid request field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 */
 bStatus_t ATT_ExchangeMTUReq( unsigned short int connHandle, attExchangeMTUReq_t *pReq );

/**
 * @brief   Send Find Information Request.
 *
 * @param   connHandle - connection to use
 * @param   pReq - pointer to request to be sent
 *
 * @return  SUCCESS: Request was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid request field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 */
 bStatus_t ATT_FindInfoReq( unsigned short int connHandle, attFindInfoReq_t *pReq );

/**
 * @brief   Send Find By Type Value Request.
 *
 * @param   connHandle - connection to use
 * @param   pReq - pointer to request to be sent
 *
 * @return  SUCCESS: Request was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid request field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 */
 bStatus_t ATT_FindByTypeValueReq( unsigned short int connHandle, attFindByTypeValueReq_t *pReq );

/**
 * @brief   Send Read By Type Request.
 *
 * @param   connHandle - connection to use
 * @param   pReq - pointer to request to be sent
 *
 * @return  SUCCESS: Request was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid request field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 */
 bStatus_t ATT_ReadByTypeReq( unsigned short int connHandle, attReadByTypeReq_t *pReq );

/**
 * @brief   Send Read Request.
 *
 * @param   connHandle - connection to use
 * @param   pReq - pointer to request to be sent
 *
 * @return  SUCCESS: Request was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid request field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 */
 bStatus_t ATT_ReadReq( unsigned short int connHandle, attReadReq_t *pReq );

/**
 * @brief   Send Read Blob Request.
 *
 * @param   connHandle - connection to use
 * @param   pReq - pointer to request to be sent
 *
 * @return  SUCCESS: Request was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid request field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 */
 bStatus_t ATT_ReadBlobReq( unsigned short int connHandle, attReadBlobReq_t *pReq );

/**
 * @brief   Send Read Multiple Request.
 *
 * @param   connHandle - connection to use
 * @param   pReq - pointer to request to be sent
 *
 * @return  SUCCESS: Request was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid request field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 */
 bStatus_t ATT_ReadMultiReq( unsigned short int connHandle, attReadMultiReq_t *pReq );

/**
 * @brief   Send Read By Group Type Request.
 *
 * @param   connHandle - connection to use
 * @param   pReq - pointer to request to be sent
 *
 * @return  SUCCESS: Request was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid request field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 */
 bStatus_t ATT_ReadByGrpTypeReq( unsigned short int connHandle, attReadByGrpTypeReq_t *pReq );

/**
 * @brief   Send Write Request.
 *
 * @param   connHandle - connection to use
 * @param   pReq - pointer to request to be sent
 *
 * @return  SUCCESS: Request was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid request field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 *          bleLinkEncrypted: Connection is already encrypted.<BR>
 */
 bStatus_t ATT_WriteReq( unsigned short int connHandle, attWriteReq_t *pReq );

/**
 * @brief   Send Prepare Write Request.
 *
 * @param   connHandle - connection to use
 * @param   pReq - pointer to request to be sent
 *
 * @return  SUCCESS: Request was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid request field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 */
 bStatus_t ATT_PrepareWriteReq( unsigned short int connHandle, attPrepareWriteReq_t *pReq );

/**
 * @brief   Send Execute Write Request.
 *
 * @param   connHandle - connection to use
 * @param   pReq - pointer to request to be sent
 *
 * @return  SUCCESS: Request was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid request field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 */
 bStatus_t ATT_ExecuteWriteReq( unsigned short int connHandle, attExecuteWriteReq_t *pReq );

/**
 * @brief   Send Handle Value Confirmation.
 *
 * @param   connHandle - connection to use
 *
 * @return  SUCCESS: Confirmation was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid confirmation field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 */
 bStatus_t ATT_HandleValueCfm( unsigned short int connHandle );

/**
 * @}
 */

/*-------------------------------------------------------------------
 *  Attribute Server Public APIs
 */

/**
 * @defgroup ATT_SERVER_API ATT Server API Functions
 * 
 * @{
 */

/**
 * @brief   Send Error Response.
 *
 * @param   connHandle - connection to use
 * @param   pRsp - pointer to error response to be sent
 *
 * @return  SUCCESS: Response was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid response field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 */
 bStatus_t ATT_ErrorRsp( unsigned short int connHandle, attErrorRsp_t *pRsp );

/**
 * @brief   Send Exchange MTU Response.
 *
 * @param   connHandle - connection to use
 * @param   pRsp - pointer to request to be sent
 *
 * @return  SUCCESS: Response was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid response field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 */
 bStatus_t ATT_ExchangeMTURsp( unsigned short int connHandle, attExchangeMTURsp_t *pRsp );

/**
 * @brief   Send Find Information Response.
 *
 * @param   connHandle - connection to use
 * @param   pRsp - pointer to response to be sent
 *
 * @return  SUCCESS: Response was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid response field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 */
 bStatus_t ATT_FindInfoRsp( unsigned short int connHandle, attFindInfoRsp_t *pRsp );

/**
 * @brief   Send Find By Tyep Value Response.
 *
 * @param   connHandle - connection to use
 * @param   pRsp - pointer to response to be sent
 *
 * @return  SUCCESS: Response was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid response field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 */
 bStatus_t ATT_FindByTypeValueRsp( unsigned short int connHandle, attFindByTypeValueRsp_t *pRsp );

/**
 * @brief   Send Read By Type Respond.
 *
 * @param   connHandle - connection to use
 * @param   pRsp - pointer to response to be sent
 *
 * @return  SUCCESS: Response was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid response field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 */
 bStatus_t ATT_ReadByTypeRsp( unsigned short int connHandle, attReadByTypeRsp_t *pRsp );

/**
 * @brief   Send Read Response.
 *
 * @param   connHandle - connection to use
 * @param   pRsp - pointer to response to be sent
 *
 * @return  SUCCESS: Response was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid response field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 */
 bStatus_t ATT_ReadRsp( unsigned short int connHandle, attReadRsp_t *pRsp );

/**
 * @brief   Send Read Blob Response.
 *
 * @param   connHandle - connection to use
 * @param   pRsp - pointer to response to be sent
 *
 * @return  SUCCESS: Response was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid response field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 */
 bStatus_t ATT_ReadBlobRsp( unsigned short int connHandle, attReadBlobRsp_t *pRsp );

/**
 * @brief   Send Read Multiple Response.
 *
 * @param   connHandle - connection to use
 * @param   pRsp - pointer to response to be sent
 *
 * @return  SUCCESS: Response was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid response field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 */
 bStatus_t ATT_ReadMultiRsp( unsigned short int connHandle, attReadMultiRsp_t *pRsp ) ;

/**
 * @brief   Send Read By Group Type Respond.
 *
 * @param   connHandle - connection to use
 * @param   pRsp - pointer to response to be sent
 *
 * @return  SUCCESS: Response was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid response field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 */
 bStatus_t ATT_ReadByGrpTypeRsp( unsigned short int connHandle, attReadByGrpTypeRsp_t *pRsp );

/**
 * @brief   Send Write Response.
 *
 * @param   connHandle - connection to use
 *
 * @return  SUCCESS: Response was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid response field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 */
 bStatus_t ATT_WriteRsp( unsigned short int connHandle );

/**
 * @brief   Send Prepare Write Response.
 *
 * @param   connHandle - connection to use
 * @param   pRsp - pointer to response to be sent
 *
 * @return  SUCCESS: Response was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid response field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 */
 bStatus_t ATT_PrepareWriteRsp( unsigned short int connHandle, attPrepareWriteRsp_t *pRsp );

/**
 * @brief   Send Execute Write Response.
 *
 * @param   connHandle - connection to use
 *
 * @return  SUCCESS: Response was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid response field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 */
 bStatus_t ATT_ExecuteWriteRsp( unsigned short int connHandle );

/**
 * @brief   Send Handle Value Notification.
 *
 * @param   connHandle - connection to use
 * @param   pNoti - pointer to notification to be sent
 *
 * @return  SUCCESS: Notification was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid notification field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 */
 bStatus_t ATT_HandleValueNoti( unsigned short int connHandle, attHandleValueNoti_t *pNoti );

/**
 * @brief   Send Handle Value Indication.
 *
 * @param   connHandle - connection to use
 * @param   pInd - pointer to indication to be sent
 *
 * @return  SUCCESS: Indication was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid indication field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 */
 bStatus_t ATT_HandleValueInd( unsigned short int connHandle, attHandleValueInd_t *pInd );

/**
 * @}
 */

/**
 * @brief   Set a ATT Parameter value.  Use this function to change 
 *          the default ATT parameter values.
 *
 * @param   value - new param value
 *
 * @return  void
 */
 void ATT_SetParamValue( unsigned short int value );

/**
 * @brief   Get a ATT Parameter value.
 *
 * @param   none
 *
 * @return  ATT Parameter value
 */
 unsigned short int ATT_GetParamValue( void );


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* ATT_H */
