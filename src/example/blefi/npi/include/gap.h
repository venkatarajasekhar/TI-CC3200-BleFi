
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



/** ****************************************************************************
 *	@file gap.h
 *
 * 	@detail	Description:
 * 	THe header file of gap.c
 *	Encode and Decode For GAP packets. For GAP Command, fill GAP related packet
 *	and then send the packet to CC2650 through SPI; For GAP Event, now just
 *	return the Status when receive the packet from CC2650 through SPI.
 *
 *  ***************************************************************************/

#ifndef GAP_H_
#define GAP_H_

#ifdef __cplusplus
extern "C"
{
#endif
/*-------------------------------------------------------------------
 * INCLUDES
 */
#include "hci.h"
#include "sm.h"
#include "att.h"

// ******** Definitions ******************************************************//
/** @defgroup GAP_MSG_EVENT_DEFINES GAP Message IDs
 * @{
 */
#define GAP_DEVICE_INIT_DONE_EVENT            0x00 //!< Sent when the Device Initialization is complete.  This event is sent as an OSAL message defined as gapDeviceInitDoneEvent_t.
#define GAP_DEVICE_DISCOVERY_EVENT            0x01 //!< Sent when the Device Discovery Process is complete. This event is sent as an OSAL message defined as gapDevDiscEvent_t.
#define GAP_ADV_DATA_UPDATE_DONE_EVENT        0x02 //!< Sent when the Advertising Data or SCAN_RSP Data has been updated. This event is sent as an OSAL message defined as gapAdvDataUpdateEvent_t.
#define GAP_MAKE_DISCOVERABLE_DONE_EVENT      0x03 //!< Sent when the Make Discoverable Request is complete. This event is sent as an OSAL message defined as gapMakeDiscoverableRspEvent_t.
#define GAP_END_DISCOVERABLE_DONE_EVENT       0x04 //!< Sent when the Advertising has ended. This event is sent as an OSAL message defined as gapEndDiscoverableRspEvent_t.
#define GAP_LINK_ESTABLISHED_EVENT            0x05 //!< Sent when the Establish Link Request is complete. This event is sent as an OSAL message defined as gapEstLinkReqEvent_t.
#define GAP_LINK_TERMINATED_EVENT             0x06 //!< Sent when a connection was terminated. This event is sent as an OSAL message defined as gapTerminateLinkEvent_t.
#define GAP_LINK_PARAM_UPDATE_EVENT           0x07 //!< Sent when an Update Parameters Event is received. This event is sent as an OSAL message defined as gapLinkUpdateEvent_t.
#define GAP_RANDOM_ADDR_CHANGED_EVENT         0x08 //!< Sent when a random address was changed. This event is sent as an OSAL message defined as gapRandomAddrEvent_t.
#define GAP_SIGNATURE_UPDATED_EVENT           0x09 //!< Sent when the device's signature counter is updated. This event is sent as an OSAL message defined as gapSignUpdateEvent_t.
#define GAP_AUTHENTICATION_COMPLETE_EVENT     0x0A //!< Sent when the Authentication (pairing) process is complete. This event is sent as an OSAL message defined as gapAuthCompleteEvent_t.
#define GAP_PASSKEY_NEEDED_EVENT              0x0B //!< Sent when a Passkey is needed.  This is part of the pairing process. This event is sent as an OSAL message defined as gapPasskeyNeededEvent_t.
#define GAP_SLAVE_REQUESTED_SECURITY_EVENT    0x0C //!< Sent when a Slave Security Request is received. This event is sent as an OSAL message defined as gapSlaveSecurityReqEvent_t.
#define GAP_DEVICE_INFO_EVENT                 0x0D //!< Sent during the Device Discovery Process when a device is discovered. This event is sent as an OSAL message defined as gapDeviceInfoEvent_t.
#define GAP_BOND_COMPLETE_EVENT               0x0E //!< Sent when the bonding(bound) process is complete. This event is sent as an OSAL message defined as gapBondCompleteEvent_t.
#define GAP_PAIRING_REQ_EVENT                 0x0F //!< Sent when an unexpected Pairing Request is received. This event is sent as an OSAL message defined as gapPairingReqEvent_t.
/** @} End GAP_MSG_EVENT_DEFINES */

/** @defgroup GAP_CONN_HANDLE_DEFINES GAP Special Connection Handles
 * Used by GAP_TerminateLinkReq()
 * @{
 */
#define GAP_CONNHANDLE_INIT     0xFFFE  //!< terminates a link create
#define GAP_CONNHANDLE_ALL      0xFFFF  //!< terminates all links for the matching task ID.
/** @} End GAP_CONN_HANDLE_DEFINES */

/** @defgroup GAP_PROFILE_ROLE_DEFINES GAP Profile Roles
 * Bit mask values
 * @{
 */
#define GAP_PROFILE_BROADCASTER   0x01 //!< A device that sends advertising events only.
#define GAP_PROFILE_OBSERVER      0x02 //!< A device that receives advertising events only.
#define GAP_PROFILE_PERIPHERAL    0x04 //!< A device that accepts the establishment of an LE physical link using the connection establishment procedure
#define GAP_PROFILE_CENTRAL       0x08 //!< A device that supports the Central role initiates the establishment of a physical connection
/** @} End GAP_PROFILE_ROLE_DEFINES */

/**
 * @defgroup GAP_PARAMETER_ID_DEFINES GAP Parameter IDs
 * Used in place of gapParamIDs_t.
 * @{
 */
// Timers
#define TGAP_GEN_DISC_ADV_MIN          0  //!< Minimum time to remain advertising, when in Discoverable mode (mSec).  Setting this parameter to 0 turns off the timeout (default).
#define TGAP_LIM_ADV_TIMEOUT           1  //!< Maximum time to remain advertising, when in Limited Discoverable mode. In seconds (default 180 seconds)
#define TGAP_GEN_DISC_SCAN             2  //!< Minimum time to perform scanning, when performing General Discovery proc (mSec)
#define TGAP_LIM_DISC_SCAN             3  //!< Minimum time to perform scanning, when performing Limited Discovery proc (mSec)
#define TGAP_CONN_EST_ADV_TIMEOUT      4  //!< Advertising timeout, when performing Connection Establishment proc (mSec)
#define TGAP_CONN_PARAM_TIMEOUT        5  //!< Link Layer connection parameter update notification timer, connection parameter update proc (mSec)

// Constants
#define TGAP_LIM_DISC_ADV_INT_MIN      6  //!< Minimum advertising interval, when in limited discoverable mode (n * 0.625 mSec)
#define TGAP_LIM_DISC_ADV_INT_MAX      7  //!< Maximum advertising interval, when in limited discoverable mode (n * 0.625 mSec)
#define TGAP_GEN_DISC_ADV_INT_MIN      8  //!< Minimum advertising interval, when in General discoverable mode (n * 0.625 mSec)
#define TGAP_GEN_DISC_ADV_INT_MAX      9  //!< Maximum advertising interval, when in General discoverable mode (n * 0.625 mSec)
#define TGAP_CONN_ADV_INT_MIN         10  //!< Minimum advertising interval, when in Connectable mode (n * 0.625 mSec)
#define TGAP_CONN_ADV_INT_MAX         11  //!< Maximum advertising interval, when in Connectable mode (n * 0.625 mSec)
#define TGAP_CONN_SCAN_INT            12  //!< Scan interval used during Link Layer Initiating state, when in Connectable mode (n * 0.625 mSec)
#define TGAP_CONN_SCAN_WIND           13  //!< Scan window used during Link Layer Initiating state, when in Connectable mode (n * 0.625 mSec)
#define TGAP_CONN_HIGH_SCAN_INT       14  //!< Scan interval used during Link Layer Initiating state, when in Connectable mode, high duty scan cycle scan paramaters (n * 0.625 mSec)
#define TGAP_CONN_HIGH_SCAN_WIND      15  //!< Scan window used during Link Layer Initiating state, when in Connectable mode, high duty scan cycle scan paramaters (n * 0.625 mSec)
#define TGAP_GEN_DISC_SCAN_INT        16  //!< Scan interval used during Link Layer Scanning state, when in General Discovery proc (n * 0.625 mSec)
#define TGAP_GEN_DISC_SCAN_WIND       17  //!< Scan window used during Link Layer Scanning state, when in General Discovery proc (n * 0.625 mSec)
#define TGAP_LIM_DISC_SCAN_INT        18  //!< Scan interval used during Link Layer Scanning state, when in Limited Discovery proc (n * 0.625 mSec)
#define TGAP_LIM_DISC_SCAN_WIND       19  //!< Scan window used during Link Layer Scanning state, when in Limited Discovery proc (n * 0.625 mSec)
#define TGAP_CONN_EST_ADV             20  //!< Advertising interval, when using Connection Establishment proc (n * 0.625 mSec). Obsolete - Do not use.
#define TGAP_CONN_EST_INT_MIN         21  //!< Minimum Link Layer connection interval, when using Connection Establishment proc (n * 1.25 mSec)
#define TGAP_CONN_EST_INT_MAX         22  //!< Maximum Link Layer connection interval, when using Connection Establishment proc (n * 1.25 mSec)
#define TGAP_CONN_EST_SCAN_INT        23  //!< Scan interval used during Link Layer Initiating state, when using Connection Establishment proc (n * 0.625 mSec)
#define TGAP_CONN_EST_SCAN_WIND       24  //!< Scan window used during Link Layer Initiating state, when using Connection Establishment proc (n * 0.625 mSec)
#define TGAP_CONN_EST_SUPERV_TIMEOUT  25  //!< Link Layer connection supervision timeout, when using Connection Establishment proc (n * 10 mSec)
#define TGAP_CONN_EST_LATENCY         26  //!< Link Layer connection slave latency, when using Connection Establishment proc (in number of connection events)
#define TGAP_CONN_EST_MIN_CE_LEN      27  //!< Local informational parameter about min len of connection needed, when using Connection Establishment proc (n * 0.625 mSec)
#define TGAP_CONN_EST_MAX_CE_LEN      28  //!< Local informational parameter about max len of connection needed, when using Connection Establishment proc (n * 0.625 mSec)
#define TGAP_PRIVATE_ADDR_INT         29  //!< Minimum Time Interval between private (resolvable) address changes. In minutes (default 15 minutes)
#define TGAP_CONN_PAUSE_CENTRAL       30  //!< Central idle timer. In seconds (default 1 second)
#define TGAP_CONN_PAUSE_PERIPHERAL    31  //!< Minimum time upon connection establishment before the peripheral starts a connection update procedure. In seconds (default 5 seconds)

// Proprietary
#define TGAP_SM_TIMEOUT               32  //!< SM Message Timeout (milliseconds). Default 30 seconds.
#define TGAP_SM_MIN_KEY_LEN           33  //!< SM Minimum Key Length supported. Default 7.
#define TGAP_SM_MAX_KEY_LEN           34  //!< SM Maximum Key Length supported. Default 16.
#define TGAP_FILTER_ADV_REPORTS       35  //!< Filter duplicate advertising reports. Default TRUE.
#define TGAP_SCAN_RSP_RSSI_MIN        36  //!< Minimum RSSI required for scan responses to be reported to the app. Default -127.
#define TGAP_REJECT_CONN_PARAMS       37  //!< Whether or not to reject Connection Parameter Update Request received on Central device. Default FALSE.

#if !defined ( TESTMODES )
  #define TGAP_AUTH_TASK_ID           38  //!< Task ID override for Task Authentication control (for stack internal use only)
  #define TGAP_PARAMID_MAX            39  //!< ID MAX-valid Parameter ID
#else
  #define TGAP_GAP_TESTCODE           38  //!< GAP TestCodes - puts GAP into a test mode
  #define TGAP_SM_TESTCODE            39  //!< SM TestCodes - puts SM into a test mode
  #define TGAP_AUTH_TASK_ID           40  //!< Task ID override for Task Authentication control (for stack internal use only)
  #define TGAP_PARAMID_MAX            41  //!< ID MAX-valid Parameter ID

  #define TGAP_GATT_TESTCODE          100 //!< GATT TestCodes - puts GATT into a test mode (paramValue maintained by GATT)
  #define TGAP_ATT_TESTCODE           101 //!< ATT TestCodes - puts ATT into a test mode (paramValue maintained by ATT)
  #define TGAP_GGS_TESTCODE           102 //!< GGS TestCodes - puts GGS into a test mode (paramValue maintained by GGS)
#endif

/** @} End GAP_PARAMETER_ID_DEFINES */

/** @defgroup GAP_DEVDISC_MODE_DEFINES GAP Device Discovery Modes
 * @{
 */
#define DEVDISC_MODE_NONDISCOVERABLE  0x00    //!< No discoverable setting
#define DEVDISC_MODE_GENERAL          0x01    //!< General Discoverable devices
#define DEVDISC_MODE_LIMITED          0x02    //!< Limited Discoverable devices
#define DEVDISC_MODE_ALL              0x03    //!< Not filtered
/** @} End GAP_DEVDISC_MODE_DEFINES */

/** @defgroup GAP_ADDR_TYPE_DEFINES GAP Address Types
 * @{
 */
#define ADDRTYPE_PUBLIC               0x00  //!< Use the BD_ADDR
#define ADDRTYPE_STATIC               0x01  //!< Static address
#define ADDRTYPE_PRIVATE_NONRESOLVE   0x02  //!< Generate Non-Resolvable Private Address
#define ADDRTYPE_PRIVATE_RESOLVE      0x03  //!< Generate Resolvable Private Address
/** @} End GAP_ADDR_TYPE_DEFINES */

/** @defgroup GAP_ADVERTISEMENT_TYPE_DEFINES GAP Advertising Event Types
 * for eventType field in gapAdvertisingParams_t
 * @{
 */
#define GAP_ADTYPE_ADV_IND                0x00  //!< Connectable undirected advertisement
#define GAP_ADTYPE_ADV_HDC_DIRECT_IND     0x01  //!< Connectable high duty cycle directed advertisement
#define GAP_ADTYPE_ADV_SCAN_IND           0x02  //!< Scannable undirected advertisement
#define GAP_ADTYPE_ADV_NONCONN_IND        0x03  //!< Non-Connectable undirected advertisement
#define GAP_ADTYPE_ADV_LDC_DIRECT_IND     0x04  //!< Connectable low duty cycle directed advertisement
/** @} End GAP_ADVERTISEMENT_TYPE_DEFINES */

/** @defgroup GAP_ADVERTISEMENT_REPORT_TYPE_DEFINES GAP Advertising Report Event Types
 * for eventType field in gapDevRec_t and gapDeviceInfoEvent_t
 * @{
 */
#define GAP_ADRPT_ADV_IND                 0x00  //!< Connectable undirected advertisement
#define GAP_ADRPT_ADV_DIRECT_IND          0x01  //!< Connectable directed advertisement
#define GAP_ADRPT_ADV_SCAN_IND            0x02  //!< Scannable undirected advertisement
#define GAP_ADRPT_ADV_NONCONN_IND         0x03  //!< Non-Connectable undirected advertisement
#define GAP_ADRPT_SCAN_RSP                0x04  //!< Scan Response
/** @} End GAP_ADVERTISEMENT_REPORT_TYPE_DEFINES */

/** @defgroup GAP_FILTER_POLICY_DEFINES GAP Advertiser Filter Scan Parameters
 * @{
 */
#define GAP_FILTER_POLICY_ALL         0x00 //!< Allow Scan Request from Any, Allow Connect Request from Any (default).
#define GAP_FILTER_POLICY_WHITE_SCAN  0x01 //!< Allow Scan Request from White List Only, Allow Connect from Any
#define GAP_FILTER_POLICY_WHITE_CON   0x02 //!< Allow Scan Request from Any, Connect from White List Only
#define GAP_FILTER_POLICY_WHITE       0x03 //!< Allow Scan Request and Connect from White List Only
/** @} End GAP_FILTER_POLICY_DEFINES */

//! Advertiser Channel Map
#define ADV_CHANMAP_SIZE                 5

//! Maximum Pairing Passcode/Passkey value.  Range of a passkey can be 0 - 999,999.
#define GAP_PASSCODE_MAX                 999999

/** Sign Counter Initialized - Sign counter hasn't been used yet.  Used when setting up
 *  a connection's signing information.
 */
#define GAP_INIT_SIGN_COUNTER            0xFFFFFFFF

/** @defgroup GAP_ADVCHAN_DEFINES GAP Advertisement Channel Map
 * @{
 */
#define GAP_ADVCHAN_37  0x01  //!< Advertisement Channel 37
#define GAP_ADVCHAN_38  0x02  //!< Advertisement Channel 38
#define GAP_ADVCHAN_39  0x04  //!< Advertisement Channel 39
#define GAP_ADVCHAN_ALL (GAP_ADVCHAN_37 | GAP_ADVCHAN_38 | GAP_ADVCHAN_39) //!< All Advertisement Channels Enabled
/** @} End GAP_ADVCHAN_DEFINES */

/** @defgroup GAP_WHITELIST_DEFINES GAP White List Options
 * @{
 */
#define WL_NOTUSED    0x00    //!< White list not used but the advertiser's address in this command is used
#define WL_USED       0x01    //!< White list is used and the advertiser's address in this command is not used.
/** @} End GAP_WHITELIST_DEFINES */

/** @defgroup GAP_ADTYPE_DEFINES GAP Advertisment Data Types
 * These are the data type identifiers for the data tokens in the advertisement data field.
 * @{
 */
#define GAP_ADTYPE_FLAGS                        0x01 //!< Discovery Mode: @ref GAP_ADTYPE_FLAGS_MODES
#define GAP_ADTYPE_16BIT_MORE                   0x02 //!< Service: More 16-bit UUIDs available
#define GAP_ADTYPE_16BIT_COMPLETE               0x03 //!< Service: Complete list of 16-bit UUIDs
#define GAP_ADTYPE_32BIT_MORE                   0x04 //!< Service: More 32-bit UUIDs available
#define GAP_ADTYPE_32BIT_COMPLETE               0x05 //!< Service: Complete list of 32-bit UUIDs
#define GAP_ADTYPE_128BIT_MORE                  0x06 //!< Service: More 128-bit UUIDs available
#define GAP_ADTYPE_128BIT_COMPLETE              0x07 //!< Service: Complete list of 128-bit UUIDs
#define GAP_ADTYPE_LOCAL_NAME_SHORT             0x08 //!< Shortened local name
#define GAP_ADTYPE_LOCAL_NAME_COMPLETE          0x09 //!< Complete local name
#define GAP_ADTYPE_POWER_LEVEL                  0x0A //!< TX Power Level: 0xXX: -127 to +127 dBm
#define GAP_ADTYPE_OOB_CLASS_OF_DEVICE          0x0D //!< Simple Pairing OOB Tag: Class of device (3 octets)
#define GAP_ADTYPE_OOB_SIMPLE_PAIRING_HASHC     0x0E //!< Simple Pairing OOB Tag: Simple Pairing Hash C (16 octets)
#define GAP_ADTYPE_OOB_SIMPLE_PAIRING_RANDR     0x0F //!< Simple Pairing OOB Tag: Simple Pairing Randomizer R (16 octets)
#define GAP_ADTYPE_SM_TK                        0x10 //!< Security Manager TK Value
#define GAP_ADTYPE_SM_OOB_FLAG                  0x11 //!< Secutiry Manager OOB Flags
#define GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE    0x12 //!< Min and Max values of the connection interval (2 octets Min, 2 octets Max) (0xFFFF indicates no conn interval min or max)
#define GAP_ADTYPE_SIGNED_DATA                  0x13 //!< Signed Data field
#define GAP_ADTYPE_SERVICES_LIST_16BIT          0x14 //!< Service Solicitation: list of 16-bit Service UUIDs
#define GAP_ADTYPE_SERVICES_LIST_128BIT         0x15 //!< Service Solicitation: list of 128-bit Service UUIDs
#define GAP_ADTYPE_SERVICE_DATA                 0x16 //!< Service Data
#define GAP_ADTYPE_APPEARANCE                   0x19 //!< Appearance
#define GAP_ADTYPE_MANUFACTURER_SPECIFIC        0xFF //!< Manufacturer Specific Data: first 2 octets contain the Company Identifier Code followed by the additional manufacturer specific data
/** @} End GAP_ADTYPE_DEFINES */

/** @defgroup GAP_ADTYPE_FLAGS_MODES GAP ADTYPE Flags Discovery Modes
 * @{
 */
#define GAP_ADTYPE_FLAGS_LIMITED                0x01 //!< Discovery Mode: LE Limited Discoverable Mode
#define GAP_ADTYPE_FLAGS_GENERAL                0x02 //!< Discovery Mode: LE General Discoverable Mode
#define GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED    0x04 //!< Discovery Mode: BR/EDR Not Supported
/** @} End GAP_ADTYPE_FLAGS_MODES */

/** @defgroup GAP_APPEARANCE_VALUES GAP Appearance Values
 * @{
 */
#define GAP_APPEARE_UNKNOWN                     0x0000 //!< Unknown
#define GAP_APPEARE_GENERIC_PHONE               0x0040 //!< Generic Phone
#define GAP_APPEARE_GENERIC_COMPUTER            0x0080 //!< Generic Computer
#define GAP_APPEARE_GENERIC_WATCH               0x00C0 //!< Generic Watch
#define GAP_APPEARE_WATCH_SPORTS                0x00C1 //!< Watch: Sports Watch
#define GAP_APPEARE_GENERIC_CLOCK               0x0100 //!< Generic Clock
#define GAP_APPEARE_GENERIC_DISPLAY             0x0140 //!< Generic Display
#define GAP_APPEARE_GENERIC_RC                  0x0180 //!< Generic Remote Control
#define GAP_APPEARE_GENERIC_EYE_GALSSES         0x01C0 //!< Generic Eye-glasses
#define GAP_APPEARE_GENERIC_TAG                 0x0200 //!< Generic Tag
#define GAP_APPEARE_GENERIC_KEYRING             0x0240 //!< Generic Keyring
#define GAP_APPEARE_GENERIC_MEDIA_PLAYER        0x0280 //!< Generic Media Player
#define GAP_APPEARE_GENERIC_BARCODE_SCANNER     0x02C0 //!< Generic Barcode Scanner
#define GAP_APPEARE_GENERIC_THERMOMETER         0x0300 //!< Generic Thermometer
#define GAP_APPEARE_GENERIC_THERMO_EAR          0x0301 //!< Thermometer: Ear
#define GAP_APPEARE_GENERIC_HR_SENSOR           0x0340 //!< Generic Heart rate Sensor
#define GAP_APPEARE_GENERIC_HRS_BELT            0x0341 //!< Heart Rate Sensor: Heart Rate Belt
#define GAP_APPEARE_GENERIC_BLOOD_PRESSURE      0x0380 //!< Generic Blood Pressure
#define GAP_APPEARE_GENERIC_BP_ARM              0x0381 //!< Blood Pressure: Arm
#define GAP_APPEARE_GENERIC_BP_WRIST            0x0382 //!< Blood Pressure: Wrist
#define GAP_APPEARE_GENERIC_HID                 0x03C0 //!< Generic Human Interface Device (HID)
#define GAP_APPEARE_HID_KEYBOARD                0x03C1 //!< HID Keyboard
#define GAP_APPEARE_HID_MOUSE                   0x03C2 //!< HID Mouse
#define GAP_APPEARE_HID_JOYSTIC                 0x03C3 //!< HID Joystick
#define GAP_APPEARE_HID_GAMEPAD                 0x03C4 //!< HID Gamepad
#define GAP_APPEARE_HID_DIGITIZER_TYABLET       0x03C5 //!< HID Digitizer Tablet
#define GAP_APPEARE_HID_DIGITAL_CARDREADER      0x03C6 //!< HID Card Reader
#define GAP_APPEARE_HID_DIGITAL_PEN             0x03C7 //!< HID Digital Pen
#define GAP_APPEARE_HID_BARCODE_SCANNER         0x03C8 //!< HID Barcode Scanner
/** @} End GAP_APPEARANCE_VALUES */

// ******** Enumerations ******************************************************** //


typedef enum 	//function GAP_LinkEstablished
{
	PPM_500	= 0x00,
	PPM_250 = 0x01,
	PPM_150 = 0x02,
	PPM_100 = 0x03,
	PPM_75  = 0x04,
	PPM_50  = 0x05,
	PPM_30  = 0x06,
	PPM_20  = 0x07
}GAP_ClockAccuracy_t;


// ******** Struct ****************************************************************** //



/**
 * GAP Parameters IDs: @ref GAP_PARAMETER_ID_DEFINES
 */
typedef unsigned short int gapParamIDs_t;

/**
 * GAP event header format.
 */
typedef struct __attribute__((__packed__))
{
	EventHeader_t  hdr;           	//!< GAP_MSG_EVENT
	unsigned char opcode;                    //!< GAP type of command. Ref: @ref GAP_MSG_EVENT_DEFINES
	unsigned char event;
	bStatus_t status;
} gapEventHdr_t;

/**
 * GAP_RANDOM_ADDR_CHANGED_EVENT message format.  This message is sent to the
 * app when the random address changes.
 */
typedef struct __attribute__((__packed__))
{
  EventHeader_t  hdr;           	//!< GAP_MSG_EVENT
  unsigned char opcode;                    //!< GAP type of command. Ref: @ref GAP_MSG_EVENT_DEFINES
  unsigned char event;
  bStatus_t status;
  unsigned char addrType;                     //!< Address type: @ref GAP_ADDR_TYPE_DEFINES
  unsigned char newRandomAddr[B_ADDR_LEN];    //!< the new calculated private addr
} gapRandomAddrEvent_t;

/**
 * Connection parameters for the peripheral device.  These numbers are used
 * to compare against connection events and request connection parameter
 * updates with the master.
 */
typedef struct __attribute__((__packed__))
{
  /** Minimum value for the connection event (interval. 0x0006 - 0x0C80 * 1.25 ms) */
  unsigned short int intervalMin;
  /** Maximum value for the connection event (interval. 0x0006 - 0x0C80 * 1.25 ms) */
  unsigned short int intervalMax;
  /** Number of LL latency connection events (0x0000 - 0x03e8) */
  unsigned short int latency;
  /** Connection Timeout (0x000A - 0x0C80 * 10 ms) */
  unsigned short int timeout;
} gapPeriConnectParams_t;

/**
 * GAP_DEVICE_INIT_DONE_EVENT message format.  This message is sent to the
 * app when the Device Initialization is done [initiated by calling
 * GAP_DeviceInit()].
 */
typedef struct __attribute__((__packed__))
{
  EventHeader_t  hdr;           	//!< GAP_MSG_EVENT
  unsigned char opcode;                    //!< GAP type of command. Ref: @ref GAP_MSG_EVENT_DEFINES
  unsigned char event;
  bStatus_t status;
  unsigned char devAddr[B_ADDR_LEN];          //!< Device's BD_ADDR
  unsigned short int dataPktLen;                  //!< HC_LE_Data_Packet_Length
  unsigned char numDataPkts;                  //!< HC_Total_Num_LE_Data_Packets
  uint8 IRK[16];                      //!< 16 Byte Idetity Resolving Key
  uint8 CSRK[16];                     //!< 16 Byte Connection Signature Resolving Key
} gapDeviceInitDoneEvent_t;

/**
 * GAP_SIGNATURE_UPDATED_EVENT message format.  This message is sent to the
 * app when the signature counter has changed.  This message is to inform the
 * application in case it wants to save it to be restored on reboot or reconnect.
 * This message is sent to update a connection's signature counter and to update
 * this device's signature counter.  If devAddr == BD_ADDR, then this message pertains
 * to this device.
 */
typedef struct __attribute__((__packed__))
{
  EventHeader_t  hdr;           	//!< GAP_MSG_EVENT
  unsigned char opcode;                    //!< GAP type of command. Ref: @ref GAP_MSG_EVENT_DEFINES
  unsigned char event;
  bStatus_t status;
  unsigned char addrType;                     //!< Device's address type for devAddr
  unsigned char devAddr[B_ADDR_LEN];          //!< Device's BD_ADDR, could be own address
  unsigned long signCounter;                 //!< new Signed Counter
} gapSignUpdateEvent_t;

/**
 * GAP_DEVICE_INFO_EVENT message format.  This message is sent to the
 * app during a Device Discovery Request, when a new advertisement or scan
 * response is received.
 */
typedef struct __attribute__((__packed__))
{
  EventHeader_t  hdr;           	//!< GAP_MSG_EVENT
  unsigned char opcode;                    //!< GAP type of command. Ref: @ref GAP_MSG_EVENT_DEFINES
  unsigned char event;
  bStatus_t status;
  unsigned char eventType;          //!< Advertisement Type: @ref GAP_ADVERTISEMENT_REPORT_TYPE_DEFINES
  unsigned char addrType;           //!< address type: @ref GAP_ADDR_TYPE_DEFINES
  unsigned char addr[B_ADDR_LEN];   //!< Address of the advertisement or SCAN_RSP
  unsigned char rssi;                //!< Advertisement or SCAN_RSP RSSI
  unsigned char dataLen;            //!< Length (in uint8s) of the data field (evtData)
  unsigned char *pEvtData;          //!< Data field of advertisement or SCAN_RSP
} gapDeviceInfoEvent_t;

/*-------------------------------------------------------------------
 * TYPEDEFS - Device Discovery
 */

/**
 * Type of device discovery (Scan) to perform.
 */
typedef struct __attribute__((__packed__))
{
  unsigned char mode;         //!< Discovery Mode: @ref GAP_DEVDISC_MODE_DEFINES
  unsigned char activeScan;   //!< TRUE for active scanning
  unsigned char whiteList;    //!< TRUE to only allow advertisements from devices in the white list.
} gapDevDiscReq_t;

/**
 * Type of device discovery (Scan) to perform.
 */
typedef struct __attribute__((__packed__))
{
  unsigned char eventType;        //!< Indicates advertising event type used by the advertiser: @ref GAP_ADVERTISEMENT_REPORT_TYPE_DEFINES
  unsigned char addrType;         //!< Address Type: @ref GAP_ADDR_TYPE_DEFINES
  unsigned char addr[B_ADDR_LEN]; //!< Device's Address
} gapDevRec_t;

/**
 * GAP_DEVICE_DISCOVERY_EVENT message format. This message is sent to the
 * Application after a scan is performed.
 */
typedef struct __attribute__((__packed__))
{
	EventHeader_t  hdr;           	//!< GAP_MSG_EVENT
	unsigned char opcode;                    //!< GAP type of command. Ref: @ref GAP_MSG_EVENT_DEFINES
	unsigned char event;
	bStatus_t status;
	unsigned char numDevs;         //!< Number of devices found during scan
  //gapDevRec_t *pDevList; //!< array of device records
  	gapDevRec_t DevList; //!< array of device records
} gapDevDiscEvent_t;

/**
 * Advertising Parameters
 */
typedef struct __attribute__((__packed__))
{
  unsigned char eventType;          //!< Advertise Event Type: @ref GAP_ADVERTISEMENT_TYPE_DEFINES
  unsigned char initiatorAddrType;  //!< Initiator's address type: @ref GAP_ADDR_TYPE_DEFINES
  unsigned char initiatorAddr[B_ADDR_LEN];  //!< Initiator's addr - used only with connectable directed eventType (ADV_EVTTYPE_CONNECTABLE_DIRECTED).
  unsigned char channelMap;         //!< Channel Map: Bit mask @ref GAP_ADVCHAN_DEFINES
  unsigned char filterPolicy;       //!< Filer Policy: @ref GAP_FILTER_POLICY_DEFINES. Ignored when directed advertising is used.
} gapAdvertisingParams_t;

/**
 * GAP_MAKE_DISCOVERABLE_DONE_EVENT message format.  This message is sent to the
 * app when the Advertise config is complete.
 */
typedef struct __attribute__((__packed__))
{
	EventHeader_t  hdr;           	//!< GAP_MSG_EVENT
	unsigned char opcode;                    //!< GAP type of command. Ref: @ref GAP_MSG_EVENT_DEFINES
	unsigned char event;
	bStatus_t status;
	unsigned short int interval;       //!< actual advertising interval selected by controller
} gapMakeDiscoverableRspEvent_t;

/**
 * GAP_END_DISCOVERABLE_DONE_EVENT message format.  This message is sent to the
 * app when the Advertising has stopped.
 */
typedef struct __attribute__((__packed__))
{
	EventHeader_t  hdr;           	//!< GAP_MSG_EVENT
	unsigned char opcode;                    //!< GAP type of command. Ref: @ref GAP_MSG_EVENT_DEFINES
	unsigned char event;
	bStatus_t status;          //!< GAP_END_DISCOVERABLE_DONE_EVENT
} gapEndDiscoverableRspEvent_t;

/**
 * GAP_ADV_DATA_UPDATE_DONE_EVENT message format.  This message is sent to the
 * app when Advertising Data Update is complete.
 */
typedef struct __attribute__((__packed__))
{
	EventHeader_t  hdr;           	//!< GAP_MSG_EVENT
	unsigned char opcode;                    //!< GAP type of command. Ref: @ref GAP_MSG_EVENT_DEFINES
	unsigned char event;
	bStatus_t status;
	unsigned char adType;          //!< TRUE if advertising data, FALSE if SCAN_RSP
} gapAdvDataUpdateEvent_t;

/*-------------------------------------------------------------------
 * TYPEDEFS - Link Establishment
 */

/**
 * Establish Link Request parameters
 */
typedef struct __attribute__((__packed__))
{
  unsigned char highDutyCycle;        //!< TRUE to high duty cycle scan, FALSE if not.
  unsigned char whiteList;            //!< Determines use of the white list: @ref GAP_WHITELIST_DEFINES
  unsigned char addrTypePeer;         //!< Address type of the advertiser: @ref GAP_ADDR_TYPE_DEFINES
  unsigned char peerAddr[B_ADDR_LEN]; //!< Advertiser's address
} gapEstLinkReq_t;

/**
 * Update Link Parameters Request parameters
 */
typedef struct __attribute__((__packed__))
{
  unsigned short int connectionHandle; //!< Connection handle of the update
  unsigned short int intervalMin;      //!< Minimum Connection Interval
  unsigned short int intervalMax;      //!< Maximum Connection Interval
  unsigned short int connLatency;      //!< Connection Latency
  unsigned short int connTimeout;      //!< Connection Timeout
} gapUpdateLinkParamReq_t;

/**
 * GAP_LINK_ESTABLISHED_EVENT message format.  This message is sent to the app
 * when the link request is complete.<BR>
 * <BR>
 * For an Observer, this message is sent to complete the Establish Link Request.<BR>
 * For a Peripheral, this message is sent to indicate that a link has been created.
 */
typedef struct __attribute__((__packed__))
{
  EventHeader_t  hdr;           	//!< GAP_MSG_EVENT
  unsigned char opcode;                    //!< GAP type of command. Ref: @ref GAP_MSG_EVENT_DEFINES
  unsigned char event;
  unsigned char status;
  unsigned char devAddrType;         //!< Device address type: @ref GAP_ADDR_TYPE_DEFINES
  unsigned char devAddr[B_ADDR_LEN]; //!< Device address of link
  unsigned short int connectionHandle;   //!< Connection Handle from controller used to ref the device
  unsigned short int connInterval;       //!< Connection Interval
  unsigned short int connLatency;        //!< Conenction Latency
  unsigned short int connTimeout;        //!< Connection Timeout
  unsigned char clockAccuracy;       //!< Clock Accuracy
} gapEstLinkReqEvent_t;

/**
 * GAP_LINK_PARAM_UPDATE_EVENT message format.  This message is sent to the app
 * when the connection parameters update request is complete.
 */
typedef struct __attribute__((__packed__))
{
  EventHeader_t  hdr;           	//!< GAP_MSG_EVENT
  unsigned char opcode;                    //!< GAP type of command. Ref: @ref GAP_MSG_EVENT_DEFINES
  unsigned char event;
  unsigned char status;             //!< bStatus_t
  unsigned short int connectionHandle;  //!< Connection handle of the update
  unsigned short int connInterval;      //!< Requested connection interval
  unsigned short int connLatency;       //!< Requested connection latency
  unsigned short int connTimeout;       //!< Requested connection timeout
} gapLinkUpdateEvent_t;

/**
 * GAP_LINK_TERMINATED_EVENT message format.  This message is sent to the
 * app when a link to a device is terminated.
 */
typedef struct __attribute__((__packed__))
{
  EventHeader_t  hdr;           	//!< GAP_MSG_EVENT
  unsigned char opcode;                    //!< GAP type of command. Ref: @ref GAP_MSG_EVENT_DEFINES
  unsigned char event;
  bStatus_t status;
  unsigned short int connectionHandle; //!< connection Handle
  unsigned char reason;            //!< termination reason from LL
} gapTerminateLinkEvent_t;


/*-------------------------------------------------------------------
 * TYPEDEFS - Authentication, Bounding and Pairing
 */

/**
 * GAP_PASSKEY_NEEDED_EVENT message format.  This message is sent to the
 * app when a Passkey is needed from the app's user interface.
 */
typedef struct __attribute__((__packed__))
{
	EventHeader_t  hdr;           	//!< GAP_MSG_EVENT
	unsigned char opcode;                    //!< GAP type of command. Ref: @ref GAP_MSG_EVENT_DEFINES
	unsigned char event;
	bStatus_t status;
	unsigned char deviceAddr[B_ADDR_LEN]; 	//!< address of device to pair with, and could be either public or random.
	unsigned short int connectionHandle;      	//!< Connection handle
	unsigned char uiInputs;              	//!< Pairing User Interface Inputs - Ask user to input passcode
	unsigned char uiOutputs;              	//!< Pairing User Interface Outputs - Display passcode
} gapPasskeyNeededEvent_t;

/**
 * GAP_AUTHENTICATION_COMPLETE_EVENT message format.  This message is sent to the app
 * when the authentication request is complete.
 */
typedef struct __attribute__((__packed__))
{
	EventHeader_t  hdr;           	//!< GAP_MSG_EVENT
	uint8 opcode;                    //!< GAP type of command. Ref: @ref GAP_MSG_EVENT_DEFINES
	uint8 event;
	bStatus_t status;
	uint16 connectionHandle;         	//!< Connection Handle from controller used to ref the device
	uint8 authState;                 	//!< TRUE if the pairing was authenticated (MITM)
	smSecInfo_t   pSecurityInfo; 	    //!< BOUND - security information from this device
	smSecInfo_t   pDevSecInfo;   	    //!< BOUND - security information from connected device
	smIdInfo_t    pIdentityInfo; 	    //!< BOUND - identity information
	smSignInfo_t  pSigningInfo;   	    //!< Signing information
} gapAuthCompleteEvent_t;

/**
 * securityInfo and identityInfo are only used if secReqs.bondable == BOUND, which means that
 * the device is already bound and we should use the security information and keys.
 */
typedef struct __attribute__((__packed__))
{
  unsigned short int connectionHandle;      //!< Connection Handle from controller,
  smLinkSecurityReq_t  secReqs; //!< Pairing Control info
} gapAuthParams_t;

/**
 * GAP_SLAVE_REQUESTED_SECURITY_EVENT message format.  This message is sent to the app
 * when a Slave Security Request is received.
 */
typedef struct __attribute__((__packed__))
{
	EventHeader_t  hdr;           	//!< GAP_MSG_EVENT
	unsigned char opcode;                    //!< GAP type of command. Ref: @ref GAP_MSG_EVENT_DEFINES
	unsigned char event;
	bStatus_t status;
	unsigned char connectionHandle;      	//!< Connection Handle
	unsigned char deviceAddr[B_ADDR_LEN]; 	//!< address of device requesting security
	unsigned char authReq;                	//!< Authentication Requirements: Bit 2: MITM, Bits 0-1: bonding (0 - no bonding, 1 - bonding)

} gapSlaveSecurityReqEvent_t;

/**
 * GAP_BOND_COMPLETE_EVENT message format.  This message is sent to the
 * app when a bonding is complete.  This means that a key is loaded and the link is encrypted.
 */
typedef struct __attribute__((__packed__))
{
	EventHeader_t  hdr;           	//!< GAP_MSG_EVENT
	unsigned char opcode;                    //!< GAP type of command. Ref: @ref GAP_MSG_EVENT_DEFINES
	unsigned char event;
	unsigned short int connectionHandle; 	//!< connection Handle
} gapBondCompleteEvent_t;

/**
 * Pairing Request fields - the parsed fields of the SMP Pairing Request command.
 */
typedef struct __attribute__((__packed__))
{

    uint8 enable;           //!< Pairing Request enable/disable
	uint8 ioCap;         	//!< Pairing Request ioCap field
	uint8 oobDataFlag;   	//!< Pairing Request OOB Data Flag field
	uint8 authReq;       	//!< Pairing Request Auth Req field
	uint8 maxEncKeySize; 	//!< Pairing Request Maximum Encryption Key Size field
	uint8 keyDist;   		//!< Pairing Request Key Distribution field
} gapPairingReq_t;

/**
 * GAP_PAIRING_REQ_EVENT message format.<BR>
 * <BR>
 * This message is sent to the
 * app when an unexpected Pairing Request is received.  The application is
 * expected to setup for a Security Manager pairing/bonding.<BR>
 * <BR>
 * To setup an SM Pairing, the application should call GAP_Authenticate() with these "pairReq" fields.<BR>
 * <BR>
* NOTE: This message should only be sent to peripheral devices.
 */
typedef struct __attribute__((__packed__))
{
  EventHeader_t  hdr;           	//!< GAP_MSG_EVENT
  unsigned char opcode;                    //!< GAP type of command. Ref: @ref GAP_MSG_EVENT_DEFINES
  unsigned char event;
  bStatus_t status;
  unsigned short int connectionHandle; //!< connection Handle
  gapPairingReq_t pairReq; //!< The Pairing Request fields received.
} gapPairingReqEvent_t;

/**
 * GAP Advertisement/Scan Response Data Token - These data items are stored as low unsigned char first (OTA
 * format).  The data space for these items are passed in and maintained by
 * the calling application
 */
typedef struct __attribute__((__packed__))
{
  unsigned char adType;     //!< ADTYPE value: @ref GAP_ADTYPE_DEFINES
  unsigned char attrLen;    //!< Number of uint8s in the attribute data
  unsigned char *pAttrData; //!< pointer to Attribute data
} gapAdvDataToken_t;




typedef struct __attribute__((__packed__))
{
	unsigned char ProfileRole;
	unsigned char MaxScanResponses;
	unsigned char* pIRK;
	unsigned char* pSRK;
	unsigned char* pSignCounter;
} gapDeviceInit_t;


/**
 *
 * @see GAP_ConfigDeviceAddr
 */
typedef struct __attribute__((__packed__)) GapConfigDevAddr_
{
	unsigned char addrType;    //!< Address Type
	unsigned char devAddr[B_ADDR_LEN]; //!< Device Address
} gapConfigDevAddr_t;


typedef struct __attribute__((__packed__))
{
	unsigned char AdType;
	unsigned char DataLen;
	unsigned char* AdvertData;
}gapUpdateAdvertData_t;

typedef struct __attribute__((__packed__))
{
	unsigned char EventType;
	unsigned char InitiatorAddrType;
	unsigned char InitiatorAddr[B_ADDR_LEN];
	unsigned char ChannelMap;
	unsigned char FilterPolicy;
}gapMakeDiscoverable_t;


/**
 *
 * @see GAP_TerminateLinkReq
 */
typedef struct  __attribute__((__packed__))
{
	unsigned short int 	connHandle; //!< connection handle
	unsigned char 	reason;      //!< reason
} gapTerminateLink_t;


/**
 *
 * @see GAP_Authenticate
 */
typedef struct __attribute__((__packed__))
{
  gapAuthParams_t *pParams;  //!< Auth params for GAP
  gapPairingReq_t *pPairReq; //!< pairing request params
} gapAuthenticateParams_t;


/**
 *
 * @see GAP_TerminateAuth
 */
typedef struct __attribute__((__packed__))
{
  unsigned short int connHandle; //!< connHandle
  unsigned char reason;      //!< terminate reason
} gapTerminateAuth_t;

/**
 *
 * @see GAPBond
 */
typedef struct __attribute__((__packed__))
{
  unsigned short int connHandle;       //!< connection handle
  unsigned char authenticated;     //!< field for authentication
  smSecurityInfo_t *pParams; //!< security parameters
  unsigned char startEncryption;   //!< start Encryption
} gapBond_t;

/**
 *
 * @see GAP_Signable
 */
typedef struct  __attribute__((__packed__))
{
  uint16  connHandle;      //!< connection handle
  uint8   authenticated;    //!< authenticated
  smSigningInfo_t *pParams; //!< signing parameters
} gapSignable_t;

/**
 *
 * @see GAP_PasskeyUpdate
 */
typedef struct  __attribute__((__packed__))
{
  uint16   connHandle; //!< connection Handle
  uint8   *pPasskey;   //!< passKey
} gapPassKeyUpdateParam_t;

/**
 * @see GAP_GetParamValue()
 */
typedef struct  __attribute__((__packed__))
{
  unsigned char paramID;    //!< paramID
} gapGetParam_t;

/**
 * @see GAP_SetParamValue()
 */
typedef struct __attribute__((__packed__))
{
  unsigned char paramID;    //!< paramID
  unsigned short int paramValue; //!< paramValue
} gapSetParam_t;

/**
 *
 * @see GAP_ResolvePrivateAddr
 */
typedef struct __attribute__((__packed__))
{
  uint8   *pIRK;       //!< IRK
  uint8   *pAddr;      //!< private address
} gapResolvePrivateAddr_t;

/**
 *
 * @see GAP_SendSlaveSecurityRequest
 */
typedef struct __attribute__((__packed__))
{
  uint16  connHandle; //!< connection handle
  uint8   authReq;     //!< auth request
} gapSendSlaveSecReq_t;

/**
 * GAPBond Manager Set Parameters containing parameter Id, length and value
 */
typedef struct __attribute__((__packed__))
{
  unsigned short int paramId;         			//!< param Id
  unsigned char   paramDatalen;             	//!< param length
  unsigned char   *pValue;             		//!< param value
}gapBondMgrSetParams_t;


/**
 * GAPBond Manager Get Parameters containing parameter Id
 */
typedef struct __attribute__((__packed__))
{
  unsigned short int paramId;         //!< param Id
}gapBondMgrGetParams_t;


/**
 *
 * @see GAPBondMgr_PasscodeRsp
 */
typedef struct __attribute__((__packed__))
{
  unsigned short int connHandle; //!< connection handle
  unsigned char 	 status;      //!< status
  unsigned long passcode;   //!< passcode
} gapBondMgrPasscodeRsp_t;
#if 0
typedef struct
{
  bStatus_t status;
  unsigned short int OpCode;
  unsigned char DataLen;
  unsigned char* PayLoad;
}CommandStatus_t;
#endif

typedef struct __attribute__((__packed__))
{
	bStatus_t status;			//SUCCESS = 0x00
	unsigned char DevAddr[6];		//The device's public address (BD_ADDR)
	unsigned short int DataPktLen;		//HC_LE_Data_Packet_Length
	unsigned char NumDataPkts;		//HC_Total_Num_LE_Data_Packets
	unsigned char IRK[16];			//16 unsigned char Indentity Resolving Key (IRK).
	unsigned char CSRK[16];			//16 uint8s Connection Signature Resolving Key (CSRK).
}gapDeviceInitDone_t;


typedef struct __attribute__((__packed__))
{
	EventHeader_t  hdr;           	//!< GAP_MSG_EVENT
	unsigned char opcode;                    //!< GAP type of command. Ref: @ref GAP_MSG_EVENT_DEFINES
	unsigned char event;
	bStatus_t status;			//SUCCESS = 0x00
	unsigned char numDevs;			//The num of advertising Devices detected during the scan.
	unsigned char eventType;			//
	unsigned char addrType;			//
	unsigned char addr[B_ADDR_LEN];	//The device's public address (BD_ADDR)
}gapDeviceDiscoveryDone_t;

typedef struct __attribute__((__packed__))
{
	bStatus_t status;
	unsigned char AdType;
}gapAdvertDataUpdateDone_t;

typedef struct __attribute__((__packed__))
{
	EventHeader_t  hdr;           	//!< GAP_MSG_EVENT
	unsigned char opcode;                    //!< GAP type of command. Ref: @ref GAP_MSG_EVENT_DEFINES
	unsigned char event;
	bStatus_t status;
	unsigned short int Interval;		//Don't have in 'HCI Reference Guide'
}gapMakeDiscoverableDone_t;


typedef struct __attribute__((__packed__))
{
	EventHeader_t  hdr;           	//!< GAP_MSG_EVENT
	unsigned char opcode;                    //!< GAP type of command. Ref: @ref GAP_MSG_EVENT_DEFINES
	unsigned char event;
	bStatus_t Status;
}gapEndDiscoverableDone_t;


/**
 * GAP_AUTHENTICATION_COMPLETE_EVENT message format.  This message is sent to the app
 * when the authentication request is complete.
 */
typedef struct __attribute__((__packed__))
{
	bStatus_t status;
	unsigned short int connectionHandle;         	//!< Connection Handle from controller used to ref the device
	unsigned char authState;                 	//!< TRUE if the pairing was authenticated (MITM)
	smSecurityInfo_t *pSecurityInfo; 	//!< BOUND - security information from this device
	smSigningInfo_t *pSigningInfo;   	//!< Signing information
	smSecurityInfo_t *pDevSecInfo;   	//!< BOUND - security information from connected device
	smIdentityInfo_t *pIdentityInfo; 	//!< BOUND - identity information
}gapAuthenticationComplete_t;



typedef struct __attribute__((__packed__))
{
	bStatus_t status;
	unsigned char eventType;
	unsigned char addrType;
	unsigned char addr[B_ADDR_LEN];
	unsigned char rssi;
	unsigned char dataLen;
	unsigned char *pDataField;

}gapDeviceInfo_t;




/*********************************************************************
 * Use this function to initialize GAP Device parameters
 *
 * Public function defined in gap.h.
 */
bStatus_t GAP_DeviceInit(unsigned char taskID, unsigned char profileRole,  unsigned char maxScanResponses, unsigned char *pIRK, unsigned char *pSRK,  unsigned char *pSignCounter);

/*********************************************************************
 * Setup the device's address type.
 *
 * Public function defined in gap.h
 */
bStatus_t GAP_ConfigDeviceAddr(unsigned char addrType, unsigned char *pStaticAddr);


/*********************************************************************
 * Establish a link to a slave device.
 *
 * Public function defined in gap.h.
 */
bStatus_t GAP_EstablishLinkReq( gapEstLinkReq_t *pParams );

/*********************************************************************
 * Update the link parameters to a slave device.
 *
 * Public function defined in gap.h.
 */
bStatus_t GAP_UpdateLinkParamReq( gapUpdateLinkParamReq_t *pParams );

/*********************************************************************
 * Terminate a link connection.
 *
 * Public function defined in gap.h.
 */
bStatus_t GAP_TerminateLinkReq(unsigned char taskID, unsigned short int connHandle, unsigned char reason);

/*********************************************************************
 * Start a device discovery scan.
 *
 * Public function defined in gap.h.
 */
bStatus_t GAP_DeviceDiscoveryRequest( gapDevDiscReq_t *pParams );

/*********************************************************************
 * Cancel an existing device discovery request.
 *
 * Public function defined in gap.h.
 */
bStatus_t GAP_DeviceDiscoveryCancel(unsigned char taskID);

/*********************************************************************
 * GAP Authenticate
 *
 * Public function defined in gap.h.
 */
bStatus_t GAP_Authenticate(gapAuthParams_t *pParams, gapPairingReq_t *pPairReq);

/*********************************************************************
 * Terminate an authentication/pairing process.
 *
 * Public function defined in gap.h.
 *
 * @return  SUCCESS or FAILURE
 */
bStatus_t GAP_TerminateAuth(unsigned short int connectionHandle, unsigned char reason);


/*********************************************************************
* @brief  GAP_Bond API
*
* Public function defined in gap.h.
*/
bStatus_t GAP_Bond(unsigned short int connHandle, unsigned char authenticated,  smSecurityInfo_t *pParams, unsigned char startEncryption);


/*********************************************************************
 * Set up the connection to accept signed data.
 *
 * Public function defined in gap.h.
 */
bStatus_t GAP_Signable(uint16 connectionHandle, uint8 authenticated, smSigningInfo_t *pParams);

/*********************************************************************
 * GAP Update the passkey.
 *
 * Public function defined in gap.h.
 *
 * @return  SUCCESS or FAILURE
 */
bStatus_t GAP_PasskeyUpdate(uint8 *pPasskey, uint16 connectionHandle);

/*********************************************************************
 * Set a GAP Parameter value.  Use this function to change the default
 * GAP parameter values.
 *
 * Public function defined in gap.h.
 */
bStatus_t GAP_SetParamValue(gapParamIDs_t paramID, uint16 paramValue);


/*********************************************************************
 * Get a GAP Parameter value.  Use this function to get GAP parameter values.
 *
 * Public function defined in gap.h.
 */
unsigned short int GAP_GetParamValue(gapParamIDs_t paramID);

/*********************************************************************
 * Resolve private address
 *
 * Public function defined in gap.h.
 */
bStatus_t GAP_ResolvePrivateAddr(uint8 *pIRK, uint8 *pAddr);

/*********************************************************************
 * Send Slave Security Request
 *
 * Public function defined in gap.h.
 */
bStatus_t GAP_SendSlaveSecurityRequest(uint16 connHandle, uint8 authReq);

/*********************************************************************
 * @brief   Set a GAP Bond Manager parameter.
 *
 * Public function defined in gapbondmgr.h.
 */
bStatus_t GAPBondMgr_SetParameter(unsigned short int param, unsigned char len, void *pValue);


/*********************************************************************
 * @brief   Get a GAP Bond Manager parameter.
 *
 * Public function defined in gapbondmgr.h.
 */
bStatus_t GAPBondMgr_GetParameter(unsigned short int param, void *pValue);

/*
 * Events
 */
bStatus_t GAP_deviceInitDone(unsigned char* packet);
bStatus_t GAP_advertDataUpdateDone(unsigned char* packet);
bStatus_t GAP_makeDiscoverableDone(unsigned char* packet);
bStatus_t GAP_linkEstablished(unsigned char* packet);
bStatus_t GAP_linkTerminated(unsigned char* packet);

#endif /* GAP_H_ */
