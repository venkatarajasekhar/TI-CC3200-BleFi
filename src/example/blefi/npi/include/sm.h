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


#ifndef SM_H
#define SM_H

#ifdef __cplusplus
extern "C"
{
#endif

/*-------------------------------------------------------------------
 * INCLUDES
 */
#include "hci.h"



/*-------------------------------------------------------------------
 * MACROS
 */
#ifndef B_ADDR_LEN
#define B_ADDR_LEN                                6
#endif
/*-------------------------------------------------------------------
 * CONSTANTS
 */
/** @defgroup SM_IO_CAP_DEFINES SM I/O Capabilities
 * @{
 */
#define DISPLAY_ONLY              0x00  //!< Display Only Device
#define DISPLAY_YES_NO            0x01  //!< Display and Yes and No Capable
#define KEYBOARD_ONLY             0x02  //!< Keyboard Only
#define NO_INPUT_NO_OUTPUT        0x03  //!< No Display or Input Device
#define KEYBOARD_DISPLAY          0x04  //!< Both Keyboard and Display Capable
/** @} End SM_IO_CAP_DEFINES */

#define SM_AUTH_MITM_MASK(a)    (((a) & 0x04) >> 2)

/** @defgroup SM_PASSKEY_TYPE_DEFINES SM Passkey Types (Bit Masks)
 * @{
 */
#define SM_PASSKEY_TYPE_INPUT   0x01    //!< Input the passkey
#define SM_PASSKEY_TYPE_DISPLAY 0x02    //!< Display the passkey
/** @} End SM_PASSKEY_TYPE_DEFINES */

/** @defgroup SM_BONDING_FLAGS_DEFINES SM AuthReq Bonding Flags
 * Bonding flags 0x02 and 0x03 are reserved.
 * @{
 */
#define SM_AUTH_REQ_NO_BONDING    0x00  //!< No bonding
#define SM_AUTH_REQ_BONDING       0x01  //!< Bonding
/** @} End SM_BONDING_FLAGS_DEFINES */

#define PASSKEY_LEN     6   //! Passkey Character Length (ASCII Characters)

#define SM_AUTH_STATE_AUTHENTICATED       0x04  //! Authenticate requested
#define SM_AUTH_STATE_BONDING             0x01  //! Bonding requested

//#define B_RANDOM_NUM_SIZE                               32
#define B_RANDOM_NUM_SIZE                               8
/*-------------------------------------------------------------------
 * General TYPEDEFS
 */

/**
 * SM_NEW_RAND_KEY_EVENT message format.  This message is sent to the
 * requesting task.
 */
typedef struct __attribute__((__packed__))
{
	EventHeader_t  hdr;      	//!< SM_NEW_RAND_KEY_EVENT and status
	uint8 newKey[KEYLEN];       //!< New key value - if status is SUCCESS
} smNewRandKeyEvent_t;

/**
 * Key Distribution field  - True or False fields
 */
typedef struct __attribute__((__packed__))
{
  unsigned int sEncKey:1;    //!< Set to distribute slave encryption key
  unsigned int sIdKey:1;     //!< Set to distribute slave identity key
  unsigned int sSign:1;      //!< Set to distribute slave signing key
  unsigned int sLinkKey:1;   //!<Set to distribute slave Link key
  unsigned int mEncKey:1;    //!< Set to distribute master encryption key
  unsigned int mIdKey:1;     //!< Set to distribute master identity key
  unsigned int mSign:1;      //!< Set to distribute master signing key
  unsigned int mLinkKey:1;   //!< Reserved - not to be used
} keyDist_t;

/**
 * Link Security Requirements
 */
typedef struct __attribute__((__packed__))
{
  uint8 ioCaps;               //!< I/O Capabilities (ie.
  uint8 oobAvailable;         //!< True if Out-of-band key available
  uint8 oob[KEYLEN];          //!< Out-Of-Bounds key
  uint8 authReq;              //!< Authentication Requirements
  keyDist_t keyDist;          //!< Key Distribution mask
  uint8 maxEncKeySize;        //!< Maximum Encryption Key size (7-16 uint8s)
} smLinkSecurityReq_t;

/**
 * Link Security Information
 */
typedef struct __attribute__((__packed__))
{
  uint8 keySize;                  //!< LTK Key Size (7-16 uint8s)
  uint8 ltk[KEYLEN];              //!< Long Term Key (LTK)
  uint16 div;                     //!< LTK Diversifier
  //uint8 rand[8];  //!< LTK random number
  uint8 rand[B_RANDOM_NUM_SIZE];  //!< LTK random number
} smSecurityInfo_t;

/**
 * Link Security Information
 */
typedef struct __attribute__((__packed__))
{
	uint8 secInfoEnable;
	smSecurityInfo_t secInfo;
} smSecInfo_t;

/**
 * Link Identity Information
 */
typedef struct __attribute__((__packed__))
{
  uint8 irk[KEYLEN];          //!< Identity Resolving Key (IRK)
  uint8 bd_addr[B_ADDR_LEN];  //!< The advertiser may set this to zeroes to not disclose its BD_ADDR (public address).
} smIdentityInfo_t;

/**
 * Link Identity Information
 */
typedef struct __attribute__((__packed__))
{
  uint8 idInfoEnable;
  smIdentityInfo_t idInfo;
} smIdInfo_t;

/**
 * Signing Information
 */
typedef struct __attribute__((__packed__))
{
  uint8  srk[KEYLEN]; //!< Signature Resolving Key (CSRK)
  //uint32 signCounter; //!< Sign Counter
  uint16 signCounter; //!< Sign Counter
} smSigningInfo_t;

/**
 * Signing Information
 */
typedef struct __attribute__((__packed__))
{
  uint8  signInfoEnable;
  smSigningInfo_t signInfo; //!< Sign Counter
} smSignInfo_t;

/**
 * Pairing Request & Response - authReq field
 */
typedef struct __attribute__((__packed__))
{
  unsigned int bonding:2;    //!< Bonding flags
  unsigned int mitm:1;       //!< Man-In-The-Middle (MITM)
  unsigned int reserved:5;   //!< Reserved - don't use
} authReq_t;



#ifdef __cplusplus
}
#endif

#endif /* SM_H */
