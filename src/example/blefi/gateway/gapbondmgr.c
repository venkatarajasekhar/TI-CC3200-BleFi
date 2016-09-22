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
#include "string.h"
#include "gap.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "hci.h"
#include "gapbondmgr.h"
#include "ble_central.h"
#include "gateway_api.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */


/**
 * GAP Bond Manager NV layout
 *
 * The NV definitions:
 *     BLE_NVID_GAP_BOND_START - starting NV ID
 *     GAP_BONDINGS_MAX - Maximum number of bonding allowed (10 is max for number of NV IDs allocated in bcomdef.h).
 *
 * A single bonding entry consists of 6 components (NV items):
 *     Bond Record - defined as gapBondRec_t and uses GAP_BOND_REC_ID_OFFSET for an NV ID
 *     local LTK Info - defined as gapBondLTK_t and uses GAP_BOND_LOCAL_LTK_OFFSET for an NV ID
 *     device LTK Info - defined as gapBondLTK_t and uses GAP_BOND_DEV_LTK_OFFSET for an NV ID
 *     device IRK - defined as "uint8 devIRK[KEYLEN]" and uses GAP_BOND_DEV_IRK_OFFSET for an NV ID
 *     device CSRK - defined as "uint8 devCSRK[KEYLEN]" and uses GAP_BOND_DEV_CSRK_OFFSET for an NV ID
 *     device Sign Counter - defined as a uint32 and uses GAP_BOND_DEV_SIGN_COUNTER_OFFSET for an NV ID
 *
 * When the device is initialized for the first time, all (GAP_BONDINGS_MAX) NV items are created and
 * initialized to all 0xFF's. A bonding record of all 0xFF's indicates that the bonding record is empty
 * and free to use.
 *
 * The calculation for each bonding records NV IDs:
 *    mainRecordNvID = ((bondIdx * GAP_BOND_REC_IDS) + BLE_NVID_GAP_BOND_START)
 *    localLTKNvID = (((bondIdx * GAP_BOND_REC_IDS) + GAP_BOND_LOCAL_LTK_OFFSET) + BLE_NVID_GAP_BOND_START)
 *
 */

// Key Size Limits
#define MIN_ENC_KEYSIZE                     7  //!< Minimum number of bytes for the encryption key
#define MAX_ENC_KEYSIZE                     16 //!< Maximum number of bytes for the encryption key

/*********************************************************************
 * TYPEDEFS
 */


/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */


/*********************************************************************
 * EXTERNAL FUNCTIONS
 */
extern void HandleError(unsigned char *, unsigned int, unsigned char error);
extern void UpdateBlefiCfg(void);
/*********************************************************************
 * LOCAL VARIABLES
 */

// GAPBonding Parameters
static uint8 gapBond_PairingMode = GAPBOND_PAIRING_MODE_INITIATE;
static uint16 gapBond_InitiateWait = 1000;  // Default to 1 second
static uint8 gapBond_MITM = FALSE;
static uint8 gapBond_IOCap = GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT;
static uint8 gapBond_OOBDataFlag = FALSE;
static uint8 gapBond_OOBData[KEYLEN] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static uint8 gapBond_Bonding = TRUE;
static uint8 gapBond_AutoFail = FALSE;
static uint8 gapBond_AutoFailReason = SMP_PAIRING_FAILED_NOT_SUPPORTED;
static uint8 gapBond_KeyDistList =
(
  GAPBOND_KEYDIST_SENCKEY     // sEncKey enabled, to send the encryption key
   | GAPBOND_KEYDIST_SIDKEY   // sIdKey enabled, to send the IRK, and BD_ADDR
   | GAPBOND_KEYDIST_SSIGN    // sSign enabled, to send the CSRK
   | GAPBOND_KEYDIST_MENCKEY  // mEncKey enabled, to get the master's encryption key
   | GAPBOND_KEYDIST_MIDKEY   // mIdKey enabled, to get the master's IRK and BD_ADDR
   | GAPBOND_KEYDIST_MSIGN    // mSign enabled, to get the master's CSRK
);
static uint32 gapBond_Passcode = 0;
static uint8  gapBond_KeySize = MAX_ENC_KEYSIZE;

volatile static uint8  gapBond_BondFailOption = GAPBOND_FAIL_TERMINATE_LINK;

#if AUTH_ENABLE
volatile static const gapBondCBs_t *pGapBondCB = NULL;
#endif

static uint8 autoSyncWhiteList = FALSE;

extern blefiCfg_t blefiCfgRec;


/*********************************************************************
 * LOCAL FUNCTIONS
 */

static uint8 isBufset( uint8 *pBuf , uint8 value, uint8 len );
static linkDBItem_t *linkDB_Find( uint16 connectionHandle );
static uint8 gapBondMgrGetStateFlags( uint8 idx );
static bStatus_t gapBondMgrGetPublicAddr( uint8 idx, uint8 *pAddr );
static uint8 gapBondMgrFindReconnectAddr( uint8 *pReconnectAddr );
static uint8 gapBondMgrFindAddr( uint8 *pDevAddr );
static uint8 gapBondMgrResolvePrivateAddr( uint8 *pAddr );
//static void gapBondMgrReadBonds( void );
uint8 gapBondMgrFindEmpty( void );
static uint8 gapBondMgrBondTotal( void );
bStatus_t gapBondMgrEraseAllBondings( void );
static void gapBondMgrBondReq( uint16 connHandle, uint8 idx, uint8 stateFlags,
                               uint8 role, uint8 startEncryption );
static void gapBondMgrAuthenticate( uint16 connHandle, uint8 addrType,
                                    gapPairingReq_t *pPairReq );


/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */



void gapBondMgrFlashBondRecUpdate()
{
	UpdateBlefiCfg();
}


static uint8 isBufset( uint8 *pBuf , uint8 value, uint8 len )
{
	bStatus_t ret = SUCCESS;  // return value
	uint8 i = 0;
	for(i=0;i<len;i++)
	{
		if( *(pBuf+i) != value)
		{
			ret = FAILURE;
			break;
		}
	}

	return ret;
}

/*
 * linkDB_Find - Find link database item (link information)
 *
 *    returns a pointer to the link item, NULL if not found
 */
static linkDBItem_t *linkDB_Find( uint16 connectionHandle )
{
	return NULL;
}







/*********************************************************************
 * @brief   Set a GAP Bond Manager parameter.
 *
 * Public function defined in gapbondmgr.h.
 */
bStatus_t GAPBondMgr_SetParam( uint16 param, uint8 len, void *pValue )
{
  bStatus_t ret = SUCCESS;  // return value

  switch ( param )
  {
    case GAPBOND_PAIRING_MODE:
      if ( (len == sizeof ( uint8 )) && (*((uint8*)pValue) <= GAPBOND_PAIRING_MODE_INITIATE) )
      {
        gapBond_PairingMode = *((uint8*)pValue);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPBOND_INITIATE_WAIT:
      if ( len == sizeof ( uint16 ) )
      {
        gapBond_InitiateWait = *((uint16*)pValue);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPBOND_MITM_PROTECTION:
      if ( (len == sizeof ( uint8 )) && (*((uint8*)pValue) <= TRUE) )
      {
        gapBond_MITM = *((uint8*)pValue);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPBOND_IO_CAPABILITIES:
      if ( (len == sizeof ( uint8 )) && (*((uint8*)pValue) <= GAPBOND_IO_CAP_KEYBOARD_DISPLAY) )
      {
        gapBond_IOCap = *((uint8*)pValue);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPBOND_OOB_ENABLED:
      if ( (len == sizeof ( uint8 )) && (*((uint8*)pValue) <= TRUE) )
      {
        gapBond_OOBDataFlag = *((uint8*)pValue);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPBOND_OOB_DATA:
      if ( len == KEYLEN )
      {
        memcpy( gapBond_OOBData, pValue, KEYLEN ) ;
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPBOND_BONDING_ENABLED:
      if ( (len == sizeof ( uint8 )) && (*((uint8*)pValue) <= TRUE) )
      {
        gapBond_Bonding = *((uint8*)pValue);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPBOND_KEY_DIST_LIST:
      if ( len == sizeof ( uint8 ) )
      {
        gapBond_KeyDistList = *((uint8*)pValue);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPBOND_DEFAULT_PASSCODE:
      if ( (len == sizeof ( uint32 ))
          && (*((uint32*)pValue) <= GAP_PASSCODE_MAX) )
      {
        gapBond_Passcode = *((uint32*)pValue);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPBOND_ERASE_ALLBONDS:
       break;

    case GAPBOND_ERASE_SINGLEBOND:

      break;
      
    case GAPBOND_AUTO_FAIL_PAIRING:
      if ( (len == sizeof ( uint8 )) && (*((uint8*)pValue) <= TRUE) )
      {
        gapBond_AutoFail = *((uint8*)pValue);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPBOND_AUTO_FAIL_REASON:
      if ( (len == sizeof ( uint8 )) && (*((uint8*)pValue) <= SMP_PAIRING_FAILED_REPEATED_ATTEMPTS) )
      {
        gapBond_AutoFailReason = *((uint8*)pValue);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPBOND_KEYSIZE:
      if ( (len == sizeof ( uint8 ))
          && ((*((uint8*)pValue) >= MIN_ENC_KEYSIZE) && (*((uint8*)pValue) <= MAX_ENC_KEYSIZE)) )
      {
        gapBond_KeySize = *((uint8*)pValue);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPBOND_AUTO_SYNC_WL:
      if ( len == sizeof( uint8 ) )
      {
        uint8 oldVal = autoSyncWhiteList;

        autoSyncWhiteList = *((uint8 *)pValue);

        // only call if parameter changes from FALSE to TRUE
        if ( ( oldVal == FALSE ) && ( autoSyncWhiteList == TRUE ) )
        {

        // make sure bond is updated from NV
       	//gapBondMgrBondRecInit();

        }
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GAPBOND_BOND_FAIL_ACTION:
      if ( (len == sizeof ( uint8 )) && (*((uint8*)pValue) <= GAPBOND_FAIL_TERMINATE_ERASE_BONDS) )
      {
        gapBond_BondFailOption = *((uint8*)pValue);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;


    default:
      // The param value isn't part of this profile, try the GAP.
      if ( (param < TGAP_PARAMID_MAX) && (len == sizeof ( uint16 )) )
      {
        ret = GAP_SetParamValue( param, *((uint16*)pValue) );
      }
      else
      {
        ret = INVALIDPARAMETER;
      }
      break;
  }

  return ( ret );
}

/*********************************************************************
 * @brief   Get a GAP Bond Manager parameter.
 *
 * Public function defined in gapbondmgr.h.
 */
bStatus_t GAPBondMgr_GetParam( uint16 param, void *pValue )
{
  bStatus_t ret = SUCCESS;  // return value

  switch ( param )
  {
    case GAPBOND_PAIRING_MODE:
      *((uint8*)pValue) = gapBond_PairingMode;
      break;

    case GAPBOND_INITIATE_WAIT:
      *((uint16*)pValue) = gapBond_InitiateWait;
      break;

    case GAPBOND_MITM_PROTECTION:
      *((uint8*)pValue) = gapBond_MITM;
      break;

    case GAPBOND_IO_CAPABILITIES:
      *((uint8*)pValue) = gapBond_IOCap;
      break;

    case GAPBOND_OOB_ENABLED:
      *((uint8*)pValue) = gapBond_OOBDataFlag;
      break;

    case GAPBOND_OOB_DATA:
    	memcpy( pValue, gapBond_OOBData, KEYLEN ) ;
      break;

    case GAPBOND_BONDING_ENABLED:
      *((uint8*)pValue) = gapBond_Bonding;
      break;

    case GAPBOND_KEY_DIST_LIST:
      *((uint8*)pValue) = gapBond_KeyDistList;
      break;

    case GAPBOND_DEFAULT_PASSCODE:
      *((uint32*)pValue) = gapBond_Passcode;
      break;

    case GAPBOND_AUTO_FAIL_PAIRING:
      *((uint8*)pValue) = gapBond_AutoFail;
      break;

    case GAPBOND_AUTO_FAIL_REASON:
      *((uint8*)pValue) = gapBond_AutoFailReason;
      break;

    case GAPBOND_KEYSIZE:
      *((uint8*)pValue) = gapBond_KeySize;
      break;

    case GAPBOND_AUTO_SYNC_WL:
      *((uint8*)pValue) = autoSyncWhiteList;
      break;

    case GAPBOND_BOND_COUNT:
      *((uint8*)pValue) = gapBondMgrBondTotal();
      break;

    default:
      // The param value isn't part of this profile, try the GAP.
      if ( param < TGAP_PARAMID_MAX )
      {
        *((uint16*)pValue) = GAP_GetParamValue( param );
      }
      else
      {
        ret = INVALIDPARAMETER;
      }
      break;
  }

  return ( ret );
}


/*********************************************************************
 * @brief   Resolve an address from bonding information.
 *
 * Public function defined in gapbondmgr.h.
 */
uint8 GAPBondMgr_ResolveAddr( uint8 addrType, uint8 *pDevAddr, uint8 *pResolvedAddr )
{
  uint8 idx = GAP_BONDINGS_MAX;

  switch ( addrType )
  {
    case ADDRTYPE_PUBLIC:
    case ADDRTYPE_STATIC:
      idx = gapBondMgrFindAddr( pDevAddr );
      if ( (idx < GAP_BONDINGS_MAX) && (pResolvedAddr != NULL) )
      {
        memcpy( pResolvedAddr, pDevAddr, B_ADDR_LEN );
      }
      break;

    case ADDRTYPE_PRIVATE_NONRESOLVE:
      // This could be a reconnection address
      idx = gapBondMgrFindReconnectAddr( pDevAddr );
      if ( (idx < GAP_BONDINGS_MAX) && (pResolvedAddr) )
      {
        gapBondMgrGetPublicAddr( idx, pResolvedAddr );
      }
      break;

    case ADDRTYPE_PRIVATE_RESOLVE:
      // Master's don't use Private Resolvable addresses but just in case
      idx = gapBondMgrResolvePrivateAddr( pDevAddr );
      if ( (idx < GAP_BONDINGS_MAX) && (pResolvedAddr) )
      {
        gapBondMgrGetPublicAddr( idx, pResolvedAddr );
      }
      break;

    default:
      break;
  }

  return ( idx );
}

/*********************************************************************
 * @brief   Notify the Bond Manager that a connection has been made.
 *
 * Public function defined in gapbondmgr.h.
 */
bStatus_t GAPBondMgr_LinkEst( uint8 addrType, uint8 *pDevAddr, uint16 connHandle, uint8 role )
{
  uint8 idx;                          // NV Index
  uint8 publicAddr[B_ADDR_LEN] = {0, 0, 0, 0, 0, 0};      // Place to put the public address
  gapPairingReq_t pairReq;
	BondRec_t * bonds;
	bonds = (BondRec_t *)blefiCfgRec.bonds;

  // fixing kwk warning
  pairReq.authReq = 0;

  idx = GAPBondMgr_ResolveAddr( addrType, pDevAddr, publicAddr );
  if ( idx < GAP_BONDINGS_MAX )
  {
    uint8 stateFlags = gapBondMgrGetStateFlags( idx );
    smSigningInfo_t signingInfo;

	//Save the Connection Handle to a Bond Record
	bonds[idx].connHndl = connHandle;
	gapBondMgrFlashBondRecUpdate();

    // On central and initiaiting security, load key to initiate encyption
    gapBondMgrBondReq( connHandle, idx, stateFlags, role,
                       ((gapBond_PairingMode == GAPBOND_PAIRING_MODE_INITIATE ) ? TRUE : FALSE) );

    // Load the Signing Key
    memset( &signingInfo, 0, sizeof ( smSigningInfo_t ) );
    memcpy(&(signingInfo.srk[0]), &(bonds[idx].devCSRK[0]), KEYLEN );
	if ( isBufset( signingInfo.srk, 0xFF, KEYLEN ) == FAILURE )
	{
		// Load the signing information for this connection
		signingInfo.signCounter = (bonds[idx].signCounter);
		GAP_Signable( connHandle,
			((stateFlags & GAP_BONDED_STATE_AUTHENTICATED) ? TRUE : FALSE), &signingInfo );
	}

  }
  else if ( (role == GAP_PROFILE_CENTRAL) &&
           (gapBond_PairingMode == GAPBOND_PAIRING_MODE_INITIATE) )
  {

	//Find Empty Bond Entry and allocate them
	idx = gapBondMgrFindEmpty();

	if(idx >= GAP_BONDINGS_MAX)
	{
		ASSERT_ON_ERROR(0);
		HandleError(__FILE__,__LINE__,1); // Fatal error
		return 1;
	}


	bonds[idx].connHndl = connHandle;
	gapBondMgrFlashBondRecUpdate();

    // If Central and initiating and not bonded, then initiate pairing
    gapBondMgrAuthenticate( connHandle, addrType, &pairReq );

  }

  return ( SUCCESS );
}

/*********************************************************************
 * @brief   Notify the Bond Manager that a connection has been terminated.
 *
 * Public function defined in gapbondmgr.h.
 */
void GAPBondMgr_LinkTerm(uint16 connHandle)
{
  (void)connHandle;
  

}


/*********************************************************************
 * @brief   Notify the Bond Manager that a Slave Security Request is received.
 *
 * Public function defined in gapbondmgr.h.
 */
void GAPBondMgr_SlaveReqSecurity(uint16 connHandle)
{
  uint8 idx;
  uint8 publicAddr[B_ADDR_LEN] = {0, 0, 0, 0, 0, 0};
  linkDBItem_t *pLink = linkDB_Find( connHandle );  //Arun
  gapPairingReq_t pairReq;

  // If link found and not already initiating security
  if (pLink != NULL && gapBond_PairingMode != GAPBOND_PAIRING_MODE_INITIATE)
  {
    // If already bonded initiate encryption
    idx = GAPBondMgr_ResolveAddr( pLink->addrType, pLink->addr, publicAddr );
    if ( idx < GAP_BONDINGS_MAX )
    {
      gapBondMgrBondReq( connHandle, idx, gapBondMgrGetStateFlags( idx ),
                         GAP_PROFILE_CENTRAL, TRUE );
    }
    // Else if no pairing allowed
    else if ( gapBond_PairingMode == GAPBOND_PAIRING_MODE_NO_PAIRING )
    {
      // Send error
      GAP_TerminateAuth( connHandle, SMP_PAIRING_FAILED_NOT_SUPPORTED );
    }
    // Else if waiting for request
    else if (gapBond_PairingMode == GAPBOND_PAIRING_MODE_WAIT_FOR_REQ)
    {
      // Initiate pairing
      gapBondMgrAuthenticate( connHandle, pLink->addrType, &pairReq );
    }
  }
}




/*********************************************************************
 * @brief   Register callback functions with the bond manager.
 *
 * Public function defined in gapbondmgr.h.
 */
#ifdef AUTH_ENABLE
void GAPBondMgr_Register( gapBondCBs_t *pCB )
{

  pGapBondCB = pCB;

}
#endif

/*********************************************************************
 * @brief   Respond to a passcode request.
 *
 * Public function defined in gapbondmgr.h.
 */
bStatus_t GAPBondMgr_PasscodeRsp( uint16 connectionHandle, uint8 status, uint32 passcode )
{
  bStatus_t ret = SUCCESS;

  return ret;
}


/*********************************************************************
 * LOCAL FUNCTION PROTOTYPES
 */


/*********************************************************************
 * @fn      gapBondMgrGetStateFlags
 *
 * @brief   Gets the state flags field of a bond record in NV
 *
 * @param   idx
 *
 * @return  stateFlags field
 */
static uint8 gapBondMgrGetStateFlags( uint8 idx )
{
	BondRec_t * bonds;
	bonds = (BondRec_t *)blefiCfgRec.bonds;
	// Check parameters
	if (idx >= GAP_BONDINGS_MAX)
	{
		return ( INVALIDPARAMETER );
	}
	else
	{
		return (bonds[idx].bond_rec.stateFlags );
	}

}

/*********************************************************************
 * @fn      gapBondMgrGetPublicAddr
 *
 * @brief   Copy the public Address from a bonding record
 *
 * @param   idx - Bond record index
 * @param   pAddr - a place to put the public address from NV
 *
 * @return  SUCCESS if successful.
 *          Otherwise failure.
 */
static bStatus_t gapBondMgrGetPublicAddr( uint8 idx, uint8 *pAddr )
{
	BondRec_t * bonds;
	bonds = (BondRec_t *)blefiCfgRec.bonds;
  bStatus_t stat = FAILURE;         // return value

  // Check parameters
  if ( (idx >= GAP_BONDINGS_MAX) || (pAddr == NULL) )
  {
    return ( INVALIDPARAMETER );
  }

  // Update the Bind table from Flash before using the values. TO BE DONE

  memcpy( pAddr, &(bonds[idx].bond_rec.publicAddr[0]), B_ADDR_LEN );

  stat = SUCCESS;

  return ( stat );
}

/*********************************************************************
 * @fn      gapBondMgrFindReconnectAddr
 *
 * @brief   Look through the bonding entries to find a
 *          reconnection address.
 *
 * @param   pReconnectAddr - device address to look for
 *
 * @return  index to found bonding (0 - (GAP_BONDINGS_MAX-1),
 *          GAP_BONDINGS_MAX if no empty entries
 */
static uint8 gapBondMgrFindReconnectAddr( uint8 *pReconnectAddr )
{
  // Item doesn't exist, so create all the items
  uint8 idx;
	BondRec_t * bonds;
	bonds = (BondRec_t *)blefiCfgRec.bonds;
  for ( idx = 0; idx < GAP_BONDINGS_MAX; idx++ )
  {
    // compare reconnection address
    if ( memcmp(bonds[idx].bond_rec.reconnectAddr, pReconnectAddr, B_ADDR_LEN ) == SUCCESS)
    {
      return ( idx ); // Found it
    }
  }

  return ( GAP_BONDINGS_MAX );
}

/*********************************************************************
 * @fn      gapBondMgrFindAddr
 *
 * @brief   Look through the bonding entries to find an address.
 *
 * @param   pDevAddr - device address to look for
 *
 * @return  index to empty bonding (0 - (GAP_BONDINGS_MAX-1),
 *          GAP_BONDINGS_MAX if no empty entries
 */
static uint8 gapBondMgrFindAddr( uint8 *pDevAddr )
{
  // Item doesn't exist, so create all the items
  uint8 idx;
	BondRec_t * bonds;
	bonds = (BondRec_t *)blefiCfgRec.bonds;
  for ( idx = 0; idx < GAP_BONDINGS_MAX; idx++ )
  {
    //Check the Main Bond Record and compare public address
    if ( memcmp( bonds[idx].bond_rec.publicAddr, pDevAddr, B_ADDR_LEN ) == SUCCESS )
    {
      return ( idx ); // Found it
    }
  }

  return ( GAP_BONDINGS_MAX );
}

/*********************************************************************
 * @fn      gapBondMgrResolvePrivateAddr
 *
 * @brief   Look through the NV bonding entries to resolve a private
 *          address.
 *
 * @param   pDevAddr - device address to look for
 *
 * @return  index to found bonding (0 - (GAP_BONDINGS_MAX-1),
 *          GAP_BONDINGS_MAX if no entry found
 */
static uint8 gapBondMgrResolvePrivateAddr( uint8 *pDevAddr )
{
	uint8 idx;
	BondRec_t * bonds;
	bonds = (BondRec_t *)blefiCfgRec.bonds;
	for ( idx = 0; idx < GAP_BONDINGS_MAX; idx++ )
	{
		uint8 IRK[KEYLEN];

		// Read in NV IRK Record and compare resovable address
		memcpy( &(IRK[0]), &(bonds[idx].devIRK[0]), KEYLEN );

		if ( ( isBufset( IRK, 0xFF, KEYLEN ) == FAILURE ) && ( GAP_ResolvePrivateAddr( IRK, pDevAddr ) == SUCCESS ) )
		{
			return ( idx ); // Found it
		}

	}

	return ( GAP_BONDINGS_MAX );
}


/*********************************************************************
 * @fn      gapBondMgrFindEmpty
 *
 * @brief   Look through the bonding NV entries to find an empty.
 *
 * @param   none
 *
 * @return  index to empty bonding (0 - (GAP_BONDINGS_MAX-1),
 *          GAP_BONDINGS_MAX if no empty entries
 */
uint8 gapBondMgrFindEmpty( void )
{
	// Item doesn't exist, so create all the items
	uint8 idx;
	BondRec_t * bonds;
	bonds = (BondRec_t *)blefiCfgRec.bonds;
	for ( idx = 0; idx < GAP_BONDINGS_MAX; idx++ )
	{
		// Look for public address of all 0x00's
		if ( isBufset( bonds[idx].bond_rec.publicAddr, 0x00, B_ADDR_LEN ) == SUCCESS)
		{
		  return ( idx ); // Found one
		}
	}
	if(idx==GAP_BONDINGS_MAX)
	{
		memcpy(bonds,&(bonds[1]),(sizeof(BondRec_t)*(GAP_BONDINGS_MAX-1)));
	}
	return ( GAP_BONDINGS_MAX-1 );
}

/*********************************************************************
 * @fn      gapBondMgrFindEmpty
 *
 * @brief   Look through the bonding NV entries to find the bond entry corresponding to the connection Handle.
 *
 * @param   uint16 connectionHandle
 *
 * @return  index to  bonding entry(0 - (GAP_BONDINGS_MAX-1),
 *          GAP_BONDINGS_MAX if not found
 */
uint8 gapBondMgrFindBondEntry( unsigned char * deviceId )
{
  // Item doesn't exist, so create all the items
  uint8 idx;
	BondRec_t * bonds;
	bonds = (BondRec_t *)blefiCfgRec.bonds;
  for ( idx = 0; idx < GAP_BONDINGS_MAX; idx++ )
  {
	    // Look Connection Handle match
#if AUTH_ENABLE
		if ( bonds[idx].connHndl == connectionHandle )
#else
		if(memcmp(deviceId,bonds[idx].bond_rec.publicAddr,B_ADDR_LEN)==0)
#endif
		{
		  return ( idx ); // Found one
		}
  }

  return ( GAP_BONDINGS_MAX );
}


/*********************************************************************
 * @fn      gapBondMgrBondTotal
 *
 * @brief   Look through the bonding NV entries calculate the number
 *          entries.
 *
 * @param   none
 *
 * @return  total number of bonds found
 */
static uint8 gapBondMgrBondTotal( void )
{
  uint8 idx;
  uint8 numBonds = 0;
	BondRec_t * bonds;
	bonds = (BondRec_t *)blefiCfgRec.bonds;

  // Item doesn't exist, so create all the items
  for ( idx = 0; idx < GAP_BONDINGS_MAX; idx++ )
  {
    // Look for public address that are not 0xFF's
    if ( isBufset(bonds[idx].bond_rec.publicAddr, 0xFF, B_ADDR_LEN ) == FAILURE )
    {
      numBonds++; // Found one
    }
  }

  return ( numBonds );
}

/*********************************************************************
 * @fn      gapBondMgrAddBond
 *
 * @brief   Save a bond from a GAP Auth Complete Event
 *
 * @param   pBondRec - basic bond record
 * @param   pLocalLTK - LTK used by this device during pairing
 * @param   pDevLTK - LTK used by the connected device during pairing
 * @param   pIRK - IRK used by the connected device during pairing
 * @param   pSRK - SRK used by the connected device during pairing
 * @param   signCounter - Sign counter used by the connected device during pairing
 *
 * @return  TRUE, if done processing bond record. FALSE, otherwise.
 */
uint8 gapBondMgrAddBond( BondRec_t *pBondRec, gapAuthCompleteEvent_t *pPkt )
{
	// Make sure we have bonding info
	if ( ( pBondRec == NULL ) || ( pPkt == NULL ) )
	{
	  return ( FALSE );
	}

	// Do we have a public address in the data?
	if ( pPkt->pIdentityInfo.idInfoEnable )
	{
		memcpy(&(pBondRec->devIRK[0]), &(pPkt->pIdentityInfo.idInfo.irk[0]), KEYLEN);
	}
	if ( pPkt->pDevSecInfo.secInfoEnable )
	{
		memcpy(&(pBondRec->dev_ltk), &(pPkt->pDevSecInfo.secInfo), sizeof(gapBondLTK_t));
	}
	if( pPkt->pSecurityInfo.secInfoEnable )
	{
		memcpy(&(pBondRec->local_ltk), &(pPkt->pSecurityInfo.secInfo), sizeof(gapBondLTK_t));
	}

	if( pPkt->pSigningInfo.signInfoEnable )
	{
		pBondRec->signCounter = pPkt->pSigningInfo.signInfo.signCounter;
		memcpy(&(pBondRec->devCSRK[0]), &(pPkt->pSigningInfo.signInfo.srk[0]), KEYLEN);
	}

	return ( TRUE );
}

/*********************************************************************
 * @fn      gapBondMgrEraseAllBondings
 *
 * @brief   Write all 0xFF's to all of the bonding entries
 *
 * @param   none
 *
 * @return  SUCCESS if successful.
 *          Otherwise, NV_OPER_FAILED for failure.
 */
bStatus_t gapBondMgrEraseAllBondings( void )
{
  uint8 idx;
  bStatus_t stat = SUCCESS;  // return value
  BondRec_t * bonds;
  bonds = (BondRec_t *)blefiCfgRec.bonds;


  // Item doesn't exist, so create all the items
  for ( idx = 0; (idx < GAP_BONDINGS_MAX) && (stat == SUCCESS); idx++ )
  {
    // Erasing will write/create a bonding entry
    memset( (void *)&(bonds[idx]), (int)0xFF, sizeof(BondRec_t));
    gapBondMgrFlashBondRecUpdate();
  }

  return ( stat );
}





/*********************************************************************
 * @fn      gapBondMgrAuthenticate
 *
 * @brief   Initiate authentication
 *
 * @param   connHandle - connection handle
 * @param   addrType - peer address type
 * @param   pPairReq - Enter these parameters if the Pairing Request was already received.
 *          NULL, if waiting for Pairing Request or if initiating.
 *
 * @return  none
 */
static void gapBondMgrAuthenticate( uint16 connHandle, uint8 addrType, gapPairingReq_t *pPairReq )
{
  gapAuthParams_t params;

  memset( &params, 0, sizeof ( gapAuthParams_t ) );

  // Setup the pairing parameters
  params.connectionHandle = connHandle;
  params.secReqs.ioCaps = gapBond_IOCap;
  params.secReqs.oobAvailable = gapBond_OOBDataFlag;
  params.secReqs.maxEncKeySize = gapBond_KeySize;

  params.secReqs.keyDist.sEncKey = (gapBond_KeyDistList & GAPBOND_KEYDIST_SENCKEY) ? TRUE : FALSE;
  params.secReqs.keyDist.sIdKey = (gapBond_KeyDistList & GAPBOND_KEYDIST_SIDKEY) ? TRUE : FALSE;
  params.secReqs.keyDist.mEncKey = (gapBond_KeyDistList & GAPBOND_KEYDIST_MENCKEY) ? TRUE : FALSE;
  params.secReqs.keyDist.mIdKey = (gapBond_KeyDistList & GAPBOND_KEYDIST_MIDKEY) ? TRUE : FALSE;
  params.secReqs.keyDist.mSign = (gapBond_KeyDistList & GAPBOND_KEYDIST_MSIGN) ? TRUE : FALSE;
  params.secReqs.keyDist.sSign = (gapBond_KeyDistList & GAPBOND_KEYDIST_SSIGN) ? TRUE : FALSE;

  // Is bond manager setup for OOB data?
  if ( gapBond_OOBDataFlag )
  {
    memcpy( &(params.secReqs.oob[0]), &(gapBond_OOBData[0]), KEYLEN );
  }

  if ( gapBond_Bonding && addrType != ADDRTYPE_PUBLIC )
  {
    // Force a slave ID key
    params.secReqs.keyDist.sIdKey = TRUE;
  }

  params.secReqs.authReq |= (gapBond_Bonding) ? SM_AUTH_STATE_BONDING : 0;
  params.secReqs.authReq |= (gapBond_MITM) ? SM_AUTH_STATE_AUTHENTICATED : 0;

  pPairReq->enable  =  0;   //Disable
  pPairReq->authReq |= (gapBond_Bonding) ? SM_AUTH_STATE_BONDING : 0;
  pPairReq->authReq |= (gapBond_MITM) ? SM_AUTH_STATE_AUTHENTICATED : 0;
  pPairReq->ioCap = gapBond_IOCap;
  pPairReq->oobDataFlag = gapBond_OOBDataFlag;
  pPairReq->maxEncKeySize = gapBond_KeySize;
  memcpy(&(pPairReq->keyDist), &(params.secReqs.keyDist), sizeof(uint8));

  GAP_Authenticate( &params, pPairReq );
}

/*********************************************************************
 * @fn      gapBondMgrBondReq
 *
 * @brief   Initiate a GAP bond request
 *
 * @param   connHandle - connection handle
 * @param   idx - NV index of bond entry
 * @param   stateFlags - bond state flags
 * @param   role - master or slave role
 * @param   startEncryption - whether or not to start encryption
 *
 * @return  none
 */
static void gapBondMgrBondReq( uint16 connHandle, uint8 idx, uint8 stateFlags,
                               uint8 role, uint8 startEncryption )
{
	smSecurityInfo_t ltk;
	BondRec_t * bonds;
	bonds = (BondRec_t *)blefiCfgRec.bonds;

	// Initialize the NV structures
	memset( &ltk, 0, sizeof ( smSecurityInfo_t ) );

	memcpy( &ltk, &(bonds[idx].dev_ltk) ,sizeof ( smSecurityInfo_t ) );

	if ( (ltk.keySize >= MIN_ENC_KEYSIZE) && (ltk.keySize <= MAX_ENC_KEYSIZE) )
	{
	  GAP_Bond( connHandle,	((stateFlags & GAP_BONDED_STATE_AUTHENTICATED) ? TRUE : FALSE), &ltk, startEncryption );
	}

}


unsigned int getSizeofBondCfg()
{
	return (sizeof(BondRec_t) * GAP_BONDINGS_MAX);
}

void fillBondDefaultCfg(void * bondRec)
{
	memset(blefiCfgRec.bonds,0x00,sizeof(BondRec_t) * GAP_BONDINGS_MAX);
}

void GC_PairDevList(fptrPairlist_t fptr)
{
	char idx;
	BondRec_t * bonds;
	bonds = (BondRec_t *)blefiCfgRec.bonds;
	for ( idx = 0; idx < GAP_BONDINGS_MAX; idx++ )
	{
		// Look for public address of all 0x00's
		if ( isBufset( bonds[idx].bond_rec.publicAddr, 0x00, B_ADDR_LEN ) != SUCCESS)
		{
			(*fptr)(idx,bonds[idx].bond_rec.publicAddr);
		}
	}

}

int GC_UnPair(unsigned int idx)
{
	BondRec_t * bonds;
	bonds = (BondRec_t *)blefiCfgRec.bonds;

	if(idx>=GAP_BONDINGS_MAX)
		return -1;
	if ( isBufset( bonds[idx].bond_rec.publicAddr, 0x00, B_ADDR_LEN ) != SUCCESS)
	{
		memset(bonds[idx].bond_rec.publicAddr,0x00,B_ADDR_LEN);
		UpdateBlefiCfg();
		return 0;
	}
	return -1;

}

/*********************************************************************
*********************************************************************/
