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
  @file  BleAPI.c
  @brief BLE stack C interface implementation
  -->
*/

/*******************************************************************************
 * INCLUDES
 */
#include <stdio.h>
#include <stdlib.h>
#include "gap.h"
#include "gap_packets.h"




#define MSG_BUFFER_NOT_AVAIL FAILURE

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */
extern void HandleError(unsigned char error);
/*********************************************************************
 * LOCAL FUNCTIONS
 */


/*********************************************************************
 * API FUNCTIONS
 */


/*********************************************************************
 * Use this function to initialize GAP Device parameters
 *
 * Public function defined in gap.h.
 */
bStatus_t GAP_DeviceInit(unsigned char taskID, unsigned char profileRole,
                         unsigned char maxScanResponses, unsigned char *pIRK, unsigned char *pSRK,
                         unsigned char *pSignCounter)
{

	bStatus_t status_flag = FAILURE;
  // Allocate message buffer space
	gapDeviceInit_t *msg =
    (gapDeviceInit_t *)malloc(sizeof(gapDeviceInit_t));

	if(msg == NULL)
	{
		//while(1);
		HandleError(1); // Fatal Error
	}

  if (msg)
  {
    // Set GAP Device parameters
    msg->ProfileRole = profileRole;
    msg->MaxScanResponses = maxScanResponses;

    /* Note that decision to map the data pointer as is
     * depends on how the pointer is actually used by the other side.
     * It is safely done so here since GAP_DeviceInit() expects,
     * the buffers passed by pointers to persist while the device
     * is powered on.
     */
    msg->pIRK = pIRK;
    msg->pSRK = pSRK;
    msg->pSignCounter = pSignCounter;

    // Send the message
    GAP_deviceInitPacketize(msg);

	//
	// Wait to receive Command Status event from the CC2650
	//
    status_flag =  BLEAPI_waitForSpecficEvent(GAP_HCI_EXTENTIONCOMMANDSTATUS, GAP_DEVICEINIT, TRUE);

	free(msg);

	return status_flag;
  }

  return MSG_BUFFER_NOT_AVAIL;
}


/*********************************************************************
 * Setup the device's address type.
 *
 * Public function defined in gap.h
 */
bStatus_t GAP_ConfigDeviceAddr(unsigned char addrType, unsigned char *pStaticAddr)
{

	bStatus_t status_flag = FAILURE;

	// Allocate message buffer space
	gapConfigDevAddr_t *msg =
    (gapConfigDevAddr_t *)malloc(sizeof(gapConfigDevAddr_t));

  if (msg)
  {
    // Set address type and copy address
    msg->addrType = addrType;
    memcpy(msg->devAddr, pStaticAddr, B_ADDR_LEN);

    GAP_configDeviceAddrPacketize(msg);

	//
	// Wait to receive Command Status event from the CC2650
	//
	status_flag = BLEAPI_waitForSpecficEvent(GAP_HCI_EXTENTIONCOMMANDSTATUS, GAP_CONFIGUREDEVICEADDR, TRUE);

	free(msg);

	return status_flag;

  }

  return MSG_BUFFER_NOT_AVAIL;
}


/*********************************************************************
 * Establish a link to a slave device.
 *
 * Public function defined in gap.h.
 */
bStatus_t GAP_EstablishLinkReq( gapEstLinkReq_t *pParams )
{
	bStatus_t status_flag = FAILURE;

	if (pParams)
	{

		// Send the message
		GAP_establishLinkRequestPacketize(pParams);

		//
		// Wait to receive Command Status event from the CC2650
		//
		status_flag  = BLEAPI_waitForSpecficEvent(GAP_HCI_EXTENTIONCOMMANDSTATUS, GAP_ESTABLISHLINKREQUEST, TRUE);

		return  status_flag;
	}

  return MSG_BUFFER_NOT_AVAIL;
}

/*********************************************************************
 * Update the link parameters to a slave device.
 *
 * Public function defined in gap.h.
 */
bStatus_t GAP_UpdateLinkParamReq( gapUpdateLinkParamReq_t *pParams )
{
  if (pParams)
  {
    // Send the message
    GAP_updateLinkParamterRequestPacketize(pParams);
	//
	// Wait to receive Command Status event from the CC2650
	//
	return BLEAPI_waitForSpecficEvent(GAP_HCI_EXTENTIONCOMMANDSTATUS, GAP_UPDATELINKPARAMREQUEST, TRUE);
  }

  return MSG_BUFFER_NOT_AVAIL;
}

/*********************************************************************
 * Terminate a link connection.
 *
 * Public function defined in gap.h.
 */
bStatus_t GAP_TerminateLinkReq(unsigned char taskID, unsigned short int connHandle, unsigned char reason)
{

	bStatus_t status_flag = FAILURE;
    // Allocate message buffer space
	gapTerminateLink_t *msg =
    (gapTerminateLink_t *)malloc(sizeof(gapTerminateLink_t));

  if (msg)
  {

    msg->connHandle = connHandle;
    msg->reason = reason;

    // Send the message
    GAP_terminateLinkRequestPackeize(msg);

	//
	// Wait to receive Command Status event from the CC2650
	//
    status_flag =  BLEAPI_waitForSpecficEvent(GAP_HCI_EXTENTIONCOMMANDSTATUS, GAP_TERMINATELINKREQUEST, TRUE);

	free(msg);

	return status_flag;
  }

  return MSG_BUFFER_NOT_AVAIL;
}

/*********************************************************************
 * Start a device discovery scan.
 *
 * Public function defined in gap.h.
 */
bStatus_t GAP_DeviceDiscoveryRequest( gapDevDiscReq_t *pParams )
{
  if (pParams)
  {
    // Send the message
    GAP_deviceDiscoveryRequestPacketize(pParams);
	//
	// Wait to receive Command Status event from the CC2650
	//
	return BLEAPI_waitForSpecficEvent(GAP_HCI_EXTENTIONCOMMANDSTATUS, GAP_DEVICEDISCOVERYREQUEST, TRUE);

  }

  return MSG_BUFFER_NOT_AVAIL;
}

/*********************************************************************
 * Cancel an existing device discovery request.
 *
 * Public function defined in gap.h.
 */
bStatus_t GAP_DeviceDiscoveryCancel(unsigned char taskID)
{

    // Send the message
    GAP_deviceDiscoveryCancelPacketize();

	//
	// Wait to receive Command Status event from the CC2650
	//
	return BLEAPI_waitForSpecficEvent(GAP_HCI_EXTENTIONCOMMANDSTATUS, GAP_DEVICEDISCOVERYCANCEL, TRUE);
}

/*********************************************************************
 * GAP Authenticate
 *
 * Public function defined in gap.h.
 */
bStatus_t GAP_Authenticate(gapAuthParams_t *pParams,
                           gapPairingReq_t *pPairReq)
{

	bStatus_t status_flag = FAILURE;
	/* Allocate message buffer space */
	gapAuthenticateParams_t *msg =
    (gapAuthenticateParams_t *)malloc(sizeof(gapAuthenticateParams_t));

  if (msg)
  {
    // Set params
    msg->pParams = pParams;
    msg->pPairReq = pPairReq;

    // Send the message
    GAP_authenticatePacketize(msg);
	//
	// Wait to receive Command Status event from the CC2650
	//
    status_flag =  BLEAPI_waitForSpecficEvent(GAP_HCI_EXTENTIONCOMMANDSTATUS, GAP_AUTHENTICATE, TRUE);
    free(msg);
	return status_flag;
  }

  return MSG_BUFFER_NOT_AVAIL;
}

/*********************************************************************
 * Terminate an authentication/pairing process.
 *
 * Public function defined in gap.h.
 *
 * @return  SUCCESS or FAILURE
 */
bStatus_t GAP_TerminateAuth(unsigned short int connectionHandle, unsigned char reason)
{

	bStatus_t status_flag = FAILURE;
	gapTerminateAuth_t *msg =
    (gapTerminateAuth_t *)malloc(sizeof(gapTerminateAuth_t));

  if (msg)
  {
    // Set connHandle and reason
    msg->connHandle = connectionHandle;
    msg->reason = reason;

    // Send the message
    GAP_TerminateAuthPacketize(msg);
	//
	// Wait to receive Command Status event from the CC2650
	//
    status_flag =  BLEAPI_waitForSpecficEvent(GAP_HCI_EXTENTIONCOMMANDSTATUS, GAP_TERMINATEAUTH, TRUE);
    free(msg);
	return status_flag;
  }

  return MSG_BUFFER_NOT_AVAIL;
}

/*********************************************************************
* @brief  GAP_Bond API
*
* Public function defined in gap.h.
*/
bStatus_t GAP_Bond(unsigned short int connHandle, unsigned char authenticated,
                  smSecurityInfo_t *pParams, unsigned char startEncryption)
{

	bStatus_t status_flag = FAILURE;
	/* Allocate message buffer space */
	gapBond_t *msg =
    (gapBond_t *)malloc(sizeof(gapBond_t));

  if (msg)
  {
    // Set connHandle and other params for bonding
    msg->connHandle = connHandle;
    msg->authenticated = authenticated;
    msg->pParams = pParams;
    msg->startEncryption = startEncryption;

    // Send the message
    GAP_BondPacketize(msg);
	//
	// Wait to receive Command Status event from the CC2650
	//
    status_flag =  BLEAPI_waitForSpecficEvent(GAP_HCI_EXTENTIONCOMMANDSTATUS, GAP_BOND, TRUE);
    free(msg);
	return status_flag;
  }

  return MSG_BUFFER_NOT_AVAIL;
}

/*********************************************************************
 * Set up the connection to accept signed data.
 *
 * Public function defined in gap.h.
 */
bStatus_t GAP_Signable(uint16 connectionHandle, uint8 authenticated,
                       smSigningInfo_t *pParams)
{
	bStatus_t status_flag = FAILURE;
	/* Allocate message buffer space */
	gapSignable_t *msg =
    (gapSignable_t *)malloc(sizeof(gapSignable_t));

  if (msg)
  {
    // set connection handle and other params
    msg->connHandle = connectionHandle;
    msg->authenticated = authenticated;
    msg->pParams = pParams;

    // Send the message
    GAP_SignablePacketize(msg);
 	//
 	// Wait to receive Command Status event from the CC2650
 	//
     status_flag =  BLEAPI_waitForSpecficEvent(GAP_HCI_EXTENTIONCOMMANDSTATUS, GAP_SIGNABLE, TRUE);
     free(msg);
 	 return status_flag;
  }

  return MSG_BUFFER_NOT_AVAIL;
}

/*********************************************************************
 * GAP Update the passkey.
 *
 * Public function defined in gap.h.
 *
 * @return  SUCCESS or FAILURE
 */
bStatus_t GAP_PasskeyUpdate(uint8 *pPasskey, uint16 connectionHandle)
{
	bStatus_t status_flag = FAILURE;
	/* Allocate message buffer space */
	gapPassKeyUpdateParam_t *msg =
    (gapPassKeyUpdateParam_t *)malloc(sizeof(gapPassKeyUpdateParam_t));

  if (msg)
  {

    // Set connection handle and passkey
    msg->connHandle = connectionHandle;
    msg->pPasskey = pPasskey;

    // Send the message
    GAP_PasskeyUpdatePacketize(msg);
 	//
 	// Wait to receive Command Status event from the CC2650
 	//
     status_flag =  BLEAPI_waitForSpecficEvent(GAP_HCI_EXTENTIONCOMMANDSTATUS, GAP_PASSKEY_UPDATE, TRUE);
     free(msg);
 	 return status_flag;
  }

  return MSG_BUFFER_NOT_AVAIL;
}

/*********************************************************************
 * Set a GAP Parameter value.  Use this function to change the default
 * GAP parameter values.
 *
 * Public function defined in gap.h.
 */
bStatus_t GAP_SetParamValue(gapParamIDs_t paramID, unsigned short int paramValue)
{

	bStatus_t status_flag = FAILURE;
	/* Allocate message buffer space */
	gapSetParam_t *msg =
    (gapSetParam_t *)malloc(sizeof(gapSetParam_t));

  if (msg)
  {
    // Set param ID and value
    msg->paramID = paramID;
    msg->paramValue = paramValue;

    // Send the message
    GAP_setParameterPacketize(msg);
	//
	// Wait to receive Command Status event from the CC2650
	//
    status_flag =  BLEAPI_waitForSpecficEvent(GAP_HCI_EXTENTIONCOMMANDSTATUS, GAP_SETPARAMETER, TRUE);
    free(msg);
	return status_flag;
  }

  return MSG_BUFFER_NOT_AVAIL;
}

/*********************************************************************
 * Get a GAP Parameter value.  Use this function to get GAP parameter values.
 *
 * Public function defined in gap.h.
 */
unsigned short int GAP_GetParamValue(gapParamIDs_t paramID)
{

	unsigned short int paramValue = 0;
	bStatus_t status_flag = FAILURE;

	/* Allocate message buffer space */
	gapGetParam_t *msg = (gapGetParam_t *)malloc(sizeof(gapGetParam_t));
	if(msg)
	{
		msg->paramID = paramID;
		// Send the message
		GAP_getParameterPacketize(msg);

		//
		// Wait to receive Command Status event from the CC2650
		//
		status_flag = BLEAPI_waitForSpecficEvent(GAP_HCI_EXTENTIONCOMMANDSTATUS, GAP_GETPARAMETER, TRUE);

		if(status_flag == FAILURE)
		{
			// tbd
		}

		free(msg);
	}

	return paramValue;
}

/*********************************************************************
 * Resolve private address
 *
 * Public function defined in gap.h.
 */
bStatus_t GAP_ResolvePrivateAddr(uint8 *pIRK, uint8 *pAddr)
{
	bStatus_t status_flag = FAILURE;
	/* Allocate message buffer space */
	gapResolvePrivateAddr_t *msg =
    (gapResolvePrivateAddr_t *)malloc(sizeof(gapResolvePrivateAddr_t));

  if (msg)
  {

    // copy parameters
    msg->pIRK = pIRK;
    msg->pAddr = pAddr;

	// Send the message
    GAP_ResolvePrivateAddrPacketize(msg);

	//
	// Wait to receive Command Status event from the CC2650
	//
	status_flag = BLEAPI_waitForSpecficEvent(GAP_HCI_EXTENTIONCOMMANDSTATUS, GAP_RESOLVE_PRIVATE_ADDR, TRUE);
	if(status_flag == FAILURE)
	{
		// tbd
	}


	free(msg);
  }

  return MSG_BUFFER_NOT_AVAIL;
}

/*********************************************************************
 * Send Slave Security Request
 *
 * Public function defined in gap.h.
 */
bStatus_t GAP_SendSlaveSecurityRequest(uint16 connHandle, uint8 authReq)
{
	bStatus_t status_flag = FAILURE;
	/* Allocate message buffer space */
	gapSendSlaveSecReq_t *msg =
    (gapSendSlaveSecReq_t *)malloc(sizeof(gapSendSlaveSecReq_t));

  if (msg)
  {

    // copy parameters
    msg->connHandle = connHandle;
    msg->authReq = authReq;

	// Send the message
    GAP_SendSlaveSecurityRequestPacketize(msg);

	//
	// Wait to receive Command Status event from the CC2650
	//
	status_flag = BLEAPI_waitForSpecficEvent(GAP_HCI_EXTENTIONCOMMANDSTATUS, GAP_SLAVE_SECURITY_REQ_UPDATE, TRUE);
	if(status_flag == FAILURE)
	{
		// tbd
	}


	free(msg);
  }

  return MSG_BUFFER_NOT_AVAIL;
}


/*********************************************************************
 * @brief   Set a GAP Bond Manager parameter.
 *
 * Public function defined in gapbondmgr.h.
 */
bStatus_t GAPBondMgr_SetParameter(unsigned short int param, unsigned char len, void *pValue)
{

	bStatus_t status_flag = FAILURE;
	/* Allocate message buffer space */
	gapBondMgrSetParams_t *msg =
			(gapBondMgrSetParams_t *)malloc(sizeof(gapBondMgrSetParams_t));

	  unsigned char *paramIdLenValue = (unsigned char *)malloc(len);

	  if(paramIdLenValue == NULL)
	  {
		  free(msg);
		  HandleError(1); //fatal error
		  return 1;
	  }

  if (msg)
  {
    /* create message header */
    msg->paramId = param;
    msg->paramDatalen = len;
    msg->pValue = 	paramIdLenValue;

    /* copy message body */
    memcpy(msg->pValue, pValue, len);

    /* Send the message. */
    GAPBondMgr_setParameterPacketize(msg);
	//
	// Wait to receive Command Status event from the CC2650
	//
    status_flag =  BLEAPI_waitForSpecficEvent(GAP_HCI_EXTENTIONCOMMANDSTATUS, GAPBONDMGR_SETPARAMETER, TRUE);
    free(msg);
	return status_flag;
  }

  // msg = NULL
  free(paramIdLenValue);
  return MSG_BUFFER_NOT_AVAIL;
}

/*********************************************************************
 * @brief   Get a GAP Bond Manager parameter.
 *
 * Public function defined in gapbondmgr.h.
 */
bStatus_t GAPBondMgr_GetParameter(unsigned short int param, void *pValue)
{

	bStatus_t status_flag = FAILURE;
	/* Allocate message buffer space */
	gapBondMgrGetParams_t *msg = (gapBondMgrGetParams_t *)malloc(sizeof(gapBondMgrGetParams_t));

  if (msg)
  {
    // Set paramID
    msg->paramId = param;

    // Send the message
    GAPBondMgr_getParameterPacketize(msg);

	//
	// Wait to receive Command Status event from the CC2650
	//
    status_flag =  BLEAPI_waitForSpecficEvent(GAP_HCI_EXTENTIONCOMMANDSTATUS, GAPBONDMGR_GETPARAMETER, TRUE);
    free(msg);
	return status_flag;

  }
  return MSG_BUFFER_NOT_AVAIL;
}


/*
 * Events
 *
 */



/**
 * @brief <b>Fucntion Name </b> GAP_deviceInitDone
 * @brief <b>Description</b>:
 * It is already decoded before this function, that
 * packet[0] = Total length of payload
 * packet[1] = PacketType(which is 0x04, Event)
 * packet[2] = 0xFF (Vendor Specific Events)
 * packet[3] = Length
 * packet[4:5] == EventOpcode  (GAP_DEVICEINITDONE 0x0600)
 *  Using EventOpcode, Event functions will be selected in the Switch() case:
 * In this function, rest of the packet will be decoded and assigned to the struct
 * @param Input Parameters:
 * <BR> unsigned char* packet is the packets we receive through SPI.
 * @return Return value:
 * <BR> bStatus_t is the status of this event.
 */
bStatus_t GAP_deviceInitDone(unsigned char* packet)
{
	gapDeviceInitDone_t event;
//	unsigned int i;

	//Status (1 octet)
	event.status = (bStatus_t) packet[5];

	return event.status;
}

/**
 * @brief <b>Fucntion Name </b> GAP_advertDataUpdateDone
 * @brief <b>Description</b>:
 * It is already decoded before this function, that
 * packet[0] = Total length of payload
 * packet[1] = PacketType(which is 0x04, Event)
 * packet[2] = 0xFF (Vendor Specific Events)
 * packet[3] = Length
 * packet[4:5] = EventOpcode  (GAP_ADVERTDATAUPDATEDONE 0x0602)
 *  Using EventOpcode, Event functions will be selected in the Switch() case:
 * In this function, rest of the packet will be decoded and assigned to the struct
 * @param Input Parameters:
 * <BR> unsigned char* packet is the packets we receive through SPI.
 * @return Return value:
 * <BR> bStatus_t is the status of this event.
 */
bStatus_t GAP_advertDataUpdateDone(unsigned char* packet)
{
	gapAdvertDataUpdateDone_t event;

	//Status (1 octet)
	event.status = (bStatus_t) packet[5];

	return event.status;
}

/**
 * @brief <b>Fucntion Name </b> GAP_makeDiscoverableDone
 * @brief <b>Description</b>:
 * It is already decoded before this function, that
 * packet[0] = Total length of payload
 * packet[1] == PacketType(which is 0x04, Event)
 * packet[2] == 0xFF (Vendor Specific Events)
 * packet[3] == Length
 * packet[4:5] == EventOpcode  (GAP_MAKEDISCOVERABLEDONE  0x0603)
 *  Using EventOpcode, Event functions will be selected in the Switch() case:
 * In this function, rest of the packet will be decoded and assigned to the struct
 * @param Input Parameters:
 * <BR> unsigned char* packet is the packets we receive through SPI.
 * @return Return value:
 * <BR> bStatus_t is the status of this event.
 */
bStatus_t GAP_makeDiscoverableDone(unsigned char* packet)
{
	gapMakeDiscoverableDone_t event;

	//Status (1 octet)
	event.status = (bStatus_t) packet[5];

	return event.status;
}

/**
 * @brief <b>Fucntion Name </b> GAP_linkEstablished
 * @brief <b>Description</b>:
 * It is already decoded before this function, that
 * packet[0] = Total length of payload
 * packet[1] = PacketType(which is 0x04, Event)
 * packet[2] = 0xFF (Vendor Specific Events)
 * packet[3] = Length
 * packet[4:5] = EventOpcode  (GAP_ESTABLISHLINK 0x0605)
 *  Using EventOpcode, Event functions will be selected in the Switch() case:
 * In this function, rest of the packet will be decoded and assigned to the struct
 * @param Input Parameters:
 * <BR> unsigned char* packet is the packets we receive through SPI.
 * @return Return value:
 * <BR> bStatus_t is the status of this event.
 */
bStatus_t GAP_linkEstablished(unsigned char* packet)
{
	gapEstLinkReqEvent_t event;
//	unsigned int i;

	//Status (1 octet)
	event.status = (bStatus_t) packet[5];

	return event.status;
}

/**
 * @brief <b>Fucntion Name </b> GAP_linkTerminated
 * @brief <b>Description</b>:
 * It is already decoded before this function, that
 * packet[0] = Total length of payload
 * packet[1] = PacketType(which is 0x04, Event)
 * packet[2] = 0xFF (Vendor Specific Events)
 * packet[3] = Length
 * packet[4:5] = EventOpcode  (GAP_LINKTERMINATED 0x0606)
 *  Using EventOpcode, Event functions will be selected in the Switch() case:
 * In this function, rest of the packet will be decoded and assigned to the struct
 * @param Input Parameters:
 * <BR> unsigned char* packet is the packets we receive through SPI.
 * @return Return value:
 * <BR> bStatus_t is status of this event.
 */
bStatus_t GAP_linkTerminated(unsigned char* packet)
{
	gapLinkUpdateEvent_t event;

	//Status (1 octet)
	event.status = (bStatus_t) packet[5];

	return event.status;
}

/*********************************************************************
*********************************************************************/
