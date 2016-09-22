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
 *	@file gap_packets.h
 *
 * 	@detail	Description:
 * 	THe header file of gap_packets.c
 *	Encode and Decode For GAP packets. For GAP Command, fill GAP related packet
 *	and then send the packet to CC2650 through SPI; For GAP Event, now just
 *	return the Status when receive the packet from CC2650 through SPI.
 *
 *  ***************************************************************************/

#ifndef GAP_PACKETS_H_
#define GAP_PACKETS_H_

/*
 * Commands
 */
extern void GAP_deviceInitPacketize(gapDeviceInit_t* para);
extern void GAP_configDeviceAddrPacketize(gapConfigDevAddr_t* para);
extern void GAP_establishLinkRequestPacketize(gapEstLinkReq_t *para);
extern void GAP_updateLinkParamterRequestPacketize(gapUpdateLinkParamReq_t *para);
extern void GAP_terminateLinkRequestPackeize(gapTerminateLink_t *para);
extern void GAP_deviceDiscoveryRequestPacketize(gapDevDiscReq_t *para);
extern void GAP_deviceDiscoveryCancelPacketize(void);
extern void GAP_makeDiscoverablePacketize(gapMakeDiscoverable_t* para);
extern void GAP_updateAdvertisingDataPacketize(gapUpdateAdvertData_t* para);
extern void GAP_authenticatePacketize(gapAuthenticateParams_t *para);
extern void GAP_TerminateAuthPacketize(gapTerminateAuth_t* para);
extern void GAP_BondPacketize(gapBond_t *para);
extern void GAP_SignablePacketize(gapSignable_t *para);
extern void GAP_PasskeyUpdatePacketize(gapPassKeyUpdateParam_t *para);
extern void GAP_setParameterPacketize(gapSetParam_t *para);
extern void GAP_getParameterPacketize(gapGetParam_t *para);
extern void GAP_ResolvePrivateAddrPacketize(gapResolvePrivateAddr_t *para);
extern void GAP_SendSlaveSecurityRequestPacketize(gapSendSlaveSecReq_t *para);
extern void GAPBondMgr_setParameterPacketize(gapBondMgrSetParams_t *para);
extern void GAPBondMgr_getParameterPacketize(gapBondMgrGetParams_t *para);

#endif /* GAP_PACKETS_H */
