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

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include "datatypes.h"
#include "gatt.h"
#include "hci.h"
#include "ble_central.h"
#include "gateway_task.h"
#include "gateway_cmds.h"


/*
 * EXTERN VARIABLES
 * */
extern gw_context_t gw_context;

// To be removed
unsigned short int selfEntity = 0;

extern void TurnOnLed(LedNum LedInstance);


/*********************************************************************
 * @fn      process_Scan
 *
 * @brief
 *
 * @param   input command parameters
 *
 * @return  none
 */

void process_Scan(void** inParams )
{

	//if(get_gwState() != GW_DISC_IN_PROGRESS && get_gwState() != GW_NO_INIT)
	if(get_gwState() != GW_DISC_IN_PROGRESS)
	{
		gwcontext_Init();
		set_gwState(GW_DISC_IN_PROGRESS);
		gw_context.scanCnt = 0;
		UART_PRINT("\n\r[GW] Discovering...");
		TurnOnLed(LED_2);

		GAPCentralRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
									  DEFAULT_DISCOVERY_ACTIVE_SCAN,
									  DEFAULT_DISCOVERY_WHITE_LIST);
	}
	else
	{
		DBG_PRINT("\n\r Discovery already in progress..");
	}
}


/*********************************************************************
 * @fn      process_ScanStop
 *
 * @brief
 *
 * @param   input command parameters
 *
 * @return  none
 */

void process_ScanStop(void** inParams )
{
	// Stop Device Discovery
	set_gwState(GW_DISC_IN_PROGRESS);
	GAPCentralRole_CancelDiscovery();
}


/*********************************************************************
 * @fn      process_LinkEstablish
 *
 * @brief
 *
 * @param   input command parameters
 *
 * @return  none
 */

void process_LinkEstablish(void** inParams )
{
	//  LINK_ESTABLISH

   unsigned char deviceIdx;
   unsigned char addrType;
   unsigned char *peerAddr;

	// Establish link to the specified Device ID
    deviceIdx = (((param_LinkEstablish_t *)*inParams)->DeviceId);

    // Connect or disconnect
    if (gw_context.state == GW_DISC_DONE)
    {
      // if there is a scan result
      if (gw_context.scanCnt > 0)
      {

        peerAddr = (gw_context.scanRespArray[deviceIdx]->addr);
        addrType = (gw_context.scanRespArray[deviceIdx]->addrType);

        gw_context.scanRespArray[deviceIdx]->deviceRec->dev_state = DEV_CONNECTING;

        GAPCentralRole_EstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                           DEFAULT_LINK_WHITE_LIST,
                                           addrType, peerAddr);

        UART_PRINT("\n\r[GW] Connecting to the device...");
      }
    }
}


/*********************************************************************
 * @fn      process_LinkTerminate
 *
 * @brief
 *
 * @param   input command parameters
 *
 * @return  none
 */
void process_LinkTerminate(void** inParams )
{
	// LINK_TERMINATE

	unsigned char deviceIdx;

	// Terminate link to the specified Device ID
    deviceIdx = (((param_LinkTerminate_t *)*inParams)->DeviceId);

	// disconnect
    gw_context.scanRespArray[deviceIdx]->deviceRec->dev_state = DEV_DISCONNECTING;

    GAPCentralRole_TerminateLink(deviceIdx);
    UART_PRINT("\n\r[GW] Disconnecting from Device...");

}


/*********************************************************************
 * @fn      process_GattDiscAllPrimaryServices
 *
 * @brief
 *
 * @param   input command parameters
 *
 * @return  none
 */
// Do a write
void process_GattDiscAllPrimaryServices(void** inParams )
{
	GATT_DiscAllPrimaryServices( ((param_GattDiscAllPrimaryServices_t *)*inParams)->connHandle,
								   selfEntity);
}




/*********************************************************************
 * @fn      process_GattDiscAllChars
 *
 * @brief
 *
 * @param   input command parameters
 *
 * @return  none
 */
// Do a write
void process_GattDiscAllChars(void** inParams )
{

	 GATT_DiscAllChars(  ((param_GATT_DiscAllChars_t *)*inParams)->connHandle,
			             ((param_GATT_DiscAllChars_t *)*inParams)->startHandle,
			             ((param_GATT_DiscAllChars_t *)*inParams)->endHandle,
			               selfEntity);
}


/*********************************************************************
 * @fn      process_GattDiscAllChars
 *
 * @brief
 *
 * @param   input command parameters
 *
 * @return  none
 */
// Do a write
void process_GATTFindCharDescs(void** inParams )
{

	GATT_DiscAllCharDescs(  ((param_GATTFindCharDescs_t *)*inParams)->connHandle,
							((param_GATTFindCharDescs_t *)*inParams)->starthandle,
							((param_GATTFindCharDescs_t *)*inParams)->endhandle,
							  selfEntity);
}



/*********************************************************************
 * @fn      process_ReadHandle
 *
 * @brief
 *
 * @param   input command parameters
 *
 * @return  none
 */
// Do a Read of the handle
attReadReq_t attReadReq;
void process_ReadHandle(unsigned short int connhandle, unsigned short int char_handle)
{
	attReadReq.handle = char_handle;

	GATT_ReadCharValue( connhandle, &attReadReq, selfEntity );
}

/*********************************************************************
 * @fn      process_WriteHandle
 *
 * @brief
 *
 * @param   input command parameters
 *
 * @return  none
 */
// Do a Read of the handle
attWriteReq_t attWriteReq;
void process_WriteHandle(unsigned short int connhandle, unsigned short int char_handle ,unsigned char* pValue , unsigned short int ValueLen)
{
	attWriteReq.handle = char_handle;
	attWriteReq.len = ValueLen;

	if(ValueLen > (ATT_MTU_SIZE-2))
	{
		HandleError(__FILE__,__LINE__,1); //fatal error
		return;
	}

	memcpy(attWriteReq.value ,  pValue , ValueLen);
	attWriteReq.sig = 0;
	attWriteReq.cmd = 0;

	GATT_WriteCharValue( connhandle, &attWriteReq, selfEntity );
}


/*********************************************************************
 * @fn      process_GattReadUsingCharUuid
 *
 * @brief
 *
 * @param   input command parameters
 *
 * @return  none
 */
attReadByTypeReq_t attReadByTypeReq;
void process_GattReadUsingCharUuid(void** inParams )
{

	attReadByTypeReq.startHandle = ((param_GattReadUsingCharUuid_t *)*inParams)->starthandle;
	attReadByTypeReq.endHandle = ((param_GattReadUsingCharUuid_t *)*inParams)->endhandle;
	attReadByTypeReq.type.len = ((param_GattReadUsingCharUuid_t *)*inParams)->CharUuidlen;
	memcpy(attReadByTypeReq.type.uuid , ((param_GattReadUsingCharUuid_t *)*inParams)->pCharUuid ,attReadByTypeReq.type.len);

	GATT_ReadUsingCharUUID(  ((param_GattReadUsingCharUuid_t *)*inParams)->connHandle,
								&attReadByTypeReq,
							  selfEntity);
}


/*********************************************************************
 * @fn      GatewayCentral_handleinput
 *
 * @brief   Handles all input events for this device.
 *
 *
 *
 * @return  none
 */
void GatewayCentral_handleinput(cmd_hdr** inParams)
{
  unsigned char ucCmdReceived = ((cmd_hdr*)*inParams)->Cmd;

  switch(ucCmdReceived)
  {
	  case(DEVICE_SCAN):
	  {
		  process_Scan( (void**)inParams );
		  break;
	  }
	  case(DEVICE_STOPSCAN):
	  {
		  process_ScanStop( (void**)inParams );
		  break;
	  }
	  case(LINK_ESTABLISH):
	  {
		  process_LinkEstablish((void**) inParams );
		  break;
	  }
	  case(LINK_TERMINATE):
	  {
		  process_LinkTerminate( (void**)inParams );
		  break;
	  }
	  //Get or Set
  }

}

