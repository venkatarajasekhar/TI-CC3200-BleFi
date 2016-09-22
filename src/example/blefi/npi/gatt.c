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
 *	@file gatt.c
 *
 * 	@detail	Description:
 *	Encode For GATT packets. For GATT Command, fill GATT related packet
 *	and then send the packet to CC2650 through SPI.
 *
 *
 *  ***************************************************************************/

// Include Header Files
#include "gatt.h"
#include "spi_def.h"

#ifndef NPI_UART
#define NPI_UART 1
#endif

#define GATT_PACKET_SIZE 45
unsigned char gGattPacket[GATT_PACKET_SIZE];


#ifdef NPI_SPI

#include "spi_def.h"

// Local/Global Variables
extern unsigned char*   gSPIpacket;
extern unsigned char    gSPIpacketSize;

extern bool_t  gOutGoingPacket;

#endif  //ifdef NPI_SPI

#ifdef NPI_UART

#include "uart_def.h"

#endif  //ifdef NPI_UART

extern void HandleError(unsigned char error);

static void GATT_sendPacket( unsigned char* pGattPacket, unsigned char packetSize)
{

#ifdef NPI_SPI
	gSPIpacket  	= pGattPacket;
	gSPIpacketSize  = packetSize;

	gOutGoingPacket = TRUE;

	SignalMasterReadyLow();
#endif

#ifdef NPI_UART

	UartWritePacket(pGattPacket, packetSize);

#endif

}


/**
 * @brief   Initialize the Generic Attribute Profile Client.
 *
 * @return  SUCCESS: Client initialized successfully.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 */
 bStatus_t GATT_InitClient( void )
{
 	 return 0;
}

/**
 * @brief   Register to receive incoming ATT Indications or Notifications
 *          of attribute values.
 *
 * @param   taskId – task to forward indications or notifications to
 *
 * @return  void
 */
 void GATT_RegisterForInd( unsigned char taskId )
 {

 }

/**
 * @brief   The Prepare Write Request is used to request the server to
 *          prepare to write the value of an attribute.
 *
 *          Note: This function is needed only for GATT testing.
 *
 * @param   connHandle - connection to use
 * @param   pReq - pointer to request to be sent
 * @param   taskId - task to be notified of response
 *
 * @return  SUCCESS: Request was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid connection handle or request field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          blePending: A response is pending with this server.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 *          bleTimeout: Previous transaction timed out.<BR>
 */
 bStatus_t GATT_PrepareWriteReq( unsigned short int connHandle, attPrepareWriteReq_t *pReq, unsigned char taskId )
 {
 	 return 0;
 }
/**
 * @brief   The Execute Write Request is used to request the server to
 *          write or cancel the write of all the prepared values currently
 *          held in the prepare queue from this client.
 *
 *          Note: This function is needed only for GATT testing.
 *
 * @param   connHandle - connection to use
 * @param   pReq - pointer to request to be sent
 * @param   taskId - task to be notified of response
 *
 * @return  SUCCESS: Request was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid connection handle or request field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          blePending: A response is pending with this server.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 *          bleTimeout: Previous transaction timed out.<BR>
 */
 bStatus_t GATT_ExecuteWriteReq( unsigned short int connHandle, attExecuteWriteReq_t *pReq, unsigned char taskId )
 {
 	 return 0;
 }

/**
 * @}
 */

/*-------------------------------------------------------------------
 * GATT Server Public APIs
 */

/**
 * @defgroup GATT_SERVER_API GATT Server API Functions
 *
 * @{
 */

/**
 * @brief   Initialize the Generic Attribute Profile Server.
 *
 * @return  SUCCESS: Server initialized successfully.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 */
 bStatus_t GATT_InitServer( void )
 {
 	 return 0;
 }

/**
 * @brief   Register a service attribute list with the GATT Server. A service
 *          is composed of characteristics or references to other services.
 *          Each characteristic contains a value and may contain optional
 *          information about the value. There are two types of services:
 *          primary service and secondary service.
 *
 *          A service definition begins with a service declaration and ends
 *          before the next service declaration or the maximum Attribute Handle.
 *
 *          A characteristic definition begins with a characteristic declaration
 *          and ends before the next characteristic or service declaration or
 *          maximum Attribute Handle.
 *
 *          The attribute server will only keep a pointer to the attribute
 *          list, so the calling application will have to maintain the code
 *          and RAM associated with this list.
 *
 * @param   pService - pointer to service attribute list to be registered
 *
 * @return  SUCCESS: Service registered successfully.<BR>
 *          INVALIDPARAMETER: Invalid service field.<BR>
 *          FAILURE: Not enough attribute handles available.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 */
 bStatus_t GATT_RegisterService( gattService_t *pService )
 {
 	 return 0;
 }

/**
 * @brief   Deregister a service attribute list with the GATT Server.
 *
 *          NOTE: It's the caller's responsibility to free the service attribute
 *          list returned from this API.
 *
 * @param   handle - handle of service to be deregistered
 * @param   pService - pointer to deregistered service (to be returned)
 *
 * @return  SUCCESS: Service deregistered successfully.<BR>
 *          FAILURE: Service not found.<BR>
 */
 bStatus_t GATT_DeregisterService( unsigned short int handle, gattService_t *pService )
 {
 	 return 0;
 }

/**
 * @brief   Register to receive incoming ATT Requests.
 *
 * @param   taskId – task to forward requests to
 *
 * @return  void
 */
 void GATT_RegisterForReq( unsigned char taskId )
 {

 }

/**
 * @brief   Verify the permissions of an attribute for reading.
 *
 * @param   connHandle - connection to use
 * @param   permissions - attribute permissions
 *
 * @return  SUCCESS: Attribute can be read.<BR>
 *          ATT_ERR_READ_NOT_PERMITTED: Attribute cannot be read.<BR>
 *          ATT_ERR_INSUFFICIENT_AUTHEN: Attribute requires authentication.<BR>
 *          ATT_ERR_INSUFFICIENT_KEY_SIZE: Key Size used for encrypting is insufficient.<BR>
 *          ATT_ERR_INSUFFICIENT_ENCRYPT: Attribute requires encryption.<BR>
 */
 bStatus_t GATT_VerifyReadPermissions( unsigned short int connHandle, unsigned char permissions )
 {
 	 return 0;
 }

/**
 * @brief   Verify the permissions of an attribute for writing.
 *
 * @param   connHandle - connection to use
 * @param   permissions - attribute permissions
 * @param   pReq - pointer to write request
 *
 * @return  SUCCESS: Attribute can be written.<BR>
 *          ATT_ERR_READ_NOT_PERMITTED: Attribute cannot be written.<BR>
 *          ATT_ERR_INSUFFICIENT_AUTHEN: Attribute requires authentication.<BR>
 *          ATT_ERR_INSUFFICIENT_KEY_SIZE: Key Size used for encrypting is insufficient.<BR>
 *          ATT_ERR_INSUFFICIENT_ENCRYPT: Attribute requires encryption.<BR>
 */
 bStatus_t GATT_VerifyWritePermissions( unsigned short int connHandle, unsigned char permissions, attWriteReq_t *pReq )
 {
 	 return 0;
 }

/**
 * @brief   Send out a Service Changed Indication.
 *
 * @param   connHandle - connection to use
 * @param   taskId - task to be notified of confirmation
 *
 * @return  SUCCESS: Indication was sent successfully.<BR>
 *          FAILURE: Service Changed attribute not found.<BR>
 *          INVALIDPARAMETER: Invalid connection handle or request field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          blePending: A confirmation is pending with this client.<BR>
 */
 unsigned char GATT_ServiceChangedInd( unsigned short int connHandle, unsigned char taskId )
 {
 	 return 0;
 }

/**
 * @brief   Find the attribute record for a given handle and UUID.
 *
 * @param   startHandle - first handle to look for
 * @param   endHandle - last handle to look for
 * @param   pUUID - pointer to UUID to look for
 * @param   len - length of UUID
 * @param   pHandle - handle of owner of attribute (to be returned)
 *
 * @return  Pointer to attribute record. NULL, otherwise.
 */
 gattAttribute_t *GATT_FindHandleUUID( unsigned short int startHandle, unsigned short int endHandle, const unsigned char *pUUID,
                                             unsigned short int len, unsigned short int *pHandle )
 {
 	 return 0;
 }
/**
 * @brief   Find the attribute record for a given handle
 *
 * @param   handle - handle to look for
 * @param   pHandle - handle of owner of attribute (to be returned)
 *
 * @return  Pointer to attribute record. NULL, otherwise.
 */
 gattAttribute_t *GATT_FindHandle( unsigned short int handle, unsigned short int *pHandle )
 {
 	 return 0;
 }

/**
 * @brief   Find the next attribute of the same type for a given attribute.
 *
 * @param   pAttr - pointer to attribute to find a next for
 * @param   endHandle - last handle to look for
 * @param   service - handle of owner service
 * @param   pLastHandle - handle of last attribute (to be returned)
 *
 * @return  Pointer to next attribute record. NULL, otherwise.
 */
 gattAttribute_t *GATT_FindNextAttr( gattAttribute_t *pAttr, unsigned short int endHandle,
                                           unsigned short int service, unsigned short int *pLastHandle )
 {
 	 return 0;
 }
/**
 * @brief   Get the number of attributes for a given service
 *
 * @param   handle - service handle to look for
 *
 * @return  Number of attributes. 0, otherwise.
 */
 unsigned short int GATT_ServiceNumAttrs( unsigned short int handle )
 {
 	 return 0;
 }

/**
 * @}
 */

/*-------------------------------------------------------------------
 * GATT Server Sub-Procedure APIs
 */

/**
 * @defgroup GATT_SERVER_SUB_PROCEDURE_API GATT Server Sub-Procedure API Functions
 *
 * @{
 */

/**
 * @brief   This sub-procedure is used when a server is configured to
 *          indicate a characteristic value to a client and expects an
 *          attribute protocol layer acknowledgement that the indication
 *          was successfully received.
 *
 *          The ATT Handle Value Indication is used in this sub-procedure.
 *
 *          If the return status from this function is SUCCESS, the calling
 *          application task will receive an OSAL GATT_MSG_EVENT message.
 *          The type of the message will be ATT_HANDLE_VALUE_CFM.
 *
 *          Note: This sub-procedure is complete when ATT_HANDLE_VALUE_CFM
 *                (with SUCCESS or bleTimeout status) is received by the
 *                calling application task.
 *
 * @param   connHandle - connection to use
 * @param   pInd - pointer to indication to be sent
 * @param   authenticated - whether an authenticated link is required
 * @param   taskId - task to be notified of response
 *
 * @return  SUCCESS: Indication was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid connection handle or request field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          blePending: A confirmation is pending with this client.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 *          bleTimeout: Previous transaction timed out.<BR>
 */
 bStatus_t GATT_Indication( unsigned short int connHandle, attHandleValueInd_t *pInd,
                                  unsigned char authenticated, unsigned char taskId )
 {
	 	unsigned char j, i=0;
		bStatus_t status_flag = FAILURE;
#ifdef NPI_SPI
		// Fill Header and Packet Length First. Checked by CC26XX
		gGattPacket[i++] = 0xFE;        //Packet Header
		gGattPacket[i++] = 0x08+pInd->len;        //Packet Length
#endif
		//
		// Command Byte
		//
		gGattPacket[i++] = COMMAND;
		//
		// Opcode Bytes (2 Bytes)
		//
		gGattPacket[i++] = (unsigned char) GATT_INDICATION;
		gGattPacket[i++] = (unsigned char) (GATT_INDICATION >> 8);
		//
		// Length byte
		//
		gGattPacket[i++] = 0x04+pInd->len;

		//
		// Connection Handle (2 Bytes)
		//
		gGattPacket[i++] = (unsigned char)connHandle;
		gGattPacket[i++] = (unsigned char)(connHandle >> 8);


		gGattPacket[i++] = (unsigned char)(pInd->handle);
		gGattPacket[i++] = (unsigned char)(pInd->handle >> 8);

		for(j=0;j<pInd->len;j++)
			gGattPacket[i++] = pInd->value[j];

#ifdef NPI_SPI
		gGattPacket[i++] = 0x0C; //CRC Check.-Footer
#endif

		if(i > GATT_PACKET_SIZE)
		{
	        // error - GattPacket buffer overflow
			HandleError(1);
			return -1; // will never reach here
		}

		GATT_sendPacket(gGattPacket, i);

		//
		// Wait to receive Command Status event from the CC2650
		//
	    status_flag =  BLEAPI_waitForSpecficEvent(GAP_HCI_EXTENTIONCOMMANDSTATUS, GATT_INDICATION, TRUE);

		return status_flag;
 }

/**
 * @brief   This sub-procedure is used when a server is configured to
 *          notify a characteristic value to a client without expecting
 *          any attribute protocol layer acknowledgement that the
 *          notification was successfully received.
 *
 *          The ATT Handle Value Notification is used in this sub-procedure.
 *
 *          Note: A notification may be sent at any time and does not
 *          invoke a confirmation.
 *
 *          No confirmation will be sent to the calling application task for
 *          this sub-procedure.
 *
 * @param   connHandle - connection to use
 * @param   pNoti - pointer to notification to be sent
 * @param   authenticated - whether an authenticated link is required
 *
 * @return  SUCCESS: Notification was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid connection handle or request field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 *          bleTimeout: Previous transaction timed out.<BR>
 */
 bStatus_t GATT_Notification( unsigned short int connHandle, attHandleValueNoti_t *pNoti,
                                    unsigned char authenticated )
 {
	 	unsigned char j, i=0;
		bStatus_t status_flag = FAILURE;
#ifdef NPI_SPI
		// Fill Header and Packet Length First. Checked by CC26XX
		gGattPacket[i++] = 0xFE;        //Packet Header
		gGattPacket[i++] = 0x08+pNoti->pdulen;        //Packet Length
#endif
		//
		// Command Byte
		//
		gGattPacket[i++] = COMMAND;
		//
		// Opcode Bytes (2 Bytes)
		//
		gGattPacket[i++] = (unsigned char) GATT_NOTIFICATION;
		gGattPacket[i++] = (unsigned char) (GATT_NOTIFICATION >> 8);
		//
		// Length byte
		//
		gGattPacket[i++] = 0x04+pNoti->pdulen;

		//
		// Connection Handle (2 Bytes)
		//
		gGattPacket[i++] = (unsigned char)connHandle;
		gGattPacket[i++] = (unsigned char)(connHandle >> 8);


		gGattPacket[i++] = (unsigned char)(pNoti->handle);
		gGattPacket[i++] = (unsigned char)(pNoti->handle >> 8);

		for(j=0;j<pNoti->pdulen;j++)
			gGattPacket[i++] = pNoti->value[j];

#ifdef NPI_SPI
		gGattPacket[i++] = 0x0C; //CRC Check.-Footer
#endif

		if(i > GATT_PACKET_SIZE)
		{
	        // error - GattPacket buffer overflow
			HandleError(1);
			return -1; // will never reach here
		}

		GATT_sendPacket(gGattPacket, i);

		//
		// Wait to receive Command Status event from the CC2650
		//
	    status_flag =  BLEAPI_waitForSpecficEvent(GAP_HCI_EXTENTIONCOMMANDSTATUS, GATT_NOTIFICATION, TRUE);

		return status_flag;
 }

/**
 * @}
 */

/*-------------------------------------------------------------------
 * GATT Client Sub-Procedure APIs
 */

/**
 * @defgroup GATT_CLIENT_SUB_PROCEDURE_API GATT Client Sub-Procedure API Functions
 *
 * @{
 */

/**
 * @brief   This sub-procedure is used by the client to set the ATT_MTU
 *          to the maximum possible value that can be supported by both
 *          devices when the client supports a value greater than the
 *          default ATT_MTU for the Attribute Protocol. This sub-procedure
 *          shall only be initiated once during a connection.
 *
 *          The ATT Exchange MTU Request is used by this sub-procedure.
 *
 *          If the return status from this function is SUCCESS, the calling
 *          application task will receive an OSAL GATT_MSG_EVENT message.
 *          The type of the message will be either ATT_EXCHANGE_MTU_RSP or
 *          ATT_ERROR_RSP (if an error occurred on the server).
 *
 *          Note: This sub-procedure is complete when either ATT_EXCHANGE_MTU_RSP
 *                (with SUCCESS or bleTimeout status) or ATT_ERROR_RSP (with
 *                SUCCESS status) is received by the calling application task.
 *
 * @param   connHandle - connection to use
 * @param   pReq - pointer to request to be sent
 * @param   taskId - task to be notified of response
 *
 * @return  SUCCESS: Request was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid connection handle or request field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          blePending: A response is pending with this server.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 *          bleTimeout: Previous transaction timed out.<BR>
 */
 bStatus_t GATT_ExchangeMTU( unsigned short int connHandle, attExchangeMTUReq_t *pReq, unsigned char taskId )
 {
 	 return 0;
 }

/**
 * @brief   This sub-procedure is used by a client to discover all
 *          the primary services on a server.
 *
 *          The ATT Read By Group Type Request is used with the Attribute
 *          Type parameter set to the UUID for "Primary Service". The
 *          Starting Handle is set to 0x0001 and the Ending Handle is
 *          set to 0xFFFF.
 *
 *          If the return status from this function is SUCCESS, the calling
 *          application task will receive multiple OSAL GATT_MSG_EVENT messages.
 *          The type of the messages will be either ATT_READ_BY_GRP_TYPE_RSP
 *          or ATT_ERROR_RSP (if an error occurred on the server).
 *
 *          Note: This sub-procedure is complete when either ATT_READ_BY_GRP_TYPE_RSP
 *                (with bleProcedureComplete or bleTimeout status) or ATT_ERROR_RSP
 *                (with SUCCESS status) is received by the calling application
 *                task.
 *
 * @param   connHandle - connection to use
 * @param   taskId - task to be notified of response
 *
 * @return  SUCCESS: Request was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid connection handle or request field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          blePending: A response is pending with this server.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 *          bleTimeout: Previous transaction timed out.<BR>
 */
 bStatus_t GATT_DiscAllPrimaryServices( unsigned short int connHandle, unsigned char taskId )
{
	unsigned char i = 0;
	bStatus_t status_flag = FAILURE;
#ifdef NPI_SPI
	// Fill Header and Packet Length First. Checked by CC26XX
	gGattPacket[i++] = 0xFE;        //Packet Header
	gGattPacket[i++] = 0x06;        //Packet Length
#endif
	//
	// Command Byte
	//
	gGattPacket[i++] = COMMAND;
	//
	// Opcode Bytes (2 Bytes)
	//
	gGattPacket[i++] = (unsigned char) GATT_DISCALLPRIMARYSERVICES;
	gGattPacket[i++] = (unsigned char) (GATT_DISCALLPRIMARYSERVICES >> 8);
	//
	// Length byte
	//
	gGattPacket[i++] = 0x02;

	//
	// Connection Handle (2 Bytes)
	//
	gGattPacket[i++] = (unsigned char)connHandle;
	gGattPacket[i++] = (unsigned char)(connHandle >> 8);


#ifdef NPI_SPI
	gGattPacket[i++] = 0x0C; //CRC Check.-Footer
#endif

	if(i > GATT_PACKET_SIZE)
	{
        // error - GattPacket buffer overflow
		HandleError(1);
		return -1; // will never reach here
	}

	GATT_sendPacket(gGattPacket, i);

	//
	// Wait to receive Command Status event from the CC2650
	//
    status_flag =  BLEAPI_waitForSpecficEvent(GAP_HCI_EXTENTIONCOMMANDSTATUS, GATT_DISCALLPRIMARYSERVICES, TRUE);

	return status_flag;
}

/**
 * @brief   This sub-procedure is used by a client to discover a specific
 *          primary service on a server when only the Service UUID is
 *          known. The primary specific service may exist multiple times
 *          on a server. The primary service being discovered is identified
 *          by the service UUID.
 *
 *          The ATT Find By Type Value Request is used with the Attribute
 *          Type parameter set to the UUID for "Primary Service" and the
 *          Attribute Value set to the 16-bit Bluetooth UUID or 128-bit
 *          UUID for the specific primary service. The Starting Handle shall
 *          be set to 0x0001 and the Ending Handle shall be set to 0xFFFF.
 *
 *          If the return status from this function is SUCCESS, the calling
 *          application task will receive multiple OSAL GATT_MSG_EVENT messages.
 *          The type of the messages will be either ATT_FIND_BY_TYPE_VALUE_RSP
 *          or ATT_ERROR_RSP (if an error occurred on the server).
 *
 *          Note: This sub-procedure is complete when either ATT_FIND_BY_TYPE_VALUE_RSP
 *                (with bleProcedureComplete or bleTimeout status) or ATT_ERROR_RSP
 *                (with SUCCESS status) is received by the calling application task.
 *
 * @param   connHandle - connection to use
 * @param   pValue - pointer to value to look for
 * @param   len - length of value
 * @param   taskId - task to be notified of response
 *
 * @return  SUCCESS: Request was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid connection handle or request field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          blePending: A response is pending with this server.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 *          bleTimeout: Previous transaction timed out.<BR>
 */
 bStatus_t GATT_DiscPrimaryServiceByUUID( unsigned short int connHandle, unsigned char *pValue,
                                                unsigned char len, unsigned char taskId )
{
	unsigned char j, i=0;
	bStatus_t status_flag = FAILURE;
#ifdef NPI_SPI
	// Fill Header and Packet Length First. Checked by CC26XX
	gGattPacket[i++] = 0xFE;        //Packet Header
	gGattPacket[i++] = 0x06 + len;        //Packet Length
#endif
	//
	// Command Byte
	//
	gGattPacket[i++] = COMMAND;
	//
	// Opcode Bytes (2 Bytes)
	//
	gGattPacket[i++] = (unsigned char) GATT_DISCAPRIMARYSERVICEBYUUID;
	gGattPacket[i++] = (unsigned char) (GATT_DISCAPRIMARYSERVICEBYUUID >> 8);
	//
	// Length byte
	//
	gGattPacket[i++] = 0x02+len;

	gGattPacket[i++] = (unsigned char)connHandle;
	gGattPacket[i++] = (unsigned char)(connHandle >> 8);

	for(j=0;j<len;j++)
		gGattPacket[i++] = *pValue++;

#ifdef NPI_SPI
	gGattPacket[i++] = 0x0C; //CRC Check.-Footer
#endif

	if(i > GATT_PACKET_SIZE)
	{
        // error - GattPacket buffer overflow
		HandleError(1);
		return -1; // will never reach here
	}

	GATT_sendPacket(gGattPacket, i);

	//
	// Wait to receive Command Status event from the CC2650
	//
    status_flag =  BLEAPI_waitForSpecficEvent(GAP_HCI_EXTENTIONCOMMANDSTATUS, GATT_DISCAPRIMARYSERVICEBYUUID, TRUE);

	return status_flag;
}
/**
 * @brief   This sub-procedure is used by a client to find include
 *          service declarations within a service definition on a
 *          server. The service specified is identified by the service
 *          handle range.
 *
 *          The ATT Read By Type Request is used with the Attribute
 *          Type parameter set to the UUID for "Included Service". The
 *          Starting Handle is set to starting handle of the specified
 *          service and the Ending Handle is set to the ending handle
 *          of the specified service.
 *
 *          If the return status from this function is SUCCESS, the calling
 *          application task will receive multiple OSAL GATT_MSG_EVENT messages.
 *          The type of the messages will be either ATT_READ_BY_TYPE_RSP
 *          or ATT_ERROR_RSP (if an error occurred on the server).
 *
 *          Note: This sub-procedure is complete when either ATT_READ_BY_TYPE_RSP
 *                (with bleProcedureComplete or bleTimeout status) or ATT_ERROR_RSP
 *                (with SUCCESS status) is received by the calling application task.
 *
 * @param   connHandle - connection to use
 * @param   startHandle - starting handle
 * @param   endHandle - end handle
 * @param   taskId - task to be notified of response
 *
 * @return  SUCCESS: Request was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid connection handle or request field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          blePending: A response is pending with this server.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 *          bleTimeout: Previous transaction timed out.<BR>
 */
 bStatus_t GATT_FindIncludedServices( unsigned short int connHandle, unsigned short int startHandle,
                                            unsigned short int endHandle, unsigned char taskId )
{
	unsigned char i = 0;
	bStatus_t status_flag = FAILURE;
#ifdef NPI_SPI
	// Fill Header and Packet Length First. Checked by CC26XX
	gGattPacket[i++] = 0xFE;        //Packet Header
	gGattPacket[i++] = 0x0A;        //Packet Length
#endif
	//
	// Command Byte
	//
	gGattPacket[i++] = COMMAND;
	//
	// Opcode Bytes (2 Bytes)
	//
	gGattPacket[i++] = (unsigned char) GATT_FINDINCLUDEDSERVICES;
	gGattPacket[i++] = (unsigned char) (GATT_FINDINCLUDEDSERVICES >> 8);
	//
	// Length byte
	//
	gGattPacket[i++] = 0x06;

	//
	// Connection Handle (2 Bytes)
	//
	gGattPacket[i++] = (unsigned char)connHandle;
	gGattPacket[i++] = (unsigned char)(connHandle >> 8);

	gGattPacket[i++] = (unsigned char)startHandle;
	gGattPacket[i++] = (unsigned char)(startHandle >> 8);

	gGattPacket[i++] = (unsigned char)endHandle;
	gGattPacket[i++] = (unsigned char)(endHandle >> 8);

#ifdef NPI_SPI
	gGattPacket[i++] = 0x0C; //CRC Check.-Footer
#endif
	if(i > GATT_PACKET_SIZE)
	{
        // error - GattPacket buffer overflow
		HandleError(1);
		return -1; // will never reach here
	}

	GATT_sendPacket(gGattPacket, i);

	//
	// Wait to receive Command Status event from the CC2650
	//
    status_flag =  BLEAPI_waitForSpecficEvent(GAP_HCI_EXTENTIONCOMMANDSTATUS, GATT_FINDINCLUDEDSERVICES, TRUE);

	return status_flag;
}
/**
 * @brief   This sub-procedure is used by a client to find all the
 *          characteristic declarations within a service definition on
 *          a server when only the service handle range is known. The
 *          service specified is identified by the service handle range.
 *
 *          The ATT Read By Type Request is used with the Attribute Type
 *          parameter set to the UUID for "Characteristic". The Starting
 *          Handle is set to starting handle of the specified service and
 *          the Ending Handle is set to the ending handle of the specified
 *          service.
 *
 *          If the return status from this function is SUCCESS, the calling
 *          application task will receive multiple OSAL GATT_MSG_EVENT messages.
 *          The type of the messages will be either ATT_READ_BY_TYPE_RSP
 *          or ATT_ERROR_RSP (if an error occurred on the server).
 *
 *          Note: This sub-procedure is complete when either ATT_READ_BY_TYPE_RSP
 *                (with bleProcedureComplete or bleTimeout status) or ATT_ERROR_RSP
 *                (with SUCCESS status) is received by the calling application task.
 *
 * @param   connHandle - connection to use
 * @param   startHandle - starting handle
 * @param   endHandle - end handle
 * @param   taskId - task to be notified of response
 *
 * @return  SUCCESS: Request was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid connection handle or request field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          blePending: A response is pending with this server.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 *          bleTimeout: Previous transaction timed out.<BR>
 */
 bStatus_t GATT_DiscAllChars( unsigned short int connHandle, unsigned short int startHandle,
                                    unsigned short int endHandle, unsigned char taskId )
 {
	unsigned char i = 0;
	bStatus_t status_flag = FAILURE;
#ifdef NPI_SPI
	// Fill Header and Packet Length First. Checked by CC26XX
	gGattPacket[i++] = 0xFE;        //Packet Header
	gGattPacket[i++] = 0x0A;        //Packet Length
#endif
	//
	// Command Byte
	//
	gGattPacket[i++] = COMMAND;
	//
	// Opcode Bytes (2 Bytes)
	//
	gGattPacket[i++] = (unsigned char) GATT_DISALLCHARS;
	gGattPacket[i++] = (unsigned char) (GATT_DISALLCHARS >> 8);
	//
	// Length byte
	//
	gGattPacket[i++] = 0x06;

	//
	// Connection Handle (2 Bytes)
	//
	gGattPacket[i++] = (unsigned char)connHandle;
	gGattPacket[i++] = (unsigned char)(connHandle >> 8);

	gGattPacket[i++] = (unsigned char)startHandle;
	gGattPacket[i++] = (unsigned char)(startHandle >> 8);

	gGattPacket[i++] = (unsigned char)endHandle;
	gGattPacket[i++] = (unsigned char)(endHandle >> 8);

#ifdef NPI_SPI
	gGattPacket[i++] = 0x0C; //CRC Check.-Footer
#endif

	if(i > GATT_PACKET_SIZE)
	{
        // error - GattPacket buffer overflow
		HandleError(1);
		return -1; // will never reach here
	}

	GATT_sendPacket(gGattPacket, i);

	//
	// Wait to receive Command Status event from the CC2650
	//
	status_flag =  BLEAPI_waitForSpecficEvent(GAP_HCI_EXTENTIONCOMMANDSTATUS, GATT_DISALLCHARS, TRUE);

	return status_flag;
 }
/**
 * @brief   This sub-procedure is used by a client to discover service
 *          characteristics on a server when only the service handle
 *          ranges are known and the characteristic UUID is known.
 *          The specific service may exist multiple times on a server.
 *          The characteristic being discovered is identified by the
 *          characteristic UUID.
 *
 *          The ATT Read By Type Request is used with the Attribute Type
 *          is set to the UUID for "Characteristic" and the Starting
 *          Handle and Ending Handle parameters is set to the service
 *          handle range.
 *
 *          If the return status from this function is SUCCESS, the calling
 *          application task will receive multiple OSAL GATT_MSG_EVENT messages.
 *          The type of the messages will be either ATT_READ_BY_TYPE_RSP
 *          or ATT_ERROR_RSP (if an error occurred on the server).
 *
 *          Note: This sub-procedure is complete when either ATT_READ_BY_TYPE_RSP
 *                (with bleProcedureComplete or bleTimeout status) or ATT_ERROR_RSP
 *                (with SUCCESS status) is received by the calling application task.
 *
 * @param   connHandle - connection to use
 * @param   pReq - pointer to request to be sent
 * @param   taskId - task to be notified of response
 *
 * @return  SUCCESS: Request was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid connection handle or request field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          blePending: A response is pending with this server.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 *          bleTimeout: Previous transaction timed out.<BR>
 */
 bStatus_t GATT_DiscCharsByUUID( unsigned short int connHandle, attReadByTypeReq_t *pReq, unsigned char taskId )
 {
	unsigned char j, i=0;
	bStatus_t status_flag = FAILURE;
#ifdef NPI_SPI
	// Fill Header and Packet Length First. Checked by CC26XX
	gGattPacket[i++] = 0xFE;        //Packet Header
	gGattPacket[i++] = 0x0A+pReq->type.len;        //Packet Length
#endif
	//
	// Command Byte
	//
	gGattPacket[i++] = COMMAND;
	//
	// Opcode Bytes (2 Bytes)
	//
	gGattPacket[i++] = (unsigned char) GATT_DISCCHARSBYUUID;
	gGattPacket[i++] = (unsigned char) (GATT_DISCCHARSBYUUID >> 8);
	//
	// Length byte
	//
	gGattPacket[i++] = 0x06+pReq->type.len;

	//
	// Connection Handle (2 Bytes)
	//
	gGattPacket[i++] = (unsigned char)connHandle;
	gGattPacket[i++] = (unsigned char)(connHandle >> 8);

	gGattPacket[i++] = (unsigned char)(pReq->startHandle);
	gGattPacket[i++] = (unsigned char)(pReq->startHandle >> 8);

	gGattPacket[i++] = (unsigned char)pReq->endHandle;
	gGattPacket[i++] = (unsigned char)(pReq->endHandle >> 8);

	for(j=0;j<pReq->type.len;j++)
		gGattPacket[i++] = pReq->type.uuid[j];

#ifdef NPI_SPI
	gGattPacket[i++] = 0x0C; //CRC Check.-Footer
#endif


	if(i > GATT_PACKET_SIZE)
	{
        // error - GattPacket buffer overflow
		HandleError(1);
		return -1; // will never reach here
	}

	GATT_sendPacket(gGattPacket, i);
	//
	// Wait to receive Command Status event from the CC2650
	//
	status_flag =  BLEAPI_waitForSpecficEvent(GAP_HCI_EXTENTIONCOMMANDSTATUS, GATT_DISCCHARSBYUUID, TRUE);

	return status_flag;
 }

/**
 * @brief   This sub-procedure is used by a client to find all the
 *          characteristic descriptor’s Attribute Handles and Attribute
 *          Types within a characteristic definition when only the
 *          characteristic handle range is known. The characteristic
 *          specified is identified by the characteristic handle range.
 *
 *          The ATT Find Information Request is used with the Starting
 *          Handle set to starting handle of the specified characteristic
 *          and the Ending Handle set to the ending handle of the specified
 *          characteristic. The UUID Filter parameter is NULL (zero length).
 *
 *          If the return status from this function is SUCCESS, the calling
 *          application task will receive multiple OSAL GATT_MSG_EVENT messages.
 *          The type of the messages will be either ATT_FIND_INFO_RSP or
 *          ATT_ERROR_RSP (if an error occurred on the server).
 *
 *          Note: This sub-procedure is complete when either ATT_FIND_INFO_RSP
 *                (with bleProcedureComplete or bleTimeout status) or ATT_ERROR_RSP
 *                (with SUCCESS status) is received by the calling application task.
 *
 * @param   connHandle - connection to use
 * @param   startHandle - starting handle
 * @param   endHandle - end handle
 * @param   taskId - task to be notified of response
 *
 * @return  SUCCESS: Request was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid connection handle or request field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          blePending: A response is pending with this server.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 *          bleTimeout: Previous transaction timed out.<BR>
 */
 bStatus_t GATT_DiscAllCharDescs( unsigned short int connHandle, unsigned short int startHandle,
                                        unsigned short int endHandle, unsigned char taskId )
 {
	unsigned char i = 0;
	bStatus_t status_flag = FAILURE;
#ifdef NPI_SPI
	// Fill Header and Packet Length First. Checked by CC26XX
	gGattPacket[i++] = 0xFE;        //Packet Header
	gGattPacket[i++] = 0x0A;        //Packet Length
#endif
	//
	// Command Byte
	//
	gGattPacket[i++] = COMMAND;
	//
	// Opcode Bytes (2 Bytes)
	//
	gGattPacket[i++] = (unsigned char) GATT_DISCALLCHARDESCS;
	gGattPacket[i++] = (unsigned char) (GATT_DISCALLCHARDESCS >> 8);
	//
	// Length byte
	//
	gGattPacket[i++] = 0x06;

	//
	// Connection Handle (2 Bytes)
	//
	gGattPacket[i++] = (unsigned char)connHandle;
	gGattPacket[i++] = (unsigned char)(connHandle >> 8);

	gGattPacket[i++] = (unsigned char)startHandle;
	gGattPacket[i++] = (unsigned char)(startHandle >> 8);

	gGattPacket[i++] = (unsigned char)endHandle;
	gGattPacket[i++] = (unsigned char)(endHandle >> 8);

#ifdef NPI_SPI
	gGattPacket[i++] = 0x0C; //CRC Check.-Footer
#endif

	if(i > GATT_PACKET_SIZE)
	{
        // error - GattPacket buffer overflow
		HandleError(1);
		return -1; // will never reach here
	}

	GATT_sendPacket(gGattPacket, i);

	//
	// Wait to receive Command Status event from the CC2650
	//
	status_flag =  BLEAPI_waitForSpecficEvent(GAP_HCI_EXTENTIONCOMMANDSTATUS, GATT_DISCALLCHARDESCS, TRUE);

	return status_flag;
 }
/**
 * @brief   This sub-procedure is used to read a Characteristic Value
 *          from a server when the client knows the Characteristic Value
 *          Handle. The ATT Read Request is used with the Attribute Handle
 *          parameter set to the Characteristic Value Handle. The Read
 *          Response returns the Characteristic Value in the Attribute
 *          Value parameter.
 *
 *          The Read Response only contains a Characteristic Value that
 *          is less than or equal to (ATT_MTU – 1) octets in length. If
 *          the Characteristic Value is greater than (ATT_MTU – 1) octets
 *          in length, the Read Long Characteristic Value procedure may
 *          be used if the rest of the Characteristic Value is required.
 *
 *          If the return status from this function is SUCCESS, the calling
 *          application task will receive an OSAL GATT_MSG_EVENT message.
 *          The type of the message will be either ATT_READ_RSP or
 *          ATT_ERROR_RSP (if an error occurred on the server).
 *
 *          Note: This sub-procedure is complete when either ATT_READ_RSP
 *                (with SUCCESS or bleTimeout status) or ATT_ERROR_RSP (with
 *                SUCCESS status) is received by the calling application task.
 *
 * @param   connHandle - connection to use
 * @param   pReq - pointer to request to be sent
 * @param   taskId - task to be notified of response
 *
 * @return  SUCCESS: Request was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid connection handle or request field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          blePending: A response is pending with this server.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 *          bleTimeout: Previous transaction timed out.<BR>
 */
 bStatus_t GATT_ReadCharValue( unsigned short int connHandle, attReadReq_t *pReq, unsigned char taskId )
 {
	unsigned char i = 0;
	bStatus_t status_flag = FAILURE;
#ifdef NPI_SPI
	// Fill Header and Packet Length First. Checked by CC26XX
	gGattPacket[i++] = 0xFE;        //Packet Header
	gGattPacket[i++] = 0x08;        //Packet Length
#endif
	//
	// Command Byte
	//
	gGattPacket[i++] = COMMAND;
	//
	// Opcode Bytes (2 Bytes)
	//
	gGattPacket[i++] = (unsigned char) GATT_READCHARVALUE;
	gGattPacket[i++] = (unsigned char) (GATT_READCHARVALUE >> 8);
	//
	// Length byte
	//
	gGattPacket[i++] = 0x04;

	//
	// Connection Handle (2 Bytes)
	//
	gGattPacket[i++] = (unsigned char)connHandle;
	gGattPacket[i++] = (unsigned char)(connHandle >> 8);

	gGattPacket[i++] = (unsigned char)(pReq->handle);
	gGattPacket[i++] = (unsigned char)(pReq->handle >> 8);

#ifdef NPI_SPI
	gGattPacket[i++] = 0x0C; //CRC Check.-Footer
#endif

	if(i > GATT_PACKET_SIZE)
	{
        // error - GattPacket buffer overflow
		HandleError(1);
		return -1; // will never reach here
	}

	GATT_sendPacket(gGattPacket, i);
	//
	// Wait to receive Command Status event from the CC2650
	//
	status_flag =  BLEAPI_waitForSpecficEvent(GAP_HCI_EXTENTIONCOMMANDSTATUS, GATT_READCHARVALUE, TRUE);

	return status_flag;
 }

/**
 * @brief   This sub-procedure is used to read a Characteristic Value
 *          from a server when the client only knows the characteristic
 *          UUID and does not know the handle of the characteristic.
 *
 *          The ATT Read By Type Request is used to perform the sub-procedure.
 *          The Attribute Type is set to the known characteristic UUID and
 *          the Starting Handle and Ending Handle parameters shall be set
 *          to the range over which this read is to be performed. This is
 *          typically the handle range for the service in which the
 *          characteristic belongs.
 *
 *          If the return status from this function is SUCCESS, the calling
 *          application task will receive an OSAL GATT_MSG_EVENT messages.
 *          The type of the message will be either ATT_READ_BY_TYPE_RSP
 *          or ATT_ERROR_RSP (if an error occurred on the server).
 *
 *          Note: This sub-procedure is complete when either ATT_READ_BY_TYPE_RSP
 *                (with SUCCESS or bleTimeout status) or ATT_ERROR_RSP (with
 *                SUCCESS status) is received by the calling application task.
 *
 * @param   connHandle - connection to use
 * @param   pReq - pointer to request to be sent
 * @param   taskId - task to be notified of response
 *
 * @return  SUCCESS: Request was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid connection handle or request field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          blePending: A response is pending with this server.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 *          bleTimeout: Previous transaction timed out.<BR>
 */
 bStatus_t GATT_ReadUsingCharUUID( unsigned short int connHandle, attReadByTypeReq_t *pReq, unsigned char taskId )
 {
	unsigned char j, i=0;
	bStatus_t status_flag = FAILURE;
#ifdef NPI_SPI
	// Fill Header and Packet Length First. Checked by CC26XX
	gGattPacket[i++] = 0xFE;        //Packet Header
	gGattPacket[i++] = 0x0A + pReq->type.len;        //Packet Length
#endif
	//
	// Command Byte
	//
	gGattPacket[i++] = COMMAND;
	//
	// Opcode Bytes (2 Bytes)
	//
	gGattPacket[i++] = (unsigned char) GATT_READUSINGCHARUUID;
	gGattPacket[i++] = (unsigned char) (GATT_READUSINGCHARUUID >> 8);
	//
	// Length byte
	//
	gGattPacket[i++] = 0x06 + pReq->type.len ;

	//
	// Connection Handle (2 Bytes)
	//
	gGattPacket[i++] = (unsigned char)connHandle;
	gGattPacket[i++] = (unsigned char)(connHandle >> 8);

	gGattPacket[i++] = (unsigned char)(pReq->startHandle);
	gGattPacket[i++] = (unsigned char)(pReq->startHandle >> 8);

	gGattPacket[i++] = (unsigned char)(pReq->endHandle);
	gGattPacket[i++] = (unsigned char)(pReq->endHandle >> 8);

	for(j=0;j<pReq->type.len;j++)
		gGattPacket[i++] = pReq->type.uuid[j];

#ifdef NPI_SPI
	gGattPacket[i++] = 0x0C; //CRC Check.-Footer
#endif

	if(i > GATT_PACKET_SIZE)
	{
        // error - GattPacket buffer overflow
		HandleError(1);
		return -1; // will never reach here
	}

	GATT_sendPacket(gGattPacket, i);

	//
	// Wait to receive Command Status event from the CC2650
	//
	status_flag =  BLEAPI_waitForSpecficEvent(GAP_HCI_EXTENTIONCOMMANDSTATUS, GATT_READUSINGCHARUUID, TRUE);

	return status_flag;
 }
/**
 * @brief   This sub-procedure is used to read a Characteristic Value from
 *          a server when the client knows the Characteristic Value Handle
 *          and the length of the Characteristic Value is longer than can
 *          be sent in a single Read Response Attribute Protocol message.
 *
 *          The ATT Read Blob Request is used in this sub-procedure.
 *
 *          If the return status from this function is SUCCESS, the calling
 *          application task will receive multiple OSAL GATT_MSG_EVENT messages.
 *          The type of the messages will be either ATT_READ_BLOB_RSP or
 *          ATT_ERROR_RSP (if an error occurred on the server).
 *
 *          Note: This sub-procedure is complete when either ATT_READ_BLOB_RSP
 *                (with bleProcedureComplete or bleTimeout status) or ATT_ERROR_RSP
 *                (with SUCCESS status) is received by the calling application task.
 *
 * @param   connHandle - connection to use
 * @param   pReq - pointer to request to be sent
 * @param   taskId - task to be notified of response
 *
 * @return  SUCCESS: Request was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid connection handle or request field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          blePending: A response is pending with this server.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 *          bleTimeout: Previous transaction timed out.<BR>
 */
 bStatus_t GATT_ReadLongCharValue( unsigned short int connHandle, attReadBlobReq_t *pReq, unsigned char taskId )
 {
 	 return 0;
 }

/**
 * @brief   This sub-procedure is used to read multiple Characteristic Values
 *          from a server when the client knows the Characteristic Value
 *          Handles. The Attribute Protocol Read Multiple Requests is used
 *          with the Set Of Handles parameter set to the Characteristic Value
 *          Handles. The Read Multiple Response returns the Characteristic
 *          Values in the Set Of Values parameter.
 *
 *          The ATT Read Multiple Request is used in this sub-procedure.
 *
 *          If the return status from this function is SUCCESS, the calling
 *          application task will receive an OSAL GATT_MSG_EVENT message.
 *          The type of the message will be either ATT_READ_MULTI_RSP
 *          or ATT_ERROR_RSP (if an error occurred on the server).
 *
 *          Note: This sub-procedure is complete when either ATT_READ_MULTI_RSP
 *                (with SUCCESS or bleTimeout status) or ATT_ERROR_RSP (with
 *                SUCCESS status) is received by the calling application task.
 *
 * @param   connHandle - connection to use
 * @param   pReq - pointer to request to be sent
 * @param   taskId - task to be notified of response
 *
 * @return  SUCCESS: Request was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid connection handle or request field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          blePending: A response is pending with this server.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 *          bleTimeout: Previous transaction timed out.<BR>
 */
 bStatus_t GATT_ReadMultiCharValues( unsigned short int connHandle, attReadMultiReq_t *pReq, unsigned char taskId )
 {
 	 return 0;
 }

/**
 * @brief   This sub-procedure is used to write a Characteristic Value
 *          to a server when the client knows the Characteristic Value
 *          Handle and the client does not need an acknowledgement that
 *          the write was successfully performed. This sub-procedure
 *          only writes the first (ATT_MTU – 3) octets of a Characteristic
 *          Value. This sub-procedure can not be used to write a long
 *          characteristic; instead the Write Long Characteristic Values
 *          sub-procedure should be used.
 *
 *          The ATT Write Command is used for this sub-procedure. The
 *          Attribute Handle parameter shall be set to the Characteristic
 *          Value Handle. The Attribute Value parameter shall be set to
 *          the new Characteristic Value.
 *
 *          No response will be sent to the calling application task for this
 *          sub-procedure. If the Characteristic Value write request is the
 *          wrong size, or has an invalid value as defined by the profile,
 *          then the write will not succeed and no error will be generated
 *          by the server.
 *
 * @param   connHandle - connection to use
 * @param   pReq - pointer to command to be sent
 *
 * @return  SUCCESS: Request was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid connection handle or request field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 *          bleTimeout: Previous transaction timed out.<BR>
 */
 bStatus_t GATT_WriteNoRsp( unsigned short int connHandle, attWriteReq_t *pReq )
 {
	unsigned char j, i=0;
	bStatus_t status_flag = FAILURE;
#ifdef NPI_SPI
	// Fill Header and Packet Length First. Checked by CC26XX
	gGattPacket[i++] = 0xFE;        //Packet Header
	gGattPacket[i++] = 0x0A+pReq->len;        //Packet Length
#endif
	//
	// Command Byte
	//
	gGattPacket[i++] = COMMAND;
	//
	// Opcode Bytes (2 Bytes)
	//
	gGattPacket[i++] = (unsigned char) GATT_WRITENORSP;
	gGattPacket[i++] = (unsigned char) (GATT_WRITENORSP >> 8);
	//
	// Length byte
	//
	gGattPacket[i++] = 0x06+pReq->len;

	//
	// Connection Handle (2 Bytes)
	//
	gGattPacket[i++] = (unsigned char)connHandle;
	gGattPacket[i++] = (unsigned char)(connHandle >> 8);

	// Signature
	gGattPacket[i++] = pReq->sig;

	//Command
	gGattPacket[i++] = pReq->cmd;

	gGattPacket[i++] = (unsigned char)(pReq->handle);
	gGattPacket[i++] = (unsigned char)(pReq->handle >> 8);

	for(j=0;j<pReq->len;j++)
		gGattPacket[i++] = pReq->value[j];

#ifdef NPI_SPI
	gGattPacket[i++] = 0x0C; //CRC Check.-Footer
#endif

	if(i > GATT_PACKET_SIZE)
	{
        // error - GattPacket buffer overflow
		HandleError(1);
		return -1; // will never reach here
	}

	GATT_sendPacket(gGattPacket, i);
	//
	// Wait to receive Command Status event from the CC2650
	//
	status_flag =  BLEAPI_waitForSpecficEvent(GAP_HCI_EXTENTIONCOMMANDSTATUS, GATT_WRITENORSP, TRUE);

	return status_flag;
 }

/**
 * @brief   This sub-procedure is used to write a Characteristic Value
 *          to a server when the client knows the Characteristic Value
 *          Handle and the ATT Bearer is not encrypted. This sub-procedure
 *          shall only be used if the Characteristic Properties authenticated
 *          bit is enabled and the client and server device share a bond as
 *          defined in the GAP.
 *
 *          This sub-procedure only writes the first (ATT_MTU – 15) octets
 *          of an Attribute Value. This sub-procedure cannot be used to
 *          write a long Attribute.
 *
 *          The ATT Write Command is used for this sub-procedure. The
 *          Attribute Handle parameter shall be set to the Characteristic
 *          Value Handle. The Attribute Value parameter shall be set to
 *          the new Characteristic Value authenticated by signing the
 *          value, as defined in the Security Manager.
 *
 *          No response will be sent to the calling application task for this
 *          sub-procedure. If the authenticated Characteristic Value that is
 *          written is the wrong size, or has an invalid value as defined by
 *          the profile, or the signed value does not authenticate the client,
 *          then the write will not succeed and no error will be generated by
 *          the server.
 *
 * @param   connHandle - connection to use
 * @param   pReq - pointer to command to be sent
 *
 * @return  SUCCESS: Request was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid connection handle or request field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 *          bleLinkEncrypted: Connection is already encrypted.<BR>
 *          bleTimeout: Previous transaction timed out.<BR>
 */
 bStatus_t GATT_SignedWriteNoRsp( unsigned short int connHandle, attWriteReq_t *pReq )
 {
 	 return 0;
 }

/**
 * @brief   This sub-procedure is used to write a characteristic value
 *          to a server when the client knows the characteristic value
 *          handle. This sub-procedure only writes the first (ATT_MTU-3)
 *          octets of a characteristic value. This sub-procedure can not
 *          be used to write a long attribute; instead the Write Long
 *          Characteristic Values sub-procedure should be used.
 *
 *          The ATT Write Request is used in this sub-procedure. The
 *          Attribute Handle parameter shall be set to the Characteristic
 *          Value Handle. The Attribute Value parameter shall be set to
 *          the new characteristic.
 *
 *          If the return status from this function is SUCCESS, the calling
 *          application task will receive an OSAL GATT_MSG_EVENT message.
 *          The type of the message will be either ATT_WRITE_RSP
 *          or ATT_ERROR_RSP (if an error occurred on the server).
 *
 *          Note: This sub-procedure is complete when either ATT_WRITE_RSP
 *                (with SUCCESS or bleTimeout status) or ATT_ERROR_RSP (with
 *                SUCCESS status) is received by the calling application task.
 *
 * @param   connHandle - connection to use
 * @param   pReq - pointer to request to be sent
 * @param   taskId - task to be notified of response
 *
 * @return  SUCCESS: Request was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid connection handle or request field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          blePending: A response is pending with this server.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 *          bleTimeout: Previous transaction timed out.<BR>
 */
 bStatus_t GATT_WriteCharValue( unsigned short int connHandle, attWriteReq_t *pReq, unsigned char taskId )
 {
	unsigned char j, i=0;
	bStatus_t status_flag = FAILURE;
#ifdef NPI_SPI
	// Fill Header and Packet Length First. Checked by CC26XX
	gGattPacket[i++] = 0xFE;        //Packet Header
	gGattPacket[i++] = 0x0A+pReq->len;        //Packet Length
#endif
	//
	// Command Byte
	//
	gGattPacket[i++] = COMMAND;
	//
	// Opcode Bytes (2 Bytes)
	//
	gGattPacket[i++] = (unsigned char) GATT_WRITECHARVALUE;
	gGattPacket[i++] = (unsigned char) (GATT_WRITECHARVALUE >> 8);
	//
	// Length byte
	//
	gGattPacket[i++] = 0x04+pReq->len;

	//
	// Connection Handle (2 Bytes)
	//
	gGattPacket[i++] = (unsigned char)connHandle;
	gGattPacket[i++] = (unsigned char)(connHandle >> 8);

	gGattPacket[i++] = (unsigned char)(pReq->handle);
	gGattPacket[i++] = (unsigned char)(pReq->handle >> 8);

	for(j=0;j<pReq->len;j++)
		gGattPacket[i++] = pReq->value[j];

#ifdef NPI_SPI
	gGattPacket[i++] = 0x0C; //CRC Check.-Footer
#endif

	if(i > GATT_PACKET_SIZE)
	{
        // error - GattPacket buffer overflow
		HandleError(1);
		return -1; // will never reach here
	}

	GATT_sendPacket(gGattPacket, i);

	//
	// Wait to receive Command Status event from the CC2650
	//
	status_flag =  BLEAPI_waitForSpecficEvent(GAP_HCI_EXTENTIONCOMMANDSTATUS, GATT_WRITECHARVALUE, TRUE);

	return status_flag;
 }

/**
 * @brief   This sub-procedure is used to write a Characteristic Value to
 *          a server when the client knows the Characteristic Value Handle
 *          but the length of the Characteristic Value is longer than can
 *          be sent in a single Write Request Attribute Protocol message.
 *
 *          The ATT Prepare Write Request and Execute Write Request are
 *          used to perform this sub-procedure.
 *
 *          If the return status from this function is SUCCESS, the calling
 *          application task will receive multiple OSAL GATT_MSG_EVENT messages.
 *          The type of the messages will be either ATT_PREPARE_WRITE_RSP,
 *          ATT_EXECUTE_WRITE_RSP or ATT_ERROR_RSP (if an error occurred on
 *          the server).
 *
 *          Note: This sub-procedure is complete when either ATT_PREPARE_WRITE_RSP
 *                (with bleTimeout status), ATT_EXECUTE_WRITE_RSP (with SUCCESS
 *                or bleTimeout status), or ATT_ERROR_RSP (with SUCCESS status)
 *                is received by the calling application task.
 *
 *          Note: The 'pReq->pValue' pointer will be freed when the sub-procedure
 *                is complete.
 *
 * @param   connHandle - connection to use
 * @param   pReq - pointer to request to be sent
 * @param   taskId - task to be notified of response
 *
 * @return  SUCCESS: Request was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid connection handle or request field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          blePending: A response is pending with this server.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 *          bleTimeout: Previous transaction timed out.<BR>
 */
 bStatus_t GATT_WriteLongCharValue( unsigned short int connHandle, gattPrepareWriteReq_t *pReq, unsigned char taskId )
 {
 	 return 0;
 }

/**
 * @brief   This sub-procedure is used to write a Characteristic Value to
 *          a server when the client knows the Characteristic Value Handle,
 *          and assurance is required that the correct Characteristic Value
 *          is going to be written by transferring the Characteristic Value
 *          to be written in both directions before the write is performed.
 *          This sub-procedure can also be used when multiple values must
 *          be written, in order, in a single operation.
 *
 *          The sub-procedure has two phases, the first phase prepares the
 *          characteristic values to be written.  Once this is complete,
 *          the second phase performs the execution of all of the prepared
 *          characteristic value writes on the server from this client.
 *
 *          In the first phase, the ATT Prepare Write Request is used.
 *          In the second phase, the attribute protocol Execute Write
 *          Request is used.
 *
 *          If the return status from this function is SUCCESS, the calling
 *          application task will receive multiple OSAL GATT_MSG_EVENT messages.
 *          The type of the messages will be either ATT_PREPARE_WRITE_RSP,
 *          ATT_EXECUTE_WRITE_RSP or ATT_ERROR_RSP (if an error occurred on
 *          the server).
 *
 *          Note: This sub-procedure is complete when either ATT_PREPARE_WRITE_RSP
 *                (with bleTimeout status), ATT_EXECUTE_WRITE_RSP (with SUCCESS
 *                or bleTimeout status), or ATT_ERROR_RSP (with SUCCESS status)
 *                is received by the calling application task.
 *
 *          Note: The 'pReqs' pointer will be freed when the sub-procedure is
 *                complete.
 *
 * @param   connHandle - connection to use
 * @param   pReqs - pointer to requests to be sent (must be allocated)
 * @param   numReqs - number of requests in pReq
 * @param   flags - execute write request flags
 * @param   taskId - task to be notified of response
 *
 * @return  SUCCESS: Request was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid connection handle or request field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          blePending: A response is pending with this server.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 *          bleTimeout: Previous transaction timed out.<BR>
 */
 bStatus_t GATT_ReliableWrites( unsigned short int connHandle, attPrepareWriteReq_t *pReqs,
                                      unsigned char numReqs, unsigned char flags, unsigned char taskId )
 {
 	 return 0;
 }
/**
 * @brief   This sub-procedure is used to read a characteristic descriptor
 *          from a server when the client knows the characteristic descriptor
 *          declaration’s Attribute handle.
 *
 *          The ATT Read Request is used for this sub-procedure. The Read
 *          Request is used with the Attribute Handle parameter set to the
 *          characteristic descriptor handle. The Read Response returns the
 *          characteristic descriptor value in the Attribute Value parameter.
 *
 *          If the return status from this function is SUCCESS, the calling
 *          application task will receive an OSAL GATT_MSG_EVENT message.
 *          The type of the message will be either ATT_READ_RSP or
 *          ATT_ERROR_RSP (if an error occurred on the server).
 *
 *          Note: This sub-procedure is complete when either ATT_READ_RSP
 *                (with SUCCESS or bleTimeout status) or ATT_ERROR_RSP (with
 *                SUCCESS status) is received by the calling application task.
 *
 * @param   connHandle - connection to use
 * @param   pReq - pointer to request to be sent
 * @param   taskId - task to be notified of response
 *
 * @return  SUCCESS: Request was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid connection handle or request field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          blePending: A response is pending with this server.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 *          bleTimeout: Previous transaction timed out.<BR>
 */
 bStatus_t GATT_ReadCharDesc( unsigned short int connHandle, attReadReq_t *pReq, unsigned char taskId )
 {
 	 return 0;
 }

/**
 * @brief   This sub-procedure is used to read a characteristic descriptor
 *          from a server when the client knows the characteristic descriptor
 *          declaration’s Attribute handle and the length of the characteristic
 *          descriptor declaration is longer than can be sent in a single Read
 *          Response attribute protocol message.
 *
 *          The ATT Read Blob Request is used to perform this sub-procedure.
 *          The Attribute Handle parameter shall be set to the characteristic
 *          descriptor handle. The Value Offset parameter shall be the offset
 *          within the characteristic descriptor to be read.
 *
 *          If the return status from this function is SUCCESS, the calling
 *          application task will receive multiple OSAL GATT_MSG_EVENT messages.
 *          The type of the messages will be either ATT_READ_BLOB_RSP or
 *          ATT_ERROR_RSP (if an error occurred on the server).
 *
 *          Note: This sub-procedure is complete when either ATT_READ_BLOB_RSP
 *                (with bleProcedureComplete or bleTimeout status) or ATT_ERROR_RSP
 *                (with SUCCESS status) is received by the calling application task.
 *
 * @param   connHandle - connection to use
 * @param   pReq - pointer to request to be sent
 * @param   taskId - task to be notified of response
 *
 * @return  SUCCESS: Request was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid connection handle or request field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          blePending: A response is pending with this server.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 *          bleTimeout: Previous transaction timed out.<BR>
 */
 bStatus_t GATT_ReadLongCharDesc( unsigned short int connHandle, attReadBlobReq_t *pReq, unsigned char taskId )
 {
 	 return 0;
 }

/**
 * @brief   This sub-procedure is used to write a characteristic
 *          descriptor value to a server when the client knows the
 *          characteristic descriptor handle.
 *
 *          The ATT Write Request is used for this sub-procedure. The
 *          Attribute Handle parameter shall be set to the characteristic
 *          descriptor handle. The Attribute Value parameter shall be
 *          set to the new characteristic descriptor value.
 *
 *          If the return status from this function is SUCCESS, the calling
 *          application task will receive an OSAL GATT_MSG_EVENT message.
 *          The type of the message will be either ATT_WRITE_RSP
 *          or ATT_ERROR_RSP (if an error occurred on the server).
 *
 *          Note: This sub-procedure is complete when either ATT_WRITE_RSP
 *                (with SUCCESS or bleTimeout status) or ATT_ERROR_RSP (with
 *                SUCCESS status) is received by the calling application task.
 *
 * @param   connHandle - connection to use
 * @param   pReq - pointer to request to be sent
 * @param   taskId - task to be notified of response
 *
 * @return  SUCCESS: Request was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid connection handle or request field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          blePending: A response is pending with this server.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 *          bleTimeout: Previous transaction timed out.<BR>
 */
 bStatus_t GATT_WriteCharDesc( unsigned short int connHandle, attWriteReq_t *pReq, unsigned char taskId )
 {
 	 return 0;
 }

/**
 * @brief   This sub-procedure is used to write a Characteristic Value to
 *          a server when the client knows the Characteristic Value Handle
 *          but the length of the Characteristic Value is longer than can
 *          be sent in a single Write Request Attribute Protocol message.
 *
 *          The ATT Prepare Write Request and Execute Write Request are
 *          used to perform this sub-procedure.
 *
 *          If the return status from this function is SUCCESS, the calling
 *          application task will receive multiple OSAL GATT_MSG_EVENT messages.
 *          The type of the messages will be either ATT_PREPARE_WRITE_RSP,
 *          ATT_EXECUTE_WRITE_RSP or ATT_ERROR_RSP (if an error occurred on
 *          the server).
 *
 *          Note: This sub-procedure is complete when either ATT_PREPARE_WRITE_RSP
 *                (with bleTimeout status), ATT_EXECUTE_WRITE_RSP (with SUCCESS
 *                or bleTimeout status), or ATT_ERROR_RSP (with SUCCESS status)
 *                is received by the calling application task.
 *
 *          Note: The 'pReq->pValue' pointer will be freed when the sub-procedure
 *                is complete.
 *
 * @param   connHandle - connection to use
 * @param   pReq - pointer to request to be sent
 * @param   taskId - task to be notified of response
 *
 * @return  SUCCESS: Request was sent successfully.<BR>
 *          INVALIDPARAMETER: Invalid connection handle or request field.v
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          blePending: A response is pending with this server.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 *          bleTimeout: Previous transaction timed out.<BR>
 */
 bStatus_t GATT_WriteLongCharDesc( unsigned short int connHandle, gattPrepareWriteReq_t *pReq, unsigned char taskId )
 {
 	 return 0;
 }

/**
 * @}
 */

/*-------------------------------------------------------------------
 * GATT Flow Control APIs
 */

/**
 * @defgroup GATT_FLOW_CTRL_API GATT Flow Control API Functions
 *
 * @{
 */

/**
 * @brief   This API is used by the Application to turn flow control on
 *          or off for GATT messages sent from the Host to the Application.
 *
 *          Note: If the flow control is enabled then the Application must
 *                call the GATT_AppCompletedMsg() API when it completes
 *                processing an incoming GATT message.
 *
 * @param   heapSize - internal heap size
 * @param   flowCtrlMode – flow control mode: TRUE or FALSE
 *
 * @return  void
 */
 void GATT_SetHostToAppFlowCtrl( unsigned short int heapSize, unsigned char flowCtrlMode )
 {

 }

/**
 * @brief   This API is used by the Application to notify GATT that
 *          the processing of a message has been completed.
 *
 * @param   pMsg – pointer to the processed GATT message
 *
 * @return  void
 */
 void GATT_AppCompletedMsg( gattMsgEvent_t *pMsg )
 {

 }

/**
 * @}
 */

/*-------------------------------------------------------------------
 * Internal API - This function is only called from GATT Qualification modules.
 */

  /**
   * @internal
   *
   * @brief       Set the next available attribute handle.
   *
   * @param       handle - next attribute handle.
   *
   * @return      none
   */
 void GATT_SetNextHandle( unsigned short int handle )
 {

 }

/*-------------------------------------------------------------------
 * TASK API - These functions must only be called by OSAL.
 */

/**
 * @internal
 *
 * @brief   GATT Task initialization function.
 *
 * @param   taskId - GATT task ID.
 *
 * @return  void
 */
 void GATT_Init( unsigned char taskId )
 {

 }

/**
 * @internal
 *
 * @brief   GATT Task event processing function.
 *
 * @param   taskId - GATT task ID
 * @param   events - GATT events.
 *
 * @return  events not processed
 */
 unsigned short int GATT_ProcessEvent( unsigned char taskId, unsigned short int events )
 {
	 return 0;
 }
