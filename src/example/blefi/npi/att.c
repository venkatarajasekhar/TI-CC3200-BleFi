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
// Include Library Header Files
#include "att.h"
#include "hci.h"

#include "spi_def.h"


// Local/Global Variables
extern unsigned char*   gSPIpacket;
extern unsigned char    gSPIpacketSize;

extern unsigned char  gOutGoingPacket;
unsigned char gAttPacket[45];


/*-------------------------------------------------------------------
 *  General Utility APIs
 */

/*
 * Parse an attribute protocol message.
 */
 unsigned char ATT_ParsePacket( l2capDataEvent_t *pL2capMsg, attPacket_t *pPkt )
 {
	 return 0;
 }
/*
 * Compare two UUIDs. The UUIDs are converted if necessary.
 */
 unsigned char ATT_CompareUUID( const unsigned char *pUUID1, unsigned short int len1,
                              const unsigned char *pUUID2, unsigned short int len2 )
 {
	 return 0;
 }
/*
 * Convert a 16-bit UUID to 128-bit UUID.
 */
 unsigned char ATT_ConvertUUIDto128( const unsigned char *pUUID16, unsigned char *pUUID128 )
 {
	 return 0;
 }

/*
 * Convert a 128-bit UUID to 16-bit UUID.
 */
 unsigned char ATT_ConvertUUIDto16( const unsigned char *pUUID128, unsigned char *pUUID16 )
 {
	 return 0;
 }

/*-------------------------------------------------------------------
 *  Attribute Client Utility APIs
 */

/*
 * Build Error Response.
 */
 unsigned short int ATT_BuildErrorRsp( unsigned char *pBuf, unsigned char *pMsg )
 {
	 return 0;
 }
/*
 * Parse Error Response.
 */
 bStatus_t ATT_ParseErrorRsp( unsigned char *pParams, unsigned short int len, attMsg_t *pMsg )
 {
	 return 0;
 }
/*
 * Build Exchange MTU Request.
 */
 unsigned short int ATT_BuildExchangeMTUReq( unsigned char *pBuf, unsigned char *pMsg )
 {
	 return 0;
 }
/*
 * Build Exchange MTU Respnose.
 */
 unsigned short int ATT_BuildExchangeMTURsp( unsigned char *pBuf, unsigned char *pMsg )
 {
	 return 0;
 }
/*
 * Parse Exchange MTU Response.
 */
 bStatus_t ATT_ParseExchangeMTURsp( unsigned char *pParams, unsigned short int len, attMsg_t *pMsg )
 {
	 return 0;
 }
/*
 * Build Find Information Request.
 */
 unsigned short int ATT_BuildFindInfoReq( unsigned char *pBuf, unsigned char *pMsg )
 {
	 return 0;
 }
/*
 * Parse Find Information Response.
 */
 bStatus_t ATT_ParseFindInfoRsp( unsigned char *pParams, unsigned short int len, attMsg_t *pMsg )
 {
	 return 0;
 }
/*
 * Build Find Information Response.
 */
 unsigned short int ATT_BuildFindInfoRsp( unsigned char *pBuf, unsigned char *pMsg )
 {
	 return 0;
 }
/*
 * Build Find By Type Value Request.
 */
 unsigned short int ATT_BuildFindByTypeValueReq( unsigned char *pBuf, unsigned char *pMsg )
 {
	 return 0;
 }
/*
 * Build Find By Type Value Response.
 */
 unsigned short int ATT_BuildFindByTypeValueRsp( unsigned char *pBuf, unsigned char *pMsg )
 {
	 return 0;
 }
/*
 * Parse Find By Type Value Response.
 */
 bStatus_t ATT_ParseFindByTypeValueRsp( unsigned char *pParams, unsigned short int len, attMsg_t *pMsg )
 {
	 return 0;
 }
/*
 * Build Read By Type Request.
 */
 unsigned short int ATT_BuildReadByTypeReq( unsigned char *pBuf, unsigned char *pMsg )
 {
	 return 0;
 }
/*
 * Build Read By Type Response.
 */
 unsigned short int ATT_BuildReadByTypeRsp( unsigned char *pBuf, unsigned char *pMsg )
 {
	 return 0;
 }
/*
 * Parse Read By Type Response.
 */
 bStatus_t ATT_ParseReadByTypeRsp( unsigned char *pParams, unsigned short int len, attMsg_t *pMsg )
 {
	 return 0;
 }
/*
 * Build Read Request.
 */
 unsigned short int ATT_BuildReadReq( unsigned char *pBuf, unsigned char *pMsg )
 {
	 return 0;
 }
/*
 * Build Read Response.
 */
 unsigned short int ATT_BuildReadRsp( unsigned char *pBuf, unsigned char *pMsg )
 {
	 return 0;
 }
/*
 * Parse Read Response.
 */
 bStatus_t ATT_ParseReadRsp( unsigned char *pParams, unsigned short int len, attMsg_t *pMsg )
 {
	 return 0;
 }
/*
 * Build Read Blob Request.
 */
 unsigned short int ATT_BuildReadBlobReq( unsigned char *pBuf, unsigned char *pMsg )
 {
	 return 0;
 }
/*
 * Build Read Blob Response.
 */
 unsigned short int ATT_BuildReadBlobRsp( unsigned char *pBuf, unsigned char *pMsg )
 {
	 return 0;
 }
/*
 * Parse Read Blob Response.
 */
 bStatus_t ATT_ParseReadBlobRsp( unsigned char *pParams, unsigned short int len, attMsg_t *pMsg )
 {
	 return 0;
 }
/*
 * Build Read Multiple Request.
 */
 unsigned short int ATT_BuildReadMultiReq( unsigned char *pBuf, unsigned char *pMsg )
 {
	 return 0;
 }
/*
 * Build Read Multiple Response.
 */
 unsigned short int ATT_BuildReadMultiRsp( unsigned char *pBuf, unsigned char *pMsg )
 {
	 return 0;
 }
/*
 * Parse Read Multiple Response.
 */
 bStatus_t ATT_ParseReadMultiRsp( unsigned char *pParams, unsigned short int len, attMsg_t *pMsg )
 {
	 return 0;
 }
/*
 * Build Read By Group Type Response.
 */
 unsigned short int ATT_BuildReadByGrpTypeRsp( unsigned char *pBuf, unsigned char *pMsg )
 {
	 return 0;
 }
/*
 * Parse Read By Group Type Response.
 */
 bStatus_t ATT_ParseReadByGrpTypeRsp( unsigned char *pParams, unsigned short int len, attMsg_t *pMsg )
 {
	 return 0;
 }
/*
 * Build Write Request.
 */
 unsigned short int ATT_BuildWriteReq( unsigned char *pBuf, unsigned char *pMsg )
 {
	 return 0;
 }
/*
 * Parse Write Response.
 */
 bStatus_t ATT_ParseWriteRsp( unsigned char *pParams, unsigned short int len, attMsg_t *pMsg )
 {
	 return 0;
 }
/*
 * Build Prepare Write Request.
 */
 unsigned short int ATT_BuildPrepareWriteReq( unsigned char *pBuf, unsigned char *pMsg )
 {
	 return 0;
 }
/*
 * Build Prepare Write Response.
 */
 unsigned short int ATT_BuildPrepareWriteRsp( unsigned char *pBuf, unsigned char *pMsg )
 {
	 return 0;
 }
/*
 * Parse Prepare Write Response.
 */
 bStatus_t ATT_ParsePrepareWriteRsp( unsigned char *pParams, unsigned short int len, attMsg_t *pMsg )
 {
	 return 0;
 }
/*
 * Build Execute Write Request.
 */
 unsigned short int ATT_BuildExecuteWriteReq( unsigned char *pBuf, unsigned char *pMsg )
 {
	 return 0;
 }
/*
 * Parse Execute Write Response.
 */
 bStatus_t ATT_ParseExecuteWriteRsp( unsigned char *pParams, unsigned short int len, attMsg_t *pMsg )
 {
	 return 0;
 }
/*
 * Build Handle Value Indication.
 */
 unsigned short int ATT_BuildHandleValueInd( unsigned char *pBuf, unsigned char *pMsg )
 {
	 return 0;
 }
/*
 * Parse Handle Value Indication.
 */
 bStatus_t ATT_ParseHandleValueInd( unsigned char sig, unsigned char cmd, unsigned char *pParams, unsigned short int len, attMsg_t *pMsg )
 {
	 return 0;
 }

/*-------------------------------------------------------------------
 *  Attribute Server Utility APIs
 */

/*
 * Parse Exchange MTU Request.
 */
 bStatus_t ATT_ParseExchangeMTUReq( unsigned char sig, unsigned char cmd, unsigned char *pParams, unsigned short int len, attMsg_t *pMsg )
 {
	 return 0;
 }
/*
 * Parse Find Information Request.
 */
 bStatus_t ATT_ParseFindInfoReq( unsigned char sig, unsigned char cmd, unsigned char *pParams, unsigned short int len, attMsg_t *pMsg )
 {
	 return 0;
 }
/*
 * Parse Find By Type Value Request.
 */
 bStatus_t ATT_ParseFindByTypeValueReq( unsigned char sig, unsigned char cmd, unsigned char *pParams, unsigned short int len, attMsg_t *pMsg )
 {
	 return 0;
 }
/*
 * Parse Read By Type Request.
 */
 bStatus_t ATT_ParseReadByTypeReq( unsigned char sig, unsigned char cmd, unsigned char *pParams, unsigned short int len, attMsg_t *pMsg )
 {
	 return 0;
 }
/*
 * Parse Read Request.
 */
 bStatus_t ATT_ParseReadReq( unsigned char sig, unsigned char cmd, unsigned char *pParams, unsigned short int len, attMsg_t *pMsg )
 {
	 return 0;
 }
/*
 * Parse Write Blob Request.
 */
 bStatus_t ATT_ParseReadBlobReq( unsigned char sig, unsigned char cmd, unsigned char *pParams, unsigned short int len, attMsg_t *pMsg )
 {
	 return 0;
 }
/*
 * Parse Read Multiple Request.
 */
 bStatus_t ATT_ParseReadMultiReq( unsigned char sig, unsigned char cmd, unsigned char *pParams, unsigned short int len, attMsg_t *pMsg )
 {
	 return 0;
 }
/*
 * Parse Write Request.
 */
 bStatus_t ATT_ParseWriteReq( unsigned char sig, unsigned char cmd, unsigned char *pParams, unsigned short int len, attMsg_t *pMsg )
 {
	 return 0;
 }
/*
 * Parse Execute Write Request.
 */
 bStatus_t ATT_ParseExecuteWriteReq( unsigned char sig, unsigned char cmd, unsigned char *pParams, unsigned short int len, attMsg_t *pMsg )
 {
	 return 0;
 }
/*
 * Parse Prepare Write Request.
 */
 bStatus_t ATT_ParsePrepareWriteReq( unsigned char sig, unsigned char cmd, unsigned char *pParams, unsigned short int len, attMsg_t *pMsg )
 {
	 return 0;
 }
/*
 * Parse Handle Value Confirmation.
 */
 bStatus_t ATT_ParseHandleValueCfm( unsigned char *pParams, unsigned short int len, attMsg_t *pMsg )
 {
	 return 0;
 }

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
 bStatus_t ATT_ExchangeMTUReq( unsigned short int connHandle, attExchangeMTUReq_t *pReq )
 {
	 return 0;
 }
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
 bStatus_t ATT_FindInfoReq( unsigned short int connHandle, attFindInfoReq_t *pReq )
 {
		bStatus_t status_flag = FAILURE;

		// Fill Header and Packet Length First. Checked by CC26XX
		gAttPacket[0] = 0xFE;        //Packet Header
		gAttPacket[1] = 0x0A;        //Packet Length
		//
		// gAttPacket[2] command
		//
		gAttPacket[2] = COMMAND;
		//
		// gAttPacket[3:4] = opcode
		//
		gAttPacket[3] = (unsigned char)ATT_CMD_FINDINFOREQ;
		gAttPacket[4] = (unsigned char)(ATT_CMD_FINDINFOREQ >> 8);
		//
		// gAttPacket[5] = Length byte
		//
		gAttPacket[5] = 0x06;

		//
		// gAttPacket[6:7] = Connection Handle
		//
		gAttPacket[6] = (unsigned char)connHandle;
		gAttPacket[7] = (unsigned char)(connHandle >> 8);

		//
		// gAttPacket[8:9] = Start Handle
		//
		gAttPacket[8] = (unsigned char)(pReq->startHandle);
		gAttPacket[9] = (unsigned char)(pReq->startHandle >> 8);

		//
		// gAttPacket[10:11] = End Handle
		//
		gAttPacket[10] = (unsigned char)(pReq->endHandle);
		gAttPacket[11] = (unsigned char)(pReq->endHandle >> 8);

		gAttPacket[12] = 0x0C; //CRC Check.-Footer

		gSPIpacket  	= gAttPacket;
		gSPIpacketSize  = 13;

		gOutGoingPacket = TRUE;


		SignalMasterReadyLow();

		//
		// Wait to receive Command Status event from the CC2650
		//
	    status_flag =  BLEAPI_waitForSpecficEvent(GAP_HCI_EXTENTIONCOMMANDSTATUS, ATT_CMD_FINDINFOREQ, TRUE);

		return status_flag;
 }
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
 bStatus_t ATT_FindByTypeValueReq( unsigned short int connHandle, attFindByTypeValueReq_t *pReq )
 {
	 return 0;
 }
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
 bStatus_t ATT_ReadByTypeReq( unsigned short int connHandle, attReadByTypeReq_t *pReq )
 {
	 return 0;
 }
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
 bStatus_t ATT_ReadReq( unsigned short int connHandle, attReadReq_t *pReq )
 {
		bStatus_t status_flag = FAILURE;

		// Fill Header and Packet Length First. Checked by CC26XX
		gAttPacket[0] = 0xFE;        //Packet Header
		gAttPacket[1] = 0x08;        //Packet Length
		//
		// gAttPacket[2] command
		//
		gAttPacket[2] = COMMAND;
		//
		// gAttPacket[3:4] = opcode
		//
		gAttPacket[3] = (unsigned char)ATT_CMD_READREQ;
		gAttPacket[4] = (unsigned char)(ATT_CMD_READREQ >> 8);
		//
		// gAttPacket[5] = Length byte
		//
		gAttPacket[5] = 0x04;

		//
		// gAttPacket[6:7] = Connection Handle
		//
		gAttPacket[6] = (unsigned char)connHandle;
		gAttPacket[7] = (unsigned char)(connHandle >> 8);

		//
		// gAttPacket[8:9] = Handle
		//
		gAttPacket[8] = (unsigned char)(pReq->handle);
		gAttPacket[9] = (unsigned char)(pReq->handle >> 8);


		gAttPacket[10] = 0x0C; //CRC Check.-Footer

		gSPIpacket  	= gAttPacket;
		gSPIpacketSize  = 11;

		gOutGoingPacket = TRUE;


		SignalMasterReadyLow();

		//
		// Wait to receive Command Status event from the CC2650
		//
	    status_flag =  BLEAPI_waitForSpecficEvent(GAP_HCI_EXTENTIONCOMMANDSTATUS, ATT_CMD_READREQ, TRUE);

		return status_flag;
 }
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
 bStatus_t ATT_ReadBlobReq( unsigned short int connHandle, attReadBlobReq_t *pReq )
 {
	 return 0;
 }
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
 bStatus_t ATT_ReadMultiReq( unsigned short int connHandle, attReadMultiReq_t *pReq )
 {
	 return 0;
 }
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
 bStatus_t ATT_ReadByGrpTypeReq( unsigned short int connHandle, attReadByGrpTypeReq_t *pReq )
 {
	 return 0;
 }
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
 bStatus_t ATT_WriteReq( unsigned short int connHandle, attWriteReq_t *pReq )
 {
	 	unsigned char i=0;
		bStatus_t status_flag = FAILURE;

		// Fill Header and Packet Length First. Checked by CC26XX
		gAttPacket[0] = 0xFE;        //Packet Header
		gAttPacket[1] = 0x0A+ pReq->len;        //Packet Length
		//
		// gAttPacket[2] command
		//
		gAttPacket[2] = COMMAND;
		//
		// gAttPacket[3:4] = opcode
		//
		gAttPacket[3] = (unsigned char)ATT_CMD_WRITEREQ;
		gAttPacket[4] = (unsigned char)(ATT_CMD_WRITEREQ >> 8);
		//
		// gAttPacket[5] = Length byte
		//
		gAttPacket[5] = 0x06 + pReq->len;

		//
		// gAttPacket[6:7] = Connection Handle
		//
		gAttPacket[6] = (unsigned char)connHandle;
		gAttPacket[7] = (unsigned char)(connHandle >> 8);


		// Signature
		gAttPacket[8] = pReq->sig;

		//Command
		gAttPacket[9] = pReq->cmd;

		//
		// gAttPacket[10:11] = Handle
		//
		gAttPacket[10] = (unsigned char)(pReq->handle);
		gAttPacket[11] = (unsigned char)(pReq->handle >> 8);

		for(i=0;i<=pReq->len;i++)
		{
			gAttPacket[12+i] = pReq->value[i];
		}

		gAttPacket[12+pReq->len] = 0x0C; //CRC Check.-Footer

		gSPIpacket  	= gAttPacket;
		gSPIpacketSize  = 13+pReq->len;

		gOutGoingPacket = TRUE;


		SignalMasterReadyLow();

		//
		// Wait to receive Command Status event from the CC2650
		//
	    status_flag =  BLEAPI_waitForSpecficEvent(GAP_HCI_EXTENTIONCOMMANDSTATUS, ATT_CMD_WRITEREQ, TRUE);

		return status_flag;
 }
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
 bStatus_t ATT_PrepareWriteReq( unsigned short int connHandle, attPrepareWriteReq_t *pReq )
 {
	 return 0;
 }
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
 bStatus_t ATT_ExecuteWriteReq( unsigned short int connHandle, attExecuteWriteReq_t *pReq )
 {
	 return 0;
 }
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
 bStatus_t ATT_HandleValueCfm( unsigned short int connHandle )
 {
	 return 0;
 }
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
 bStatus_t ATT_ErrorRsp( unsigned short int connHandle, attErrorRsp_t *pRsp )
 {
	bStatus_t status_flag = FAILURE;

	// Fill Header and Packet Length First. Checked by CC26XX
	gAttPacket[0] = 0xFE;        //Packet Header
	gAttPacket[1] = 0x0A;        //Packet Length
	//
	// gAttPacket[2] command
	//
	gAttPacket[2] = COMMAND;
	//
	// gAttPacket[3:4] = opcode
	//
	gAttPacket[3] = (unsigned char)ATT_CMD_ERRORRSP;
	gAttPacket[4] = (unsigned char)(ATT_CMD_ERRORRSP >> 8);
	//
	// gAttPacket[5] = Length byte
	//
	gAttPacket[5] = 0x06;

	//
	// gAttPacket[6:7] = Connection Handle
	//
	gAttPacket[6] = (unsigned char)connHandle;
	gAttPacket[7] = (unsigned char)(connHandle >> 8);

	//
	// gAttPacket[8] = reqOpcode
	//
	gAttPacket[8] = pRsp->reqOpcode;

	//
	// gAttPacket[9:10] = Handle
	//
	gAttPacket[9] = (unsigned char)(pRsp->handle);
	gAttPacket[10] = (unsigned char)(pRsp->handle >> 8);

	//
	// gAttPacket[11] = errCode
	//
	gAttPacket[11] = pRsp->errCode;

	gAttPacket[12] = 0x0C; //CRC Check.-Footer

	gSPIpacket  	= gAttPacket;
	gSPIpacketSize  = 13;

	gOutGoingPacket = TRUE;


	SignalMasterReadyLow();

	//
	// Wait to receive Command Status event from the CC2650
	//
    status_flag =  BLEAPI_waitForSpecficEvent(GAP_HCI_EXTENTIONCOMMANDSTATUS, ATT_CMD_ERRORRSP, TRUE);

	return status_flag;
 }
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
 bStatus_t ATT_ExchangeMTURsp( unsigned short int connHandle, attExchangeMTURsp_t *pRsp )
 {
	 return 0;
 }

#if 0
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
 bStatus_t ATT_FindInfoRsp( unsigned short int connHandle, attFindInfoRsp_t *pRsp )
 {
	 unsigned char i,j = 0;
	 	unsigned char multiplyFactor  = 1;
		bStatus_t status_flag = FAILURE;

		if( pRsp->format == 0x01)
			multiplyFactor = 4;
		else
			multiplyFactor = 18;

		// Fill Header and Packet Length First. Checked by CC26XX
		gAttPacket[0] = 0xFE;        //Packet Header
		gAttPacket[1] = 0x07 + (multiplyFactor*pRsp->numInfo);        //Packet Length
		//
		// gAttPacket[2] command
		//
		gAttPacket[2] = COMMAND;
		//
		// gAttPacket[3:4] = opcode
		//
		gAttPacket[3] = (unsigned char)ATT_CMD_FINDINFORSP;
		gAttPacket[4] = (unsigned char)(ATT_CMD_FINDINFORSP >> 8);
		//
		// gAttPacket[5] = Length byte
		//
		gAttPacket[5] = 0x03 + (multiplyFactor*pRsp->numInfo);

		//
		// gAttPacket[6:7] = Connection Handle
		//
		gAttPacket[6] = (unsigned char)connHandle;
		gAttPacket[7] = (unsigned char)(connHandle >> 8);

		//
		// gAttPacket[8] = format
		//
		gAttPacket[8] = pRsp->format;

		for(i=0;i<pRsp->numInfo;i++)
		{

			if(pRsp->format == 0x01)
			{
				//
				// gAttPacket[9:10] = Handle
				//
				gAttPacket[9+i*multiplyFactor] = (unsigned char)(pRsp->info.btPair[i].handle);
				gAttPacket[10+i*multiplyFactor] = (unsigned char)(pRsp->info.btPair[i].handle >> 8);

				for(j = 0;j<2;j++)
				{
					gAttPacket[11+i*multiplyFactor + j] = (unsigned char)(pRsp->info.btPair[i].uuid[j]);
				}
			}
			else
			{
				//
				// gAttPacket[9:10] = Handle
				//
				gAttPacket[9+i*multiplyFactor] = (unsigned char)(pRsp->info.pair[i].handle);
				gAttPacket[10+i*multiplyFactor] = (unsigned char)(pRsp->info.pair[i].handle >> 8);
				for(j = 0;j<2;j++)
				{
					gAttPacket[11+i*multiplyFactor + j] = (unsigned char)(pRsp->info.pair[i].uuid[j]);
				}
			}
		}

		gAttPacket[9+(multiplyFactor*pRsp->numInfo)] =  0x0C; //CRC Check.-Footer

		gSPIpacket  	= gAttPacket;
		gSPIpacketSize  = 10+(multiplyFactor*pRsp->numInfo);

		gOutGoingPacket = TRUE;


		SignalMasterReadyLow();

		//
		// Wait to receive Command Status event from the CC2650
		//
	    status_flag =  BLEAPI_waitForSpecficEvent(GAP_HCI_EXTENTIONCOMMANDSTATUS, ATT_CMD_FINDINFORSP, TRUE);

		return status_flag;
 }
#endif
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
 bStatus_t ATT_FindByTypeValueRsp( unsigned short int connHandle, attFindByTypeValueRsp_t *pRsp )
 {
	 return 0;
 }
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
 bStatus_t ATT_ReadByTypeRsp( unsigned short int connHandle, attReadByTypeRsp_t *pRsp )
 {
	 return 0;
 }
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
 bStatus_t ATT_ReadRsp( unsigned short int connHandle, attReadRsp_t *pRsp )
 {
#if 0
	 unsigned char i = 0;
		bStatus_t status_flag = FAILURE;

		// Fill Header and Packet Length First. Checked by CC26XX
		gAttPacket[0] = 0xFE;        //Packet Header
		gAttPacket[1] = 0x06 + pRsp->len;        //Packet Length
		//
		// gAttPacket[2] command
		//
		gAttPacket[2] = COMMAND;
		//
		// gAttPacket[3:4] = opcode
		//
		gAttPacket[3] = (unsigned char)ATT_CMD_READRSP;
		gAttPacket[4] = (unsigned char)(ATT_CMD_READRSP >> 8);
		//
		// gAttPacket[5] = Length byte
		//
		gAttPacket[5] = 0x02 + (pRsp->len);

		//
		// gAttPacket[6:7] = Connection Handle
		//
		gAttPacket[6] = (unsigned char)connHandle;
		gAttPacket[7] = (unsigned char)(connHandle >> 8);


		for(i=0;i<pRsp->len;i++)
		{

				gAttPacket[8+i] = (unsigned char)(pRsp->value[i]);

		}

		gAttPacket[8 + pRsp->len] =  0x0C; //CRC Check.-Footer

		gSPIpacket  	= gAttPacket;
		gSPIpacketSize  = 9 + pRsp->len;

		gOutGoingPacket = TRUE;


		SignalMasterReadyLow();

		//
		// Wait to receive Command Status event from the CC2650
		//
	    status_flag =  BLEAPI_waitForSpecficEvent(GAP_HCI_EXTENTIONCOMMANDSTATUS, ATT_CMD_READRSP, TRUE);

		return status_flag;
#else
		return 0;
#endif
 }
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
 bStatus_t ATT_ReadBlobRsp( unsigned short int connHandle, attReadBlobRsp_t *pRsp )
 {
	 return 0;
 }
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
 bStatus_t ATT_ReadMultiRsp( unsigned short int connHandle, attReadMultiRsp_t *pRsp )
 {
	 return 0;
 }
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
 bStatus_t ATT_ReadByGrpTypeRsp( unsigned short int connHandle, attReadByGrpTypeRsp_t *pRsp )
 {
	 return 0;
 }
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
 bStatus_t ATT_WriteRsp( unsigned short int connHandle )
 {
	 	bStatus_t status_flag = FAILURE;

		// Fill Header and Packet Length First. Checked by CC26XX
		gAttPacket[0] = 0xFE;        //Packet Header
		gAttPacket[1] = 0x06;        //Packet Length
		//
		// gAttPacket[2] command
		//
		gAttPacket[2] = COMMAND;
		//
		// gAttPacket[3:4] = opcode
		//
		gAttPacket[3] = (unsigned char)ATT_CMD_WRITERSP;
		gAttPacket[4] = (unsigned char)(ATT_CMD_WRITERSP >> 8);
		//
		// gAttPacket[5] = Length byte
		//
		gAttPacket[5] = 0x02;

		//
		// gAttPacket[6:7] = Connection Handle
		//
		gAttPacket[6] = (unsigned char)connHandle;
		gAttPacket[7] = (unsigned char)(connHandle >> 8);


		gAttPacket[8] =  0x0C; //CRC Check.-Footer

		gSPIpacket  	= gAttPacket;
		gSPIpacketSize  = 9;

		gOutGoingPacket = TRUE;


		SignalMasterReadyLow();

		//
		// Wait to receive Command Status event from the CC2650
		//
	    status_flag =  BLEAPI_waitForSpecficEvent(GAP_HCI_EXTENTIONCOMMANDSTATUS, ATT_CMD_WRITERSP, TRUE);

		return status_flag;
 }
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
 bStatus_t ATT_PrepareWriteRsp( unsigned short int connHandle, attPrepareWriteRsp_t *pRsp )
 {
	 return 0;
 }
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
 bStatus_t ATT_ExecuteWriteRsp( unsigned short int connHandle )
 {
	 return 0;
 }
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
 bStatus_t ATT_HandleValueNoti( unsigned short int connHandle, attHandleValueNoti_t *pNoti )
 {
#if 0
	 unsigned char i=0;
		bStatus_t status_flag = FAILURE;

		// Fill Header and Packet Length First. Checked by CC26XX
		gAttPacket[0] = 0xFE;        //Packet Header
		gAttPacket[1] = 0x08+ pNoti->len;        //Packet Length
		//
		// gAttPacket[2] command
		//
		gAttPacket[2] = COMMAND;
		//
		// gAttPacket[3:4] = opcode
		//
		gAttPacket[3] = (unsigned char)ATT_CMD_HANDLEVALUENOTI;
		gAttPacket[4] = (unsigned char)(ATT_CMD_HANDLEVALUENOTI >> 8);
		//
		// gAttPacket[5] = Length byte
		//
		gAttPacket[5] = 0x04 + pNoti->len;

		//
		// gAttPacket[6:7] = Connection Handle
		//
		gAttPacket[6] = (unsigned char)connHandle;
		gAttPacket[7] = (unsigned char)(connHandle >> 8);

		//
		// gAttPacket[9:9] = Handle
		//
		gAttPacket[8] = (unsigned char)(pNoti->handle);
		gAttPacket[9] = (unsigned char)(pNoti->handle >> 8);

		for(i=0;i<=pNoti->len;i++)
		{
			gAttPacket[10+i] = pNoti->value[i];
		}

		gAttPacket[10+pNoti->len] = 0x0C; //CRC Check.-Footer

		gSPIpacket  	= gAttPacket;
		gSPIpacketSize  = 11+pNoti->len;

		gOutGoingPacket = TRUE;


		SignalMasterReadyLow();

		//
		// Wait to receive Command Status event from the CC2650
		//
	    status_flag =  BLEAPI_waitForSpecficEvent(GAP_HCI_EXTENTIONCOMMANDSTATUS, ATT_CMD_HANDLEVALUENOTI, TRUE);

		return status_flag;
#else
		return 0;
#endif
 }
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
 bStatus_t ATT_HandleValueInd( unsigned short int connHandle, attHandleValueInd_t *pInd )
 {
	 	unsigned char i=0;
		bStatus_t status_flag = FAILURE;

		// Fill Header and Packet Length First. Checked by CC26XX
		gAttPacket[0] = 0xFE;        //Packet Header
		gAttPacket[1] = 0x08+ pInd->len;        //Packet Length
		//
		// gAttPacket[2] command
		//
		gAttPacket[2] = COMMAND;
		//
		// gAttPacket[3:4] = opcode
		//
		gAttPacket[3] = (unsigned char)ATT_CMD_HANDLEVALUEIND;
		gAttPacket[4] = (unsigned char)(ATT_CMD_HANDLEVALUEIND >> 8);
		//
		// gAttPacket[5] = Length byte
		//
		gAttPacket[5] = 0x04 + pInd->len;

		//
		// gAttPacket[6:7] = Connection Handle
		//
		gAttPacket[6] = (unsigned char)connHandle;
		gAttPacket[7] = (unsigned char)(connHandle >> 8);

		//
		// gAttPacket[8:9] = Handle
		//
		gAttPacket[8] = (unsigned char)(pInd->handle);
		gAttPacket[9] = (unsigned char)(pInd->handle >> 8);

		for(i=0;i<=pInd->len;i++)
		{
			gAttPacket[10+i] = pInd->value[i];
		}

		gAttPacket[10+pInd->len] = 0x0C; //CRC Check.-Footer

		gSPIpacket  	= gAttPacket;
		gSPIpacketSize  = 11+pInd->len;

		gOutGoingPacket = TRUE;


		SignalMasterReadyLow();

		//
		// Wait to receive Command Status event from the CC2650
		//
	    status_flag =  BLEAPI_waitForSpecficEvent(GAP_HCI_EXTENTIONCOMMANDSTATUS, ATT_CMD_HANDLEVALUEIND, TRUE);

		return status_flag;
 }
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
 void ATT_SetParamValue( unsigned short int value )
 {

 }

/**
 * @brief   Get a ATT Parameter value.
 *
 * @param   none
 *
 * @return  ATT Parameter value
 */
 unsigned short int ATT_GetParamValue( void )
 {
	 return 0;
 }
