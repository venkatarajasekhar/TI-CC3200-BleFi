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

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include "datatypes.h"
#include <string.h>
#include <xdc/std.h>
#include "gatt.h"
#include "hci.h"
#include "ble_central.h"
#include "osi.h"
#include "uart_if.h"
#include "gateway_api.h"
#include "gateway_cmds.h"
#include "gateway_task.h"
#include "gapbondmgr.h"

/*********************************************************************
 * MACROS
 */

#define OSI_STACK_SIZE	2048

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Entity ID globally used to check for source and/or destination of messages
extern unsigned short selfEntity;

Msg_t sMsg;

gw_context_t gw_context;

//OsiSyncObj_t g_GWScanSyncObj;

unsigned char SBL_IsDone(void);

unsigned char gSblDone = FALSE;
/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern OsiSyncObj_t g_GWGenSyncObj;
/*********************************************************************
 * LOCAL VARIABLES
 */
GC_StateCB_t pGC_StateCB;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void GatewayCentral_init(void);
static void GatewayEvent_taskFxn(void* pValue);
static void GatewayCommand_taskFxn(void* pValue);
static void GatewayCentral_processStackMsg(gapEventHdr_t **pMsg);

/*********************************************************************
 * PROFILE CALLBACKS
 */

/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern bool NPI_IsReady(void);
extern void NPI_UartEventTaskFxn(void);
extern void NPI_UartEventCreateTask(void);
extern void CC26xx_SBL_Dnld_Init(void);
extern void TurnOffLed(unsigned char LedInstance);
extern void Button2IntClear(void);
extern void Button2Init(void);
extern bool IsSimpleLinkReady(void);
int AutoScan_TimerInit(void);

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*Initialize NPI*/
void NPI_Init() {
	/* Kick off NPI task - Priority 3 */
	NPI_UartEventCreateTask();
	NPI_RegisterTask((void*) &GC_StackEventHandler_CB);
}



/*********************************************************************
 * @fn      GatewayCentral_createTask
 *
 * @brief   Task creation function for the Simple BLE Peripheral.
 *
 * @param   none
 *
 * @return  none
 */
void Gateway_Init(void) {
	int i;
	OsiReturnVal_e eRetVal;

	/*Initialize NPI*/
	NPI_Init();

	eRetVal = osi_LockObjCreate(&GC_ApiLockObj);
	LOOP_ON_ERROR(eRetVal);

	eRetVal = osi_SyncObjCreate(&gw_context.gw_GapSyncObj);
	LOOP_ON_ERROR(eRetVal);

	for (i = 0; i < MAX_APPCB_SUPPORTED; i++) {
		gw_context.fptrEvent[i] = NULL;
	}

	gwcontext_Init();

	DB_DevRecDbInit();

	eRetVal = osi_MsgQCreate(&sGC_StackMsgQueue, "GatewayStackMsgQ",
			sizeof(Msg_t), MAX_GATEWAY_MSGS);

	LOOP_ON_ERROR(eRetVal);

	eRetVal = osi_MsgQCreate(&sGC_AppMsgQueue, "GatewayAppMsgQ", sizeof(Msg_t),
			MAX_APP_MSGS);
	LOOP_ON_ERROR(eRetVal);

	eRetVal = osi_TaskCreate(GatewayEvent_taskFxn,
			(const signed char *) "GatewayEvent",
			OSI_STACK_SIZE, NULL, 2, NULL);
	LOOP_ON_ERROR(eRetVal);

	eRetVal = osi_TaskCreate(GatewayCommand_taskFxn,
				(const signed char *) "GatewayCommand",
				OSI_STACK_SIZE, NULL, 2, NULL);
	LOOP_ON_ERROR(eRetVal);



	return;
}

/*********************************************************************
 * @fn      GatewayCentral_Init
 *
 * @brief   Initialization function for the Gateway Central App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   none
 *
 * @return  none
 */
static void GatewayCentral_init(void) {
	volatile unsigned char LoopVar = 0xFF;

	// CC26xx firmware update
	CC26xx_SBL_Dnld_Init();
	gSblDone = TRUE;

	if (NPI_IsReady() != TRUE) {
		UART_PRINT("\n\r[GW] Waiting for NPI initialization ... ");
	}

	while (LoopVar) {
		if (NPI_IsReady() == TRUE) {
			break;
		}
		osi_Sleep(250);
	}

	UART_PRINT("\n\r[GW] NPI initialized.");
	UART_PRINT("\n\r[GW] Starting the BLE Stack");

	// Setup Central Profile
	{
		unsigned char scanRes = DEFAULT_MAX_SCAN_RES;

		GAPCentralRole_SetParameter(GAPCENTRALROLE_MAX_SCAN_RES,
				sizeof(unsigned char), &scanRes);
	}

	// Start the Device
	GAPCentralRole_StartDevice();

	// Gateway Init done . Now turn off LED 2 to indicate it
	TurnOffLed(LED_2);

	return;
}

/*********************************************************************
 * @fn      GatewayCentral_taskFxn
 *
 * @brief   Application task entry point for the Gateway Central.
 *
 * @param   none
 *
 * @return  events not processed
 */
static void GatewayEvent_taskFxn(void* pValue)
{
	OsiReturnVal_e eRetVal;
	Msg_t GC_StackMsg_t;
	volatile unsigned char LoopVar = 0xff;

	//simpleLink init should happen before we write to flash
	//osi_Sleep(5000);
	while(IsSimpleLinkReady()==false)
	{
		osi_Sleep(250);
	}

	Message("\n\r[GW] SimpleLink Ready, starting Gateway");
	// Initialize application
	GatewayCentral_init();
	set_gwState(GW_INIT_DONE);

	// Start Auto Scan once BLE stack is ready
	//AutoScan_TimerInit();


	// Application main loop
	while(LoopVar)
	{
		eRetVal = osi_MsgQRead(&sGC_StackMsgQueue, &GC_StackMsg_t, OSI_WAIT_FOREVER);
		if(eRetVal != OSI_OK)
		{
			Message("\n\r[GW] Central task read fail");
			continue;
		}
		{
			// Message present, process stack message
			GatewayCentral_processStackMsg(
					(gapEventHdr_t **) (&GC_StackMsg_t.pData));
			//Report("\n\rF1: %p",GC_StackMsg_t.pData);
			if (GC_StackMsg_t.pData)
			{
				//Report("\n\rF2: %p", GC_StackMsg_t.pData);
				free(GC_StackMsg_t.pData);
			}
		}
	}
}


static void GatewayCommand_taskFxn(void* pValue)
{
	OsiReturnVal_e eRetVal;
	Msg_t GC_AppMsg_t;
	volatile unsigned char LoopVar = 0xff;

	while(LoopVar)
	{
		eRetVal = osi_MsgQRead(&sGC_AppMsgQueue, &GC_AppMsg_t, OSI_WAIT_FOREVER);
		if(eRetVal != OSI_OK)
		{
			Message("\n\r[GW] Command task read fail");
			continue;
		}
		else
		{
			// Process Application message
			GatewayCentral_handleinput((cmd_hdr**) (&GC_AppMsg_t.pData));
			if (GC_AppMsg_t.pData)
			{
				free(GC_AppMsg_t.pData);
			}
		}

	}
}
/*********************************************************************
 * @fn      GatewayCentral_processStackMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void GatewayCentral_processStackMsg(gapEventHdr_t **pMsgPtr) {
	gapEventHdr_t *pMsg = *(pMsgPtr);

	switch (pMsg->event) {
	case GAP_MSG_EVENT:
		GatewayCentral_processRoleEvent((gapCentralRoleEvent_t **) pMsgPtr);
		break;
	case GATT_MSG_EVENT:
		GatewayCentral_processGattEvent((gattMsg_t **) pMsgPtr);
		break;

	default:
		break;
	}
}

/*********************************************************************
 * @fn      get_gwState
 *
 * @brief
 *
 * @return  none
 */
unsigned char get_gwState() {
	return (gw_context.state);
}

/*********************************************************************
 * @fn      set_gwState
 *
 * @brief Return updated state, or the previous state if the 'state' is wrong.
 *
 * @return  none
 */
unsigned char set_gwState(GW_STATE state) {
	if (state <= GW_DISC_DONE) {
		gw_context.state = state;
	}
	return (gw_context.state);
}

/*********************************************************************
 * @fn      set_gwCallType
 *
 * @brief Return updated state, or the previous state if the 'state' is wrong.
 *
 * @return  none
 */
GW_CALL set_gwCallType(cmd_no_e gwCmd, GW_CALL CallType) {
	switch (gwCmd) {
	case DEVICE_SCAN: {
		gw_context.gwCallType.gwScanCallType = CallType;

		return (gw_context.gwCallType.gwScanCallType);
		//break;
	}
	case DEVICE_STOPSCAN: {
		gw_context.gwCallType.gwStopScanCallType = CallType;

		return (gw_context.gwCallType.gwStopScanCallType);
		//break;
	}
	case LINK_ESTABLISH: {
		gw_context.gwCallType.gwLinkEstCallType = CallType;

		return (gw_context.gwCallType.gwLinkEstCallType);
		//break;
	}
	case LINK_TERMINATE: {
		gw_context.gwCallType.gwLinkTermCallType = CallType;

		return (gw_context.gwCallType.gwLinkTermCallType);
		//break;
	}
	default:
		return (GW_CALL_NONBLOCKING);
	}
}

/*********************************************************************
 * @fn      get_gwCallType
 *
 * @brief Return updated state, or the previous state if the 'state' is wrong.
 *
 * @return  none
 */
GW_CALL get_gwCallType(cmd_no_e gwCmd) {
	switch (gwCmd) {
	case DEVICE_SCAN:
		return (gw_context.gwCallType.gwScanCallType);

	case DEVICE_STOPSCAN:
		return (gw_context.gwCallType.gwStopScanCallType);

	case LINK_ESTABLISH:
		return (gw_context.gwCallType.gwLinkEstCallType);

	case LINK_TERMINATE:
		return (gw_context.gwCallType.gwLinkTermCallType);

	default:
		return (GW_CALL_NONBLOCKING);
	}
}

/*********************************************************************
 * @fn      get_gwDevCnt
 *
 * @brief Return updated state, or the previous state if the 'state' is wrong.
 *
 * @return  none
 */
unsigned char get_gwDevCnt() {
	return (gw_context.scanCnt);
}

/*********************************************************************
 * @fn      get_gw_devState
 *
 * @brief
 *
 * @return
 */
unsigned char get_gw_devState(unsigned char index) {
	device_db_t * tempDevRec;
	tempDevRec = gw_context.scanRespArray[index]->deviceRec;
	return (tempDevRec->dev_state);
}

/*********************************************************************
 * @fn      get_gw_devConnectionid
 *
 * @brief
 *
 * @return
 */
int get_gw_devConnectionid(unsigned char index) {
	device_db_t * tempDevRec;
	tempDevRec = gw_context.scanRespArray[index]->deviceRec;
	if (tempDevRec->dev_state == DEV_READY) {
		return (tempDevRec->connHandle);
	} else {
		return (-1);
	}
}

/*******************************************************************************
 * @fn          GatewayCentral_RegisterGC_CB
 *
 * @brief       This routine registers the GC state call back from User input layer
 *
 * input parameters
 *
 * @param       GC State Callback pointer
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void GatewayCentral_RegisterGC_CB(GC_StateCB_t pCBs) {
	pGC_StateCB = pCBs;
}

/*******************************************************************************
 * @fn          gwcontext_Init
 *
 * @brief
 *
 * @param
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void gwcontext_Init() {
	int i;
	gw_context.state = GW_NO_INIT;
	gw_context.scanCnt = 0;

	for (i = 0; i < MAX_SCAN_RESPONSE; i++) {
		if(gw_context.scanRespArray[i] != NULL)
		{
			free(gw_context.scanRespArray[i]);
			gw_context.scanRespArray[i] = NULL;
		}
	}

	for (i = 0; i < MAX_APPCB_SUPPORTED; i++) {
		// Do we need to do this twice or only once in GW_Init?
		//gw_context.fptrEvent[i] = NULL;
	}
}

/*******************************************************************************
 * @fn          gwcontext_IsScanIdValid
 *
 * @brief
 *
 * @param
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
signed int gwcontext_IsScanIdValid(unsigned int uScanIdx) {

	if(uScanIdx >= MAX_SCAN_RESPONSE )
		return (-1);

	if(gw_context.scanRespArray[uScanIdx]->deviceRec != NULL)
	{
		// Device record found . Device is already linked.
		return(-1);
	}
	else
	{
		// no Device record found . Device is not linked yet
		return(0);
	}

}


/*******************************************************************************
 * @fn          RegisterGwCB
 *
 * @brief
 *
 * input parameters
 *
 * @param
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */

int RegisterGwCB(unsigned char NotificationType, fptrCB_t fptr) {
	int i;

	switch (NotificationType) {
	case GW_EVENT_CB: {
		for (i = 0; i < MAX_APPCB_SUPPORTED; i++) {
			if (gw_context.fptrEvent[i] == NULL) {
				gw_context.fptrEvent[i] = fptr;
				break;
			}
		}
	}
		break;

	case GW_DATA_CB: {
		for (i = 0; i < MAX_APPCB_SUPPORTED; i++) {
			if (gw_context.fptrData[i] == NULL) {
				gw_context.fptrData[i] = fptr;
				break;
			}
		}
	}
		break;

	default:
		break;
	}

	return 0;
}

/*******************************************************************************
 * @fn          CallGwEventCB
 *
 * @brief
 *
 * input parameters
 *
 * @param
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void CallGwEventCB(unsigned char EventType, void* EventParams) {
	int i;
	fptrCB_t fptr;

	for (i = 0; i < MAX_APPCB_SUPPORTED; i++) {
		if (gw_context.fptrEvent[i] != NULL) {
			fptr = gw_context.fptrEvent[i];
			(*fptr)(EventType, (ErrorFlag_t) 0, EventParams);
		}
	}
}

/*******************************************************************************
 * @fn          CallGwDataCB
 *
 * @brief
 *
 * input parameters
 *
 * @param
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void CallGwDataCB(unsigned char DataType, ErrorFlag_t ErrorFlag,
		void* DataParams) {
	int i;
	fptrCB_t fptr;

	for (i = 0; i < MAX_APPCB_SUPPORTED; i++) {
		if (gw_context.fptrData[i] != NULL) {
			fptr = gw_context.fptrData[i];
			(*fptr)(DataType, ErrorFlag, DataParams);
		}
	}
}

/*******************************************************************************
 * @fn          GC_StackEventHandler_CB
 *
 * @brief
 *
 * input parameters
 *
 * @param
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void GC_StackEventHandler_CB(void *inPtr_event) {
	OsiReturnVal_e retVal;
	//post App message
	sMsg.pData = inPtr_event;

	//post in App message Queue
	retVal = osi_MsgQWrite(&sGC_StackMsgQueue, &sMsg, OSI_NO_WAIT);
	if(retVal != OSI_OK)
	{
		Report("\n\rGateway MessageQ Error %d",retVal);
		HandleError(__FILE__,__LINE__,2);
		if(sMsg.pData !=NULL)
		{
			free(sMsg.pData);
		}
	}
	return;
}

/*********************************************************************
 * @fn      Util_convertBdAddr2Str
 *
 * @brief   Convert Bluetooth address to string. Only needed when
 *          LCD display is used.
 *
 * @param   pAddr - BD address
 *
 * @return  BD address as a string
 */
char *Util_convertBdAddr2Str(unsigned char *pAddr) {
	unsigned char charCnt;
	char hex[] = "0123456789ABCDEF";
	static char str[(2 * B_ADDR_LEN) + 3];
	char *pStr = str;

	*pStr++ = '0';
	*pStr++ = 'x';

	// Start from end of addr
	pAddr += B_ADDR_LEN;

	for (charCnt = B_ADDR_LEN; charCnt > 0; charCnt--) {
		*pStr++ = hex[*--pAddr >> 4];
		*pStr++ = hex[*pAddr & 0x0F];
	}

	pStr = NULL;

	return str;
}

/*********************************************************************
 * @fn      NPI_IsReady
 *
 * @brief   Returns the Ready state of NPI layer.
 *
 * @param   none
 *
 * @return  none
 */
unsigned char SBL_IsDone() {
	return (gSblDone);
}

/*********************************************************************
 *********************************************************************/
