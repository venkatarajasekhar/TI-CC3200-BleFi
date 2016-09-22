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

//*****************************************************************************
//
//! \addtogroup mqtt_client
//! @{
//
//*****************************************************************************

// Standard includes
#include <stdlib.h>

#include "datatypes.h"
// driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "osi.h"
#include "gateway_api.h"
// simplelink includes
#include "simplelink.h"

// common interface includes
#ifndef NOTERM
#include "uart_if.h"
#endif

#include "common.h"
#include "utils.h"
#include "sl_mqtt_client.h"
#include "mqtt_app.h"

extern bool is_blefiCfgReady;
extern blefiCfg_t blefiCfgRec;
extern short mqttDevMode;

//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//*****************************************************************************
void MqttClient(void *pvParameters);
extern Wifi_GetMAC(unsigned char *);
extern unsigned long  g_ulStatus;//SimpleLink Status
static void MqttLibRecvCB(void *app_hndl, const char  *topstr, _i32 top_len, const void *payload,
                       _i32 pay_len, bool dup,unsigned char qos, bool retain);
static void MqttLibEvtCB(void *app_hndl,_i32 evt, const void *buf,_u32 len);
static void MqttLibDisconnectCB(void *app_hndl);
static void MqttFreeMsg(mqtt_gw_messages_t * RecvQue);
extern void HandleError(unsigned char *, unsigned int, unsigned char error);


/*Message Queue*/
OsiMsgQ_t g_Mqtt_queue;

/*Defining the topics to which the BleFi Gateway context subscribes
 * The macaddr01234 is a placeholder, it will be replaced later during the execution
 * once the MAC address of the WLAN is found
 * */

#define GW_BLEFI_FIND_TOPIC "/blefi/find"
#define GW_BLEFI_SCAN_TOPIC "/blefi/macaddr01234/scan"
#define GW_BLEFI_LINKE_TOPIC "/blefi/macaddr01234/linke"
#define GW_BLEFI_LINKT_TOPIC "/blefi/macaddr01234/linkt"

/*Pointers that point to subsctiption topics */
char * gw_blefi_scan_topic;
char * gw_blefi_linke_topic;
char * gw_blefi_linkt_topic;

/*Defining Publish Topic of BleFi Gateway context
 * These are mostly responses for the Subscribed topics
 * */
char * pub_gw_blefi_find_topic = "/blefi/found";
char * pub_gw_scanres_topic = "/blefi/macaddr01234/scanres";
char * pub_gw_linkeres_topic = "/blefi/macaddr01234/linkeres";
char * pub_gw_linktres_topic = "/blefi/macaddr01234/linktres";

char blefi_mac_string[ADDR_STRING_LEN];
char device_bdAddr[ADDR_STRING_LEN];
unsigned char wlan_mac[SL_MAC_ADDR_LEN];
_i32 dummy_print(const char *pcFormat, ...);
bool MqttIpAcquired = false;
bool MqttLibInit = false;
bool MqttTopicBuilt = false;
#define BLE_GW_CONTEXT 0
unsigned short mqttGwMode = 0;
#define QUICKSTART_CLIENT_ID_LEN 32 //d:quickstart:blefi:5c313e032051

/*
 * MAC String is published by blefi /blefi/find message is received
 *
 * */
connection_config_t gw_connect_config =
{

	{
		{
			SL_MQTT_NETCONN_URL,
			NULL,
			PORT_NUMBER,
			0,
			0,
			0,
			NULL
		},
		SERVER_MODE,
		true,
	},
	NULL,
	"macaddr01234",
	NULL,
	NULL,
	true,
	KEEP_ALIVE_TIMER,
	{MqttLibRecvCB, MqttLibEvtCB, MqttLibDisconnectCB},
	GW_TOPIC_COUNT,
	{GW_BLEFI_FIND_TOPIC,GW_BLEFI_SCAN_TOPIC,GW_BLEFI_LINKE_TOPIC,GW_BLEFI_LINKT_TOPIC},
	{QOS2,QOS2,QOS2,QOS2},
	{WILL_TOPIC,WILL_MSG,WILL_QOS,WILL_RETAIN},
	false

};

/* library configuration */
SlMqttClientLibCfg_t Mqtt_Client={
    1882,
    TASK_PRIORITY,
    30,
    true,
    dummy_print
    //UART_PRINT
};






//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

//****************************************************************************
//! Defines Mqtt_Pub_Message_Receive event handler.
//! Client App needs to register this event handler with sl_ExtLib_mqtt_Init 
//! API. Background receive task invokes this handler whenever MQTT Client 
//! receives a Publish Message from the broker.
//!
//!\param[out]     topstr => pointer to topic of the message
//!\param[out]     top_len => topic length
//!\param[out]     payload => pointer to payload
//!\param[out]     pay_len => payload length
//!\param[out]     retain => Tells whether its a Retained message or not
//!\param[out]     dup => Tells whether its a duplicate message or not
//!\param[out]     qos => Tells the Qos level
//!
//!\return none
//****************************************************************************

static void
MqttLibRecvCB(void *app_hndl, const char  *topstr, _i32 top_len, const void *payload,
                       _i32 pay_len, bool dup,unsigned char qos, bool retain)
{
    
    mqtt_gw_messages_t * var;
    var = malloc(sizeof(mqtt_gw_messages_t));
    if(var == NULL)
    {
		HandleError(__FILE__,__LINE__,1); // fatal error
		return;
    }

//Fill the Topic
    var->topic = (char*)malloc(top_len+1);
    if(var->topic == NULL)
    {
    	free(var);
		HandleError(__FILE__,__LINE__,1); // fatal error
		return;
    }
    strncpy(var->topic, (char*)topstr, top_len);
    var->topic[top_len]='\0';
    //Fill the data
    var->datastring = (char*)malloc(pay_len+1);
    if(var->datastring == NULL)
    {
    	free(var->topic);
    	free(var);
		HandleError(__FILE__,__LINE__,1); // fatal error
		return;
    }
	strncpy(var->datastring, (char*)payload, pay_len);
	var->datastring[pay_len]='\0';
//Fill the apphandle
	var->app_handle = app_hndl;
	var->src = MQTT_BLEFI;
//Post the message
	OsiReturnVal_e retVal;
	retVal = osi_MsgQWrite(&g_Mqtt_queue,&var,OSI_NO_WAIT);
	if(retVal !=OSI_OK)
	 {
		 Report("\n\rMessage post fail %d",retVal);
	//	This is seen when lot of notifications arrive. Better to ignore.
		 free(var->topic);
		 free(var->datastring);
		 free(var);
	 }
    return;
}


void MqttGwEventCB(unsigned char EventType, ErrorFlag_t ErrorFlag, void* EventParams)
{
	gwEvent_t* pEvent = (gwEvent_t*)EventParams;
	mqtt_gw_messages_t * var;
	if(EventType==GW_EVENT_LINKESTABLISH || EventType == GW_EVENT_LINKTERMINATE)
	{
		var = malloc(sizeof(mqtt_gw_messages_t));
	    if(var == NULL)
	    {
			HandleError(__FILE__,__LINE__,1); // fatal error
			return;
	    }
		var->src = BLE_EVENT;
		var->evnt = (mqtt_messages)EventType;
		var->data = pEvent->linkEst.connhandle;
		/*
		 * Fill the devic name in the topic
		 * */
		var->topic = NULL;
		if(EventType==GW_EVENT_LINKESTABLISH)
		{
			if(pEvent->linkEst.deviceName!=NULL)
			{
				var->topic = malloc(strlen(pEvent->linkEst.deviceName)+1);
			    if(var->topic == NULL)
				{
					free(var);
					HandleError(__FILE__,__LINE__,1); // fatal error
					return;
				}
				strcpy(var->topic,pEvent->linkEst.deviceName);
			}
		}
		if(pEvent->linkEst.bdAddr != NULL)
		{
			var->datastring = malloc(B_ADDR_LEN);
		    if(var->datastring == NULL)
		    {
		    	free(var->topic);
		    	free(var);
				HandleError(__FILE__,__LINE__,1); // fatal error
				return;
		    }
			memcpy(var->datastring,pEvent->linkEst.bdAddr,B_ADDR_LEN);
		}
		else
		{
			var->datastring = NULL;
		}
		OsiReturnVal_e retVal;
		retVal = osi_MsgQWrite(&g_Mqtt_queue,&var,OSI_NO_WAIT);

		if(retVal !=OSI_OK)
		 {
			 Report("\n\rMessage post fail %d",retVal);
		//	This is seen when lot of notifications arrive. Better to ignore.
			 if(var->datastring!=NULL)
			 {
				 free(var->datastring);
			 }
			 if(var->topic!=NULL)
			 {
				 free(var->topic);
			 }
			 free(var);
		 }
	}
}


void MqttIpAcquiredPostMsg()
{
    mqtt_gw_messages_t * var;
    OsiReturnVal_e retVal;

    var = malloc(sizeof(mqtt_gw_messages_t));
    if(var == NULL)
    {
		HandleError(__FILE__,__LINE__,1); // fatal error
		return;
    }
    var->src = CONNECT_MSG;
    var->datastring = NULL;
    var->topic = NULL;

    retVal = osi_MsgQWrite(&g_Mqtt_queue,&var,OSI_NO_WAIT);
	if(retVal !=OSI_OK)
	 {
		 Report("\n\rMessage post fail %d",retVal);
	//	This is seen when lot of notifications arrive. Better to ignore.
		 free(var);
	 }
}

void MqttTriggerScan()
{

	mqtt_gw_messages_t * var;
	var = malloc(sizeof(mqtt_gw_messages_t));
	if(var==NULL)
	{
		HandleError(__FILE__,__LINE__,1);
		return;
	}
	var->app_handle = NULL;
	var->datastring = NULL;
	var->topic = NULL;
	var->src = MQTT_BLEFI;
	//Post the message

	OsiReturnVal_e retVal;
    retVal = osi_MsgQWrite(&g_Mqtt_queue,&var,OSI_NO_WAIT);
	if(retVal !=OSI_OK)
	 {
		HandleError(__FILE__,__LINE__,2);
	//	This is seen when lot of notifications arrive. Better to ignore.
		 free(var);
	 }
	return;
}



//****************************************************************************
//! Defines MqttLibEvtCB event handler.
//! Client App needs to register this event handler with sl_ExtLib_mqtt_Init 
//! API. Background receive task invokes this handler whenever MQTT Client 
//! receives an ack(whenever user is in non-blocking mode) or encounters an error.
//!
//! param[out]      evt => Event that invokes the handler. Event can be of the
//!                        following types:
//!                        MQTT_ACK - Ack Received 
//!                        MQTT_ERROR - unknown error
//!                        
//!  
//! \param[out]     buf => points to buffer
//! \param[out]     len => buffer length
//!       
//! \return none
//****************************************************************************
static void
MqttLibEvtCB(void *app_hndl,_i32 evt, const void *buf,_u32 len)
{
    int i;
    switch(evt)
    {
      case SL_MQTT_CL_EVT_PUBACK:
        UART_PRINT("PubAck:\n\r");
        UART_PRINT("%s\n\r",buf);
        break;
    
      case SL_MQTT_CL_EVT_SUBACK:
        UART_PRINT("\n\rGranted QoS Levels are:\n\r");
        
        for(i=0;i<len;i++)
        {
          UART_PRINT("QoS %d\n\r",((unsigned char*)buf)[i]);
        }
        break;
        
      case SL_MQTT_CL_EVT_UNSUBACK:
        UART_PRINT("UnSub Ack \n\r");
        UART_PRINT("%s\n\r",buf);
        break;
    
      default:
        break;
  
    }
}

//****************************************************************************
//
//! callback event in case of MQTT disconnection
//!
//! \param app_hndl is the handle for the disconnected connection
//!
//! return none
//
//****************************************************************************

static void
MqttLibDisconnectCB(void *app_hndl)
{
	mqtt_gw_messages_t * var;
	OsiReturnVal_e retVal;
	var = malloc(sizeof(mqtt_gw_messages_t));
	if(var == NULL)
	{
		HandleError(__FILE__,__LINE__,1); // fatal error
		return;
	}
	var->data = 0;
	var->app_handle = app_hndl;
	var->datastring = NULL;
	var->topic = NULL;
	retVal = osi_MsgQWrite(&g_Mqtt_queue,&var,OSI_NO_WAIT);
	if(retVal !=OSI_OK)
	 {
		 Report("\n\r[MQTT] Message post fail %d",retVal);
	//	This is seen when lot of notifications arrive. Better to ignore.
		 free(var);
	 }
    return;
}


void MqttGwBuildTopics()
{

	 /*Build the Subscription topics
	  * */
	//Report("\n\rMQTT - Get MAC");
	Wifi_GetMAC(wlan_mac);
	if(mqttGwMode == MQTT_QUICKSTART)
	{
		gw_connect_config.client_id = malloc(QUICKSTART_CLIENT_ID_LEN);
		if(gw_connect_config.client_id==NULL)
		{
			Message("\n\r[MQTT] Malloc Error");
			return;
		}
		sprintf(gw_connect_config.client_id,"d:quickstart:blefi:%02x%02x%02x%02x%02x%02x",wlan_mac[0],wlan_mac[1],wlan_mac[2],wlan_mac[3],wlan_mac[4],wlan_mac[5]);
		gw_connect_config.client_id[QUICKSTART_CLIENT_ID_LEN-1] = '\0';
	}
	else
	{
		//MQTT Demo

		sprintf(blefi_mac_string,"%02x%02x%02x%02x%02x%02x",wlan_mac[0],wlan_mac[1],wlan_mac[2],wlan_mac[3],wlan_mac[4],wlan_mac[5]);
		blefi_mac_string[12] = '\0';

		/*
		 * Build the Subscription topics
		 *
		 * */
		gw_connect_config.client_id = malloc(strlen(blefi_mac_string)+1);
		if(gw_connect_config.client_id==NULL)
		{
			HandleError(__FILE__,__LINE__,1);
			return;
		}
		strcpy(gw_connect_config.client_id,blefi_mac_string);
		gw_connect_config.client_id[12]='\0';

		sprintf(gw_connect_config.topic[1],"/blefi/%s/scan",blefi_mac_string);
		sprintf(gw_connect_config.topic[2],"/blefi/%s/linke",blefi_mac_string);
		sprintf(gw_connect_config.topic[3],"/blefi/%s/linkt",blefi_mac_string);

		gw_blefi_scan_topic = gw_connect_config.topic[1];
	    gw_blefi_linke_topic = gw_connect_config.topic[2];
		gw_blefi_linkt_topic = gw_connect_config.topic[3];

		/*
		 * Build the Publishing Topics
		 *
		 * */
		sprintf(pub_gw_scanres_topic,"/blefi/%s/scanres",blefi_mac_string);
		sprintf(pub_gw_linkeres_topic,"/blefi/%s/linkeres",blefi_mac_string);
		sprintf(pub_gw_linktres_topic,"/blefi/%s/linktres",blefi_mac_string);

	}
	UART_PRINT("\n\r[MQTT] Client ID %s",gw_connect_config.client_id);
}


//*****************************************************************************
//
//! Mqtt_publish_dev_list
//!
//! @param  none
//!
//! @return none
//!
//! @brief  Parse the received Dev List String
//
//*****************************************************************************
void fnPtrMqttAdvBuild(unsigned char devIndex, char * devName , unsigned char *bdAddr, unsigned char addrType);
void fnPtrMqttConBuild(unsigned char devIndex, char * devName , unsigned char *bdAddr, unsigned char addrType);
void fnPtrMqttAddCon(unsigned char devIndex, char * devName , unsigned char *bdAddr, unsigned char addrType);


int MqttDevListPublish()
{
	if(mqttGwMode != MQTT_QUICKSTART)
	{
		UART_PRINT("\n\r[MQTT] Device Scan Cmd");
		//UART_PRINT("\n\r");
		GC_DeviceScanList(fnPtrMqttAdvBuild);
		//UART_PRINT("\n\r");
	}
	//UART_PRINT("\n\r Connected Devices");
	//UART_PRINT("\n\r");
	GC_DeviceConnList(fnPtrMqttConBuild);
	//UART_PRINT("\n\r");

	return 0;
}



char gw_scanres_message[100];




void fnPtrMqttAdvBuild(unsigned char devIndex,  char * devName , unsigned char *bdAddr,unsigned char addrType)
{

	sprintf(gw_scanres_message,"%d %s     %s Not-Connected",devIndex ,devName, Util_convertBdAddr2Str(bdAddr));
	sl_ExtLib_MqttClientSend((void*)gw_connect_config.clt_ctx,
						pub_gw_scanres_topic,gw_scanres_message,strlen((char*)gw_scanres_message),QOS2,RETAIN);
	//UART_PRINT("\n\r%s",gw_scanres_message);

}

void fnPtrMqttConBuild(unsigned char devIndex,  char * devName , unsigned char *bdAddr,unsigned char addrType)
{

	if(mqttGwMode == MQTT_QUICKSTART)
	{
		sprintf((char*)gw_scanres_message,"{\n\"d\":{\n\"%d\":\"%s-%s\"\n}\n}",devIndex,devName,Util_convertBdAddr2Str(bdAddr));
		sl_ExtLib_MqttClientSend((void*)gw_connect_config.clt_ctx,
						QS_PUBLISH_TOPIC,gw_scanres_message,strlen((char*)gw_scanres_message),QOS0,RETAIN);
	}
	else
	{
		sprintf(gw_scanres_message,"%d %s     %s Connected",devIndex ,devName, Util_convertBdAddr2Str(bdAddr));
		sl_ExtLib_MqttClientSend((void*)gw_connect_config.clt_ctx,
							pub_gw_scanres_topic,gw_scanres_message,strlen((char*)gw_scanres_message),QOS2,RETAIN);
		UART_PRINT("\n\r%s",gw_scanres_message);
	}


}


char gw_link_message[50];
unsigned char bdaddress[6];
void MqttDevConnectPublish(unsigned int deviceId, char * bdAddr)
{
	/*
	 * Link establish
	 * */
	if(bdAddr!=NULL)
	{
		sprintf(gw_link_message,"Device <%d, %s> Connected",deviceId,Util_convertBdAddr2Str((unsigned char *)bdAddr));
	}
	else
	{
		sprintf(gw_link_message,"Device <%d> Connected",deviceId);
	}
	sl_ExtLib_MqttClientSend((void*)gw_connect_config.clt_ctx,
		        			pub_gw_linkeres_topic,gw_link_message,strlen((char*)gw_link_message),QOS2,RETAIN);
}


void MqttDevDisconnectPublish(unsigned int deviceId,char * bdAddr)
{
	if(bdAddr!=NULL)
	{
		sprintf(gw_link_message,"Device <%d, %s> Disconnected",deviceId,Util_convertBdAddr2Str((unsigned char *)bdAddr));
	}
	else
	{
		sprintf(gw_link_message,"Device <%d> Disconnected",deviceId);
	}

	sl_ExtLib_MqttClientSend((void*)gw_connect_config.clt_ctx,
		        			pub_gw_linktres_topic,gw_link_message,strlen((char*)gw_link_message),QOS2,RETAIN);
}


void MqttBleFiHandler (mqtt_gw_messages_t * RecvQue)
{
	if(mqttGwMode == MQTT_QUICKSTART)
	{
		/*
		 * The only trigger in QS mode is publishing the connected device list.
		 *
		 */
		MqttDevListPublish();
		return;
	}


	if(strcmp(RecvQue->topic,GW_BLEFI_FIND_TOPIC) == 0)
	{
        //
        // send publish message
        //
		sl_ExtLib_MqttClientSend((void*)gw_connect_config.clt_ctx,
				pub_gw_blefi_find_topic,blefi_mac_string,strlen((char*)blefi_mac_string),QOS2,RETAIN);
	}
	else if(strcmp(RecvQue->topic,gw_blefi_scan_topic) == 0)
	{
		GC_Scan(GW_CALL_BLOCKING);
		MqttDevListPublish();
	}
	else if(strcmp(RecvQue->topic,gw_blefi_linke_topic) == 0)
	{
		short int uConnId;
		RecvQue->data = strtoul(RecvQue->datastring,NULL,10);
		uConnId = GC_LinkEstablish(RecvQue->data , GW_CALL_BLOCKING, bdaddress);
		if(uConnId>=0)
		{

			/*
			 * Dont publish anything here.
			 * The Success publish will be taken care in the GW-EVENT-PATH
			 * */
			return;

		}
		else
		{
			sprintf(gw_link_message,"FAIL - Device <%d> Not-Connected",RecvQue->data);
			sl_ExtLib_MqttClientSend((void*)gw_connect_config.clt_ctx,
				        			pub_gw_linkeres_topic,gw_link_message,strlen((char*)gw_link_message),QOS2,RETAIN);
			UART_PRINT("\n\r%s",gw_link_message);
			return;
		}

	}
	else if(strcmp(RecvQue->topic,gw_blefi_linkt_topic) == 0)
	{
		RecvQue->data = strtoul(RecvQue->datastring,NULL,10);
		GC_LinkTerminate(RecvQue->data, GW_CALL_BLOCKING);
	}
	else
	{
		//Unwanted Topic Do nothing
		UART_PRINT("\n\rMQTT - Unknown Message Received");
	}

}


/*End Functions for Building List*/

/*
 * The Below functions are used to initialize the device related contexts
 * Call the gateway API to find if any devices are connected.
 * If Yes, then add the context for that device.
 *
 * */

void fnPtrMqttAddCon(unsigned char devIndex,  char * devName , unsigned char *bdAddr,unsigned char addrType)
{
	MqttDeviceCtxAdd(devIndex, devName,bdAddr);
}


void MqttDeviceCtxInit()
{
	GC_DeviceConnList(fnPtrMqttAddCon);
}
/*End functions for adding Device context*/


char * ibmdata[300];
int MqttInitGwContext()
{
	int lRetVal = 0;
    //
    // Initialze MQTT client lib
    //
	if (MqttLibInit == false)
	{
		lRetVal = sl_ExtLib_MqttClientInit(&Mqtt_Client);
	   	if(lRetVal != 0)
		{
			// lib initialization failed
			UART_PRINT("MQTT Client lib initialization failed\n\r");
			return (-1);
		}
		MqttLibInit = true;
	}

    /******************* connection to the broker ***************************/
	//create client context
    gw_connect_config.clt_ctx =
	sl_ExtLib_MqttClientCtxCreate(&(gw_connect_config.broker_config),
								  &(gw_connect_config.CallBAcks),
								  &gw_connect_config);

	//
	// Set Client ID
	//
	sl_ExtLib_MqttClientSet((void*)gw_connect_config.clt_ctx,
						SL_MQTT_PARAM_CLIENT_ID,
						gw_connect_config.client_id,
						strlen((char*)(gw_connect_config.client_id)));

	//
	// Set will Params
	//
	if(gw_connect_config.will_params.will_topic != NULL)
	{
		sl_ExtLib_MqttClientSet((void*)gw_connect_config.clt_ctx,
								SL_MQTT_PARAM_WILL_PARAM,
								&(gw_connect_config.will_params),
								sizeof(SlMqttWill_t));
	}

	//
	// setting username and password
	//
	if(gw_connect_config.usr_name != NULL)
	{
		sl_ExtLib_MqttClientSet((void*)gw_connect_config.clt_ctx,
							SL_MQTT_PARAM_USER_NAME,
							gw_connect_config.usr_name,
							strlen((char*)gw_connect_config.usr_name));

		if(gw_connect_config.usr_pwd != NULL)
		{
			sl_ExtLib_MqttClientSet((void*)gw_connect_config.clt_ctx,
							SL_MQTT_PARAM_PASS_WORD,
							gw_connect_config.usr_pwd,
							strlen((char*)gw_connect_config.usr_pwd));
		}
	}

	Report("\n\r[MQTT] Server : %s",gw_connect_config.broker_config.server_info.server_addr);
	Report("\n\r[MQTT] Client-Id : %s",gw_connect_config.client_id);
	//
	// connection to the broker
	//
	UART_PRINT("\n\r[MQTT] Connecting to broker... ");
	if((sl_ExtLib_MqttClientConnect((void*)gw_connect_config.clt_ctx,
			gw_connect_config.is_clean,
			gw_connect_config.keep_alive_time) & 0xFF) != 0)
	{
		UART_PRINT("\n\r[MQTT] Broker connect fail ");
		//delete the context for this connection
		sl_ExtLib_MqttClientCtxDelete(gw_connect_config.clt_ctx);
		return (-1);
	}

	UART_PRINT("\n\r[MQTT] Broker connect Successful ");

	if(mqttGwMode != MQTT_QUICKSTART)
	{
		//
		// No topic subscription in case of Quickstart
		//
		UART_PRINT("\n\r[MQTT] Subscribing to topics... ");
		if(sl_ExtLib_MqttClientSub((void*)gw_connect_config.clt_ctx,
				gw_connect_config.topic,
				gw_connect_config.qos, GW_TOPIC_COUNT) < 0)
		{
			UART_PRINT("\n\r[MQTT] ERROR - Subscription Error");
			UART_PRINT("\n\r[MQTT] Disconnecting from the broker");
			sl_ExtLib_MqttClientDisconnect(gw_connect_config.clt_ctx);
			gw_connect_config.is_connected = false;
			//delete the context for this connection
			sl_ExtLib_MqttClientCtxDelete(gw_connect_config.clt_ctx);
			return(-1);
		}
		else
		{
			int iSub;
			UART_PRINT("\n\r[MQTT] Subscription to these topics successful");
			for(iSub = 0; iSub < gw_connect_config.num_topics; iSub++)
			{
				UART_PRINT("\n\r%s", gw_connect_config.topic[iSub]);
			}
			UART_PRINT("\n\r");
		}
	}
	gw_connect_config.is_connected = true;
	return(0);

}

void mqttCleanupCtx()
{
//First clean up the device contexts	 var->topic = blefiMalloc(strlen((const char *)pData->getRsp.pCurrentGetCharName)+1);
	 	MqttDevClean();
		if(gw_connect_config.is_connected == true)
		{
			if(mqttGwMode != MQTT_QUICKSTART)
			{
				sl_ExtLib_MqttClientUnsub((void*)gw_connect_config.clt_ctx,
						gw_connect_config.topic,
						GW_TOPIC_COUNT);
			}
			sl_ExtLib_MqttClientDisconnect(gw_connect_config.clt_ctx);
			sl_ExtLib_MqttClientCtxDelete(gw_connect_config.clt_ctx);
			gw_connect_config.is_connected = false;
		}


}
//*****************************************************************************
//
//! Task implementing MQTT client communication to other web client through
//!    a broker
//!
//! \param  none
//!
//! This function
//!    1. Initializes network driver and connects to the default AP
//!    2. Initializes the mqtt library and set up MQTT connection configurations
//!    3. set up the button events and their callbacks(for publishing)
//!    4. handles the callback signals
//!
//! \return None
//!
//*****************************************************************************
void MqttClient(void *pvParameters)
{
    static mqtt_gw_messages_t * RecvQue;
    volatile unsigned char LoopVar=0xFF;
    //Report("MQTT Client application -> waiting for IP address");
   	//
   	// Register the callback function to gateway module
   	// the callback will be called whenever an event is generated from teh gateway
   	//

   	RegisterGwCB(GW_EVENT_CB,MqttGwEventCB);
	RegisterGwCB(GW_DATA_CB,MqttDevDataCB);

	while (LoopVar)
	{
		if (is_blefiCfgReady == true)
		{
			break;
		}
		osi_Sleep(250);
	}

	/*
	 * Read the MQTT mode from NVMEM
	 * */
	ReadMQTTMode();
    while(LoopVar)
    {

/*
 * The task handles two kinds of mesages
 * 1 -> messages from MQTT Broker (based on the subscribed topics)
 * 2 -> messages from the BLE Gateway (eg: device connected, disconnected, etc)
 */
        osi_MsgQRead( &g_Mqtt_queue, &RecvQue, OSI_WAIT_FOREVER);
        switch(RecvQue->src)
        {
        case MQTT_BLEFI:
        	/*
        	 * These messages are originated from MQTT library, and for BLEFI context
        	 * The Topics are find, scan, linke and linkt
        	 *
        	 * */
        	MqttBleFiHandler(RecvQue);
        	break;
        case MQTT_DEVICE:
        	/*
			 * These messages are originated from MQTT library, and for Device context
			 * function is defined in the device_mqtt file
			 * These events should not be triggered in QUICKSTART MODE
			 * */
        	MqttDeviceHandler(RecvQue);
        	break;
        case BLE_EVENT:


          	if(RecvQue->evnt == (mqtt_messages)GW_EVENT_LINKESTABLISH)
        	{
          		/*
				 * Link Establish
				 * */
          		if(mqttGwMode!=MQTT_QUICKSTART)
          		{
          		 	MqttDevConnectPublish(RecvQue->data, RecvQue->datastring);
          		}
				MqttDeviceCtxAdd(RecvQue->data,RecvQue->topic,(unsigned char *)RecvQue->datastring);
        	}
          	else
          	{
          		/*
          		 * Link Terminate
          		 * */
          		if(mqttGwMode!=MQTT_QUICKSTART)
          		{
          			MqttDevDisconnectPublish(RecvQue->data, RecvQue->datastring);
          		}
          		MqttDeviceCtxDel(RecvQue->data);

          	}
          	break;
        case BLE_DATA:
        	/*
			 * These messages are generated by BLE Data event handler, mostly for set and get
			 * */

        	MqttDeviceGetPublish(RecvQue);
        	break;
        case CONNECT_MSG:
        	/*
        	 * Called when the IP is acquired, the first time contexts get prepared
        	 * */
    		if(MqttTopicBuilt == false)
    		{
    			MqttGwBuildTopics();
    			MqttTopicBuilt = true;
    		}
    		//Cleanup the contexts if previously initialized
        	mqttCleanupCtx();
        	//Initialize the gateway context
        	MqttInitGwContext();
        	//Initialize the Device context
        	MqttDeviceCtxInit();
        	break;
        case CLEANUP_MSG:
        	if(RecvQue->data == 0)
        	{
        		if(gw_connect_config.is_connected == true)
        		{
        			if(mqttGwMode != MQTT_QUICKSTART)
        			{
        				sl_ExtLib_MqttClientUnsub((void*)gw_connect_config.clt_ctx,
        						gw_connect_config.topic,
        						GW_TOPIC_COUNT);
        			}
        			sl_ExtLib_MqttClientDisconnect(gw_connect_config.clt_ctx);
        			sl_ExtLib_MqttClientCtxDelete(gw_connect_config.clt_ctx);
        			gw_connect_config.is_connected = false;
        		}
        	}
        	else
        	{
        		MqttDeviceCtxClean((void*)RecvQue->app_handle);
        	}
        	break;
        case PERIODIC_TASK:
        	if(mqttDevMode == MQTT_QUICKSTART)
        	{
        		MqttDevicePubsishQS();
        	}
        	break;
        default:
        	break;
        }
        MqttFreeMsg(RecvQue);
    } // for
}

void MqttFreeMsg(mqtt_gw_messages_t * RecvQue)
{
	if(RecvQue!=NULL)
	{
		if(RecvQue->datastring != NULL)
			free(RecvQue->datastring);
		if(RecvQue->topic != NULL)
			free(RecvQue->topic);
		free(RecvQue);
	}
}

//*****************************************************************************
//
//! Main 
//!
//! \param  none
//!
//! This function Creates the Mqtt Client task.
//!
//! \return None
//!
//*****************************************************************************
void MQTT_Init()
{ 
    long lRetVal = -1;

    //
    // Start the MQTT Client task
    //
    osi_MsgQCreate(&g_Mqtt_queue,"MqttGWqueue",sizeof(mqtt_gw_messages_t *),40);

    lRetVal = osi_TaskCreate(MqttClient,
                            (const signed char *)"Mqtt Client App",
                            OSI_STACK_SIZE, NULL, 2, NULL );

    if(lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        LOOP_FOREVER();
    }

}

char * MqttBlefiMAC()
{
	return blefi_mac_string;
}

_i32 dummy_print(const char *pcFormat, ...)
{
	//do nothing
	return(0);
}

#if 0
void MqttGwInfo()
{
	unsigned char i;
	Report("\n\rGateway Connection Details");
	Report("\n\rServer     : %s",gw_connect_config.broker_config.server_info.server_addr);
	Report("\n\rConnected? : %s",((gw_connect_config.is_connected==true)?"YES":"NO"));
	Report("\n\rSubscribed Topics");
	for(i=0;i<GW_TOPIC_COUNT;i++)
	{
		Report("\n\r	%s",gw_connect_config.topic[i]);
	}
}
#endif


unsigned int getSizeofMqttCfg()
{
	return(sizeof(mqttCfg_t));
}

void fillMqttDefaultCfg(void * mqttRec)
{
	mqttCfg_t * mqttTempRec;
	mqttTempRec = mqttRec;
	mqttTempRec->mqttGwMode = MQTT_DEMO;
	mqttTempRec->mqttDevMode = MQTT_QUICKSTART;
	strncpy(mqttTempRec->demoserver,DEFAULT_DEMO_SERVER,strlen(DEFAULT_DEMO_SERVER));
}


void ReadMQTTMode()
{
	unsigned int len;
	mqttCfg_t * mqttCfg;
	mqttCfg = (mqttCfg_t *) blefiCfgRec.mqttCfg;
	mqttGwMode = mqttCfg->mqttGwMode;
	if(mqttGwMode == MQTT_QUICKSTART)
	{
		len = strlen(QS_SERVER);
		gw_connect_config.broker_config.server_info.server_addr = (char*)malloc(len +1);
		if(gw_connect_config.broker_config.server_info.server_addr == NULL)
		{
			HandleError(__FILE__,__LINE__,1);
			return;
		}
		strcpy((char*)gw_connect_config.broker_config.server_info.server_addr,QS_SERVER);
	}
	else
	{
		len = strlen(mqttCfg->demoserver);
		gw_connect_config.broker_config.server_info.server_addr = (char*)malloc(len+1);
		if(gw_connect_config.broker_config.server_info.server_addr == NULL)
		{
			HandleError(__FILE__,__LINE__,1);
			return;
		}
		strcpy((char*)gw_connect_config.broker_config.server_info.server_addr,mqttCfg->demoserver);

	}
	Report("\n\r[MQTT] GW Mode   : %s",(mqttGwMode==0?"demo":"quickstart"));
	Report("\n\r[MQTT] Gw Server : %s",gw_connect_config.broker_config.server_info.server_addr);
	InitDevModeServer();
}

void cfgGwMqttMode(unsigned int mode,char * mqttServer)
{
	mqttCfg_t * mqttCfg;
	mqttCfg = (mqttCfg_t *) blefiCfgRec.mqttCfg;
	mqttCfg->mqttGwMode = mode;
	if(mode!=MQTT_QUICKSTART)
	{
		strncpy(mqttCfg->demoserver,mqttServer,sizeof(mqttCfg->demoserver)-1);
		mqttCfg->demoserver[sizeof(mqttCfg->demoserver)-1] = '\0';
	}
	UpdateBlefiCfg();
	return;

}

unsigned short readGwMqttCfg(char ** mqttServer)
{
	mqttCfg_t * mqttCfg;
	mqttCfg = (mqttCfg_t *) blefiCfgRec.mqttCfg;
	if(mqttGwMode==MQTT_QUICKSTART)
	{
		*mqttServer = QS_SERVER;
	}
	else
	{
		*mqttServer = mqttCfg->demoserver;

	}
	return(mqttCfg->mqttGwMode);
}
