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
#include "hw_types.h"
#include "hw_ints.h"
#include "osi.h"
#include "gateway_api.h"
#include "simplelink.h"
#include "uart_if.h"
#include "common.h"
#include "utils.h"
#include "sl_mqtt_client.h"
#include "mqtt_app.h"

//#define MQTT_PUBLISH_NON_OPTIMAL
#define INCLUDE_DEVICE_DEMO_MODE

/*Message Queue*/
extern OsiMsgQ_t g_Mqtt_queue;
extern bool MqttIpAcquired;
extern bool MqttLibInit;
#define INVALID_ID 0xFFFF
#define MAX_NOF_MQTT_ARGS	10
/*subscription topics and messages
 * The bdaddr012345 is placeholder,
 * actual bdaddr will be replaced in
 * this place once connected
 * */
#define SUB_WHOAMI_TOPIC "/bdaddr012345/whoami"
#define SUB_LIST_TOPIC "/bdaddr012345/list"
#define SUB_SET_TOPIC "/bdaddr012345/get"
#define SUB_GET_TOPIC "/bdaddr012345/set"

/*publishing topics and messages
 * The bdaddr012345 is placeholder,
 * actual bdaddr will be replaced in
 * this place once connected
 * */
#define PUB_IAM_TOPIC "/bdaddr012345/iam"
#define PUB_LIST_TOPIC "/bdaddr012345/listres"
#define PUB_SET_TOPIC "/bdaddr012345/getres"
#define PUB_GET_TOPIC "/bdaddr012345/setres"
#define DEV_QUICKSTART_CLIENT_ID_LEN 33
#define DEMO_DEV_CLIENT_ID_LEN 13

extern void HandleError(unsigned char *, unsigned int, unsigned char error);

/*
 * Common connection configuration parameters for the devices.
 * */

typedef struct device_con_config{
    SlMqttClientCtxCfg_t broker_config;
    unsigned char *usr_name;
    unsigned char *usr_pwd;
    bool is_clean;
    unsigned int keep_alive_time;
    SlMqttClientCbs_t CallBAcks;
    unsigned char qos[SUB_TOPIC_COUNT];
    SlMqttWill_t will_params;
}device_con_config_t;

/*
 * Common connection configuration parameters for the devices.
 * */

//#define INCLUDE_DEVICE_DEMO_MODE

unsigned short mqttDevMode = 1;




static void MqttLibDeviceRecvCB(void *dev_hndl, const char  *topstr, _i32 top_len, const void *payload,
                       _i32 pay_len, bool dup,unsigned char qos, bool retain);
static void MqttLibDeviceDisconnectCB(void *app_hndl);
static void MqttLibDeviceEvtCB(void *app_hndl,_i32 evt, const void *buf,_u32 len);
static int MqttDeviceFindHdl(unsigned char idx);
static unsigned short MqttFindIdx(unsigned short connhandle);
static void MqttDeviceBuildTopics(unsigned char device_idx, unsigned char * bdaddress);

extern void collectCC26XXSensorData(device_handle_t * temp_dev_hndl,char * topic,char * datastring );
extern void sensorTagEnableNoti(unsigned char dev_idx,unsigned char con_id);
extern void publishCC26XXSensorData(device_handle_t * temp_dev_hndl );
extern unsigned int getGenericTaskTicksPerSec();
extern char * MqttBlefiMAC();
extern blefiCfg_t blefiCfgRec;

device_con_config_t mqtt_device_cfg =
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
	NULL,
	true,
	KEEP_ALIVE_TIMER,
	{MqttLibDeviceRecvCB, MqttLibDeviceEvtCB, MqttLibDeviceDisconnectCB},
    {QOS2, QOS2,QOS2,QOS2},
    {WILL_TOPIC,WILL_MSG,WILL_QOS,WILL_RETAIN},
};


device_handle_t device_hdl[MAX_DEVICE_CONNECT] =
{
	{
		NULL,
		NULL, //placeholder
		false,
		INVALID_ID,
		{SUB_WHOAMI_TOPIC, SUB_LIST_TOPIC,SUB_SET_TOPIC,SUB_GET_TOPIC},
		{PUB_IAM_TOPIC, PUB_LIST_TOPIC,PUB_SET_TOPIC,PUB_GET_TOPIC},
		{0,0,0,0,0},
		{false,false,false,false},
		false,
		false,
		false
	},
	{
		NULL,
		NULL, //placeholder
		false,
		INVALID_ID,
		{SUB_WHOAMI_TOPIC, SUB_LIST_TOPIC,SUB_SET_TOPIC,SUB_GET_TOPIC},
		{PUB_IAM_TOPIC, PUB_LIST_TOPIC,PUB_SET_TOPIC,PUB_GET_TOPIC},
		{0,0,0,0,0},
		{false,false,false,false},
		false,
		false,
		false
	},
	{
		NULL,
		NULL, //placeholder
		false,
		INVALID_ID,
		{SUB_WHOAMI_TOPIC, SUB_LIST_TOPIC,SUB_SET_TOPIC,SUB_GET_TOPIC},
		{PUB_IAM_TOPIC, PUB_LIST_TOPIC,PUB_SET_TOPIC,PUB_GET_TOPIC},
		{0,0,0,0,0},
		{false,false,false,false},
		false,
		false,
		false
	}
};

unsigned char mqtt_device_connected = 0;
#if 0
#define blefiMalloc(x) malloc(x); Report("<M-%d-%d>",__LINE__,x);
#define blefiFree(x) free (x); Report("<F-%d>",__LINE__);
#else
#define blefiMalloc(x) malloc(x);
#define blefiFree(x) free(x);
#endif

char listcharstr[50];

/*
 * Not good to keep it here, this is a workaround for some bad include file errors
 * */
#define GATT_PROP_READ                   0x02 //!< Permits reads of the Characteristic Value
#define GATT_PROP_WRITE                  0x08 //!< Permits writes of the Characteristic Value with response
#define gattPropRead( a )         		   ( (a) & GATT_PROP_READ )
#define gattPropWrite( a )        		   ( (a) & GATT_PROP_WRITE )

//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

//****************************************************************************
//! Defines MqttDeviceRecvCB event handler.
//! Client App needs to register this event handler with sl_ExtLib_mqtt_Init
//! API. Background receive task invokes this handler whenever MQTT Client
//! receives a Publish Message from the broker for Device context.
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
MqttLibDeviceRecvCB(void *dev_hndl, const char  *topstr, _i32 top_len, const void *payload,
		_i32 pay_len, bool dup,unsigned char qos, bool retain)
{

    mqtt_gw_messages_t * var;
    OsiReturnVal_e retVal;
    var = blefiMalloc(sizeof(mqtt_gw_messages_t));
    if(var == NULL)
    {
		HandleError(__FILE__,__LINE__,1); // fatal error
		return;
    }

//Fill the Topic
    var->topic = (char*)blefiMalloc(top_len+1);

    if(var->topic == NULL)
    {
    	blefiFree(var);
		HandleError(__FILE__,__LINE__,1); // fatal error
		return;
    }

    strncpy(var->topic, (char*)topstr, top_len);
    var->topic[top_len]='\0';
//Fill the data
    if(pay_len > 0)
    {
		var->datastring = (char*)blefiMalloc(pay_len+1);

	    if(var->datastring == NULL)
	    {
	    	blefiFree(var->topic);
	    	blefiFree(var);
			HandleError(__FILE__,__LINE__,1); // fatal error
			return;
	    }

		strncpy(var->datastring, (char*)payload, pay_len);
		var->datastring[pay_len]='\0';
    }
    else
    {
    	var->datastring = NULL;
    }
//Fill the apphandle
	var->app_handle = dev_hndl;
	var->src = MQTT_DEVICE;
//Post the message
	retVal = osi_MsgQWrite(&g_Mqtt_queue,&var,OSI_NO_WAIT);
	if(retVal !=OSI_OK)
	 {
		 Report("\n\rMessage post fail %d",retVal);
	//	This is seen when lot of notifications arrive. Better to ignore.
		 blefiFree(var->topic);
		 blefiFree(var->datastring);
		 blefiFree(var);
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
MqttLibDeviceEvtCB(void *app_hndl,_i32 evt, const void *buf,_u32 len)
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



void
MqttLibDeviceDisconnectCB(void *app_hndl)
{

	mqtt_gw_messages_t * var;
	OsiReturnVal_e retVal;
	var = blefiMalloc(sizeof(mqtt_gw_messages_t));
	if(var == NULL)
	{
		HandleError(__FILE__,__LINE__,1); // fatal error
		return;
	}
	var->data = 1;
	var->app_handle = app_hndl;
	var->datastring = NULL;
	var->topic = NULL;
	var->src = CLEANUP_MSG;
	retVal = osi_MsgQWrite(&g_Mqtt_queue,&var,OSI_NO_WAIT);
	Message("\n\r[MQTT]Device Mqtt Disconnect");
	if(retVal !=OSI_OK)
	 {
		 Report("\n\r[MQTT] Message post fail %d",retVal);
	//	This is seen when lot of notifications arrive. Better to ignore.
		 blefiFree(var);
	 }
	return;
}


//*****************************************************************************
//
//! Mqtt_Data Handler, registered to Gateway
//!
//! \param
//! \param
//!
//! \return none
//!
void MqttDevDataCB(uint8 DataType, ErrorFlag_t ErrorFlag,void* DataParams)
{
	gwData_t* pData = (gwData_t*)DataParams;
	uint8 i;
	int dev_idx ;
	char * tempstr;
	mqtt_gw_messages_t * var;
	OsiReturnVal_e retVal;

	 dev_idx = MqttDeviceFindHdl(pData->getRsp.connhandle);
	 if(dev_idx<0)
	 {
		 Message("\n\r[MQTT] BLE Device not subscribed to MQTT");
		 return;
	 }


	listcharstr[0]='\0';

	 switch (DataType)
	 {
		 case GW_DATA_GET:
		 {

			 //Post the message to MQTT App Task here

			 if(ErrorFlag == NO_ERROR)
			 {
				var = blefiMalloc(sizeof(mqtt_gw_messages_t));
				if(var == NULL)
				{
					HandleError(__FILE__,__LINE__,1); // fatal error
					return;
				}

			//	 Report("\n\r[MQTT] Char Name  %s",pData->getRsp.pCurrentGetCharName);
				 if(mqttDevMode == MQTT_QUICKSTART)
				 {
					 // QS Mode - Value is integer
					 var->datastring = blefiMalloc(pData->getRsp.ValueLen + 1);
					 if(var->datastring == NULL)
					 {

						blefiFree(var);
						HandleError(__FILE__,__LINE__,1); // fatal error
						return;
					 }
					 //First byte : Length,Rest : Data
					 var->datastring[0] = pData->getRsp.ValueLen;
					 memcpy(&(var->datastring[1]),pData->getRsp.pValueBuffer,pData->getRsp.ValueLen);
					 var->data = dev_idx;
					 var->topic = blefiMalloc(strlen((const char *)pData->getRsp.pCurrentGetCharName)+1);
					 if(var->topic == NULL)
					 {
						blefiFree(var->datastring);
						blefiFree(var);
						HandleError(__FILE__,__LINE__,1); // fatal error
						return;
					 }
					 strcpy(var->topic,(const char *)pData->getRsp.pCurrentGetCharName);

#ifdef MQTT_PUBLISH_NON_OPTIMAL
					 retVal = osi_MsgQWrite(&g_Mqtt_queue,&var,OSI_NO_WAIT);
					 if(retVal !=OSI_OK)
					 {
						 Report("\n\rMessage post fail %d",retVal);
						 HandleError(__FILE__,__LINE__,1);
					//	This is seen when lot of notifications arrive. Better to ignore.
						 blefiFree(var->topic);
						 blefiFree(var->datastring);
						 blefiFree(var);
					 }
#else
					 {
					 device_handle_t *temp_dev_hndl = (device_handle_t*) device_hdl + var->data;
					 collectCC26XXSensorData(temp_dev_hndl,var->topic,var->datastring );
					 blefiFree(var->topic);
					 blefiFree(var->datastring);
					 blefiFree(var);
	//				 Report("-{%d}",dev_idx);
					 device_hdl[dev_idx].is_getInProgress=false;
					 }
#endif

				 }
				 else //mqtt-demo
				 {
#ifdef INCLUDE_DEVICE_DEMO_MODE
					 // Demo mode, the value is like a string
					 for(i=0; i<pData->getRsp.ValueLen ;i++)
					 {
						 if(strlen(listcharstr)>=sizeof(listcharstr))
						 {
							 break;
						 }
						 tempstr = (char *) (listcharstr + strlen(listcharstr)) ;
						 sprintf(tempstr," 0x%x", pData->getRsp.pValueBuffer[i]);
					 }
					 var->datastring = blefiMalloc(strlen(listcharstr)+1);
					 if(var->datastring == NULL)
					 {
						blefiFree(var);
						HandleError(__FILE__,__LINE__,1); // fatal error
						return;
					 }
					 strcpy(var->datastring,listcharstr);
					 var->data = dev_idx;
					 var->topic = blefiMalloc(strlen((const char *)pData->getRsp.pCurrentGetCharName)+1);
					 if(var->topic == NULL)
					 {
						blefiFree(var->datastring);
						blefiFree(var);
						HandleError(__FILE__,__LINE__,1); // fatal error
						return;
					 }
					 strcpy(var->topic,(const char *)pData->getRsp.pCurrentGetCharName);
					 var->src = BLE_DATA;
					 var->evnt = BLE_DEVICE_GET;
					 retVal = osi_MsgQWrite(&g_Mqtt_queue,&var,OSI_NO_WAIT);
					 if(retVal !=OSI_OK)
					 {
						 Report("\n\rMessage post fail %d",retVal);
						 HandleError(__FILE__,__LINE__,2);
					//	This is seen when lot of notifications arrive. Better to ignore.
						 blefiFree(var->topic);
						 blefiFree(var->datastring);
						 blefiFree(var);
					 }
#endif
				 }

			 }
			 else
			 {
				 Message("\n\r[MQTT] ERROR in executing GET ");
			 }
		 }
		 break;

		 case GW_DATA_SET:
			 if(ErrorFlag == NO_ERROR)
			 {
				 int dev_idx;
				 dev_idx = MqttDeviceFindHdl(pData->setRsp.connhandle);
				 if(dev_idx<0)
				 {
					 Message("\n\r[MQTT] BLE Device not subscribed to MQTT");
					 return;
				 }
				 device_hdl[dev_idx].is_setInProgress=false;
			 }
			 else
			 {
				 Report("\n\r[MQTT] ERROR in executing SET %d",ErrorFlag);

			 }
		 break;
		 default:
			 break;
	 }
}




static unsigned short mqtt_parse(char *arg,char *argv[])
{
	unsigned short argc=0;

	while(*arg!='\0')
	{
		while(*arg==' ' || *arg=='\t')
				arg++;

                if(*arg=='\0')
                  break;

		argv[argc++]=arg;

		while(*arg!=' ' && *arg!='\t' && *arg!='\0')
				arg++;

		if(*arg!='\0')
		{
			*arg='\0';
			arg++;
		}
	}

	return argc;
}


void MqttDeviceBuildTopics(unsigned char device_idx, unsigned char * bdaddress)
{

	if(mqttDevMode == MQTT_QUICKSTART)
	{

		device_hdl[device_idx].client_id = blefiMalloc(DEV_QUICKSTART_CLIENT_ID_LEN);
		if(device_hdl[device_idx].client_id==NULL)
		{
			Message("\n\r[MQTT] Malloc Error");
			HandleError(__FILE__,__LINE__,1);
			return;
		}
		sprintf(device_hdl[device_idx].client_id,"d:quickstart:st-app:%02x%02x%02x%02x%02x%02x",bdaddress[5],bdaddress[4],bdaddress[3],bdaddress[2],bdaddress[1],bdaddress[0]);
		device_hdl[device_idx].client_id[DEV_QUICKSTART_CLIENT_ID_LEN-1]='\0';
		Report("\n\r[MQTT] Dev Client Id : %s",device_hdl[device_idx].client_id);
	}
	else
	{
#ifdef INCLUDE_DEVICE_DEMO_MODE
		device_hdl[device_idx].client_id = blefiMalloc(DEMO_DEV_CLIENT_ID_LEN);
		if(device_hdl[device_idx].client_id==NULL)
		{
			Message("\n\r[MQTT] Malloc Error");
			return;
		}
		sprintf(device_hdl[device_idx].client_id,"%02x%02x%02x%02x%02x%02x",bdaddress[5],bdaddress[4],bdaddress[3],bdaddress[2],bdaddress[1],bdaddress[0]);
		device_hdl[device_idx].client_id[DEMO_DEV_CLIENT_ID_LEN-1]='\0';

		 /*Build the Subscription topics
		  * */
		sprintf(device_hdl[device_idx].topic[0],"/%s/whoami",device_hdl[device_idx].client_id);
		sprintf(device_hdl[device_idx].topic[1],"/%s/list",device_hdl[device_idx].client_id);
		sprintf(device_hdl[device_idx].topic[2],"/%s/get",device_hdl[device_idx].client_id);
		sprintf(device_hdl[device_idx].topic[3],"/%s/set",device_hdl[device_idx].client_id);

		/*
		 * Build the Publishing Topics
		 *
		 * */
		sprintf(device_hdl[device_idx].pub_topic[0],"/%s/iam",device_hdl[device_idx].client_id);
		sprintf(device_hdl[device_idx].pub_topic[1],"/%s/listres",device_hdl[device_idx].client_id);
		sprintf(device_hdl[device_idx].pub_topic[2],"/%s/getres",device_hdl[device_idx].client_id);
		sprintf(device_hdl[device_idx].pub_topic[3],"/%s/setres",device_hdl[device_idx].client_id);
#endif
	}
}


#ifdef INCLUDE_DEVICE_DEMO_MODE
void fnPtrDevCharListMqttPrint(unsigned short connhandle, unsigned short char_index, char* charName, unsigned char getsetflag)
{
	unsigned short dev_idx = MqttFindIdx(connhandle);
	if (dev_idx>=MAX_DEVICE_CONNECT)
	{
		return;
	}
	if((gattPropRead(getsetflag) !=0) && (gattPropWrite(getsetflag) !=0) )
	{

		sprintf(listcharstr,"%d   %s   [GET/SET]",char_index, charName);

	}
	else if((gattPropRead(getsetflag) !=0) )
	{

		sprintf(listcharstr,"%d   %s   [GET]",char_index, charName);
	}
	else if((gattPropWrite(getsetflag) !=0) )
	{
		sprintf(listcharstr,"%d   %s   [SET]",char_index, charName);

	}
	else
	{
		// No permissions to read or write
		sprintf(listcharstr,"%d   %s   [NO GET/SET]",char_index, charName);
	}

	UART_PRINT("\n\r %s",listcharstr);

	sl_ExtLib_MqttClientSend((void*)device_hdl[dev_idx].clt_ctx,
			device_hdl[dev_idx].pub_topic[1],listcharstr,strlen((char*)listcharstr),QOS2,RETAIN);
}


void MqttDeviceListCharPublish(unsigned char DeviceId )
{
	GC_GetDevCharList(DeviceId , fnPtrDevCharListMqttPrint);
}

void MqttDeviceGetPublish(mqtt_gw_messages_t * RecvQue)
{
	char * topic;
	device_handle_t *temp_dev_hndl = (device_handle_t*) device_hdl + RecvQue->data;

	if(mqttDevMode == MQTT_QUICKSTART)
	{
#ifdef MQTT_PUBLISH_NON_OPTIMAL
		collectCC26XXSensorData(temp_dev_hndl,(char*)RecvQue->topic,(char*)RecvQue->datastring );
#endif
		return;
	}

	if(RecvQue->topic!=NULL)
	{
		topic = blefiMalloc(strlen(temp_dev_hndl->client_id)+2+strlen(RecvQue->topic)+1);
	    if(topic == NULL)
	    {
			HandleError(__FILE__,__LINE__,1); // fatal error
			return;
	    }
		sprintf(topic,"/%s%s",temp_dev_hndl->client_id,RecvQue->topic);
	}
	else
	{
		topic = blefiMalloc(strlen(temp_dev_hndl->client_id)+2+1);
	    if(topic == NULL)
	    {
			HandleError(__FILE__,__LINE__,1); // fatal error
			return;
	    }
		sprintf(topic,"/%s",temp_dev_hndl->client_id);
	}
	sl_ExtLib_MqttClientSend((void*)temp_dev_hndl->clt_ctx,
			topic,RecvQue->datastring,strlen((char*)RecvQue->datastring),QOS2,RETAIN);
	blefiFree(topic);


}
#endif
#ifdef INCLUDE_DEVICE_DEMO_MODE
void MqttDeviceGetHandler(unsigned char DeviceId, char * pInputString  )
{
	/*
	 * Get response is not published here,
	 * Once the Get is successfull, the Gateway calls the callback, and the message is published then.
	 * */
    if(GC_GetSet(DeviceId ,pInputString ,(GC_GETSETFLAG)0,0,NULL) != NO_ERROR)
    {
    	Message("\n\r[MQTT] Get not successful");
    }
	return;
}


void MqttDeviceSetHandler(unsigned char DeviceId, char * pInputString  )
{
	char *argv[MAX_NOF_MQTT_ARGS];
	int argc;
    char* pInputCharString;
    unsigned int Value;
    unsigned char *pValue;
    unsigned char ValueLen;

	argc=mqtt_parse(pInputString,argv);
	if(argc!=0)
	{
		UART_PRINT("\n\r[MQTT] string received  = %s \n\r",pInputString );
	}

    pInputCharString = argv[0];
    ValueLen 	= strtoul(argv[1],NULL,10);
    Value = strtoul(argv[2],NULL,10);
    pValue = (unsigned char *)&Value;


    if(GC_GetSet(DeviceId ,pInputCharString,(GC_GETSETFLAG)1,ValueLen,pValue) != NO_ERROR)
    {
       	Message("\n\r[MQTT] Get not successful");
    }
    blefiFree(pInputString);
	return;
}



void MqttDeviceWhoamiPublish(device_handle_t *temp_dev_hndl)
{
	char * whoami_mac_string;
	char whoamistr [80];
	whoami_mac_string = MqttBlefiMAC();
	sprintf(whoamistr,"[MQTT] Connected to BleFi %s, Id 0x%x",whoami_mac_string,temp_dev_hndl->connection_id);
	sl_ExtLib_MqttClientSend((void*)temp_dev_hndl->clt_ctx,
			temp_dev_hndl->pub_topic[0],whoamistr,strlen((char*)whoamistr),QOS2,RETAIN);
}


void MqttDeviceHandler (mqtt_gw_messages_t * RecvQue)
{
	device_handle_t *temp_dev_hndl = (device_handle_t*) RecvQue->app_handle;

    if(strcmp(RecvQue->topic,temp_dev_hndl->topic[0]) == 0)
    {
    	// WhoAmI message
    	MqttDeviceWhoamiPublish(temp_dev_hndl);

    }
    else if(strcmp(RecvQue->topic,temp_dev_hndl->topic[1]) == 0)
    {
    	//List Char
    	MqttDeviceListCharPublish(temp_dev_hndl->connection_id);

    }
    else if(strcmp(RecvQue->topic,temp_dev_hndl->topic[2]) == 0)
    {
    	//Get
    	MqttDeviceGetHandler(temp_dev_hndl->connection_id, RecvQue->datastring);

    }
    else if(strcmp(RecvQue->topic,temp_dev_hndl->topic[3]) == 0)
    {
    	//Set
    	MqttDeviceSetHandler(temp_dev_hndl->connection_id, RecvQue->datastring);
    }
    else
    {
    	//Unwanted Topic
    }

}
#else
void MqttDeviceGetPublish(mqtt_gw_messages_t * RecvQue)
{
	return;
}

void MqttDeviceHandler (mqtt_gw_messages_t * RecvQue)
{
	return;
}

#endif

void MqttDeviceCtxClean(void * dev_hdl)
{
	device_handle_t * temp_hdl;
	temp_hdl = (device_handle_t *)dev_hdl;
	if(mqttDevMode != MQTT_QUICKSTART)
	{
		sl_ExtLib_MqttClientUnsub(temp_hdl->clt_ctx, temp_hdl->topic,SUB_TOPIC_COUNT);
	}
	sl_ExtLib_MqttClientDisconnect(temp_hdl->clt_ctx);
	sl_ExtLib_MqttClientCtxDelete(temp_hdl->clt_ctx);
	temp_hdl->connection_id = INVALID_ID;
	temp_hdl->is_connected = false;
	if(temp_hdl->client_id != NULL)
	{
		blefiFree(temp_hdl->client_id);
	}
	return;
}


int MqttDeviceCtxDel(unsigned char dev_con_id)
{
	UINT16 dev_idx = MqttFindIdx(dev_con_id);
	if (dev_idx>=MAX_DEVICE_CONNECT)
	{
		return(-1);
	}
	MqttDeviceCtxClean((void *)(device_hdl+dev_idx));
	return(0);
}





int MqttDeviceCtxAdd(unsigned char dev_con_id,char * devName, unsigned char * bdaddress)
{
	short int dev_idx = -1;
	char bdaddress_str[ADDR_STRING_LEN];
	short int count;

	if (mqtt_device_connected>=MAX_DEVICE_CONNECT)
	{
		return(-1);
	}

	/*
	 * find the device handle instance
	 */

	sprintf(bdaddress_str,"%02x%02x%02x%02x%02x%02x",bdaddress[0],bdaddress[1],bdaddress[2],bdaddress[3],bdaddress[4],bdaddress[5]);

	for(count=(MAX_DEVICE_CONNECT-1);count>=0;count--)
	{

		if(device_hdl[count].connection_id == INVALID_ID)
		{
			dev_idx = count;
		//	break;
		}
		if(mqttDevMode == MQTT_QUICKSTART)
		{
			if(strncmp(bdaddress_str,&(device_hdl[count].client_id[20]),(ADDR_STRING_LEN-1))==0)
			{
				Message("\n\r[MQTT] Mqtt Duplicate device");
				return(-1);
			}
		}
		else
		{
			if(strncmp(bdaddress_str,device_hdl[count].client_id,(ADDR_STRING_LEN-1))==0)
			{
				Message("\n\r[MQTT] Mqtt Duplicate device");
				return(-1);
			}
		}

	}
	if(dev_idx<0)
	{
		//No Slots available in the device handle instance
		return(-1);
	}

	//
	//generate topics and client id using bdaddress
	//

	MqttDeviceBuildTopics(dev_idx, bdaddress);


    //create client context
	device_hdl[dev_idx].clt_ctx =
      sl_ExtLib_MqttClientCtxCreate(&(mqtt_device_cfg.broker_config),
                                    &(mqtt_device_cfg.CallBAcks),
                                    &(device_hdl[dev_idx]));

      //
      // Set Client ID
      //
      sl_ExtLib_MqttClientSet((void*)device_hdl[dev_idx].clt_ctx,
                          SL_MQTT_PARAM_CLIENT_ID,
                          device_hdl[dev_idx].client_id,
                          strlen((char*)(device_hdl[dev_idx].client_id)));

      //
      // Set will Params
      //
      if(mqtt_device_cfg.will_params.will_topic != NULL)
      {
          sl_ExtLib_MqttClientSet((void*)device_hdl[dev_idx].clt_ctx,
                                  SL_MQTT_PARAM_WILL_PARAM,
                                  &(mqtt_device_cfg.will_params),
                                  sizeof(SlMqttWill_t));
      }

      //
      // setting username and password
      //
      if(mqtt_device_cfg.usr_name != NULL)
      {
          sl_ExtLib_MqttClientSet((void*)device_hdl[dev_idx].clt_ctx,
                              SL_MQTT_PARAM_USER_NAME,
                              mqtt_device_cfg.usr_name,
                              strlen((char*)mqtt_device_cfg.usr_name));

          if(mqtt_device_cfg.usr_pwd != NULL)
          {
              sl_ExtLib_MqttClientSet((void*)device_hdl[dev_idx].clt_ctx,
                              SL_MQTT_PARAM_PASS_WORD,
                              mqtt_device_cfg.usr_pwd,
                              strlen((char*)mqtt_device_cfg.usr_pwd));
          }
      }

      //
      // connection to the broker
      //
      if((sl_ExtLib_MqttClientConnect((void*)device_hdl[dev_idx].clt_ctx,
    		  	  	  mqtt_device_cfg.is_clean,
    		  	  	  mqtt_device_cfg.keep_alive_time) & 0xFF) != 0)
      {
          UART_PRINT("\n\r[MQTT] ERROR - Dev-context <%d> Broker connection fail \n\r",dev_idx);
          //delete the context for this connection
          sl_ExtLib_MqttClientCtxDelete(device_hdl[dev_idx].clt_ctx);
		  device_hdl[dev_idx].connection_id = INVALID_ID;
		  device_hdl[dev_idx].is_connected = false;
		  if(device_hdl[dev_idx].client_id!=NULL)
		  {
			  blefiFree(device_hdl[dev_idx].client_id);
		  }
          return(-1);

      }
      else
      {
          UART_PRINT("\n\r[MQTT] Dev-context <%d> broker connection success\n\r", dev_idx);
          device_hdl[dev_idx].is_connected = true;
      }

	  if(mqttDevMode != MQTT_QUICKSTART)
	  {
		  //
		  // Subscribe to topics
		  //
		  if(sl_ExtLib_MqttClientSub((void*)device_hdl[dev_idx].clt_ctx,
				  device_hdl[dev_idx].topic,
									 mqtt_device_cfg.qos, SUB_TOPIC_COUNT) < 0)
		  {
			  UART_PRINT("\n\r[MQTT] Subscription Error for conn no. %d", dev_idx);
			  UART_PRINT("\n\r[MQTT] Disconnecting from the broker\r\n");
			  sl_ExtLib_MqttClientDisconnect(device_hdl[dev_idx].clt_ctx);
			  //delete the context for this connection
			  sl_ExtLib_MqttClientCtxDelete(device_hdl[dev_idx].clt_ctx);
			  device_hdl[dev_idx].connection_id = INVALID_ID;
			  device_hdl[dev_idx].is_connected = false;
			  if(device_hdl[dev_idx].client_id!=NULL)
			  {
				  blefiFree(device_hdl[dev_idx].client_id);
			  }
			  return(-1);

		  }
		  else
		  {
			  int iSub;
			  UART_PRINT("\n\r[MQTT] Client subscribed to following topics:\n\r");
			  for(iSub = 0; iSub < SUB_TOPIC_COUNT; iSub++)
			  {
				  UART_PRINT("%s\n\r", device_hdl[dev_idx].topic[iSub]);
			  }
		  }
	  }
	  device_hdl[dev_idx].connection_id = dev_con_id;
	  device_hdl[dev_idx].is_connected = true;
  	if(strcmp(devName,"CC2650 SensorTag")==0)
  	{
  		if(mqttDevMode == MQTT_QUICKSTART)
  		{
  			sensorTagEnableNoti(dev_idx, device_hdl[dev_idx].connection_id);
  		}
  		Message("\n\r[MQTT] SensorTag Found");
  		device_hdl[dev_idx].is_sensorTag = true;
  	}
  	else
  	{
  		Message("\n\r[MQTT] Not SensorTag");
  		device_hdl[dev_idx].is_sensorTag = false;
  	}
    return(0);
}


void MqttDevClean()
{
	UINT16 dev_idx;
	for(dev_idx=0;dev_idx<MAX_DEVICE_CONNECT;dev_idx++)
	{
		if(device_hdl[dev_idx].is_connected==true)
		{
			sl_ExtLib_MqttClientDisconnect(device_hdl[dev_idx].clt_ctx);
			sl_ExtLib_MqttClientCtxDelete(device_hdl[dev_idx].clt_ctx);
			device_hdl[dev_idx].connection_id = INVALID_ID;
		    if(device_hdl[dev_idx].client_id!=NULL)
		    {
			   blefiFree(device_hdl[dev_idx].client_id);
		    }
			device_hdl[dev_idx].is_sensorTag = false;
			device_hdl[dev_idx].is_setInProgress = false;
			device_hdl[dev_idx].is_getInProgress = false;
		}
	}
}

int MqttDeviceFindHdl(unsigned char idx)
{
	unsigned char i;
	for (i=0;i<MAX_DEVICE_CONNECT;i++)
	{
		if(device_hdl[i].connection_id ==idx )
			return(i);
	}
	return(-1);
}


unsigned short MqttFindIdx(unsigned short connhandle)
{
	unsigned short iCount;
	for(iCount=0;iCount<MAX_DEVICE_CONNECT;iCount++)
	{
		if(device_hdl[iCount].connection_id==connhandle)
		{
			return(iCount);
		}
	}
	return(INVALID_ID);
}


void InitDevModeServer()
{
	unsigned char len;
	mqttCfg_t * mqttCfg;
	mqttCfg = (mqttCfg_t *) blefiCfgRec.mqttCfg;
	mqttDevMode = mqttCfg->mqttDevMode;
	if(mqttDevMode == MQTT_QUICKSTART)
	{
		len = strlen(QS_SERVER);
		mqtt_device_cfg.broker_config.server_info.server_addr = (char*)malloc(len +1);
		if(mqtt_device_cfg.broker_config.server_info.server_addr == NULL)
		{
			HandleError(__FILE__,__LINE__,1);
			return;
		}
		strcpy((char*)mqtt_device_cfg.broker_config.server_info.server_addr,QS_SERVER);
	}
	else
	{
		len = strlen(mqttCfg->demoserver);
		mqtt_device_cfg.broker_config.server_info.server_addr = (char*)malloc(len+1);
		if(mqtt_device_cfg.broker_config.server_info.server_addr == NULL)
		{
			HandleError(__FILE__,__LINE__,1);
			return;
		}
		strcpy((char*)mqtt_device_cfg.broker_config.server_info.server_addr,mqttCfg->demoserver);
	}
	Report("\n\r[MQTT] Dev Mode   : %s",(mqttDevMode==0?"demo":"quickstart"));
	Report("\n\r[MQTT] Dev Server : %s",mqtt_device_cfg.broker_config.server_info.server_addr);
}

#if 0
void MqttDeviceInfo()
{
	unsigned char dev_idx;
	unsigned char i;
	Report("\n\r[MQTT] Gateway Connection Details");
	Report("\n\rServer     : %s",mqtt_device_cfg.broker_config.server_info.server_addr);
	for(dev_idx=0;dev_idx<MAX_DEVICE_CONNECT;dev_idx++)
	{
		if(device_hdl[dev_idx].is_connected==true)
		{
			Report("\n\rDevice Id : %d------",dev_idx);
			Report("\n\rConnection Handle : %d",device_hdl[dev_idx].connection_id);
			Report("\n\rClient Id : %s",device_hdl[dev_idx].client_id);
			Report("\n\rSubscribed Topics");
			for(i=0;i<SUB_TOPIC_COUNT;i++)
			{
				Report("\n\r	%s",device_hdl[dev_idx].pub_topic[i]);
			}
		}

	}

}
#endif

void MqttDevicePubsishQS()
{
	unsigned char count;
	for(count=0;count<MAX_DEVICE_CONNECT;count++)
	{
		if((device_hdl[count].is_connected==true) && (device_hdl[count].is_sensorTag==true))
		{
			publishCC26XXSensorData(&(device_hdl[count]));
		}
	}
}



bool is_anyDeviceMqttConnected()
{
	unsigned char count;
	for(count=0;count<MAX_DEVICE_CONNECT;count++)
	{
		if(device_hdl[count].is_connected == true)
		{
			return(true);
		}
	}
	return(false);
}


#define MQTT_QS_PUBLISH_SECS 		3
extern bool MqttLibInit;
extern bool MqttIpAcquired;
unsigned int mqttQStimeout;
void mqttPublishData()
{
	OsiReturnVal_e retVal;
	if(mqttDevMode == MQTT_QUICKSTART)
	{
		static unsigned int timeout = 0;
		if(MqttLibInit!=true)
		{
			return;
		}
		if (MQTT_QS_PUBLISH_SECS ==0)
		{
			return;
		}

		mqttQStimeout = getGenericTaskTicksPerSec() * MQTT_QS_PUBLISH_SECS;
		timeout++;
		if(is_anyDeviceMqttConnected() == false)
		{
			return;
		}
		if(timeout >=mqttQStimeout)
		{
			mqtt_gw_messages_t * var;
			var = blefiMalloc(sizeof(mqtt_gw_messages_t));
		    if(var == NULL)
		    {
				HandleError(__FILE__,__LINE__,1); // fatal error
				return;
		    }
			timeout = 0;
			var->src = PERIODIC_TASK;
			var->datastring = NULL;
			var->topic = NULL;
			retVal = osi_MsgQWrite(&g_Mqtt_queue,&var,OSI_NO_WAIT);
			if(retVal !=OSI_OK)
			 {
				 Report("\n\rMessage post fail %d",retVal);
			//	This is seen when lot of notifications arrive. Better to ignore.
				 blefiFree(var);
			 }
		}
	}
}



void cfgDevMqttMode(unsigned int mode,char * mqttServer)
{
	mqttCfg_t * mqttCfg;
	mqttCfg = (mqttCfg_t *) blefiCfgRec.mqttCfg;
	mqttCfg->mqttDevMode = mode;
	if(mode!=MQTT_QUICKSTART)
	{
		strncpy(mqttCfg->demoserver,mqttServer,sizeof(mqttCfg->demoserver)-1);
		mqttCfg->demoserver[sizeof(mqttCfg->demoserver)-1] = '\0';
	}
	UpdateBlefiCfg();
	return;

}

unsigned short readDevMqttCfg(char ** mqttServer)
{
	mqttCfg_t * mqttCfg;
	mqttCfg = (mqttCfg_t *) blefiCfgRec.mqttCfg;
	if(mqttDevMode==MQTT_QUICKSTART)
	{
		*mqttServer = QS_SERVER;
	}
	else
	{
		*mqttServer = mqttCfg->demoserver;

	}
	return(mqttCfg->mqttDevMode);
}

