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

char sensor_message[300];
char *charNameMqtt[MAX_CHAR] = {"/Hum/","/Luxo/","/Temp/","/Bar/"};
extern device_handle_t device_hdl[];

//#define MQTT_NOTIFICACTIONS_ENABLE

unsigned int pow(int a, int b)
{
	unsigned int count, result;
	result = 1;
	for(count=0;count<b;count++)
	{
		result = result * a;
	}
	return result;
}


void collectCC26XXSensorData(device_handle_t * temp_dev_hndl,char * topic,char * datastring )
{

	if(strcmp(topic,"/Temp/Data")==0)
	{
		unsigned short int temp=0;
		temp = datastring[1]|(datastring[2]<<8);
		temp_dev_hndl->curSensorVal.object_temp = (int)temp/128;
//		Report("\n\r[MQTT] Object Temp - %d",temp_dev_hndl->curSensorVal.object_temp);

		temp = datastring[3]|(datastring[4]<<8);
		temp_dev_hndl->curSensorVal.ambient_temp = (int)temp/128;
//		Report("\n\r[MQTT] Ambient Temp - %d",temp_dev_hndl->curSensorVal.ambient_temp);
	}
	else if(strcmp(topic,"/Hum/Data")==0)
	{
		unsigned short int temp=0;
		temp = datastring[3]|(datastring[4]<<8);
		temp_dev_hndl->curSensorVal.humidity = (temp*100)/65536;
//		Report("\n\r[MQTT] Humidity - %d",temp_dev_hndl->curSensorVal.humidity);

	}
	else if(strcmp(topic,"/Bar/Data")==0)
	{
		int pressure;
		pressure = (datastring[6]<<16) | (datastring[5] <<8) | datastring[4];
		temp_dev_hndl->curSensorVal.air_pressure = pressure/100;
//		Report("\n\r[MQTT] Air PRessure - %d",temp_dev_hndl->curSensorVal.air_pressure);
	}
	else if(strcmp(topic,"/Luxo/Data")==0)
	{
		unsigned char exponent;
		unsigned int mantissa;
		exponent = (datastring[1]&0xF0)>>4;
		mantissa = (datastring[2])|((datastring[1]&0x0F)<<8);
		temp_dev_hndl->curSensorVal.light = (mantissa/100) * pow(2,exponent);
//		Report("\n\r[MQTT] Light - %d",temp_dev_hndl->curSensorVal.light);
	}
}


void publishCC26XXSensorData(device_handle_t * temp_dev_hndl )
{
	sprintf((char*)sensor_message,
			"{\n\"d\":{\n\"humidity\":\"%d\",\"light\":\"%d\",\"ambient_temp\":\"%d\",\"air_pressure\":\"%d\",\"object_temp\":\"%d\"\n}\n}",
			temp_dev_hndl->curSensorVal.humidity,
			temp_dev_hndl->curSensorVal.light,
			temp_dev_hndl->curSensorVal.ambient_temp,
			temp_dev_hndl->curSensorVal.air_pressure,
			temp_dev_hndl->curSensorVal.object_temp
			);
//	Report("\n\r[MQTT] %s",sensor_message);
	sl_ExtLib_MqttClientSend((void*)temp_dev_hndl->clt_ctx,
						QS_PUBLISH_TOPIC,sensor_message,strlen((char*)sensor_message),QOS0,RETAIN);

#ifndef MQTT_NOTIFICACTIONS_ENABLE
	char pInputString[20];
	unsigned char count;
	for(count=0;count<MAX_CHAR;count++)
	{


		sprintf(pInputString,"%sData",charNameMqtt[count]);
		//Report("\n\rId %d, String %s",temp_dev_hndl->connection_id,pInputString);
		//Report("\n\r[%d]",temp_dev_hndl->connection_id);
		temp_dev_hndl->is_getInProgress=true;
		if(temp_dev_hndl->is_getValid[count]==true)
		{
			GC_GetSet(temp_dev_hndl->connection_id ,pInputString ,(GC_GETSETFLAG)0,1,NULL);
			osi_Sleep(200);
		}

	}
#endif

}




void sensorTagEnableNoti(unsigned char dev_idx,unsigned char con_id)
{
	unsigned char count;
	unsigned char cfg_val = 1;
	char pInputString[20];
	ErrorFlag_t ret_val;
	char max_retries = 25; //5 seconds
	for(count=0;count<MAX_CHAR;count++)
	{
		/*
		 * First enable the sensor by writing to cfg register
		 *
		 * */
		sprintf(pInputString,"%sCfg",charNameMqtt[count]);
		Report("\n\rId %d, String %s",con_id,pInputString);
		device_hdl[dev_idx].is_setInProgress=true;
		ret_val =  GC_GetSet(con_id ,pInputString ,(GC_GETSETFLAG)1,1,&cfg_val);
		if(ret_val == NO_ERROR)
		{
			while(device_hdl[dev_idx].is_setInProgress==true)
			{
				osi_Sleep(200);
				max_retries--;
				if(max_retries == 0)
				{
					break;
				}
			}
			if(max_retries!=0)
			{
				device_hdl[dev_idx].is_getValid[count]=true;
			}
			else
			{
				device_hdl[dev_idx].is_setInProgress=false;
				device_hdl[dev_idx].is_getValid[count]=false;
				Message("\n\r[MQTT] Timeout in Set");
			}
		}
		else
		{
			device_hdl[dev_idx].is_setInProgress=false;
			device_hdl[dev_idx].is_getValid[count]=false;
			Message("\n\r[MQTT] Error in Set");
		}

#if 0
		/*
		 * Increase Notification to 2 secs
		 *
		 * */
		unsigned char period = 200;
		sprintf(pInputString,"%sPeriod",charNameMqtt[count]);
		Report("\n\rId %d, String %s",con_id,pInputString);
		GC_GetSet(con_id ,pInputString ,(GC_GETSETFLAG)1,1,&period);
		device_hdl[dev_idx].is_setInProgress=true;
		while(device_hdl[dev_idx].is_setInProgress==true)
		{
			osi_Sleep(200);
		}
#endif
#ifdef MQTT_NOTIFICACTIONS_ENABLE
		/*
		 * Enable Notification
		 *
		 * */
		unsigned char noti_val = 1;
		sprintf(pInputString,"%sData/noti",charNameMqtt[count]);
		Report("\n\rId %d, String %s",con_id,pInputString);
		GC_GetSet(con_id ,pInputString ,(GC_GETSETFLAG)1,2,&noti_val);
		device_hdl[dev_idx].is_setInProgress=true;
		while(device_hdl[dev_idx].is_setInProgress==true)
		{
			osi_Sleep(200);
		}
#endif
	}
}


