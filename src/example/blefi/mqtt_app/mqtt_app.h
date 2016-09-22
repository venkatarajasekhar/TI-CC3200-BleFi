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
// mqtt_app.h - Defines and Macros for mqtt client application
//
//*****************************************************************************

#ifndef __MQTT_APP_H_
#define __MQTT_APP_H_

/*Operate Lib in MQTT 3.1 mode.*/
#define MQTT_3_1_1              false /*MQTT 3.1.1 */
#define MQTT_3_1                true /*MQTT 3.1*/

#define WILL_TOPIC              "Client"
#define WILL_MSG                "Client Stopped"
#define WILL_QOS                QOS2
#define WILL_RETAIN             false

/*Defining Broker IP address and port Number*/
#define PORT_NUMBER             1883

#define MAX_BROKER_CONN         1

#define SERVER_MODE             MQTT_3_1
/*Specifying Receive time out for the Receive task*/
#define RCV_TIMEOUT             30

/*Background receive task priority*/
#define TASK_PRIORITY           3

/* Keep Alive Timer value*/
#define KEEP_ALIVE_TIMER        25

/*Clean session flag*/
#define CLEAN_SESSION           true

/*Retain Flag. Used in publish message. */
#define RETAIN                  1

/*Defining Number of topics for subscription*/
#define GW_TOPIC_COUNT      4

/*Defining QOS levels*/
#define QOS0                    0
#define QOS1                    1
#define QOS2                    2

/*Spawn task priority and OSI Stack Size*/
#define OSI_STACK_SIZE          2048
#define UART_PRINT              Report

typedef struct connection_config{
    SlMqttClientCtxCfg_t broker_config;
    void *clt_ctx;
    char *client_id;
    unsigned char *usr_name;
    unsigned char *usr_pwd;
    bool is_clean;
    unsigned int keep_alive_time;
    SlMqttClientCbs_t CallBAcks;
    int num_topics;
    char *topic[GW_TOPIC_COUNT];
    unsigned char qos[GW_TOPIC_COUNT];
    SlMqttWill_t will_params;
    bool is_connected;
}connection_config_t;

typedef enum events
{
	WIFI_IP_ACQUIRED,
	MQTT_BLEFI_FIND,
	MQTT_GW_SCAN,
	MQTT_GW_LINKE,
	MQTT_GW_LINKT,
	MQTT_BROKER_DISCONNECTION,
	BLE_DEVICE_CONNECTED,
	BLE_DEVICE_DISCONNECTED,
	BLE_DEVICE_LIST,
	BLE_DEVICE_GET,
	BLE_DEVICE_SET,
	BLE_DEVICE_WHOAMI
}mqtt_messages;

typedef enum source
{
	MQTT_BLEFI,
	MQTT_DEVICE,
	BLE_EVENT,
	BLE_DATA,
	CONNECT_MSG,
	CLEANUP_MSG,
	PERIODIC_TASK
}mqtt_source;


#define	MQTT_DEMO 				0
#define MQTT_QUICKSTART			1
#define MQTT_REGISTERED			2

#define MAX_CHAR 4
#define SUB_TOPIC_COUNT 4
#define PUB_TOPIC_COUNT 4
#define MAX_DEVICE_CONNECT 3

void ReadMQTTMode();

typedef struct sensor_values{
	int humidity;
	int light;
	int ambient_temp;
	int air_pressure;
	int object_temp;
}sensor_values_t;


typedef struct mqtt_gw_messages
{
	mqtt_source src;
	mqtt_messages evnt;
	char data;
	char* topic;
	char* datastring;
	void* app_handle;
}mqtt_gw_messages_t;

typedef struct mqttCfg_s
{
	unsigned short mqttGwMode;
	unsigned short mqttDevMode;
	char demoserver[100];
}mqttCfg_t;

/*
 * Device Handle to be sent to the mqtt library while connecting
 * */

typedef struct device_handle{
	void *clt_ctx;
    char * client_id; //bdaddress is used as client_id
    bool is_connected;
    unsigned int connection_id;
    char *topic[SUB_TOPIC_COUNT];
    char *pub_topic[PUB_TOPIC_COUNT];
    sensor_values_t curSensorVal;
    bool is_getValid[MAX_CHAR];
    bool is_sensorTag;
    bool is_setInProgress;
    bool is_getInProgress;
}device_handle_t;

#define ADDR_STRING_LEN 13 //used for MAC address and BD address
#define QS_PUBLISH_TOPIC "iot-2/evt/status/fmt/json"
#define QS_SERVER "quickstart.messaging.internetofthings.ibmcloud.com"
#define DEFAULT_DEMO_SERVER "192.84.45.44"



void MqttDevDataCB(uint8 DataType, ErrorFlag_t ErrorFlag,void* DataParams);
void MqttGwEventCB(unsigned char EventType, ErrorFlag_t ErrorFlag, void* EventParams);
int MqttDeviceCtxAdd(unsigned char dev_con_id,char* devName,unsigned char * bdaddress);
void MqttDeviceGetPublish(mqtt_gw_messages_t * RecvQue);
void MqttDevClean();
int MqttDeviceCtxDel(unsigned char dev_con_id);
void MqttDeviceHandler (mqtt_gw_messages_t * RecvQue);
void InitDevModeServer();
void MqttDeviceCtxClean(void * dev_hdl);
void MqttDevicePubsishQS();

extern void UpdateBlefiCfg();

#endif /* __MQTT_IBM_H_ */
