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

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "datatypes.h"

#include "gateway_api.h"
#include "database.h"
#include "gatt.h"
#include "gattservapp.h"
#include "cli_handler.h"
#include "mqtt_api.h"
#include "wifi_api.h"

#define UART_PRINT Report

extern int print_usage(cli_cmd_t *cmd);
extern void CLI_puts(char *str);
extern void RebootMCU();
extern int OtaAddOtaMeta(signed char *pOtaMeta,unsigned char ucOtaMetaLen );
extern void cfgDevMqttMode(unsigned int mode,char * mqttServer);
extern void cfgDevMqttMode(unsigned int mode,char * mqttServer);
void cfgGwMqttMode(unsigned int mode,char * mqttServer);
extern void enforceCC26XXFwChecsum(unsigned int checksum);
extern int configureAutoScan(unsigned int timeout);
extern int readAutoScan();
extern void fillDefaultCfgVal();
extern unsigned short readDevMqttCfg(char ** mqttServer);
extern unsigned short readGwMqttCfg(char ** mqttServer);
extern void UpdateBlefiCfg();
extern void HandleError(unsigned char *, unsigned int, unsigned char error);
extern void GC_PairDevList(fptrPairlist_t fptr);
extern int GC_UnPair(unsigned int idx);
extern void disableAutoconnect();
extern void enableAutoconnect();
//****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS
//****************************************************************************


//*****************************************************************************
//
//!CLI_CB
//!
//! \param
//! \param
//!
//! \return none
//!
void CLI_EventCb(unsigned char EventType, ErrorFlag_t ErrorFlag,void* EventParams)
{
	gwEvent_t* pEvent = (gwEvent_t*)EventParams;

	 switch (EventType)
	 {
		 case GW_EVENT_SCANRSP:
		 {
			 UART_PRINT("\n\rDevice Found -  ");
			 UART_PRINT("%d  %s %s %d %d ",(pEvent->scanRsp.scanIdx - 1),
					 	 	 	 	 	 	pEvent->scanRsp.deviceName ,
					 	 	 	 	 	 	Util_convertBdAddr2Str(pEvent->scanRsp.addr) ,
					 	 	 	 	 	 	pEvent->scanRsp.addrType ,
					 	 	 	 	 	 	pEvent->scanRsp.rssi);
		 }
		 break;

		 case GW_EVENT_SCANCOMPLETE:
		 {

			 UART_PRINT("\n\rScan Complete \n\r");

		 }
		 break;

		 case GW_EVENT_LINKESTABLISH:
		 {
			UART_PRINT("\n\rConnected to Device - ");
			UART_PRINT("%d  %s %s",pEvent->linkEst.connhandle ,
								   pEvent->linkEst.deviceName ,
								   Util_convertBdAddr2Str(pEvent->linkEst.bdAddr));
			UART_PRINT("\n\r");
		 }
		 break;

		 case GW_EVENT_LINKTERMINATE:
		 {
			UART_PRINT("\n\rDisconnected the Device - ");
			UART_PRINT("%d  %s %s reason- 0x%x",pEvent->linkTerm.connhandle ,
												pEvent->linkTerm.deviceName ,
												Util_convertBdAddr2Str(pEvent->linkTerm.bdAddr),
												pEvent->linkTerm.reason);
			UART_PRINT("\n\r");
		 }
		 break;

		 default:
			 break;
	 }
}


bool is_notiEnable = false;
//*****************************************************************************
//
//! CLI_DataHandler
//!
//! \param
//! \param
//!
//! \return none
//!
void CLI_DataCb(unsigned char DataType, ErrorFlag_t ErrorFlag,void* DataParams)
{
	gwData_t* pData = (gwData_t*)DataParams;
	unsigned char i ;

	 switch (DataType)
	 {
		 case GW_DATA_GET:
		 {
			 if(ErrorFlag == NO_ERROR)
			 {
				 if(is_notiEnable == true)
				 {
					 UART_PRINT("\n\r");
					 UART_PRINT(" Device - %d , char - %s ,value -  ",pData->getRsp.connhandle,pData->getRsp.pCurrentGetCharName );
					 for(i=0; i<pData->getRsp.ValueLen ;i++)
					 {
						 UART_PRINT(" 0x%x", pData->getRsp.pValueBuffer[i]);
					 }
					 UART_PRINT("\n\r");
					 is_notiEnable = false;
				     Message(COMMAND_PROMPT);
				 }


			 }
			 else if (ErrorFlag == ERROR_INVALIDCONNHNDL)
			 {
				UART_PRINT("\n\rInvalid connection handle");
				UART_PRINT("\n\r");
				Message(COMMAND_PROMPT);
			 }
			 else if (ErrorFlag == ERROR_INVALIDSTRING)
			 {
				UART_PRINT("\n\rInvalid character string");
				UART_PRINT("\n\r");
				Message(COMMAND_PROMPT);
			 }
			 else
			 {
				UART_PRINT("\n\rERROR in executing GET ");
				UART_PRINT("\n\r");
				Message(COMMAND_PROMPT);
			 }
		 }
		 break;

		 case GW_DATA_SET:
		 {
			if(ErrorFlag == NO_ERROR)
			{
				UART_PRINT("Device - %d ",pData->setRsp.connhandle);
				UART_PRINT("\n\r");
				Message(COMMAND_PROMPT);
			}
			else if (ErrorFlag == ERROR_INVALIDCONNHNDL)
			{
				UART_PRINT("\n\rInvalid connection handle");
				UART_PRINT("\n\r");
				Message(COMMAND_PROMPT);
			}
			else if (ErrorFlag == ERROR_INVALIDSTRING)
			{
				UART_PRINT("\n\rInvalid character string");
				UART_PRINT("\n\r");
				Message(COMMAND_PROMPT);
			}
			else
			{
				 UART_PRINT("\n\rERROR in executing SET ");
				 UART_PRINT("\n\r");
	             Message(COMMAND_PROMPT);
			}
		 }
		 break;

		 default:
			 break;
	 }
}


//*****************************************************************************
//
//! DoDeviceDiscovery
//!
//! @param  none
//!
//! @return none
//!
//! @brief  Parse the received DeviceDisc String
//
//*****************************************************************************

int DoScan(cli_cmd_t *cmd,int argc,char *argv[])
{
   int devices_found;

	devices_found = GC_Scan(GW_CALL_BLOCKING);
	if (devices_found < 0)
	{
		UART_PRINT("\n\rERROR - Discovery already in progress");
	}
	else
	{
		UART_PRINT("\n\rDevices discovered : %d", devices_found);
		UART_PRINT("\n\rUse <list> command to display the devices \n\n\r");
	}

   return devices_found;
}

#if 0
//*****************************************************************************
//
//! DoStopScan
//!
//! @param  none
//!
//! @return none
//!
//! @brief  Parse the received DoStopScan String
//
//*****************************************************************************

int DoStopScan(cli_cmd_t *cmd,int argc,char *argv[])
{
   GC_StopScan(GW_CALL_BLOCKING);

   UART_PRINT("\n\r Scanning cancelled");

   return 0;
}
#endif

//*****************************************************************************
//
//! DoDevList
//!
//! @param  none
//!
//! @return none
//!
//! @brief  Parse the received Dev List String
//
//*****************************************************************************
void fnPtrCLIPRINT(unsigned char devIndex, char * devName , unsigned char *bdAddr, unsigned char addrType);
int DoDevList(cli_cmd_t *cmd,int argc,char *argv[])
{
	UART_PRINT("\n\rScanned Devices");
	UART_PRINT("\n\r");
	GC_DeviceScanList(fnPtrCLIPRINT);
	UART_PRINT("\n\r");

	UART_PRINT("\n\rConnected Devices");
	UART_PRINT("\n\r");
	GC_DeviceConnList(fnPtrCLIPRINT);
	UART_PRINT("\n\r");

	return 0;
}


//*****************************************************************************
//
//! fnPtrCLIPRINT
//!
//! @param  unsigned char devIndex, unsigned char * devName , unsigned char *bdAddr,unsigned char addrType
//!
//! @return none
//!
//! @brief
//
//*****************************************************************************


void fnPtrCLIPRINT(unsigned char devIndex, char * devName , unsigned char *bdAddr,unsigned char addrType)
{
	UART_PRINT("\n\r %d     %s     %s      %d",devIndex ,devName, Util_convertBdAddr2Str(bdAddr), addrType);

}
//*****************************************************************************
//
//! ParseLinkEstablishString
//!
//! @param  none
//!
//! @return none
//!
//! @brief  Parse the received Link Establish String
//
//*****************************************************************************
int DoLinkEstablish(cli_cmd_t *cmd,int argc,char *argv[])
{
    short int uConnId;
    unsigned char bdAddr[B_ADDR_LEN];
	param_LinkEstablish_t LinkEstablish;

   if(argc < 2)
   {
	   goto err;
   }

    LinkEstablish.DeviceId = strtoul(argv[1],NULL,10);

    uConnId = GC_LinkEstablish(LinkEstablish.DeviceId , GW_CALL_BLOCKING , (unsigned char*)bdAddr );

    if(uConnId >= 0)
    {
    	UART_PRINT("\n\rDevice successfully connected , Conn Id - %d BD addr -%s\n\r",uConnId ,Util_convertBdAddr2Str(bdAddr));
    }
    else
    {
    	UART_PRINT("\n\rDevice connection not successful");
    }

    return 0;

 err:
    print_usage(cmd);
    return -1;
}



//*****************************************************************************
//
//! DoLinkTerminate
//!
//! @param  Device Id
//!
//! @return none
//!
//! @brief  Parse the Link Terminate String
//
//*****************************************************************************
int DoLinkTerminate(cli_cmd_t *cmd,int argc,char *argv[])
{
    signed int linkt_status;

	param_LinkTerminate_t LinkTerminate;

   if(argc < 2)
   {
       goto err;
   }

   LinkTerminate.DeviceId = strtoul(argv[1],NULL,10);

   linkt_status =GC_LinkTerminate(LinkTerminate.DeviceId, GW_CALL_BLOCKING);

   if(linkt_status == 0)
   {
	   UART_PRINT("\n\rDevice successfully disconnected ");
   }
   else
   {
	   UART_PRINT("\n\rDevice termination not successful");
   }

   return 0;

err:
   print_usage(cmd);
   return -1;
}

void fnPtrDevCharListCLIPRINT(unsigned short int connhandle, unsigned short int char_index, char* charName, unsigned char getsetflag)
{
	UART_PRINT("\n\r %d   %s   ",char_index, charName);

	if((gattPropRead(getsetflag) !=0) && (gattPropWrite(getsetflag) !=0) )
	{
		UART_PRINT("[GET/SET]");
	}
	else if((gattPropRead(getsetflag) !=0) )
	{
		UART_PRINT("[GET]");
	}
	else if((gattPropWrite(getsetflag) !=0) )
	{
		UART_PRINT("[SET]");

	}
	else
	{
		// No permissions to read or write
		UART_PRINT("[NO GET/SET PERMITTED]");
	}
}


//*****************************************************************************
//
//! DoGetDevCharList
//!
//! @param  none
//!
//! @return none
//!
//! @brief  Parse the received GetDevCharList String
//
//*****************************************************************************
int DoListChar(cli_cmd_t *cmd,int argc,char *argv[])
{

	param_GetDevCharList_t GetDevCharList;

	if(argc < 2)
	{
		goto err;
	}

	GetDevCharList.DeviceId = strtoul(argv[1],NULL,10);

	UART_PRINT("\n\rDevice chars are  \n\r");
	GC_GetDevCharList(GetDevCharList.DeviceId , fnPtrDevCharListCLIPRINT);
	UART_PRINT("\n\r");

	return 0;

err:
	print_usage(cmd);
	return -1;
}


//*****************************************************************************
//
//! ParseGetString
//!
//! @param  none
//!
//! @return none
//!
//! @brief  Parse the Link Terminate String
//
//*****************************************************************************
int DoGet(cli_cmd_t *cmd,int argc,char *argv[])
{
    unsigned short int connhandle ;
    char* pInputString;

	if(argc < 3)
	{
		goto err;
	}

    connhandle = strtoul(argv[1],NULL,10);
    pInputString = argv[2];

    if(connhandle > MAX_DEVICE_DB_SUPPORTED)
    {
    	HandleError(__FILE__,__LINE__,1);
    	return (-1);
    }
    is_notiEnable = true;
    if(GC_GetSet(connhandle ,pInputString , CHAR_GET,0,NULL) != NO_ERROR)
    {
    	Message("\n\r GET Error");
    }

	return 0;

err:
	print_usage(cmd);
	return -1;
}


//*****************************************************************************
//
//! DoSet
//!
//! @param  none
//!
//! @return none
//!
//! @brief  Parse the Set String
//
//*****************************************************************************
int DoSet(cli_cmd_t *cmd,int argc,char *argv[])
{
    unsigned short int connhandle ;
    char* pInputString;
    //unsigned char Value[] ;
    unsigned int Value ;
    unsigned long ValueLen;

	if(argc < 5)
	{
		goto err;
	}

    connhandle = strtoul(argv[1],NULL,10);
    pInputString = argv[2];
    ValueLen 	= strtoul(argv[3],NULL,10);
    Value = strtoul(argv[4],NULL,10);
    //pValue = (unsigned char *)&Value;

    if(connhandle > MAX_DEVICE_DB_SUPPORTED)
    {
    	HandleError(__FILE__,__LINE__,1);
    	return (-1);
    }

    if(GC_GetSet(connhandle,pInputString , CHAR_SET , ValueLen , (unsigned char *)&Value)!=NO_ERROR)
    {
       	Message("\n\r SET Error");
    }

	return 0;

err:
	print_usage(cmd);
	return -1;
}


#if 0
//*****************************************************************************
//
//! DoSet
//!
//! @param  none
//!
//! @return none
//!
//! @brief  Parse the Set String
//
//*****************************************************************************
int DoEraseBonds(cli_cmd_t *cmd,int argc,char *argv[])
{
	GC_EraseAllBondings();

	return 0;
#if 0
err:
	print_usage(cmd);
	return -1;
#endif
}
#endif

// Wifi Configuration  Functions


//*****************************************************************************
//
//! DoWlanconnect
//!
//! @param  none
//!
//! @return none
//!
//! @brief  Parse the Wlanconnect String
//
//*****************************************************************************
int DoWlanconnect(cli_cmd_t *cmd,int argc,char *argv[])
{

	signed char *pcSsid;
	unsigned long ulSsidLen;
	unsigned char ucSecType;
	signed char *pcKey;
	unsigned long ulKeyLen;

	if(argc < 2)
	{
		goto err;
	}

	pcSsid = (signed char *)(argv[1]);
	ulSsidLen = (unsigned char)strlen((const char *)pcSsid);
	pcKey = (signed char *)(argv[2]);
	ulKeyLen = (unsigned char)strlen((const char *)pcKey);


	if(argc == 2)
	{
		ucSecType = 0; // open
	}
	else
	{
		ucSecType = 2; // WPA
	}

	if(WlanConnectAP(pcSsid , ulSsidLen,0,ucSecType,NULL,0 ,pcKey,ulKeyLen) !=0);
	{
		Message("\n\r Wlan Connect Failure");
	}
	return 0;

err:
	print_usage(cmd);
	return -1;
}


int DoWlanDisconnect(cli_cmd_t *cmd,int argc,char *argv[])
{
    long lRetVal = 0;

	lRetVal = WlanDisconnectAP();
	return lRetVal;
}



// not needed
int DoCc26xxFwUpdate(cli_cmd_t *cmd,int argc,char *argv[])
{

	enforceCC26XXFwChecsum(0);
	Message("\n\r Please restart the System to enable CC26XX Firmware update");
	return 0;
}



int DoReset(cli_cmd_t *cmd,int argc,char *argv[])
{
	RebootMCU();
	return 0;
}

int DoAddOtaMeta(cli_cmd_t *cmd,int argc,char *argv[])
{
	signed char *pcOtaMeta;
	unsigned char ucOtaMetaLen;

	if(argc < 2)
	{
		goto err;
	}

	pcOtaMeta = (signed char *)(argv[1]);
	ucOtaMetaLen = (unsigned char)strlen((const char *)pcOtaMeta);

	if(OtaAddOtaMeta(pcOtaMeta,ucOtaMetaLen)!=0)
	{
		Report("\n\rFAIL - Could not add ota meta data %d",ucOtaMetaLen);
		return -1;
	}
	return 0;

err:
	print_usage(cmd);
	return -1;
}


int triggerOta(cli_cmd_t *cmd,int argc,char *argv[])
{
	OtaDownload();
	return 0;
}

#if 0
int DoMqttReset(cli_cmd_t *cmd,int argc,char *argv[])
{
	MqttIpAcquiredPostMsg();
	return 0;
}

int DoMqttGwInfo(cli_cmd_t *cmd,int argc,char *argv[])
{
	MqttGwInfo();
	return 0;
}

int DoMqttDevInfo(cli_cmd_t *cmd,int argc,char *argv[])
{
	MqttDeviceInfo();
	return 0;
}
#endif
int DoAutoScan(cli_cmd_t *cmd,int argc,char *argv[])
{
    unsigned int timeout ;
    int ret;

	if(argc > 2)
	{
		goto err;
	}
	else if(argc == 1)
	{
		ret = readAutoScan();
		if(ret==0)
		{
			Message("\n\rAutoscan Disabled");
		}
		else
		{
			Report("\n\rAutoscan period is  %d secs",ret);
		}
		return 0;

	}

    timeout = strtoul(argv[1],NULL,10);
    ret = configureAutoScan(timeout);
    if(ret<0)
    {
    	goto err;
    }
    else
    {
    	if(ret==0)
    	{
    		Message("\n\rAutoscan Disabled");
    	}
    	else
    	{
    		Report("\n\rAutoscan period changed to %d secs",ret);
    	}
    	return 0;

    }
err:
	print_usage(cmd);
	return -1;
}

int DoMqttAddDev(cli_cmd_t *cmd,int argc,char *argv[])
{
	unsigned char dev_con_id;
	atolong(argv[1], (unsigned long *)&dev_con_id);
	MqttDeviceCtxAdd(dev_con_id, (unsigned char *)argv[2]);
	return 0;
}

int DoMqttDelDev(cli_cmd_t *cmd,int argc,char *argv[])
{
	unsigned char dev_con_id;
	atolong(argv[1],(unsigned long *) &dev_con_id);
	MqttDeviceCtxDel(dev_con_id);
	return 0;
}

int DomqttGwMode(cli_cmd_t *cmd,int argc,char *argv[])
{
    unsigned int mode ;
    char * mqttServer;


	if((argc>3))
	{
		goto err;
	}
	if(argc==1)
	{
		/*
		 * Display the Mqtt Mode configuration
		 * */
		mode = readGwMqttCfg(&mqttServer);
		Report("\n\rMqtt Gateway Mode = %s",((mode==0)?"demo":"quickstart"));
		Report("\n\rMqtt Server : %s", mqttServer);
		return 0;
	}
	mode = strtoul(argv[1],NULL,10);
	if (mode==0)
	{
		/*
		 *Demo mode
		 *
		 * */
		if(argc==3)
		{
			mqttServer = argv[2];
			if(strlen(mqttServer)>=100)
			{
				Message("\n\rServer name should not exceed 100 characters");
				return -1;
			}
		}
		else
		{
			goto err;
		}
	}
	else if(mode==1)
	{
		/*
		 *
		 * Quickstart mode
		 * */
		mqttServer = NULL;

	}
	else
	{
		goto err;
	}
	cfgGwMqttMode(mode,mqttServer);
	Message("\n\rMqtt Mode set, please reset the device for it to take effect");
	return 0;
err:
	print_usage(cmd);
	return -1;
}

int DomqttDevMode(cli_cmd_t *cmd,int argc,char *argv[])
{
    unsigned int mode ;
    char * mqttServer;


	if((argc>3))
	{
		goto err;
	}
	if(argc==1)
	{
		/*
		 * Display the Mqtt Mode configuration
		 * */
		mode = readDevMqttCfg(&mqttServer);
		Report("\n\rMqtt Gateway Mode = %s",((mode==0)?"demo":"quickstart"));
		Report("\n\rMqtt Server : %s", mqttServer);
		return 0;
	}
	mode = strtoul(argv[1],NULL,10);
	if (mode==0)
	{
		/*
		 *Demo mode
		 *
		 * */
		if(argc==3)
		{
			mqttServer = argv[2];
			if(strlen(mqttServer)>=100)
			{
				Message("\n\rServer name should not exceed 100 characters");
				return -1;
			}
		}
		else
		{
			goto err;
		}
	}
	else if(mode==1)
	{
		/*
		 *
		 * Quickstart mode
		 * */
		mqttServer = NULL;

	}
	else
	{
		goto err;
	}
	cfgDevMqttMode(mode,mqttServer);
	Message("\n\rMqtt Mode set, please reset the device for it to take effect");
	return 0;
err:
	print_usage(cmd);
	return -1;
}

int DoDefault(cli_cmd_t *cmd,int argc,char *argv[])
{
	fillDefaultCfgVal();
	UpdateBlefiCfg();
	osi_Sleep(1000); //Wait until the file is written
	Message("\n\rBlefi Default mode loaded, please restart device");
	return(0);
}

void fnPtrPairPRINT(unsigned char pairIndex, unsigned char *bdAddr)
{
	UART_PRINT("\n\r %d     %s",pairIndex ,Util_convertBdAddr2Str(bdAddr));

}

//*****************************************************************************
//
//! DoDevList
//!
//! @param  none
//!
//! @return none
//!
//! @brief  Parse the received Dev List String
//
//*****************************************************************************
void fnPtrPairPRINT(unsigned char pairIndex, unsigned char *bdAddr);
int DoPairList(cli_cmd_t *cmd,int argc,char *argv[])
{
	UART_PRINT("\n\rPaired Devices");
	UART_PRINT("\n\r");

	GC_PairDevList(fnPtrPairPRINT);
	UART_PRINT("\n\r");

	return 0;
}


int DoUnPair(cli_cmd_t *cmd,int argc,char *argv[])
{
	unsigned char idx;
	if((argc!=2))
	{
		goto err;
	}
	idx = strtoul(argv[1],NULL,10);
	if (idx>=10)
	{
		Message("\n\r Index out of bounds");
		goto err;
	}
	UART_PRINT("\n\rUnpairing Pair-Id %d",idx);
	if(GC_UnPair(idx)==0)
	{
		Message("\n\rDevice Successfully unpaired");
		return 0;
	}
	Message("\n\rDevice not found - Unpair Failure");

err:
	print_usage(cmd);
	return -1;
}

int DoAutoconnect(cli_cmd_t *cmd,int argc,char *argv[])
{
	unsigned char idx;
	if((argc!=2))
	{
		goto err;
	}
	idx = strtoul(argv[1],NULL,10);
	if (idx==0)
	{
		disableAutoconnect();
		Message("\n\rAuto connect disabled");
		return 0;
	}
	else
	{
		enableAutoconnect();
		Message("\n\rAuto connect enabled");
		return 0;
	}

err:
	print_usage(cmd);
	return -1;
}

