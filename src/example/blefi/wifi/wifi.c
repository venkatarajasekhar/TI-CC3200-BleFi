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
//
// wifi.c - wifi/wlan related functions for CC3200 wireless MCU
//
//
//
//*****************************************************************************

//****************************************************************************
//
//! \
//! @{
//
//****************************************************************************

// Standard includes
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "datatypes.h"
// Simplelink includes
#include "simplelink.h"
#include "netcfg.h"

// Driverlib includes
#include "hw_ints.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "interrupt.h"
#include "utils.h"
#include "rom.h"
#include "rom_map.h"
#include "prcm.h"
#include "pin.h"

// OS includes
#include "osi.h"

// Common interface includes
#include "blefi_gpio_if.h"
#include "uart_if.h"
#include "common.h"
#include "gpio.h"

// App Includes
#include "device_status.h"
#include "smartconfig.h"
#include "pinmux.h"
#include "wlan.h"
#include "cc3200.h"
#include "mqtt_api.h"



#define APPLICATION_VERSION              "1.1.0"
#define APP_NAME                         "Wifi Config"
#define WIFI_TASK_PRIORITY				(3)
#define OSI_STACK_SIZE                   2048
#define AP_SSID_LEN_MAX                 32
#define SH_GPIO_3                       3       /* P58 - Device Mode */
#define AUTO_CONNECTION_TIMEOUT_COUNT   20      /*  Sec */
#define SMART_CONFIG_TIMEOUT_COUNT   	50     /* 10 Sec */
#define SL_STOP_TIMEOUT                 200
#define SCAN_TABLE_SIZE           20

bool is_slReady = false;


//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
static const char pcDigits[] = "0123456789";
unsigned long  g_ulStatus = 0;//SimpleLink Status
unsigned char  g_ucConnectionSSID[SSID_LEN_MAX+1]; //Connection SSID
unsigned char  g_ucConnectionBSSID[BSSID_LEN_MAX]; //Connection BSSID


 // do we need these?

 signed char g_cWlanSSID[AP_SSID_LEN_MAX];
 signed char g_cWlanSecurityKey[50];
 unsigned char g_ucConnectedToConfAP = 0, g_ucProvisioningDone = 0;
 unsigned char g_ucPriority = 0;


#define HTTP_ENABLED TRUE
#ifdef HTTP_ENABLED
extern void Http_Init();

extern void HttpGetToken(SlHttpServerEvent_t *pSlHttpServerEvent,
        SlHttpServerResponse_t *pSlHttpServerResponse);

extern void HttpPostToken( SlHttpServerEvent_t *pSlHttpServerEvent,
        SlHttpServerResponse_t *pSlHttpServerResponse);
#endif


extern void otaSignalWLANConnect();
extern void BlinkLed(LedNum LedInstance,unsigned short iBlinkCount, LedBlinkrate iBlinkRate);

//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************


//*****************************************************************************
//
//! itoa
//!
//!    @brief  Convert integer to ASCII in decimal base
//!
//!     @param  cNum is input integer number to convert
//!     @param  cString is output string
//!
//!     @return number of ASCII parameters
//!
//!
//
//*****************************************************************************
unsigned short itoa(char cNum, char *cString)
{
    char* ptr;
    char uTemp = cNum;
    unsigned short length;

    // value 0 is a special case
    if (cNum == 0)
    {
        length = 1;
        *cString = '0';

        return length;
    }

    // Find out the length of the number, in decimal base
    length = 0;
    while (uTemp > 0)
    {
        uTemp /= 10;
        length++;
    }

    // Do the actual formatting, right to left
    uTemp = cNum;
    ptr = cString + length;
    while (uTemp > 0)
    {
        --ptr;
        *ptr = pcDigits[uTemp % 10];
        uTemp /= 10;
    }

    return length;
}




//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- Start
//*****************************************************************************


//*****************************************************************************
//
//! \brief The Function Handles WLAN Events
//!
//! \param[in]  pWlanEvent - Pointer to WLAN Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    if(pWlanEvent == NULL)
    {
        UART_PRINT("Null pointer\n\r");
        LOOP_FOREVER();
    }
    switch(pWlanEvent->Event)
    {
        case SL_WLAN_CONNECT_EVENT:
        {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);

            //
            // Information about the connected AP (like name, MAC etc) will be
            // available in 'slWlanConnectAsyncResponse_t'
            // Applications can use it if required
            //
            //  slWlanConnectAsyncResponse_t *pEventData = NULL;
            // pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
            //

            // Copy new connection SSID and BSSID to global parameters
            memcpy(g_ucConnectionSSID,pWlanEvent->EventData.
                   STAandP2PModeWlanConnected.ssid_name,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.ssid_len);
            memcpy(g_ucConnectionBSSID,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.bssid,
                   SL_BSSID_LENGTH);

            UART_PRINT("\n\r");
            UART_PRINT("[WLAN EVENT] Device Connected to the AP: %s , "
                       "BSSID: %x:%x:%x:%x:%x:%x\n\r",
                      g_ucConnectionSSID,g_ucConnectionBSSID[0],
                      g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                      g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                      g_ucConnectionBSSID[5]);
            UART_PRINT("\n\r");

            TurnOffLed(LED_1);
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT:
        {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            // If the user has initiated 'Disconnect' request,
            //'reason_code' is SL_USER_INITIATED_DISCONNECTION
            if(SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
            {
                UART_PRINT("[WLAN EVENT] Device disconnected from the AP: %s, "
                           "BSSID: %x:%x:%x:%x:%x:%x on application's "
                           "request \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            else
            {
                UART_PRINT("[WLAN ERROR] Device disconnected from the AP AP: %s, "
                           "BSSID: %x:%x:%x:%x:%x:%x on an ERROR..!! \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
            memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
        }
        break;

        case SL_WLAN_STA_CONNECTED_EVENT:
        {
            // when device is in AP mode and any client connects to device cc3xxx
            //SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            //CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION_FAILED);

            //
            // Information about the connected client (like SSID, MAC etc) will
            // be available in 'slPeerInfoAsyncResponse_t' - Applications
            // can use it if required
            //
            // slPeerInfoAsyncResponse_t *pEventData = NULL;
            // pEventData = &pSlWlanEvent->EventData.APModeStaConnected;
            //

            UART_PRINT("[WLAN EVENT] Station connected to device\n\r");
        }
        break;

        case SL_WLAN_STA_DISCONNECTED_EVENT:
        {
            // when client disconnects from device (AP)
            //CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            //CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_LEASED);

            //
            // Information about the connected client (like SSID, MAC etc) will
            // be available in 'slPeerInfoAsyncResponse_t' - Applications
            // can use it if required
            //
            // slPeerInfoAsyncResponse_t *pEventData = NULL;
            // pEventData = &pSlWlanEvent->EventData.APModestaDisconnected;
            //
            UART_PRINT("[WLAN EVENT] Station disconnected from device\n\r");
        }
        break;

        case SL_WLAN_SMART_CONFIG_COMPLETE_EVENT:
        {
            //SET_STATUS_BIT(g_ulStatus, STATUS_BIT_SMARTCONFIG_START);

            //
            // Information about the SmartConfig details (like Status, SSID,
            // Token etc) will be available in 'slSmartConfigStartAsyncResponse_t'
            // - Applications can use it if required
            //
            //  slSmartConfigStartAsyncResponse_t *pEventData = NULL;
            //  pEventData = &pSlWlanEvent->EventData.smartConfigStartResponse;
            //

        }
        break;

        case SL_WLAN_SMART_CONFIG_STOP_EVENT:
        {
            // SmartConfig operation finished
            //CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_SMARTCONFIG_START);

            //
            // Information about the SmartConfig details (like Status, padding
            // etc) will be available in 'slSmartConfigStopAsyncResponse_t' -
            // Applications can use it if required
            //
            // slSmartConfigStopAsyncResponse_t *pEventData = NULL;
            // pEventData = &pSlWlanEvent->EventData.smartConfigStopResponse;
            //
        }
        break;

        default:
        {
            UART_PRINT("[WLAN EVENT] Unexpected event [0x%x]\n\r",
                       pWlanEvent->Event);
        }
        break;
    }
}

//*****************************************************************************
//
//! \brief This function handles network events such as IP acquisition, IP
//!           leased, IP released etc.
//!
//! \param[in]  pNetAppEvent - Pointer to NetApp Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    if(pNetAppEvent == NULL)
    {
        UART_PRINT("Null pointer\n\r");
        LOOP_FOREVER();
    }

    switch(pNetAppEvent->Event)
    {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
        {
            SlIpV4AcquiredAsync_t *pEventData = NULL;

            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            //Ip Acquired Event Data
            pEventData = &pNetAppEvent->EventData.ipAcquiredV4;

            UART_PRINT("[NETAPP EVENT] IP Acquired: IP=%d.%d.%d.%d , "
                       "Gateway=%d.%d.%d.%d\n\r",
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,3),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,2),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,1),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,0),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,3),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,2),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,1),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,0));

            UNUSED(pEventData);

            MqttIpAcquiredPostMsg();
            otaSignalWLANConnect();


        }
        break;

        case SL_NETAPP_IP_LEASED_EVENT:
        {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_LEASED);

            //
            // Information about the IP-Leased details(like IP-Leased,lease-time,
            // mac etc) will be available in 'SlIpLeasedAsync_t' - Applications
            // can use it if required
            //
            // SlIpLeasedAsync_t *pEventData = NULL;
            // pEventData = &pNetAppEvent->EventData.ipLeased;
            //

        }
        break;

        case SL_NETAPP_IP_RELEASED_EVENT:
        {
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_LEASED);

            //
            // Information about the IP-Released details (like IP-address, mac
            // etc) will be available in 'SlIpReleasedAsync_t' - Applications
            // can use it if required
            //
            // SlIpReleasedAsync_t *pEventData = NULL;
            // pEventData = &pNetAppEvent->EventData.ipReleased;
            //
        }
		break;

        default:
        {
            UART_PRINT("[NETAPP EVENT] Unexpected event [0x%x] \n\r",
                       pNetAppEvent->Event);
        }
        break;
    }
}


//*****************************************************************************
//
//! This function gets triggered when HTTP Server receives Application
//! defined GET and POST HTTP Tokens.
//!
//! \param pSlHttpServerEvent - Pointer indicating http server event
//! \param pSlHttpServerResponse - Pointer indicating http server response
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pSlHttpServerEvent,
        SlHttpServerResponse_t *pSlHttpServerResponse)
{

    switch (pSlHttpServerEvent->Event)
    {
        case SL_NETAPP_HTTPGETTOKENVALUE_EVENT:
        {

#ifdef HTTP_ENABLED
        	HttpGetToken(pSlHttpServerEvent,
                    pSlHttpServerResponse);
#endif
		}
        break;

        case SL_NETAPP_HTTPPOSTTOKENVALUE_EVENT:
        {

#ifdef HTTP_ENABLED
        	HttpPostToken(pSlHttpServerEvent,
                    pSlHttpServerResponse);
#endif
            break;
        }
      default:
          break;
    }
}


//*****************************************************************************
//
//! \brief This function handles General Events
//!
//! \param[in]     pDevEvent - Pointer to General Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    if(pDevEvent == NULL)
    {
        UART_PRINT("Null pointer\n\r");
        LOOP_FOREVER();
    }

    //
    // Most of the general errors are not FATAL are are to be handled
    // appropriately by the application
    //
    UART_PRINT("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
               pDevEvent->EventData.deviceEvent.status,
               pDevEvent->EventData.deviceEvent.sender);
}


//*****************************************************************************
//
//! This function handles socket events indication
//!
//! \param[in]      pSock - Pointer to Socket Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    if(pSock == NULL)
    {
        UART_PRINT("Null pointer\n\r");
        LOOP_FOREVER();
    }
    //
    // This application doesn't work w/ socket - Events are not expected
    //
    switch( pSock->Event )
    {
        case SL_SOCKET_TX_FAILED_EVENT:
            switch( pSock->socketAsyncEvent.SockTxFailData.status)
            {
                case SL_ECLOSE:
                    UART_PRINT("[SOCK ERROR] - close socket (%d) operation "
                                "failed to transmit all queued packets\n\n",
                                    pSock->socketAsyncEvent.SockTxFailData.sd);
                    break;
                default:
                    UART_PRINT("[SOCK ERROR] - TX FAILED  :  socket %d , reason "
                                "(%d) \n\n",
                                pSock->socketAsyncEvent.SockTxFailData.sd, pSock->socketAsyncEvent.SockTxFailData.status);
                  break;
            }
            break;

        default:
        	UART_PRINT("[SOCK EVENT] - Unexpected Event [%x0x]\n\n",pSock->Event);
          break;
    }
}

//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- End
//*****************************************************************************

//*****************************************************************************
//
//! \brief This function initializes the application variables
//!
//! \param    None
//!
//! \return None
//!
//*****************************************************************************
static void InitializeAppVariables()
{
    g_ulStatus = 0;
    memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
    memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
}

//****************************************************************************
//
//!    \brief Connects to the Network in AP or STA Mode - If ForceAP Jumper is
//!                                             Placed, Force it to AP mode
//!
//! \return  0 - Success
//!            -1 - Failure
//
//****************************************************************************

unsigned char g_macAddressLen = SL_MAC_ADDR_LEN;
unsigned char g_macAddressVal[SL_MAC_ADDR_LEN];
bool g_macAddressGet = false;
long ConnectToNetwork()
{
    long lRetVal = -1;
    unsigned int uiConnectTimeoutCnt =0 , uiSmartCfgConnectTimeoutCnt=0;

    // starting simplelink

    lRetVal =  sl_Start(NULL,NULL,NULL);
    ASSERT_ON_ERROR( lRetVal);
    if(lRetVal >=0)
    {
    	is_slReady = true;
    }
    sl_NetCfgGet(SL_MAC_ADDRESS_GET,NULL,&g_macAddressLen, g_macAddressVal);
    g_macAddressGet = true;

    UART_PRINT("\n\r[WiFi] Connecting to network");
    UART_PRINT("\n\r[WiFi] Auto Connecting....");
    {
    	//waiting for the device to Auto Connect
        while(!IS_CONNECTED(g_ulStatus) || !IS_IP_ACQUIRED(g_ulStatus))
        {
            uiConnectTimeoutCnt++;
            if(uiConnectTimeoutCnt>=AUTO_CONNECTION_TIMEOUT_COUNT)
            {
            	break;
            }
        	osi_Sleep(1000);
        }
        //Couldn't connect Using Auto Profile
        if(uiConnectTimeoutCnt >= AUTO_CONNECTION_TIMEOUT_COUNT)
        {
            //Blink Red LED to Indicate Connection Error
        	// BlinkLed(10, FAST1);
        	UART_PRINT("\n\r[WiFi] Auto Connect Failed, trying SmartConfig...");
        	UART_PRINT("\n\r[WiFi] Please open the SmartConfig Mobile App");
            CLR_STATUS_BIT_ALL(g_ulStatus);

            //Connect Using Smart Config
            lRetVal = SmartConfigConnect();
            ASSERT_ON_ERROR(lRetVal);

            //Waiting for the device to Auto Connect
            while(uiSmartCfgConnectTimeoutCnt<SMART_CONFIG_TIMEOUT_COUNT &&
            	((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus))))
            {
            	 BlinkLed(LED_1,1, FAST1);
                 MAP_UtilsDelay(500);
                 uiSmartCfgConnectTimeoutCnt++;
            }
            //Couldn't connect Using Smart Config
            if(uiSmartCfgConnectTimeoutCnt == SMART_CONFIG_TIMEOUT_COUNT)
            {
           		//Turn WIFI LED On
            	//GPIOPinWrite(GPIOA1_BASE, 0x10, 0x10);
            	TurnOnLed(LED_1);
				UART_PRINT("\n\r[WiFi] SmartConfig Failed, use <wlan_connect> CLI command");
            	sl_WlanSmartConfigStop();
            }
            else
            {
            	UART_PRINT("\n\r[WiFi] SmartConfig Success");
                //Turn WIFI LED Off
                //GPIOPinWrite(GPIOA1_BASE, 0x10, 0);
                //TurnOffLed(LED_1);
            }

        }
        else
        {
        	UART_PRINT("\n\r[WiFi] Autoconnect Success");
        	// Connected to AP using Auto connect.Turn WIFI LED Off
            //GPIOPinWrite(GPIOA1_BASE, 0x10, 0);
        	//TurnOffLed(LED_1);

        }


    }
    return SUCCESS;
}

int WlanDisconnectAP()
{
	return(sl_WlanDisconnect());
}

//****************************************************************************
//
//! \brief WlanConnectAP -
//!
//! \param[in]
//!
//! \return                        None
//
//****************************************************************************
#define WLAN_CONNECT_TIMEOUT 20
int WlanConnectAP(signed char *pcSsid , unsigned long ulSsidLen,  unsigned long iLantype,unsigned char ucSecType ,signed char *pcUserName , unsigned long ulUserNameLen,signed char *pcKey , unsigned long ulKeyLen)
{
	volatile long lRetVal = 0;

	 SlSecParams_t 	SecParams;
	 volatile unsigned char uiConnectTimeoutCnt = WLAN_CONNECT_TIMEOUT;
	SecParams.Type = ucSecType ;


	switch(SecParams.Type)
	{
		case SL_SEC_TYPE_OPEN:
			SecParams.Key = "";
			SecParams.KeyLen = 0;
			SecParams.Type = SL_SEC_TYPE_OPEN;
			break;

		case SL_SEC_TYPE_WEP:

			SecParams.Key = (signed char *)pcKey;
			SecParams.KeyLen = (unsigned char)ulKeyLen;
			SecParams.Type = SL_SEC_TYPE_WEP;
			break;

		case SL_SEC_TYPE_WPA:
			SecParams.Key = (signed char *)pcKey;
			SecParams.KeyLen = (unsigned char)ulKeyLen;
			SecParams.Type = SL_SEC_TYPE_WPA;

			break;

	}

	lRetVal = sl_WlanConnect(pcSsid, ulSsidLen, 0, &SecParams, 0);
	while(!IS_CONNECTED(g_ulStatus) || !IS_IP_ACQUIRED(g_ulStatus))
	{
		uiConnectTimeoutCnt--;
		if(uiConnectTimeoutCnt==0)
		{
			break;
		}
		osi_Sleep(250);
	}
	if(uiConnectTimeoutCnt)
	{
		lRetVal = sl_WlanProfileAdd(pcSsid, ulSsidLen, NULL, \
									  &SecParams , NULL, 6, 0);
	}
	return 0;
}


//****************************************************************************
//
//!    \brief Wifi Application Main Task - Initializes SimpleLink Driver and
//!                                              Handles HTTP Requests
//! \param[in]                  pvParameters is the data passed to the Task
//!
//! \return                        None
//
//****************************************************************************
void NetworkInit()
{
    long   lRetVal = -1;

    //Connect to Network
    lRetVal = ConnectToNetwork();
    if(lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        LOOP_ON_ERROR(lRetVal);
    }
}


//****************************************************************************
//                            MAIN FUNCTION
//****************************************************************************
void SimpleLink_Init()
{
    long   lRetVal = -1;

    // Initialize Global Variables
    InitializeAppVariables();

#ifdef HTTP_ENABLED
    // Initialize HTTP callback
    Http_Init();
#endif

    // Simplelinkspawntask
    lRetVal = VStartSimpleLinkSpawnTask(SPAWN_TASK_PRIORITY);
    if(lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        LOOP_FOREVER();
    }


}

int Wifi_GetMAC(unsigned char * macAddressVal)
{
	volatile unsigned char LoopVar = 0xFF;

	while(LoopVar)
	{
		if(g_macAddressGet == true)
		{
			break;
		}
		else
		{
			osi_Sleep(500);
		}
	}
	memcpy(macAddressVal,g_macAddressVal,SL_MAC_ADDR_LEN);
	return 0;
}

bool IsSimpleLinkReady()
{
	return(is_slReady);
}
