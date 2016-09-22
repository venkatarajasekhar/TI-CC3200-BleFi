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
// http.c - contains the html hooks for blefi html server
//
//
//*****************************************************************************
#define HTTP_ENABLED TRUE
#ifdef HTTP_ENABLED

//****************************************************************************
//
//! \
//! @{
//
//****************************************************************************

// Standard includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "http.h"

// Common interface includes
#include "uart_if.h"
#include "common.h"


//*****************************************************************************
//                  MACROS
//*****************************************************************************

#define TOKEN_SCAN_ARRAY_SIZE          20
#define TOKEN_CONN_ARRAY_SIZE          12
#define TOKEN_OTHER_ARRAY_SIZE			5
#define TOKEN_GETCHAR_ARRAY_SIZE	  150
#define POSTTOKEN_ARRAY_SIZE 		   15
#define STRING_TOKEN_SIZE         	   10



//*****************************************************************************
//                  EXTERN FUNCTIONS
//*****************************************************************************

extern char *Util_convertBdAddr2Str(unsigned char *pAddr);
extern unsigned short itoa(char cNum, char *cString);
extern void DB_GetDevCharList(unsigned short int connhandle , fptrDevCharlist_t fptr);
extern unsigned char get_gwDevCnt();

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************

char g_gettoken_scan [TOKEN_SCAN_ARRAY_SIZE][STRING_TOKEN_SIZE] = { "__SL_G_UA1","__SL_G_UB1","__SL_G_UC1",  "__SL_G_UD1",
																    "__SL_G_UA2","__SL_G_UB2","__SL_G_UC2",  "__SL_G_UD2",
															 	    "__SL_G_UA3","__SL_G_UB3","__SL_G_UC3",  "__SL_G_UD3",
																    "__SL_G_UA4","__SL_G_UB4","__SL_G_UC4",  "__SL_G_UD4",
																    "__SL_G_UA5","__SL_G_UB5","__SL_G_UC5",  "__SL_G_UD5",
															      };


char g_gettoken_conn [TOKEN_CONN_ARRAY_SIZE][STRING_TOKEN_SIZE]={  "__SL_G_DA1","__SL_G_DB1","__SL_G_DC1",  "__SL_G_DD1",
															   	   "__SL_G_DA2","__SL_G_DB2","__SL_G_DC2",  "__SL_G_DD2",
															   	   "__SL_G_DA3","__SL_G_DB3","__SL_G_DC3",  "__SL_G_DD3",
															 	};

char g_gettoken_other [TOKEN_OTHER_ARRAY_SIZE][STRING_TOKEN_SIZE]={ "__SL_G_GC1","__SL_G_SC1","__SL_G_NSC","__SL_G_NCO",
																	"__SL_G_NDC",
															      };


char g_gettoken_getchar [TOKEN_GETCHAR_ARRAY_SIZE][STRING_TOKEN_SIZE] = {    "__SL_G_001","__SL_G_002","__SL_G_003",
																			 "__SL_G_004","__SL_G_005","__SL_G_006",
																			 "__SL_G_007","__SL_G_008","__SL_G_009",
																			 "__SL_G_010","__SL_G_011","__SL_G_012",
																			 "__SL_G_013","__SL_G_014","__SL_G_015",
																			 "__SL_G_016","__SL_G_017","__SL_G_018",
																			 "__SL_G_019","__SL_G_020","__SL_G_021",
																			 "__SL_G_022","__SL_G_023","__SL_G_024",
																			 "__SL_G_025","__SL_G_026","__SL_G_027",
																			 "__SL_G_028","__SL_G_029","__SL_G_030",
																			 "__SL_G_031","__SL_G_032","__SL_G_033",
																			 "__SL_G_034","__SL_G_035","__SL_G_036",
																			 "__SL_G_037","__SL_G_038","__SL_G_039",
																			 "__SL_G_040","__SL_G_041","__SL_G_042",
																			 "__SL_G_043","__SL_G_044","__SL_G_045",
																			 "__SL_G_046","__SL_G_047","__SL_G_048",
																			 "__SL_G_049","__SL_G_050","__SL_G_051",
																			 "__SL_G_052","__SL_G_053","__SL_G_054",
																			 "__SL_G_055","__SL_G_056","__SL_G_057",
																			 "__SL_G_058","__SL_G_059","__SL_G_060",
																			 "__SL_G_061","__SL_G_062","__SL_G_063",
																			 "__SL_G_064","__SL_G_065","__SL_G_066",
																			 "__SL_G_067","__SL_G_068","__SL_G_069",
																			 "__SL_G_070","__SL_G_071","__SL_G_072",
																			 "__SL_G_073","__SL_G_074","__SL_G_075",
																			 "__SL_G_076","__SL_G_077","__SL_G_078",
																			 "__SL_G_079","__SL_G_080","__SL_G_081",
																			 "__SL_G_082","__SL_G_083","__SL_G_084",
																			 "__SL_G_085","__SL_G_086","__SL_G_087",
																			 "__SL_G_088","__SL_G_089","__SL_G_090",
																			 "__SL_G_091","__SL_G_092","__SL_G_093",
																			 "__SL_G_094","__SL_G_095","__SL_G_096",
																			 "__SL_G_097","__SL_G_098","__SL_G_099",
																			 "__SL_G_100","__SL_G_101","__SL_G_102",
																			 "__SL_G_103","__SL_G_104","__SL_G_105",
																			 "__SL_G_106","__SL_G_107","__SL_G_108",
																			 "__SL_G_109","__SL_G_110","__SL_G_111",
																			 "__SL_G_112","__SL_G_113","__SL_G_114",
																			 "__SL_G_115","__SL_G_116","__SL_G_117",
																			 "__SL_G_118","__SL_G_119","__SL_G_120",
																			 "__SL_G_121","__SL_G_122","__SL_G_123",
																			 "__SL_G_124","__SL_G_125","__SL_G_126",
																			 "__SL_G_127","__SL_G_128","__SL_G_129",
																			 "__SL_G_130","__SL_G_131","__SL_G_132",
																			 "__SL_G_133","__SL_G_134","__SL_G_135",
																			 "__SL_G_136","__SL_G_137","__SL_G_138",
																			 "__SL_G_139","__SL_G_140","__SL_G_141",
																			 "__SL_G_142","__SL_G_143","__SL_G_144",
																			 "__SL_G_145","__SL_G_146","__SL_G_147",
																			 "__SL_G_148","__SL_G_149","__SL_G_150",
																	 };


char g_posttoken [POSTTOKEN_ARRAY_SIZE][STRING_TOKEN_SIZE] = {    "__SL_P_PS1","__SL_P_PS2","__SL_P_PS3",
																  "__SL_P_PS4","__SL_P_PS5","__SL_P_PS6",
																  "__SL_P_PS7","__SL_P_PS8","__SL_P_PS9",
																  "__SL_P_P10","__SL_P_P11","__SL_P_P12",
																  "__SL_P_P13","__SL_P_P14","__SL_P_P15",
															 };

typedef struct gw_scanlist_t
{
	unsigned char devIndex;
	char* devName;
	char bdAddr[15];
	unsigned char addrType;
}gw_scanlist_t;


//static gw_scanlist_t gw_scanlist[DEFAULT_MAX_SCAN_RES];
gw_scanlist_t gw_scanlist[DEFAULT_MAX_SCAN_RES];


typedef struct gw_connlist_t
{
	unsigned char devIndex;
	char* devName;
	char bdAddr[15];
	unsigned char addrType;
}gw_connlist_t;


//static gw_connlist_t gw_connlist[DEFAULT_MAX_CONN_RES];
gw_connlist_t gw_connlist[DEFAULT_MAX_CONN_RES];


typedef struct gw_charlist_t
{
	unsigned short int char_index;
	char* charName;
	unsigned char getsetflag;
}gw_charlist_t;


static gw_charlist_t gw_charlist[MAX_CHAR_IN_DEV_SUPPORTED];

#define MAX_INSTRING_LEN  30
#define MAX_GETBUF_LEN  20
#define MAX_SETBUF_LEN  20

typedef struct getsetParams_t
{
   unsigned short int g_GetSetConnHndl;
   unsigned char g_GetSetInputString[MAX_INSTRING_LEN] ;
   unsigned char g_getsetFlag;
   unsigned char getbufLen;
   unsigned char getbuf[MAX_GETBUF_LEN];
   unsigned char setbufLen;
   unsigned long setbuf;
}getsetParams_t;


static getsetParams_t getsetParams =
{
	0xFFFF,
	NULL,
	CHAR_GET,
	0,
	0,
	0,
	0,
};


//extern int GC_Scan(GW_CALL_TYPE calltype);
//warning !! all the device ids should be short
#if 0
static unsigned char g_scanIndex =0;
static unsigned char g_connIndex =0;
static unsigned char g_charIndex =0;
#else
unsigned char g_scanIndex =0;
unsigned char g_connIndex =0;
unsigned char g_charIndex =0;

#endif

unsigned char tokenmatch_scan_index = 0xFF;
unsigned char tokenmatch_conn_index = 0xFF;
unsigned char tokenmatch_getchar_index = 0xFF;

unsigned char sGetSetFlag[2];
char cCharGetSetFlag ;
short sCharGetSetFlagLen ;
unsigned char cCountScanDevices;

#ifdef HTTP_DEBUG_ENABLED
#define  HTTP_PRINT UART_PRINT
#else
#define  HTTP_PRINT(x,...)
#endif


#ifdef HTTP_DEBUG_ENABLED
#define  HTTP_DBG_PRINT UART_PRINT
#else
#define   HTTP_DBG_PRINT(x,...)
#endif




//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

//*****************************************************************************
//
//!HTTP_EventCb
//!
//! \param
//! \param
//!
//! \return none
//!
void HTTP_EventCb(unsigned char EventType,ErrorFlag_t ErrorFlag, void* EventParams)
{
#ifdef HTTP_DEBUG_ENABLED
	 gwEvent_t* pEvent = (gwEvent_t*)EventParams;
#endif


	 switch (EventType)
	 {
		 case GW_EVENT_SCANRSP:
		 {
			 HTTP_PRINT("\n\r HTTP app - Device Found -  ");
			 HTTP_PRINT("%d  %s %s %d %d ",(pEvent->scanRsp.scanIdx - 1),
					 	 	 	 	 	 	pEvent->scanRsp.deviceName ,
					 	 	 	 	 	 	Util_convertBdAddr2Str(pEvent->scanRsp.addr) ,
					 	 	 	 	 	 	pEvent->scanRsp.addrType ,
					 	 	 	 	 	 	pEvent->scanRsp.rssi);
		 }
		 break;

		 case GW_EVENT_SCANCOMPLETE:
		 {
			 HTTP_PRINT("\n\r HTTP app - Scan Complete \n\r");
		 }
		 break;

		 case GW_EVENT_LINKESTABLISH:
		 {
			HTTP_PRINT("\n\r HTTP app - Connected to Device - ");
			HTTP_PRINT("%d  %s %s",pEvent->linkEst.connhandle ,
								   pEvent->linkEst.deviceName ,
								   Util_convertBdAddr2Str(pEvent->linkEst.bdAddr));
			HTTP_PRINT("\n\r");
		 }
		 break;

		 case GW_EVENT_LINKTERMINATE:
		 {
			HTTP_PRINT("\n\r HTTP app - Disconnected the Device - ");
			HTTP_PRINT("%d  %s %s reason- 0x%x",pEvent->linkTerm.connhandle ,
												pEvent->linkTerm.deviceName ,
												Util_convertBdAddr2Str(pEvent->linkTerm.bdAddr),
												pEvent->linkTerm.reason);
			HTTP_PRINT("\n\r");
		 }
		 break;

		 default:
			 break;
	 }
}



//*****************************************************************************
//
//! HTTP_DataCb
//!
//! \param
//! \param
//!
//! \return none
//!
void HTTP_DataCb(unsigned char DataType, ErrorFlag_t ErrorFlag,void* DataParams)
{
	gwData_t* pData = (gwData_t*)DataParams;
	unsigned char i ;

	 switch (DataType)
	 {
		 case GW_DATA_GET:
		 {
			 if(ErrorFlag == 0)
			 {
				 HTTP_PRINT("\n\r HTTP app -  ");
				 HTTP_PRINT(" Device - %d , value -  ",pData->getRsp.connhandle);

				 getsetParams.getbufLen = pData->getRsp.ValueLen;

				 for(i=0; i<pData->getRsp.ValueLen ;i++)
				 {
					 HTTP_PRINT(" 0x%x", pData->getRsp.pValueBuffer[i]);

					 getsetParams.getbuf[i] = pData->getRsp.pValueBuffer[i];

				 }
				 HTTP_PRINT("\n\r");
			 }
			 else
			 {
				 HTTP_PRINT("\n\r  HTTP app -  ERROR in executing GET ");
				 HTTP_PRINT("\n\r");
			 }
		 }
		 break;

		 case GW_DATA_SET:
		 {
			if(ErrorFlag == 0)
			{
				HTTP_PRINT("\n\r HTTP app -  Device - -%d");
				HTTP_PRINT("\n\r");
			}
			else
			{
				 HTTP_PRINT("\n\r HTTP app -  ERROR in executing SET ");
				 HTTP_PRINT("\n\r");
			}
		 }
		 break;

		 default:
			 break;
	 }
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
void fnPtrHTTPscanlist(unsigned char devIndex, char * devName , unsigned char *bdAddr,unsigned char addrType)
{
	if(g_scanIndex < DEFAULT_MAX_SCAN_RES)
	{
		gw_scanlist[devIndex].devIndex = devIndex;
		// need malloc?
		gw_scanlist[devIndex].devName = devName;
		//gw_scanlist[devIndex].bdAddr  = (unsigned char *)(Util_convertBdAddr2Str(bdAddr));
		strncpy(gw_scanlist[devIndex].bdAddr,(char *)(Util_convertBdAddr2Str(bdAddr)), 14);
		gw_scanlist[devIndex].bdAddr[14]='\0';

		//HTTP_DBG_PRINT("\n\r DEVID = %d , BD addr of SCAN list =  %s \n\n\r",devIndex,gw_scanlist[devIndex].bdAddr);
        unsigned char idx;
        for(idx=0 ; idx < 2 ; idx++)
        {

          HTTP_DBG_PRINT("\n\n\rInside fnPtrHTTPscanlist -- DEV ID = %d ",devIndex );
      	  HTTP_DBG_PRINT("\n\n\r in FPTR DEVID %d  - BDADDR = %s   \n\r",idx , (char*)gw_scanlist[idx].bdAddr);

        }
		gw_scanlist[devIndex].addrType  = addrType;
		g_scanIndex++;
	}
}


//*****************************************************************************
//
//! fnPtrHTTPconnlist
//!
//! @param  unsigned char devIndex, unsigned char * devName , unsigned char *bdAddr,unsigned char addrType
//!
//! @return none
//!
//! @brief
//
//*****************************************************************************
void fnPtrHTTPconnlist(unsigned char devIndex, char * devName , unsigned char *bdAddr,unsigned char addrType)
{
	if(g_connIndex< DEFAULT_MAX_CONN_RES)
	{
		gw_connlist[g_connIndex].devIndex = devIndex;
		// need malloc?
		gw_connlist[g_connIndex].devName = devName;
		//gw_connlist[g_connIndex].bdAddr  = (unsigned char*)(Util_convertBdAddr2Str(bdAddr));
		strncpy(gw_connlist[g_connIndex].bdAddr,(char*)(Util_convertBdAddr2Str(bdAddr)),14);
		gw_connlist[g_connIndex].bdAddr[14]='\0';

		gw_connlist[g_connIndex].addrType  = addrType;
		g_connIndex++;
	}
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
void fnPtrDevCharListHttpPRINT(unsigned short int connhandle, unsigned short int char_index, char* charName, unsigned char getsetflag)
{
	 if (g_charIndex < MAX_CHAR_IN_DEV_SUPPORTED)
	 {
		 gw_charlist[g_charIndex].char_index = char_index;
		 // malloc this ?
		 gw_charlist[g_charIndex].charName = charName;

		 gw_charlist[g_charIndex].getsetflag = getsetflag;

		 g_charIndex++;

	 }
}



//*****************************************************************************
//
//! HttpGetTokenScan
//!
//! @param
//! @return none
//!
//! @brief
//
//*****************************************************************************
void HttpGetTokenScan(unsigned char tokenmatch_scan_index ,  SlHttpServerEvent_t *pSlHttpServerEvent,
        SlHttpServerResponse_t *pSlHttpServerResponse)
{
	unsigned char token_scan_id ,token_scan_dev_id , i;

    token_scan_id = tokenmatch_scan_index % 4;
    token_scan_dev_id = (tokenmatch_scan_index /4 ) ;

    switch(token_scan_id)
    {
      case 0:
      {
		if(tokenmatch_scan_index == 0)
		{
			g_scanIndex = 0;

		   for(i=0;i<DEFAULT_MAX_SCAN_RES; i++)
		   {
			   gw_scanlist[i].devIndex = 9;
			   gw_scanlist[i].devName = "xxxxxx\0";
			   //gw_scanlist[i].bdAddr = "xx";
			   gw_scanlist[i].bdAddr[0] = 'x';
			   gw_scanlist[i].bdAddr[1] = 'x';
			   gw_scanlist[i].bdAddr[2] = '\0';
			   gw_scanlist[i].addrType = 'x';

		   }

			GC_DeviceScanList(fnPtrHTTPscanlist);
		}

		unsigned char sDeviceId[2];
		char cDeviceId = (char)gw_scanlist[token_scan_dev_id].devIndex;
		short sDeviceIdLen = itoa(cDeviceId,(char*)sDeviceId);

		sDeviceId[1] = '\0';

		// Scanned devices - addr type
		memcpy(pSlHttpServerResponse->ResponseData.token_value.data, \
			sDeviceId, strlen((const char *)sDeviceId));
		pSlHttpServerResponse->ResponseData.token_value.len = strlen((const char *)sDeviceId);
      }
      	break;

      case 1:
    	 // UB1- UB5
      {
          // Scanned devices - Device name
      	memcpy(pSlHttpServerResponse->ResponseData.token_value.data, \
          		gw_scanlist[token_scan_dev_id].devName, strlen((const char*)gw_scanlist[token_scan_dev_id].devName));

      	pSlHttpServerResponse->ResponseData.token_value.len = strlen((const char *)gw_scanlist[token_scan_dev_id].devName);
      }
      	break;

      case 2:
    // UC1-UC5
      {
          // Scanned devices - BDaddr
          memcpy(pSlHttpServerResponse->ResponseData.token_value.data, \
          	(char*)gw_scanlist[token_scan_dev_id].bdAddr, strlen((const char *)gw_scanlist[token_scan_dev_id].bdAddr));

          pSlHttpServerResponse->ResponseData.token_value.len = strlen((const char *)gw_scanlist[token_scan_dev_id].bdAddr);

          //HTTP_DBG_PRINT("\n\r UC%d  - BDADDR = %s   \n\r",token_scan_dev_id+1 , (char*)gw_scanlist[token_scan_dev_id].bdAddr);


          unsigned char idx;
          for(idx=0 ; idx < 5 ; idx++)
          {
        	  HTTP_DBG_PRINT("\n\n\r in UC DEVID %d  - BDADDR = %s   \n\r",idx , (char*)gw_scanlist[idx].bdAddr);
          }

      }
      	break;

      case 3:
      {
			unsigned char sAddrType[2];
			char cAddrType = (char)gw_scanlist[token_scan_dev_id].addrType;
			short sAddrTypeLen = itoa(cAddrType,(char*)sAddrType);

			sAddrType[1] = '\0';

			// Scanned devices - addr type
			memcpy(pSlHttpServerResponse->ResponseData.token_value.data, \
				sAddrType, strlen((const char *)sAddrType));
			pSlHttpServerResponse->ResponseData.token_value.len = strlen((const char *)sAddrType);

      }
    	break;

      default :
      	// No Matching token found

      	break;

    }// end switch(token_scan_id)

}


//*****************************************************************************
//
//! HttpGetTokenConn
//!
//! @param
//! @return none
//!
//! @brief
//
//*****************************************************************************
void HttpGetTokenConn(unsigned char tokenmatch_conn_index ,  SlHttpServerEvent_t *pSlHttpServerEvent,
        SlHttpServerResponse_t *pSlHttpServerResponse)
{
	unsigned char token_conn_id ,token_conn_dev_id , i;

    token_conn_id = tokenmatch_conn_index % 4;
    token_conn_dev_id = (tokenmatch_conn_index /4 ) ;

    switch(token_conn_id)
    {
      case 0:
      {
      	if(tokenmatch_conn_index == 0)
      	{
      		g_connIndex = 0;

		   for(i=0;i<DEFAULT_MAX_CONN_RES; i++)
		   {
			   gw_connlist[i].devIndex = 9;
			   gw_connlist[i].devName = "xxxxxx\0";
			   gw_connlist[i].bdAddr[0] = 'x';
			   gw_connlist[i].bdAddr[1] = 'x';
			   gw_connlist[i].bdAddr[2] = '\0';
			   gw_connlist[i].addrType = 'x';
		   }

		   GC_DeviceConnList(fnPtrHTTPconnlist);
      	}

		unsigned char sDeviceId[2];
		char cDeviceId = (char)gw_connlist[token_conn_dev_id].devIndex;
		short sDeviceIdLen = itoa(cDeviceId,(char*)sDeviceId);

		sDeviceId[1] = '\0';

		// Connected devices - addr type
		memcpy(pSlHttpServerResponse->ResponseData.token_value.data, \
			sDeviceId, strlen((const char *)sDeviceId));
		pSlHttpServerResponse->ResponseData.token_value.len = strlen((const char *)sDeviceId);
      }
      	break;

      case 1:
      {
          // Connected  devices - Device name
      	memcpy(pSlHttpServerResponse->ResponseData.token_value.data, \
          		gw_connlist[token_conn_dev_id].devName, strlen((const char *)gw_connlist[token_conn_dev_id].devName));

        pSlHttpServerResponse->ResponseData.token_value.len = strlen((const char*)gw_connlist[token_conn_dev_id].devName);
      }
      	break;

      case 2:
      {
          // Connected  devices - BDaddr
          memcpy(pSlHttpServerResponse->ResponseData.token_value.data, \
          	(char*)gw_connlist[token_conn_dev_id].bdAddr, strlen((const char *)gw_connlist[token_conn_dev_id].bdAddr));

          pSlHttpServerResponse->ResponseData.token_value.len = strlen((const char*)gw_connlist[token_conn_dev_id].bdAddr);


          HTTP_DBG_PRINT("\n\r DC%d  - BDADDR = %s   \n\r",token_conn_dev_id+1 , (char*)gw_connlist[token_conn_dev_id].bdAddr);
      }
      	break;

      case 3:
      {
			unsigned char sAddrType[2];
			char cAddrType = (char)gw_connlist[token_conn_dev_id].addrType;
			short sAddrTypeLen = itoa(cAddrType,(char*)sAddrType);

			sAddrType[1] = '\0';

			// Scanned devices - addr type
			memcpy(pSlHttpServerResponse->ResponseData.token_value.data, \
				sAddrType, strlen((const char *)sAddrType));

			pSlHttpServerResponse->ResponseData.token_value.len = strlen((const char *)sAddrType);
      }
      	break;

      default :
      	// No Matching token found

      	break;
    }// end switch
}




//*****************************************************************************
//
//! HttpGetTokenConn
//!
//! @param
//! @return none
//!
//! @brief
//
//*****************************************************************************

void HttpGetTokenGetChar(unsigned char tokenmatch_getchar_index ,  SlHttpServerEvent_t *pSlHttpServerEvent,
        SlHttpServerResponse_t *pSlHttpServerResponse)
{

	unsigned char token_getchar_id,token_getchar_dev_id;


    token_getchar_id = tokenmatch_getchar_index % 3;

    token_getchar_dev_id = (tokenmatch_getchar_index /3 ) ;

    switch(token_getchar_id)
    {
      case 0:
	  {
			unsigned char sCharId[3];
			// char or short?
			char cCharId = (char)gw_charlist[token_getchar_dev_id].char_index;
			short sCharIdLen = itoa(cCharId,(char*)sCharId);

			if(cCharId <10 )
			{
				sCharId[1] = '\0';
			}

			sCharId[2] = '\0';

			// char id
			memcpy(pSlHttpServerResponse->ResponseData.token_value.data, \
					sCharId, strlen((const char *)sCharId));

			pSlHttpServerResponse->ResponseData.token_value.len = strlen((const char *)sCharId);

			break;
	}
  	case 1:
	{
		  // Scanned devices - Device name
			memcpy(pSlHttpServerResponse->ResponseData.token_value.data, \
					gw_charlist[token_getchar_dev_id].charName, strlen((const char *)gw_charlist[token_getchar_dev_id].charName));

			pSlHttpServerResponse->ResponseData.token_value.len = strlen((const char *)gw_charlist[token_getchar_dev_id].charName);

		  break;
	}
  	case 2:
	{
		unsigned char getsetflag = gw_charlist[token_getchar_dev_id].getsetflag;


		if((gattPropRead(getsetflag) !=0) && (gattPropWrite(getsetflag) !=0) )
		{
			memcpy(pSlHttpServerResponse->ResponseData.token_value.data, \
					"[GET/SET]\0", strlen("[GET/SET]\0"));
			pSlHttpServerResponse->ResponseData.token_value.len = strlen("[GET/SET]\0");
		}
		else if((gattPropRead(getsetflag) !=0) )
		{
			memcpy(pSlHttpServerResponse->ResponseData.token_value.data, \
					"[GET]\0", strlen("[GET]\0"));
			pSlHttpServerResponse->ResponseData.token_value.len = strlen("[GET]\0");
		}
		else if((gattPropWrite(getsetflag) !=0) )
		{
			memcpy(pSlHttpServerResponse->ResponseData.token_value.data, \
					"[SET]\0", strlen("[SET]\0"));
			pSlHttpServerResponse->ResponseData.token_value.len = strlen("[SET]\0");

		}
		else
		{
			// No permissions to read or write
			memcpy(pSlHttpServerResponse->ResponseData.token_value.data, \
					"[NO GET/SET]\0", strlen("[NO GET/SET]\0"));
			pSlHttpServerResponse->ResponseData.token_value.len = strlen("[NO GET/SET]\0");

		}

    	break;
	}

  	default:

  		break;
   }
}


//*****************************************************************************
//
//! HttpGetTokenOther
//!
//! @param
//! @return none
//!
//! @brief
//
//*****************************************************************************

void HttpGetTokenOther(unsigned char tokenmatch_other_index ,  SlHttpServerEvent_t *pSlHttpServerEvent,
        SlHttpServerResponse_t *pSlHttpServerResponse)
{

    unsigned char i;

    char* listcharstr;
    char * tempstr;

	if(0== memcmp(pSlHttpServerEvent->EventData.httpTokenName.data, \
						"__SL_G_GC1", \
						pSlHttpServerEvent->EventData.httpTokenName.len))
	{
		// 5 is for each character in say 0xAB:
		listcharstr = malloc((5 * getsetParams.getbufLen) + 1 );
		if(listcharstr == NULL)
		{
			HTTP_DBG_PRINT("\n\r [DB] Error - Not enough memory for listcharstr \n");
			return;
		}
		listcharstr[0]='\0';


		 for(i=0; i<getsetParams.getbufLen ;i++)
		 {
			 tempstr = (char *) (listcharstr + strlen(listcharstr)) ;
			 //sprintf(tempstr,"0x%2x:", getsetParams.getbuf[i]);
			 sprintf(tempstr,"%#04x:", getsetParams.getbuf[i]);
			 HTTP_PRINT("0x%x:", getsetParams.getbuf[i]);
		 }

		listcharstr[(5 * getsetParams.getbufLen) - 1] = '\0';

		memcpy(pSlHttpServerResponse->ResponseData.token_value.data, \
				listcharstr , strlen((const char *)listcharstr));

		pSlHttpServerResponse->ResponseData.token_value.len = strlen((const char *)listcharstr);


		free(listcharstr);


	}

	if(0== memcmp(pSlHttpServerEvent->EventData.httpTokenName.data, \
						"__SL_G_SC1", \
						pSlHttpServerEvent->EventData.httpTokenName.len))
	{


		memcpy(pSlHttpServerResponse->ResponseData.token_value.data, \
				"SUCCESS" , strlen("SUCCESS"));

		pSlHttpServerResponse->ResponseData.token_value.len = strlen("SUCCESS");


	}

	if(0== memcmp(pSlHttpServerEvent->EventData.httpTokenName.data, \
						"__SL_G_NSC", \
						pSlHttpServerEvent->EventData.httpTokenName.len))
	{
		HTTP_DBG_PRINT("\n\r __SL_G_NSC token received and scan_count = %d", get_gwDevCnt());

		unsigned char s_g_scanIndex[2];
		short s_g_scanIndexLen = itoa(get_gwDevCnt(),(char*)s_g_scanIndex);

		s_g_scanIndex[1] = '\0';


		memcpy(pSlHttpServerResponse->ResponseData.token_value.data, \
				s_g_scanIndex , strlen((const char *)s_g_scanIndex));

		pSlHttpServerResponse->ResponseData.token_value.len = strlen((const char *)s_g_scanIndex);


	}

	if(0== memcmp(pSlHttpServerEvent->EventData.httpTokenName.data, \
						"__SL_G_NCO", \
						pSlHttpServerEvent->EventData.httpTokenName.len))
	{
      	{
			g_connIndex = 0;
			unsigned char i ;

			for(i=0;i<DEFAULT_MAX_CONN_RES; i++)
			{
			   gw_connlist[i].devIndex = 9;
			   gw_connlist[i].devName = "xxxxxx\0";
			   gw_connlist[i].bdAddr[0] = 'x';
			   gw_connlist[i].bdAddr[1] = 'x';
			   gw_connlist[i].bdAddr[2] = '\0';
			   gw_connlist[i].addrType = 'x';
			}

			GC_DeviceConnList(fnPtrHTTPconnlist);
      	}

      	HTTP_DBG_PRINT("\n\r __SL_G_NCO token received and conn_count = %d", g_connIndex);

		unsigned char s_g_connIndex[2];
		short s_g_connIndexLen = itoa(g_connIndex,(char*)s_g_connIndex);

		s_g_connIndex[1] = '\0';

		memcpy(pSlHttpServerResponse->ResponseData.token_value.data, \
				s_g_connIndex , strlen((const char *)s_g_connIndex));

		pSlHttpServerResponse->ResponseData.token_value.len = strlen((const char *)s_g_connIndex);
	}

#if 0
	if(0== memcmp(pSlHttpServerEvent->EventData.httpTokenName.data, \
						"__SL_G_NDC", \
						pSlHttpServerEvent->EventData.httpTokenName.len))
	{

  		g_devCharIndex = 0;
        unsigned char i ;


		   // Initilise gw_charlist
		   for(i=0;i < MAX_CHAR_IN_DEV_SUPPORTED; i++)
		   {
			   gw_charlist[i].char_index = 'x';
			   gw_charlist[i].charName = malloc(20);
			   gw_charlist[i].charName = "xxxxxx\0";
			   gw_charlist[i].getsetflag = 'x';
		   }


		   DB_GetDevCharList(g_GetCharDeviceId , fnPtrDevCharListHttpPRINT);


	}

#endif


}


//*****************************************************************************
//
//! HttpGetToken
//!
//! @param
//! @return none
//!
//! @brief
//
//*****************************************************************************
void HttpGetToken(SlHttpServerEvent_t *pSlHttpServerEvent,
        SlHttpServerResponse_t *pSlHttpServerResponse)
{

	bool tokenmatch_flag=FALSE;
	 unsigned char i;



#ifdef HTTP_DEBUG_ENABLED

	 if( memcmp(pSlHttpServerEvent->EventData.httpTokenName.data, \
	 						  "__SL_P_PS1", \
	 						  strlen("__SL_P_PS1")) ==0)
	 {

		 HTTP_DBG_PRINT("\n\r GET TOKEN received- __SL_P_PS1 ");
	 }
#endif

	 // Check for scanned devices tokens
	for(i=0;i<TOKEN_SCAN_ARRAY_SIZE;i++)
	{
		  if(0== memcmp(pSlHttpServerEvent->EventData.httpTokenName.data, \
						  g_gettoken_scan[i], \
						  pSlHttpServerEvent->EventData.httpTokenName.len))
		  {
			  tokenmatch_flag = TRUE;

			  HttpGetTokenScan(i,pSlHttpServerEvent ,pSlHttpServerResponse);

			  break;
		  }
	}

	if(tokenmatch_flag == TRUE)
	{
		return ;
	}

	// Check for connected devices tokens
	for(i=0;i<TOKEN_CONN_ARRAY_SIZE;i++)
	{
		  if(0== memcmp(pSlHttpServerEvent->EventData.httpTokenName.data, \
						  g_gettoken_conn [i], \
						  pSlHttpServerEvent->EventData.httpTokenName.len))
		  {
			  tokenmatch_flag = TRUE;

			  HttpGetTokenConn(i,pSlHttpServerEvent ,pSlHttpServerResponse);

			  break; // break for loop
		  }
	}

	if(tokenmatch_flag == TRUE)
	{
		return;
	}

	for(i=0;i<TOKEN_OTHER_ARRAY_SIZE;i++)
	{
	  if(0== memcmp(pSlHttpServerEvent->EventData.httpTokenName.data, \
					  g_gettoken_other [i], \
					  pSlHttpServerEvent->EventData.httpTokenName.len))
	  {
		  tokenmatch_flag = TRUE;

		  HttpGetTokenOther(i,pSlHttpServerEvent ,pSlHttpServerResponse);

		  break;
	  }
	}

	if(tokenmatch_flag == TRUE)
	{
		return;
	}

	// Check for GetChar tokens
	for(i=0;i<TOKEN_GETCHAR_ARRAY_SIZE;i++)
	{
		  if(0== memcmp(pSlHttpServerEvent->EventData.httpTokenName.data, \
						  g_gettoken_getchar [i], \
						  pSlHttpServerEvent->EventData.httpTokenName.len))
		  {
			  tokenmatch_flag = TRUE;

			  HttpGetTokenGetChar(i,pSlHttpServerEvent ,pSlHttpServerResponse);

			  break;
		  }
	}

	if(tokenmatch_flag == TRUE)
	{
		return;
	}

}


//*****************************************************************************
//
//! HttpPostToken
//!
//! @param
//! @return none
//!
//! @brief
//
//*****************************************************************************
unsigned char bdAddr[B_ADDR_LEN];
void HttpPostToken( SlHttpServerEvent_t *pSlHttpServerEvent,
        SlHttpServerResponse_t *pSlHttpServerResponse)
{
	 static unsigned short int g_LinkEstablishDeviceId = 0xFFFF;
	 static unsigned short int g_LinkTerminateDeviceId = 0xFFFF;
	 static unsigned short int g_GetCharDeviceId = 0xFFFF;
     unsigned char index, posttoken_matchindex;
	 bool posttoken_matchfound = FALSE;

#ifdef HTTP_DEBUG_ENABLED

#warning for HTTP Debug

     char tempStr[20];

     strncpy((char *)tempStr, (const char *)pSlHttpServerEvent->EventData.httpPostData.token_name.data,
    		  pSlHttpServerEvent->EventData.httpPostData.token_name.len);
	 tempStr[pSlHttpServerEvent->EventData.httpPostData.token_name.len] = '\0';

	 HTTP_DBG_PRINT("\n\r POST TOKEN -  ");
	 HTTP_DBG_PRINT("%s",tempStr);
	 HTTP_DBG_PRINT("\n\r");

#endif

	 for(index=0 ; index < POSTTOKEN_ARRAY_SIZE ; index++)
	 {
		if(0 == memcmp(pSlHttpServerEvent->EventData.httpPostData.token_name.data, \
					 g_posttoken[index], \
					 pSlHttpServerEvent->EventData.httpPostData.token_name.len))
		{
			posttoken_matchfound = TRUE;
			posttoken_matchindex = index;

			break;
		}
	 }

	  if(posttoken_matchfound == TRUE)
	  {
		  switch(posttoken_matchindex)
		  {
		  	  // Scan - __SL_P_PS1
			  case 0:
			  {
				   int devices_found;

					devices_found = GC_Scan(GW_CALL_BLOCKING);
					cCountScanDevices= devices_found;

					if (devices_found < 0)
					{
						HTTP_PRINT("\n\r ERROR - Discovery already in progress");
					}
					else
					{
						HTTP_PRINT("\n\r Devices discovered : %d", devices_found);
						HTTP_PRINT("\n\r Use dev_list command to display the devices \n\n\r");
					}
				  break;
			  }
			  //  Link establish Scan ID __SL_P_PS2
			  case 1:
			  {
				  g_LinkEstablishDeviceId = strtoul((const char *)pSlHttpServerEvent->EventData.httpPostData.token_value.data,NULL,10);

				  break;
			  }
			  //  Link establish cmd  __SL_P_PS3
			  case 2:
			  {
					unsigned short int uConnId;
					//unsigned char bdAddr[B_ADDR_LEN];

					uConnId = GC_LinkEstablish(g_LinkEstablishDeviceId , GW_CALL_NONBLOCKING ,(unsigned char*)bdAddr);

					if(uConnId != 0xFFFF)
					{
						HTTP_PRINT("\n\r Device successfully connected %d   %s\n\r",uConnId , Util_convertBdAddr2Str(bdAddr));
					}
					else
					{
						HTTP_PRINT("\n\r Device connection not successful");
					}

				  break;
			  }
			  //  Link terminate connection Id   __SL_P_PS4
			  case 3:
			  {
				  g_LinkTerminateDeviceId = strtoul((const char *)pSlHttpServerEvent->EventData.httpPostData.token_value.data,NULL,10);

				  break;
			  }
			  //  Link terminate cmd   __SL_P_PS5
			  case 4:
			  {
				  GC_LinkTerminate(g_LinkTerminateDeviceId, GW_CALL_BLOCKING);

				  break;
			  }
			  //  Get device char  list - device Id   __SL_P_PS6
			  case 5:
			  {
				  g_GetCharDeviceId = strtoul((const char *)pSlHttpServerEvent->EventData.httpPostData.token_value.data,NULL,10);

				  break;
			  }
			  //  Get device char  list - cmd  __SL_P_PS7
			  case 6:
			  {
				  DB_GetDevCharList(g_GetCharDeviceId , fnPtrDevCharListHttpPRINT);

				  break;
			  }
			  //  Connection handle GET/SET - cmd  __SL_P_PS8
			  case 7:
			  {
				  getsetParams.g_GetSetConnHndl = strtoul((const char *)pSlHttpServerEvent->EventData.httpPostData.token_value.data,NULL,10);

				  break;
			  }
			  //  char String GET/SET - cmd  __SL_P_PS9
			  case 8:
			  {
				  memcpy(getsetParams.g_GetSetInputString,  \
				  pSlHttpServerEvent->EventData.httpPostData.token_value.data, \
				  pSlHttpServerEvent->EventData.httpPostData.token_value.len);

				  break;
			  }
			  //  SET buffer -  __SL_P_P10
			  case 9:
			  {
				  getsetParams.setbuf = 	strtoul((const char *)pSlHttpServerEvent->EventData.httpPostData.token_value.data,NULL,10);

				  getsetParams.setbufLen = 	pSlHttpServerEvent->EventData.httpPostData.token_value.len;

				  break;
			  }
			  //  GET/SET cmd  __SL_P_P11
			  case 10:
			  {
				  if (*(pSlHttpServerEvent->EventData.httpPostData.token_value.data) == 0x30 ) // 0x30 -ascii for '0'
				  {
					  // GET cmd received
					  GC_GetSet(getsetParams.g_GetSetConnHndl ,(char *)getsetParams.g_GetSetInputString , CHAR_GET,0,NULL);
				  }
				  else
				  {
					  // SET cmd received
					  GC_GetSet(getsetParams.g_GetSetConnHndl ,(char *)getsetParams.g_GetSetInputString, CHAR_SET ,
							    getsetParams.setbufLen , (unsigned char *)&getsetParams.setbuf);
				  }

				  break;
			  }
			  case 11:
			  {
				  break;
			  }
			  case 12:
			  {
				  break;
			  }
			  case 13:
			  {
				  break;
			  }
			  case 14:
			  {
				  break;
			  }
			  case 15:
			  {

				  break;
			  }
			  case 16:
			  {

				  break;
			  }

			  default :
			  {

				  break;
			  }

		  } //end  switch(posttoken_matchindex)

		} // end if(posttoken_matchfound == TRUE)
}


//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- End
//*****************************************************************************

//*****************************************************************************
//
//! \brief This function initializes the Ble Scan List global  variables
//!
//! \param    None
//!
//! \return None
//!
//*****************************************************************************
static void InitializeHttpGwVariables()
{
   unsigned char i;

   for(i=0;i<DEFAULT_MAX_SCAN_RES; i++)
   {
	   gw_scanlist[i].devIndex = 9;
	   gw_scanlist[i].devName = malloc(20);
	   gw_scanlist[i].devName = "xxxxxx\0";
	  // gw_scanlist[i].bdAddr = malloc(6);
	  // gw_scanlist[i].bdAddr = "xx";
	   gw_scanlist[i].bdAddr[0] = 'x';
	   gw_scanlist[i].bdAddr[1] = 'x';
	   gw_scanlist[i].bdAddr[2] = '\0';
	   gw_scanlist[i].addrType = 'x';
   }

   for(i=0;i<DEFAULT_MAX_CONN_RES; i++)
   {
	   gw_connlist[i].devIndex = 9;
	   gw_connlist[i].devName = malloc(20);
	   gw_connlist[i].devName = "xxxxxx\0";
	//   gw_connlist[i].bdAddr = malloc(6);
	   //gw_connlist[i].bdAddr = "xx";
	   gw_connlist[i].bdAddr[0] = 'x';
	   gw_connlist[i].bdAddr[1] = 'x';
	   gw_connlist[i].bdAddr[2] = '\0';
	   gw_connlist[i].addrType = 'x';
   }

   // Initilise gw_charlist
   for(i=0;i < MAX_CHAR_IN_DEV_SUPPORTED; i++)
   {
	   gw_charlist[i].char_index = 'x';
	   gw_charlist[i].charName = malloc(20);
	   gw_charlist[i].charName = "xxxxxx\0";
	   gw_charlist[i].getsetflag = 'x';
   }
}


//****************************************************************************
//                           Http_Init FUNCTION
//****************************************************************************
void Http_Init()
{
    // Initialize Global Variables to be used in HTTP
    InitializeHttpGwVariables();
	RegisterGwCB(GW_EVENT_CB,HTTP_EventCb);
	RegisterGwCB(GW_DATA_CB,HTTP_DataCb);
}

#endif
