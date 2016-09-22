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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "datatypes.h"
#include "osi.h"
#include "gateway_api.h"
#include "cli_handler.h"
#include "wifi_api.h"
#include "gapbondmgr.h"

//#define UNUSED(x) ((x) = (x))

static char his_tbl[MAX_HISTORY][MAX_CMD_LEN+2];
static char cmdl[MAX_CMD_LEN];
cli_cmd_t cmd_tbl[MAX_COMMANDS_SUPPORTED];
unsigned char cli_cmd_count = 0;

extern void KickGenericTask();

void CliPuts(char *str)
{
	while(*str!='\0')
		MAP_UARTCharPut(CONSOLE, *str++);
}

//*****************************************************************************
//
//! Interpreter_Task
//!
//!  \param  pvParameters
//!
//!  \return none
//!
//!  \brief Task handler function to handle the Serial-WiFi functionality
//
//*****************************************************************************
static void CliInitHistory()
{
	unsigned short i;
	for(i=0;i<MAX_HISTORY;i++)
	{
		his_tbl[i][MAX_CMD_LEN]=(i+1)%MAX_HISTORY;
		his_tbl[i][0]='\0';
		his_tbl[i][MAX_CMD_LEN+1]=(i-1)%MAX_HISTORY;;
	}
	his_tbl[0][MAX_CMD_LEN+1]=MAX_HISTORY-1;

}



static unsigned short parse(char *arg,char *argv[])
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


OsiSyncObj_t pcliSyncObj;
char gCLIChar = 0;
void uarta0IntFuncHandler (void)
{
	gCLIChar = MAP_UARTCharGetNonBlocking(CONSOLE);
	UARTIntClear(CONSOLE,UART_INT_RT);
	osi_SyncObjSignalFromISR(&pcliSyncObj);
}

char CliGetcharOs()
{
	osi_SyncObjWait(&pcliSyncObj,OSI_WAIT_FOREVER);
	return(gCLIChar);
}
static int DoList(char *pCmd,int len)
{
	return 0;
}

static char CliGetCmd(int *argc,char *argv[])
{
	char c;
	int i=0;
	static int cur=0;
	int t_cur,tmp;
        char tabPress=0;
        char spPress=0;

	t_cur=cur;
	while(1)
	{
		c=CliGetcharOs();
		if(c==27)
		{
                        tabPress =0 ;
			c=CliGetcharOs();
			if(c==91)
			{
				c=CliGetcharOs();
				tmp=t_cur;
				if (c=='A')
				{
					t_cur=his_tbl[t_cur][MAX_CMD_LEN+1];
				}
				else if(c=='B')
				{
					t_cur=his_tbl[t_cur][MAX_CMD_LEN];

				}
				else
				{
					continue;
				}

				if(his_tbl[t_cur][0]=='\0')
					t_cur=tmp;
				cli_printf("\33[2K\r%s%s",COMMAND_PROMPT_SAMELINE,his_tbl[t_cur]);
				i=strlen(his_tbl[t_cur]);
			}


		}
                else if(c=='\t' && (i==0 ||spPress==0))
                {
                  tabPress++;
                  if(tabPress==2)
                  {
                    tabPress=0;
                   if(DoList(his_tbl[cur],i))
                   {
                      his_tbl[cur][i]='\0';
                      cli_printf("\33[2K\r%s%s",COMMAND_PROMPT,his_tbl[t_cur]);
                   }
                  }
                }
		else
		{	tabPress =0 ;
			if(t_cur!=cur)
			{
				strcpy(his_tbl[cur],his_tbl[t_cur]);
				t_cur=cur;
			}

			if(c==13)
			{
				his_tbl[cur][i]='\0';
				CliPuts("\n\r");
				break;
			}
			else
			{
				if(c!=8 && i<MAX_CMD_LEN)
				{
					his_tbl[cur][i++]=c;
					his_tbl[cur][i]='\0';
					cli_putchar(c);
				}
				else if(i>0)
				{
					his_tbl[cur][--i]='\0';
					cli_printf("\33[2K\r%s%s",COMMAND_PROMPT_SAMELINE,his_tbl[t_cur]);
				}
                                if(c==' ')
                                  spPress++;

			}
		}
	}

	strncpy(cmdl,his_tbl[cur],(sizeof(cmdl)-1));
	cmdl[sizeof(cmdl)-1] = '\0';
	*argc=parse(cmdl,argv);
	if(*argc!=0)
	{
		cur=(cur+1)%MAX_HISTORY;
		his_tbl[cur][0]='\0';
		return 0;
	}
	return 1;

}

int print_usage(cli_cmd_t *cmd)
{
	cli_printf("\n\rUsage :\n\r-----------\n\r%s\n\r\n\r",cmd->usage);
	return 1;
}


static int RunCommand(int argc,char *argv[])
{

	unsigned int i;

	for(i=0; i< cli_cmd_count; i++)
	{
		if(strcmp(cmd_tbl[i].name,argv[0])==0)
		{
			return (cmd_tbl[i].handler(cmd_tbl+i,argc,argv));
		}
	}
	cli_printf("\n%s command not found.\n\n\r",argv[0]);
	return -1;
}

static int CliAddCmd(char *name, char *usage, char *help, int (*handler)(cli_cmd_t *cmdtbl,int argc,char **argv))
{
	if (cli_cmd_count>=MAX_COMMANDS_SUPPORTED)
	{
		return -1;
	}

	cmd_tbl[cli_cmd_count].name = malloc(strlen(name)+1);
	strcpy(cmd_tbl[cli_cmd_count].name,name);

	cmd_tbl[cli_cmd_count].usage = malloc(strlen(usage)+1);
	strcpy(cmd_tbl[cli_cmd_count].usage,usage);

	cmd_tbl[cli_cmd_count].help = malloc(strlen(help)+1);
	strcpy(cmd_tbl[cli_cmd_count].help,help);


	cmd_tbl[cli_cmd_count].handler = handler;
	cli_cmd_count++;

	return 0;
}


static int DoHelp(cli_cmd_t *cmd,int argc,char *argv[])
{

	int i;

   //
   // argc=2 means a specific command help is being asked
   //

   if(argc == 2)
   {

	   for(i=0; i< cli_cmd_count; i++)
	   	{
	   		if(strcmp(cmd_tbl[i].name,argv[1])==0)
	   		{
	   			cli_printf("\n\r%s\n\r\n\rUsage :\n\r-----------\n\r%s\n\r\n\r",cmd[i].help,cmd[i].usage);
	   			return 0;
	   		}
	   	}
	cli_printf("\n\rUnknown command %s.Use help without parameters to list all commands\n\r\n\r",argv[1]);
	return 1;
   }
   else if (argc ==1)
   {
	//
	// argc=1 help of all commands needs to be displayed
	//
    cli_printf("\n\r%#-15s   %#-20s\n\r-----------------------------------\n\r","Command","Descrtion");
    for(i=0; i< cli_cmd_count; i++)
    {
	     cli_printf("%#-15s : %#-20s\n\r",cmd[i].name,cmd[i].help);
	}
	CliPuts("\n\r");
    }
    else
    {
    	return print_usage(cmd);
    }

	return 0;
}

static void CliInitCommandList()
{
	CliAddCmd("help","help [command]","Print help",DoHelp);
	CliAddCmd("scan","scan","Scan for devices",DoScan);
	CliAddCmd("list","list","List the devices",DoDevList);
	CliAddCmd("linke","linke [deviceId]","Establish link with device",DoLinkEstablish);
	CliAddCmd("linkt","linkt [deviceId]","Terminate link with device",DoLinkTerminate);
	CliAddCmd("listchar","listchar [deviceId]","Get the value of the Characteristic value",DoListChar);
	CliAddCmd("get","get [deviceId] [CharString]","Get the value of the Characteristic value",DoGet);
	CliAddCmd("set","set [deviceId] [CharString] [Length of the value] [Value]","Set the value of the Characteristic value",DoSet);
	CliAddCmd("wlan_connect","wlan_connect [ssid] [key]\r\n  ssid: AP name key: password in case of WPA","Connect to wlan AP.",DoWlanconnect);
	CliAddCmd("wlan_disconnect","wlan_disconnect","wlan_disconnect",DoWlanDisconnect);
	CliAddCmd("bleupdate","bleupdate","Update the BLE Software",DoCc26xxFwUpdate);
	CliAddCmd("reset","reset","blefi reset",DoReset);
	CliAddCmd("addotameta","addotameta [otametastring]","add otameta",DoAddOtaMeta);
	CliAddCmd("autoscan", "autoscan [0 to disable, 30-600 secs]","change autoscan period",DoAutoScan);
	CliAddCmd("mqttgwmode", "mqttgwmode [mode] [server]\r\n mode 0-demo,1-quickstart \r\nserver in demo mode","set Gatway Mqtt Mode",DomqttGwMode);
	CliAddCmd("mqttdevmode", "mqttdevmode [mode] [server]\r\n mode 0-demo,1-quickstart \r\nserver in demo mode","set Device Mqtt Mode",DomqttDevMode);
	CliAddCmd("loaddefault", "Loads default values for autoscan, mqtt, ble-fw","put blefi in default",DoDefault);
	CliAddCmd("triggerota", "triggerota","Trigger OTA",triggerOta);
	CliAddCmd("autoconlist", "pairlist","List of autoconnect devices",DoPairList);
	CliAddCmd("rmautocon", "unpair","remove from autoconnect list",DoUnPair);
	CliAddCmd("autocon", "autocon 0/1","Autoconnect Enable/Disable",DoAutoconnect);



}


void ClitaskFxn(void* pValue)
{
	char *argv[MAX_NOF_ARGS];
	int argc;
	volatile unsigned char LoopVar = 0xFF;


	/*
	 * Connect to network before proceeding with CLI
	 * */
    NetworkInit();
    KickGenericTask();
    CliInitHistory();
	CliInitCommandList();

	while(LoopVar)
	{
		osi_Sleep(100);
		CliPuts(COMMAND_PROMPT);
		if(CliGetCmd(&argc,argv)==0)
		RunCommand(argc,argv);
	}
}

//*****************************************************************************
//
//!CLI_Init
//!
//! \param
//! \param
//!
//! \return none
//!
void CLI_Init(void)
{
	OsiReturnVal_e eRetVal;

	eRetVal = osi_SyncObjCreate(&pcliSyncObj);
	LOOP_ON_ERROR(eRetVal);

#ifdef BLEFI_BOARD_V1
	osi_InterruptRegister(INT_UARTA0, (P_OSI_INTR_ENTRY)uarta0IntFuncHandler, 0x20);
#else
	osi_InterruptRegister(INT_UARTA1, (P_OSI_INTR_ENTRY)uarta0IntFuncHandler, 0x20);
#endif
	UARTIntClear(CONSOLE,UART_INT_RT);
	UARTIntEnable(CONSOLE,UART_INT_RT);

	eRetVal = osi_TaskCreate(ClitaskFxn,(const signed char *)"CLI Interpreter",
					OSI_STACK_SIZE, NULL,1, NULL);
	LOOP_ON_ERROR(eRetVal);

	RegisterGwCB(GW_EVENT_CB,CLI_EventCb);

	RegisterGwCB(GW_DATA_CB,CLI_DataCb);
}
