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

#ifndef __CLI_HANDLER_H__
#define	__CLI_HANDLER_H__

#include "hw_types.h"
#include "datatypes.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "datatypes.h"
#include "osi.h"
#include "uart.h"
#include "pin.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "uart_if.h"
#include "gateway_api.h"
#include "cli.h"



/****************** typedef declarations ******************/


/****************** Globals declaration ******************/




/****************** Functions declaration ******************/


void CLI_EventCb(uint8 EventType, ErrorFlag_t ErrorFlag,void* EventParams);
void CLI_DataCb(uint8 DataType, ErrorFlag_t ErrorFlag,void* DataParams);
int DoScan(cli_cmd_t *cmd,int argc,char *argv[]);
int DoStopScan(cli_cmd_t *cmd,int argc,char *argv[]);
int DoDevList(cli_cmd_t *cmd,int argc,char *argv[]);
int DoLinkEstablish(cli_cmd_t *cmd,int argc,char *argv[]);
int DoLinkTerminate(cli_cmd_t *cmd,int argc,char *argv[]);
int DoGetDevCharList(cli_cmd_t *cmd,int argc,char *argv[]);
int DoListChar(cli_cmd_t *cmd,int argc,char *argv[]);
int DoGet(cli_cmd_t *cmd,int argc,char *argv[]);
int DoSet(cli_cmd_t *cmd,int argc,char *argv[]);
int DoEraseBonds(cli_cmd_t *cmd,int argc,char *argv[]);
int DoWlanconnect(cli_cmd_t *cmd,int argc,char *argv[]);
int DoWlanDisconnect(cli_cmd_t *cmd,int argc,char *argv[]);
int DoCc26xxFwUpdate(cli_cmd_t *cmd,int argc,char *argv[]);
int DoReset(cli_cmd_t *cmd,int argc,char *argv[]);
int DoAddOtaMeta(cli_cmd_t *cmd,int argc,char *argv[]);
int DoMqttReset(cli_cmd_t *cmd,int argc,char *argv[]);
int DoMqttGwInfo(cli_cmd_t *cmd,int argc,char *argv[]);
int DoMqttDevInfo(cli_cmd_t *cmd,int argc,char *argv[]);
int DoMqttAddDev(cli_cmd_t *cmd,int argc,char *argv[]);
int DoMqttDelDev(cli_cmd_t *cmd,int argc,char *argv[]);
int DoAutoScan(cli_cmd_t *cmd,int argc,char *argv[]);
int DomqttGwMode(cli_cmd_t *cmd,int argc,char *argv[]);
int DomqttDevMode(cli_cmd_t *cmd,int argc,char *argv[]);
int DoDefault(cli_cmd_t *cmd,int argc,char *argv[]);
int triggerOta(cli_cmd_t *cmd,int argc,char *argv[]);
int OtaDownload();
int DoPairList(cli_cmd_t *cmd,int argc,char *argv[]);
int DoUnPair(cli_cmd_t *cmd,int argc,char *argv[]);
int DoAutoconnect(cli_cmd_t *cmd,int argc,char *argv[]);


#endif //__CLI_HANDLER_H__
