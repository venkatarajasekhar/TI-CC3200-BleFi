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

#ifndef __CLI_H__
#define	__CLI_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "datatypes.h"
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "osi.h"
#include "uart.h"
#include "pin.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "uart_if.h"

/****************** typedef declarations ******************/

//******************************************************************************
// Maximum no. of characters per command
//******************************************************************************
#define MAX_CMD_LEN     100

//******************************************************************************
// Maximum no. of arguments per command
//******************************************************************************

#define MAX_NOF_ARGS	10

//******************************************************************************
// Maximum no. commands stored in history
//******************************************************************************

#define MAX_HISTORY	10


#define MAX_COMMANDS_SUPPORTED				(21)
#define MAX_LENGTH_COMMAND					(20)
#define TERMINATE_INTERPRETER_MODE          (-2)

#define COMMAND_PROMPT						"\r\nG:> "
#define COMMAND_PROMPT_SAMELINE				"\rG:> "
#define ILLEGAL_COMMAND						"Illegal command"
#define ILLEGAL_INPUT						"Illegal input"
#define cli_printf 							Report

#define N_HELP_ENTRIES 15
#define BUF_SIZE 						(100)
#define MCU_RX_BUFFER_OVERHEAD_SIZE		(32)

#define OSI_STACK_SIZE	2048

#ifndef JTAG_TERM
#define cli_putchar(c) MAP_UARTCharPut(CONSOLE,c)
#define cli_getchar() MAP_UARTCharGet(CONSOLE)
#else
#define cli_putchar(c) printf("%c",c)
#define cli_getchar() getchar()
#endif


typedef struct cli_cmd
{
	char *name;
	char *usage;
	char *help;
	int (*handler)(struct cli_cmd *cmdtbl,int argc,char **argv);
}cli_cmd_t;


/****************** Globals declaration ******************/


extern char g_cmd_help[N_HELP_ENTRIES][20];


/****************** Functions declaration ******************/


int btoa();
int htoa();
unsigned long atolong(char *data, unsigned long *retLong);
int CLI_task(void);
void basic_Interpreter(void);
extern void CLI_CreateTask(void);

#endif // __CLI_H__
