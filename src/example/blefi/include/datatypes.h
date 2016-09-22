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

/**
 * @defgroup datatypes
 *
 * @{
 */


#ifndef __DATATYPES_H__
#define __DATATYPES_H__


#ifdef	__cplusplus
extern "C" {
#endif

#include "uart_if.h"

#ifndef NULL
#define NULL        (0)
#endif

#ifndef FALSE
#define FALSE       (0)
#endif

#ifndef TRUE
#define TRUE        (!FALSE)
#endif

#ifndef OK
#define OK          (0)
#endif

#ifndef _INT8
#define _INT8
typedef signed   char   INT8;
#endif

#ifndef _UINT8
#define _UINT8
typedef unsigned char   UINT8;
#endif

#ifndef _INT16
#define _INT16
typedef signed   short  INT16;
#endif

#ifndef _UINT16
#define _UINT16
typedef unsigned short  UINT16;
#endif

#ifndef _BOOLEAN
#define _BOOLEAN
typedef unsigned char   BOOLEAN;
#endif

#ifdef _WIN32
    typedef unsigned int    UINT32, *PUINT32;
    typedef signed   int    INT32, *PINT32;
#else

#ifndef _INT32
#define _INT32
typedef signed   long   INT32;
#endif

#ifndef _UINT32
#define _UINT32
typedef unsigned long   UINT32;
#endif

#ifndef _UINT64
#define _UINT64
typedef unsigned long long   UINT64;
#endif

#endif /* _WIN32 */

typedef int             INT;
typedef char            CHAR;

typedef float			FLOAT;
typedef double			DOUBLE;

#define KEYLEN 16
#define B_MAX_ADV_LEN 31

// ******** Enumerations ******************************************************** //
#if 0
#undef FALSE
#undef TRUE


typedef enum _bool_t
{
	FALSE = 0,
	TRUE  = 1,
}bool_t;
#endif

#ifndef CONST
  #define CONST const
#endif

#ifndef GENERIC
  #define GENERIC
#endif

typedef unsigned char uint8;
typedef unsigned short int  uint16;
typedef unsigned long uint32;
typedef signed char   int8;
typedef signed int    int16;
typedef signed long   int32;

typedef signed char   bStatus_t;

#ifdef NOTERM
#define UART_PRINT(x,...)
#define DBG_PRINT(x,...)
#define ERR_PRINT(x)
#else
#define UART_PRINT Report
#define DBG_PRINT(x,...)
#define ERR_PRINT(x) Report("Error [%d] at line [%d] in function [%s]  \n\r",x,__LINE__,__FUNCTION__)
#endif

// Loop forever, user can change it as per application's requirement
#define LOOP_FOREVER() \
            {\
                while(1); \
            }

// check the error code and handle it
#define ASSERT_ON_ERROR(error_code)\
            {\
                 if(error_code < 0) \
                   {\
                        ERR_PRINT(error_code);\
                        return error_code;\
                 }\
            }

#define LOOP_ON_ERROR(error_code)\
            {\
                 if(error_code < 0) \
                   {\
                        ERR_PRINT(error_code);\
                        while(1);\
                 }\
            }

// for one sec, 20000000 counts
//#define            DELAY_COUNT 20000000 - working - 1Sec
//#define            DELAY_COUNT (20000 * 100) // 100 Ms - Working
#define DELAY_1_MS 20000 // 1 ms
#define DELAY_1_SEC 20000000
#define DELAY_100_MS 2000000
#define DELAY_500_MS 10000000


typedef struct blefiCfg
{
	unsigned int * cc26xxCRC;
	void *  mqttCfg;
	void *  bonds;
	/*
	 * Value in seconds. If < 30 then autoScan is disabled
	 * Max timeout 300 seconds.
	 */
	unsigned short * autoScan;
	unsigned short * autocon;
	char * otaMeta;

}blefiCfg_t;

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif /* __DATATYPES_H__ */
