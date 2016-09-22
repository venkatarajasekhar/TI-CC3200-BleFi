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

#ifndef SCHEMA_H_
#define SCHEMA_H_

#include "database.h"
/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 *  MACROS
 */

/************** IR temp service********************/



//Sensor Tag schema

/**
 * SENSOR Attribute Type format.
 */
typedef struct uuidinfo_t
{
	unsigned char uuid_len;         //!< Length of UUID
	unsigned char *uuid; //!< Pointer to char UUID
} uuidinfo_t;


/**
 * GATT Characteristic format.
 */
typedef struct Characteristic_t
{

	uuidinfo_t char_uuid; //!< Attribute type (2 or 16 octet UUIDs)
	char* pCharName;
}Characteristic_t;


/**
 * GATT Service format.
 */
typedef struct Service_t
{
  uuidinfo_t serv_uuid; //!< Attribute type (2 or 16 octet UUIDs)
  unsigned char* pServName;
  unsigned short int char_count;
  Characteristic_t* Characterstic;
}Service_t;


/**
 * BLE Profile format.
 */
typedef struct Profile_t
{
  unsigned char dev_count;
  unsigned short int serv_count;
  char * pProfileName;
  Service_t* Service;// pointer to the service array 
}Profile_t;

/**
 * Schema format.
 */
#define MAX_PROFILES 10
typedef struct Schema_t
{
  Profile_t* Profile[MAX_PROFILES];
}Schema_t;

#if 0
typedef struct schema_uuid_info_t
{
  unsigned char serv_uuid_len;         //!< Length of UUID
  unsigned char *serv_uuid; //!< Pointer to service UUID
  unsigned char char_uuid_len;         //!< Length of UUID
  unsigned char *char_uuid; //!< Pointer to char UUID
}schema_uuid_info_t;
#endif

#define MAX_FILE_NAME 32
#define SCHEMA_EXT ".sch"
#define FIRST_SERVICE_START 2
#define SCH_SERVICE(i,j) sch_nvmemload.Profile[i]->Service[j]
#define SCH_CHAR(i,j,k) sch_nvmemload.Profile[i]->Service[j].Characterstic[k]
#define JSON_STRLEN(i) (tokenList[i].end - tokenList[i].start)
#define CHAR_TOKEN_NUM 3

/*********************************************************************
 *  FUNCTION DECLARATIONS
 */

unsigned char Schema_FindProfile(char * devicename , unsigned char devicename_len,void** pProfilePtr);
void Schema_FindServUUID( Profile_t* pProfilePtr, uint16* pServCnt , uuidinfo_t* pServUuidInfo  );
void Schema_FindCharUUID( Service_t* pServicePtr, uint16* pCharCnt , uuidinfo_t* pCharUuidInfo  );
void Schema_FindAttrUUID( Characteristic_t* pCharPtr, uint16* pAttrCnt , uuidinfo_t* pAttrUuidInfo  );
void Schema_FindString( void* pProfilePtr , db_uuid_info_t* pdb_uuid_info , char** pCharString );
void Schema_FindCharString( Profile_t* pProfilePtr, db_uuid_info_t* pdb_uuid_info , char**  pCharString  );
void Schema_DelProfile( Profile_t* pProfilePtr);






/**************************************************/

#endif /* SCHEMA_H_ */
