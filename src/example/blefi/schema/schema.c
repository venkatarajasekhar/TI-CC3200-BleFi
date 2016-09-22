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

//
//schema.c - contains the schema related functions of BleFi
//

/*********************************************************************
 * INCLUDES
 */
#include <stdlib.h>
#include <stdio.h>
#include "datatypes.h"
#include "schema.h"
#include "jsmn.h"
#include "simplelink.h"
#include "uart_if.h"

/*********************************************************************
* GLOBAL VARIABLES
*/

Schema_t sch_nvmemload;

/*********************************************************************
* FUNCTIONS
*/

void sch_copy_charstring(unsigned int token_idx, unsigned int p_idx, unsigned int serv_idx, unsigned int char_idx, char * ptr);
int sch_copy_char_uuid(unsigned int token_idx, unsigned int p_idx, unsigned int serv_idx, unsigned int char_idx, char * ptr);
int sch_copy_serv_uuid(unsigned int token_idx, unsigned int p_idx, unsigned int serv_idx, char * ptr);
int ParseJSONData(char *ptr, char * devicename, char p_idx);
long ReadFileFromDevice(char * filename, char ** ptr);
int Schema_loadfile(char * devicename, char ** ptr);

extern void HandleError(char *, unsigned int, unsigned char error);


jsmntok_t   *tokenList;

void Schema_init()
{
	unsigned char i;
	for(i=0;i<MAX_PROFILES;i++)
	{
		sch_nvmemload.Profile[i] = NULL;
	}
}


/*********************************************************************
 * @fn       
 *
 * @brief  
 *
 * @param   none
 *
 * @return  none
 */
unsigned char Schema_FindProfile(char * devicename , unsigned char devicename_len,void** pProfilePtr)
{
	unsigned char profile_found = FALSE;
	char first_null = 0xFF;
	unsigned short int i ;
	char * JsonString;

	/*
	 *
	 * First check if the schema is already loaded on to RAM
	 *
	 * */

    for(i=0 ; i<MAX_PROFILES ; i++)
   	{
    	if((Profile_t*)sch_nvmemload.Profile[i]!=NULL)
    	{
    		if (strncmp (((Profile_t*)sch_nvmemload.Profile[i])->pProfileName,
			          devicename, devicename_len) == 0 )
    		{
    			profile_found = TRUE;

    			(((Profile_t*)sch_nvmemload.Profile[i])->dev_count)++;
    			*pProfilePtr = ((Profile_t*)sch_nvmemload.Profile[i]);
    			return profile_found;
    		}
		}
    	else
    	{
    		if(first_null == 0xFF)
    		{
    			first_null = i;
    		}

    	}
   	}

	if(first_null >= MAX_PROFILES)
	{
		HandleError(__FILE__,__LINE__,1); // fatal error
		return 1;
	}

	/*
	 * If the Schema is not present in RAM
	 * then check if it is present in FLASH
	 * <devicename.sch> would be the file name in FLASH
	 * */

	/*
	 * Check if the devicename.sch is present in the SFLASH
	 * if not, then schema is not present, return false
	 * if yes, then load the schema
	 * */

    /*
     * Schema_load file allocates the memory for JsonString
     * */

	if(Schema_loadfile(devicename, &JsonString)<0)
	{
		//DBG_PRINT("\n\r[SCH] Error loading the file %s",devicename);
		profile_found=FALSE;
		return NULL;
	}
	if(ParseJSONData(JsonString, devicename,first_null)<0)
	{
		DBG_PRINT("\n\r[SCH] Error parsing JSON string");
		profile_found=FALSE;
		free(JsonString);
		return NULL;
	}
	profile_found = TRUE;

	*pProfilePtr = (Profile_t*)(sch_nvmemload.Profile[first_null]);
	free(JsonString);
	return profile_found;
}

/*********************************************************************
 * @fn
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */

int Schema_loadfile(char * devicename, char ** ptr)
{
	unsigned char devname_len;
	char * dev_filename;
	int retVal;
	/*
	 * Initial checks for device name
	 * */

	if(devicename == NULL)
	{
		DBG_PRINT("\n\rDevicename NULL");
		return (-1);
	}
	devname_len = strlen(devicename);
	if(devname_len > MAX_FILE_NAME)
	{
		DBG_PRINT("\n\rDevicename BIG");
		return (-1);
	}
	dev_filename = malloc(devname_len+strlen(SCHEMA_EXT)+1);
	if(dev_filename == NULL)
	{
		DBG_PRINT("\n\r[SCH] Error - not enough memory for dev_filename ...");
		HandleError(__FILE__, __LINE__, 1);
		return (-1);
	}
	sprintf(dev_filename,"%s%s",devicename,SCHEMA_EXT);
	retVal = ReadFileFromDevice(dev_filename, ptr);
	free(dev_filename);
	return(retVal);
}

/*********************************************************************
 * @fn
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */
long ReadFileFromDevice(char * filename, char ** ptr)
{
    long lRetVal = -1;
    unsigned long ulToken;
    long lFileHandle;
    SlFsFileInfo_t pFsFileInfo;
    unsigned int ui32ByteCount;

    lRetVal = sl_FsGetInfo((unsigned char *)filename, 0, &pFsFileInfo);
    if(lRetVal < 0)
    {
    	DBG_PRINT("\n\r[SCH] Schema file : %s Not present",filename);
    	return(lRetVal);
    }
    DBG_PRINT("\n\r[SCH] Schema file : %s present",filename);
    ui32ByteCount = pFsFileInfo.FileLen;
    *ptr = malloc(ui32ByteCount);
    if(*ptr == NULL)
    {
    	DBG_PRINT("\n\r[SCH] ERROR : Not enough memory for *ptr ");
    	return -1;
    }

    lRetVal = sl_FsOpen((unsigned char *) filename, FS_MODE_OPEN_READ, &ulToken, &lFileHandle);
    if(lRetVal < 0)
    {
    	    lRetVal = sl_FsClose(lFileHandle,0,0,0);
        	DBG_PRINT("\n\r[SCH] ERROR : Schema file-open failed");
        	free(*ptr);
        	return(-1);
    }
    lRetVal = sl_FsRead(lFileHandle,0,(unsigned char *)*ptr,(unsigned int)ui32ByteCount);

    lRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
    if (SL_RET_CODE_OK != lRetVal)
    {
    	DBG_PRINT("\n\r[SCH] ERROR : File Info not present");
    	free(*ptr);
    	return(lRetVal);
    }

    return 0;
}



/*********************************************************************
 * @fn Schema_FindServString
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */

void Schema_FindCharString( Profile_t* pProfilePtr, db_uuid_info_t* pdb_uuid_info  , char**  pCharString)
{

  unsigned short int i ;
  unsigned short int ServCnt = pProfilePtr->serv_count;
  unsigned short int ServIndexMatch;
  volatile unsigned short int CharIndexMatch;
  unsigned char ServIndexMatchFlag = FALSE , CharIndexMatchFlag = FALSE;


  for(i=0 ; i<ServCnt; i++)
  {
	  if(( ((pProfilePtr->Service + i)->serv_uuid.uuid_len) == pdb_uuid_info->serv_uuid_len )
	  		&&
	  ( memcmp((((pProfilePtr->Service + i)->serv_uuid.uuid)),
			   pdb_uuid_info->serv_uuid , pdb_uuid_info->serv_uuid_len )== 0 )  )
	  {
		  ServIndexMatch = i;
		  ServIndexMatchFlag = TRUE;
		  break;
	  }
  }

	if(ServIndexMatchFlag == TRUE)
	{
	  for(i=0 ; i<(pProfilePtr->Service + ServIndexMatch)->char_count ; i++ )
	  {
		  Characteristic_t* Characterstic_ptr = ((pProfilePtr->Service + ServIndexMatch)->Characterstic + i);
		  if(((Characterstic_ptr->char_uuid.uuid_len) == pdb_uuid_info->char_uuid_len)
					&&
			  (memcmp(((Characterstic_ptr->char_uuid.uuid)),
					   pdb_uuid_info->char_uuid , pdb_uuid_info->char_uuid_len )== 0))
		  {
			  CharIndexMatch = i;
			  *pCharString = Characterstic_ptr->pCharName ;
			  CharIndexMatchFlag = TRUE;
			  break;

		  }
	  }
	}

  if(CharIndexMatchFlag != TRUE)
  {
	  *pCharString =  (void*)(NULL);

  }

	// return CharIndexMatchFlag ;

}



/*********************************************************************
 * @fn
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */

void Schema_FindString( void* pProfilePtr , db_uuid_info_t* pdb_uuid_info , char** pCharString )
{
  if(pProfilePtr != NULL)
  {
	  Schema_FindCharString( (Profile_t*)pProfilePtr,pdb_uuid_info , pCharString  );
  }

}


/*********************************************************************
 * @fn
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */

void Schema_DelProfile( Profile_t* pProfilePtr)
{
	unsigned char i ;

	for ( i=0; i< MAX_PROFILES ; i++)
	{
		// check if the profile  is found in Schema in RAM
		if(sch_nvmemload.Profile[i] == pProfilePtr)
		{
			sch_nvmemload.Profile[i]->dev_count--;
			if(sch_nvmemload.Profile[i]->dev_count == 0)
			{
				free(sch_nvmemload.Profile[i]);
				sch_nvmemload.Profile[i] = NULL;
			}
			return;
		}
	}
}


//*****************************************************************************
//
//! \brief Handler for parsing JSON data
//!
//! \param[in]  ptr - Pointer to http response body data
//!
//! \return 0 on success else error code on failure
//!
//*****************************************************************************


int ParseJSONData(char *ptr, char * devicename, char p_idx)
{
    int noOfToken;
    jsmn_parser parser;

    unsigned int token_index;
    unsigned int serv_index, char_token, char_index;

    jsmn_init(&parser);
    noOfToken = jsmn_parse(&parser, ptr, strlen(ptr), NULL, 10);

    tokenList = (jsmntok_t *) malloc(noOfToken*sizeof(jsmntok_t));
    if(tokenList == NULL)
    {
        UART_PRINT("\n\r[SCH] Failed to allocate memory\n\r");
        HandleError(__FILE__, __LINE__, 1);

        // Control should never reach here
        return (-1);
    }
    jsmn_init(&parser);
    noOfToken = jsmn_parse(&parser, ptr, strlen(ptr), tokenList, noOfToken);

//Load schema to sch_nvmemload
    sch_nvmemload.Profile[p_idx] = malloc(sizeof(Profile_t));
    if(sch_nvmemload.Profile[p_idx] == NULL)
    {
        UART_PRINT("\n\r[SCH] Failed to allocate memory\n\r");
        HandleError(__FILE__, __LINE__, 1);
    }


//Now fill the profile
    /**
     * BLE Profile format.
     */

    sch_nvmemload.Profile[p_idx]->serv_count = tokenList[0].size;
    sch_nvmemload.Profile[p_idx]->pProfileName = malloc(strlen(devicename)+1);
    if(sch_nvmemload.Profile[p_idx]->pProfileName == NULL)
    {
    	UART_PRINT("\n\rMalloc Error");
    	HandleError(__FILE__, __LINE__, 1); // Fatal Error
    }
    strcpy(sch_nvmemload.Profile[p_idx]->pProfileName,devicename);
    sch_nvmemload.Profile[p_idx]->Service = malloc(sizeof(Service_t)*sch_nvmemload.Profile[p_idx]->serv_count);
    if(sch_nvmemload.Profile[p_idx]->Service == NULL)
    {
        	UART_PRINT("\n\rMalloc Error");
        	//while(1);
        	HandleError(__FILE__, __LINE__, 1); // Fatal Error
    }
    sch_nvmemload.Profile[p_idx]->dev_count = 1; //This is the first time loading a profile

//Now fill the services
    token_index = FIRST_SERVICE_START;
    for(serv_index=0;serv_index<sch_nvmemload.Profile[p_idx]->serv_count; serv_index++)
    {
    	//copy the service uuid
    	if(sch_copy_serv_uuid(token_index-1,p_idx,serv_index,ptr)<0)
    	{
    		DBG_PRINT("\n\r Error in Service UUID");
    		return(-1);
    	}

    	//fill the number of characterstics, given by the array size
    	SCH_SERVICE(p_idx,serv_index).char_count = tokenList[token_index].size;
    	//allocate the memory for characterstics and copy them
    	SCH_SERVICE(p_idx,serv_index).Characterstic = malloc(sizeof(Characteristic_t)*SCH_SERVICE(p_idx,serv_index).char_count);
    	if(SCH_SERVICE(p_idx,serv_index).Characterstic == NULL)
    	{
    		Report("\n\rMalloc Error");
    		//while(1);
    		HandleError(__FILE__, __LINE__, 1); // Fatal Error
    	}

    	char_token = token_index + 1;
    	for(char_index=0;char_index<SCH_SERVICE(p_idx,serv_index).char_count;char_index++)
    	{

    		if(sch_copy_char_uuid(char_token+1,p_idx,serv_index,char_index, ptr)<0)
    		{
    			DBG_PRINT("\n\r Wrong UUID");
    		}
    		sch_copy_charstring(char_token+2,p_idx,serv_index,char_index, ptr);
    		char_token += CHAR_TOKEN_NUM;
    	}
    	token_index = char_token+1;
    }

    free(tokenList);

    return 0;
}

/*********************************************************************
 * @fn
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */

unsigned char sch_char_to_hex(char c)
{
  if (c >= 'A')
    return c - 'A' + 10;
  else
    return c - '0';
}

/*********************************************************************
 * @fn
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */

int sch_copy_serv_uuid(unsigned int token_idx, unsigned int p_idx, unsigned int serv_idx, char * ptr)
{
	int char_len, uuid_len,char_index;
	char lsb, msb;
	char_len = JSON_STRLEN(token_idx);
	// The UUID should be 2 Octets or 16 octets
	if (char_len != 4 && char_len !=32)
	{
		return(-1);
	}
	uuid_len = char_len/2;
	SCH_SERVICE(p_idx,serv_idx).serv_uuid.uuid_len = uuid_len;
	SCH_SERVICE(p_idx,serv_idx).serv_uuid.uuid = malloc(uuid_len);
	if(SCH_SERVICE(p_idx, serv_idx).serv_uuid.uuid == NULL)
	{
		Report("\n\r Malloc Error");
	}
	char_index = 0;
	while(uuid_len>0)
	{
		msb = sch_char_to_hex(*(ptr+tokenList[token_idx].start+char_index));
		char_index++;
		lsb = sch_char_to_hex(*(ptr+tokenList[token_idx].start+char_index));
		char_index++;
		SCH_SERVICE(p_idx,serv_idx).serv_uuid.uuid[uuid_len-1] = msb*16 + lsb;
		uuid_len -=1;
	}

	return 0;
}

/*********************************************************************
 * @fn
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */

int sch_copy_char_uuid(unsigned int token_idx, unsigned int p_idx, unsigned int serv_idx, unsigned int char_idx, char * ptr)
{
	int char_len, uuid_len,char_index;
	char lsb, msb;
	char_len = JSON_STRLEN(token_idx);
	// The UUID should be 2 Octets or 16 octets
	if (char_len != 4 && char_len !=32)
	{
		return(-1);
	}
	uuid_len = char_len/2;
	SCH_CHAR(p_idx,serv_idx,char_idx).char_uuid.uuid_len = uuid_len;
	SCH_CHAR(p_idx,serv_idx,char_idx).char_uuid.uuid = malloc(uuid_len);
	char_index = 0;
	while(uuid_len>0)
	{
		msb = sch_char_to_hex(*(ptr+tokenList[token_idx].start+char_index));
		char_index++;
		lsb = sch_char_to_hex(*(ptr+tokenList[token_idx].start+char_index));
		char_index++;
		SCH_CHAR(p_idx,serv_idx,char_idx).char_uuid.uuid[uuid_len-1] = msb*16 + lsb;
		uuid_len -=1;
	}

	return 0;
}

/*********************************************************************
 * @fn
 *
 * @brief
 *
 * @param   none
 *
 * @return  none
 */
void sch_copy_charstring(unsigned int token_idx,unsigned int p_idx, unsigned int serv_idx, unsigned int char_idx, char * ptr)
{
	unsigned int char_len;
	char_len = JSON_STRLEN(token_idx);
	SCH_CHAR(p_idx,serv_idx,char_idx).pCharName = malloc(char_len + 1);
	if(SCH_CHAR(p_idx, serv_idx,char_idx).pCharName==NULL)
	{
		UART_PRINT("\n\r Malloc Error");
		//while(1);
		HandleError(__FILE__, __LINE__,1); // Fatal Error
	}
	strncpy(SCH_CHAR(p_idx, serv_idx,char_idx).pCharName,ptr+tokenList[token_idx].start,char_len);
	SCH_CHAR(p_idx,serv_idx,char_idx).pCharName[char_len]='\0';
}



#if 0
New schema
Format
{
   "F000AA0004514000B000000000000000": [
    {
      "F000AA0104514000B000000000000000": "/Temp/Data"

    },
   {
      "F000AA0204514000B000000000000000": "/Temp/Cfg"
     }
    ],
    "F000AA1004514000B000000000000000": [
    {
      "F000AA1104514000B000000000000000": "/Acc/Data"
     }
  ]
}

The profile is an object with variable array (array of services)
Service is an array of variable characterstics
Characterstic is an object of size two elements each

token 0 -> start of profile object, size is npo
token 1 -> first service UUID string. Need to use the start and end values and compute the size of uuid.
the uuid is in string format, hence need to convert to hex.
token 2 -> first service array start, size is nsa
token 3 -> character object start, size is nco (which is always 2)
token 4 -> the UUID of character
token 5 -> the string of character eg "/temp/data"
Every character object takes 3 tokens
Next service string(ss[i+1] is always at (3*nsa[i])+1+1 from previous service string.

#endif
