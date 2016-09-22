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


// Driverlib includes

//-----------------------------------------------------------------------------
/** \brief Send command.
 *
 * \param[in] ui32Cmd
 *      The command to send.
 * \param[in] pcSendData
 *      Pointer to the data to send with the command.
 * \param[in] ui32SendLen
 *      The number of bytes to send from \e pcSendData.
 * \return
 *      Returns SBL_SUCCESS, ...
 */
//-----------------------------------------------------------------------------

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include "datatypes.h"
#include "osi.h"
#include "rom.h"
#include "rom_map.h"
#include "hw_memmap.h"
#include "hw_types.h"
#include "uart.h"
#include <simplelink.h>

#define SBL_CC2650_PAGE_ERASE_SIZE          4096
#define SBL_CC2650_FLASH_START_ADDRESS      0x00000000
#define SBL_CC2650_RAM_START_ADDRESS        0x20000000
#define SBL_CC2650_ACCESS_WIDTH_32B         1
#define SBL_CC2650_ACCESS_WIDTH_8B          0
#define SBL_CC2650_PAGE_ERASE_TIME_MS       20
#define SBL_CC2650_MAX_BYTES_PER_TRANSFER   252
#define SBL_CC2650_FLASH_SIZE_CFG           0x4003002C
#define SBL_CC2650_RAM_SIZE_CFG             0x40082250
#define SBL_CC2650_BL_CONFIG_PAGE_OFFSET    0xFDB
#define SBL_CC2650_BL_CONFIG_ENABLED_BM     0xc5
#define SBL_CC2650_MAX_BYTES_PER_TRANSFER   252
#define CRC_BLOCK_SIZE						200


enum {
	CMD_PING             = 0x20,
	CMD_DOWNLOAD         = 0x21,
	CMD_GET_STATUS       = 0x23,
	CMD_SEND_DATA        = 0x24,
	CMD_RESET            = 0x25,
	CMD_SECTOR_ERASE	 = 0x26,
	CMD_CRC32            = 0x27,
	CMD_GET_CHIP_ID      = 0x28,
	CMD_MEMORY_READ      = 0x2A,
	CMD_MEMORY_WRITE     = 0x2B,
	CMD_BANK_ERASE		 = 0x2C,
	CMD_SET_CCFG         = 0x2D,
};


enum {
    CMD_RET_SUCCESS      = 0x40,
    CMD_RET_UNKNOWN_CMD  = 0x41,
    CMD_RET_INVALID_CMD  = 0x42,
    CMD_RET_INVALID_ADR  = 0x43,
    CMD_RET_FLASH_FAIL   = 0x44,
};


#define SBL_PORT_ERROR (-1)
#define SBL_ARGUMENT_ERROR 2

#define DEVICE_ACK 1
#define DEVICE_NACK 0

#define SBL_FALSE 0
#define SBL_TRUE 1

#define SBL_SUCCESS 0
#define SBL_ERROR 1

#define sbl_send(x) MAP_UARTCharPut(UARTA0_BASE,x)
#define sbl_recv() MAP_UARTCharGet(UARTA0_BASE)

#define USER_FILE_NAME "cc26xx.bin"

#define CC26XX_FW_UPDATE_NEEDED 	1
#define CC26XX_FW_UPDATE_NOTNEEDED 	0

/*
 * EXTERN VARIABKES
 *
 **/
extern blefiCfg_t blefiCfgRec;


/**
 * Function Definitions
 * */
unsigned char addressInFlash(unsigned int ui32StartAddress,
                                         unsigned int ui32ByteCount/* = 1*/);
unsigned int cmdDownload(unsigned int ui32Address, unsigned int ui32Size);
unsigned int cmdSendData(const char *pcData, unsigned int ui32ByteCount);
unsigned int sendAutoBaud();
unsigned int sendCmd(unsigned int ui32Cmd, const char *pcSendData/* = NULL*/,
		unsigned int ui32SendLen/* = 0*/);
unsigned int writeFlashRange(unsigned int ui32StartAddress);
unsigned int sbl_min(unsigned int a, unsigned int b);
void sbl_close_bin(void);





//-----------------------------------------------------------------------------
/*
 * CC3200 functions related to SBL
 *
 */
//-----------------------------------------------------------------------------

long sblFileHandle;

//-----------------------------------------------------------------------------
/** Function sbl_min
*\brief
 *
 *
 */
//-----------------------------------------------------------------------------
unsigned int sbl_min(unsigned int a, unsigned int b)
{
	if (a<b)
	{
		return(a);
	}
	return(b);

}


//-----------------------------------------------------------------------------
/**\ FUNCTION
 *
 *
 */
//-----------------------------------------------------------------------------
unsigned int sbl_get_bin_size()
{
	SlFsFileInfo_t  	pFsFileInfo;
	int lRetVal;
	lRetVal = sl_FsGetInfo((unsigned char *)USER_FILE_NAME,0,&pFsFileInfo);
	if(lRetVal < 0)
	{
		Message("\n\rError Opening the file");
		return(0);
	}
	return(pFsFileInfo.FileLen);
}


//-----------------------------------------------------------------------------
/**\ FUNCTION
 *
 *
 */
//-----------------------------------------------------------------------------
int sbl_open_bin_for_read()
{

    unsigned long ulToken;
    int lRetVal;
	//
	// open a user file for reading
	//
	lRetVal = sl_FsOpen((unsigned char *)USER_FILE_NAME,
					 FS_MODE_OPEN_READ,
					 &ulToken,
					 &sblFileHandle);

	if(lRetVal < 0)
	{
		sl_FsClose(sblFileHandle, 0, 0, 0);
		Message("\n\rERROR - Could not open the file");
		return(lRetVal);
	}
	return 0;
}


//-----------------------------------------------------------------------------
/**\brief This function closes the SBL binary file
 *
 *
 */
//-----------------------------------------------------------------------------
void sbl_close_bin()
{
	//
	// close the user file
	//
	sl_FsClose(sblFileHandle, 0, 0, 0);

}


//-----------------------------------------------------------------------------
/**\ FUNCTION
 *
 *
 */
//-----------------------------------------------------------------------------
int sbl_read_bin(unsigned int offset,unsigned char * pcData, unsigned int bytesInTransfer)
{
	return (sl_FsRead(sblFileHandle,offset,pcData,bytesInTransfer));
}


//-----------------------------------------------------------------------------
/** \brief This function sets the the SBL status and the SBL error string.
 *
 * \param[in] ui32Status
 *      The new SBL status. SBL_SUCCESS, SBL_ERROR, ...
 * \param[in] pcFormat
 *      'printf' like format string.
 * \param[in] ...
 *      Input variables to the \e pcFormat string.
 *
 * \return
 *      Returns SBL_SUCCESS, ...
 */
//-----------------------------------------------------------------------------
unsigned char
generateCheckSum(unsigned int ui32Cmd, const char *pcData,
		unsigned int ui32DataLen)
{
	unsigned char ui8CheckSum = (unsigned char)ui32Cmd;
	unsigned int i;
    for(i = 0; i < ui32DataLen; i++)
    {
        ui8CheckSum += pcData[i];
    }
    return ui8CheckSum;
}


unsigned int
sendCmd(unsigned int ui32Cmd, const char *pcSendData/* = NULL*/,
		unsigned int ui32SendLen/* = 0*/)
{

    unsigned char pktLen = ui32SendLen + 3; // +3 => <1b Length>, <1B cksum>, <1B cmd>
    unsigned char pktSum = generateCheckSum(ui32Cmd, pcSendData, ui32SendLen);
    unsigned int i;

    //
    // Send packet
    //
    sbl_send(pktLen);
    sbl_send(pktSum);
    sbl_send((unsigned char)ui32Cmd);
    for(i=0;i<ui32SendLen;i++)
    {
    	sbl_send(*(pcSendData+i));
    }
    //
    // Empty and dealloc vector
    //
    return SBL_SUCCESS;
}


//-----------------------------------------------------------------------------
/** \brief Get ACK/NAK from the CC2650 device.
 *
 * \param[out] bAck
 *      True if response is ACK, false if response is NAK.
 * \param[in] ui32MaxRetries (optional)
 *      How many times ComPort::readBytes() can time out before fail is issued.
 *
 * \return
 *      Returns SBL_SUCCESS, ...
 */
//-----------------------------------------------------------------------------
unsigned int
getCmdResponse(unsigned char * bAck)
{
    unsigned char pIn[2];
    unsigned int recvData = 0;
    *bAck = 0;
    unsigned int bytesRecv = 0;

    //
    // Expect 2 bytes (ACK or NAK)
    //

    do
    {
    	recvData = sbl_recv();
    	//Report("\n\r[%x]",recvData);
    	if((recvData&0x00000500) == 0)
    	{
    		pIn[bytesRecv]=(unsigned char) recvData;
    		bytesRecv += 1;
    		//Report("\n\r{%x}",bytesRecv);
    	}
    }
    while(bytesRecv < 2);

    if(bytesRecv < 2)
    {
         return SBL_ERROR;
    }
    else
    {
        if(pIn[0] == 0x00 && pIn[1] == 0xCC)
        {
            *bAck = DEVICE_ACK;
            return SBL_SUCCESS;
        }
        else if(pIn[0] == 0x00 && pIn[1] == 0x33)
        {
        	Report("\t\t Error - NACK\n\r");
            *bAck = DEVICE_NACK;
        	return SBL_SUCCESS;
        }
        else
        {
        	Report("\t\t Error - Device did not respond with ACK or NACK\n\r");
            return SBL_ERROR;
        }
    }

}


//-----------------------------------------------------------------------------
/** \brief Send auto baud.
 *
 * \param[out] bBaudSetOk
 *      True if response is ACK, false otherwise
 *
 * \return
 *      Returns SBL_SUCCESS, ...
 */
//-----------------------------------------------------------------------------
unsigned int
sendAutoBaud()
{
    unsigned char bBaudSetOk = 0;
    //
    // Send 0x55 0x55 and expect ACK
    //

    sbl_send(0x55);
    sbl_send(0x55);

    if(getCmdResponse(&bBaudSetOk) != SBL_SUCCESS)
    {
        // No response received. Invalid baud rate?

    	Report("\t\t Error - Device did not receive Auto Baud Command. Invalid baud rate\n\r");
         return SBL_ERROR;
    }

    return SBL_SUCCESS;
}


//-----------------------------------------------------------------------------
/** \brief Get response data from CC2650 device.
 *
 * \param[out] pcData
 *      Pointer to where received data is stored.
 * \param[in|out] ui32MaxLen
 *      Max number of bytes that can be received. Is populated with the actual
 *      number of bytes received.
 * \param[in] ui32MaxRetries (optional)
 *      How many times ComPort::readBytes() can time out before fail is issued.
 * \return
 *      Returns SBL_SUCCESS, ...
 */
//-----------------------------------------------------------------------------
unsigned int
getResponseData(char *pcData, unsigned int *ui32MaxLen)
{
	char pcHdr[2];
    unsigned int numPayloadBytes;
    unsigned char hdrChecksum, dataChecksum;
    unsigned int bytesRecv = 0;

    //
    // Read length and checksum
    //
    do
	{
	 pcHdr[bytesRecv]=(unsigned char) sbl_recv();
	 bytesRecv += 1;
	}
	while(bytesRecv < 2);

    numPayloadBytes = pcHdr[0]-2;
    hdrChecksum = pcHdr[1];

    //
    // Check if length byte is too long.
    //
    if(numPayloadBytes > *ui32MaxLen)
    {
    	Report("\t\t Error - payload length byte is too long \n\r");
    	return SBL_ERROR;
    }

    //
    // Read the payload data
    //
    bytesRecv = 0;

    do
    {
    	*(pcData+bytesRecv) = (unsigned char) sbl_recv();
    	bytesRecv++;
    }
    while(bytesRecv < numPayloadBytes);

    //
    // Verify data checksum
    //
    dataChecksum = generateCheckSum(0, pcData, numPayloadBytes);
    if(dataChecksum != hdrChecksum)
    {
    	Report("\t\t Error - Data Checksum\n\r");
    	return SBL_ERROR;
    }

    *ui32MaxLen = bytesRecv;
    return SBL_SUCCESS;
}

//-----------------------------------------------------------------------------
/** \brief Send command response (ACK/NAK) to CC2650 device.
 *
 * \param[in] bAck
 *      True if response is ACK, false if response is NAK.
 * \return
 *      Returns SBL_SUCCESS, ...
 */
//-----------------------------------------------------------------------------
unsigned int
sendCmdResponse(unsigned char bAck)
{

    //
    // Send response
    //
   	sbl_send(0x00);
    sbl_send((bAck) ? 0xCC : 0x33);
    //Report("\t\t Sending Command response to CC2650 device\n\r");
    return SBL_SUCCESS;
}


//-----------------------------------------------------------------------------
/** \brief Get status from device.
 *
 * \param[out] pStatus
 *      Pointer to where status is stored.
 * \return
 *      Returns SBL_SUCCESS, ...
 */
//-----------------------------------------------------------------------------
unsigned int
readStatus(unsigned int *pui32Status)
{
	unsigned int retCode = SBL_SUCCESS;
	unsigned char bSuccess = SBL_FALSE;

    //
    // Send command
    //
    if((retCode = sendCmd(CMD_GET_STATUS,0,0)) != SBL_SUCCESS)
    {
    	Report("\t\t Error - Send Cmd Fail\n\r");
        return retCode;
    }

    //
    // Receive command response
    //
    if((retCode = getCmdResponse(&bSuccess)) != SBL_SUCCESS)
    {
    	Report("\t\t Error - Cmd Response Fail\n\r");
        return retCode;
    }

    if(!bSuccess)
    {
    	Report("\t\t Error - Not  able to get device status\n\r");
    	return SBL_ERROR;
    }

    //
    // Receive command response data
    //
    char status = 0;
    unsigned int ui32NumBytes = 1;
    if((retCode = getResponseData(&status, &ui32NumBytes)) != SBL_SUCCESS)
    {
        //
        // Respond with NAK
        //
        sendCmdResponse(SBL_FALSE);
        return retCode;
    }

    //
    // Respond with ACK
    //
    sendCmdResponse(SBL_TRUE);
    *pui32Status = status;
    return SBL_SUCCESS;
}


//-----------------------------------------------------------------------------
/** \brief Utility function for splitting 32 bit variable into char array
 *      (4 elements). Data are converted MSB, ie. \e pcDst[0] is the
 *      most significant byte.
 *
 * \param[in] ui32Src
 *      The 32 bit variable to convert.
 * \param[out] pcDst
 *      Pointer to the char array where the data is stored.
 *
 * \return
 *      void
 */
//-----------------------------------------------------------------------------
/*static */void
ulToCharArray(const unsigned int ui32Src, char *pcDst)
{
    // MSB first
    pcDst[0] =  (unsigned char)(ui32Src >> 24);
    pcDst[1] =  (unsigned char)(ui32Src >> 16);
    pcDst[2] =  (unsigned char)(ui32Src >> 8);
    pcDst[3] =  (unsigned char)(ui32Src >> 0);
}

unsigned int eraseFlashRange(unsigned int ui32StartAddress,unsigned int ui32ByteCount)
{
	unsigned int retCode = SBL_SUCCESS;
	unsigned char bSuccess = SBL_FALSE;
    char pcPayload[4];
    unsigned int devStatus;
    unsigned int i;


    //
    // Calculate retry count
    //
    unsigned int ui32PageCount = ui32ByteCount / SBL_CC2650_PAGE_ERASE_SIZE;
    if( ui32ByteCount % SBL_CC2650_PAGE_ERASE_SIZE) ui32PageCount ++;
	for( i = 0; i < ui32PageCount; i++)
	{

		//
		// Build payload
		// - 4B address (MSB first)
		//
		ulToCharArray(ui32StartAddress + i*(4096), &pcPayload[0]);

		//
		// Send command
		//
		if((retCode = sendCmd(CMD_SECTOR_ERASE, pcPayload, 4)) != SBL_SUCCESS)
		{
			return retCode;
		}

		//
		// Receive command response (ACK/NAK)
		//
		if((retCode = getCmdResponse(&bSuccess)) != SBL_SUCCESS)
		{
			return retCode;
		}
		if(!bSuccess)
		{
			Report("\t\t Error - Payload is not right \n\r");
			return SBL_ERROR;
		}

		//
		// Check device status (Flash failed if page(s) locked)
		//
		readStatus(&devStatus);
		if(devStatus != CMD_RET_SUCCESS)
		{
			Report("\t\t Error - Device Flash has failed or FLash page(s) are locked\n\r");
			return SBL_ERROR;
		}

	}
    return SBL_SUCCESS;
}


// Calculate crc32 checksum the way CC2538 and CC2650 does it.
int calcCrcLikeChip(const unsigned char *pData, unsigned long ulByteCount)
{
    unsigned long d, ind;
    unsigned long acc = 0xFFFFFFFF;
    const unsigned long ulCrcRand32Lut[] =
    {
        0x00000000, 0x1DB71064, 0x3B6E20C8, 0x26D930AC,
        0x76DC4190, 0x6B6B51F4, 0x4DB26158, 0x5005713C,
        0xEDB88320, 0xF00F9344, 0xD6D6A3E8, 0xCB61B38C,
        0x9B64C2B0, 0x86D3D2D4, 0xA00AE278, 0xBDBDF21C
    };

    while ( ulByteCount-- )
    {
        d = *pData++;
        ind = (acc & 0x0F) ^ (d & 0x0F);
        acc = (acc >> 4) ^ ulCrcRand32Lut[ind];
        ind = (acc & 0x0F) ^ (d >> 4);
        acc = (acc >> 4) ^ ulCrcRand32Lut[ind];
    }

    return (acc ^ 0xFFFFFFFF);
}


//-----------------------------------------------------------------------------
/** \brief Write \e unitCount words of data to device FLASH. Source array is
 *      8 bit wide. Parameters \e startAddress and \e unitCount must be a
 *      a multiple of 4. This function does not erase the flash before writing
 *      data, this must be done using e.g. eraseFlashRange().
 *
 * \param[in] ui32StartAddress
 *      Start address in device. Must be a multiple of 4.
 * \param[in] ui32ByteCount
 *      Number of bytes to program. Must be a multiple of 4.
 * \param[in] pcData
 *      Pointer to source data.
 * \return
 *      Returns SBL_SUCCESS, ...
 */
//-----------------------------------------------------------------------------
unsigned int
writeFlashRange(unsigned int ui32StartAddress)
{
	//unsigned int devStatus = CMD_RET_UNKNOWN_CMD;
	unsigned int retCode = SBL_SUCCESS;
	unsigned int bytesLeft, bytesInTransfer;
	unsigned int transferNumber = 0;
	unsigned int ui32ByteCount;
    unsigned char pcData[SBL_CC2650_MAX_BYTES_PER_TRANSFER];
    unsigned int devStatus;



    ui32ByteCount = sbl_get_bin_size();
    if(ui32ByteCount==0)
    {
    	Report("\n\rERROR - Binary not present");
    	return SBL_ERROR;
    }

    //
    //Send command to Erase the CC26xx flash
    //
    eraseFlashRange(ui32StartAddress,ui32ByteCount);

    //
	// Send download command to CC26xx
	//
	if((retCode = cmdDownload(ui32StartAddress,
			ui32ByteCount)) != SBL_SUCCESS)
	{
		return retCode;
	}

	//
	// Check status after download command
	//
	retCode = readStatus(&devStatus);
	if(retCode != SBL_SUCCESS)
	{
	   return retCode;
	}
	if(devStatus != CMD_RET_SUCCESS)
	{
	   return SBL_ERROR;
	}


	if( sbl_open_bin_for_read() <0)
	{
		Report("\n\rERROR - Binary cannot be read");
		return SBL_ERROR;
	}

	//
	// Send data in chunks
	//
	bytesLeft = ui32ByteCount;

	while(bytesLeft)
	{

		//
		// Limit transfer count
		//
		bytesInTransfer = sbl_min(SBL_CC2650_MAX_BYTES_PER_TRANSFER, bytesLeft);
		//Report("\n\rAdds:%x, count :%d, bytes-left :%d ",(ui32ByteCount-bytesLeft),transferNumber,bytesLeft);
		Report("\rbytes-left :%6d ",bytesLeft);

		//Read from SFLASH, and store it in pcData
		sbl_read_bin((unsigned int)(ui32ByteCount-bytesLeft),
				  pcData, bytesInTransfer);


		//
		// Send Data command
		//
		if((retCode = cmdSendData((const char *)pcData, bytesInTransfer)) != SBL_SUCCESS)
		{
			Report("\n\rError during flash download - Send Data");
			sbl_close_bin();
			//Report("FILL THIS");
			return retCode;

		}


		devStatus = 0;
		retCode = readStatus(&devStatus);
		if(retCode != SBL_SUCCESS)
		{
			Report("\n\rError during flash download-Read Status");
			sbl_close_bin();
		}
		if(devStatus != CMD_RET_SUCCESS)
		{
			Report("Device returned status %d\n", devStatus);
			continue;
		}
					//
		// Update index and bytesLeft
		//
		bytesLeft -= bytesInTransfer;
		transferNumber++;
	}

	//
	// close the user file
	//
	sbl_close_bin();
    return SBL_SUCCESS;
}


//-----------------------------------------------------------------------------
/** \brief This function sends the CC2650 download command and handles the
 *      device response.
 *
 * \param[in] ui32Address
 *      The start address in CC2650 flash.
 * \param[in] ui32ByteCount
 *      The total number of bytes to program on the device.
 *
 * \return
 *      Returns SBL_SUCCESS if command and response was successful.
 */
//-----------------------------------------------------------------------------
unsigned int
cmdDownload(unsigned int ui32Address, unsigned int ui32Size)
{
    int retCode = SBL_SUCCESS;
    unsigned char bSuccess = SBL_FALSE;


    //
    // Check input arguments
    //
    if(!addressInFlash(ui32Address, ui32Size))
    {
        //setState(SBL_ARGUMENT_ERROR, "Flash download: Address range (0x%08X + %d bytes) is not in device FLASH nor RAM.\n", ui32Address, ui32Size);
        return SBL_ARGUMENT_ERROR;
    }
    if(ui32Size & 0x03)
    {
        //setState(SBL_ARGUMENT_ERROR, "Flash download: Byte count must be a multiple of 4\n");
        return SBL_ARGUMENT_ERROR;
    }

    //
    // Generate payload
    // - 4B Program address
    // - 4B Program size
    //
    char pcPayload[8];
    ulToCharArray(ui32Address, &pcPayload[0]);
    ulToCharArray(ui32Size, &pcPayload[4]);

    //
    // Send command
    //
    if((retCode = sendCmd(CMD_DOWNLOAD, pcPayload, 8)) != SBL_SUCCESS)
    {
        return retCode;
    }

    //
    // Receive command response (ACK/NAK)
    //
    if((retCode = getCmdResponse(&bSuccess)) != SBL_SUCCESS)
    {
        return retCode;
    }

    //
    // Return command response
    //
    return (bSuccess) ? SBL_SUCCESS : SBL_ERROR;
}



//-----------------------------------------------------------------------------
/** \brief This function sends the CC2650 SendData command and handles the
 *      device response.
 *
 * \param[in] pcData
 *      Pointer to the data to send.
 * \param[in] ui32ByteCount
 *      The number of bytes to send.
 *
 * \return
 *      Returns SBL_SUCCESS if command and response was successful.
 */
//-----------------------------------------------------------------------------
unsigned int
cmdSendData(const char *pcData, unsigned int ui32ByteCount)
{
    unsigned int retCode = SBL_SUCCESS;
    unsigned char bSuccess = SBL_FALSE;

    //
    // Check input arguments
    //
    if(ui32ByteCount > SBL_CC2650_MAX_BYTES_PER_TRANSFER)
    {
       // setState(SBL_ERROR, "Error: Byte count (%d) exceeds maximum transfer size %d.\n", ui32ByteCount, SBL_CC2650_MAX_BYTES_PER_TRANSFER);
    	Report("\n\r Send Data : Error 1");
        return SBL_ERROR;
    }

    //
    // Send command
    //
    if((retCode = sendCmd(CMD_SEND_DATA, pcData, ui32ByteCount)) != SBL_SUCCESS)
    {
    	Report("\n\r Send Data : Error 2 : %d",retCode);
        return retCode;
    }

    //
    // Receive command response (ACK/NAK)
    //
    if((retCode = getCmdResponse(&bSuccess)) != SBL_SUCCESS)
    {
        return retCode;
    }
    if(!bSuccess)
    {
    	Report("\n\r Send Data : Error 3 : %d",bSuccess);
        return SBL_ERROR;
    }

    return SBL_SUCCESS;
}

//-----------------------------------------------------------------------------
/** \brief This function checks if the specified \e ui32StartAddress (and range)
 *      is located within the device RAM area.
 *
 * \return
 *      Returns true if the address/range is within the device RAM.
 */
//-----------------------------------------------------------------------------
unsigned char
addressInFlash(unsigned int ui32StartAddress,
                                         unsigned int ui32ByteCount/* = 1*/)
{
#if 0
    unsigned int ui32EndAddr = ui32StartAddress + ui32ByteCount;
    if(ui32StartAddress < SBL_CC2650_FLASH_START_ADDRESS)
    {
        return SBL_FALSE;
    }
    if(ui32EndAddr > (SBL_CC2650_FLASH_START_ADDRESS + getFlashSize()))
    {
        return SBL_FALSE;
    }
#endif
    return SBL_TRUE;
}


void Do_SBL_Dnld()
{
    sendCmd(0,0,0);
    //Report("SENDCOMMAND Sent\n\r");
    if(sendAutoBaud()==SBL_ERROR)
    {
    	Report("ERROR - Auto Baud Command Not Sent\n\r");
    }
    else
    {
    	Report("Downloading Firmware to CC26XX\n\r");
    	writeFlashRange(0x0);
    }
}


unsigned char CC26xx_SBL_CheckIfUpdateFwNeeded()
{
    int lRetVal;
	unsigned long ulByteCount;
	unsigned char *cc26xxData;
	SlFsFileInfo_t  	pFsFileInfo;
    long lFileHandle;
    unsigned long ulToken;
    unsigned int cc26xxCrc = 0x0;
//    unsigned char crcAvailFlag = (unsigned char)CRC_FILE_ABSENT_FLAG;
    unsigned int bytesLeft, bytesInTransfer;
	unsigned int transferNumber = 0;

	// Is cc26xx.bin present is Flash?
    lRetVal = sl_FsGetInfo((unsigned char *)USER_FILE_NAME,0,&pFsFileInfo);
    if(lRetVal < 0)
    {
    	Message("\n\r[SBL] No cc26xx.bin file to update");
    	return(CC26XX_FW_UPDATE_NOTNEEDED); // no update
    }
    // execution reaches here only if the CC26XX bin file is present
    // file is present in Flash . check if update is needed

	// read cc26xx.bin file from flash
	ulByteCount = pFsFileInfo.FileLen;
	//
	// open a user file for reading
	//
	lRetVal = sl_FsOpen((unsigned char *)USER_FILE_NAME,
						 FS_MODE_OPEN_READ,
						 &ulToken,
						 &lFileHandle);
    if(lRetVal < 0)
    {
		 lRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
		 Message("\n\r[SBL] Could not open the cc26xx.bin file");
		 return (0);
	}
    // cc26xx.bin file present. Now calculate crc
	cc26xxData = (unsigned char *)malloc(CRC_BLOCK_SIZE);
	if (cc26xxData == NULL)
	{
		Message("\n\r[SBL] NOT ENOUGH MEMORY TO CALCULATE THE CRC OF cc26xx.bin file");
		lRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
		// could'nt open crc.bin file . Update anyway.
		return (1);
	}

	// Now start calculating CRC of the bin file
	bytesLeft = ulByteCount;
	// Initialising before CRC calculation
	while(bytesLeft)
	{
		bytesInTransfer = sbl_min(CRC_BLOCK_SIZE, bytesLeft);

		// Enable this only for debug
		//Report("\n\rAdds:%x, count :%d ",(ulByteCount-bytesLeft),transferNumber);


		lRetVal = sl_FsRead(lFileHandle,
						  (unsigned int)(ulByteCount-bytesLeft),
						  cc26xxData, bytesInTransfer);

		//
		// Calculate CRC from USER_FILE_NAME
		//
		 cc26xxCrc = calcCrcLikeChip((const unsigned char *)cc26xxData, bytesInTransfer);		//
		// Update index and bytesLeft
		//
		bytesLeft -= bytesInTransfer;
		transferNumber++;
	}
	// End calculating CRC on bin file
	free(cc26xxData);

	lRetVal = sl_FsClose(lFileHandle, 0, 0, 0);

	//Read the CRC value that is stored in the CRC file stored in SFLASH
	if(cc26xxCrc!=*(blefiCfgRec.cc26xxCRC))
	{
	  Report("\n\r[SBL] stored crc %x, 26xx file crc %x",*(blefiCfgRec.cc26xxCRC),cc26xxCrc);
		//Update needed
	  *(blefiCfgRec.cc26xxCRC) = cc26xxCrc;
	  return(1);
	}
	else
	{
		//Update not needed
	  return(0);
	}

}



//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

    

