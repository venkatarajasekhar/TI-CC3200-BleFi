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


long atolong(char *data, unsigned long *retLong);



//*****************************************************************************
//
//! atod
//!
//! \param  none
//!
//! \return Decimal value of ASCII char
//!
//! \brief  Convert ASCII char to decimal
//
//*****************************************************************************
unsigned char atod(char data)
{
    unsigned char retVal = 0xff;

    if ((data >= 0x30) && (data <= 0x39))
    {
        retVal = data - 0x30;
    }

    return retVal;
}


//*****************************************************************************
//
//! atolong
//!
//! \param  none
//!
//! \return Return long value else -1 as error
//!
//! \brief  Convert ASCII string to long
//
//*****************************************************************************
long atolong(char *data, unsigned long *retLong)
{
    unsigned char cycleCount = 0;
    unsigned char digit;

    if((data == NULL) || (retLong == NULL))
    {
        return (-1);
    }

    *retLong = 0;
    while ((digit = atod(*data)) != 0xff)
    {
        *retLong *= 10;
        *retLong += digit;
        data++;
        cycleCount++;
    }

    return cycleCount;
}
