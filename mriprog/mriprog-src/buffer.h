/*  Copyright (C) 2014  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
/*  'Class' which represents a text buffer.  Has routines to both extract and inject strings of various types into the
    buffer while verifying that no overflow takes place. */
#ifndef _BUFFER_H_
#define _BUFFER_H_

#include <stddef.h>
#include <stdint.h>

typedef struct
{
    char*   pStart;
    char*   pEnd;
    char*   pCurrent;
} Buffer;


void     bufferInit(Buffer* pBuffer, char* pBufferStart, size_t bufferSize);
void     bufferReset(Buffer* pBuffer);
void     bufferSetEndOfBuffer(Buffer* pBuffer);
size_t   bufferBytesLeft(Buffer* pBuffer);
int      bufferOverrunDetected(Buffer* pBuffer);
size_t   bufferGetLength(Buffer* pBuffer);
char*    bufferGetArray(Buffer* pBuffer);
void     bufferWriteChar(Buffer* pBuffer, char character);
char     bufferReadChar(Buffer* pBuffer);
void     bufferWriteByteAsHex(Buffer* pBuffer, uint8_t byte);
uint8_t  bufferReadByteAsHex(Buffer* pBuffer);
void     bufferWriteString(Buffer* pBuffer, const char* pString);
void     bufferWriteSizedString(Buffer* pBuffer, const char* pString, size_t length);
uint32_t bufferReadUIntegerAsHex(Buffer* pBuffer);
void     bufferWriteUIntegerAsHex(Buffer* pBuffer, uint32_t value);
int32_t  bufferReadIntegerAsHex(Buffer* pBuffer);
void     bufferWriteIntegerAsHex(Buffer* pBuffer, int32_t value);
int      bufferIsNextCharEqualTo(Buffer* pBuffer, char thisChar);
int      bufferMatchesString(Buffer* pBuffer, const char* pString, size_t stringLength);

#endif /* _BUFFER_H_ */
