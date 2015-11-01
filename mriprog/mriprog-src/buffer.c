/*  Copyright (C) 2015  Adam Green (https://github.com/adamgreen)

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
#include <limits.h>
#include <string.h>
#include "buffer.h"
#include "try_catch.h"
#include "hexConvert.h"

void bufferInit(Buffer* pBuffer, char* pBufferStart, size_t bufferSize)
{
    pBuffer->pStart = pBufferStart;
    pBuffer->pEnd = pBufferStart + bufferSize;
    bufferReset(pBuffer);
}


void bufferReset(Buffer* pBuffer)
{
    pBuffer->pCurrent = pBuffer->pStart;
}


void bufferSetEndOfBuffer(Buffer* pBuffer)
{
    if (pBuffer->pCurrent < pBuffer->pEnd)
        pBuffer->pEnd = pBuffer->pCurrent;
}


size_t bufferBytesLeft(Buffer* pBuffer)
{
    if (bufferOverrunDetected(pBuffer))
        return 0;

    return (size_t)(pBuffer->pEnd - pBuffer->pCurrent);
}


size_t bufferGetLength(Buffer* pBuffer)
{
    if (pBuffer->pStart > pBuffer->pEnd)
        return 0;

    return (size_t)(pBuffer->pEnd - pBuffer->pStart);
}


char* bufferGetArray(Buffer* pBuffer)
{
    return pBuffer->pStart;
}


int bufferOverrunDetected(Buffer* pBuffer)
{
    return pBuffer->pCurrent > pBuffer->pEnd;
}


static void throwExceptionAndFlagBufferOverrunIfBufferLeftIsSmallerThan(Buffer* pBuffer, size_t size);
static void recordThatBufferOverrunHasOccurred(Buffer* pBuffer);
void bufferWriteChar(Buffer* pBuffer, char character)
{
    throwExceptionAndFlagBufferOverrunIfBufferLeftIsSmallerThan(pBuffer, 1);

    *(pBuffer->pCurrent++) = character;
}

static void throwExceptionAndFlagBufferOverrunIfBufferLeftIsSmallerThan(Buffer* pBuffer, size_t size)
{
    if (bufferBytesLeft(pBuffer) < size)
    {
        recordThatBufferOverrunHasOccurred(pBuffer);
        __throw(bufferOverrunException);
    }
}

static void recordThatBufferOverrunHasOccurred(Buffer* pBuffer)
{
    pBuffer->pCurrent = pBuffer->pEnd + 1;
}


char bufferReadChar(Buffer* pBuffer)
{
    throwExceptionAndFlagBufferOverrunIfBufferLeftIsSmallerThan(pBuffer, 1);

    return *(pBuffer->pCurrent++);
}


void bufferWriteByteAsHex(Buffer* pBuffer, uint8_t byte)
{
    throwExceptionAndFlagBufferOverrunIfBufferLeftIsSmallerThan(pBuffer, 2);

    *(pBuffer->pCurrent++) = NibbleToHexChar[EXTRACT_HI_NIBBLE(byte)];
    *(pBuffer->pCurrent++) = NibbleToHexChar[EXTRACT_LO_NIBBLE(byte)];
}


uint8_t bufferReadByteAsHex(Buffer* pBuffer)
{
    unsigned char byte;

    throwExceptionAndFlagBufferOverrunIfBufferLeftIsSmallerThan(pBuffer, 2);

    byte = HexCharToNibble(pBuffer->pCurrent[0]) << 4;
    byte |= HexCharToNibble(pBuffer->pCurrent[1]);
    pBuffer->pCurrent += 2;

    return byte;
}


void bufferWriteString(Buffer* pBuffer, const char* pString)
{
    bufferWriteSizedString(pBuffer, pString, strlen(pString));
}


void bufferWriteSizedString(Buffer* pBuffer, const char* pString, size_t length)
{
    throwExceptionAndFlagBufferOverrunIfBufferLeftIsSmallerThan(pBuffer, length);

    while (length--)
        *(pBuffer->pCurrent++) = *pString++;
}


static uint32_t parseNextHexDigitAndAddNibbleToValue(Buffer* pBuffer, uint32_t currentValue);
static void     pushBackLastChar(Buffer* pBuffer);
static void     clearOverrun(Buffer* pBuffer);
uint32_t bufferReadUIntegerAsHex(Buffer* pBuffer)
{
    int      hexDigitsParsed;
    uint32_t value = 0;

    for (hexDigitsParsed = 0 ; ; hexDigitsParsed++)
    {
        __try
            value = parseNextHexDigitAndAddNibbleToValue(pBuffer, value);
        __catch
            break;
    }
    /* Read buffer until non-hex digit or end of buffer was detected but don't want to return the invalid hex digit
       or buffer overrun exception to the caller so clear out the exception code that might have gotten us out of
       the above loop. */
    clearExceptionCode();
    clearOverrun(pBuffer);

    if (hexDigitsParsed == 0)
        __throw(invalidValueException);

    return value;
}

static uint32_t parseNextHexDigitAndAddNibbleToValue(Buffer* pBuffer, uint32_t currentValue)
{
    char     nextChar;
    uint32_t nibbleValue;

    __try
        nextChar = bufferReadChar(pBuffer);
    __catch
        __rethrow;

    __try
    {
        nibbleValue = HexCharToNibble(nextChar);
    }
    __catch
    {
        pushBackLastChar(pBuffer);
        __rethrow;
    }

    return (currentValue << 4) + nibbleValue;
}

static void pushBackLastChar(Buffer* pBuffer)
{
    if (pBuffer->pCurrent > pBuffer->pStart)
        pBuffer->pCurrent--;
}

static void clearOverrun(Buffer* pBuffer)
{
    if (bufferOverrunDetected(pBuffer))
        pBuffer->pCurrent = pBuffer->pEnd;
}


static int     countLeadingZeroBytes(uint32_t value);
static uint8_t extractByteAtIndex(uint32_t value, int index);
void bufferWriteUIntegerAsHex(Buffer* pBuffer, uint32_t value)
{
    int              leadingZeroBytes;
    int              currentByteIndex;

    if (value == 0)
    {
        bufferWriteByteAsHex(pBuffer, 0);
        return;
    }

    leadingZeroBytes = countLeadingZeroBytes(value);
    currentByteIndex = ((int)sizeof(value) - leadingZeroBytes) - 1;
    while (currentByteIndex >= 0)
        bufferWriteByteAsHex(pBuffer, extractByteAtIndex(value, currentByteIndex--));
}

static int countLeadingZeroBytes(uint32_t value)
{
    uint32_t mask = 0xFF000000;
    int      count = 0;

    while (mask && 0 == (value & mask))
    {
        count++;
        mask >>= 8;
    }

    return count;
}

static uint8_t extractByteAtIndex(uint32_t value, int index)
{
    static const int bitsPerByte = 8;
    uint32_t         shiftAmount = index * bitsPerByte;

    return (uint8_t)((value >> shiftAmount) & 0xff);
}


static int32_t convertToIntegerAndThrowIfOutOfRange(int isNegative, uint32_t value);
int32_t bufferReadIntegerAsHex(Buffer* pBuffer)
{
    uint32_t value = 0;
    int      isNegative = 0;

    isNegative = bufferIsNextCharEqualTo(pBuffer, '-');
    value = bufferReadUIntegerAsHex(pBuffer);
    return convertToIntegerAndThrowIfOutOfRange(isNegative, value);
}

static int32_t convertToIntegerAndThrowIfOutOfRange(int isNegative, uint32_t value)
{
    if (!isNegative && value > INT_MAX)
        __throw(invalidValueException);

    if (isNegative && value > ((uint32_t)INT_MAX + 1))
    {
        __throw(invalidValueException);
    }

    return isNegative ? -(int)value : (int)value;
}


static int32_t calculateAbsoluteValueAndWriteMinusSignForNegativeValue(Buffer* pBuffer, int32_t value);
void bufferWriteIntegerAsHex(Buffer* pBuffer, int32_t value)
{
    value = calculateAbsoluteValueAndWriteMinusSignForNegativeValue(pBuffer, value);
    bufferWriteUIntegerAsHex(pBuffer, (uint32_t)value);
}

static int32_t calculateAbsoluteValueAndWriteMinusSignForNegativeValue(Buffer* pBuffer, int32_t value)
{
    if (value < 0)
    {
        value = -value;
        bufferWriteChar(pBuffer, '-');
    }

    return value;
}


static void throwExceptionIfBufferLeftIsSmallerThan(Buffer* pBuffer, size_t size);
static char peekAtNextChar(Buffer* pBuffer);
static void advanceToNextChar(Buffer* pBuffer);
int bufferIsNextCharEqualTo(Buffer* pBuffer, char thisChar)
{
    throwExceptionIfBufferLeftIsSmallerThan(pBuffer, 1);

    if (peekAtNextChar(pBuffer) == thisChar)
    {
        advanceToNextChar(pBuffer);
        return 1;
    }

    return 0;
}

static void throwExceptionIfBufferLeftIsSmallerThan(Buffer* pBuffer, size_t size)
{
    if (bufferBytesLeft(pBuffer) < size)
        __throw(bufferOverrunException);
}

static char peekAtNextChar(Buffer* pBuffer)
{
    return *pBuffer->pCurrent;
}

static void advanceToNextChar(Buffer* pBuffer)
{
    pBuffer->pCurrent++;
}


static int doesBufferContainThisString(Buffer* pBuffer, const char* pDesiredString, size_t stringLength);
int bufferMatchesString(Buffer* pBuffer, const char* pString, size_t stringLength)
{
    throwExceptionIfBufferLeftIsSmallerThan(pBuffer, stringLength);

    if(doesBufferContainThisString(pBuffer, pString, stringLength))
    {
        pBuffer->pCurrent += stringLength;
        return 1;
    }

    return 0;
}

static int doesBufferContainThisString(Buffer* pBuffer, const char* pDesiredString, size_t stringLength)
{
    const char* pBufferString = pBuffer->pCurrent;

    return (strncmp(pBufferString, pDesiredString, stringLength) == 0) &&
           (bufferBytesLeft(pBuffer) == stringLength ||
            pBufferString[stringLength] == ':' ||
            pBufferString[stringLength-1] == '=');
}
