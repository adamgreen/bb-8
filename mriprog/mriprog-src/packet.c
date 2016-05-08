/*  Copyright (C) 2016  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
/* 'Class' to manage the sending and receiving of packets to/from gdb.  Takes care of crc and ack/nak handling too. */
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include "IComm.h"
#include "packet.h"
#include "hexConvert.h"


static IComm* g_pComm;


void packetInit(IComm* pComm)
{
    g_pComm = pComm;
}

void packetUninit(void)
{
    g_pComm = NULL;
}

static void initPacketStruct(Packet* pPacket, Buffer* pBuffer);
static void getMostRecentPacket(Packet* pPacket);
static void getPacketDataAndExpectedChecksum(Packet* pPacket);
static void waitForStartOfNextPacket(Packet* pPacket);
static char getNextCharFromGdb(Packet* pPacket);
static int  getPacketData(Packet* pPacket);
static void clearChecksum(Packet* pPacket);
static void updateChecksum(Packet* pPacket, char nextChar);
static void extractExpectedChecksum(Packet* pPacket);
static int  isChecksumValid(Packet* pPacket);
static void sendACKToGDB(void);
static void sendNAKToGDB(void);
static void resetBufferToEnableFutureReadingOfValidPacketData(Packet* pPacket);
void packetGet(Packet* pPacket, Buffer* pBuffer)
{
    initPacketStruct(pPacket, pBuffer);
    do
    {
        getMostRecentPacket(pPacket);
    } while(!isChecksumValid(pPacket));

    resetBufferToEnableFutureReadingOfValidPacketData(pPacket);
}

static void initPacketStruct(Packet* pPacket, Buffer* pBuffer)
{
    memset(pPacket, 0, sizeof(*pPacket));
    pPacket->pBuffer = pBuffer;
}

static void getMostRecentPacket(Packet* pPacket)
{
    do
    {
        getPacketDataAndExpectedChecksum(pPacket);
    } while (IComm_HasReceiveData(g_pComm));

    if (!isChecksumValid(pPacket))
    {
        sendNAKToGDB();
        return;
    }

    sendACKToGDB();
}

static void getPacketDataAndExpectedChecksum(Packet* pPacket)
{
    int completePacket;

    do
    {
        waitForStartOfNextPacket(pPacket);
        completePacket = getPacketData(pPacket);
    } while (!completePacket);

    extractExpectedChecksum(pPacket);
}

static void waitForStartOfNextPacket(Packet* pPacket)
{
    char nextChar = pPacket->lastChar;

    /* Wait for the packet start character, '$', and ignore all other characters. */
    while (nextChar != '$')
        nextChar = getNextCharFromGdb(pPacket);
}

static char getNextCharFromGdb(Packet* pPacket)
{
    char nextChar = IComm_ReceiveChar(g_pComm);
    pPacket->lastChar = nextChar;
    return nextChar;
}

static int getPacketData(Packet* pPacket)
{
    char nextChar;

    bufferReset(pPacket->pBuffer);
    clearChecksum(pPacket);
    nextChar = getNextCharFromGdb(pPacket);
    while (bufferBytesLeft(pPacket->pBuffer) > 0 && nextChar != '$' && nextChar != '#')
    {
        updateChecksum(pPacket, nextChar);
        bufferWriteChar(pPacket->pBuffer, nextChar);
        nextChar = getNextCharFromGdb(pPacket);
    }

    /* Return success if the expected end of packet character, '#', was received. */
    return (nextChar == '#');
}

static void clearChecksum(Packet* pPacket)
{
    pPacket->calculatedChecksum = 0;
}

static void updateChecksum(Packet* pPacket, char nextChar)
{
    pPacket->calculatedChecksum += (unsigned char)nextChar;
}

static void extractExpectedChecksum(Packet* pPacket)
{
    __try
    {
        char char1 = getNextCharFromGdb(pPacket);
        char char2 = getNextCharFromGdb(pPacket);
        unsigned char expectedChecksumHiNibble = HexCharToNibble(char1);
        unsigned char expectedChecksumLoNibble = HexCharToNibble(char2);
        pPacket->expectedChecksum = (expectedChecksumHiNibble << 4) | expectedChecksumLoNibble;
    }
    __catch
    {
        /* Force the checksum to mismatch if invalid hex digit was encountered. */
        pPacket->expectedChecksum = ~pPacket->calculatedChecksum;
    }
}

static int isChecksumValid(Packet* pPacket)
{
    return (pPacket->expectedChecksum == pPacket->calculatedChecksum);
}

static void sendACKToGDB(void)
{
    IComm_SendChar(g_pComm, '+');
}

static void sendNAKToGDB(void)
{
    IComm_SendChar(g_pComm, '-');
}

static void resetBufferToEnableFutureReadingOfValidPacketData(Packet* pPacket)
{
    bufferSetEndOfBuffer(pPacket->pBuffer);
    bufferReset(pPacket->pBuffer);
}


static void sendPacket(Packet* pPacket);
static void setPacketHeaderByte(Packet* pPacket);
static void setPacketData(Packet* pPacket);
static void setPacketChecksum(Packet* pPacket);
static void setByteAsHex(Packet* pPacket, unsigned char byte);
static int  receiveCharAfterSkippingControlC(Packet* pPacket);
void packetSend(Packet* pPacket, Buffer* pBuffer)
{
    char  charFromGdb;

    /* Keeps looping until GDB sends back the '+' packet acknowledge character.  If GDB sends a '$' then it is trying
       to send a packet so cancel this send attempt. */
    initPacketStruct(pPacket, pBuffer);
    do
    {
        sendPacket(pPacket);
        charFromGdb = receiveCharAfterSkippingControlC(pPacket);
    } while (charFromGdb != '+' && charFromGdb != '$');
}

static void sendPacket(Packet* pPacket)
{
    static char cache[64 * 1024];
    size_t      charCount;

    /* Make sure that the buffer isn't too large to fit in the cache. */
    if (bufferGetLength(pPacket->pBuffer) + 4 > sizeof(cache))
    {
        assert( bufferGetLength(pPacket->pBuffer) + 4 <= sizeof(cache) );
        return;
    }

    /* Send packet of format: "$<DataInHex>#<1ByteChecksumInHex> */
    bufferReset(pPacket->pBuffer);
    clearChecksum(pPacket);
    pPacket->pCacheCurr = cache;

    setPacketHeaderByte(pPacket);
    setPacketData(pPacket);
    setPacketChecksum(pPacket);

    charCount = pPacket->pCacheCurr - cache;
    IComm_SendBytes(g_pComm, cache, charCount);
}

static void setPacketHeaderByte(Packet* pPacket)
{
    *pPacket->pCacheCurr++ = '$';
}

static void setPacketData(Packet* pPacket)
{
    while (bufferBytesLeft(pPacket->pBuffer) > 0)
    {
        char currChar = bufferReadChar(pPacket->pBuffer);
        *pPacket->pCacheCurr++ =  currChar;
        updateChecksum(pPacket, currChar);
    }
}

static void setPacketChecksum(Packet* pPacket)
{
    *pPacket->pCacheCurr++ =  '#';
    setByteAsHex(pPacket, pPacket->calculatedChecksum);
}

static void setByteAsHex(Packet* pPacket, unsigned char byte)
{
    *pPacket->pCacheCurr++ = NibbleToHexChar[EXTRACT_HI_NIBBLE(byte)];
    *pPacket->pCacheCurr++ = NibbleToHexChar[EXTRACT_LO_NIBBLE(byte)];
}

static int receiveCharAfterSkippingControlC(Packet* pPacket)
{
    static const int controlC = 0x03;
    int              nextChar;

    do
    {
        nextChar = getNextCharFromGdb(pPacket);
    }
    while (nextChar == controlC);

    return nextChar;
}
