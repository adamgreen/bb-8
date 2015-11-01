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
#include <string.h>
#include "gdbRemote.h"
#include "packet.h"


static uint32_t g_maximumPacketSize;
static char*    g_pPacketData;


static uint32_t getMaximumPacketSize(void);
static void sendQuerySupportedCommand(void);
static uint32_t findPacketSizeInResponse();
static void skipToNextFieldInResponse(Buffer* pBuffer);
static void sendGetRegistersCommand(void);
static void receiveAndParseByteArrayResponse(uint8_t* pReadBuffer, uint8_t readSize);
static void sendSetRegisterCommand(const ArmContext* pContext);
static void sendByteArrayAsHex(Buffer* pBuffer, const uint8_t* pWriteBuffer, uint32_t writeSize);
static void verifyOkResponse(void);
static void sendReadMemoryCommand(uint32_t address, uint32_t readSize);
static void sendSingleStepCommand(void);
static int  parseSingleStepResponse(void);


void gdbRemoteInit(IComm* pComm)
{
    packetInit(pComm);
    g_maximumPacketSize = getMaximumPacketSize();
    g_pPacketData = malloc(g_maximumPacketSize);
    if (!g_pPacketData)
        __throw(outOfMemoryException);
}

static uint32_t getMaximumPacketSize(void)
{
    sendQuerySupportedCommand();
    return findPacketSizeInResponse();
}

static void sendQuerySupportedCommand(void)
{
    Buffer   buffer;
    Packet   packet;
    char     data[512];

    bufferInit(&buffer, data, sizeof(data));
    bufferWriteString(&buffer, "qSupported");
    bufferSetEndOfBuffer(&buffer);
    packetSend(&packet, &buffer);
}

static uint32_t findPacketSizeInResponse()
{
    static const char packetSize[] = "PacketSize=";
    Buffer            buffer;
    Packet            packet;
    char              data[512];

    bufferInit(&buffer, data, sizeof(data));
    packetGet(&packet, &buffer);
    while (bufferBytesLeft(&buffer) > 0)
    {
        if (bufferMatchesString(&buffer, packetSize, sizeof(packetSize)-1))
            return bufferReadUIntegerAsHex(&buffer);
        skipToNextFieldInResponse(&buffer);
    }
    __throw(badResponseException);
}

static void skipToNextFieldInResponse(Buffer* pBuffer)
{
    /* Fields are separated by semi-colon. */
    while (bufferBytesLeft(pBuffer) > 0 && bufferReadChar(pBuffer) != ';')
    {
    }
}


void gdbRemoteUninit(void)
{
    packetUninit();
    free(g_pPacketData);
    g_pPacketData = NULL;
    g_maximumPacketSize = 0;
}


void gdbRemoteGetRegisters(ArmContext* pContext)
{
    sendGetRegistersCommand();
    receiveAndParseByteArrayResponse((uint8_t*)&pContext->R[0], 17 * sizeof(uint32_t));
}

static void sendGetRegistersCommand(void)
{
    Buffer   buffer;
    Packet   packet;

    bufferInit(&buffer, g_pPacketData, g_maximumPacketSize);
    bufferWriteString(&buffer, "g");
    bufferSetEndOfBuffer(&buffer);
    packetSend(&packet, &buffer);
}

static void receiveAndParseByteArrayResponse(uint8_t* pReadBuffer, uint8_t readSize)
{
    uint8_t* pCurr;
    uint8_t* pEnd = pReadBuffer + readSize;
    Buffer   buffer;
    Packet   packet;

    bufferInit(&buffer, g_pPacketData, g_maximumPacketSize);
    packetGet(&packet, &buffer);

    for (pCurr = pReadBuffer ; pCurr < pEnd ; pCurr++)
        *pCurr = bufferReadByteAsHex(&buffer);
}



void gdbRemoteSetRegisters(const ArmContext* pContext)
{
    sendSetRegisterCommand(pContext);
    verifyOkResponse();
}

static void sendSetRegisterCommand(const ArmContext* pContext)
{
    Buffer   buffer;
    Packet   packet;

    bufferInit(&buffer, g_pPacketData, g_maximumPacketSize);
    bufferWriteChar(&buffer, 'G');
    sendByteArrayAsHex(&buffer, (uint8_t*)&pContext->R[0], 17 * sizeof(uint32_t));
    bufferSetEndOfBuffer(&buffer);
    packetSend(&packet, &buffer);
}

static void sendByteArrayAsHex(Buffer* pBuffer, const uint8_t* pWriteBuffer, uint32_t writeSize)
{
    const uint8_t* pEnd = pWriteBuffer + writeSize;
    const uint8_t* pCurr;

    for (pCurr = pWriteBuffer ; pCurr < pEnd ; pCurr++)
        bufferWriteByteAsHex(pBuffer, *pCurr);
}

static void verifyOkResponse(void)
{
    static const char expectedResponse[] = "OK";
    Buffer            buffer;
    Packet            packet;

    bufferInit(&buffer, g_pPacketData, g_maximumPacketSize);
    packetGet(&packet, &buffer);
    if (!bufferMatchesString(&buffer, expectedResponse, sizeof(expectedResponse)-1))
        __throw(badResponseException);
}



void gdbRemoteReadMemory(uint32_t address, void* pvReadBuffer, uint32_t readSize)
{
    sendReadMemoryCommand(address, readSize);
    receiveAndParseByteArrayResponse((uint8_t*)pvReadBuffer, readSize);
}

static void sendReadMemoryCommand(uint32_t address, uint32_t readSize)
{
    Buffer   buffer;
    Packet   packet;

    bufferInit(&buffer, g_pPacketData, g_maximumPacketSize);
    bufferWriteChar(&buffer, 'm');
    bufferWriteUIntegerAsHex(&buffer, address);
    bufferWriteChar(&buffer, ',');
    bufferWriteUIntegerAsHex(&buffer, readSize);
    bufferSetEndOfBuffer(&buffer);
    packetSend(&packet, &buffer);
}



void gdbRemoteWriteMemory(uint32_t address, const void* pvWriteBuffer, uint32_t writeSize)
{
    static const uint32_t packetOverhead = 1 + 8 + 1 + 4 + 1;
    uint32_t              maxDataBytes = (g_maximumPacketSize - packetOverhead) / 2;
    uint8_t*              pWriteBuffer = (uint8_t*)pvWriteBuffer;
    Buffer                buffer;
    Packet                packet;

    while (writeSize > 0)
    {
        uint32_t bytesToWrite = writeSize > maxDataBytes ? maxDataBytes : writeSize;

        bufferInit(&buffer, g_pPacketData, g_maximumPacketSize);
        bufferWriteChar(&buffer, 'M');
        bufferWriteUIntegerAsHex(&buffer, address);
        bufferWriteChar(&buffer, ',');
        bufferWriteUIntegerAsHex(&buffer, bytesToWrite);
        bufferWriteChar(&buffer, ':');
        sendByteArrayAsHex(&buffer, (uint8_t*)pWriteBuffer, bytesToWrite);
        bufferSetEndOfBuffer(&buffer);
        packetSend(&packet, &buffer);
        verifyOkResponse();

        writeSize -= bytesToWrite;
        address += bytesToWrite;
        pWriteBuffer += bytesToWrite;
    }
}



int gdbRemoteSingleStep(void)
{
    sendSingleStepCommand();
    return parseSingleStepResponse();
}

static void sendSingleStepCommand(void)
{
    Buffer   buffer;
    Packet   packet;

    bufferInit(&buffer, g_pPacketData, g_maximumPacketSize);
    bufferWriteString(&buffer, "s");
    bufferSetEndOfBuffer(&buffer);
    packetSend(&packet, &buffer);
}

static int parseSingleStepResponse(void)
{
    Buffer  buffer;
    Packet  packet;
    char    packetType;

    bufferInit(&buffer, g_pPacketData, g_maximumPacketSize);
    packetGet(&packet, &buffer);

    while ('O' == (packetType = bufferReadChar(&buffer)))
    {
        bufferInit(&buffer, g_pPacketData, g_maximumPacketSize);
        packetGet(&packet, &buffer);
    }

    if (packetType == 'T')
        return (int)bufferReadByteAsHex(&buffer);
    else
        __throw(badResponseException);
}

__throws void gdbRemoteContinue(void)
{
    Buffer   buffer;
    Packet   packet;

    bufferInit(&buffer, g_pPacketData, g_maximumPacketSize);
    bufferWriteString(&buffer, "c");
    bufferSetEndOfBuffer(&buffer);
    packetSend(&packet, &buffer);
}
