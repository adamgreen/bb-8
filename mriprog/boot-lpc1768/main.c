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
/* UART based FLASH bootloader for the NXP LPC1768. */
#include <string.h>
#include "LPC17xx.h"
#include "protocol.h"


/* Labels exported by linker from LPC1768.ld */
extern uint32_t __bss_start__[1];
extern uint32_t __bss_end__[1];


static LPC_UART_TypeDef* const g_uartConfigurations[] =
{
    (LPC_UART_TypeDef*)LPC_UART0,
    (LPC_UART_TypeDef*)LPC_UART1,
    (LPC_UART_TypeDef*)LPC_UART2,
    (LPC_UART_TypeDef*)LPC_UART3
};

static LPC_UART_TypeDef* g_pCurrentUart;

static int      g_allSectorsErased;
static uint32_t g_lastSectorErased = 0xFFFFFFFF;

static const uint8_t g_startOfPacket[4] = PACKET_START_MARKER;


/* LPC In-Application Programming Interface */
#define IAP_LOCATION ((void (*)(uint32_t* ,uint32_t*)) 0x1FFF1FF1)
void (*iapEntry)(uint32_t* ,uint32_t*) = IAP_LOCATION;

#define IAP_PREPARE_SECTORS     50
#define IAP_COPY_RAM_TO_FLASH   51
#define IAP_ERASE_SECTORS       52

#define IAP_CMD_SUCCESS         0
#define IAP_INVALID_COMMAND     1
#define IAP_SRC_ADDR_ERROR      2
#define IAP_DST_ADDR_ERROR      3
#define IAP_SRC_ADDR_NOT_MAPPED 4
#define IAP_DST_ADDR_NOT_MAPPED 5
#define IAP_COUNT_ERROR         6
#define IAP_INVALID_SECTOR      7
#define IAP_SECTOR_NOT_BLANK    8
#define IAP_SECTOR_NOT_PREPARED_FOR_WRITE_OPERATION 9
#define IAP_COMPARE_ERROR       10
#define IAP_BUSY                11

#define LPC1768_SECTOR_COUNT    30
// UNDONE: Should be calculated.
#define LPC1768_CPU_SPEED_KHZ   96000


static void zeroFillBssSection(void);
static void findMriUart(void);
static uint32_t isInterruptEnabled(IRQn_Type IRQn);
static int uartReceiveChar(void);
static void waitForUartToReceiveData(void);
static uint32_t uartHasReceiveData(void);
static void uartSendChar(int Character);
static void waitForUartToAllowTransmit(void);
static uint32_t targetUartCanTransmit(void);
static void sendInitCompletedPacket(void);
static void sendPacketHeader(PacketHeader* pHeader);
static void sendBytes(const void* pvBuffer, size_t bufferSize);
static uint32_t calculateCRC(const PacketHeader* pHeader, const void* pvData);
static void readPacket(PacketHeader* pHeader, uint8_t* pData, size_t dataSize);
static void readPacketHeader(PacketHeader* pHeader);
static void waitForStartOfPacket(void);
static void readBytesFromComm(void* pvBuffer, size_t bufferSize);
static void sendAckPacket(void);
static void sendNakPacket(uint32_t errorCode);
static void waitForUartToFlush(void);
static uint32_t targetUartHasFlushed(void);
static int iapEraseAll(void);
static int iapPrepareSectors(uint32_t startSector, uint32_t endSector);
static int iapEraseSectors(uint32_t startSector, uint32_t endSector);
static int iapWriteSectors(uint32_t flashAddress, uint8_t* pData, uint32_t dataLength);
static uint32_t iapSectorFromAddress(uint32_t flashAddress);
static int iapCopyRamToFlash(uint32_t flashAddress, uint8_t* pRam, uint32_t length);


int main(void)
{
    PacketHeader header;
    uint8_t*     pData = (uint8_t*)0x10000000;

    __disable_irq();

    zeroFillBssSection();
    crcTableInit();
    findMriUart();

    sendInitCompletedPacket();

    while (1)
    {
        int iapResult = -1;

        readPacket(&header, pData, PACKET_MAX_DATA);
        switch (header.type)
        {
        case PACKET_TYPE_ERASE_ALL:
            iapResult = iapEraseAll();
            if (iapResult == IAP_CMD_SUCCESS)
            {
                g_allSectorsErased = 1;
                sendAckPacket();
            }
            else
            {
                sendNakPacket(iapResult);
            }
            break;
        case PACKET_TYPE_WRITE:
            iapResult = iapWriteSectors(header.address, pData, header.length);
            if (iapResult == IAP_CMD_SUCCESS)
                sendAckPacket();
            else
                sendNakPacket(iapResult);
            break;
        case PACKET_TYPE_RESET:
            sendAckPacket();
            waitForUartToFlush();
            NVIC_SystemReset();
            break;
        default:
            sendNakPacket(0xFFFFFFFF);
        }
    }

    return 0;
}

static void zeroFillBssSection(void)
{
    int bssSize = (int)__bss_end__ - (int)__bss_start__;
    memset(__bss_start__, 0, bssSize);
}

static void findMriUart(void)
{
    g_pCurrentUart = g_uartConfigurations[0];
    uint32_t highestPriority = NVIC_GetPriority(UART0_IRQn);

    /* Find UART being used by MRI which is the UART interrupt at the highest priority. */
    for (int i = 1 ; i < 4 ; i++)
    {
        uint32_t priority = NVIC_GetPriority(UART0_IRQn + i);
        if (priority < highestPriority && isInterruptEnabled(UART0_IRQn + i))
        {
            g_pCurrentUart = g_uartConfigurations[i];
            highestPriority = priority;
            return;
        }
    }
}

static uint32_t isInterruptEnabled(IRQn_Type IRQn)
{
    return NVIC->ISER[((uint32_t)(IRQn) >> 5)] & (1 << ((uint32_t)(IRQn) & 0x1F));
}

static int uartReceiveChar(void)
{
    waitForUartToReceiveData();

    return (int)g_pCurrentUart->RBR;
}

static void waitForUartToReceiveData(void)
{
    while (!uartHasReceiveData())
    {
    }
}

static uint32_t uartHasReceiveData(void)
{
    static const uint8_t receiverDataReadyBit = 1 << 0;

    return g_pCurrentUart->LSR & receiverDataReadyBit;
}

static void uartSendChar(int Character)
{
    waitForUartToAllowTransmit();

    g_pCurrentUart->THR = (uint8_t)Character;
}

static void waitForUartToAllowTransmit(void)
{
    while (!targetUartCanTransmit())
    {
    }
}

static uint32_t targetUartCanTransmit(void)
{
    static const uint8_t transmitterHoldRegisterEmptyBit = 1 << 5;

    return g_pCurrentUart->LSR & transmitterHoldRegisterEmptyBit;
}

static void sendInitCompletedPacket(void)
{
    PacketHeader header;

    memset(&header, 0, sizeof(header));
    header.type = PACKET_TYPE_INIT_COMPLETED;
    sendPacketHeader(&header);
}

static void sendPacketHeader(PacketHeader* pHeader)
{
    sendBytes(g_startOfPacket, sizeof(g_startOfPacket));

    pHeader->crc = calculateCRC(pHeader, NULL);
    sendBytes(pHeader, sizeof(*pHeader));
}

static void sendBytes(const void* pvBuffer, size_t bufferSize)
{
    const uint8_t* pBuffer = (uint8_t*)pvBuffer;

    while (bufferSize--)
        uartSendChar(*pBuffer++);
}

static uint32_t calculateCRC(const PacketHeader* pHeader, const void* pvData)
{
    crcStart();
    crcData(&pHeader->length, sizeof(*pHeader) - offsetof(PacketHeader, length));
    crcData(pvData, pHeader->length);
    return crcEnd();
}

static void readPacket(PacketHeader* pHeader, uint8_t* pData, size_t dataSize)
{
    readPacketHeader(pHeader);
    if (pHeader->length > dataSize)
    {
        pHeader->type = PACKET_TYPE_BAD;
        return;
    }

    readBytesFromComm(pData, pHeader->length);
    if (pHeader->crc != calculateCRC(pHeader, pData))
    {
        pHeader->type = PACKET_TYPE_BAD;
        return;
    }
}

static void readPacketHeader(PacketHeader* pHeader)
{
    waitForStartOfPacket();
    readBytesFromComm(pHeader, sizeof(*pHeader));
}

static void waitForStartOfPacket(void)
{
    uint8_t buffer[4] = { 0x00, 0x00, 0x00, 0x00 };

    while (1)
    {
        /* Slide newest byte into the end of the buffer, shifting the rest over to make room. */
        memmove(&buffer[0], &buffer[1], 3);
        buffer[3] = uartReceiveChar();
        if (0 == memcmp(buffer, g_startOfPacket, sizeof(buffer)))
            return;
    }
}

static void readBytesFromComm(void* pvBuffer, size_t bufferSize)
{
    uint8_t* pBuffer = (uint8_t*)pvBuffer;

    while (bufferSize--)
        *pBuffer++ = uartReceiveChar();
}

static void sendAckPacket(void)
{
    PacketHeader header;

    memset(&header, 0, sizeof(header));
    header.type = PACKET_TYPE_ACK;
    sendPacketHeader(&header);
}

static void sendNakPacket(uint32_t errorCode)
{
    PacketHeader header;

    memset(&header, 0, sizeof(header));
    header.type = PACKET_TYPE_NAK;
    header.errorCode = errorCode;
    sendPacketHeader(&header);
}

static void waitForUartToFlush(void)
{
    while (!targetUartHasFlushed())
    {
    }
}

static uint32_t targetUartHasFlushed(void)
{
    static const uint8_t transmitterEmptyBit = 1 << 6;

    return g_pCurrentUart->LSR & transmitterEmptyBit;
}

static int iapEraseAll(void)
{
    int result = -1;

    result = iapPrepareSectors(0, LPC1768_SECTOR_COUNT - 1);
    if (result != IAP_CMD_SUCCESS)
        return result;
    return iapEraseSectors(0, LPC1768_SECTOR_COUNT - 1);
}

static int iapPrepareSectors(uint32_t startSector, uint32_t endSector)
{
    uint32_t commandParameters[] = { IAP_PREPARE_SECTORS, startSector, endSector};
    uint32_t results[1];

    iapEntry(commandParameters, results);
    return results[0];
}

static int iapEraseSectors(uint32_t startSector, uint32_t endSector)
{
    uint32_t commandParameters[] = { IAP_ERASE_SECTORS, startSector, endSector, LPC1768_CPU_SPEED_KHZ };
    uint32_t results[1];

    iapEntry(commandParameters, results);
    return results[0];
}

static int iapWriteSectors(uint32_t flashAddress, uint8_t* pData, uint32_t dataLength)
{
    int iapResult = -1;

    while (dataLength > 0)
    {
        uint32_t actualBytes = 4096;
        uint32_t sector = iapSectorFromAddress(flashAddress);

        if (dataLength < 4096)
        {
            /* Pad out extra part of write with 0xFF.  Since my data buffers are 16k, they are always divisible by
               4096 and can be padded safely without worrying about overflow. */
            memset(pData + dataLength, 0xFF, 4096 - dataLength);
            actualBytes = dataLength;
        }

        if (!g_allSectorsErased && g_lastSectorErased != sector)
        {
            /* If not all sectors were erased then erase each one the first time we write bytes to it. */
            iapResult = iapPrepareSectors(sector, sector);
            if (iapResult != IAP_CMD_SUCCESS)
                return iapResult;

            iapResult = iapEraseSectors(sector, sector);
            if (iapResult != IAP_CMD_SUCCESS)
                return iapResult;

            g_lastSectorErased = sector;
        }

        iapResult = iapPrepareSectors(sector, sector);
        if (iapResult != IAP_CMD_SUCCESS)
            return iapResult;
        iapResult = iapCopyRamToFlash(flashAddress, pData, 4096);
        if (iapResult != IAP_CMD_SUCCESS)
            return iapResult;

        flashAddress += 4096;
        pData += 4096;
        dataLength -= actualBytes;
    }

    return IAP_CMD_SUCCESS;
}

static uint32_t iapSectorFromAddress(uint32_t flashAddress)
{
    if (flashAddress < 0x10000)
        return flashAddress / 4096;
    else
        return 16 + ((flashAddress - 0x10000) / 32768);
}

static int iapCopyRamToFlash(uint32_t flashAddress, uint8_t* pRam, uint32_t length)
{
    uint32_t commandParameters[] = { IAP_COPY_RAM_TO_FLASH, flashAddress, (uint32_t)pRam, length, LPC1768_CPU_SPEED_KHZ };
    uint32_t results[1];

    iapEntry(commandParameters, results);
    return results[0];
}
