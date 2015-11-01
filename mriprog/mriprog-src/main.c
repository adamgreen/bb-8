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
#include <mach/mach_time.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <protocol.h>
#include "common.h"
#include "gdbRemote.h"
#include "packet.h"
#include "SerialIComm.h"
#include "SocketIComm.h"
#include "boot-lpc1768.h"


static void displayUsage(void)
{
    fprintf(stderr, "Usage: mriprog deviceName flashImage.bin\n"
                    "Where: deviceName is serial or TCP/IP connection to use for\n"
                    "                  communicating with MRI debug monitor.\n"
                    "                  Examples: /dev/tty.usbmodem1412\n"
                    "                            192.168.0.129:23\n"
                    "       flashImage.bin is the pathname of the new image to\n"
                    "                      load into FLASH.\n");
}

/* Locations in RAM where LPC1768 serial bootloader should be loaded. */
#define LPC1768_RAM_START 0x2007c000
#define LPC1768_RAM_SIZE  (32 * 1024)
#define LPC1768_RAM_END   (LPC1768_RAM_START + LPC1768_RAM_SIZE)

/* Locations in FLASH where LPC1768 code should be loaded. */
#define LPC1768_FLASH_START 0x0000000


typedef struct BinaryFile
{
    uint8_t* pBinary;
    uint32_t binarySize;
} BinaryFile;

static const uint8_t g_startOfPacket[4] = PACKET_START_MARKER;


static BinaryFile openBinaryFile(const char* binaryFilename);
static void closeBinaryFile(BinaryFile* pFile);
static void checksumVectors(BinaryFile* pFile);
static void waitForInitCompletedPacket(IComm* pComm);
static void readPacketHeader(IComm* pComm, PacketHeader* pHeader);
static void waitForStartOfPacket(IComm* pComm);
static void sendEraseAllPacket(IComm* pComm);
static void sendPacket(IComm* pComm, PacketHeader* pHeader, const void* pvData);
static uint32_t calculateCRC(const PacketHeader* pHeader, const void* pvData);
static void sendWritePacket(IComm* pComm, uint32_t flashAddress, const uint8_t* pData, uint32_t dataSize);
static void sendResetPacket(IComm* pComm);
static uint32_t getTimeInMicroseconds(void);


int main(int argc, const char** argv)
{
    int                 returnValue = 0;
    IComm*              pComm = NULL;
    const char*         pDeviceName = NULL;
    const char*         pFlashImagePath = NULL;
    uint32_t            flashBytesLeft = 0;
    uint32_t            flashAddress = 0;
    uint8_t*            pFlash = NULL;
    uint32_t            startFlashTime = 0;
    uint32_t            endFlashTime = 0;
    ArmContext          context;
    BinaryFile          bootloader = { g_lpc1768Bootloader, sizeof(g_lpc1768Bootloader) };
    BinaryFile          image;

    /* Parse command line parameters. */
    if (argc != 3)
    {
        displayUsage();
        return -1;
    }
    pDeviceName = argv[1];
    pFlashImagePath = argv[2];

    __try
    {
        crcTableInit();

        printf("Opening connection to %s...\n", pDeviceName);
        if (pDeviceName == strstr(pDeviceName, "/dev/tty"))
            pComm = SerialIComm_Init(pDeviceName, 230400, 1000);
        else
            pComm = SocketIComm_Init(pDeviceName, 10000);

        printf("Loading new flash image %s...\n", pFlashImagePath);
        image = openBinaryFile(pFlashImagePath);

        printf("Checksumming flash vector table...\n");
        checksumVectors(&image);

        /* Sending '+' will ACK any outstanding packet from MRI or force MRI to stop execution if still running. */
        IComm_SendChar(pComm, '+');
        gdbRemoteInit(pComm);

        printf("Uploading bootloader to RAM of device (%u bytes)...\n", bootloader.binarySize);
        gdbRemoteWriteMemory(LPC1768_RAM_START, bootloader.pBinary, bootloader.binarySize);
        gdbRemoteGetRegisters(&context);
        context.spMain = LPC1768_RAM_END;
        context.lr = LPC1768_RAM_START | 1;
        context.pc = LPC1768_RAM_START;
        gdbRemoteSetRegisters(&context);
        gdbRemoteContinue();
        waitForInitCompletedPacket(pComm);

        printf("Erasing all of FLASH...\n");
        sendEraseAllPacket(pComm);

        printf("Writing %u bytes to FLASH...\n", image.binarySize);
        startFlashTime = getTimeInMicroseconds();
        flashBytesLeft = image.binarySize;
        flashAddress = LPC1768_FLASH_START;
        pFlash = image.pBinary;
        while (flashBytesLeft > 0)
        {
            uint32_t bytesToSend = flashBytesLeft > PACKET_MAX_DATA ? PACKET_MAX_DATA : flashBytesLeft;
            sendWritePacket(pComm, flashAddress, pFlash, bytesToSend);
            flashBytesLeft -= bytesToSend;
            flashAddress += bytesToSend;
            pFlash += bytesToSend;
        }
        endFlashTime = getTimeInMicroseconds();
        printf("    Wrote %u bytes in %u microseconds\n", image.binarySize, endFlashTime - startFlashTime);
        printf("    %.3f kB / second\n", (image.binarySize / 1024.0f) / ((endFlashTime - startFlashTime) / 1000000.0f));

        printf("Resetting device...\n");
        sendResetPacket(pComm);
    }
    __catch
    {
        switch (getExceptionCode())
        {
        case timeoutException:
            fprintf(stderr, "error: Timed out waiting for response from device.\n");
            break;
        default:
            fprintf(stderr, "error: Encountered unexpected exception %d\n", getExceptionCode());
            break;
        }

        returnValue = -1;
    }

    closeBinaryFile(&image);
    gdbRemoteUninit();
    SerialIComm_Uninit(pComm);

    return returnValue;
}

static BinaryFile openBinaryFile(const char* binaryFilename)
{
    FILE*      pFile = NULL;
    BinaryFile file;

    pFile = fopen(binaryFilename, "r");
    if (!pFile)
    {
        fprintf(stderr, "error: Failed to open %s: ", binaryFilename);
        perror(NULL);
        __throw(fileException);
    }
    memset(&file, 0, sizeof(file));

    __try
    {
        size_t bytesRead = 0;

        file.binarySize = GetFileSize(pFile);
        file.pBinary = malloc(file.binarySize);
        if (!file.pBinary)
            __throw(outOfMemoryException);
        bytesRead = fread(file.pBinary, 1, file.binarySize, pFile);
        if (bytesRead != file.binarySize)
        {
            fprintf(stderr, "error: Failed to read %u bytes from %s: ", file.binarySize, binaryFilename);
            perror(NULL);
        }
    }
    __catch
    {
        free(file.pBinary);
        fclose(pFile);
        __rethrow;
    }

    fclose(pFile);
    return file;
}

static void closeBinaryFile(BinaryFile* pFile)
{
    if (!pFile)
        return;
    free(pFile->pBinary);
    memset(pFile, 0, sizeof(*pFile));
}

static void checksumVectors(BinaryFile* pFile)
{
    uint32_t* pVectors = (uint32_t*)pFile->pBinary;
    uint32_t  checksum = 0;
    int       i;

    if (pFile->binarySize < 8 * sizeof(uint32_t))
        return;
    for (i = 0 ; i < 7 ; i++)
        checksum += pVectors[i];
    pVectors[7] = -checksum;

}

static void waitForInitCompletedPacket(IComm* pComm)
{
    PacketHeader header;

    readPacketHeader(pComm, &header);
    if (header.type != PACKET_TYPE_INIT_COMPLETED)
    {
        fprintf(stderr, "error: Failed to receive bootloader init completed packet.\n");
        __throw(badResponseException);
    }
}

static void readPacketHeader(IComm* pComm, PacketHeader* pHeader)
{
    waitForStartOfPacket(pComm);
    IComm_ReceiveBytes(pComm, pHeader, sizeof(*pHeader));

    if (pHeader->type == PACKET_TYPE_NAK)
    {
        fprintf(stderr, "error: Device returned NAK with error %u\n", pHeader->errorCode);
        return;
    }

    /* Verify packet header is as expected and if not, flag as bad packet type. */
    if ((pHeader->type != PACKET_TYPE_ACK && pHeader->type != PACKET_TYPE_INIT_COMPLETED) ||
        pHeader->length != 0 ||
        pHeader->crc != calculateCRC(pHeader, NULL))
    {
        pHeader->type = PACKET_TYPE_BAD;
        return;
    }

    return;
}

static void waitForStartOfPacket(IComm* pComm)
{
    uint8_t buffer[4] = { 0x00, 0x00, 0x00, 0x00 };

    while (1)
    {
        /* Slide newest byte into the end of the buffer, shifting the rest over to make room. */
        memmove(&buffer[0], &buffer[1], 3);
        buffer[3] = IComm_ReceiveChar(pComm);
        if (0 == memcmp(buffer, g_startOfPacket, sizeof(buffer)))
            return;
    }

}

static void sendEraseAllPacket(IComm* pComm)
{
    PacketHeader header;

    memset(&header, 0, sizeof(header));
    header.type = PACKET_TYPE_ERASE_ALL;
    sendPacket(pComm, &header, NULL);
}

static void sendPacket(IComm* pComm, PacketHeader* pHeader, const void* pvData)
{
    int                  attempt = 0;
    PacketHeader         responseHeader;

    pHeader->crc = calculateCRC(pHeader, pvData);
    do
    {
        if (attempt > 0)
            fprintf(stderr, "error: Retrying packet send.\n");

        IComm_SendBytes(pComm, g_startOfPacket, sizeof(g_startOfPacket));
        IComm_SendBytes(pComm, pHeader, sizeof(*pHeader));
        IComm_SendBytes(pComm, pvData, pHeader->length);
        readPacketHeader(pComm, &responseHeader);
        attempt++;
    } while (responseHeader.type != PACKET_TYPE_ACK);
}

static uint32_t calculateCRC(const PacketHeader* pHeader, const void* pvData)
{
    crcStart();
    crcData(&pHeader->length, sizeof(*pHeader) - offsetof(PacketHeader, length));
    crcData(pvData, pHeader->length);
    return crcEnd();
}

static void sendWritePacket(IComm* pComm, uint32_t flashAddress, const uint8_t* pData, uint32_t dataSize)
{
    PacketHeader header;

    memset(&header, 0, sizeof(header));
    header.type = PACKET_TYPE_WRITE;
    header.length = dataSize;
    header.address = flashAddress;
    sendPacket(pComm, &header, pData);
}

static void sendResetPacket(IComm* pComm)
{
    PacketHeader header;

    memset(&header, 0, sizeof(header));
    header.type = PACKET_TYPE_RESET;
    sendPacket(pComm, &header, NULL);
}

static uint32_t getTimeInMicroseconds(void)
{
    mach_timebase_info_data_t machTimebaseInfo;

    mach_timebase_info(&machTimebaseInfo);
    return (uint32_t)((mach_absolute_time() * machTimebaseInfo.numer) / (1000 * machTimebaseInfo.denom));
}
