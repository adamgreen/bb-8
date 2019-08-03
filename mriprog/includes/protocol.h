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
/* Share protocol definitions used between mriprog and serial bootloader. */
#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_

#include <stdint.h>

/* Start of a new packet will start with these 4 bytes. */
#define PACKET_START_MARKER "aDaM"

/* Maximum number of bytes in packet data. */
#define PACKET_MAX_DATA (16 * 1024)

/* Packet types that can be found in PacketHeader::type */
#define PACKET_TYPE_INIT_COMPLETED 0xFFFF0000
#define PACKET_TYPE_ACK            0xFFFE0001
#define PACKET_TYPE_NAK            0xFFFD0002
#define PACKET_TYPE_ERASE_ALL      0xFFFC0003
#define PACKET_TYPE_WRITE          0xFFFB0004
#define PACKET_TYPE_RESET          0xFFFA0005
#define PACKET_TYPE_BAD            0x0000FFFF

typedef struct PacketHeader
{
    uint32_t type;
    uint32_t crc;
    uint32_t length;
    union {
        uint32_t address;
        uint32_t errorCode;
    };
} PacketHeader;



/* This CRC-32 code is based on the crc32c() implementation found at
     http://www.hackersdelight.org/hdcodetxt/crc.c.txt
*/
static uint32_t g_crc;
static uint32_t g_crcTable[256];

static inline void crcTableInit(void)
{
    int i;
    for (i = 0; i <= 255; i++)
    {
        uint32_t crc = i;
        int j;
        for (j = 7; j >= 0; j--)
        {
            uint32_t mask = -(crc & 1);
            crc = (crc >> 1) ^ (0xEDB88320 & mask);
        }
        g_crcTable[i] = crc;
    }
}

static inline void crcStart(void)
{
    g_crc = 0xFFFFFFFF;
}

static inline void crcData(const void* pvData, size_t dataLength)
{
    const uint8_t* pData = (const uint8_t*)pvData;

    while (dataLength--)
    {
        g_crc = (g_crc >> 8) ^ g_crcTable[(g_crc ^ *pData++) & 0xFF];
    }
}

static inline uint32_t crcEnd(void)
{
    return ~g_crc;
}

#endif /* _PROTOCOL_H_ */
