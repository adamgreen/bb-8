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
#ifndef _PACKET_H_
#define _PACKET_H_

#include <stdio.h>
#include "buffer.h"
#include "IComm.h"

typedef struct
{
    Buffer*        pBuffer;
    char*          pCacheCurr;
    char           lastChar;
    unsigned char  calculatedChecksum;
    unsigned char  expectedChecksum;
} Packet;

void packetInit(IComm* pComm);
void packetUninit(void);
void packetGet(Packet* pPacket, Buffer* pBuffer);
void packetSend(Packet* pPacket, Buffer* pBuffer);


#endif /* _PACKET_H_ */
