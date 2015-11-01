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
#ifndef _GDB_REMOTE_H_
#define _GDB_REMOTE_H_

#include <stdint.h>
#include "IComm.h"
#include "try_catch.h"

/* Register names / indices into ArmContext::R array for first 13 registers. */
#define R0  0
#define R1  1
#define R2  2
#define R3  3
#define R4  4
#define R5  5
#define R6  6
#define R7  7
#define R8  8
#define R9  9
#define R10 10
#define R11 11
#define R12 12
#define SP  13
#define LR  14
#define PC  15

typedef struct ArmContext
{
    uint32_t R[13];
    uint32_t spMain;
    uint32_t lr;
    uint32_t pc;
    uint32_t xPSR;
} ArmContext;

         void gdbRemoteInit(IComm* pComm);
         void gdbRemoteUninit(void);
         void gdbRemoteGetRegisters(ArmContext* pContext);
__throws void gdbRemoteSetRegisters(const ArmContext* pContext);
         void gdbRemoteReadMemory(uint32_t address, void* pReadBuffer, uint32_t readSize);
__throws void gdbRemoteWriteMemory(uint32_t address, const void* pWriteBuffer, uint32_t writeSize);
__throws int  gdbRemoteSingleStep(void);
__throws void gdbRemoteContinue(void);


#endif /* _GDB_REMOTE_H_ */
