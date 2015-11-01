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
#ifndef _ICOMM_H_
#define _ICOMM_H_

#include <stdint.h>


typedef struct IComm IComm;

typedef struct ICommVTable
{
    int  (*hasReceiveData)(IComm* pComm);
    int  (*receiveChar)(IComm* pComm);
    void (*receiveBytes)(IComm* pComm, void* pBuffer, size_t bufferSize);
    void (*sendChar)(IComm* pComm, int character);
    void (*sendBytes)(IComm* pComm, const void* pBuffer, size_t bufferSize);
} ICommVTable;

struct IComm
{
    ICommVTable* pVTable;
};


static inline int IComm_HasReceiveData(IComm* pThis)
{
    return pThis->pVTable->hasReceiveData(pThis);
}

static inline int IComm_ReceiveChar(IComm* pThis)
{
    return pThis->pVTable->receiveChar(pThis);
}

static inline void IComm_ReceiveBytes(IComm* pThis, void* pvBuffer, size_t bufferSize)
{
    pThis->pVTable->receiveBytes(pThis, pvBuffer, bufferSize);
}

static inline void IComm_SendChar(IComm* pThis, int character)
{
    pThis->pVTable->sendChar(pThis, character);
}

static inline void IComm_SendBytes(IComm* pThis, const void* pvBuffer, size_t bufferSize)
{
    pThis->pVTable->sendBytes(pThis, pvBuffer, bufferSize);
}


#endif /* _ICOMM_H_ */
