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
#ifndef _SERIAL_ICOMM_H_
#define _SERIAL_ICOMM_H_

#include "IComm.h"
#include "try_catch.h"


__throws IComm* SerialIComm_Init(const char* pDevicePath, uint32_t baudRate, uint32_t msecTimeout);
         void   SerialIComm_Uninit(IComm* pComm);


#endif /* _SERIAL_ICOMM_H_ */
