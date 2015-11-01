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
#ifndef _SOCKET_ICOMM_H_
#define _SOCKET_ICOMM_H_

#include "IComm.h"
#include "try_catch.h"


__throws IComm* SocketIComm_Init(const char* remoteAddress, uint32_t msecTimeout);
         void   SocketIComm_Uninit(IComm* pComm);


#endif /* _SOCKET_ICOMM_H_ */
