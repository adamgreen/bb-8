/*  Copyright (C) 2014  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/

#ifndef __INTERLOCK_H__
#define __INTERLOCK_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t interlockedIncrement(volatile uint32_t* pValue);
uint32_t interlockedDecrement(volatile uint32_t* pValue);
uint32_t interlockedAdd(volatile int32_t* pVal1, int32_t val2);
uint32_t interlockedSubtract(volatile int32_t* pVal1, int32_t val2);

#ifdef __cplusplus
}
#endif

#endif // __INTERLOCK_H__
