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
#ifndef _ELF_LOAD_H_
#define _ELF_LOAD_H_

#include <stdint.h>
#include "try_catch.h"

#define ELF_REGION_FLAGS_ALLOCATED (1 << 0)

typedef struct ElfRegion
{
    void*       pData;
    uint32_t    address;
    uint32_t    size;
    uint32_t    flags;
} ElfRegion;

typedef struct ElfRegions
{
    ElfRegion* pRegions;
    size_t     count;
} ElfRegions;

__throws ElfRegions* ElfLoad_FromMemory(void* pElf, size_t elfSize);
         void        ElfLoad_FreeRegions(ElfRegions* pRegions);

#endif /* _ELF_LOAD_H_ */
