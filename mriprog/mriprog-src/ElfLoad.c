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
#include <string.h>
#include "ElfLoad.h"
#include "ElfPriv.h"

typedef struct
{
    char*   pBlob;
    size_t  blobSize;
} SizedBlob;

typedef struct
{
    const Elf32_Ehdr* pElfHeader;
    SizedBlob         sizedBlob;
    Elf32_Off         pgmHeaderOffset;
} LoadObject;

static ElfRegions* allocateElfRegionsStruct(void);
static LoadObject initLoadObject(char* pBlob, size_t blobSize);
static SizedBlob initSizedBlob(char* pBlob, size_t blobSize);
static void* fetchSizedByteArray(const SizedBlob* pBlob, uint32_t offset, uint32_t size);
static void validateElfHeaderContents(const Elf32_Ehdr* pHeader);
static void loadLoadableEntries(ElfRegions* pRegions, LoadObject* pObject);
static void loadIfLoadableEntry(ElfRegions* pRegions, LoadObject* pObject, const Elf32_Phdr* pPgmHeader);
static int isLoadableEntry(const Elf32_Phdr* pHeader);
static void addElfRegion(ElfRegions* pRegions, uint32_t address, void* pData, uint32_t size);
static int coalesceIfPossible(ElfRegions* pRegions, uint32_t address, void* pData, uint32_t size);
static void growRegionArrayToFitNewEntry(ElfRegions* pRegions);


__throws ElfRegions* ElfLoad_FromMemory(void* pElf, size_t elfSize)
{
    ElfRegions* volatile pRegions = NULL;

    __try
    {
        pRegions = allocateElfRegionsStruct();
        LoadObject object = initLoadObject(pElf, elfSize);
        validateElfHeaderContents(object.pElfHeader);
        loadLoadableEntries(pRegions, &object);
        if (pRegions->count == 0)
            __throw(elfFormatException);
    }
    __catch
    {
        ElfLoad_FreeRegions(pRegions);
        __rethrow;
    }

    return pRegions;
}

static ElfRegions* allocateElfRegionsStruct(void)
{
    ElfRegions* pRegions = malloc(sizeof(*pRegions));
    if (!pRegions)
        __throw(outOfMemoryException);
    memset(pRegions, 0, sizeof(*pRegions));
    return pRegions;
}

static LoadObject initLoadObject(char* pBlob, size_t blobSize)
{
    LoadObject object;

    memset(&object, 0, sizeof(object));
    object.sizedBlob = initSizedBlob(pBlob, blobSize);
    object.pElfHeader = fetchSizedByteArray(&object.sizedBlob, 0, sizeof(*object.pElfHeader));
    return object;
}

static SizedBlob initSizedBlob(char* pBlob, size_t blobSize)
{
    SizedBlob blob = { pBlob, blobSize };
    return blob;
}

static void* fetchSizedByteArray(const SizedBlob* pBlob, uint32_t offset, uint32_t size)
{
    if (offset > pBlob->blobSize || (offset + size) > pBlob->blobSize)
        __throw(elfFormatException);
    return (void*)(pBlob->pBlob + offset);
}

static void validateElfHeaderContents(const Elf32_Ehdr* pHeader)
{
    const unsigned char expectedIdent[4] = { ELFMAG0, ELFMAG1, ELFMAG2, ELFMAG3 };
    if (memcmp(pHeader->e_ident, expectedIdent, sizeof(expectedIdent)) != 0 ||
        pHeader->e_ident[EI_CLASS] != ELFCLASS32 ||
        pHeader->e_ident[EI_DATA] != ELFDATA2LSB ||
        pHeader->e_type != ET_EXEC ||
        pHeader->e_phoff == 0 ||
        pHeader->e_phnum == 0 ||
        pHeader->e_phentsize < sizeof(Elf32_Phdr))
    {
        __throw(elfFormatException);
    }
}

static void loadLoadableEntries(ElfRegions* pRegions, LoadObject* pObject)
{
    const Elf32_Phdr*   pPgmHeader = NULL;
    Elf32_Half          i = 0;

    for (i = 0, pObject->pgmHeaderOffset = pObject->pElfHeader->e_phoff ; i < pObject->pElfHeader->e_phnum ; i++)
    {
        pPgmHeader = fetchSizedByteArray(&pObject->sizedBlob, pObject->pgmHeaderOffset, sizeof(*pPgmHeader));
        loadIfLoadableEntry(pRegions, pObject, pPgmHeader);
        pObject->pgmHeaderOffset += pObject->pElfHeader->e_phentsize;
    }
}

static void loadIfLoadableEntry(ElfRegions* pRegions, LoadObject* pObject, const Elf32_Phdr* pPgmHeader)
{
    void* pData = NULL;

    if (!isLoadableEntry(pPgmHeader))
        return;

    pData = fetchSizedByteArray(&pObject->sizedBlob, pPgmHeader->p_offset, pPgmHeader->p_filesz);
    addElfRegion(pRegions, pPgmHeader->p_paddr, pData, pPgmHeader->p_filesz);
}

static int isLoadableEntry(const Elf32_Phdr* pHeader)
{
    return (pHeader->p_type == PT_LOAD && pHeader->p_filesz != 0);
}

static void addElfRegion(ElfRegions* pRegions, uint32_t address, void* pData, uint32_t size)
{
    ElfRegion* pNewRegion = NULL;

    if (coalesceIfPossible(pRegions, address, pData, size))
        return;

    growRegionArrayToFitNewEntry(pRegions);
    pNewRegion = &pRegions->pRegions[pRegions->count - 1];
    pNewRegion->pData = pData;
    pNewRegion->address = address;
    pNewRegion->size = size;
}

static int coalesceIfPossible(ElfRegions* pRegions, uint32_t address, void* pData, uint32_t size)
{
    ElfRegion* pPrevRegion;

    if (pRegions->count < 1)
        return 0;
    pPrevRegion = &pRegions->pRegions[pRegions->count - 1];
    if (pPrevRegion->address + pPrevRegion->size == address)
    {
        uint32_t newSize = pPrevRegion->size + size;
        uint8_t* pAlloc = NULL;

        if (pPrevRegion->flags & ELF_REGION_FLAGS_ALLOCATED)
        {
            pAlloc = realloc(pPrevRegion->pData, newSize);
            if (!pAlloc)
                __throw(outOfMemoryException);
        }
        else
        {
            pAlloc = malloc(newSize);
            if (!pAlloc)
                __throw(outOfMemoryException);
            memcpy(pAlloc, pPrevRegion->pData, pPrevRegion->size);
        }

        memcpy(pAlloc + pPrevRegion->size, pData, size);
        pPrevRegion->pData = pAlloc;
        pPrevRegion->size = newSize;
        pPrevRegion->flags |= ELF_REGION_FLAGS_ALLOCATED;

        return 1;
    }

    return 0;
}

static void growRegionArrayToFitNewEntry(ElfRegions* pRegions)
{
    size_t newCount = pRegions->count + 1;
    size_t newSize = sizeof(ElfRegion) * newCount;
    ElfRegion* pRealloc = realloc(pRegions->pRegions, newSize);
    if (!pRealloc)
        __throw(outOfMemoryException);

    memset(&pRealloc[newCount-1], 0, sizeof(*pRealloc));
    pRegions->pRegions = pRealloc;
    pRegions->count = newCount;
}



void ElfLoad_FreeRegions(ElfRegions* pRegions)
{
    size_t i;

    if (!pRegions)
        return;

    for (i = 0 ; i < pRegions->count ; i++)
    {
        ElfRegion* pRegion = &pRegions->pRegions[i];
        if (pRegion->flags & ELF_REGION_FLAGS_ALLOCATED)
            free(pRegion->pData);
    }
    free(pRegions->pRegions);
    free(pRegions);
}
