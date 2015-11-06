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
/* Class to implement a simple RAM logger. */
#ifndef RAM_LOGGER_H_
#define RAM_LOGGER_H_

#include <assert.h>

class CircularRamLogger
{
public:
    CircularRamLogger(void* pRam, size_t ramSize, size_t entrySize)
    {
        m_pRamStart = (uint8_t*)pRam;
        m_pRamEnd = m_pRamStart + ramSize;
        m_pFind = NULL;
        m_ramSize = ramSize;
        m_entrySize = entrySize;
        start();
    }

    void start()
    {
        memset(m_pRamStart, 0, m_ramSize);
        m_pCurr = m_pRamStart;
        m_pLast = NULL;
        m_pFind = NULL;
    }

    void logEntry(const void* pvEntry, size_t entrySize)
    {
        size_t         bytesLeft = m_pRamEnd - m_pCurr;
        const uint8_t* pEntry = (const uint8_t*)pvEntry;

        assert (entrySize == m_entrySize);

        // Remember where we started to store this latest item.
        m_pLast = m_pCurr;

        // Now store the data in the RAM buffer.
        if (bytesLeft < entrySize)
        {
            // Need to split the write between end and start of buffer.
            memcpy(m_pCurr, pEntry, bytesLeft);
            pEntry += bytesLeft;
            entrySize -= bytesLeft;
            m_pCurr = m_pRamStart;
        }
        memcpy(m_pCurr, pEntry, entrySize);
        m_pCurr += entrySize;
    }

    bool findOldestEntry(void* pEntry, size_t entrySize)
    {
        assert (entrySize == m_entrySize);

        if (!m_pLast)
            return false;

        m_pFind = m_pLast;
        for (;;)
        {
            size_t   bytesLeft = m_pFind - m_pRamStart;
            uint8_t* pNext = NULL;

            if (bytesLeft < entrySize)
            {
                // Entry is split across RAM buffer.
                if (foundNewestInReverseWalk(m_pFind, m_pRamStart))
                {
                    // Found newest entry at beginning of RAM buffer so return last one.
                    copyFoundEntry(pEntry, entrySize);
                    return true;
                }
                pNext = m_pRamEnd - (entrySize - bytesLeft);
                if (foundNewestInReverseWalk(m_pRamEnd, pNext))
                {
                    // Found newest entry at end of RAM buffer so return last one.
                    copyFoundEntry(pEntry, entrySize);
                    return true;
                }
            }
            else
            {
                pNext = m_pFind - entrySize;
                if (foundNewestInReverseWalk(m_pFind, pNext))
                {
                    // Found newest entry in middle of RAM buffer.
                    copyFoundEntry(pEntry, entrySize);
                    return true;
                }
            }
            m_pFind = pNext;
        }
    }

    bool findNextEntry(void* pEntry, size_t entrySize)
    {
        assert (entrySize == m_entrySize);

        if (!m_pFind)
            return false;

        size_t   bytesLeft = m_pRamEnd - m_pFind;
        uint8_t* pPrev = m_pFind;
        if (bytesLeft < entrySize)
        {
            // Entry is split across RAM buffer.
            if (foundNewestInForwardWalk(pPrev, m_pRamEnd))
            {
                // Found newest entry at end of RAM so we are at end of list.
                return false;
            }
            entrySize -= bytesLeft;
            pPrev = m_pRamStart;
        }

        uint8_t* pNext = pPrev + entrySize;
        if (foundNewestInForwardWalk(pPrev, pNext))
        {
            // Found newest entry so we are at end of list.
            return false;
        }
        m_pFind = pNext;
        copyFoundEntry(pEntry, entrySize);
        return true;
    }

protected:
    bool foundNewestInReverseWalk(const uint8_t* pPrev, const uint8_t* pNext)
    {
        return (pPrev >= m_pCurr && pNext < m_pCurr);
    }

    bool foundNewestInForwardWalk(const uint8_t* pPrev, const uint8_t* pNext)
    {
        return (pPrev < m_pCurr && pNext >= m_pCurr);
    }

    void copyFoundEntry(void* pvEntry, size_t entrySize)
    {
        size_t   bytesLeft = m_pRamEnd - m_pFind;
        uint8_t* pEntry = (uint8_t*)pvEntry;
        uint8_t* pFind = m_pFind;

        assert (entrySize == m_entrySize);

        if (bytesLeft < entrySize)
        {
            // Need to split the read between end and start of buffer.
            memcpy(pEntry, pFind, bytesLeft);
            pEntry += bytesLeft;
            entrySize -= bytesLeft;
            pFind = m_pRamStart;
        }
        memcpy(pEntry, pFind, entrySize);
    }

    uint8_t*  m_pRamStart;
    uint8_t*  m_pRamEnd;
    uint8_t*  m_pCurr;
    uint8_t*  m_pLast;
    uint8_t*  m_pFind;
    size_t    m_ramSize;
    size_t    m_entrySize;
};

#endif // RAM_LOGGER_H_
