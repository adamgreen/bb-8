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
#include <assert.h>
#include <ctype.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "common.h"
#include "SocketIComm.h"


/* Set SOCKET_LOG to 1 to have all serial I/O logged to socket.log file and set to 0 otherwise. */
#define SOCKET_LOG 1


/* Implementation of IComm interface. */
typedef struct SocketIComm SocketIComm;

static int  hasReceiveData(IComm* pComm);
static int  receiveChar(IComm* pComm);
static void receiveBytes(IComm* pComm, void* pBuffer, size_t bufferSize);
static void sendChar(IComm* pComm, int character);
static void sendBytes(IComm* pComm, const void* pBuffer, size_t bufferSize);

static ICommVTable g_icommVTable = {hasReceiveData, receiveChar, receiveBytes, sendChar, sendBytes};

static struct SocketIComm
{
    ICommVTable* pVTable;
    FILE*        pLogFile;
    int          socket;
    int          writeLast;
    uint32_t     secTimeout;
    uint32_t     usecTimeout;
} g_comm = {&g_icommVTable, NULL, -1, 0, 0, 0};


static void openLogFileIfNeeded(SocketIComm* pThis);
static void lookupAddress(struct sockaddr_in* pServerAddress, const char* nameWithPort);
static void openSocket(SocketIComm* pThis, struct sockaddr_in* pServerAddress);
static int socketHasDataToRead(int socket);


__throws IComm* SocketIComm_Init(const char* remoteName, uint32_t msecTimeout)
{
    SocketIComm*       pThis = &g_comm;

    /* Remember the desired timeout for reads/writes */
    pThis->secTimeout = msecTimeout / 1000;
    msecTimeout = msecTimeout % 1000;
    pThis->usecTimeout = msecTimeout * 1000;

    __try
    {
        struct sockaddr_in remoteAddress;

        openLogFileIfNeeded(pThis);
        lookupAddress(&remoteAddress, remoteName);
        openSocket(pThis, &remoteAddress);
    }
    __catch
    {
        SocketIComm_Uninit((IComm*)pThis);
        __rethrow;
    }

    return (IComm*)pThis;
}

static void openLogFileIfNeeded(SocketIComm* pThis)
{
    if (!SOCKET_LOG)
        return;
    pThis->pLogFile = fopen("socket.log", "w");
    pThis->writeLast = 0;
}

static void lookupAddress(struct sockaddr_in* pServerAddress, const char* nameWithPort)
{
    struct hostent*     pHostEntry = NULL;
    const char*         pColonLocation = NULL;
    uint16_t            portNumber;
    char                remoteName[256];

    pColonLocation = strchr(nameWithPort, ':');
    if (pColonLocation)
    {
        size_t len = pColonLocation - nameWithPort;
        if (len > sizeof(remoteName) - 1)
        {
            fprintf(stderr, "error: %s is too long for server name\n", nameWithPort);
            __throw(socketException);
        }
        memcpy(remoteName, nameWithPort, len);
        remoteName[len] = '\0';
        portNumber = atoi(pColonLocation + 1);
    }
    else
    {
        if (strlen(nameWithPort) > sizeof(remoteName) - 1)
        {
            fprintf(stderr, "error: %s is too long for server name\n", nameWithPort);
            __throw(socketException);
        }
        strcpy(remoteName, nameWithPort);
        portNumber = 23;
    }
    pHostEntry = gethostbyname(remoteName);
    if (!pHostEntry)
    {
        fprintf(stderr, "error: Failed to lookup address for %s.\n", remoteName);
        __throw(socketException);
    }

    memset(pServerAddress, 0, sizeof(*pServerAddress));
    pServerAddress->sin_family = AF_INET;
    memcpy(&pServerAddress->sin_addr.s_addr, pHostEntry->h_addr_list[0], sizeof(pServerAddress->sin_addr.s_addr));
    pServerAddress->sin_port = htons(portNumber);
}

static void openSocket(SocketIComm* pThis, struct sockaddr_in* pServerAddress)
{
    int result = -1;

    pThis->socket = socket(PF_INET, SOCK_STREAM, 0);
    if (pThis->socket == -1)
        __throw(socketException);
    result = connect(pThis->socket, (struct sockaddr *)pServerAddress, sizeof(*pServerAddress));
    if (result)
        __throw(socketException);
}


void SocketIComm_Uninit(IComm* pComm)
{
    SocketIComm* pThis = (SocketIComm*)pComm;

    if (!pThis)
        return;

    if (pThis->pLogFile)
    {
        fclose(pThis->pLogFile);
        pThis->pLogFile = NULL;
    }
    if (pThis->socket != -1)
    {
        close(pThis->socket);
        pThis->socket = -1;
    }
}



/* IComm Interface Implementation. */
static int hasReceiveData(IComm* pComm)
{
    int            hasData = FALSE;
    SocketIComm*   pThis = (SocketIComm*)pComm;

    __try
    {
        hasData = socketHasDataToRead(pThis->socket);
    }
    __catch
    {
        return FALSE;
    }
    return hasData;
}

static int socketHasDataToRead(int socket)
{
    int            result = -1;
    struct timeval zeroTimeout = {0, 0};
    fd_set         readSet;

    FD_ZERO(&readSet);
    FD_SET(socket, &readSet);
    result = select(socket + 1, &readSet, NULL, NULL, &zeroTimeout);
    if (result == -1)
        __throw(socketException);
    return result;
}

static int receiveChar(IComm* pComm)
{
    SocketIComm*   pThis = (SocketIComm*)pComm;
    ssize_t        bytesRead = 0;
    uint8_t        byte = 0;
    int            selectResult = -1;
    fd_set         readfds;
    fd_set         errorfds;
    struct timeval timeOut = {pThis->secTimeout, pThis->usecTimeout};

    FD_ZERO(&readfds);
    FD_ZERO(&errorfds);
    FD_SET(pThis->socket, &readfds);

    selectResult = select(pThis->socket+1, &readfds, NULL, &errorfds, &timeOut);
    if (selectResult == -1)
        __throw(socketException);
    if (selectResult == 0)
        __throw(timeoutException);

    bytesRead = recv(pThis->socket, &byte, sizeof(byte), 0);
    if (bytesRead == -1)
        __throw(socketException);

    if (SOCKET_LOG && pThis->pLogFile)
    {
        if (pThis->writeLast)
            fwrite("\n[r]", 1, 4, pThis->pLogFile);
        pThis->writeLast = 0;
        if (isprint(byte))
            fwrite(&byte, 1, 1, pThis->pLogFile);
        else
            fprintf(pThis->pLogFile, "\\x%02x", byte);
        fflush(pThis->pLogFile);
    }

    return (int)byte;
}

static void receiveBytes(IComm* pComm, void* pvBuffer, size_t bytesToRead)
{
    SocketIComm*   pThis = (SocketIComm*)pComm;
    uint8_t*       pCurr = (uint8_t*)pvBuffer;
    size_t         bytesLeft = bytesToRead;
    ssize_t        bytesRead = 0;

    while (bytesLeft > 0)
    {
        bytesRead = recv(pThis->socket, pCurr, bytesLeft, 0);
        if (bytesRead == -1)
            __throw(socketException);
        pCurr += bytesRead;
        bytesLeft -= bytesRead;
    }

    if (SOCKET_LOG && pThis->pLogFile)
    {
        if (pThis->writeLast)
            fwrite("\n[r]", 1, 4, pThis->pLogFile);
        pThis->writeLast = 0;

        pCurr = (uint8_t*)pvBuffer;
        size_t   bytesToLog = bytesToRead;
        while (bytesToLog-- > 0)
        {
            uint8_t byte = *pCurr++;
            if (isprint(byte))
                fwrite(&byte, 1, 1, pThis->pLogFile);
            else
                fprintf(pThis->pLogFile, "\\x%02x", byte);
        }
        fflush(pThis->pLogFile);
    }
}

static void sendChar(IComm* pComm, int character)
{
    SocketIComm*   pThis = (SocketIComm*)pComm;
    uint8_t        byte = (uint8_t)character;
    int            result = -1;
    fd_set         writefds;
    fd_set         errorfds;
    struct timeval timeOut = {pThis->secTimeout, pThis->usecTimeout};

    FD_ZERO(&writefds);
    FD_ZERO(&errorfds);
    FD_SET(pThis->socket, &writefds);

    result = select(pThis->socket+1, NULL, &writefds, &errorfds, &timeOut);
    if (result == -1)
        __throw(socketException);
    if (result == 0)
        __throw(timeoutException);

    result = send(pThis->socket, &byte, sizeof(byte), 0);
    if (result == -1)
        __throw(socketException);

    if (SOCKET_LOG && pThis->pLogFile)
    {
        if (!pThis->writeLast)
            fwrite("\n[w]", 1, 4, pThis->pLogFile);
        pThis->writeLast = 1;
        if (isprint(byte))
            fwrite(&byte, 1, 1, pThis->pLogFile);
        else
            fprintf(pThis->pLogFile, "\\x%02x", byte);
        fflush(pThis->pLogFile);
    }
}

static void sendBytes(IComm* pComm, const void* pvBuffer, size_t bufferSize)
{
    SocketIComm*   pThis = (SocketIComm*)pComm;
    int            result = -1;

    result = send(pThis->socket, pvBuffer, bufferSize, 0);
    if (result == -1)
        __throw(socketException);

    if (SOCKET_LOG && pThis->pLogFile)
    {
        if (!pThis->writeLast)
            fwrite("\n[w]", 1, 4, pThis->pLogFile);
        pThis->writeLast = 1;

        uint8_t* pBytes = (uint8_t*)pvBuffer;
        size_t   bytesToLog = bufferSize;
        while (bytesToLog-- > 0)
        {
            uint8_t byte = *pBytes++;
            if (isprint(byte))
                fwrite(&byte, 1, 1, pThis->pLogFile);
            else
                fprintf(pThis->pLogFile, "\\x%02x", byte);
        }
        fflush(pThis->pLogFile);
    }
}
