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
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include "SerialIComm.h"


/* Set SERIAL_LOG to 1 to have all serial I/O logged to serial.log file and set to 0 otherwise. */
#define SERIAL_LOG 1


/* Implementation of IComm interface. */
typedef struct SerialIComm SerialIComm;

static int  hasReceiveData(IComm* pComm);
static int  receiveChar(IComm* pComm);
static void receiveBytes(IComm* pComm, void* pvBuffer, size_t bufferSize);
static void sendChar(IComm* pComm, int character);
static void sendBytes(IComm* pComm, const void* pvBuffer, size_t bufferSize);

static ICommVTable g_icommVTable = {hasReceiveData, receiveChar, receiveBytes, sendChar, sendBytes};

static struct SerialIComm
{
    ICommVTable*   pVTable;
    FILE*          pLogFile;
    struct termios origTerm;
    int            file;
    int            origTermValid;
    int            writeLast;
    uint32_t       secTimeout;
    uint32_t       usecTimeout;
} g_comm;


static void openLogFileIfNeeded(SerialIComm* pThis);
static void openAndConfigureSerialPort(SerialIComm* pThis, const char* pDevicePath, uint32_t baudRate, uint32_t msecTimeout);


__throws IComm* SerialIComm_Init(const char* pDevicePath, uint32_t baudRate, uint32_t msecTimeout)
{
    SerialIComm* pThis = &g_comm;

    memset(pThis, 0, sizeof(*pThis));
    pThis->pVTable = &g_icommVTable;
    pThis->file = -1;
    __try
    {
        openLogFileIfNeeded(pThis);
        openAndConfigureSerialPort(pThis, pDevicePath, baudRate, msecTimeout);
    }
    __catch
    {
        SerialIComm_Uninit((IComm*)pThis);
        __rethrow;
    }

    return (IComm*)pThis;
}

static void openLogFileIfNeeded(SerialIComm* pThis)
{
    if (!SERIAL_LOG)
        return;
    pThis->pLogFile = fopen("serial.log", "w");
    pThis->writeLast = 0;
}

static void openAndConfigureSerialPort(SerialIComm* pThis, const char* pDevicePath, uint32_t baudRate, uint32_t msecTimeout)
{
    int            result = -1;
    struct termios newTerm;

    pThis->file = open(pDevicePath, O_RDWR | O_NONBLOCK);
    if (pThis->file == -1)
    {
        perror("error: Failed to open serial port");
        __throw(serialException);
    }

    /* Remember the desired timeout for reads/writes */
    pThis->secTimeout = msecTimeout / 1000;
    msecTimeout = msecTimeout % 1000;
    pThis->usecTimeout = msecTimeout * 1000;

    /* Configure serial port settings. */
    result = tcgetattr(pThis->file, &pThis->origTerm);
    if (result == -1)
        __throw(serialException);
    pThis->origTermValid = 1;
    newTerm = pThis->origTerm;

    /* Set the baud rate. */
    result = cfsetspeed(&newTerm, baudRate);
    if (result == -1)
    {
        perror("error: Failed to set baud rate");
        __throw(serialException);
    }

    /* Use Non-canonical mode. */
    cfmakeraw(&newTerm);

    /* No input or output mapping. */
    newTerm.c_iflag = 0;
    newTerm.c_oflag = 0;


    /* Configure for 8n1 format and disable modem flow control. */
    newTerm.c_cflag = CS8 | CREAD | CLOCAL;

    /* Set MIN characters and TIMEout for non-canonical mode. */
    newTerm.c_cc[VMIN] = 0;
    newTerm.c_cc[VTIME] = 0;

    result = tcflush(pThis->file, TCIOFLUSH);
    if (result != 0)
        __throw(serialException);

    result = tcsetattr(pThis->file, TCSANOW, &newTerm);
    if (result == -1)
    {
        perror("error: Failed to configure serial port");
        __throw(serialException);
    }

    /* This might only be required due to the mbed CDC firmware. */
    usleep(100000);
}


void SerialIComm_Uninit(IComm* pComm)
{
    SerialIComm* pThis = (SerialIComm*)pComm;

    if (!pThis)
        return;

    if (pThis->origTermValid)
        tcsetattr(pThis->file, TCSAFLUSH, &pThis->origTerm);
    if (pThis->file != -1)
    {
        close(pThis->file);
        pThis->file = -1;
    }
    if (pThis->pLogFile)
    {
        fclose(pThis->pLogFile);
        pThis->pLogFile = NULL;
    }
}


/* IComm Interface Implementation. */
static int hasReceiveData(IComm* pComm)
{
    SerialIComm*   pThis = (SerialIComm*)pComm;
    int            selectResult = -1;
    struct timeval timeOut = {0, 0};
    fd_set         readfds;
    fd_set         errorfds;

    FD_ZERO(&readfds);
    FD_ZERO(&errorfds);
    FD_SET(pThis->file, &readfds);

    selectResult = select(pThis->file+1, &readfds, NULL, &errorfds, &timeOut);
    if (selectResult == -1)
        __throws(serialException);
    if (selectResult == 0)
        return 0;
    else
        return 1;
}

static int receiveChar(IComm* pComm)
{
    SerialIComm*   pThis = (SerialIComm*)pComm;
    ssize_t        bytesRead = 0;
    uint8_t        byte = 0;
    int            selectResult = -1;
    fd_set         readfds;
    fd_set         errorfds;
    struct timeval timeOut = {pThis->secTimeout, pThis->usecTimeout};

    FD_ZERO(&readfds);
    FD_ZERO(&errorfds);
    FD_SET(pThis->file, &readfds);

    selectResult = select(pThis->file+1, &readfds, NULL, &errorfds, &timeOut);
    if (selectResult == -1)
        __throw(serialException);
    if (selectResult == 0)
        __throw(timeoutException);

    bytesRead = read(pThis->file, &byte, sizeof(byte));
    if (bytesRead == -1)
        __throw(serialException);

    if (SERIAL_LOG && pThis->pLogFile)
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

static void receiveBytes(IComm* pComm, void* pvBuffer, size_t bufferSize)
{
    uint8_t* pBuffer = (uint8_t*)pvBuffer;
    while (bufferSize-- > 0)
        *pBuffer++ = receiveChar(pComm);
}

static void sendChar(IComm* pComm, int character)
{
    SerialIComm*   pThis = (SerialIComm*)pComm;
    uint8_t        byte = (uint8_t)character;
    ssize_t        bytesWritten = 0;
    int            selectResult = -1;
    fd_set         writefds;
    fd_set         errorfds;
    struct timeval timeOut = {pThis->secTimeout, pThis->usecTimeout};

    FD_ZERO(&writefds);
    FD_ZERO(&errorfds);
    FD_SET(pThis->file, &writefds);

    selectResult = select(pThis->file+1, NULL, &writefds, &errorfds, &timeOut);
    if (selectResult == -1)
        __throw(serialException);
    if (selectResult == 0)
        __throw(timeoutException);

    bytesWritten = write(pThis->file, &byte, sizeof(byte));
    if (bytesWritten != sizeof(byte))
        __throw(serialException);
    if (SERIAL_LOG && pThis->pLogFile)
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
    const uint8_t* pBuffer = (const uint8_t*)pvBuffer;
    while (bufferSize-- > 0)
        sendChar(pComm, *pBuffer++);
}
