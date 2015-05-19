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
/* This will become the firmware for controlling my Star Wars BB-8
   replica but at this time I am just bootstrapping the drivers that
   I need for this final control system.
*/
#include <ctype.h>
#include <mbed.h>

// Set to 1 to have serial data echoed back to terminal.
#define SERIAL_ECHO 1

typedef struct MotorState
{
    float    pulseWidth;
    uint32_t input;
} MotorState;

static MotorState g_rightMotor;
static MotorState g_leftMotor;
static float      g_period = 1.0f / 20000.0f;
static Serial     g_serial(USBTX, USBRX);
static PwmOut     g_pwmRight(p21); // B
static PwmOut     g_pwmLeft(p22);  // A
static BusOut     g_motorOut(p27, p26, p29, p30); // B1, B2, A1, A2
static DigitalOut g_standby(p28);

static void updateMotorOutputs(void);
static void serialRxHandler(void);
static void parseCommand(char* pCommand);
static uint32_t parseMotorInput(char** ppCommand);
static float parseMotorWidth(char** ppCommand);
static void skipWhitespace(char** ppCommand);
static float parseOptionalPeriod(char* pCommand, float defaultVal);

int main()
{
    g_serial.baud(230400);

    updateMotorOutputs();
    g_serial.attach(serialRxHandler);

    for (;;)
    {
    }

    return 0;
}

static void updateMotorOutputs(void)
{
    g_standby = 1;
    g_pwmRight.period(g_period);
    g_pwmRight = g_rightMotor.pulseWidth;
    g_pwmLeft = g_leftMotor.pulseWidth;
    g_motorOut = (((g_leftMotor.input & 3) << 2) | (g_rightMotor.input & 3));

    printf("frequency = %f\n", 1.0f / g_period);
    printf("right=%lu,%f\n", g_rightMotor.input, g_rightMotor.pulseWidth * 100.0f);
    printf("left=%lu,%f\n", g_leftMotor.input, g_leftMotor.pulseWidth * 100.0f);
}

static void serialRxHandler(void)
{
    static char  buffer[128];
    static char* pCurr = buffer;
    static char* pEnd = buffer + sizeof(buffer) - 1;

    while (g_serial.readable())
    {
        char curr = g_serial.getc();
        if (SERIAL_ECHO)
            g_serial.putc(curr);

        if (curr == '\n')
        {
            *pCurr = '\0';
            parseCommand(buffer);
            pCurr = buffer;
        }
        else if (curr != '\r' && pCurr < pEnd)
        {
            *pCurr++ = curr;
        }
    }
}

static bool g_badInput;
static void parseCommand(char* pCommand)
{
    MotorState rightMotor = g_rightMotor;
    MotorState leftMotor = g_leftMotor;

    g_badInput = false;

    rightMotor.input = parseMotorInput(&pCommand);
    rightMotor.pulseWidth = parseMotorWidth(&pCommand);
    skipWhitespace(&pCommand);
    leftMotor.input = parseMotorInput(&pCommand);
    leftMotor.pulseWidth = parseMotorWidth(&pCommand);
    skipWhitespace(&pCommand);
    float period = parseOptionalPeriod(pCommand, g_period);

    if (g_badInput)
    {
        // Stop the motor on bad input.
        g_rightMotor.input = 0;
        g_rightMotor.pulseWidth = 0.0f;
        g_leftMotor.input = 0;
        g_leftMotor.pulseWidth = 0.0f;
    }
    else
    {
        g_rightMotor = rightMotor;
        g_leftMotor = leftMotor;
        g_period = period;
    }

    updateMotorOutputs();
}

static uint32_t parseMotorInput(char** ppCommand)
{
    char     ch = **ppCommand;
    uint32_t retVal = 0;

    switch (tolower(ch))
    {
    case 'f':
        retVal = 2;
        break;
    case 'r':
        retVal = 1;
        break;
    case '-':
        retVal = 3;
        break;
    default:
        g_badInput = true;
        break;
    }

    if (ch != '\0')
        (*ppCommand)++;
    return retVal;
}

static float parseMotorWidth(char** ppCommand)
{
    char* pCommand = *ppCommand;
    char* pEnd = pCommand;

    float width = strtof(pCommand, &pEnd);
    if (pEnd == pCommand)
        g_badInput = true;
    *ppCommand = pEnd;

    return width / 100.0f;
}

static void skipWhitespace(char** ppCommand)
{
    char* pCommand = *ppCommand;

    while (isspace(*pCommand))
        pCommand++;
    *ppCommand = pCommand;
}

static float parseOptionalPeriod(char* pCommand, float defaultVal)
{
    if (*pCommand == '\0')
        return defaultVal;

    uint32_t freq = strtoul(pCommand, NULL, 10);
    if (freq == 0 || freq > 100000)
    {
        g_badInput = true;
        return defaultVal;
    }

    return 1.0f / freq;
}
