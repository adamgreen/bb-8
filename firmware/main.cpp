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
   I need for the final control system.
*/
#include <ctype.h>
#include <mbed.h>
#include "Encoders.h"
#include "Motor.h"

// Set to 1 to have serial data echoed back to terminal.
#define SERIAL_ECHO 1

// Default motor PWM period.
#define PWM_PERIOD (1.0f / 20000.0f)

static Serial g_serial(USBTX, USBRX);
static Motor  g_motor(p22, p29, p30, p21, p27, p26, p28, PWM_PERIOD);

static void updateMotorOutputs(float rightValue, float leftValue, float period);
static void serialRxHandler(void);
static void parseCommand(char* pCommand);
static float parseMotorValue(char** ppCommand);
static void skipWhitespace(char** ppCommand);
static float parseOptionalPeriod(char* pCommand, float defaultVal);

int main()
{
    Encoders<p12, p11, p13, p14> encoders;

    g_serial.baud(230400);

    updateMotorOutputs(0.0f, 0.0f, PWM_PERIOD);
    g_serial.attach(serialRxHandler);

    for (;;)
    {
        wait(2.0f);
        EncoderCounts encoderCounts = encoders.getAndClearEncoderCounts();
        printf("%ld, %ld\n", encoderCounts.encoder1Count, encoderCounts.encoder2Count);
    }

    return 0;
}

static void updateMotorOutputs(float rightValue, float leftValue, float period)
{
    g_motor.setPeriod(period);
    g_motor.set(leftValue, rightValue);

    printf("frequency = %f\n", 1.0f / period);
    printf("    right = %f\n", rightValue * 100.0f);
    printf("     left = %f\n", leftValue * 100.0f);
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
    float rightMotor = 0.0f;
    float leftMotor = 0.0f;

    g_badInput = false;

    rightMotor = parseMotorValue(&pCommand);
    skipWhitespace(&pCommand);
    leftMotor = parseMotorValue(&pCommand);
    skipWhitespace(&pCommand);
    float period = parseOptionalPeriod(pCommand, g_motor.getPeriod());

    if (g_badInput)
    {
        // Stop the motor on bad input.
        updateMotorOutputs(0.0f, 0.0f, period);
    }
    else
    {
        updateMotorOutputs(rightMotor, leftMotor, period);
    }
}

static float parseMotorValue(char** ppCommand)
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
