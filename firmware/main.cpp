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
#include "I2Cdev.h"
#include "Motor.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "PID.h"

// Set to 1 to have serial data echoed back to terminal.
#define SERIAL_ECHO 0

// Default motor PWM period.
#define PWM_PERIOD (1.0f / 20000.0f)

// Interval between PID updates (in seconds).
#define PID_INTERVAL (1.0f / 100.0f)

// Set to non-zero if you want to see raw gyro readings dumped to get a feel for drift.
#define DUMP_GYRO_RATINGS 0

#ifndef M_PI
const float M_PI = 3.14159265f;
#endif

struct TickInfo
{
    uint32_t      time;
    EncoderCounts counts;
    float         leftPWM;
    float         rightPWM;
    float         leftSetPoint;
    float         rightSetPoint;
};

static Serial                       g_serial(USBTX, USBRX);
static MPU6050                      g_mpu(p9, p10);
static InterruptIn                  g_imuReady(p8);
static DigitalOut                   g_imuActiveLED(LED1);
static PID                          g_rightPID(0.0056f, 0.03f, 0.0f, 0.35f, -1.0f, 1.0f, PID_INTERVAL);
static PID                          g_leftPID(0.0056f, 0.03f, 0.0f, 0.35f, -1.0f, 1.0f, PID_INTERVAL);
static Motor                        g_motors(p22, p29, p30, p21, p27, p26, p28, PWM_PERIOD);
// Note: Encoders object should be constructed after any other objects using InterruptIn so that the interrupt
//       handlers get chained together properly.
static Encoders<p12, p11, p13, p14> g_encoders;

static volatile TickInfo            g_tick;
static bool                         g_enableLogging = false;

// Used to communicate state between IMU packet interrupt handler and main code.
static uint16_t                     g_packetSize = 0;
static Quaternion                   g_latestQuaternion;
static volatile uint32_t            g_packetCount = 0;
static volatile uint32_t            g_overflowCount = 0;
static volatile uint32_t            g_spuriousCount = 0;

static int initIMU();
static void tickHandler();
static void updateMotorOutputs(float leftValue, float rightValue, float period);
static void serialRxHandler(void);
static void parseCommand(char* pCommand);
static void parseManualCommand(char* pCommand);
static float parseFloatValue(char** ppCommand);
static void skipWhitespace(char** ppCommand);
static float parseOptionalPeriod(char* pCommand, float defaultVal);
static void parseSetPointCommand(char* pCommand);
static void displayHelp();
static void imuPacketHandler();


int main()
{
    Ticker ticker;

    g_serial.baud(230400);

    int imuStatus = initIMU();
    if (imuStatus != 0)
        return imuStatus;

    updateMotorOutputs(0.0f, 0.0f, PWM_PERIOD);
    g_serial.attach(serialRxHandler);

    bool wasLoggingEnabled = g_enableLogging;
    uint32_t lastTick = g_tick.time;
    ticker.attach(tickHandler, PID_INTERVAL);

    for (;;)
    {
        // Wait for next tick update interrupt to be handled.
        while (lastTick == g_tick.time)
        {
        }
        lastTick = g_tick.time;

        if (g_enableLogging)
        {
            if (!wasLoggingEnabled && g_enableLogging)
            {
                // Just turned logging on so dump column headings.
                printf("time,leftSetPoint,leftPWM,leftEncoder,rightSetPoint,rightPWM,rightEncoder,yaw,pitch,roll\n");
            }

            VectorFloat gravity;
            float       ypr[3];
            g_mpu.dmpGetGravity(&gravity, &g_latestQuaternion);
            g_mpu.dmpGetYawPitchRoll(ypr, &g_latestQuaternion, &gravity);
            printf("%lu,%.2f,%.2f,%ld,%.2f,%.2f,%ld,%.2f,%.2f,%.2f\n",
                   g_tick.time,
                   g_tick.leftPWM, g_tick.leftSetPoint, g_tick.counts.encoder1Count,
                   g_tick.rightPWM, g_tick.rightSetPoint, g_tick.counts.encoder2Count,
                   ypr[0] * 180.0f/M_PI,
                   ypr[1] * 180.0f/M_PI,
                   ypr[2] * 180.0f/M_PI);

            // Can be useful to dump gyro values to determine drift.
            if (DUMP_GYRO_RATINGS)
            {
                int16_t gyrox, gyroy, gyroz;
                g_mpu.getRotation(&gyrox, &gyroy, &gyroz);
                printf("%d,%d,%d\n", gyrox, gyroy, gyroz);
            }
        }
        wasLoggingEnabled = g_enableLogging;
    }

    return 0;
}

static int initIMU()
{
    printf("Initializing I2C devices...\r\n");
    g_mpu.initialize();

    printf("Testing device connections...\r\n");
    if (g_mpu.testConnection())
        printf("MPU6050 connection successful\r\n");
    else
        printf("MPU6050 connection failed\r\n");

    printf("Initializing DMP...\r\n");
    uint8_t devStatus = g_mpu.dmpInitialize();
    if (devStatus != 0)
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        printf("DDMP Initialization failed (code %d)\r\n", devStatus);
        return devStatus;
    }

    printf("Setting gyro bias to reduce drift.\r\n");
	g_mpu.setXGyroOffsetTC(7);
	g_mpu.setYGyroOffsetTC(-4);
	g_mpu.setZGyroOffsetTC(11);

    printf("Enabling DMP...\r\n");
    g_mpu.setDMPEnabled(true);

    printf("Enabling interrupt detection...\r\n");
    g_imuReady.rise(&imuPacketHandler);

    // Clear any already pending interrupt bits.
    g_mpu.getIntStatus();

    printf("DMP ready!\r\n");

    // get expected DMP packet size for later comparison
    g_packetSize = g_mpu.dmpGetFIFOPacketSize();

    return 0;
}

static void tickHandler()
{
    EncoderCounts encoderCounts = g_encoders.getAndClearEncoderCounts();
    g_motors.set(g_leftPID.compute(encoderCounts.encoder1Count), g_rightPID.compute(encoderCounts.encoder2Count));

    g_tick.counts.encoder1Count = encoderCounts.encoder1Count;
    g_tick.counts.encoder2Count = encoderCounts.encoder2Count;
    g_tick.leftPWM = g_leftPID.getControlOutput();
    g_tick.rightPWM = g_rightPID.getControlOutput();
    g_tick.leftSetPoint = g_leftPID.getSetPoint();
    g_tick.rightSetPoint = g_rightPID.getSetPoint();
    g_tick.time++;
}

static void updateMotorOutputs(float leftValue, float rightValue, float period)
{
    g_motors.setPeriod(period);
    g_leftPID.setOutputManually(leftValue);
    g_rightPID.setOutputManually(rightValue);

    printf("Manual PWM Mode - PWM frequency = %f\n", 1.0f / period);
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
    char cmd = *pCommand++;
    g_badInput = false;
    switch (tolower(cmd))
    {
    case 'a':
        printf("Enabling PID automatic mode\n");
        g_leftPID.enableAutomaticMode();
        g_rightPID.enableAutomaticMode();
        break;
    case 'l':
        g_enableLogging = !g_enableLogging;
        break;
    case 'm':
        parseManualCommand(pCommand);
        break;
    case 's':
        parseSetPointCommand(pCommand);
        break;
    case 'h':
    case '?':
        displayHelp();
        break;
    default:
        g_badInput = true;
        break;
    }

    if (g_badInput)
    {
        // Stop the motor on any bad/unrecognized input.
        updateMotorOutputs(0.0f, 0.0f, g_motors.getPeriod());
    }
}

static void parseManualCommand(char* pCommand)
{
    float rightPWM = 0.0f;
    float leftPWM = 0.0f;

    leftPWM = parseFloatValue(&pCommand) / 100.0f;
    skipWhitespace(&pCommand);
    rightPWM = parseFloatValue(&pCommand) / 100.0f;
    skipWhitespace(&pCommand);
    float period = parseOptionalPeriod(pCommand, g_motors.getPeriod());

    if (!g_badInput)
    {
        updateMotorOutputs(leftPWM, rightPWM, period);
    }
}

static float parseFloatValue(char** ppCommand)
{
    char* pCommand = *ppCommand;
    char* pEnd = pCommand;

    float value = strtof(pCommand, &pEnd);
    if (pEnd == pCommand)
        g_badInput = true;
    *ppCommand = pEnd;

    return value;
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

static void parseSetPointCommand(char* pCommand)
{
    float left = 0.0f;
    float right = 0.0f;

    left = parseFloatValue(&pCommand);
    skipWhitespace(&pCommand);
    right = parseFloatValue(&pCommand);

    if (!g_badInput)
    {
        printf("Update SetPoint\n");
        g_leftPID.updateSetPoint(left);
        g_rightPID.updateSetPoint(right);
    }
}

static void displayHelp()
{
    printf("Help\n"
           "       manual motor setting: m leftPWM rightPWM (period)\n"
           "      toggle logging on/off: l\n"
           "toggle automatic PID on/off: a\n"
           "           update set point: s leftRate rightRate\n");
}

static void imuPacketHandler()
{
    uint8_t  fifoBuffer[64];

    uint8_t g_mpuIntStatus = g_mpu.getIntStatus();
    uint16_t fifoCount = g_mpu.getFIFOCount();
    if ((g_mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // Hit FIFO overflow - this shouldn't happen unless this interrupt handler is too slow.
        g_overflowCount++;
        g_mpu.resetFIFO();
        return;
    }
    else if (0 == (g_mpuIntStatus & 0x02))
    {
        // Didn't receive an interrupt indicating that FIFO has data to read.
        g_spuriousCount++;
        return;
    }

    // Wait for correct available data length, should be a VERY short wait.
    while (fifoCount < g_packetSize)
        fifoCount = g_mpu.getFIFOCount();

    while (fifoCount >= g_packetSize)
    {
        g_mpu.getFIFOBytes(fifoBuffer, g_packetSize);

        // Track FIFO count here in case there is > 1 packet available.
        fifoCount -= g_packetSize;

        // Parse out the g_latestQuaternionuaternion into global variable.
        g_mpu.dmpGetQuaternion(&g_latestQuaternion, fifoBuffer);
        g_packetCount++;

        // blink LED to indicate activity
        g_imuActiveLED = !g_imuActiveLED;
    }
}
