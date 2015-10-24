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
#include "PwmIn.h"

// Set to 1 to have serial data echoed back to terminal.
#define SERIAL_ECHO 0

// Set to 1 to disable PID loop and brake motors when radio is turned off.
#define BRAKE_ON_RADIO_TIMEOUT 1

// Enable RC control of motor velocity by setting to non-zero.
#define RADIO_ENABLE 1

// Default motor PWM period.
#define PWM_PERIOD (1.0f / 10000.0f)

// Maximum motor output PWM duty cycle.
// Useful for limiting amount of power to motors during early testing.
#define MAX_MOTOR_POWER 0.50f

// Interval between PID updates (in seconds).
#define PID_INTERVAL (1.0f / 100.0f)

// Set to non-zero if you want to see raw gyro readings dumped to get a feel for drift.
#define DUMP_GYRO_RATINGS 0

// Maximum and minimum PWM duty cycles seen from Turnigy RC receiver.
// Set both to zero if you want to disable scaling of the PWM duty cycle to a range of -1.0f to 1.0f.
#define PWM_DUTY_CYCLE_MIN 0.0530f
#define PWM_DUTY_CYCLE_MAX 0.0951f

// The radio should send a pulse 50 times a second so if we go 1/25th of a second without receiving 1 then flag as a
// time out.
#define RADIO_TIMEOUT (1000000 / 25)

#ifndef M_PI
const float M_PI = 3.14159265f;
#endif

// Which element in the IMU's ypr array represents our platform pitch.
#define PITCH_ELEMENT 1

// Should the IMU's pitch be inverted.  Want pitching the front of the platform up to be positive.
#define PITCH_INVERT  true

// How far to pitch the platform during calibration testing.
#define PITCH_CALIBRATE_ANGLE (15.0f * (M_PI / 180.f))

// PWM duty cycle to use when peforming the pitch calibration.
#define PITCH_CALIBRATE_PWM 0.20f


// Run modes for this program.
enum ProgramModes
{
    // Default run mode
    MODE_DEFAULT = 0,
    // Used to oscillate the pitch to determine PID constants.
    MODE_PITCH_CALIBRATE = 1
} g_programMode = MODE_PITCH_CALIBRATE;

struct TickInfo
{
    volatile uint32_t   time;
    EncoderCounts       counts;
    float               leftPWM;
    float               rightPWM;
    float               leftSetPoint;
    float               rightSetPoint;
    Quaternion          quaternion;
};

// MRI will take care of initializing the UART if has been linked into the program.
// UNDONE: I am planning to move MRI to it's own UART in the future.
#if !MRI_ENABLE
    static Serial g_serial(USBTX, USBRX);
    #define INIT_SERIAL_BAUD(BAUD) g_serial.baud(BAUD)
#else
    #define INIT_SERIAL_BAUD(BAUD) (void)0
#endif // !MRI_ENABLE

static MPU6050                      g_mpu(p9, p10);
static PwmIn                        g_radioYaw(p17);
static PwmIn                        g_radioPitch(p18);
static PID                          g_rightPID(0.0059f, 0.04f, 0.0f, 0.30f, -MAX_MOTOR_POWER, MAX_MOTOR_POWER, PID_INTERVAL);
static PID                          g_leftPID(0.0059f, 0.04f, 0.0f, 0.30f, -MAX_MOTOR_POWER, MAX_MOTOR_POWER, PID_INTERVAL);
static Motor                        g_motors(p22, p29, p30, p21, p20, p19, p26, MAX_MOTOR_POWER, MAX_MOTOR_POWER, PWM_PERIOD);
// Note: Encoders object should be constructed after any other objects using InterruptIn so that the interrupt
//       handlers get chained together properly.
static Encoders<p12, p11, p15, p16> g_encoders;

static TickInfo                     g_tick;
static bool                         g_enableLogging = false;
static uint16_t                     g_packetSize = 0;
static volatile uint32_t            g_overflowCount = 0;

// Keep a log in the upper RAM banks of the LPC1768.
static float*                       g_pLog;
static __attribute((section("AHBSRAM0"),aligned)) uint8_t g_log1[16 * 1024];
static __attribute((section("AHBSRAM1"),aligned)) uint8_t g_log2[16 * 1024];


static void initInterruptPriorities();
static int initIMU();
static void updateMotorOutputs(float leftValue, float rightValue, float period);
static void tickHandler();
static void readLatestImuPacket(Quaternion* pQuaternion);
static int runDefaultMode();
static void serialRxHandler(void);
static void parseCommand(char* pCommand);
static void parseManualCommand(char* pCommand);
static float parseFloatValue(char** ppCommand);
static void skipWhitespace(char** ppCommand);
static float parseOptionalPeriod(char* pCommand, float defaultVal);
static void parseSetPointCommand(char* pCommand);
static void displayHelp();
static float scalePwmDutyCycle(float dutyCycle);
static int runPitchCalibrateMode();


int main()
{
    Ticker ticker;

    INIT_SERIAL_BAUD(230400);
    initInterruptPriorities();

    int imuStatus = initIMU();
    if (imuStatus != 0)
    {
        printf("error: Failed to initialize IMU.  Shutting down.\n");
        return imuStatus;
    }

    updateMotorOutputs(0.0f, 0.0f, PWM_PERIOD);

    ticker.attach(tickHandler, PID_INTERVAL);

    switch (g_programMode)
    {
    case MODE_DEFAULT:
        return runDefaultMode();
    case MODE_PITCH_CALIBRATE:
        return runPitchCalibrateMode();
    }

    return -1;
}

static void initInterruptPriorities()
{
    // Time critical GPIO interrupts are given highest priority.
    // Getting data in from serial port is lowest priority.
    NVIC_SetPriority(EINT3_IRQn, 1);
    NVIC_SetPriority(TIMER3_IRQn, 2);
    if (!MRI_ENABLE)
        NVIC_SetPriority(UART0_IRQn, 3);
}

static int initIMU()
{
    g_mpu.initialize();

    if (!g_mpu.testConnection())
        printf("error: MPU6050 connection failed\r\n");

    uint8_t devStatus = g_mpu.dmpInitialize();
    if (devStatus != 0)
    {
        // How to interpret failure code:
        //  1 = initial memory load failed
        //  2 = DMP configuration updates failed.
        // If it's going to break, usually the code will be 1.
        printf("error: DMP Initialization failed (code %d)\n", devStatus);
        return devStatus;
    }

    // Setting gyro bias to reduce drift.
	g_mpu.setXGyroOffsetTC(7);
	g_mpu.setYGyroOffsetTC(-4);
	g_mpu.setZGyroOffsetTC(11);

    g_mpu.setDMPEnabled(true);

    // Clear any already pending interrupt bits.
    g_mpu.getIntStatus();

    // Get expected DMP packet size for later comparison.
    g_packetSize = g_mpu.dmpGetFIFOPacketSize();

    return 0;
}

static void updateMotorOutputs(float leftValue, float rightValue, float period)
{
    g_motors.setPeriod(period);
    g_leftPID.setOutputManually(leftValue);
    g_rightPID.setOutputManually(rightValue);

    printf("Manual PWM Mode - PWM frequency = %f\n", 1.0f / period);
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
    readLatestImuPacket(&g_tick.quaternion);
    g_tick.time++;
}

static void readLatestImuPacket(Quaternion* pQuaternion)
{
    uint8_t  fifoBuffer[64];

    uint8_t g_mpuIntStatus = g_mpu.getIntStatus();
    uint16_t fifoCount = g_mpu.getFIFOCount();
    if ((g_mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // Hit FIFO overflow.
        g_overflowCount++;
        g_mpu.resetFIFO();
        return;
    }

    // Read all available IMU packets.  We only care about the latest one.
    while (fifoCount >= g_packetSize)
    {
        g_mpu.getFIFOBytes(fifoBuffer, g_packetSize);

        // Track FIFO count here in case there is > 1 packet available.
        fifoCount -= g_packetSize;

        // Parse out the g_latestQuaternionuaternion into global variable.
        g_mpu.dmpGetQuaternion(pQuaternion, fifoBuffer);
    }
}

static int runDefaultMode()
{
    // UNDONE: I will need to completely replace this serial method of getting commands as it isn't compatible with
    //         MRI / GDB.
    g_serial.attach(serialRxHandler);

    bool wasLoggingEnabled = false;
    uint32_t lastTick = g_tick.time;

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
                printf("time,leftPWM,leftSetPoint,leftEncoder,rightPWM,rightSetPoint,rightEncoder,"
                       "yaw,pitch,roll,radioYaw,radioPitch\n");
            }

            VectorFloat gravity;
            float       ypr[3];
            g_mpu.dmpGetGravity(&gravity, &g_tick.quaternion);
            g_mpu.dmpGetYawPitchRoll(ypr, &g_tick.quaternion, &gravity);
            printf("%lu,%.2f,%.2f,%ld,%.2f,%.2f,%ld,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                   g_tick.time,
                   g_tick.leftPWM, g_tick.leftSetPoint, g_tick.counts.encoder1Count,
                   g_tick.rightPWM, g_tick.rightSetPoint, g_tick.counts.encoder2Count,
                   ypr[0] * 180.0f/M_PI,
                   ypr[1] * 180.0f/M_PI,
                   ypr[2] * 180.0f/M_PI,
                   g_radioYaw.hasTimedOut(RADIO_TIMEOUT) ? 0.0f : scalePwmDutyCycle(g_radioYaw.getDutyCycle()),
                   g_radioPitch.hasTimedOut(RADIO_TIMEOUT) ? 0.0f : scalePwmDutyCycle(g_radioPitch.getDutyCycle()));

            // Can be useful to dump gyro values to determine drift.
            if (DUMP_GYRO_RATINGS)
            {
                int16_t gyrox, gyroy, gyroz;
                g_mpu.getRotation(&gyrox, &gyroy, &gyroz);
                printf("%d,%d,%d\n", gyrox, gyroy, gyroz);
            }
        }
        wasLoggingEnabled = g_enableLogging;

        if (RADIO_ENABLE)
        {
            // Set forward / reverse velocity based on pitch channel of radio.
            bool hasRadioTimedOut = g_radioPitch.hasTimedOut(RADIO_TIMEOUT);
            float desiredVelocity = hasRadioTimedOut ? 0.0f : scalePwmDutyCycle(g_radioPitch.getDutyCycle()) * 32.0f;
            if (BRAKE_ON_RADIO_TIMEOUT && hasRadioTimedOut)
            {
                g_leftPID.setOutputManually(0.0f);
                g_rightPID.setOutputManually(0.0f);
            }
            else
            {
                g_leftPID.enableAutomaticMode();
                g_rightPID.enableAutomaticMode();
                g_leftPID.updateSetPoint(desiredVelocity);
                g_rightPID.updateSetPoint(desiredVelocity);
            }
        }
    }

    return 0;
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

static float scalePwmDutyCycle(float dutyCycle)
{
    if (PWM_DUTY_CYCLE_MIN != 0.0f && PWM_DUTY_CYCLE_MAX != 0.0f)
    {
        static const float min = PWM_DUTY_CYCLE_MIN;
        static const float max = PWM_DUTY_CYCLE_MAX;
        static const float center = (min + max) / 2.0f;

        float scaledResult = (dutyCycle - center) * 2.0f / (max - min);
        if (scaledResult < -1.0f)
            scaledResult = -1.0f;
        else if (scaledResult > 1.0f)
            scaledResult = 1.0f;
        return scaledResult;
    }
    else
    {
        return dutyCycle;
    }
}

static int runPitchCalibrateMode()
{
    static const float* pLogEnd = (float*)(g_log2 + sizeof(g_log2));
    DigitalOut          led1(LED1);
    LocalFileSystem     local("local");
    uint32_t            lastTick = g_tick.time;
    float               pitchPwm = PITCH_CALIBRATE_PWM;
    enum
    {
        PITCH_FORWARD = 0,
        PITCH_BACKWARD = 1
    } pitchMode = PITCH_FORWARD;


    printf("Pitch Calibration\n");
    for (;;)
    {
        // Make sure that the remote control is turned off and only start test once it has been turned on again.
        printf("Waiting for remote control to be turned OFF...\n");
        while (!g_radioPitch.hasTimedOut(RADIO_TIMEOUT))
        {
            // Blink LED FASTER to let user know that we are waiting for them to turn radio OFF.
            led1 = !led1;
            wait_ms(250);
        }

        // Wait for remote control to be turned back on to start the test.
        printf("Waiting for remote control to be turned ON...\n");
        while (g_radioPitch.hasTimedOut(RADIO_TIMEOUT))
        {
            // Blink LED SLOWER to let user know that we are waiting for them to turn radio ON.
            led1 = !led1;
            wait_ms(500);
        }
        led1 = 0;

        // Run the calibration test now by oscillating back and forth and recording the resulting data.
        printf("Running pitch calibration test...\n\n");
        printf("time,leftPWM,leftEncoder,yaw,pitch,roll\n");
        g_pLog = (float*)&g_log1[0];
        for (;;)
        {
            // Wait for next tick update interrupt to be handled.
            while (lastTick == g_tick.time)
            {
            }
            lastTick = g_tick.time;

            VectorFloat gravity;
            float       ypr[3];
            g_mpu.dmpGetGravity(&gravity, &g_tick.quaternion);
            ypr[0] = atan2f(gravity.y, gravity.z);
            ypr[1] = atan2f(gravity.x, gravity.z);
            ypr[2] = atan2f(gravity.x, gravity.y);

            float pitchAngle = ypr[PITCH_ELEMENT];
            if (PITCH_INVERT)
                pitchAngle = -pitchAngle;

            printf("%lu,%.2f,%ld,%.2f\n",
                   g_tick.time,
                   g_tick.leftPWM, g_tick.counts.encoder1Count,
                   pitchAngle * 180.0f/M_PI);

            // Keep logging data until we run out of RAM to store it all.
            *g_pLog++ = g_tick.leftPWM;
            *g_pLog++ = pitchAngle;
            if (g_pLog >= pLogEnd || g_radioPitch.hasTimedOut(RADIO_TIMEOUT))
                break;

            switch (pitchMode)
            {
            case PITCH_FORWARD:
                if (pitchAngle > PITCH_CALIBRATE_ANGLE)
                {
                    pitchPwm = -PITCH_CALIBRATE_PWM;
                    pitchMode = PITCH_BACKWARD;
                }
                break;
            case PITCH_BACKWARD:
                if (pitchAngle < -PITCH_CALIBRATE_ANGLE)
                {
                    pitchPwm = PITCH_CALIBRATE_PWM;
                    pitchMode = PITCH_FORWARD;
                }
                break;
            }

            g_leftPID.setOutputManually(pitchPwm);
        }

        // Get here once the log has been filled.
        g_leftPID.setOutputManually(0);
        FILE* pFile = fopen("/local/pitch.csv", "w");
        if (pFile)
        {
            float* pCurr = (float*)g_log1;

            fprintf(pFile, "pwm,pitch\n");
            while (pCurr < g_pLog)
            {
                fprintf(pFile, "%.2f,%.4f\n", pCurr[0], pCurr[1]);
                pCurr += 2;
            }
            fclose(pFile);
        }
    }

    return 0;
}
