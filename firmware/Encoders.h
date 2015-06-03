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
/* Class used to interface with two of the Pololu Magnetic Encoder Kits. */
#ifndef _ENCODERS_H_
#define _ENCODERS_H_

#include <assert.h>
#include <mbed.h>
#include <pinmap.h>
#include "Interlock.h"

class EncoderBase
{
protected:
    virtual void handleEncoderTick(uint32_t* pRise0, uint32_t* pFall0) = 0;

    static void interruptHandler()
    {
        uint32_t rise0 = LPC_GPIOINT->IO0IntStatR;
        uint32_t fall0 = LPC_GPIOINT->IO0IntStatF;
        uint32_t rise2 = LPC_GPIOINT->IO2IntStatR;
        uint32_t fall2 = LPC_GPIOINT->IO2IntStatF;

        // Get in and quickly let the encoder code take first crack at processing the interrupts.
        m_pEncoders->handleEncoderTick(&rise0, &fall0);

        // If there are still interrupts to handle then chain in the original GPIO interrupt handler.
        if (rise0 || fall0 || rise2 || fall2)
            m_originalHandler();
    }

    static EncoderBase* m_pEncoders;
    static void         (*m_originalHandler)(void);
};
EncoderBase* EncoderBase::m_pEncoders = NULL;
void (*EncoderBase::m_originalHandler)(void) = NULL;


struct EncoderCounts
{
    volatile int32_t encoder1Count;
    volatile int32_t encoder2Count;
};


template<PinName encoder1Pin1, PinName encoder1Pin2, PinName encoder2Pin1, PinName encoder2Pin2>
class Encoders : protected EncoderBase
{
public:
    Encoders()
    {
        // NOTE: Hard coded to only work with GPIO port 0 on the LPC1768.  The 2 pins for each encoder must
        //       be contiguous as well.
        assert( (encoder1Pin1 & ~0x1F) == LPC_GPIO0_BASE &&
                (encoder1Pin2 & ~0x1F) == LPC_GPIO0_BASE &&
                (encoder2Pin1 & ~0x1F) == LPC_GPIO0_BASE &&
                (encoder2Pin2 & ~0x1F) == LPC_GPIO0_BASE );
        assert( (encoder1Pin1 & 0x1F) - (encoder1Pin2 & 0x1F) == -1 &&
                (encoder2Pin1 & 0x1F) - (encoder2Pin2 & 0x1F) == -1 );

        // Set the pin mux to GPIO for these 4 encoder pins.
        pin_function(encoder1Pin1, 0);
        pin_function(encoder1Pin2, 0);
        pin_function(encoder2Pin1, 0);
        pin_function(encoder2Pin2, 0);

        // Disable the pull-ups on these pins since we already have external pull-ups on the hall effect sensors.
        pin_mode(encoder1Pin1, PullNone);
        pin_mode(encoder1Pin2, PullNone);
        pin_mode(encoder2Pin1, PullNone);
        pin_mode(encoder2Pin2, PullNone);

        // Calculate the mask for these 4 pins on port 0.
        uint32_t encoder1Mask = (1 << (encoder1Pin1 & 0x1F)) | (1 << (encoder1Pin2 & 0x1F));
        uint32_t encoder2Mask = (1 << (encoder2Pin1 & 0x1F)) | (1 << (encoder2Pin2 & 0x1F));
        uint32_t encoderMask = encoder1Mask | encoder2Mask;

        // Configure the 4 encoder pins as inputs.
        LPC_GPIO0->FIODIR &= ~encoderMask;

        // Record current encoder pin state.
        uint32_t port0 = LPC_GPIO0->FIOPIN;
        m_lastEncoder1 = (port0 >> (encoder1Pin1 & 0x1F)) & 0x3;
        m_lastEncoder2 = (port0 >> (encoder2Pin1 & 0x1F)) & 0x3;

        // Clear the encoder counters.
        m_encoderCounts.encoder1Count = 0;
        m_encoderCounts.encoder2Count = 0;

        // Disable GPIO ints.
        NVIC_DisableIRQ(EINT3_IRQn);

        // Configure all 4 pins to interrupt on rising and falling edges.
        LPC_GPIOINT->IO0IntEnR |= encoderMask;
        LPC_GPIOINT->IO0IntEnF |= encoderMask;

        // Record previous interrupt handler for GPIO ints.
        m_originalHandler = (void (*)())NVIC_GetVector(EINT3_IRQn);

        // Only support having one of these classes instantiated.
        assert( m_pEncoders == NULL );
        m_pEncoders = this;

        // Configure our interrupt handler for GPIO ints.
        NVIC_SetVector(EINT3_IRQn, (uint32_t)interruptHandler);
        NVIC_EnableIRQ(EINT3_IRQn);
    }

    EncoderCounts getAndClearEncoderCounts()
    {
        EncoderCounts encoderCounts = m_encoderCounts;
        m_encoderCounts.encoder1Count = 0;
        m_encoderCounts.encoder2Count = 0;
        return encoderCounts;
    }

protected:
    virtual void handleEncoderTick(uint32_t* pRise0, uint32_t* pFall0)
    {
        uint32_t rise0 = *pRise0;
        uint32_t fall0 = *pFall0;

        // Calculate the mask for these 4 pins on port 0.
        static const uint32_t encoder1Mask = (1 << (encoder1Pin1 & 0x1F)) | (1 << (encoder1Pin2 & 0x1F));
        static const uint32_t encoder2Mask = (1 << (encoder2Pin1 & 0x1F)) | (1 << (encoder2Pin2 & 0x1F));
        static const uint32_t encoderMask = encoder1Mask | encoder2Mask;

        // Check to see if any of the encoder signals have changed.
        if ((rise0 & encoderMask) || (fall0 & encoderMask))
        {
            // Fetch the current state of the encoder pins.
            uint32_t port0 = LPC_GPIO0->FIOPIN;
            uint32_t encoder1 = (port0 >> (encoder1Pin1 & 0x1F)) & 0x3;
            uint32_t encoder2 = (port0 >> (encoder2Pin1 & 0x1F)) & 0x3;

            // Look up delta based on current and last state.
            static const int32_t stateTable[4][4] = { { 0, -1,  1,  0},
                                                      { 1,  0,  0, -1},
                                                      {-1,  0,  0,  1},
                                                      { 0,  1, -1,  0} };
            int32_t encoder1Delta = stateTable[m_lastEncoder1][encoder1];
            int32_t encoder2Delta = stateTable[m_lastEncoder2][encoder2];
            interlockedAdd(&m_encoderCounts.encoder1Count, encoder1Delta);
            interlockedAdd(&m_encoderCounts.encoder2Count, encoder2Delta);

            // Record current encoder state.
            m_lastEncoder1 = encoder1;
            m_lastEncoder2 = encoder2;

            // Clear out these GPIO interrupts.
            LPC_GPIOINT->IO0IntClr = encoderMask;
            rise0 &= ~encoderMask;
            fall0 &= ~encoderMask;
        }

        *pRise0 = rise0;
        *pFall0 = fall0;
    }

    uint32_t      m_lastEncoder1;
    uint32_t      m_lastEncoder2;
    EncoderCounts m_encoderCounts;
};

#endif // _ENCODERS_H_
