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
/* Class used to capture PWM pulses from things like a RC radio receiver. This class requires that there are regular
   pulses.  A 0% duty cycle PWM will be treated as a timeout. */
#ifndef PWM_IN_H_
#define PWM_IN_H_

#include <mbed.h>

class PwmIn
{
public:
    PwmIn(PinName pwmPin, uint32_t usecTimeOut) :
        m_interruptIn(pwmPin)
    {
        m_highTime = 0;
        m_lowTime = 0;
        m_usecTimeOut = usecTimeOut;
        m_dutyCycle = 0.0f;
        m_hasTimedOut = true;
        m_timer.start();
        m_interruptIn.rise(this, &PwmIn::risingEdgeHandler);
        m_interruptIn.fall(this, &PwmIn::fallingEdgeHandler);
    }

    float getDutyCycle()
    {
        return m_dutyCycle;
    }

    bool hasTimedOut()
    {
        // Only bother to check for time-out if a timeout hasn't already been detected.
        // The timeout will be cleared the next time a pulse is detected.
        if (!m_hasTimedOut)
            m_hasTimedOut = (uint32_t)m_timer.read_us() > m_usecTimeOut;
        return m_hasTimedOut;
    }

protected:
    void risingEdgeHandler()
    {
        m_lowTime = m_timer.read_us();
        m_timer.reset();
        if (m_lowTime > 0 && m_highTime > 0 && m_lowTime + m_highTime < m_usecTimeOut)
        {
            m_dutyCycle = (float)m_highTime / (float)(m_highTime + m_lowTime);
            m_hasTimedOut = false;
        }
    }

    void fallingEdgeHandler()
    {
        m_highTime = m_timer.read_us();
        m_timer.reset();
    }

    InterruptIn m_interruptIn;
    Timer       m_timer;
    uint32_t    m_highTime;
    uint32_t    m_lowTime;
    uint32_t    m_usecTimeOut;
    float       m_dutyCycle;
    bool        m_hasTimedOut;
};

#endif // PWM_IN_H_
