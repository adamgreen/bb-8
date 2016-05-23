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
        m_lastEdgeTime = 0;
        m_highTime = 0;
        m_lowTime = 0;
        m_usecTimeOut = usecTimeOut;
        m_dutyCycle = 0.0f;
        m_hasTimedOut = true;
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
        {
            uint32_t elapsedTime = elapsedTimeSinceLastEdge();
            bool hasTimedOut = elapsedTime > m_usecTimeOut;
            if (hasTimedOut)
            {
                m_lowTime = 0;
                m_highTime = 0;
                // UNDONE: Should remove this debug printf().
                printf("%lu\n", elapsedTime);
            }
            m_hasTimedOut = hasTimedOut;
        }
        return m_hasTimedOut;
    }

protected:
    void risingEdgeHandler()
    {
        uint32_t currentTime;
        m_lowTime = elapsedTimeSinceLastEdge(&currentTime);
        m_lastEdgeTime = currentTime;
        if (m_lowTime > 0 && m_highTime > 0 && m_lowTime + m_highTime < m_usecTimeOut)
        {
            m_dutyCycle = (float)m_highTime / (float)(m_highTime + m_lowTime);
            m_hasTimedOut = false;
        }
    }

    void fallingEdgeHandler()
    {
        uint32_t currentTime;
        m_highTime = elapsedTimeSinceLastEdge(&currentTime);
        m_lastEdgeTime = currentTime;
    }

    uint32_t elapsedTimeSinceLastEdge(uint32_t* pCurrentTime = NULL)
    {
        uint32_t lastEdgeTime;
        uint32_t elapsedTime;
        uint32_t currentTime;

        // This function handles the case where m_lastEdgeTime is updated while we are attempting to calculate the
        // elapsed time.
        do
        {
            lastEdgeTime = m_lastEdgeTime;
            currentTime = us_ticker_read();
            elapsedTime = currentTime - lastEdgeTime;
        } while (lastEdgeTime != m_lastEdgeTime);

        if (pCurrentTime)
            *pCurrentTime = currentTime;

        return elapsedTime;
    }

    InterruptIn         m_interruptIn;
    volatile uint32_t   m_lastEdgeTime;
    uint32_t            m_highTime;
    uint32_t            m_lowTime;
    uint32_t            m_usecTimeOut;
    volatile float      m_dutyCycle;
    volatile bool       m_hasTimedOut;
};

#endif // PWM_IN_H_
