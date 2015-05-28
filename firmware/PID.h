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
/* Class to abstract PID (proportional, integral, derivative) algorithm.
   It is inspired by Brett Beauregard's Arduino PID Library (https://github.com/br3ttb/Arduino-PID-Library/) and
   the PID e-book available on controlguru (http://www.controlguru.com).
*/
#ifndef PID_H_
#define PID_H_

#include <assert.h>
#include <mbed.h>

class PID
{
public:
    PID(float Kc, float Ti, float Td, float controlOutputBias, float controlMin, float controlMax, float sampleTime)
    {
        m_sampleTime = sampleTime;
        setTuningParameters(Kc, Ti, Td);
        m_isAuto = false;
        m_controlOutputBias = controlOutputBias;
        m_controlMin = controlMin;
        m_controlMax = controlMax;
        m_controlOutput = controlOutputBias;
        m_setPoint = 0.0f;
        m_integral = 0.0f;
        m_processLast = 0.0f;
    }

    void updateSetPoint(float setPoint)
    {
        m_setPoint = setPoint;
    }

    float getSetPoint()
    {
        return m_setPoint;
    }

    float compute(float processCurrent)
    {
        if (m_isAuto)
        {
            float error = m_setPoint - processCurrent;
            float processDelta = processCurrent - m_processLast;
            float integral = m_integral + m_Ki * error;
            float controlOutput;

            controlOutput = m_controlOutputBias + m_Kp * error + integral - m_Kd * processDelta;
            if (controlOutput < m_controlMin)
            {
                integral += (m_controlMin - controlOutput);
                controlOutput = m_controlMin;
            }
            else if (controlOutput > m_controlMax)
            {
                integral -= (controlOutput - m_controlMax);
                controlOutput = m_controlMax;
            }
            m_integral = integral;
            m_controlOutput = controlOutput;
        }

        m_processLast = processCurrent;
        return m_controlOutput;
    }

    float getControlOutput()
    {
        return m_controlOutput;
    }

    void setOutputManually(float controlOutput)
    {
        assert( controlOutput >= m_controlMin && controlOutput <= m_controlMax );
        m_isAuto = false;
        m_controlOutput = controlOutput;
    }

    void enableAutomaticMode()
    {
        if (m_isAuto)
            return;
        m_controlOutputBias = m_controlOutput;
        m_setPoint = m_processLast;
        m_integral = 0.0f;
        m_isAuto = true;
    }

    void setTuningParameters(float Kc, float Ti, float Td)
    {
        assert( Ti >= 0.0f && Td >= 0.0f && sampleTime > 0.0f );
        m_Kp = Kc;
        m_Ki = (Ti == 0.0f) ? 0.0f : (Kc / Ti) * m_sampleTime;
        m_Kd = (Td == 0.0f) ? 0.0f : (Kc / Td) / m_sampleTime;
    }

protected:
    bool    m_isAuto;
    float   m_Kp;
    float   m_Ki;
    float   m_Kd;
    float   m_controlOutputBias;
    float   m_controlMin;
    float   m_controlMax;
    float   m_sampleTime;
    float   m_controlOutput;
    float   m_setPoint;
    float   m_integral;
    float   m_processLast;
};

#endif // PID_H_
