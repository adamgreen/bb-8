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
/* Class used to interface with the TB6612FNG dual channel motor controller. */
#ifndef MOTOR_H_
#define MOTOR_H_

#include <mbed.h>

class MotorChannel
{
public:
    MotorChannel(PinName pwmPin,
                 PinName input1Pin,
                 PinName input2Pin,
                 float maxPower = 1.0f,
                 float period = 1.0f / 10000.0f) :
        m_pwm(pwmPin), m_input(input1Pin, input2Pin)
    {
        m_maxPower = maxPower;
        setPeriod(period);
        set(0.0f);
    }

    void setPeriod(float period)
    {
        m_pwm.period(period);
    }

    void set(float value)
    {
        m_value = value;
        m_input.write(motorInputFromValue(value));

        float power = fabs(value);
        if (power > m_maxPower)
            power = m_maxPower;
        m_pwm.write(power);
    }

    float get()
    {
        return m_value;
    }

protected:
    static uint32_t motorInputFromValue(float value)
    {
        // Motor direction input is dependent on set value.
        if (value == 0.0f)
        {
            // Brake to ground.
            return 3;
        }
        else if (value > 0.0f)
        {
            // Forward direction.
            return 2;
        }
        else
        {
            // Reverse direction.
            return 1;
        }
    }

    PwmOut m_pwm;
    BusOut m_input;
    float  m_value;
    float  m_maxPower;
};


class Motor
{
public:
    Motor(PinName channelAPwmPin, PinName channelAInput1Pin, PinName channelAInput2Pin,
          PinName channelBPwmPin, PinName channelBInput1Pin, PinName channelBInput2Pin,
          PinName standbyPin,
          float channelAMax = 1.0f, float channelBMax = 1.0f, float period = 1.0f / 10000.0f) :
        m_channelA(channelAPwmPin, channelAInput1Pin, channelAInput2Pin, channelAMax),
        m_channelB(channelBPwmPin, channelBInput1Pin, channelBInput2Pin, channelBMax),
        m_standby(standbyPin)
    {
        setPeriod(period);
        enable();
    }

    void setPeriod(float period)
    {
        // On LPC1768 we only need to set PWM period on one channel since all channels share the same period.
        m_period = period;
        m_channelA.setPeriod(period);
    }

    float getPeriod()
    {
        return m_period;
    }

    void set(float channelA, float channelB)
    {
        m_channelA.set(channelA);
        m_channelB.set(channelB);
    }

    float getChannelA()
    {
        return m_channelA.get();
    }

    float getChannelB()
    {
        return m_channelB.get();
    }

    void enable()
    {
        m_standby.write(1);
    }

    void disable()
    {
        m_standby.write(0);
    }

protected:
    MotorChannel m_channelA;
    MotorChannel m_channelB;
    DigitalOut   m_standby;
    float        m_period;
};

#endif // MOTOR_H_
