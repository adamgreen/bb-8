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
/* Simple program to test my ESP8266 WiFi chip setup.  It just acts
   as a USB to serial adapter.
*/
#include <mbed.h>

int main()
{
    Serial     pc(USBTX, USBRX);
    Serial     esp(p28, p27);
    Timer      rxTimer;
    Timer      txTimer;
    Timer      blinkTimer;
    DigitalOut runningLED(LED1);
    DigitalOut rxLED(LED3);
    DigitalOut txLED(LED4);

    static const int baudRate = 230400;
    pc.baud(baudRate);
    esp.baud(baudRate);

    runningLED = 1;
    rxLED = 0;
    txLED = 0;

    rxTimer.start();
    txTimer.start();
    blinkTimer.start();

    while (1)
    {
        if (blinkTimer.read_ms() >= 1000)
        {
            runningLED = !runningLED;
            blinkTimer.reset();
        }
        if (rxLED && rxTimer.read_ms() >= 250)
        {
            rxLED = 0;
        }
        if (txLED && txTimer.read_ms() >= 250)
        {
            txLED = 0;
        }

        if (pc.readable())
        {
            esp.putc(pc.getc());
            rxLED = 1;
            rxTimer.reset();
        }
        if (esp.readable())
        {
            pc.putc(esp.getc());
            txLED = 1;
            txTimer.reset();
        }
    }

    return -1;
}
