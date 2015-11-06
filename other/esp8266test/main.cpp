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
/* Simple program to test my ESP8266 WiFi chip setup by testing MRI
   interaction with esp-link.
*/
#include <mbed.h>

int main()
{
    DigitalOut led(LED1);

    for (unsigned int i = 0; ; i++)
    {
        led = 1;
        wait(0.5f);
        led = 0;
        wait(0.5f);
    }

    return -1;
}
