/*  Copyright (C) 2014  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
/* Hexadecimal to/from text conversion helpers. */
#ifndef _HEX_CONVERT_H_
#define _HEX_CONVERT_H_

#include "try_catch.h"

#define EXTRACT_HI_NIBBLE(X) (((X) >> 4) & 0xF)
#define EXTRACT_LO_NIBBLE(X) ((X) & 0xF)

static const char NibbleToHexChar[16] = { '0', '1', '2', '3', '4', '5', '6', '7',
                                          '8', '9', 'a', 'b', 'c', 'd', 'e', 'f' };

static inline int HexCharToNibble(unsigned char HexChar)
{
    if (HexChar >= 'a' && HexChar <= 'f')
    {
        return HexChar - 'a' + 10;
    }
    if (HexChar >= 'A' && HexChar <= 'F')
    {
        return HexChar - 'A' + 10;
    }
    if (HexChar >= '0' && HexChar <= '9')
    {
        return HexChar - '0';
    }

    __throw(invalidHexDigitException);
}

#endif /* _HEX_CONVERT_H_ */
