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
/* Implementation of simple interlockeded operations for ARMv7-M based devices. */

    .text
    .code 16
    .syntax unified

    .global interlockedIncrement
    .type interlockedIncrement, function
    /* uint32_t interlockedIncrement(uint32_t* pValue); */
interlockedIncrement:
    ldrex   r1, [r0, #0]
    adds    r1, r1, #1
    strex   r2, r1, [r0, #0]
    cmp     r2, #0
    bne     interlockedIncrement
    mov     r0, r1
    bx      lr


    .global interlockedDecrement
    .type interlockedDecrement, function
    /* uint32_t interlockedDecrement(uint32_t* pValue); */
interlockedDecrement:
    ldrex   r1, [r0, #0]
    subs    r1, r1, #1
    strex   r2, r1, [r0, #0]
    cmp     r2, #0
    bne     interlockedDecrement
    mov     r0, r1
    bx      lr


    .global interlockedAdd
    .type interlockedAdd, function
    /* uint32_t interlockedAdd(uint32_t* pVal1, uint32_t val2); */
interlockedAdd:
    ldrex   r2, [r0, #0]
    adds    r2, r2, r1
    strex   r3, r2, [r0, #0]
    cmp     r3, #0
    bne     interlockedAdd
    mov     r0, r2
    bx      lr


    .global interlockedSubtract
    .type interlockedSubtract, function
    /* uint32_t interlockedSubtract(uint32_t* pVal1, uint32_t val2); */
interlockedSubtract:
    ldrex   r2, [r0, #0]
    subs    r2, r2, r1
    strex   r3, r2, [r0, #0]
    cmp     r3, #0
    bne     interlockedSubtract
    mov     r0, r2
    bx      lr
