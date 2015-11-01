/* Linker script for mbed LPC1768 */

/* Linker script to configure memory regions. */
MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 512K
  RAM(rwx) : ORIGIN = 0x2007C000, LENGTH = 16K
}

/* Linker script to place sections and symbol values. Should be used together
 * with other linker script that defines memory regions FLASH and RAM.
 * It references following symbols, which must be defined in code:
 *   Reset_Handler : Entry of reset handler
 * 
 * It defines following symbols, which code can use without definition:
 *   __exidx_start
 *   __exidx_end
 *   __etext
 *   __data_start__
 *   __preinit_array_start
 *   __preinit_array_end
 *   __init_array_start
 *   __init_array_end
 *   __fini_array_start
 *   __fini_array_end
 *   __data_end__
 *   __bss_start__
 *   __bss_end__
 *   __end__
 *   end
 *   __HeapLimit
 *   __StackLimit
 *   __StackTop
 *   __stack
 */
ENTRY(main)

SECTIONS
{
    .text :
    {
        KEEP(*(.text.startup.main))
        *(.text*)

        *(.init)
        *(.fini)

        /* .ctors */
        *crtbegin.o(.ctors)
        *crtbegin?.o(.ctors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
        *(SORT(.ctors.*))
        *(.ctors)

        /* .dtors */
        *crtbegin.o(.dtors)
        *crtbegin?.o(.dtors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
        *(SORT(.dtors.*))
        *(.dtors)

        *(.rodata*)

        *(.eh_frame*)
    } > RAM

    .ARM.extab : 
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > RAM

    __exidx_start = .;
    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > RAM
    __exidx_end = .;

    __etext = .;
        
    .data : AT (__etext)
    {
        __data_start__ = .;
        Image$$RW_IRAM1$$Base = .;
        *(vtable)
        *(.data*)

        . = ALIGN(4);
        /* preinit data */
        PROVIDE (__preinit_array_start = .);
        *(.preinit_array)
        PROVIDE (__preinit_array_end = .);

        . = ALIGN(4);
        /* init data */
        PROVIDE (__init_array_start = .);
        *(SORT(.init_array.*))
        *(.init_array)
        PROVIDE (__init_array_end = .);


        . = ALIGN(4);
        /* finit data */
        PROVIDE (__fini_array_start = .);
        *(SORT(.fini_array.*))
        *(.fini_array)
        PROVIDE (__fini_array_end = .);

        . = ALIGN(4);
        /* All data end */
        __data_end__ = .;

    } > RAM

    
    .bss :
    {
        __bss_start__ = .;
        *(.bss*)
        *(COMMON)
        __bss_end__ = .;
        Image$$RW_IRAM1$$ZI$$Limit = . ;
    } > RAM

    
    .heap :
    {
        __end__ = .;
        end = __end__;
        *(.heap*)
        __HeapLimit = .;
    } > RAM

    /* .stack_dummy section doesn't contains any symbols. It is only
     * used for linker to calculate size of stack sections, and assign
     * values to stack symbols later */
    .stack_dummy :
    {
        *(.stack)
    } > RAM

    /* Set stack top to end of RAM, and stack limit move down by
     * size of stack_dummy section */
    __StackTop = ORIGIN(RAM) + LENGTH(RAM);
    __StackLimit = __StackTop - SIZEOF(.stack_dummy);
    PROVIDE(__stack = __StackTop);
    
    /* Check if data + heap + stack exceeds RAM limit */
    ASSERT(__StackLimit >= __HeapLimit, "region RAM overflowed with stack")
}
