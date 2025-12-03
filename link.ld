/*
 * Linker script for ATSAMS70Q21 - BJT60 Presence Detection Firmware
 * Flash: 2MB @ 0x00400000
 * RAM: 384KB @ 0x20400000
 */

OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)

/* Memory layout */
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x00400000, LENGTH = 0x00200000  /* 2MB */
    RAM (rwx)   : ORIGIN = 0x20400000, LENGTH = 0x00060000  /* 384KB */
}

/* Stack and heap sizes */
STACK_SIZE = 0x2000;  /* 8KB stack */
HEAP_SIZE  = 0x4000;  /* 16KB heap */

/* Sections */
SECTIONS
{
    /* Code and read-only data in FLASH */
    .text :
    {
        . = ALIGN(4);
        _stext = .;
        KEEP(*(.vectors))           /* Vector table must be first */
        *(.text*)                   /* Program code */
        *(.rodata*)                 /* Read-only data */

        . = ALIGN(4);
        _etext = .;
    } > FLASH

    /* Initialized data (copied from FLASH to RAM at startup) */
    .data : AT (_etext)
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data*)
        *(.ramfunc*)                /* Functions to run from RAM */
        . = ALIGN(4);
        _edata = .;
    } > RAM

    /* Uninitialized data (zero-initialized at startup) */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > RAM

    /* Heap */
    .heap (NOLOAD) :
    {
        . = ALIGN(8);
        _sheap = .;
        . = . + HEAP_SIZE;
        _eheap = .;
    } > RAM

    /* Stack (grows downward) */
    .stack (NOLOAD) :
    {
        . = ALIGN(8);
        _sstack = .;
        . = . + STACK_SIZE;
        . = ALIGN(8);
        _estack = .;
    } > RAM

    /* End of RAM */
    _ram_end = ORIGIN(RAM) + LENGTH(RAM);
}
