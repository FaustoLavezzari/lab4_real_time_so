MEMORY
{
    FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 64K
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 8K
}

SECTIONS
{
    .text :
    {
        KEEP(*(.isr_vector))
        *(.text*)
        *(.rodata*)
        _etext = .;
    } > FLASH

    .data : AT (ADDR(.text) + SIZEOF(.text))
    {
        _data = .;
        *(vtable)
        *(.data)
        _edata = .;
    } > SRAM

    .stack :
    {
        _stack = .;
        *(.stack)
        _estack = .;
        . = ALIGN(4);
    } > SRAM

    .bss :
    {
        _bss = .;
        *(.bss)
        *(COMMON)
        _ebss = .;
        . = ALIGN(4);
    } > SRAM
}
