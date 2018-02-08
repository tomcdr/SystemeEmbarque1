MEMORY
{
    sram (rwx) : ORIGIN = 0x10000000, LENGTH = 4k
    flash (rx) : ORIGIN = 0x00000000, LENGTH = 32k
}

_sram_size = LENGTH(sram);
_sram_base = ORIGIN(sram);
_end_stack = (_sram_base + _sram_size);

SECTIONS {
. = ORIGIN(flash);
.text :
    {
        KEEP(*(.vectors))
    *(.text*)
    *(.rodata*)
    . = ALIGN(4);
        _end_text = .;
    } >flash

        . = ALIGN(4);

.data :
    {
        _start_data = .;
    *(.data*)
        _end_data = .;
    } >sram AT >flash

        . = ALIGN(4);

.bss :
    {
        _start_bss = .;
    *(.bss*)
    *(COMMON)
        _end_bss = .;
    } >sram

        . = ALIGN(4);

}
ENTRY(Reset_Handler)


