__stack_size = 0x2000;
__heap_size  = 0x200000;
__total_stack_size = __stack_size * 16;

MEMORY
{
    code_ram   (rx!rw) : ORIGIN = 0x80001000, LENGTH = 0x40000
    data_ram   (rw!x)  : ORIGIN = 0x80041000, LENGTH = 0x300000
}

ENTRY(crt0)

SECTIONS
{
    .text :
    {
        *(.text*)
    } > code_ram

    .data :
    {
        *(.data)
        *(.bss)
        *(.rodata*)
    } > data_ram

    .heap : ALIGN(0x10)
    {
        __heap_start = .;
        . += __heap_size;
    } > data_ram

    .stack0 : ALIGN(0x10)
    {
        . += __total_stack_size;
        . = ALIGN(0x10);
        __stack_top = .;
    } > data_ram
}

