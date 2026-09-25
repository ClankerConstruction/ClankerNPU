/*
 * AN75XX NPU firmware linker script
 *
 * Memory layout:
 *   .text + .rodata  → DRAM at 0x84000000  (becomes npu_rv32.bin)
 *   .data            → NPU SRAM at 0x3E900000  (becomes npu_data.bin)
 *   .bss             → NPU SRAM after .data (zeroed by crt0, not in any file)
 *   stacks           → DRAM after .text (not in any file, 16KB per hart)
 */

OUTPUT_ARCH(riscv)
ENTRY(_start)

MEMORY
{
    DRAM (rwx)  : ORIGIN = 0x84000000, LENGTH = 2M
    SRAM (rw)   : ORIGIN = 0x3E900000, LENGTH = 64K
}

SECTIONS
{
    .text : {
        *(.text.init)
        *(.text .text.*)
    } > DRAM

    .rodata : ALIGN(4) {
        *(.rodata .rodata.*)
        *(.srodata .srodata.*)
    } > DRAM

    __image_end = .;

    .data : ALIGN(4) {
        __data_start = .;
        *(.data .data.*)
        *(.sdata .sdata.*)
        __data_end = .;
    } > SRAM

    __global_pointer$ = __data_start + 0x800;

    .bss (NOLOAD) : ALIGN(4) {
        __bss_start = .;
        *(.bss .bss.*)
        *(.sbss .sbss.*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end = . - 4;
    } > SRAM

    /* the debug block sits at a fixed address above the globals */
    ASSERT(__bss_end < __ndbg_base, "globals overlap the debug block")

    .stack (NOLOAD) : ALIGN(16) {
        __stack_base = .;
    } > DRAM

    /DISCARD/ : {
        *(.eh_frame)
        *(.comment)
        *(.note.*)
    }
}

__STACK_SIZE = 0x4000;
__stack0 = __stack_base + 1 * __STACK_SIZE;
__stack1 = __stack_base + 2 * __STACK_SIZE;
__stack2 = __stack_base + 3 * __STACK_SIZE;
__stack3 = __stack_base + 4 * __STACK_SIZE;
__stack4 = __stack_base + 5 * __STACK_SIZE;
__stack5 = __stack_base + 6 * __STACK_SIZE;
__stack6 = __stack_base + 7 * __STACK_SIZE;
__stack7 = __stack_base + 8 * __STACK_SIZE;
