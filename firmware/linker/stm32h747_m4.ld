/* Linker script: STM32H747 M4 core
 * FLASH: 0x08100000 1024K  (M4 dedicated flash bank)
 * RAM:   SRAM2 0x10000000 288K (D2 domain)
 * Stack: at top of SRAM2
 */

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08100000, LENGTH = 1024K
  SRAM2(rwx)  : ORIGIN = 0x10000000, LENGTH = 288K
}

_estack = ORIGIN(SRAM2) + LENGTH(SRAM2);

SECTIONS
{
  .isr_vector : { KEEP(*(.isr_vector)) } >FLASH

  .text : {
    *(.text*) *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  .ARM.extab : { *(.ARM.extab*) } >FLASH
  .ARM.exidx : { *(.ARM.exidx*) } >FLASH

  _sidata = LOADADDR(.data);

  .data : {
    _sdata = .;
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >SRAM2 AT>FLASH

  .bss : {
    _sbss = .;
    *(.bss*) *(COMMON)
    . = ALIGN(4);
    _ebss = .;
  } >SRAM2
}
