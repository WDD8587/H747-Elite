/* Linker script: STM32H747 M7 core
 * FLASH: 0x08000000 1024K  (application, bootloader at bottom)
 * RAM:   DTCM 0x20000000 128K + AXI SRAM 0x24000000 512K
 * Stack: grows downward from top of DTCM
 * Heap:  SRAM3 0x38000000 64K (shared with M4)
 */

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 1024K
  DTCM (rwx)  : ORIGIN = 0x20000000, LENGTH = 128K
  AXI  (rwx)  : ORIGIN = 0x24000000, LENGTH = 512K
  SRAM3(rwx)  : ORIGIN = 0x38000000, LENGTH = 64K
}

_estack = ORIGIN(DTCM) + LENGTH(DTCM);

SECTIONS
{
  .isr_vector : { KEEP(*(.isr_vector)) } >FLASH

  .text : {
    *(.text*) *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  .ARM.extab : { *(.ARM.extab*) } >FLASH
  .ARM.exidx : { *(.ARM.exidx*) } >FLASH

  .copy.table : { __copy_table_start__ = .; KEEP(*(.copy.table)); __copy_table_end__ = .; } >FLASH
  .zero.table : { __zero_table_start__ = .; KEEP(*(.zero.table)); __zero_table_end__ = .; } >FLASH

  _sidata = LOADADDR(.data);

  .data : {
    _sdata = .;
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >AXI AT>FLASH

  .bss : {
    _sbss = .;
    *(.bss*) *(COMMON)
    . = ALIGN(4);
    _ebss = .;
  } >AXI

  _sram2_data_start  = _sdata;
  _sram2_data_end    = _edata;
  _sram2_bss_start   = _sbss;
  _sram2_bss_end     = _ebss;

  _sram3_heap_start = ORIGIN(SRAM3);
  _sram3_heap_end   = ORIGIN(SRAM3) + LENGTH(SRAM3);
}
