/**
 * @file    startup_stm32h747xx.s
 * @brief   Minimal startup assembly for STM32H747 M7 core.
 *
 * Features:
 *   - Vector table with all ISR entries (weak aliases to Default_Handler)
 *   - Stack pointer initialisation from linker script
 *   - Call SystemInit -> __libc_init_array -> main
 *   - D2 domain RAM initialisation
 *   - M4 boot control: set M4 vector table, release M4 reset
 *
 * Syntax: GNU Assembler (gas) for ARM Cortex-M7
 */

.syntax     unified
.arch       armv7e-m
.fpu        fpv5-d16
.thumb

.section    .isr_vector, "a", %progbits
.type       g_pfnVectors, %object
.size       g_pfnVectors, .-g_pfnVectors

/* External declarations */
.globl     _estack
.globl     _sidata
.globl     _sdata
.globl     _edata
.globl     _sbss
.globl     _ebss
.globl     _sram2_data_start
.globl     _sram2_data_end
.globl     _sram2_bss_start
.globl     _sram2_bss_end

.globl     SystemInit
.globl     main
.globl     __libc_init_array

/* Weak default handler stub */
.weak      Default_Handler
.type      Default_Handler, %function
Default_Handler:
    b       Default_Handler
.size      Default_Handler, .-Default_Handler

/* Macro for weak ISR aliases */
.macro  def_irq  name
    .weak   \name
    .thumb_set \name, Default_Handler
.endm

/*****************************************************************************
 * Vector table
 *****************************************************************************/
g_pfnVectors:
    .word   _estack                         /* 0:  Stack top          */
    .word   Reset_Handler                   /* 1:  Reset Handler      */
    .word   NMI_Handler                     /* 2:  NMI                */
    .word   HardFault_Handler               /* 3:  Hard Fault         */
    .word   MemManage_Handler               /* 4:  MemManage          */
    .word   BusFault_Handler                /* 5:  Bus Fault          */
    .word   UsageFault_Handler              /* 6:  Usage Fault        */
    .word   0                               /* 7:  Reserved           */
    .word   0                               /* 8:  Reserved           */
    .word   0                               /* 9:  Reserved           */
    .word   0                               /* 10: Reserved           */
    .word   SVC_Handler                     /* 11: SVCall             */
    .word   DebugMon_Handler                /* 12: Debug Monitor      */
    .word   0                               /* 13: Reserved           */
    .word   PendSV_Handler                  /* 14: PendSV             */
    .word   SysTick_Handler                 /* 15: SysTick            */

    /* External Interrupts */
    .word   WWDG_IRQHandler                 /* 16: Window Watchdog           */
    .word   PVD_PVM_IRQHandler              /* 17: PVD/PVM                   */
    .word   TAMP_STAMP_IRQHandler           /* 18: Tamper/TimeStamp          */
    .word   RTC_WKUP_IRQHandler             /* 19: RTC Wakeup                */
    .word   FLASH_IRQHandler                /* 20: Flash                     */
    .word   RCC_IRQHandler                  /* 21: RCC                       */
    .word   EXTI0_IRQHandler                /* 22: EXTI Line 0               */
    .word   EXTI1_IRQHandler                /* 23: EXTI Line 1               */
    .word   EXTI2_IRQHandler                /* 24: EXTI Line 2               */
    .word   EXTI3_IRQHandler                /* 25: EXTI Line 3               */
    .word   EXTI4_IRQHandler                /* 26: EXTI Line 4               */
    .word   DMA1_Stream0_IRQHandler         /* 27: DMA1 Stream 0             */
    .word   DMA1_Stream1_IRQHandler         /* 28: DMA1 Stream 1             */
    .word   DMA1_Stream2_IRQHandler         /* 29: DMA1 Stream 2             */
    .word   DMA1_Stream3_IRQHandler         /* 30: DMA1 Stream 3             */
    .word   DMA1_Stream4_IRQHandler         /* 31: DMA1 Stream 4             */
    .word   DMA1_Stream5_IRQHandler         /* 32: DMA1 Stream 5             */
    .word   DMA1_Stream6_IRQHandler         /* 33: DMA1 Stream 6             */
    .word   ADC_IRQHandler                  /* 34: ADC1/2                   */
    .word   FDCAN1_IRQHandler               /* 35: FDCAN1                   */
    .word   FDCAN2_IRQHandler               /* 36: FDCAN2                   */
    .word   TIM1_BRK_IRQHandler             /* 37: TIM1 Break               */
    .word   TIM1_UP_IRQHandler              /* 38: TIM1 Update              */
    .word   TIM1_TRG_COM_IRQHandler         /* 39: TIM1 Trigger/Commutation */
    .word   TIM1_CC_IRQHandler              /* 40: TIM1 Capture Compare     */
    .word   TIM2_IRQHandler                 /* 41: TIM2                     */
    .word   TIM3_IRQHandler                 /* 42: TIM3                     */
    .word   TIM4_IRQHandler                 /* 43: TIM4                     */
    .word   I2C1_EV_IRQHandler              /* 44: I2C1 Event               */
    .word   I2C1_ER_IRQHandler              /* 45: I2C1 Error               */
    .word   I2C2_EV_IRQHandler              /* 46: I2C2 Event               */
    .word   I2C2_ER_IRQHandler              /* 47: I2C2 Error               */
    .word   SPI1_IRQHandler                 /* 48: SPI1                     */
    .word   SPI2_IRQHandler                 /* 49: SPI2                     */
    .word   USART1_IRQHandler               /* 50: USART1                   */
    .word   USART2_IRQHandler               /* 51: USART2                   */
    .word   USART3_IRQHandler               /* 52: USART3                   */
    .word   EXTI9_5_IRQHandler              /* 53: EXTI 9..5               */
    .word   TIM15_IRQHandler                /* 54: TIM15                    */
    .word   TIM16_IRQHandler                /* 55: TIM16                    */
    .word   TIM17_IRQHandler                /* 56: TIM17                    */
    .word   DMA1_Stream7_IRQHandler         /* 57: DMA1 Stream 7            */
    .word   HRTIM1_Master_IRQHandler        /* 58: HRTIM1 Master            */
    .word   HRTIM1_TIMA_IRQHandler          /* 59: HRTIM1 TIMA              */
    .word   HRTIM1_TIMB_IRQHandler          /* 60: HRTIM1 TIMB              */
    .word   HRTIM1_TIMC_IRQHandler          /* 61: HRTIM1 TIMC              */
    .word   HRTIM1_TIMD_IRQHandler          /* 62: HRTIM1 TIMD              */
    .word   HRTIM1_TIME_IRQHandler          /* 63: HRTIM1 TIME              */
    .word   HRTIM1_FLT_IRQHandler           /* 64: HRTIM1 Fault             */
    .word   SAI1_IRQHandler                 /* 65: SAI1                     */
    .word   SAI2_IRQHandler                 /* 66: SAI2                     */
    .word   QUADSPI_IRQHandler              /* 67: QuadSPI                  */
    .word   LPTIM1_IRQHandler               /* 68: LPTIM1                   */
    .word   CEC_IRQHandler                  /* 69: CEC                      */
    .word   I2C3_EV_IRQHandler              /* 70: I2C3 Event               */
    .word   I2C3_ER_IRQHandler              /* 71: I2C3 Error               */
    .word   SAI3_IRQHandler                 /* 72: SAI3                     */
    .word   UART4_IRQHandler                /* 73: UART4                    */
    .word   UART5_IRQHandler                /* 74: UART5                    */
    .word   UART7_IRQHandler                /* 75: UART7                    */
    .word   UART8_IRQHandler                /* 76: UART8                    */
    .word   DMA2_Stream0_IRQHandler         /* 77: DMA2 Stream 0            */
    .word   DMA2_Stream1_IRQHandler         /* 78: DMA2 Stream 1            */
    .word   DMA2_Stream2_IRQHandler         /* 79: DMA2 Stream 2            */
    .word   DMA2_Stream3_IRQHandler         /* 80: DMA2 Stream 3            */
    .word   DMA2_Stream4_IRQHandler         /* 81: DMA2 Stream 4            */
    .word   DMA2_Stream5_IRQHandler         /* 82: DMA2 Stream 5            */
    .word   DMA2_Stream6_IRQHandler         /* 83: DMA2 Stream 6            */
    .word   DMA2_Stream7_IRQHandler         /* 84: DMA2 Stream 7            */
    .word   SDMMC1_IRQHandler               /* 85: SDMMC1                   */
    .word   SDMMC2_IRQHandler               /* 86: SDMMC2                   */
    .word   FDCAN3_IRQHandler               /* 87: FDCAN3                   */
    .word   OCTOSPI1_IRQHandler             /* 88: OCTOSPI1                 */
    .word   OCTOSPI2_IRQHandler             /* 89: OCTOSPI2                 */
    .word   I2C4_EV_IRQHandler              /* 90: I2C4 Event               */
    .word   I2C4_ER_IRQHandler              /* 91: I2C4 Error               */
    .word   LTDC_IRQHandler                 /* 92: LTDC                     */
    .word   JPEG_IRQHandler                 /* 93: JPEG                     */
    .word   FMAC_IRQHandler                 /* 94: FMAC                     */
    .word   CORDIC_IRQHandler               /* 95: CORDIC                   */

/* Vector table size: 96 entries (0-95) */

/*****************************************************************************
 * Weak alias definitions for all interrupt handlers
 *****************************************************************************/
def_irq  NMI_Handler
def_irq  HardFault_Handler
def_irq  MemManage_Handler
def_irq  BusFault_Handler
def_irq  UsageFault_Handler
def_irq  SVC_Handler
def_irq  DebugMon_Handler
def_irq  PendSV_Handler
def_irq  SysTick_Handler
def_irq  WWDG_IRQHandler
def_irq  PVD_PVM_IRQHandler
def_irq  TAMP_STAMP_IRQHandler
def_irq  RTC_WKUP_IRQHandler
def_irq  FLASH_IRQHandler
def_irq  RCC_IRQHandler
def_irq  EXTI0_IRQHandler
def_irq  EXTI1_IRQHandler
def_irq  EXTI2_IRQHandler
def_irq  EXTI3_IRQHandler
def_irq  EXTI4_IRQHandler
def_irq  DMA1_Stream0_IRQHandler
def_irq  DMA1_Stream1_IRQHandler
def_irq  DMA1_Stream2_IRQHandler
def_irq  DMA1_Stream3_IRQHandler
def_irq  DMA1_Stream4_IRQHandler
def_irq  DMA1_Stream5_IRQHandler
def_irq  DMA1_Stream6_IRQHandler
def_irq  ADC_IRQHandler
def_irq  FDCAN1_IRQHandler
def_irq  FDCAN2_IRQHandler
def_irq  TIM1_BRK_IRQHandler
def_irq  TIM1_UP_IRQHandler
def_irq  TIM1_TRG_COM_IRQHandler
def_irq  TIM1_CC_IRQHandler
def_irq  TIM2_IRQHandler
def_irq  TIM3_IRQHandler
def_irq  TIM4_IRQHandler
def_irq  I2C1_EV_IRQHandler
def_irq  I2C1_ER_IRQHandler
def_irq  I2C2_EV_IRQHandler
def_irq  I2C2_ER_IRQHandler
def_irq  SPI1_IRQHandler
def_irq  SPI2_IRQHandler
def_irq  USART1_IRQHandler
def_irq  USART2_IRQHandler
def_irq  USART3_IRQHandler
def_irq  EXTI9_5_IRQHandler
def_irq  TIM15_IRQHandler
def_irq  TIM16_IRQHandler
def_irq  TIM17_IRQHandler
def_irq  DMA1_Stream7_IRQHandler
def_irq  HRTIM1_Master_IRQHandler
def_irq  HRTIM1_TIMA_IRQHandler
def_irq  HRTIM1_TIMB_IRQHandler
def_irq  HRTIM1_TIMC_IRQHandler
def_irq  HRTIM1_TIMD_IRQHandler
def_irq  HRTIM1_TIME_IRQHandler
def_irq  HRTIM1_FLT_IRQHandler
def_irq  SAI1_IRQHandler
def_irq  SAI2_IRQHandler
def_irq  QUADSPI_IRQHandler
def_irq  LPTIM1_IRQHandler
def_irq  CEC_IRQHandler
def_irq  I2C3_EV_IRQHandler
def_irq  I2C3_ER_IRQHandler
def_irq  SAI3_IRQHandler
def_irq  UART4_IRQHandler
def_irq  UART5_IRQHandler
def_irq  UART7_IRQHandler
def_irq  UART8_IRQHandler
def_irq  DMA2_Stream0_IRQHandler
def_irq  DMA2_Stream1_IRQHandler
def_irq  DMA2_Stream2_IRQHandler
def_irq  DMA2_Stream3_IRQHandler
def_irq  DMA2_Stream4_IRQHandler
def_irq  DMA2_Stream5_IRQHandler
def_irq  DMA2_Stream6_IRQHandler
def_irq  DMA2_Stream7_IRQHandler
def_irq  SDMMC1_IRQHandler
def_irq  SDMMC2_IRQHandler
def_irq  FDCAN3_IRQHandler
def_irq  OCTOSPI1_IRQHandler
def_irq  OCTOSPI2_IRQHandler
def_irq  I2C4_EV_IRQHandler
def_irq  I2C4_ER_IRQHandler
def_irq  LTDC_IRQHandler
def_irq  JPEG_IRQHandler
def_irq  FMAC_IRQHandler
def_irq  CORDIC_IRQHandler

/*****************************************************************************
 * Reset_Handler — entry point after boot
 *****************************************************************************/
.section    .text.Reset_Handler, "ax", %progbits
.type       Reset_Handler, %function
.globl      Reset_Handler
Reset_Handler:

    /* ---- Set stack pointer from linker symbol ---- */
    ldr     r0, =_estack
    msr     msp, r0

    /* ---- Enable FPU (CP10, CP11) ---- */
    movw    r0, #0xED88
    movt    r0, #0xE000
    ldr     r1, [r0]
    orr     r1, r1, #(0xF << 20)
    str     r1, [r0]
    dsb
    isb

    /* ---- D2 domain RAM initialisation ---- */
    /*
     * The D2 domain (M4 core) has its own RAM that must be initialised
     * by the M7. Copy data sections and zero BSS for D2 SRAM1/SRAM2/SRAM4.
     * These sections are placed by the linker script at specific addresses.
     */

    /* Zero D2 BSS sections */
    ldr     r0, =_sram2_bss_start
    ldr     r1, =_ebss
    movs    r2, #0
    b       .L_d2_bss_check
.L_d2_bss_loop:
    str     r2, [r0]
    adds    r0, r0, #4
.L_d2_bss_check:
    cmp     r0, r1
    bcc     .L_d2_bss_loop

    /* ---- Standard data/BSS init ---- */
    ldr     r0, =_sdata
    ldr     r1, =_edata
    ldr     r2, =_sidata
    b       .L_data_check
.L_data_loop:
    ldr     r3, [r2]
    str     r3, [r0]
    adds    r0, r0, #4
    adds    r2, r2, #4
.L_data_check:
    cmp     r0, r1
    bcc     .L_data_loop

    /* Zero BSS */
    ldr     r0, =_sbss
    ldr     r1, =_ebss
    movs    r2, #0
    b       .L_bss_check
.L_bss_loop:
    str     r2, [r0]
    adds    r0, r0, #4
.L_bss_check:
    cmp     r0, r1
    bcc     .L_bss_loop

    /* ---- SystemInit (HAL init) ---- */
    bl      SystemInit

    /* ---- C runtime init ---- */
    bl      __libc_init_array

    /* ---- Call main ---- */
    bl      main

    /* ---- In case main returns, loop forever ---- */
    b       .

/*****************************************************************************
 * M4 boot control
 *****************************************************************************/
.section    .text.M4_Boot, "ax", %progbits
.type       M4_Boot, %function
.globl      M4_Boot

/*
 * void M4_Boot(uint32_t m4_vector_table_addr);
 *
 * Sets the M4 vector table base address and releases the M4 from reset.
 * The M4 core will begin execution from its Reset_Handler.
 *
 * Registers:
 *   R0: M4 vector table base address (e.g., 0x08100000 for M4 in separate flash)
 *
 * Hardware:
 *   RCC->GCR  bit 16: BOOT_C2 (set to 1 to boot M4)
 *   SYSCFG->UR0: M4 vector table remap
 */
M4_Boot:
    /* Set M4 vector table via SYSCFG */
    ldr     r1, =0x58000400           /* SYSCFG base */
    str     r0, [r1, #0x00]           /* SYSCFG_UR0: M4 VTOR */

    /* Release M4 reset: set BOOT_C2 in RCC GCR */
    ldr     r1, =0x58024400           /* RCC base (D3 domain) */
    ldr     r2, [r1, #0x00]           /* RCC_GCR */
    orr     r2, r2, #(1 << 16)        /* Set BOOT_C2 */
    str     r2, [r1, #0x00]

    dsb
    isb
    bx      lr

/*****************************************************************************
 * SystemInit — minimal HAL init
 *****************************************************************************/
.section    .text.SystemInit, "ax", %progbits
.type       SystemInit, %function
.globl      SystemInit
SystemInit:
    /* ---- Configure FPU ---- */
    movw    r0, #0xED88
    movt    r0, #0xE000
    ldr     r1, [r0]
    orr     r1, r1, #(0xF << 20)
    str     r1, [r0]
    dsb
    isb

    /* ---- Enable ICache and DCache ---- */
    ldr     r0, =0xE000ED00           /* SCB base */
    ldr     r1, [r0, #0x24]           /* SCB_CCR */
    orr     r1, r1, #0x00010000       /* IC:  I-cache enable */
    orr     r1, r1, #0x00020000       /* DC:  D-cache enable */
    str     r1, [r0, #0x24]
    dsb
    isb

    /* ---- Set VTOR to bootloader (first sector) ---- */
    ldr     r0, =0xE000ED08           /* SCB_VTOR */
    ldr     r1, =0x08000000           /* Bootloader base */
    str     r1, [r0]
    dsb
    isb

    bx      lr

.end
