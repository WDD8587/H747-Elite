/* Minimal startup for STM32H747 M4 core (CI compilation only).
 * The real M4 is booted by M7 calling M4_Boot(), but for CI we
 * produce a standalone ELF that compiles and links.
 */
.syntax unified
.arch armv7e-m
.fpu  fpv4-sp-d16
.thumb

.section .isr_vector, "a", %progbits
.globl g_pfnVectors

.globl _estack
.globl _sidata
.globl _sdata
.globl _edata
.globl _sbss
.globl _ebss
.globl SystemInit
.globl main

.weak Default_Handler
.type Default_Handler, %function
Default_Handler:
    b .

.macro def_irq name
    .weak \name
    .thumb_set \name, Default_Handler
.endm

g_pfnVectors:
    .word _estack
    .word Reset_Handler
    .word NMI_Handler
    .word HardFault_Handler
    .word MemManage_Handler
    .word BusFault_Handler
    .word UsageFault_Handler
    .word 0
    .word 0
    .word 0
    .word 0
    .word SVC_Handler
    .word DebugMon_Handler
    .word 0
    .word PendSV_Handler
    .word SysTick_Handler
    /* External IRQs 16-81 for STM32H7 M4 */
    .word WWDG_IRQHandler
    .word PVD_PVM_IRQHandler
    .word TAMP_STAMP_IRQHandler
    .word RTC_WKUP_IRQHandler
    .word FLASH_IRQHandler
    .word RCC_IRQHandler
    .word EXTI0_IRQHandler
    .word EXTI1_IRQHandler
    .word EXTI2_IRQHandler
    .word EXTI3_IRQHandler
    .word EXTI4_IRQHandler
    .word DMA1_Stream0_IRQHandler
    .word DMA1_Stream1_IRQHandler
    .word DMA1_Stream2_IRQHandler
    .word DMA1_Stream3_IRQHandler
    .word DMA1_Stream4_IRQHandler
    .word DMA1_Stream5_IRQHandler
    .word DMA1_Stream6_IRQHandler
    .word ADC_IRQHandler
    .word FDCAN1_IRQHandler
    .word FDCAN2_IRQHandler
    .word TIM1_BRK_IRQHandler
    .word TIM1_UP_IRQHandler
    .word TIM1_TRG_COM_IRQHandler
    .word TIM1_CC_IRQHandler
    .word TIM2_IRQHandler
    .word TIM3_IRQHandler
    .word TIM4_IRQHandler
    .word I2C1_EV_IRQHandler
    .word I2C1_ER_IRQHandler
    .word I2C2_EV_IRQHandler
    .word I2C2_ER_IRQHandler
    .word SPI1_IRQHandler
    .word SPI2_IRQHandler
    .word USART1_IRQHandler
    .word USART2_IRQHandler
    .word USART3_IRQHandler
    .word EXTI9_5_IRQHandler
    .word TIM15_IRQHandler
    .word TIM16_IRQHandler
    .word TIM17_IRQHandler
    .word DMA1_Stream7_IRQHandler
    .word 0  // HRTIM1 reserved on M4
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word SAI1_IRQHandler
    .word SAI2_IRQHandler
    .word QUADSPI_IRQHandler
    .word LPTIM1_IRQHandler
    .word CEC_IRQHandler
    .word I2C3_EV_IRQHandler
    .word I2C3_ER_IRQHandler
    .word SAI3_IRQHandler
    .word UART4_IRQHandler
    .word UART5_IRQHandler
    .word UART7_IRQHandler
    .word UART8_IRQHandler
    .word DMA2_Stream0_IRQHandler
    .word DMA2_Stream1_IRQHandler
    .word DMA2_Stream2_IRQHandler
    .word DMA2_Stream3_IRQHandler
    .word DMA2_Stream4_IRQHandler
    .word DMA2_Stream5_IRQHandler
    .word DMA2_Stream6_IRQHandler
    .word DMA2_Stream7_IRQHandler

def_irq NMI_Handler
def_irq HardFault_Handler
def_irq MemManage_Handler
def_irq BusFault_Handler
def_irq UsageFault_Handler
def_irq SVC_Handler
def_irq DebugMon_Handler
def_irq PendSV_Handler
def_irq SysTick_Handler
def_irq WWDG_IRQHandler
def_irq PVD_PVM_IRQHandler
def_irq TAMP_STAMP_IRQHandler
def_irq RTC_WKUP_IRQHandler
def_irq FLASH_IRQHandler
def_irq RCC_IRQHandler
def_irq EXTI0_IRQHandler
def_irq EXTI1_IRQHandler
def_irq EXTI2_IRQHandler
def_irq EXTI3_IRQHandler
def_irq EXTI4_IRQHandler
def_irq DMA1_Stream0_IRQHandler
def_irq DMA1_Stream1_IRQHandler
def_irq DMA1_Stream2_IRQHandler
def_irq DMA1_Stream3_IRQHandler
def_irq DMA1_Stream4_IRQHandler
def_irq DMA1_Stream5_IRQHandler
def_irq DMA1_Stream6_IRQHandler
def_irq ADC_IRQHandler
def_irq TIM1_UP_IRQHandler
def_irq TIM2_IRQHandler
def_irq TIM3_IRQHandler
def_irq I2C1_EV_IRQHandler
def_irq I2C1_ER_IRQHandler
def_irq I2C3_EV_IRQHandler
def_irq I2C3_ER_IRQHandler
def_irq SPI1_IRQHandler
def_irq SPI2_IRQHandler
def_irq UART7_IRQHandler
def_irq DMA2_Stream2_IRQHandler
def_irq DMA2_Stream3_IRQHandler
def_irq DMA2_Stream5_IRQHandler
def_irq DMA2_Stream6_IRQHandler

.section .text.Reset_Handler, "ax", %progbits
.globl Reset_Handler
.type Reset_Handler, %function
Reset_Handler:
    ldr r0, =_estack
    msr msp, r0
    ldr r0, =_sdata
    ldr r1, =_edata
    ldr r2, =_sidata
    b .L_check
.L_loop:
    ldr r3, [r2]
    str r3, [r0]
    adds r0, #4
    adds r2, #4
.L_check:
    cmp r0, r1
    bcc .L_loop
    ldr r0, =_sbss
    ldr r1, =_ebss
    movs r2, #0
    b .L_bssc
.L_bssl:
    str r2, [r0]
    adds r0, #4
.L_bssc:
    cmp r0, r1
    bcc .L_bssl
    bl SystemInit
    bl main
    b .

.globl SystemInit
.type SystemInit, %function
SystemInit:
    bx lr
.end
