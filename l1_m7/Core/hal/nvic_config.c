/**
 * @file    nvic_config.c
 * @brief   All NVIC priority configuration.
 *
 * Priority grouping: NVIC_PRIORITYGROUP_4 (4 bits pre-emption, 0 sub-priority)
 *
 * Priority assignments:
 *   TIM1_UP        (prio 0,0)  — Highest: ADC sampling trigger
 *   TIM2           (prio 0,1)  — Motor control PWM / encoder
 *   SPI1           (prio 2,0)  — IMU ICM-20948
 *   SPI2           (prio 2,1)  — External flash / optional sensor
 *   I2C1           (prio 3,0)  — BQ40Z50 fuel gauge
 *   I2C3           (prio 3,1)  — VL53L5CX ToF sensor
 *   UART7          (prio 4,0)  — Debug serial (lowest latency-sensitive)
 *   EXTI0-3        (prio 5,0-3) — External interrupts (bumpers, button)
 *   DMA1_S0-7      (prio 6,0-7) — DMA1 streams
 *   DMA2_S0-7      (prio 6,0-7) — DMA2 streams
 *
 * Lower pre-emption priority number = higher priority.
 */

#include "stm32h7xx_hal.h"
#include "nvic_config.h"

/* ---------------------------------------------------------------------------
 * Interrupt priority mapping table
 * -------------------------------------------------------------------------*/

typedef struct {
    IRQn_Type irqn;         /* Interrupt request number        */
    uint32_t  preempt_prio; /* Pre-emption priority (0..15)    */
    uint32_t  sub_prio;     /* Sub-priority (0..15; unused in group 4) */
    const char *description; /* Purpose                         */
} nvic_entry_t;

static const nvic_entry_t nvic_table[] = {
    /* Core timers */
    {TIM1_UP_IRQn,              0, 0, "TIM1 Update — ADC sampling trigger"},
    {TIM2_IRQn,                 0, 1, "TIM2 — Motor PWM / encoder capture"},

    /* SPI peripherals */
    {SPI1_IRQn,                 2, 0, "SPI1 — ICM-20948 IMU"},
    {SPI2_IRQn,                 2, 1, "SPI2 — External flash / sensor"},

    /* I2C peripherals */
    {I2C1_EV_IRQn,              3, 0, "I2C1 Event — BQ40Z50 fuel gauge"},
    {I2C1_ER_IRQn,              3, 0, "I2C1 Error — BQ40Z50"},
    {I2C3_EV_IRQn,              3, 1, "I2C3 Event — VL53L5CX ToF sensor"},
    {I2C3_ER_IRQn,              3, 1, "I2C3 Error — VL53L5CX"},

    /* UART */
    {UART7_IRQn,                4, 0, "UART7 — Debug serial port"},

    /* SPI IPC */
    {SPI6_IRQn,                 2, 2, "SPI6 — IPC SPI slave to RK3566"},

    /* USB */
    {OTG_FS_IRQn,               4, 0, "USB OTG FS — CDC ACM IPC"},

    /* External interrupts */
    {EXTI0_IRQn,                5, 0, "EXTI0 — PA0 Bumper L"},
    {EXTI1_IRQn,                5, 1, "EXTI1 — PA1 Bumper R"},
    {EXTI2_IRQn,                5, 2, "EXTI2 — PA2 (reserved)"},
    {EXTI3_IRQn,                5, 3, "EXTI3 — PA3 IR dock sensor"},

    /* DMA1 streams */
    {DMA1_Stream0_IRQn,         6, 0, "DMA1 S0 — UART7 RX"},
    {DMA1_Stream1_IRQn,         6, 1, "DMA1 S1 — UART7 TX"},
    {DMA1_Stream2_IRQn,         6, 2, "DMA1 S2 — (reserved)"},
    {DMA1_Stream3_IRQn,         6, 3, "DMA1 S3 — (reserved)"},
    {DMA1_Stream4_IRQn,         6, 4, "DMA1 S4 — TIM1_UP/ADC1"},
    {DMA1_Stream5_IRQn,         6, 5, "DMA1 S5 — (reserved)"},
    {DMA1_Stream6_IRQn,         6, 6, "DMA1 S6 — (reserved)"},
    {DMA1_Stream7_IRQn,         6, 7, "DMA1 S7 — (reserved)"},

    /* DMA2 streams */
    {DMA2_Stream0_IRQn,         6, 0, "DMA2 S0 — (reserved)"},
    {DMA2_Stream1_IRQn,         6, 1, "DMA2 S1 — (reserved)"},
    {DMA2_Stream2_IRQn,         6, 2, "DMA2 S2 — SPI1 RX (ICM-20948)"},
    {DMA2_Stream3_IRQn,         6, 3, "DMA2 S3 — SPI1 TX (ICM-20948)"},
    {DMA2_Stream4_IRQn,         6, 4, "DMA2 S4 — (reserved)"},
    {DMA2_Stream5_IRQn,         6, 5, "DMA2 S5 — SPI6 RX (IPC slave)"},
    {DMA2_Stream6_IRQn,         6, 6, "DMA2 S6 — SPI6 TX (IPC slave)"},
    {DMA2_Stream7_IRQn,         6, 7, "DMA2 S7 — (reserved)"},

    /* PendSV / SVC for RTOS */
    {PendSV_IRQn,               15, 0, "PendSV — FreeRTOS context switch"},
    {SVC_Handler_IRQn,          15, 0, "SVC — FreeRTOS system calls"},
};

static const uint32_t nvic_table_count =
    sizeof(nvic_table) / sizeof(nvic_table[0]);

/* ---------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------*/

/**
 * @brief  Initialise all NVIC priorities.
 *
 * Must be called after HAL_Init() and before any peripheral init.
 * Sets priority grouping to 4-bit pre-emption (16 levels).
 */
void nvic_config_init(void)
{
    /* Set priority grouping: 4 bits pre-emption, 0 bits sub-priority */
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    for (uint32_t i = 0; i < nvic_table_count; i++)
    {
        const nvic_entry_t *entry = &nvic_table[i];
        HAL_NVIC_SetPriority(entry->irqn,
                              entry->preempt_prio,
                              entry->sub_prio);
    }
}

/**
 * @brief  Enable a specific interrupt and set its priority.
 *
 * Use this for runtime-registered interrupts (e.g., EXTI lines
 * configured after boot).
 *
 * @param  irqn          IRQ number.
 * @param  preempt_prio  Pre-emption priority (0 = highest).
 * @param  sub_prio      Sub-priority (unused with GROUP_4).
 */
void nvic_config_enable(IRQn_Type irqn, uint32_t preempt_prio,
                         uint32_t sub_prio)
{
    HAL_NVIC_SetPriority(irqn, preempt_prio, sub_prio);
    HAL_NVIC_EnableIRQ(irqn);
}

/**
 * @brief  Disable an interrupt.
 *
 * @param  irqn  IRQ number.
 */
void nvic_config_disable(IRQn_Type irqn)
{
    HAL_NVIC_DisableIRQ(irqn);
}

/**
 * @brief  Return the current exception priority for a given IRQn.
 *
 * @param  irqn  IRQ number.
 * @return Pre-emption priority (0-15).
 */
uint32_t nvic_config_get_priority(IRQn_Type irqn)
{
    return HAL_NVIC_GetPriority(irqn);
}
