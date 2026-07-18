/**
  ******************************************************************************
  * @file    lpwr_stop2.c
  * @author  H747 Elite Team
  * @brief   ITE SLP_Sx -> STM32 Stop2 low-power mode translation.
  *
  *          Enter_Dock_Standby():
  *            1. Signal M7 to enter DeepSleep (D1 domain off)
  *            2. Save critical state to SRAM2 (retained in Stop2)
  *            3. Enter Stop2 via HAL_PWREx_EnterSTOP2Mode
  *            4. On wake (LPTIM, IR EXTI, TOUCH EXTI):
  *               - Restore clocks and peripherals
  *               - Signal M7 to wake via HSEM
  *
  *          Current consumption in Stop2: ~2-5 uA with SRAM2 retained.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include <string.h>
#include <stdint.h>

/* ---------------------------------------------------------------------------
 * Wake sources
 * --------------------------------------------------------------------------- */
typedef enum {
    WAKE_UNKNOWN  = 0,
    WAKE_IR       = 1,
    WAKE_TOUCH    = 2,
    WAKE_LPTIM    = 3
} WakeSource_t;

/* ---------------------------------------------------------------------------
 * SRAM2 backup area
 * --------------------------------------------------------------------------- */
#define SRAM2_BASE          0x30000000UL
#define SRAM2_BACKUP_OFFSET 0x00010000UL
#define SRAM2_BACKUP_ADDR   (SRAM2_BASE + SRAM2_BACKUP_OFFSET)

/* ---------------------------------------------------------------------------
 * Backup data structure stored in SRAM2
 * --------------------------------------------------------------------------- */
typedef struct __attribute__((packed)) {
    uint32_t magic;                  /* validation: 0xDEADBEEF         */
    uint8_t  dock_state;
    uint16_t bms_voltage_mV;
    uint8_t  bms_rsoc;
    int16_t  bms_temp_dK;
    uint16_t bms_cell_mV[4];
    uint8_t  fault_code;
    uint32_t uptime_s;
    uint32_t crc32;
} LPWR_BackupData_t;

/* ---------------------------------------------------------------------------
 * LPTIM1: 30 s wake from LSE
 *   LSE = 32768 Hz, prescaler /128 = 256 Hz
 *   Compare = 7680 -> 30 s
 * --------------------------------------------------------------------------- */
#define LPTIM_PRESCALER     LPTIM_PRESCALER_128
#define LPTIM_COMPARE_VALUE 7680U

static LPTIM_HandleTypeDef hlptim1;

/* ---------------------------------------------------------------------------
 * Module state
 * --------------------------------------------------------------------------- */
static WakeSource_t s_wake_source = WAKE_UNKNOWN;
static uint8_t      s_in_standby  = 0;

/*============================================================================
 *  CRC-32 for backup validation
 *============================================================================*/
static uint32_t CRC32_Compute(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint32_t j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ ((crc & 1U) ? 0xEDB88320U : 0U);
        }
    }
    return ~crc;
}

/*============================================================================
 *  LPTIM1 init
 *============================================================================*/
static void LPTIM1_Init(void)
{
    __HAL_RCC_LPTIM1_CLK_ENABLE();
    hlptim1.Instance = LPTIM1;
    hlptim1.Init.Clock.Source    = LPTIM_CLOCKSOURCE_APBCLOCK_LSE;
    hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER;
    hlptim1.Init.UltraLowPower   = LPTIM_UFLP_ENABLE;
    hlptim1.Init.Trigger.Source  = LPTIM_TRIGSOURCE_SOFTWARE;
    hlptim1.Init.Output.ActiveLevel = LPTIM_OUTPUTPOLARITY_HIGH;
    hlptim1.Init.Output.IdleState   = LPTIM_OUTPUT_IDLEVALUE_LOW;
    HAL_LPTIM_Init(&hlptim1);
    HAL_LPTIM_SetAutoreload(&hlptim1, LPTIM_COMPARE_VALUE);
    HAL_LPTIM_SetCompare(&hlptim1, LPTIM_COMPARE_VALUE);
    HAL_NVIC_SetPriority(LPTIM1_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(LPTIM1_IRQn);
}

/*============================================================================
 *  EXTI wake config
 *============================================================================*/
static void EXTI_Wake_Config(void)
{
    /* EXTI0 -- IR (PE0), falling edge */
    EXTI->IMR1 |= EXTI_IMR1_IM0;
    EXTI->FTSR1 |= EXTI_FTSR1_TR0;
    EXTI->RTSR1 &= ~EXTI_RTSR1_TR0;
    EXTI->PR1 = EXTI_PR1_PR0;
#if defined(EXTI_D1PMSK1_PMSK0)
    EXTI->D1PMSK1 |= EXTI_D1PMSK1_PMSK0;
#endif

    /* EXTI1 -- TOUCH (PE1), rising edge */
    EXTI->IMR1 |= EXTI_IMR1_IM1;
    EXTI->RTSR1 |= EXTI_RTSR1_TR1;
    EXTI->FTSR1 &= ~EXTI_FTSR1_TR1;
    EXTI->PR1 = EXTI_PR1_PR1;
#if defined(EXTI_D1PMSK1_PMSK1)
    EXTI->D1PMSK1 |= EXTI_D1PMSK1_PMSK1;
#endif
}

/*============================================================================
 *  SRAM2 backup / restore
 *============================================================================*/
static void Backup_State(void)
{
    LPWR_BackupData_t bd;
    volatile uint32_t *sram3 = (volatile uint32_t *)0x38000000UL;

    memset(&bd, 0, sizeof(bd));
    bd.magic          = 0xDEADBEEFU;
    bd.dock_state     = (uint8_t)(*(sram3 + 4) & 0xFF);    /* dock_state    */
    bd.bms_voltage_mV = (uint16_t)(*(sram3) & 0xFFFF);      /* bms_voltage   */
    bd.bms_rsoc       = (uint8_t)((*(sram3) >> 16) & 0xFF); /* bms_rsoc      */
    bd.fault_code     = (uint8_t)(*(sram3 + 8) & 0xFF);     /* fault_code    */
    bd.uptime_s       = HAL_GetTick() / 1000U;

    memcpy((uint32_t *)&bd.bms_cell_mV, (void *)(0x38000000UL + 8), 8);
    /* temp_dK at offset 6 in SRAM3 (bms_temp_dK is 2 bytes at offset 6) */
    bd.bms_temp_dK = (int16_t)(*(volatile uint16_t *)(0x38000000UL + 6));

    uint32_t crc_in[(sizeof(bd) - 4) / 4];
    memcpy(crc_in, &bd, sizeof(bd) - 4);
    bd.crc32 = CRC32_Compute((uint8_t *)crc_in, sizeof(bd) - 4);

    memcpy((void *)SRAM2_BACKUP_ADDR, &bd, sizeof(bd));
}

static uint8_t Restore_State(void)
{
    LPWR_BackupData_t bd;
    volatile uint32_t *sram3 = (volatile uint32_t *)0x38000000UL;

    memcpy(&bd, (void *)SRAM2_BACKUP_ADDR, sizeof(bd));
    if (bd.magic != 0xDEADBEEFU) return 0;

    uint32_t crc_in[(sizeof(bd) - 4) / 4];
    memcpy(crc_in, &bd, sizeof(bd) - 4);
    if (CRC32_Compute((uint8_t *)crc_in, sizeof(bd) - 4) != bd.crc32) return 0;

    *(sram3)      = ((uint32_t)bd.bms_rsoc << 16) | bd.bms_voltage_mV;
    *(sram3 + 4)  = (*(sram3 + 4) & 0xFFFFFF00UL) | bd.dock_state;
    *(sram3 + 8)  = (*(sram3 + 8) & 0xFFFFFF00UL) | bd.fault_code;
    *(volatile uint16_t *)(0x38000000UL + 6) = (uint16_t)bd.bms_temp_dK;
    memcpy((void *)(0x38000000UL + 8), bd.bms_cell_mV, 8);

    return 1;
}

/*============================================================================
 *  Signal M7 sleep/wake
 *============================================================================*/
static void Signal_M7_DeepSleep(void)
{
    volatile uint32_t *m7_cmd = (volatile uint32_t *)
        ((uint8_t *)0x38000000UL + 60);
    *m7_cmd = 0xDEADBEEFU;

    uint32_t timeout = HAL_GetTick();
    while (*m7_cmd != 0) {
        if ((HAL_GetTick() - timeout) > 10) break;
    }
}

static void Signal_M7_Wake(void)
{
    volatile uint32_t *m7_cmd = (volatile uint32_t *)
        ((uint8_t *)0x38000000UL + 60);
    *m7_cmd = 0xCAFEBABEU;
    if (HAL_HSEM_FastTake(0) == HAL_OK) {
        HAL_HSEM_Release(0);
    }
}

/*============================================================================
 *  LPWR_Init
 *============================================================================*/
void LPWR_Init(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    LPTIM1_Init();
    EXTI_Wake_Config();
    s_wake_source = WAKE_UNKNOWN;
    s_in_standby  = 0;
}

/*============================================================================
 *  Enter_Dock_Standby -- enter Stop2 low-power mode
 *============================================================================*/
void Enter_Dock_Standby(void)
{
    if (s_in_standby) return;

    Signal_M7_DeepSleep();
    Backup_State();

    /* De-init peripherals */
    extern I2C_HandleTypeDef hi2c1;
    extern I2C_HandleTypeDef hi2c3;
    HAL_I2C_DeInit(&hi2c1);
    HAL_I2C_DeInit(&hi2c3);
    __HAL_RCC_TIM1_CLK_DISABLE();
    NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
    __HAL_RCC_ADC3_CLK_DISABLE();

    /* Start LPTIM for 30 s wake */
    HAL_LPTIM_Counter_Start_IT(&hlptim1, LPTIM_COMPARE_VALUE);
    EXTI_Wake_Config();
    EXTI->PR1 = EXTI_PR1_PR0 | EXTI_PR1_PR1;

    /* Enter Stop2 */
    HAL_PWR_EnterD1DeepSleep();
    HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);

    /* ======= WAKEUP ======= */
    s_in_standby = 1;

    /* Determine wake source */
    s_wake_source = WAKE_UNKNOWN;
    if (EXTI->PR1 & EXTI_PR1_PR0) { s_wake_source = WAKE_IR;    EXTI->PR1 = EXTI_PR1_PR0; }
    else if (EXTI->PR1 & EXTI_PR1_PR1) { s_wake_source = WAKE_TOUCH; EXTI->PR1 = EXTI_PR1_PR1; }
    else { s_wake_source = WAKE_LPTIM; }

    /* Restore flash latency and peripherals */
    __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_9);
    __HAL_RCC_TIM1_CLK_ENABLE();
    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
    __HAL_RCC_ADC3_CLK_ENABLE();

    HAL_I2C_Init(&hi2c1);
    HAL_I2C_Init(&hi2c3);
    HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
    HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE);

    Restore_State();
    Signal_M7_Wake();
    s_in_standby = 0;
}

/*============================================================================
 *  LPWR_GetWakeSource
 *============================================================================*/
WakeSource_t LPWR_GetWakeSource(void) { return s_wake_source; }

/*============================================================================
 *  LPTIM1_IRQHandler
 *============================================================================*/
void LPTIM1_IRQHandler(void)   { HAL_LPTIM_IRQHandler(&hlptim1); }
void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim) { (void)hlptim; }

/*============================================================================
 *  LPWR_EnterStop2_Forced -- immediate Stop2 entry, no state save
 *============================================================================*/
void LPWR_EnterStop2_Forced(void)
{
    HAL_Delay(1);
    __HAL_RCC_I2C1_CLK_DISABLE();
    __HAL_RCC_I2C3_CLK_DISABLE();
    __HAL_RCC_TIM1_CLK_DISABLE();
    __HAL_RCC_ADC3_CLK_DISABLE();
    HAL_PWR_EnterD1DeepSleep();
    HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
    __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_9);
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_I2C3_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_ADC3_CLK_ENABLE();
}
