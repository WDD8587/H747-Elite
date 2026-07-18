/**
  ******************************************************************************
  * @file    main_m4.c
  * @author  H747 Elite Team
  * @brief   M4 core entry point. 240 MHz system clock via HSI PLL.
  *          Spawns SafetyWD(100Hz), ToF(100Hz), BMS(10Hz), Dock(10Hz) tasks.
  *          Shared memory in SRAM3 (0x38000000) for M7 <-> M4 communication.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdint.h>

/* ---------------------------------------------------------------------------
 * Shared memory layout in SRAM3 (0x38000000).  Both M7 and M4 have
 * read/write access.  HSEM(1) guards non-atomic writes from the M4 side;
 * HSEM(2) guards the BMS fields in particular.
 * --------------------------------------------------------------------------- */
#define SRAM3_BASE              0x38000000UL

typedef volatile struct {
    /* ---- BMS (written by M4 vBmsTask, read by M7) ---- */
    uint16_t  bms_voltage_mV;              /* pack voltage, mV               */
    int16_t   bms_current_mA;              /* signed pack current, mA        */
    uint8_t   bms_rsoc;                    /* Relative State-Of-Charge 0-100 */
    int16_t   bms_temp_dK;                 /* temperature, deci-Kelvin       */
    uint16_t  bms_cell_mV[4];              /* individual cell voltages       */
    uint8_t   bms_cycle_count;             /* incremented each read cycle    */

    /* ---- ToF (written by M4 vTofTask, read by M7) ---- */
    uint16_t  tof_zones_mm[16];            /* 4x4 range data, mm             */
    uint8_t   tof_new_data;                /* M4 sets=1 after read; M7 clears*/

    /* ---- Dock (written by M4 vDockTask, read by M7) ---- */
    uint8_t   dock_state;                  /* DockState_t enum               */
    uint16_t  dock_adc_mV;                 /* 19 V rail after divider, mV    */
    uint8_t   dock_new_data;               /* M4 sets=1 after update         */

    /* ---- Safety (written by M4 vSafetyWdTask) ---- */
    uint8_t   fail_safe_state;             /* FsState_t enum                 */
    uint8_t   m4_heartbeat;                /* incremented each monitor cycle */
    uint8_t   fault_code;                  /* last fault code                */

    /* ---- M7 heartbeat counter (M7 writes, M4 reads) ---- */
    volatile uint32_t m7_heartbeat_cnt;

    /* ---- Padding for alignment / future use ---- */
    uint8_t   _rsvd[32];
} SharedMem_t;

_Static_assert(sizeof(SharedMem_t) <= 64UL * 1024UL,
               "SharedMem_t exceeds SRAM3 (64 KB)");

SharedMem_t *g_shared = (SharedMem_t *)SRAM3_BASE;

/* ---------------------------------------------------------------------------
 * FreeRTOS task handles
 * --------------------------------------------------------------------------- */
static TaskHandle_t s_task_tof    = NULL;
static TaskHandle_t s_task_bms    = NULL;
static TaskHandle_t s_task_dock   = NULL;
static TaskHandle_t s_task_safety = NULL;

/* ---------------------------------------------------------------------------
 * I2C handles
 * I2C1 -- BMS SMBus (100 kHz)
 * I2C3 -- ToF + Dock (400 kHz)
 * --------------------------------------------------------------------------- */
static I2C_HandleTypeDef hi2c1;
static I2C_HandleTypeDef hi2c3;

/* ---------------------------------------------------------------------------
 * Forward declarations
 * --------------------------------------------------------------------------- */
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);

static void vTofTask(void *arg);
static void vBmsTask(void *arg);
static void vDockTask(void *arg);
static void vSafetyWdTask(void *arg);

/* External module inits */
extern void Safety_Init(void);
extern void Safety_WWDT_Init(void);
extern void Safety_Monitor(void);
extern void Safety_Failsafe(uint8_t state);
extern void BMS_Init(I2C_HandleTypeDef *i2c);
extern int  BMS_ReadAll(I2C_HandleTypeDef *i2c, void *data);
extern void ChargeFSM(void *state, void *bms, float *i_target);
extern void ToF_Init(I2C_HandleTypeDef *i2c);
extern int  ToF_Read(I2C_HandleTypeDef *i2c, uint16_t *zones_mm);
extern void Dock_Init(I2C_HandleTypeDef *i2c);
extern uint8_t Dock_Run(I2C_HandleTypeDef *i2c);
extern uint16_t Dock_Get19V_mV(void);
extern void Enter_Dock_Standby(void);

/* BMS / charge types used locally */
typedef struct {
    uint16_t voltage_mV;
    int16_t  current_mA;
    uint8_t  rsoc;
    int16_t  temp_dK;
    uint16_t cell_mV[4];
    uint8_t  valid;
} BmsData_t;

typedef enum {
    CC_0_5C, CC_1C, CC_2C, CV_16_8V, TRICKLE, DONE, FAULT
} ChargeState_t;

/*============================================================================
 *  SYSTEM CLOCK -- M4 @ 240 MHz from HSI (no HSE needed)
 *  HSI 64 MHz -> PLL1: M=8, N=60, P=2 -> PLL1_P = 240 MHz
 *  SYSCLK = PLL1_P -> D1 domain /1, D2 domain (CD) /1 -> M4 @ 240 MHz
 *============================================================================*/
static void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct   = {0};
    RCC_OscInitTypeDef RCC_OscInitStruct   = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /* ---- HSI + PLL1 ---- */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState       = RCC_HSI_DIV1;           /* 64 MHz    */
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM       = 8;    /* 64/8 = 8 MHz             */
    RCC_OscInitStruct.PLL.PLLN       = 60;   /* VCO = 8*60 = 480 MHz     */
    RCC_OscInitStruct.PLL.PLLP       = 2;    /* PLL1_P = 480/2 = 240 MHz */
    RCC_OscInitStruct.PLL.PLLQ       = 4;
    RCC_OscInitStruct.PLL.PLLR       = 4;
    RCC_OscInitStruct.PLL.PLLVCOSEL  = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLRGE     = RCC_PLL1VCIRANGE_2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Safety_Failsafe(3);
    }

    /* ---- SYSCLK = PLL1_P ---- */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK  |
                                       RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_PCLK1  |
                                       RCC_CLOCKTYPE_PCLK2  |
                                       RCC_CLOCKTYPE_D3PCLK1|
                                       RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;   /* 120 MHz */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;   /* 120 MHz */
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;   /* 120 MHz */
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;   /* 120 MHz */
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_9) != HAL_OK) {
        Safety_Failsafe(3);
    }

    /* ---- Peripheral clocks ---- */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1 |
                                        RCC_PERIPHCLK_I2C3 |
                                        RCC_PERIPHCLK_LPTIM1;
    PeriphClkInit.I2c1ClockSelection   = RCC_I2C1CLKSOURCE_PLL3;
    PeriphClkInit.I2c3ClockSelection   = RCC_I2C3CLKSOURCE_PLL3;
    PeriphClkInit.Lptim1ClockSelection = RCC_LPTIM1CLKSOURCE_LSE;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
}

/*============================================================================
 *  GPIO
 *  PWM_SHUTDOWN -- PD12, active-high push-pull
 *  IR sensor    -- PE0, falling edge EXTI
 *  TOUCH wake   -- PE1, rising edge EXTI
 *  ToF int      -- PC8, rising edge EXTI
 *============================================================================*/
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /* PWM_SHUTDOWN -- PD12, output PP, initially low */
    gpio.Pin   = GPIO_PIN_12;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOD, &gpio);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

    /* IR sensor -- PE0, falling edge (TSOP4838 active-low) */
    gpio.Pin   = GPIO_PIN_0;
    gpio.Mode  = GPIO_MODE_IT_FALLING;
    gpio.Pull  = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOE, &gpio);
    HAL_NVIC_SetPriority(EXTI0_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);

    /* TOUCH -- PE1, rising edge */
    gpio.Pin   = GPIO_PIN_1;
    gpio.Mode  = GPIO_MODE_IT_RISING;
    gpio.Pull  = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOE, &gpio);
    HAL_NVIC_SetPriority(EXTI1_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);

    /* ToF interrupt -- PC8, rising edge (data ready) */
    gpio.Pin   = GPIO_PIN_8;
    gpio.Mode  = GPIO_MODE_IT_RISING;
    gpio.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &gpio);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/*============================================================================
 *  I2C1 -- BMS SMBus @ 100 kHz
 *============================================================================*/
static void MX_I2C1_Init(void)
{
    hi2c1.Instance             = I2C1;
    hi2c1.Init.Timing          = 0x10B17DB5;  /* 100 kHz @ 240 MHz APB1=120MHz*/
    hi2c1.Init.OwnAddress1     = 0x32;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Safety_Failsafe(3);
    }
    HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
}

/*============================================================================
 *  I2C3 -- ToF + Dock @ 400 kHz
 *============================================================================*/
static void MX_I2C3_Init(void)
{
    hi2c3.Instance             = I2C3;
    hi2c3.Init.Timing          = 0x20A07DB5;  /* 400 kHz @ 240 MHz APB2=120MHz*/
    hi2c3.Init.OwnAddress1     = 0x34;
    hi2c3.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c3.Init.OwnAddress2     = 0;
    hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c3.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c3) != HAL_OK) {
        Safety_Failsafe(3);
    }
    HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE);
}

/*============================================================================
 *  FreeRTOS tasks
 *============================================================================*/

/* ---- ToF @ 100 Hz (10 ms period) ---- */
static void vTofTask(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10);

    ToF_Init(&hi2c3);

    for (;;) {
        vTaskDelayUntil(&last_wake, period);
        if (ToF_Read(&hi2c3, g_shared->tof_zones_mm) == 0) {
            g_shared->tof_new_data = 1;
        }
    }
}

/* ---- BMS @ 10 Hz (100 ms period) ---- */
static void vBmsTask(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(100);

    BmsData_t bd;
    ChargeState_t charge_state = TRICKLE;
    float i_target = 0.0f;

    BMS_Init(&hi2c1);

    for (;;) {
        vTaskDelayUntil(&last_wake, period);
        memset(&bd, 0, sizeof(bd));

        if (BMS_ReadAll(&hi2c1, &bd) == 0) {
            /* Write to SRAM3 under HSEM(2) protection */
            while (HAL_HSEM_FastTake(2) != HAL_OK) { }
            g_shared->bms_voltage_mV  = bd.voltage_mV;
            g_shared->bms_current_mA  = bd.current_mA;
            g_shared->bms_rsoc        = bd.rsoc;
            g_shared->bms_temp_dK     = bd.temp_dK;
            g_shared->bms_cell_mV[0]  = bd.cell_mV[0];
            g_shared->bms_cell_mV[1]  = bd.cell_mV[1];
            g_shared->bms_cell_mV[2]  = bd.cell_mV[2];
            g_shared->bms_cell_mV[3]  = bd.cell_mV[3];
            g_shared->bms_cycle_count++;
            HAL_HSEM_Release(2);
        }

        ChargeFSM(&charge_state, &bd, &i_target);
    }
}

/* ---- Dock @ 10 Hz (100 ms period) ---- */
static void vDockTask(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(100);

    Dock_Init(&hi2c3);

    for (;;) {
        vTaskDelayUntil(&last_wake, period);
        g_shared->dock_state   = Dock_Run(&hi2c3);
        g_shared->dock_adc_mV  = Dock_Get19V_mV();
        g_shared->dock_new_data = 1;
    }
}

/* ---- Safety watchdog @ 100 Hz (10 ms period) ---- */
static void vSafetyWdTask(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10);

    Safety_Init();

    for (;;) {
        vTaskDelayUntil(&last_wake, period);
        Safety_Monitor();
        g_shared->m4_heartbeat++;
    }
}

/*============================================================================
 *  EXTI / interrupt call-throughs
 *============================================================================*/
void EXTI0_IRQHandler(void)  { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0); }
void EXTI1_IRQHandler(void)  { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1); }
void EXTI9_5_IRQHandler(void){ HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8); }

/*============================================================================
 *  HAL MSP -- weak overrides
 *============================================================================*/
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef gpio = {0};

    if (hi2c->Instance == I2C1) {
        __HAL_RCC_I2C1_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        /* PB6=SCL, PB7=SDA -- AF4 */
        gpio.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
        gpio.Mode      = GPIO_MODE_AF_OD;
        gpio.Pull      = GPIO_PULLUP;
        gpio.Speed     = GPIO_SPEED_FREQ_LOW;
        gpio.Alternate = GPIO_AF4_I2C1;
        HAL_GPIO_Init(GPIOB, &gpio);
    }
    else if (hi2c->Instance == I2C3) {
        __HAL_RCC_I2C3_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        /* PA8=SCL -- AF4, PC9=SDA -- AF4 */
        gpio.Pin       = GPIO_PIN_8;
        gpio.Mode      = GPIO_MODE_AF_OD;
        gpio.Pull      = GPIO_PULLUP;
        gpio.Speed     = GPIO_SPEED_FREQ_LOW;
        gpio.Alternate = GPIO_AF4_I2C3;
        HAL_GPIO_Init(GPIOA, &gpio);

        gpio.Pin       = GPIO_PIN_9;
        HAL_GPIO_Init(GPIOC, &gpio);
    }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        __HAL_RCC_I2C1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7);
    } else if (hi2c->Instance == I2C3) {
        __HAL_RCC_I2C3_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_9);
    }
}

void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
}

/*============================================================================
 *  main
 *============================================================================*/
int main(void)
{
    /* Clear SRAM3 shared memory */
    memset((void *)SRAM3_BASE, 0, sizeof(SharedMem_t));

    /* HAL layer (enables HSI automatically) */
    HAL_Init();

    /* Clock, GPIO, I2C */
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_I2C3_Init();

    /* Safety watchdog -- must run before FreeRTOS starts */
    Safety_WWDT_Init();

    /* Create tasks */
    xTaskCreate(vTofTask,      "ToF",      256, NULL, 3, &s_task_tof);
    xTaskCreate(vBmsTask,      "BMS",      384, NULL, 2, &s_task_bms);
    xTaskCreate(vDockTask,     "Dock",     256, NULL, 2, &s_task_dock);
    xTaskCreate(vSafetyWdTask, "SafetyWD", 256, NULL, 4, &s_task_safety);

    /* Start FreeRTOS scheduler -- never returns */
    vTaskStartScheduler();

    /* Trap if scheduler exits */
    Safety_Failsafe(3);
    for (;;) { __NOP(); }
}
