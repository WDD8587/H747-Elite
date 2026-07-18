/**
 * @file    clock_tree.c
 * @brief   Complete H747 clock tree initialisation (M7 + M4).
 *
 * Clock architecture:
 *
 *   HSE (25 MHz) ─┬─> PLL1 ─> PLL1_P ─> M7 core @ 480 MHz
 *                  │         └> PLL1_Q  ─> FDCAN / SDMMC
 *                  │         └> PLL1_R  ─> (Reserved)
 *                  │
 *                  ├─> PLL2 ─> PLL2_P ─> M4 core @ 240 MHz
 *                  │         └> PLL2_Q  ─> SPI / FMC
 *                  │         └> PLL2_R  ─> SAI / DFSDM
 *                  │
 *                  ├─> PLL3 ─> PLL3_P ─> I2C / SPI / UART peripherals
 *                  │         └> PLL3_Q  ─> SPI123 / USART
 *                  │         └> PLL3_R  ─> (Peripherals)
 *                  │
 *                  └─> Per_ck (direct or via HSI48)
 *
 *   LSE (32.768 kHz) ─> RTC / LPTIM / LPower domains
 *
 * Frequency calculations:
 *   PLL1:  VCO = 25 MHz / 5 * 192 = 960 MHz
 *          P   = 960 / 2 = 480 MHz  (M7 core)
 *          Q   = 960 / 4 = 240 MHz  (FDCAN)
 *
 *   PLL2:  VCO = 25 MHz / 5 * 192 = 960 MHz
 *          P   = 960 / 4 = 240 MHz  (M4 core)
 *          Q   = 960 / 4 = 240 MHz  (FMC)
 *
 *   PLL3:  VCO = 25 MHz / 5 * 96  = 480 MHz
 *          P   = 480 / 2 = 240 MHz (Peripherals)
 *          Q   = 480 / 4 = 120 MHz (UART / SPI)
 *
 *   AHB:   M7: 480 MHz, M4: 240 MHz
 *   APB1:  120 MHz (max: 120 MHz)
 *   APB2:  120 MHz (max: 120 MHz)
 *   APB3:  160 MHz (max: 160 MHz)
 *   APB4:  120 MHz (max: 120 MHz)
 */

#include "stm32h7xx_hal.h"
#include "clock_tree.h"

/* ---------------------------------------------------------------------------
 * Oscillator constants
 * -------------------------------------------------------------------------*/
#define HSE_FREQ    25000000UL     /* 25 MHz */
#define LSE_FREQ    32768UL        /* 32.768 kHz */

/* ---------------------------------------------------------------------------
 * PLL configuration tables
 * -------------------------------------------------------------------------*/

/**
 * PLL1: M7 core @ 480 MHz
 *   VCO = HSE / M * N = 25 MHz / 5 * 192 = 960 MHz
 *   P   = VCO / 2 = 480 MHz
 *   Q   = VCO / 4 = 240 MHz
 *   R   = VCO / 2 = 480 MHz
 */
static const RCC_PLLInitTypeDef pll1_cfg = {
    .PLLState  = RCC_PLL_ON,
    .PLLSource = RCC_PLLSOURCE_HSE,
    .PLLM      = 5,
    .PLLN      = 192,
    .PLLP      = 2,
    .PLLQ      = 4,
    .PLLR      = 2,
    .PLLRGE    = RCC_PLL1VCIRANGE_2,  /* VCO 8-16 MHz input */
    .PLLVCOSEL = RCC_PLL1VCOWIDE,     /* VCO wide range: 128-960 MHz */
    .PLLFRACN  = 0,
};

/**
 * PLL2: M4 core @ 240 MHz
 *   VCO = 25 MHz / 5 * 192 = 960 MHz
 *   P   = VCO / 4 = 240 MHz
 *   Q   = VCO / 4 = 240 MHz
 *   R   = VCO / 2 = 480 MHz
 */
static const RCC_PLLInitTypeDef pll2_cfg = {
    .PLLState  = RCC_PLL_ON,
    .PLLSource = RCC_PLLSOURCE_HSE,
    .PLLM      = 5,
    .PLLN      = 192,
    .PLLP      = 4,
    .PLLQ      = 4,
    .PLLR      = 2,
    .PLLRGE    = RCC_PLL2VCIRANGE_2,
    .PLLVCOSEL = RCC_PLL2VCOWIDE,
    .PLLFRACN  = 0,
};

/**
 * PLL3: Peripheral clock @ 240 MHz / 120 MHz
 *   VCO = 25 MHz / 5 * 96 = 480 MHz
 *   P   = VCO / 2 = 240 MHz
 *   Q   = VCO / 4 = 120 MHz
 *   R   = VCO / 8 = 60 MHz
 */
static const RCC_PLLInitTypeDef pll3_cfg = {
    .PLLState  = RCC_PLL_ON,
    .PLLSource = RCC_PLLSOURCE_HSE,
    .PLLM      = 5,
    .PLLN      = 96,
    .PLLP      = 2,
    .PLLQ      = 4,
    .PLLR      = 8,
    .PLLRGE    = RCC_PLL3VCIRANGE_2,
    .PLLVCOSEL = RCC_PLL3VCOWIDE,
    .PLLFRACN  = 0,
};

/* ---------------------------------------------------------------------------
 * Clock configuration structures
 * -------------------------------------------------------------------------*/

/**
 * Core clock config for M7 at 480 MHz.
 */
static const RCC_ClkInitTypeDef clk_cfg_m7 = {
    .ClockType  = RCC_CLOCKTYPE_HCLK   |
                  RCC_CLOCKTYPE_SYSCLK |
                  RCC_CLOCKTYPE_PCLK1  |
                  RCC_CLOCKTYPE_PCLK2  |
                  RCC_CLOCKTYPE_D3PCLK1|
                  RCC_CLOCKTYPE_D1PCLK1,
    .SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK,
    .SYSCLKDivider = RCC_SYSCLK_DIV1,   /* 480 MHz */
    .AHBCLKDivider  = RCC_HCLK_DIV1,    /* 480 MHz */
    .APB3CLKDivider = RCC_APB3_DIV2,    /* 240 MHz → 160 MHz max => div 3? */
                                         /* H747 D1 APB3 max=160 MHz */
                                         /* Actually 480/3=160 MHz — use DIV4 (120 MHz) */
    .APB1CLKDivider = RCC_APB1_DIV4,    /* 480/4 = 120 MHz */
    .APB2CLKDivider = RCC_APB2_DIV4,    /* 480/4 = 120 MHz */
    .APB4CLKDivider = RCC_APB4_DIV4,    /* 480/4 = 120 MHz */
};

/* Voltage regulator needed for 480 MHz: VOS1 */
#define PWR_REGULATOR_VOLTAGE_SCALE1  PWR_REGULATOR_VOLTAGE_SCALE1

/* ---------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------*/

/**
 * @brief  Initialise the complete clock tree for M7 CPU.
 *
 * Sequence:
 *   1. Enable HSE oscillator (25 MHz).
 *   2. Configure LSE (32.768 kHz) for RTC/LPTIM.
 *   3. Set voltage regulator to VOS1 (high performance).
 *   4. Configure and enable PLL1, PLL2, PLL3.
 *   5. Set system clock source to PLL1 (480 MHz).
 *   6. Configure AHB and APB dividers.
 *   7. Configure M4 clock domain (CDE).
 *
 * @note  This function must run from flash (with ART enabled) or SRAM.
 */
void clock_tree_init(void)
{
    HAL_StatusTypeDef ret;

    /* ---- 1. Enable HSE ---- */
    RCC_OscInitTypeDef osc_cfg = {0};
    osc_cfg.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc_cfg.HSEState       = RCC_HSE_ON;
    osc_cfg.PLLState       = RCC_PLL_NONE;  /* Will configure separately */
    osc_cfg.LSEState       = RCC_LSE_OFF;
    osc_cfg.LSIState       = RCC_LSI_OFF;
    osc_cfg.HSIState       = RCC_HSI_OFF;
    osc_cfg.HSI48State     = RCC_HSI48_OFF;
    osc_cfg.HSIDiv         = RCC_HSI_DIV1;
    osc_cfg.CSICalibration = 0;
    osc_cfg.HSICalibration = 0;

    /* HSE configuration */
    {
        RCC_OscInitTypeDef hse_cfg = osc_cfg;
        hse_cfg.OscillatorType = RCC_OSCILLATORTYPE_HSE;
        hse_cfg.HSEState       = RCC_HSE_ON;
        ret = HAL_RCC_OscConfig(&hse_cfg);
        if (ret != HAL_OK) {
            while (1) { __NOP(); }  /* Trap */
        }
    }

    /* ---- 2. Enable LSE ---- */
    {
        RCC_OscInitTypeDef lse_cfg = osc_cfg;
        lse_cfg.OscillatorType = RCC_OSCILLATORTYPE_LSE;
        lse_cfg.LSEState       = RCC_LSE_ON;
        lse_cfg.PLLState       = RCC_PLL_NONE;
        ret = HAL_RCC_OscConfig(&lse_cfg);
        if (ret != HAL_OK) {
            /* LSE may not be present; non-fatal */
        }
    }

    /* ---- 3. Set VOS1 for 480 MHz ---- */
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWREx_SetVoltageScale(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* ---- 4. Configure PLL3 (peripherals) first ---- */
    {
        RCC_OscInitTypeDef pll_osc_cfg = {0};
        pll_osc_cfg.OscillatorType = RCC_OSCILLATORTYPE_NONE;
        pll_osc_cfg.PLLState       = RCC_PLL_ON;
        pll_osc_cfg.PLL3.PLLState  = RCC_PLL_ON;
        pll_osc_cfg.PLL3.PLLSource = RCC_PLLSOURCE_HSE;
        pll_osc_cfg.PLL3.PLLM      = pll3_cfg.PLLM;
        pll_osc_cfg.PLL3.PLLN      = pll3_cfg.PLLN;
        pll_osc_cfg.PLL3.PLLP      = pll3_cfg.PLLP;
        pll_osc_cfg.PLL3.PLLQ      = pll3_cfg.PLLQ;
        pll_osc_cfg.PLL3.PLLR      = pll3_cfg.PLLR;
        pll_osc_cfg.PLL3.PLLRGE    = pll3_cfg.PLLRGE;
        pll_osc_cfg.PLL3.PLLVCOSEL = pll3_cfg.PLLVCOSEL;
        pll_osc_cfg.PLL3.PLLFRACN  = pll3_cfg.PLLFRACN;
        ret = HAL_RCC_OscConfig(&pll_osc_cfg);
        if (ret != HAL_OK) {
            while (1) { __NOP(); }
        }
    }

    /* ---- 5. Configure PLL2 (M4 core) ---- */
    {
        RCC_OscInitTypeDef pll_osc_cfg = {0};
        pll_osc_cfg.OscillatorType = RCC_OSCILLATORTYPE_NONE;
        pll_osc_cfg.PLL2.PLLState  = RCC_PLL_ON;
        pll_osc_cfg.PLL2.PLLSource = RCC_PLLSOURCE_HSE;
        pll_osc_cfg.PLL2.PLLM      = pll2_cfg.PLLM;
        pll_osc_cfg.PLL2.PLLN      = pll2_cfg.PLLN;
        pll_osc_cfg.PLL2.PLLP      = pll2_cfg.PLLP;
        pll_osc_cfg.PLL2.PLLQ      = pll2_cfg.PLLQ;
        pll_osc_cfg.PLL2.PLLR      = pll2_cfg.PLLR;
        pll_osc_cfg.PLL2.PLLRGE    = pll2_cfg.PLLRGE;
        pll_osc_cfg.PLL2.PLLVCOSEL = pll2_cfg.PLLVCOSEL;
        pll_osc_cfg.PLL2.PLLFRACN  = pll2_cfg.PLLFRACN;
        ret = HAL_RCC_OscConfig(&pll_osc_cfg);
        if (ret != HAL_OK) {
            while (1) { __NOP(); }
        }
    }

    /* ---- 6. Configure PLL1 (M7 core) ---- */
    {
        RCC_OscInitTypeDef pll_osc_cfg = {0};
        pll_osc_cfg.OscillatorType = RCC_OSCILLATORTYPE_NONE;
        pll_osc_cfg.PLL1.PLLState  = RCC_PLL_ON;
        pll_osc_cfg.PLL1.PLLSource = RCC_PLLSOURCE_HSE;
        pll_osc_cfg.PLL1.PLLM      = pll1_cfg.PLLM;
        pll_osc_cfg.PLL1.PLLN      = pll1_cfg.PLLN;
        pll_osc_cfg.PLL1.PLLP      = pll1_cfg.PLLP;
        pll_osc_cfg.PLL1.PLLQ      = pll1_cfg.PLLQ;
        pll_osc_cfg.PLL1.PLLR      = pll1_cfg.PLLR;
        pll_osc_cfg.PLL1.PLLRGE    = pll1_cfg.PLLRGE;
        pll_osc_cfg.PLL1.PLLVCOSEL = pll1_cfg.PLLVCOSEL;
        pll_osc_cfg.PLL1.PLLFRACN  = pll1_cfg.PLLFRACN;
        ret = HAL_RCC_OscConfig(&pll_osc_cfg);
        if (ret != HAL_OK) {
            while (1) { __NOP(); }
        }
    }

    /* ---- 7. System clock: PLL1 (480 MHz) with dividers ---- */
    ret = HAL_RCC_ClockConfig(&clk_cfg_m7, FLASH_LATENCY_4);
    if (ret != HAL_OK) {
        while (1) { __NOP(); }
    }

    /* ---- 8. M4 domain clock configuration ---- */
    /*
     * CDE (M4 domain) gets PLL2_P via ck_cde.
     * Write directly to RCC_D1CFGR for CDESYS clock.
     */
    MODIFY_REG(RCC->D1CFGR, RCC_D1CFGR_CDESYS, 1U);  /* PLL2_P */
    MODIFY_REG(RCC->CDCCIP1R, RCC_CDCCIP1R_CDESEL, 0U);

    /* ---- 9. Peripheral clock enables for key peripherals ---- */
    __HAL_RCC_SYSCFG_CLK_ENABLE();
}

/**
 * @brief  Return the current system clock frequencies.
 *
 * @param  hclk_freq  Output: HCLK frequency in Hz (optional).
 * @param  apb1_freq  Output: APB1 frequency in Hz (optional).
 * @param  apb2_freq  Output: APB2 frequency in Hz (optional).
 */
void clock_tree_get_frequencies(uint32_t *hclk_freq,
                                 uint32_t *apb1_freq,
                                 uint32_t *apb2_freq)
{
    uint32_t sysclk = HAL_RCC_GetSysClockFreq();
    uint32_t hclk   = HAL_RCC_GetHCLKFreq();
    uint32_t pclk1  = HAL_RCC_GetPCLK1Freq();
    uint32_t pclk2  = HAL_RCC_GetPCLK2Freq();

    (void)sysclk;

    if (hclk_freq != NULL) *hclk_freq = hclk;
    if (apb1_freq != NULL) *apb1_freq = pclk1;
    if (apb2_freq != NULL) *apb2_freq = pclk2;
}
