/**
  ******************************************************************************
  * @file    safety_wd.c
  * @author  H747 Elite Team
  * @brief   M4 safety watchdog.
  *          - WWDG hardware watchdog (catches M4 hangs)
  *          - HSEM(0) heartbeat monitor (catches M7 death)
  *          - TIM1 BKIN + GPIO PWM shutdown (hardware kill, < 200 ns)
  *          - Fail-safe state machine (FS_NORMAL / FS_SPIN_HOME /
  *            FS_DOCK_SEARCH / FS_STOP)
  *
  * THE KILLER INTERVIEW STORY
  *   M4 @ 100 Hz polls HSEM(0).  If M7 has not taken the semaphore for 5
  *   consecutive polls (50 ms), M4 declares M7 dead, asserts the PWM
  *   SHUTDOWN GPIO (hardware kill), disables TIM1, and drives the fail-safe
  *   state machine to bring the robot home safely.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include <stdint.h>

/* ---------------------------------------------------------------------------
 * Fail-safe states
 * --------------------------------------------------------------------------- */
#define FS_NORMAL       0U   /* no fault                               */
#define FS_SPIN_HOME    1U   /* stop motors, spin in place             */
#define FS_DOCK_SEARCH  2U   /* navigate via ToF + dock beacons        */
#define FS_STOP         3U   /* hard stop, wait for human intervention */

/* PWM SHUTDOWN GPIO -- PD12 */
#define PWM_SHUT_PORT   GPIOD
#define PWM_SHUT_PIN    GPIO_PIN_12

/* HSEM ID for M7 heartbeat */
#define HSEM_HEARTBEAT  0U

/* ---------------------------------------------------------------------------
 * Module state
 * --------------------------------------------------------------------------- */
static uint8_t  m7_alive_cnt    = 0;
static uint8_t  fail_safe_state = FS_NORMAL;
static uint8_t  m7_was_dead     = 0;       /* sticky "M7 was dead" flag    */
static uint32_t last_hb_cnt     = 0;       /* last m7_heartbeat_cnt value  */

/* For WWDG early-wake handler -- counts refreshes */
static volatile uint32_t wwdg_refresh_ok = 0;

/* ---------------------------------------------------------------------------
 * Shared memory access to SRAM3
 * --------------------------------------------------------------------------- */
typedef volatile struct {
    uint16_t  bms_voltage_mV;
    int16_t   bms_current_mA;
    uint8_t   bms_rsoc;
    int16_t   bms_temp_dK;
    uint16_t  bms_cell_mV[4];
    uint8_t   bms_cycle_count;
    uint16_t  tof_zones_mm[16];
    uint8_t   tof_new_data;
    uint8_t   dock_state;
    uint16_t  dock_adc_mV;
    uint8_t   dock_new_data;
    uint8_t   fail_safe_state;
    uint8_t   m4_heartbeat;
    uint8_t   fault_code;
    volatile uint32_t m7_heartbeat_cnt;
    uint8_t   _rsvd[32];
} SharedMem_s;

#define SHMEM   ((SharedMem_s *)0x38000000UL)

/*============================================================================
 *  Safety_Init -- GPIO + TIM1 BKIN + default state
 *============================================================================*/
void Safety_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* PWM_SHUTDOWN as output, initially low (power enabled) */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    gpio.Pin   = PWM_SHUT_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(PWM_SHUT_PORT, &gpio);
    HAL_GPIO_WritePin(PWM_SHUT_PORT, PWM_SHUT_PIN, GPIO_PIN_RESET);

    /* ---- TIM1 Break input (BKIN) on PA6 ---- */
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    gpio.Pin       = GPIO_PIN_6;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLDOWN;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* TIM1 break configuration:
     *   BKE = 1   (break enable)
     *   BKP = 1   (break polarity active high)
     *   AOE = 0   (do NOT auto-exit break on ETRF -- stay safe)
     *   OSSR = 1  (idle level on break)
     *   BKF = 0   (no filter, immediate reaction)
     */
    TIM1->BDTR &= ~(TIM_BDTR_BKF_Msk | TIM_BDTR_AOE);
    TIM1->BDTR |=  TIM_BDTR_BKE | TIM_BDTR_BKP | TIM_BDTR_OSSR;
    /* On break: all TIM1 output channels enter their configured idle state,
     * killing motor PWM in hardware (< 1 clock cycle). */

    /* Initialise fail-safe state */
    fail_safe_state = FS_NORMAL;
    m7_alive_cnt    = 0;
    m7_was_dead     = 0;

    /* Enable HSEM clock */
    __HAL_RCC_HSEM_CLK_ENABLE();
}

/*============================================================================
 *  Safety_WWDT_Init -- Window Watchdog (catches M4 hangs)
 *
 *  With HCLK = 240 MHz, prescaler = 8:
 *    T_WWDG = 4096 * (counter + 1) / (HCLK / 4096) * prescaler
 *           = 4096 * 128 / 58593.75 * 8 =~ 71.6 ms  (maximum period)
 *    Window set to 80 -> earliest refresh when counter <= 80.
 *    This prevents a runaway loop from refreshing too early.
 *============================================================================*/
void Safety_WWDT_Init(void)
{
    /* Enable WWDG clock */
    __HAL_RCC_WWDG_CLK_ENABLE();

    /* WWDG_CFR: prescaler = 8, window value = 80, early wake interrupt */
    WWDG->CFR = (7U << WWDG_CFR_WDGTB_Pos)   /* prescaler = 8           */
              | (80U << WWDG_CFR_W_Pos)       /* window upper bound      */
              | WWDG_CFR_EWI;                 /* early wake interrupt    */

    /* WWDG_CR: activate watchdog, counter = 127 (7-bit max) */
    WWDG->CR = (127U << WWDG_CR_T_Pos) | WWDG_CR_WDGA;

    /* NVIC priority: highest (0) for safety */
    HAL_NVIC_SetPriority(WWDG_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(WWDG_IRQn);
}

/*============================================================================
 *  Safety_Monitor -- called every 10 ms by vSafetyWdTask
 *
 *  HSEM(0) heartbeat check:
 *    M7 periodically takes HSEM(0) in its main loop.
 *    - If M4 finds HSEM(0) taken    -> M7 is alive -> reset counter
 *    - If M4 finds HSEM(0) free     -> M7 may be dead -> increment counter
 *    - If counter exceeds threshold  -> declare M7 dead, kill PWM
 *
 *  Additional robustness: read m7_heartbeat_cnt from SRAM3.
 *    If the counter hasn't changed AND HSEM(0) is free -> M7 dead.
 *============================================================================*/
void Safety_Monitor(void)
{
    uint8_t m7_dead = 0;

    /* -----------------------------------------------------------------------
     * 1. HSEM(0) check
     * ----------------------------------------------------------------------- */
    if (HAL_HSEM_IsSemTaken(HSEM_HEARTBEAT)) {
        /* M7 has refreshed the semaphore -- alive */
        m7_alive_cnt = 0;
    } else {
        m7_alive_cnt++;
        if (m7_alive_cnt > 5) {   /* 6 consecutive misses = 60 ms */
            m7_dead = 1;
        }
    }

    /* -----------------------------------------------------------------------
     * 2. SRAM3 heartbeat counter check (belt-and-suspenders)
     * ----------------------------------------------------------------------- */
    {
        uint32_t curr_hb = SHMEM->m7_heartbeat_cnt;
        if (curr_hb != last_hb_cnt) {
            /* M7 has written to SRAM3 -- definitely alive */
            last_hb_cnt = curr_hb;
            m7_alive_cnt = 0;
            m7_dead = 0;
        }
        last_hb_cnt = curr_hb;
    }

    /* -----------------------------------------------------------------------
     * 3. Take action if M7 is dead
     * ----------------------------------------------------------------------- */
    if (m7_dead && !m7_was_dead) {
        m7_was_dead = 1;

        /* HARDWARE PWM KILL -- GPIO direct, < 200 ns propagation */
        HAL_GPIO_WritePin(PWM_SHUT_PORT, PWM_SHUT_PIN, GPIO_PIN_SET);

        /* SOFTWARE PWM KILL -- disable TIM1 update interrupt */
        NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);

        /* Force TIM1 break via software (BDTR.BK bit) -- redundant safety */
        TIM1->BDTR |= TIM_BDTR_BK;

        /* Log fault */
        SHMEM->fault_code = 0xA1;   /* M7 heartbeat loss */

        /* Transition fail-safe state */
        fail_safe_state = FS_SPIN_HOME;   /* stop motors, spin home */

        /* Disable WWDG early-wake -- let WWDG reset us if we hang too */
        WWDG->CFR &= ~WWDG_CFR_EWI;
    }
    else if (!m7_dead && m7_was_dead) {
        /* M7 recovered -- restore normal operation */
        m7_was_dead = 0;

        /* De-assert PWM shutdown (re-enable power) */
        HAL_GPIO_WritePin(PWM_SHUT_PORT, PWM_SHUT_PIN, GPIO_PIN_RESET);

        /* Re-enable TIM1 */
        TIM1->BDTR &= ~TIM_BDTR_BK;
        NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

        /* Re-enable WWDG early-wake */
        WWDG->CFR |= WWDG_CFR_EWI;

        fail_safe_state = FS_NORMAL;
        SHMEM->fault_code = 0;
    }

    /* Update shared memory */
    SHMEM->fail_safe_state = fail_safe_state;
}

/*============================================================================
 *  Safety_Failsafe -- force a specific fail-safe state
 *
 *  Called from other modules on critical faults (clock failure, BMS OVP,
 *  ToF timeout, etc.).
 *============================================================================*/
void Safety_Failsafe(uint8_t state)
{
    fail_safe_state = state;

    /* Immediate PWM shutdown */
    HAL_GPIO_WritePin(PWM_SHUT_PORT, PWM_SHUT_PIN, GPIO_PIN_SET);
    NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
    TIM1->BDTR |= TIM_BDTR_BK;

    SHMEM->fault_code = state;
    SHMEM->fail_safe_state = state;
}

/*============================================================================
 *  Safety_GetFailSafeState
 *============================================================================*/
uint8_t Safety_GetFailSafeState(void)
{
    return fail_safe_state;
}

/*============================================================================
 *  Safety_IsM7Alive
 *============================================================================*/
uint8_t Safety_IsM7Alive(void)
{
    return (m7_alive_cnt <= 5) ? 1 : 0;
}

/*============================================================================
 *  WWDG_IRQHandler -- Early Wake Interrupt
 *
 *  Called just before WWDG would reset the MCU.  We may refresh the
 *  watchdog if the system is healthy (fail_safe_state == FS_NORMAL).
 *  If we are in a fail-safe state, let the reset happen.
 *============================================================================*/
void WWDG_IRQHandler(void)
{
    if (WWDG->SR & WWDG_SR_EWIF) {
        WWDG->SR = ~WWDG_SR_EWIF;   /* clear flag */

        if (fail_safe_state == FS_NORMAL) {
            /* Refresh: counter = 127 */
            WWDG->CR = (127U << WWDG_CR_T_Pos) | WWDG_CR_WDGA;
            wwdg_refresh_ok++;
        }
        /* else: let the WWDG reset us -- we are in a bad state */
    }
}

/*============================================================================
 *  HAL_HSEM wrappers
 *============================================================================*/
int32_t Safety_TakeHSEM(uint32_t sem_id)
{
    return HAL_HSEM_FastTake(sem_id);
}

void Safety_ReleaseHSEM(uint32_t sem_id)
{
    HAL_HSEM_Release(sem_id);
}
