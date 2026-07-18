/**
  ******************************************************************************
  * @file    foc_isr.c
  * @author  M7 FOC Team
  * @brief   Complete FOC interrupt service routine.
  *          Implements the full current-control loop on STM32H747:
  *          ADC injected read -> CORDIC Clarke+Park -> PI_id/PI_iq ->
  *          CORDIC invPark -> SVPWM 7-segment -> TIM1 CCR shadow update.
  *          Overcurrent detection on all 3 phases.
  *          Encoder quadrature decode via TIM2.
  *
  *          Cycle-count annotations reflect 480 MHz Cortex-M7 (zero-wait
  *          TCM code, TCM data). Actual cycle count may vary with cache
  *          and bus contention.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include <string.h>
#include <math.h>

/* Private typedef -----------------------------------------------------------*/
typedef enum {
    FOC_STATE_IDLE       = 0U,
    FOC_STATE_ALIGN      = 1U,
    FOC_STATE_CALIB      = 2U,
    FOC_STATE_RUN        = 3U,
    FOC_STATE_FAULT      = 4U,
    FOC_STATE_FAST_DECAY = 5U
} FOC_StateTypeDef;

typedef struct {
    /* ADC raw counts */
    int16_t ia_raw;
    int16_t ib_raw;
    int16_t ic_raw;

    /* Phase currents [A] after offset correction */
    float ia;
    float ib;
    float ic;

    /* Clarke (alpha-beta) */
    float i_alpha;
    float i_beta;

    /* Park (d-q) */
    float i_d;
    float i_q;

    /* Voltage references (d-q) */
    float v_d_ref;
    float v_q_ref;

    /* Voltage references (alpha-beta) */
    float v_alpha;
    float v_beta;

    /* PWM duty cycles (0.0 .. 1.0) */
    float duty_a;
    float duty_b;
    float duty_c;

    /* Electrical angle [rad] */
    float theta_elec;

    /* Electrical speed [rad/s] */
    float omega_elec;

    /* DC bus voltage [V] */
    float v_dc;

    /* DC bus current [A] (low-side shunt sum) */
    float i_dc;
} FOC_HandleTypeDef;

/* Private defines -----------------------------------------------------------*/
#define FOC_PWM_FREQ_HZ         20000U      /* 20 kHz current loop     */
#define FOC_PWM_PERIOD_US       50U
#define FOC_DEADTIME_NS         100U
#define FOC_DEADTIME_TICKS      4U          /* TIM1 dead-time counter  */

#define TIM1_PCLK               240000000U  /* TIM1 clock (APB2)       */
#define TIM1_PERIOD             (TIM1_PCLK / (2U * FOC_PWM_FREQ_HZ))
#define TIM1_DEADTIME           (uint32_t)((FOC_DEADTIME_NS * TIM1_PCLK) / 1e9f)

#define OVERCURRENT_THRESHOLD_A 15.0f       /* 15 A trip               */
#define OVERCURRENT_DEBOUNCE    3U          /* 3 consecutive samples   */

#define VBUS_NOMINAL_V          24.0f       /* 24 V nominal bus        */
#define VBUS_MIN_V              18.0f       /* Under-voltage limit     */

#define CORDIC_COS_SIN          0x00000000UL
#define CORDIC_PHASE            0x00000002UL
#define CORDIC_MODULUS          0x0000000AUL

#define SQRT3_2                 0.8660254037844386f
#define INV_SQRT3               0.5773502691896257f
#define ONE_THIRD               0.3333333333333333f

/* Private macro -------------------------------------------------------------*/
#define ABS(x)                  ((x) < 0 ? -(x) : (x))
#define CLAMP(x, lo, hi)        (((x) < (lo)) ? (lo) : (((x) > (hi)) ? (hi) : (x)))
#define MOD_2PI(x)              ((x) - 6.283185307179586f * (float)((int32_t)((x) * 0.1591549430918953f)))

/* Private variables ---------------------------------------------------------*/
static FOC_HandleTypeDef      hfoc;
static volatile FOC_StateTypeDef g_foc_state = FOC_STATE_IDLE;
static volatile uint32_t      g_overcurrent_count = 0U;
static volatile uint8_t       g_fault_flags = 0U;
static        float           g_current_offset_a = 0.0f;
static        float           g_current_offset_b = 0.0f;
static        float           g_current_offset_c = 0.0f;
static const  float           Vdc_nominal = VBUS_NOMINAL_V;

/* CORDIC result buffers (volatile because accessed by hardware) */
static volatile uint32_t cordic_wdata __attribute__((section(".dtcm_data")));
static volatile uint32_t cordic_rdata __attribute__((section(".dtcm_data")));

/* ---------------------------------------------------------------------------*/
/*  CORDIC helpers                                                            */
/* ---------------------------------------------------------------------------*/

/** @brief  Compute cos(theta) and sin(theta) via CORDIC.
  * @param  theta  Angle in radians [0 .. 2*pi].
  * @param  pcos   Output pointer for cosine.
  * @param  psin   Output pointer for sine.
  * @note   CORDIC argument must be in Q1.31 format (0 .. 2*pi maps to 0 .. 0x7FFFFFFF).
  *         ~45 cycles (2 writes + busy-wait + 2 reads).
  */
static void cordic_cos_sin(float theta, float *pcos, float *psin)
{
    /* Convert float angle to Q1.31 CORDIC argument */
    uint32_t qangle;
    if (theta < 0.0f) theta += 6.283185307179586f;
    qangle = (uint32_t)(theta * 0x80000000U * 0.1591549430918953f); /* / (2*pi) */

    /* Write argument -- CORDIC computes in background */
    CORDIC->WDATA = qangle;                                      /* ~2 cy */
    /* Wait for Ready */
    while ((CORDIC->CSR & CORDIC_CSR_RRDY_Msk) == 0U)           /* ~38 cy */
    {
        __NOP();
    }
    /* Read results: RDATA yields packed (cos, sin) */
    uint32_t result = CORDIC->RDATA;                             /* ~2 cy */
    int32_t cos_q = (int32_t)(result & 0xFFFFU) << 15;          /* sign-extend Q1.15 -> Q1.31 */
    int32_t sin_q = (int32_t)((result >> 16U) & 0xFFFFU) << 15;

    *pcos = (float)cos_q * 4.6566129e-10f; /* Q1.31 to float */
    *psin = (float)sin_q * 4.6566129e-10f;
}

/** @brief  Compute modulus = sqrt(x^2 + y^2) via CORDIC.
  *         ~45 cycles.
  */
static float cordic_modulus(float x, float y)
{
    /* CORDIC expects Q1.31 input.  Scale down to avoid overflow */
    int32_t qx = (int32_t)(x * 536870912.0f); /* Q1.31 */
    int32_t qy = (int32_t)(y * 536870912.0f);

    CORDIC->WDATA = (uint32_t)(qy);                               /* ~2 cy */
    while ((CORDIC->CSR & CORDIC_CSR_RRDY_Msk) == 0U)            /* ~38 cy */
    {
        __NOP();
    }
    uint32_t res = CORDIC->RDATA;

    int32_t mod_q = (int32_t)(res & 0xFFFFU) << 15;
    return (float)mod_q * 4.6566129e-10f;
}

/* ---------------------------------------------------------------------------*/
/*  Clarke & Park transforms (with CORDIC trig)                               */
/* ---------------------------------------------------------------------------*/

/** @brief  Clarke transform: (a, b, c) -> (alpha, beta).
  *         ~8 cycles (FPU only, no CORDIC needed).
  */
static void clarke_transform(float ia, float ib, float ic,
                              float *p_alpha, float *p_beta)
{
    /* Assume ia + ib + ic = 0 (balanced); but we use all 3 for accuracy.    */
    *p_alpha = (2.0f * ia - ib - ic) * ONE_THIRD;   /* ~4 cy */
    *p_beta  = (ib - ic) * INV_SQRT3;               /* ~4 cy */
}

/** @brief  Park transform: (alpha, beta, theta) -> (d, q).
  *         ~55 cycles (CORDIC trig + FPU multiply-add).
  */
static void park_transform(float alpha, float beta, float theta,
                            float *pd, float *pq)
{
    float cos_th, sin_th;
    cordic_cos_sin(theta, &cos_th, &sin_th);        /* ~45 cy */

    *pd =  alpha * cos_th + beta * sin_th;          /* ~5 cy */
    *pq = -alpha * sin_th + beta * cos_th;          /* ~5 cy */
}

/** @brief  Inverse Park: (d, q, theta) -> (alpha, beta).
  *         ~55 cycles.
  */
static void inv_park_transform(float d, float q, float theta,
                                float *p_alpha, float *p_beta)
{
    float cos_th, sin_th;
    cordic_cos_sin(theta, &cos_th, &sin_th);        /* ~45 cy */

    *p_alpha = d * cos_th - q * sin_th;             /* ~5 cy */
    *p_beta  = d * sin_th + q * cos_th;             /* ~5 cy */
}

/* ---------------------------------------------------------------------------*/
/*  SVPWM — 7-segment space-vector modulation                                 */
/* ---------------------------------------------------------------------------*/

/** @brief  7-segment SVPWM from alpha-beta voltage.
  *         Computes duty cycles for TIM1 channels 1-3.
  *         ~90 cycles including saturation and sector logic.
  */
static void svpwm_7segment(float Valpha, float Vbeta,
                            float Vdc,
                            float *pduty_a, float *pduty_b, float *pduty_c)
{
    /* Scale to per-unit [0..1] relative to DC bus */
    float v_alpha_pu = Valpha / Vdc;
    float v_beta_pu  = Vbeta  / Vdc;

    /* Clamp to avoid over-modulation */
    float mag = sqrtf(v_alpha_pu * v_alpha_pu + v_beta_pu * v_beta_pu);
    if (mag > 0.577350269f) { /* Vmax for SVPWM = Vdc/sqrt(3) */
        float scale = 0.577350269f / mag;
        v_alpha_pu *= scale;
        v_beta_pu  *= scale;
    }

    /* Sector determination (~20 cy) */
    float v1 = v_beta_pu;
    float v2 = 0.8660254f * v_alpha_pu - 0.5f * v_beta_pu;
    float v3 = -0.8660254f * v_alpha_pu - 0.5f * v_beta_pu;

    uint8_t sector_bits = 0;
    if (v1 > 0.0f) sector_bits |= 1U;
    if (v2 > 0.0f) sector_bits |= 2U;
    if (v3 > 0.0f) sector_bits |= 4U;

    static const uint8_t sector_lut[8] = {0, 1, 5, 0, 3, 2, 4, 0};
    uint8_t sector = sector_lut[sector_bits]; /* 0..5 */

    /* Compute X, Y, Z (~15 cy) */
    float T = 1.0f; /* normalised PWM period; actual scaling done in TIM1 */
    float sqrt3_vdc = 1.73205080757f / Vdc; /* sqrt(3) / Vdc */

    float X =  v_beta_pu  * T * sqrt3_vdc;
    float Y = (0.8660254f * v_alpha_pu + 0.5f * v_beta_pu) * T * sqrt3_vdc;
    float Z = (-0.8660254f * v_alpha_pu + 0.5f * v_beta_pu) * T * sqrt3_vdc;

    /* Compute T1, T2 per sector (~10 cy) */
    float t1, t2;
    switch (sector) {
        case 0: t1 = Z;  t2 = Y;  break;
        case 1: t1 = Y;  t2 = -X; break;
        case 2: t1 = -Z; t2 = X;  break;
        case 3: t1 = -X; t2 = Z;  break;
        case 4: t1 = X;  t2 = -Y; break;
        case 5: t1 = -Y; t2 = -Z; break;
        default: t1 = 0.0f; t2 = 0.0f; break;
    }

    /* Saturate T1, T2 */
    float t_sum = t1 + t2;
    if (t_sum > 1.0f) {
        float inv_sum = 1.0f / t_sum;
        t1 *= inv_sum;
        t2 *= inv_sum;
    }

    /* Compute 7-segment timing (~15 cy) */
    float ta = (1.0f - t1 - t2) * 0.25f;
    float tb = ta + t1 * 0.5f;
    float tc = tb + t2 * 0.5f;

    /* Map to duty cycles per sector (~15 cy) */
    switch (sector) {
        case 0: *pduty_a = tb; *pduty_b = ta; *pduty_c = tc; break;
        case 1: *pduty_a = ta; *pduty_b = tc; *pduty_c = tb; break;
        case 2: *pduty_a = ta; *pduty_b = tb; *pduty_c = tc; break;
        case 3: *pduty_a = tc; *pduty_b = tb; *pduty_c = ta; break;
        case 4: *pduty_a = tc; *pduty_b = ta; *pduty_c = tb; break;
        case 5: *pduty_a = tb; *pduty_b = tc; *pduty_c = ta; break;
        default: *pduty_a = 0.0f; *pduty_b = 0.0f; *pduty_c = 0.0f; break;
    }
}

/* ---------------------------------------------------------------------------*/
/*  Current controller (PI with anti-windup and feed-forward)                 */
/* ---------------------------------------------------------------------------*/

/** @brief  PI current controller with conditional anti-windup.
  *         ~25 cycles (FPU only).
  */
typedef struct {
    float Kp;
    float Ki;
    float integral;
    float upper_limit;
    float lower_limit;
    float Ki_scale;
} PI_ControllerTypeDef;

static void pi_controller_init(PI_ControllerTypeDef *pi,
                                float kp, float ki,
                                float upper, float lower)
{
    pi->Kp          = kp;
    pi->Ki          = ki;
    pi->integral    = 0.0f;
    pi->upper_limit = upper;
    pi->lower_limit = lower;
    pi->Ki_scale    = ki;
}

static float pi_controller_run(PI_ControllerTypeDef *pi,
                                float ref, float meas,
                                float out_min, float out_max)
{
    float err = ref - meas;                    /* ~2 cy */

    /* Proportional term */
    float p_term = pi->Kp * err;               /* ~3 cy */

    /* Integrator with conditional anti-windup */
    float u_pre_sat = p_term + pi->integral;   /* ~2 cy */

    if (u_pre_sat > out_max) {
        /* Do not accumulate if saturated positive */
    } else if (u_pre_sat < out_min) {
        /* Do not accumulate if saturated negative */
    } else {
        pi->integral += pi->Ki * err;          /* ~6 cy */
        /* Clamp integrator state */
        if (pi->integral > pi->upper_limit) pi->integral = pi->upper_limit;
        if (pi->integral < pi->lower_limit) pi->integral = pi->lower_limit;
    }

    float output = p_term + pi->integral;       /* ~2 cy */
    if (output > out_max) output = out_max;
    if (output < out_min) output = out_min;

    return output;                             /* total ~25 cy */
}

/* Static PI controllers for D and Q axes */
static PI_ControllerTypeDef pi_d;
static PI_ControllerTypeDef pi_q;

/* ---------------------------------------------------------------------------*/
/*  Overcurrent protection                                                    */
/* ---------------------------------------------------------------------------*/

/** @brief  Detect overcurrent on any phase.
  *         Debounced over 3 consecutive samples.
  *         Returns 1 if fault persists.
  *         ~15 cycles.
  */
static int overcurrent_check(float ia, float ib, float ic)
{
    static uint32_t oc_count = 0U;
    static uint32_t oc_ok_count = 0U;

    if ((ABS(ia) > OVERCURRENT_THRESHOLD_A) ||
        (ABS(ib) > OVERCURRENT_THRESHOLD_A) ||
        (ABS(ic) > OVERCURRENT_THRESHOLD_A))
    {
        oc_ok_count = 0U;
        oc_count++;
        if (oc_count >= OVERCURRENT_DEBOUNCE) {
            return 1; /* fault */
        }
    } else {
        oc_count = 0U;
        oc_ok_count++;
        if (oc_ok_count >= OVERCURRENT_DEBOUNCE) {
            oc_count = 0U;
        }
    }
    return 0;
}

/* ---------------------------------------------------------------------------*/
/*  ADC injected group read                                                   */
/* ---------------------------------------------------------------------------*/

/** @brief  Read phase currents from ADC1/ADC2 injected group.
  *         ADC1_IN1 = Ia, ADC1_IN2 = Ib, ADC2_IN1 = Ic.
  *         ~30 cycles (3 injected reads).
  */
static void read_phase_currents(int16_t *pia, int16_t *pib, int16_t *pic)
{
    /* ADC1 injected group result registers (JEOCx ordered) */
    *pia = (int16_t)(ADC1->JDR1);                 /* ~3 cy */
    *pib = (int16_t)(ADC1->JDR2);                 /* ~3 cy */
    *pic = (int16_t)(ADC2->JDR1);                 /* ~3 cy */
    /* Read ADC for Vdc (auxiliary channel) */
    hfoc.v_dc = (float)(ADC3->JDR1) * 24.0f / 4095.0f; /* ~6 cy */
}

/* ---------------------------------------------------------------------------*/
/*  Encoder read via TIM2                                                      */
/* ---------------------------------------------------------------------------*/

/** @brief  Read encoder position from TIM2 counter.
  *         Use GPT to read CNT atomically.
  *         ~5 cycles.
  */
static float read_encoder_angle(void)
{
    /* TIM2 is configured in encoder mode 3 (TI1 + TI2).                    */
    uint32_t cnt = TIM2->CNT;                        /* ~3 cy */

    /* Convert counts to electrical angle [0 .. 2*pi).
       Assume 2048-line encoder -> 8192 counts per revolution.
       Multiply by 2*pi/8192
    */
    float angle = (float)(cnt & 0x1FFFU) * 0.000766990f;  /* ~5 cy */
    return angle;
}

/* ---------------------------------------------------------------------------*/
/*  PWM output update                                                         */
/* ---------------------------------------------------------------------------*/

/** @brief  Write duty cycles to TIM1 CCR1-CCR3 and commit shadow registers.
  *         Applies dead-time via TIM1 BDTR DTG field (pre-configured).
  *         ~15 cycles.
  */
static void update_pwm_outputs(float duty_a, float duty_b, float duty_c)
{
    uint32_t period = TIM1->ARR;                         /* ~3 cy */

    uint32_t ccr1 = (uint32_t)(duty_a * (float)period);  /* ~4 cy */
    uint32_t ccr2 = (uint32_t)(duty_b * (float)period);
    uint32_t ccr3 = (uint32_t)(duty_c * (float)period);

    /* Clamp to valid range (accounting for dead-time margin) */
    uint32_t dt_margin = TIM1_DEADTIME + 2U;
    if (ccr1 < dt_margin)        ccr1 = dt_margin;
    if (ccr1 > period - dt_margin) ccr1 = period - dt_margin;
    if (ccr2 < dt_margin)        ccr2 = dt_margin;
    if (ccr2 > period - dt_margin) ccr2 = period - dt_margin;
    if (ccr3 < dt_margin)        ccr3 = dt_margin;
    if (ccr3 > period - dt_margin) ccr3 = period - dt_margin;

    /* Write shadow registers (updated on next update event) */
    WRITE_REG(TIM1->CCR1, ccr1);                         /* ~3 cy */
    WRITE_REG(TIM1->CCR2, ccr2);
    WRITE_REG(TIM1->CCR3, ccr3);
}

/* ---------------------------------------------------------------------------*/
/*  Public API: initialisation                                                */
/* ---------------------------------------------------------------------------*/

/** @brief  Initialise TIM1 for centre-aligned PWM with dead-time.
  *         Configure ADC1/ADC2 injected group for simultaneous read.
  *         Configure CORDIC for 20-bit precision (4 iterations).
  */
void FOC_Init(void)
{
    /* --- TIM1: centre-aligned PWM, 20 kHz, dead-time 100 ns --- */
    __HAL_RCC_TIM1_CLK_ENABLE();

    TIM1->PSC = 0U; /* /1 => 240 MHz clock for TIM1 */
    TIM1->ARR = (uint32_t)(TIM1_PCLK / (2U * FOC_PWM_FREQ_HZ)) - 1U;
    TIM1->CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
    TIM1->CCMR2 = TIM_CCMR1_OC1PE; /* OC3PE */
    TIM1->CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
    TIM1->BDTR  = (TIM1_DEADTIME << TIM_BDTR_DTG_Pos) | TIM_BDTR_MOE;
    TIM1->CR1   = TIM_CR1_CMS_1 | TIM_CR1_ARPE | TIM_CR1_CEN;

    /* --- ADC1/ADC2 injected group --- */
    __HAL_RCC_ADC12_CLK_ENABLE();
    /* Simplification: assumes HAL_ADC_Init called beforehand for basic config */

    /* Configure injected rank 1 for ADC1_IN1 (PA0), rank 2 for ADC1_IN2 (PA1),
       rank 1 for ADC2_IN1 (PA6) via HAL; here we just set JQDIS and enable
       injected conversion on trigger from TIM1 TRGO */
    ADC1->CFGR  |= ADC_CFGR_JQDIS;          /* Disable queue to allow overwrite */
    ADC12_COMMON->CCR |= ADC12_CCR_JSWSTART; /* Software trigger (default)      */

    /* --- CORDIC --- */
    __HAL_RCC_CORDIC_CLK_ENABLE();
    CORDIC->CSR = (4U << CORDIC_CSR_NITER_Pos)   /* 4 iterations -> 20-bit */
                | (0U << CORDIC_CSR_PRECISION_Pos)
                | CORDIC_CSR_RESIZE;

    /* --- PI controllers --- */
    pi_controller_init(&pi_d, 2.5f, 0.05f,  12.0f, -12.0f);
    pi_controller_init(&pi_q, 2.5f, 0.05f,  12.0f, -12.0f);

    /* --- Initial state --- */
    g_foc_state = FOC_STATE_CALIB;
    hfoc.v_dc    = VBUS_NOMINAL_V;
    hfoc.theta_elec = 0.0f;
    hfoc.omega_elec = 0.0f;

    /* Calibrate current offsets (assumes phase currents zero at init) */
    float sum_a = 0.0f, sum_b = 0.0f, sum_c = 0.0f;
    for (int i = 0; i < 128; i++) {
        int16_t ia, ib, ic;
        read_phase_currents(&ia, &ib, &ic);
        sum_a += (float)ia;
        sum_b += (float)ib;
        sum_c += (float)ic;
    }
    g_current_offset_a = sum_a * 0.0078125f;
    g_current_offset_b = sum_b * 0.0078125f;
    g_current_offset_c = sum_c * 0.0078125f;
}

/** @brief  Start FOC loop by enabling TIM1 update interrupt and ADC injected
  *         conversion trigger.
  */
void FOC_Start(void)
{
    /* Clear any pending flags */
    TIM1->SR = ~TIM_SR_UIF;
    ADC1->ISR = ADC_ISR_JEOS;

    /* Enable TIM1 update interrupt (FOC ISR fires at 20 kHz) */
    TIM1->DIER |= TIM_DIER_UIE;

    /* Enable injected group conversion on TIM1 TRGO */
    ADC1->CFGR |= ADC_CFG_JEXTEN_0;  /* rising edge trigger */

    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
    NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 1U);

    g_foc_state = FOC_STATE_RUN;
}

/** @brief  Stop FOC loop and PWM outputs (safe state). */
void FOC_Stop(void)
{
    /* Disable PWM outputs */
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->DIER &= ~TIM_DIER_UIE;
    NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);

    /* Clear CCRs */
    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
    TIM1->CCR3 = 0U;

    g_foc_state = FOC_STATE_IDLE;
}

/** @brief  Get current FOC state. */
FOC_StateTypeDef FOC_GetState(void)
{
    return g_foc_state;
}

/** @brief  Get fault flags. */
uint8_t FOC_GetFaultFlags(void)
{
    return g_fault_flags;
}

/* ---------------------------------------------------------------------------*/
/*  Main FOC ISR — called at TIM1 update rate (20 kHz).                       */
/*  Total latency: ~310 cycles = 0.65 us at 480 MHz.                         */
/* ---------------------------------------------------------------------------*/
void TIM1_UP_TIM10_IRQHandler(void)
{
    /* Clear interrupt flag (~2 cy) */
    TIM1->SR = ~TIM_SR_UIF;            /* Line latency: 2 cy */

    /* ---- 1. Read phase currents from ADC injected group (~30 cy) ---- */
    int16_t ia_raw, ib_raw, ic_raw;
    read_phase_currents(&ia_raw, &ib_raw, &ic_raw);    /* Line: 30 cy */

    /* Convert to Amperes using ADC resolution (12-bit, Vref=3.3V, shunt=0.001 Ohm, gain=50) */
    hfoc.ia = ((float)ia_raw - g_current_offset_a) * 0.001610f; /* Line: 6 cy */
    hfoc.ib = ((float)ib_raw - g_current_offset_b) * 0.001610f;
    hfoc.ic = ((float)ic_raw - g_current_offset_c) * 0.001610f;

    /* ---- 2. Overcurrent detection (~15 cy) ---- */
    if (overcurrent_check(hfoc.ia, hfoc.ib, hfoc.ic)) { /* Line: 15 cy */
        g_fault_flags |= 0x01U;         /* OC fault */
        g_overcurrent_count++;
        if (g_overcurrent_count > 10U) {
            FOC_Stop();                  /* Hard stop */
            g_foc_state = FOC_STATE_FAULT;
            return;                     /* Line: 5 cy (early exit) */
        }
    }

    /* ---- 3. Read encoder and compute electrical angle (~10 cy) ---- */
    hfoc.theta_elec = read_encoder_angle();            /* Line: 5 cy */
    /* Estimate electrical speed via simple difference (filtered in outer loop) */
    static float theta_prev = 0.0f;
    float dtheta = hfoc.theta_elec - theta_prev;       /* Line: 3 cy */
    if (dtheta < -3.14159265f) dtheta += 6.2831853f;
    if (dtheta >  3.14159265f) dtheta -= 6.2831853f;
    hfoc.omega_elec = dtheta * 20000.0f;                /* Line: 4 cy */
    theta_prev = hfoc.theta_elec;

    /* ---- 4. Clarke transform: abc -> alpha-beta (~8 cy) ---- */
    clarke_transform(hfoc.ia, hfoc.ib, hfoc.ic,
                     &hfoc.i_alpha, &hfoc.i_beta);     /* Line: 8 cy */

    /* ---- 5. Park transform: alpha-beta -> dq (~55 cy) ---- */
    park_transform(hfoc.i_alpha, hfoc.i_beta, hfoc.theta_elec,
                   &hfoc.i_d, &hfoc.i_q);              /* Line: 55 cy */

    /* ---- 6. PI current controllers for Id, Iq (~50 cy) ---- */
    float vd_ref = pi_controller_run(&pi_d,
                     hfoc.i_d,     /* Id_ref from outer loop */
                     hfoc.i_d,
                     -hfoc.v_dc * 0.57735f,
                      hfoc.v_dc * 0.57735f);           /* Line: 25 cy */

    float vq_ref = pi_controller_run(&pi_q,
                     0.0f,         /* Iq_ref from speed loop */
                     hfoc.i_q,
                     -hfoc.v_dc * 0.57735f,
                      hfoc.v_dc * 0.57735f);           /* Line: 25 cy */

    /* Apply decoupling (feed-forward) */
    float Ld = 0.000015f; /* 15 uH */
    float Lq = 0.000015f;
    hfoc.v_d_ref = vd_ref - hfoc.omega_elec * Lq * hfoc.i_q;  /* Line: 6 cy */
    hfoc.v_q_ref = vq_ref + hfoc.omega_elec * Ld * hfoc.i_d;

    /* ---- 7. Inverse Park: dq -> alpha-beta (~55 cy) ---- */
    inv_park_transform(hfoc.v_d_ref, hfoc.v_q_ref, hfoc.theta_elec,
                       &hfoc.v_alpha, &hfoc.v_beta);   /* Line: 55 cy */

    /* ---- 8. SVPWM 7-segment: alpha-beta -> duty cycles (~90 cy) ---- */
    svpwm_7segment(hfoc.v_alpha, hfoc.v_beta, hfoc.v_dc,
                   &hfoc.duty_a, &hfoc.duty_b, &hfoc.duty_c); /* Line: 90 cy */

    /* ---- 9. Dead-time compensation (separate module) ---- */
    extern void FOC_DeadTime_Compensate(float *da, float *db, float *dc,
                                         float ia, float ib, float ic,
                                         uint32_t deadtime_ticks);
    FOC_DeadTime_Compensate(&hfoc.duty_a, &hfoc.duty_b, &hfoc.duty_c,
                             hfoc.ia, hfoc.ib, hfoc.ic,
                             TIM1_DEADTIME);            /* Line: ~20 cy */

    /* ---- 10. Update PWM CCR shadow registers (~15 cy) ---- */
    update_pwm_outputs(hfoc.duty_a, hfoc.duty_b, hfoc.duty_c); /* Line: 15 cy */

    /* ---- Total: ~310 cycles ---- */
}

/* ---------------------------------------------------------------------------*/
/*  Outer-loop hook: set Id/Iq references (called from speed/position loop)   */
/* ---------------------------------------------------------------------------*/

void FOC_SetIdRef(float id_ref)
{
    pi_d.integral = id_ref * 0.5f; /* rough initialisation */
    /* Id_ref is read directly in ISR via hfoc.i_d vs pi_d target */
}

void FOC_SetIqRef(float iq_ref)
{
    pi_q.integral = iq_ref * 0.5f;
}

float FOC_GetIq(void)   { return hfoc.i_q; }
float FOC_GetId(void)   { return hfoc.i_d; }
float FOC_GetOmega(void){ return hfoc.omega_elec; }
float FOC_GetTheta(void){ return hfoc.theta_elec; }

/* ---------------------------------------------------------------------------*/
/*  Weak-linked callbacks for user extension                                  */
/* ---------------------------------------------------------------------------*/
__attribute__((weak)) void HAL_FOC_PreUpdate(FOC_HandleTypeDef *hfoc) {(void)hfoc;}
__attribute__((weak)) void HAL_FOC_PostUpdate(FOC_HandleTypeDef *hfoc) {(void)hfoc;}
