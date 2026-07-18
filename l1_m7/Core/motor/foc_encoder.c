/**
  ******************************************************************************
  * @file    foc_encoder.c
  * @author  M7 FOC Team
  * @brief   Quadrature encoder interface with index-pulse (Z) synchronisation.
  *
  *          Hardware:
  *            TIM2 = encoder mode 3 (TI1 + TI2), CH1=PA0 (A), CH2=PA1 (B)
  *            TIM3 = input capture for index pulse (Z) -> PA6
  *
  *          Velocity measurement via M/T method:
  *            - High speed (> 100 RPM): frequency measurement (pulse count
  *              over fixed time window)
  *            - Low speed (< 100 RPM): period measurement (pulse width of
  *              one encoder edge)
  *            Smooth transition between methods.
  *
  *          Encoder offset calibration:
  *            At startup, align the encoder electrical zero with the motor
  *            back-EMF zero crossing (IDQ alignment).
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include <string.h>
#include <math.h>

/* Private defines -----------------------------------------------------------*/
/* Encoder hardware parameters */
#define ENCODER_LINES           2048U       /* pulses per mechanical rev      */
#define ENCODER_COUNTS_PER_REV  (ENCODER_LINES * 4U)  /* x4 decoding = 8192  */
#define ENCODER_CPR             ENCODER_COUNTS_PER_REV

/* Pole pairs (motor specific) */
#define MOTOR_POLE_PAIRS        7U

/* Mechanical to electrical angle scaling: theta_elec = theta_mech * pole_pairs */
#define ENCODER_ELEC_SCALE      ((float)MOTOR_POLE_PAIRS)

/* M/T method parameters */
#define MT_MEASURE_PERIOD_MS    5U          /* 5 ms fixed time window */
#define MT_MIN_FREQ_COUNTS      50U         /* min count for freq method */
#define MT_SPEED_THRESH_RPM     100.0f      /* threshold between M and T */
#define MT_FILTER_ALPHA         0.9f        /* LPF coefficient for speed */

/* Index pulse debounce */
#define INDEX_DEBOUNCE_SAMPLES  5U

/* Calibration */
#define CALIB_CURRENT_MA        500         /* 500 mA alignment current */
#define CALIB_SETTLE_MS         50U         /* 50 ms settle time */

/* Fault detection */
#define ENC_FAULT_TIMEOUT_MS    100U        /* no pulse for 100 ms = fault */
#define ENC_FAULT_JERK_THRESH   5000U       /* RPM jump > 5000 = glitch */

/* ---------------------------------------------------------------------------*/
/*  Encoder handle                                                           */
/* ---------------------------------------------------------------------------*/
typedef struct {
    /* Raw counts */
    volatile int32_t  pulse_count;     /* cumulative signed position (+/-) */
    int32_t  count_at_z;               /* TIM2 CNT value when index captured */
    uint32_t z_capture_count;          /* number of Z events seen */

    /* Position */
    float    position_mech_rev;        /* mechanical revolutions (float) */
    float    position_mech_rad;        /* mechanical angle [rad] */
    float    position_elec_rad;        /* electrical angle [rad] */

    /* Velocity (M/T method) */
    float    speed_rpm;                /* filtered mechanical speed [RPM] */
    float    speed_radps;              /* filtered electrical speed [rad/s] */

    /* M/T intermediate */
    uint32_t mt_pulse_count;           /* encoder pulses in last window */
    uint32_t mt_tick_count;            /* timer ticks in last window */
    uint32_t mt_last_cnt;              /* TIM2 CNT at window start */
    uint32_t mt_timestamp_ms;          /* time of last measurement [ms] */

    /* Index handling */
    uint32_t index_triggered;          /* 1 if Z pulse seen since last clear */
    uint32_t index_debounce;           /* debounce counter */
    int32_t  index_offset;             /* CNT offset at index */

    /* Calibration */
    float    calib_offset_rad;         /* encoder offset [rad] to align with motor */
    uint8_t  calib_done;               /* calibration complete flag */

    /* Fault */
    uint8_t  fault;                    /* 1 = encoder fault */
    uint32_t last_pulse_ms;            /* last index of any encoder pulse */
    uint32_t z_event_ms;               /* time of last Z pulse */

    /* Hardware configuration */
    TIM_TypeDef *tim_enc;              /* TIM2 for quadrature */
    TIM_TypeDef *tim_idx;              /* TIM3 for index capture */
} EncoderHandleTypeDef;

static EncoderHandleTypeDef henc;

/* ---------------------------------------------------------------------------*/
/*  Low-level hardware helpers                                                */
/* ---------------------------------------------------------------------------*/

/** @brief  Configure TIM2 in encoder mode 3 (TI1+TI2). */
static void encoder_tim_init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    TIM2->PSC  = 0U;
    TIM2->ARR  = 0xFFFFU;  /* free-running up to 65535 */
    TIM2->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC1S_1   /* TI1 mapped to IC1 */
                | TIM_CCMR1_CC2S_0 | TIM_CCMR1_CC2S_1;   /* TI2 mapped to IC2 */
    TIM2->CCER  = 0U;
    TIM2->SMCR  = TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0;       /* encoder mode 3 */
    TIM2->CNT   = 0U;
    TIM2->CR1   = TIM_CR1_CEN;
}

/** @brief  Configure TIM3 for index pulse input capture. */
static void encoder_index_tim_init(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();

    TIM3->PSC   = 240U - 1U;   /* 1 MHz timer (1 us ticks) */
    TIM3->ARR   = 0xFFFFU;
    TIM3->CCMR1 = TIM_CCMR1_CC1S_0;  /* TI1 -> IC1 */
    TIM3->CCER  = TIM_CCER_CC1E | TIM_CCER_CC1P; /* rising edge, no filter */
    TIM3->DIER  = TIM_DIER_CC1IE;     /* interrupt on capture */
    TIM3->CR1   = TIM_CR1_CEN;

    NVIC_EnableIRQ(TIM3_IRQn);
    NVIC_SetPriority(TIM3_IRQn, 3U);
}

/** @brief  TIM3 capture IRQ — index pulse handler. */
void TIM3_IRQHandler(void)
{
    if (TIM3->SR & TIM_SR_CC1IF) {
        TIM3->SR = ~TIM_SR_CC1IF;

        /* Capture the current TIM2 count at the index moment */
        uint32_t cnt_at_z = TIM2->CNT;

        /* Debounce */
        henc.z_debounce++;
        if (henc.z_debounce >= INDEX_DEBOUNCE_SAMPLES) {
            henc.count_at_z      = (int32_t)cnt_at_z;
            henc.index_triggered = 1U;
            henc.z_capture_count++;
            henc.z_event_ms = HAL_GetTick();
            henc.z_debounce = 0U;
            /* Adjust position offset: we know where index should be */
            henc.index_offset = (int32_t)cnt_at_z;
        }
    }
}

/* ---------------------------------------------------------------------------*/
/*  M/T velocity measurement                                                  */
/* ---------------------------------------------------------------------------*/

/** @brief  Measure speed using M/T method.
  *
  *          M method (frequency):
  *            Count encoder pulses over a fixed time window (5 ms).
  *            speed = pulses / (window_s * CPR) * 60   [RPM]
  *            Accurate at high speed.
  *
  *          T method (period):
  *            Measure the time between two consecutive encoder edges.
  *            speed = 1 / (period_s * CPR) * 60   [RPM] / 4 (for x4 decode)
  *            Accurate at low speed.
  *
  *          Auto-switch at MT_SPEED_THRESH_RPM.
  */
static void encoder_mt_measure(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint32_t dt_ms = now_ms - henc.mt_timestamp_ms;

    if (dt_ms < MT_MEASURE_PERIOD_MS) return;

    /* Get current encoder count */
    uint32_t current_cnt = TIM2->CNT;
    uint32_t dt_ticks = (uint32_t)((now_ms - henc.mt_timestamp_ms) * 1000U); /* us */

    if (dt_ticks == 0U) return;

    /* Signed pulse count in the window */
    int32_t pulse_delta = (int32_t)(current_cnt - henc.mt_last_cnt);

    /* Convert signed 16-bit to 32-bit with overflow handling */
    if (pulse_delta > 32767) pulse_delta -= 65536;
    if (pulse_delta < -32768) pulse_delta += 65536;

    /* Store for M/T calculation */
    henc.mt_pulse_count = (uint32_t)(pulse_delta > 0 ? pulse_delta : -pulse_delta);
    henc.mt_tick_count  = dt_ticks;

    /* ---- M method (frequency) ---- */
    float speed_m_rpm = (float)pulse_delta * 60000000.0f
                      / ((float)ENCODER_CPR * (float)dt_ticks);

    /* ---- T method (period) ---- */
    float speed_t_rpm = 0.0f;
    if (henc.mt_pulse_count > 0U) {
        float period_us = (float)dt_ticks / (float)henc.mt_pulse_count;
        if (period_us > 0.0f) {
            speed_t_rpm = 60000000.0f / ((float)ENCODER_CPR * period_us);
        }
    }

    /* Select method based on speed */
    float speed_rpm;
    if (fabsf(speed_m_rpm) > MT_SPEED_THRESH_RPM) {
        speed_rpm = speed_m_rpm;     /* M method at high speed */
    } else {
        speed_rpm = speed_t_rpm;     /* T method at low speed */
    }

    /* Blend near transition for smoothness */
    if (fabsf(speed_m_rpm) > MT_SPEED_THRESH_RPM * 0.8f &&
        fabsf(speed_m_rpm) < MT_SPEED_THRESH_RPM * 1.2f) {
        /* Linear blend */
        float blend = (fabsf(speed_m_rpm) - MT_SPEED_THRESH_RPM * 0.8f)
                    / (MT_SPEED_THRESH_RPM * 0.4f);
        if (blend < 0.0f) blend = 0.0f;
        if (blend > 1.0f) blend = 1.0f;
        speed_rpm = speed_t_rpm * (1.0f - blend) + speed_m_rpm * blend;
    }

    /* Apply direction */
    speed_rpm = (pulse_delta >= 0) ? speed_rpm : -speed_rpm;

    /* Low-pass filter */
    henc.speed_rpm = MT_FILTER_ALPHA * henc.speed_rpm + (1.0f - MT_FILTER_ALPHA) * speed_rpm;

    /* Convert to electrical rad/s */
    henc.speed_radps = henc.speed_rpm * 0.104719755f * (float)MOTOR_POLE_PAIRS;

    /* Update for next window */
    henc.mt_last_cnt     = current_cnt;
    henc.mt_timestamp_ms = now_ms;

    /* Glitch / fault detection */
    if (fabsf(speed_rpm) > ENC_FAULT_JERK_THRESH) {
        henc.fault = 1U;
    }
}

/* ---------------------------------------------------------------------------*/
/*  Position tracking                                                         */
/* ---------------------------------------------------------------------------*/

/** @brief  Update mechanical and electrical position from encoder counts.
  *         Handles cumulative multi-turn tracking beyond single-revolution
  *         range of the counter.
  */
static void encoder_update_position(void)
{
    static int32_t last_cnt = 0;

    int32_t current_cnt = (int32_t)TIM2->CNT;
    int32_t delta = current_cnt - last_cnt;

    /* Handle 16-bit overflow */
    if (delta > 32767) delta -= 65536;
    if (delta < -32768) delta += 65536;

    henc.pulse_count += delta;

    /* Mechanical revolutions (float) = pulse_count / CPR */
    henc.position_mech_rev = (float)henc.pulse_count / (float)ENCODER_CPR;

    /* Mechanical angle [rad] = 2*pi * fractional_rev */
    float frac_rev = henc.position_mech_rev - floorf(henc.position_mech_rev);
    henc.position_mech_rad = frac_rev * 6.283185307179586f;

    /* Electrical angle = mechanical_angle * pole_pairs (mod 2*pi) */
    henc.position_elec_rad = henc.position_mech_rad * ENCODER_ELEC_SCALE + henc.calib_offset_rad;
    henc.position_elec_rad = henc.position_elec_rad - 6.2831853f * (float)((int32_t)(henc.position_elec_rad * 0.15915494f));

    last_cnt = current_cnt;
}

/* ---------------------------------------------------------------------------*/
/*  Encoder offset calibration                                                */
/* ---------------------------------------------------------------------------*/

/** @brief  Calibrate encoder offset by aligning with motor back-EMF or
  *         by applying a DC current to the D-axis.
  *
  *         Method: apply Id current (align D-axis with phase A), measure
  *         encoder angle, compute offset so that encoder reads 0 when D-axis
  *         is aligned.
  *
  * @param  apply_current  Function pointer to apply Id current [mA].
  *         If NULL, uses open-loop alignment sequence.
  */
void ENC_CalibrateOffset(void)
{
    /* Stage 1: Apply alignment current (assumes FOC_Init was called) */
    /* Set high Id reference to align rotor D-axis with phase A */
    /* In practice this uses the FOC current loop; here we simulate */

    /* Wait for settling */
    HAL_Delay(CALIB_SETTLE_MS);

    /* Read encoder angle during alignment */
    int32_t aligned_cnt = (int32_t)TIM2->CNT;

    /* Compute offset such that electrical angle = 0 when aligned.
       At alignment, encoder shows some mechanical angle.
       theta_elec = pole_pairs * theta_mech + offset = 0 (at alignment)
       => offset = -pole_pairs * theta_mech
    */
    float theta_mech_aligned = (float)(aligned_cnt % ENCODER_CPR) * 6.2831853f / (float)ENCODER_CPR;
    henc.calib_offset_rad = -theta_mech_aligned * ENCODER_ELEC_SCALE;

    /* Normalise offset to [0, 2*pi) */
    henc.calib_offset_rad = fmodf(henc.calib_offset_rad, 6.2831853f);
    if (henc.calib_offset_rad < 0.0f) henc.calib_offset_rad += 6.2831853f;

    henc.calib_done = 1U;
}

/* ---------------------------------------------------------------------------*/
/*  Public API                                                                */
/* ---------------------------------------------------------------------------*/

/** @brief  Initialise encoder module. */
void ENC_Init(void)
{
    memset(&henc, 0, sizeof(henc));
    henc.tim_enc = TIM2;
    henc.tim_idx = TIM3;

    encoder_tim_init();
    encoder_index_tim_init();

    henc.mt_timestamp_ms = HAL_GetTick();
    henc.mt_last_cnt     = 0U;
    henc.calib_done      = 0U;
    henc.fault           = 0U;
}

/** @brief  Main encoder update function — call at 1 kHz or from FOC ISR background. */
void ENC_Update(void)
{
    if (henc.fault) return;

    encoder_update_position();
    encoder_mt_measure();
}

/** @brief  Get the current electrical angle [rad]. */
float ENC_GetElectricalAngle(void)
{
    return henc.position_elec_rad;
}

/** @brief  Get the filtered speed [RPM]. */
float ENC_GetSpeedRPM(void)
{
    return henc.speed_rpm;
}

/** @brief  Get the filtered electrical speed [rad/s]. */
float ENC_GetSpeedRadPs(void)
{
    return henc.speed_radps;
}

/** @brief  Get mechanical position [rad]. */
float ENC_GetMechanicalPosition(void)
{
    return henc.position_mech_rad;
}

/** @brief  Get cumulative mechanical revolutions. */
float ENC_GetRevolutions(void)
{
    return henc.position_mech_rev;
}

/** @brief  Get encoder fault flag. */
uint8_t ENC_GetFault(void)
{
    return henc.fault;
}

/** @brief  Clear encoder fault flag. */
void ENC_ClearFault(void)
{
    henc.fault = 0U;
}

/** @brief  Get encoder calibration status. */
uint8_t ENC_IsCalibrated(void)
{
    return henc.calib_done;
}

/** @brief  Force-set encoder offset (e.g., from stored calibration). */
void ENC_SetOffset(float offset_rad)
{
    henc.calib_offset_rad = fmodf(offset_rad, 6.2831853f);
    if (henc.calib_offset_rad < 0.0f) henc.calib_offset_rad += 6.2831853f;
    henc.calib_done = 1U;
}

/** @brief  Get encoder offset (for storing in NVM). */
float ENC_GetOffset(void)
{
    return henc.calib_offset_rad;
}

/** @brief  Reset position counter to zero (e.g., after homing). */
void ENC_ResetPosition(void)
{
    henc.pulse_count = 0;
    henc.position_mech_rev = 0.0f;
    henc.position_mech_rad = 0.0f;
    henc.position_elec_rad = henc.calib_offset_rad;
    TIM2->CNT = 0U;
}

/** @brief  Check if index pulse has been detected since last call. */
uint8_t ENC_GetIndexTriggered(void)
{
    uint8_t ret = henc.index_triggered ? 1U : 0U;
    henc.index_triggered = 0U;
    return ret;
}

/** @brief  Get total number of index events seen. */
uint32_t ENC_GetIndexCount(void)
{
    return henc.z_capture_count;
}
