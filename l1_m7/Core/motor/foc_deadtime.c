/**
  ******************************************************************************
  * @file    foc_deadtime.c
  * @author  M7 FOC Team
  * @brief   Dead-time compensation for 3-phase PWM inverter.
  *
  *          Detects current polarity per phase and adjusts PWM duty to
  *          compensate for the non-linear effect of dead-time on output
  *          voltage.  Configurable dead-time range: 50 ns .. 500 ns.
  *
  *          Theory: During the dead-time interval both MOSFETs in a half-bridge
  *          are off.  The free-wheeling diode conducts, and the output voltage
  *          is pulled to the DC rail opposite to the direction of load current.
  *          This causes a voltage error proportional to dead-time and switching
  *          frequency.  Compensation adds/subtracts dead-time to the commanded
  *          duty based on the sign of the phase current.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "foc_params.h"
#include <math.h>

/* Private defines -----------------------------------------------------------*/
#define DEADTIME_MIN_NS          50U
#define DEADTIME_MAX_NS          500U
#define DEADTIME_DEFAULT_NS      100U

/* Hysteresis band for zero-crossing detection [A] to avoid chattering */
#define ZERO_CROSS_HYSTERESIS    0.050f   /* 50 mA */

/* Low-pass filter coefficient for current sign smoothing (0..1) */
#define SIGN_FILTER_ALPHA        0.85f

/* Compensation scaling factor (duty correction per ns of dead-time).
   Derived from: delta_duty = T_dead * f_sw * 2 / Vdc  (simplified)
   At f_sw = 20 kHz, Vdc = 24 V:  1 ns ~ 1.667e-6 duty per A ... we use
   a normalised approach: full compensation = T_dead / T_pwm.               */
static float g_deadtime_ns = DEADTIME_DEFAULT_NS;
static uint32_t g_deadtime_ticks = 0U;

/* Filtered sign of each phase current (smoothed for stable compensation) */
typedef struct {
    float filtered_current;   /* low-pass filtered phase current */
    float comp_gain;          /* duty correction magnitude        */
    int8_t sign;              /* +1 (positive), -1 (negative), 0 (zero-cross) */
} PhaseCompState;

static PhaseCompState comp_phase_a;
static PhaseCompState comp_phase_b;
static PhaseCompState comp_phase_c;

/* ---------------------------------------------------------------------------*/
/*  Initialisation                                                            */
/* ---------------------------------------------------------------------------*/

/** @brief  Initialise dead-time compensation module.
  * @param  deadtime_ns  Desired dead-time in nanoseconds (50..500).
  */
void FOC_DeadTime_Init(uint32_t deadtime_ns)
{
    uint32_t clamped = deadtime_ns;
    if (clamped < DEADTIME_MIN_NS) clamped = DEADTIME_MIN_NS;
    if (clamped > DEADTIME_MAX_NS) clamped = DEADTIME_MAX_NS;
    g_deadtime_ns = (float)clamped;

    /* Convert to TIM1 dead-time counter ticks (BDTR DTG).
       TIM1 clock = 240 MHz => T_tick ~ 4.167 ns.
       DTG = round(clamped / 4.167)
    */
    g_deadtime_ticks = (uint32_t)((float)clamped * 0.24f + 0.5f);
    if (g_deadtime_ticks > 255U) g_deadtime_ticks = 255U;

    comp_phase_a.sign = 0;
    comp_phase_a.filtered_current = 0.0f;
    comp_phase_a.comp_gain = 0.0f;

    comp_phase_b.sign = 0;
    comp_phase_b.filtered_current = 0.0f;
    comp_phase_b.comp_gain = 0.0f;

    comp_phase_c.sign = 0;
    comp_phase_c.filtered_current = 0.0f;
    comp_phase_c.comp_gain = 0.0f;
}

/** @brief  Compute compensation gain factor for current dead-time and PWM period.
  *         delta_duty = T_dead / T_pwm  (normalised 0..1).
  *         Called when FOC period or dead-time changes.
  */
void FOC_DeadTime_UpdateGain(float pwm_period_s)
{
    float gain = g_deadtime_ns * 1e-9f / pwm_period_s;

    /* Apply to all three phases (symmetric) */
    comp_phase_a.comp_gain = gain;
    comp_phase_b.comp_gain = gain;
    comp_phase_c.comp_gain = gain;
}

/* ---------------------------------------------------------------------------*/
/*  Current polarity detection with hysteresis                                */
/* ---------------------------------------------------------------------------*/

/** @brief  Detect sign of current with hysteresis to prevent chatter.
  * @param  current  Phase current in Amperes.
  * @param  state    Pointer to per-phase compensation state (updated in place).
  * @return +1 (positive), -1 (negative), 0 (zero-cross band).
  *         ~8 cycles (FPU + compare).
  */
static int8_t detect_polarity(float current, PhaseCompState *state)
{
    /* Low-pass filter the current for noise rejection (~3 cy) */
    state->filtered_current = SIGN_FILTER_ALPHA * state->filtered_current
                            + (1.0f - SIGN_FILTER_ALPHA) * current;

    float fc = state->filtered_current;

    if (fc > ZERO_CROSS_HYSTERESIS) {
        state->sign = 1;
    } else if (fc < -ZERO_CROSS_HYSTERESIS) {
        state->sign = -1;
    }
    /* else: keep previous sign (in dead-band) */
    return state->sign;
}

/* ---------------------------------------------------------------------------*/
/*  Dead-time compensation: main entry point                                   */
/* ---------------------------------------------------------------------------*/

/** @brief  Apply dead-time compensation to PWM duty commands.
  *
  *          For positive current: output voltage is reduced by dead-time.
  *          We compensate by ADDING delta_duty to the commanded duty.
  *
  *          For negative current: output voltage is increased by dead-time.
  *          We compensate by SUBTRACTING delta_duty from the commanded duty.
  *
  *          Result is clamped to [deadtime_margin, 1 - deadtime_margin] to
  *          prevent shoot-through even after compensation.
  *
  * @param  da, db, dc  [in/out] Normalised duty cycles [0..1].
  * @param  ia, ib, ic  Phase currents [A].
  * @param  deadtime_ticks  Dead-time in TIM1 counter ticks (for margin calc).
  *         ~20 cycles.
  */
void FOC_DeadTime_Compensate(float *da, float *db, float *dc,
                              float ia, float ib, float ic,
                              uint32_t deadtime_ticks)
{
    /* ---- Detect polarity per phase (~24 cy total: 3 * 8 cy) ---- */
    int8_t sign_a = detect_polarity(ia, &comp_phase_a);
    int8_t sign_b = detect_polarity(ib, &comp_phase_b);
    int8_t sign_c = detect_polarity(ic, &comp_phase_c);

    /* ---- Per-phase dead-time compensation ---- */
    float margin = (float)deadtime_ticks * 1.666e-6f; /* approximate duty margin */

    /* Phase A (~4 cy) */
    if (sign_a > 0) {
        *da += comp_phase_a.comp_gain;
    } else if (sign_a < 0) {
        *da -= comp_phase_a.comp_gain;
    }
    /* Clamp with margin to preserve dead-time */
    if (*da < margin)      *da = margin;
    if (*da > 1.0f - margin) *da = 1.0f - margin;

    /* Phase B (~4 cy) */
    if (sign_b > 0) {
        *db += comp_phase_b.comp_gain;
    } else if (sign_b < 0) {
        *db -= comp_phase_b.comp_gain;
    }
    if (*db < margin)      *db = margin;
    if (*db > 1.0f - margin) *db = 1.0f - margin;

    /* Phase C (~4 cy) */
    if (sign_c > 0) {
        *dc += comp_phase_c.comp_gain;
    } else if (sign_c < 0) {
        *dc -= comp_phase_c.comp_gain;
    }
    if (*dc < margin)      *dc = margin;
    if (*dc > 1.0f - margin) *dc = 1.0f - margin;

    /* ---- Shoot-through prevention: verify no complementary pair
             can have simultaneous on-time after compensation.
             Since we maintain margin from 0 and 1, this is implicit. ---- */
}

/* ---------------------------------------------------------------------------*/
/*  Advanced: adaptive dead-time adjustment                                   */
/* ---------------------------------------------------------------------------*/

/** @brief  Adaptively adjust dead-time based on DC bus voltage.
  *         At higher voltage, dead-time may need to increase to prevent
  *         shoot-through from switching node ringing.
  *         Vdc range 18..28 V => dead-time scales +/- 20 %.
  */
void FOC_DeadTime_AdaptiveAdjust(float v_dc)
{
    if (v_dc < 1.0f) v_dc = VBUS_NOMINAL_V;

    float scale = 1.0f + (v_dc - VBUS_NOMINAL_V) * 0.02f; /* 2 % per volt deviation */
    if (scale < 0.8f) scale = 0.8f;
    if (scale > 1.2f) scale = 1.2f;

    uint32_t new_ns = (uint32_t)(g_deadtime_ns * scale + 0.5f);
    if (new_ns < DEADTIME_MIN_NS) new_ns = DEADTIME_MIN_NS;
    if (new_ns > DEADTIME_MAX_NS) new_ns = DEADTIME_MAX_NS;

    /* Re-apply with new dead-time */
    FOC_DeadTime_Init(new_ns);
}

/* ---------------------------------------------------------------------------*/
/*  Status / diagnostics                                                      */
/* ---------------------------------------------------------------------------*/

/** @brief  Get current dead-time setting in nanoseconds. */
uint32_t FOC_DeadTime_GetValueNs(void)
{
    return (uint32_t)g_deadtime_ns;
}

/** @brief  Report per-phase compensation state (for debug telemetry). */
void FOC_DeadTime_GetStates(int8_t *sign_a, int8_t *sign_b, int8_t *sign_c,
                              float *gain_a, float *gain_b, float *gain_c)
{
    *sign_a = comp_phase_a.sign;
    *sign_b = comp_phase_b.sign;
    *sign_c = comp_phase_c.sign;
    *gain_a = comp_phase_a.comp_gain;
    *gain_b = comp_phase_b.comp_gain;
    *gain_c = comp_phase_c.comp_gain;
}
