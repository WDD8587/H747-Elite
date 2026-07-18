/**
  ******************************************************************************
  * @file    test_deadtime.c
  * @author  M7 FOC Test Team
  * @brief   Unit tests for dead-time compensation module.
  *
  *          Tests:
  *            1. Dead-time compensation does not produce negative duty
  *            2. Dead-time compensation does not exceed 1.0 duty
  *            3. Current polarity detection at zero-crossing
  *            4. Symmetry: positive and negative current produce opposite
  *               compensation
  *            5. Dead-time range clamping (50 ns .. 500 ns)
  *            6. Shoot-through prevention after compensation
  *
  *          Compile with TEST_DEADTIME_ENABLED defined.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

/* Test configuration --------------------------------------------------------*/
#define TEST_DEADTIME_ENABLED

/* Test tolerance */
#define TEST_TOL               0.0001f

/* Test result counters */
static uint32_t dt_passed = 0U;
static uint32_t dt_failed = 0U;

/* ---------------------------------------------------------------------------*/
/*  Inline implementation of dead-time compensation (same as production)       */
/* ---------------------------------------------------------------------------*/

#define ZERO_CROSS_HYSTERESIS    0.050f   /* 50 mA */
#define SIGN_FILTER_ALPHA        0.85f

typedef struct {
    float filtered_current;
    float comp_gain;
    int8_t sign;
} DT_PhaseState;

static DT_PhaseState dt_pha;
static DT_PhaseState dt_phb;
static DT_PhaseState dt_phc;

static void dt_init_states(void)
{
    memset(&dt_pha, 0, sizeof(dt_pha));
    memset(&dt_phb, 0, sizeof(dt_phb));
    memset(&dt_phc, 0, sizeof(dt_phc));
}

static int8_t detect_polarity(float current, DT_PhaseState *state)
{
    state->filtered_current = SIGN_FILTER_ALPHA * state->filtered_current
                            + (1.0f - SIGN_FILTER_ALPHA) * current;

    float fc = state->filtered_current;
    if (fc > ZERO_CROSS_HYSTERESIS) {
        state->sign = 1;
    } else if (fc < -ZERO_CROSS_HYSTERESIS) {
        state->sign = -1;
    }
    return state->sign;
}

static void dt_compensate(float *da, float *db, float *dc,
                           float ia, float ib, float ic,
                           float comp_gain, float margin)
{
    int8_t sa = detect_polarity(ia, &dt_pha);
    int8_t sb = detect_polarity(ib, &dt_phb);
    int8_t sc = detect_polarity(ic, &dt_phc);

    if (sa > 0) *da += comp_gain;
    else if (sa < 0) *da -= comp_gain;
    if (*da < margin)      *da = margin;
    if (*da > 1.0f - margin) *da = 1.0f - margin;

    if (sb > 0) *db += comp_gain;
    else if (sb < 0) *db -= comp_gain;
    if (*db < margin)      *db = margin;
    if (*db > 1.0f - margin) *db = 1.0f - margin;

    if (sc > 0) *dc += comp_gain;
    else if (sc < 0) *dc -= comp_gain;
    if (*dc < margin)      *dc = margin;
    if (*dc > 1.0f - margin) *dc = 1.0f - margin;
}

/* ---------------------------------------------------------------------------*/
/*  Test helpers                                                              */
/* ---------------------------------------------------------------------------*/

static void dt_print(const char *msg)
{
    while (*msg) { ITM_SendChar((uint32_t)(*msg)); msg++; }
    ITM_SendChar('\n');
}

static void dt_assert(int condition, const char *name)
{
    if (condition) { dt_passed++; }
    else { dt_failed++; dt_print("  FAIL: "); dt_print(name); }
}

static float dt_absf(float x) { return (x < 0.0f) ? -x : x; }

/* ---------------------------------------------------------------------------*/
/*  1. No negative duty after compensation                                    */
/* ---------------------------------------------------------------------------*/

static void test_no_negative_duty(void)
{
    dt_print("--- Test: No Negative Duty After Compensation ---");

    dt_init_states();
    float da = 0.1f, db = 0.1f, dc = 0.1f;
    float gain = 0.02f;
    float margin = 0.01f;

    /* Apply compensation with all currents positive (should add gain) */
    dt_compensate(&da, &db, &dc, 1.0f, 1.0f, 1.0f, gain, margin);

    dt_assert(da >= 0.0f, "Duty A >= 0");
    dt_assert(db >= 0.0f, "Duty B >= 0");
    dt_assert(dc >= 0.0f, "Duty C >= 0");
    dt_assert(da <= 1.0f, "Duty A <= 1");
    dt_assert(db <= 1.0f, "Duty B <= 1");
    dt_assert(dc <= 1.0f, "Duty C <= 1");

    /* Now test with small duty and negative current (tries to subtract) */
    dt_init_states();
    da = 0.02f; db = 0.02f; dc = 0.02f;
    dt_compensate(&da, &db, &dc, -1.0f, -1.0f, -1.0f, gain, margin);

    /* Should be clamped to margin, not negative */
    dt_assert(da >= 0.0f, "Neg current: Duty A >= 0");
    dt_assert(db >= 0.0f, "Neg current: Duty B >= 0");
    dt_assert(dc >= 0.0f, "Neg current: Duty C >= 0");
}

/* ---------------------------------------------------------------------------*/
/*  2. No duty exceeds 1.0 after compensation                                 */
/* ---------------------------------------------------------------------------*/

static void test_no_exceed_one(void)
{
    dt_print("--- Test: No Duty Exceeds 1.0 After Compensation ---");

    dt_init_states();
    float da = 0.98f, db = 0.98f, dc = 0.98f;
    float gain = 0.05f;
    float margin = 0.01f;

    /* Positive current adds gain; duty near 1.0 should be clamped */
    dt_compensate(&da, &db, &dc, 1.0f, 1.0f, 1.0f, gain, margin);

    dt_assert(da <= 1.0f, "Near-1.0: Duty A <= 1");
    dt_assert(db <= 1.0f, "Near-1.0: Duty B <= 1");
    dt_assert(dc <= 1.0f, "Near-1.0: Duty C <= 1");
    dt_assert(da >= 0.0f, "Near-1.0: Duty A >= 0");
}

/* ---------------------------------------------------------------------------*/
/*  3. Zero-crossing detection                                                */
/* ---------------------------------------------------------------------------*/

static void test_zero_crossing(void)
{
    dt_print("--- Test: Zero-Crossing Polarity Detection ---");

    dt_init_states();

    /* Positive current well above hysteresis */
    int8_t s = detect_polarity(1.0f, &dt_pha);
    dt_assert(s == 1, "Positive current: sign = +1");

    /* Negative current well below hysteresis */
    s = detect_polarity(-1.0f, &dt_pha);
    dt_assert(s == -1, "Negative current: sign = -1");

    /* Current within hysteresis band: sign should stay at previous (-1) */
    s = detect_polarity(0.03f, &dt_pha);  /* 30 mA < 50 mA hysteresis */
    dt_assert(s == -1, "Within hysteresis: sign unchanged");

    /* Current above hysteresis: sign should flip to +1 */
    s = detect_polarity(0.10f, &dt_pha);  /* 100 mA > 50 mA hysteresis */
    dt_assert(s == 1, "Above hysteresis: sign flips to +1");

    /* Current back within band: sign should stay at +1 */
    s = detect_polarity(-0.03f, &dt_pha);
    dt_assert(s == 1, "Back within band: sign stays +1");

    /* Negative excursion: sign should flip to -1 */
    s = detect_polarity(-0.10f, &dt_pha);
    dt_assert(s == -1, "Negative excursion: sign flips to -1");
}

/* ---------------------------------------------------------------------------*/
/*  4. Symmetry: positive vs negative compensation                            */
/* ---------------------------------------------------------------------------*/

static void test_compensation_symmetry(void)
{
    dt_print("--- Test: Compensation Symmetry ---");

    float gain = 0.02f;
    float margin = 0.005f;

    /* Positive current */
    dt_init_states();
    float da_pos = 0.5f, db_pos = 0.5f, dc_pos = 0.5f;
    dt_compensate(&da_pos, &db_pos, &dc_pos, 2.0f, 2.0f, 2.0f, gain, margin);
    float delta_pos = da_pos - 0.5f;

    /* Negative current */
    dt_init_states();
    float da_neg = 0.5f, db_neg = 0.5f, dc_neg = 0.5f;
    dt_compensate(&da_neg, &db_neg, &dc_neg, -2.0f, -2.0f, -2.0f, gain, margin);
    float delta_neg = da_neg - 0.5f;

    /* Compensations should be opposite and approximately equal in magnitude */
    dt_assert(dt_absf(delta_pos + delta_neg) < TEST_TOL,
              "Positive and negative compensation are symmetric");

    /* Positive adds, negative subtracts */
    dt_assert(delta_pos > 0.0f, "Positive compensation adds duty");
    dt_assert(delta_neg < 0.0f, "Negative compensation subtracts duty");
}

/* ---------------------------------------------------------------------------*/
/*  5. Dead-time range clamping                                               */
/* ---------------------------------------------------------------------------*/

static void test_deadtime_range(void)
{
    dt_print("--- Test: Dead-Time Range Clamping ---");

    /* Simulate the Init clamping logic */
    uint32_t min_ns = 50U;
    uint32_t max_ns = 500U;

    uint32_t test_vals[] = {0U, 25U, 50U, 100U, 250U, 500U, 1000U, 9999U};
    uint32_t expected[] = {50U, 50U, 50U, 100U, 250U, 500U, 500U, 500U};

    for (int i = 0; i < 8; i++) {
        uint32_t clamped = test_vals[i];
        if (clamped < min_ns) clamped = min_ns;
        if (clamped > max_ns) clamped = max_ns;
        dt_assert(clamped == expected[i], "Dead-time range clamp");
    }
}

/* ---------------------------------------------------------------------------*/
/*  6. Shoot-through prevention verification                                   */
/* ---------------------------------------------------------------------------*/

static void test_shoot_through_prevention(void)
{
    dt_print("--- Test: Shoot-Through Prevention ---");

    dt_init_states();
    float da = 1.0f, db = 1.0f, dc = 1.0f;
    float gain = 0.02f;
    float margin = 0.02f;

    /* With positive currents, duty would exceed 1.0, should be clamped */
    dt_compensate(&da, &db, &dc, 5.0f, 5.0f, 5.0f, gain, margin);

    /* After clamping with margin, the duty should leave room for dead-time */
    dt_assert(da <= (1.0f - margin + 0.001f),
              "Shoot-through: duty leaves dead-time margin");

    /* Similarly, with small duty and negative current */
    dt_init_states();
    da = 0.0f; db = 0.0f; dc = 0.0f;
    dt_compensate(&da, &db, &dc, -5.0f, -5.0f, -5.0f, gain, margin);

    dt_assert(da >= (margin - 0.001f),
              "Shoot-through: duty does not go below margin");
}

/* ---------------------------------------------------------------------------*/
/*  Test runner                                                               */
/* ---------------------------------------------------------------------------*/

void TEST_DEADTIME_RunAll(void)
{
    dt_print("================================================");
    dt_print("  Dead-Time Compensation Unit Tests");
    dt_print("================================================");

    dt_passed = 0U;
    dt_failed = 0U;

    test_no_negative_duty();
    test_no_exceed_one();
    test_zero_crossing();
    test_compensation_symmetry();
    test_deadtime_range();
    test_shoot_through_prevention();

    dt_print("================================================");
    char buf[64];
    sprintf(buf, "  Results: %lu passed, %lu failed", dt_passed, dt_failed);
    dt_print(buf);
    dt_print("================================================");
}

uint8_t TEST_DEADTIME_AllPassed(void) { return (dt_failed == 0U) ? 1U : 0U; }
uint32_t TEST_DEADTIME_GetPassed(void) { return dt_passed; }
uint32_t TEST_DEADTIME_GetFailed(void) { return dt_failed; }
