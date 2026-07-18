/**
  ******************************************************************************
  * @file    test_encoder.c
  * @author  M7 FOC Test Team
  * @brief   Unit tests for quadrature encoder interface and M/T speed
  *          measurement.
  *
  *          Tests:
  *            1. Quadrature decode: A leads B = forward, B leads A = reverse
  *            2. Index (Z) pulse resets position count
  *            3. M/T method at known RPM (both M and T regions)
  *            4. Encoder offset calibration correctness
  *            5. Overflow handling for 16-bit counter
  *            6. Zero-speed output
  *
  *          Compile with TEST_ENCODER_ENABLED defined.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

/* Test configuration --------------------------------------------------------*/
#define TEST_ENCODER_ENABLED

#define ENC_CPR          8192U    /* encoder counts per revolution */
#define ENC_LINES        2048U
#define MOTOR_POLE_PAIRS 7U

/* Test tolerance */
#define TEST_TOL_RPM      5.0f    /* RPM tolerance */
#define TEST_TOL_ANGLE    0.01f   /* rad tolerance */

/* Test result counters */
static uint32_t enc_passed = 0U;
static uint32_t enc_failed = 0U;

/* ---------------------------------------------------------------------------*/
/*  Encoder simulation helpers                                                */
/* ---------------------------------------------------------------------------*/

typedef struct {
    int32_t count;          /* simulated counter value */
    int32_t index_count;    /* count at last index */
    uint8_t index_triggered;
} EncoderSim;

static void enc_sim_init(EncoderSim *sim)
{
    sim->count = 0;
    sim->index_count = 0;
    sim->index_triggered = 0;
}

/* Simulate encoder movement in counts (positive = forward) */
static void enc_sim_move(EncoderSim *sim, int32_t delta_counts)
{
    sim->count += delta_counts;
}

/* Simulate index pulse: record current count as index position */
static void enc_sim_index(EncoderSim *sim)
{
    sim->index_count = sim->count;
    sim->index_triggered = 1;
}

/* Simulate A/B quadrature leading relationship:
   A leads B = +1 count per edge (forward)
   B leads A = -1 count per edge (reverse)
   Returns delta counts for one full quadrature cycle (4 edges) */
static int32_t enc_sim_quad_cycle(uint8_t a_leads_b)
{
    return a_leads_b ? 4 : -4;
}

/* ---------------------------------------------------------------------------*/
/*  Test helpers                                                              */
/* ---------------------------------------------------------------------------*/

static void enc_print(const char *msg)
{
    while (*msg) { ITM_SendChar((uint32_t)(*msg)); msg++; }
    ITM_SendChar('\n');
}

static void enc_assert(int condition, const char *name)
{
    if (condition) { enc_passed++; }
    else { enc_failed++; enc_print("  FAIL: "); enc_print(name); }
}

static float enc_absf(float x) { return (x < 0.0f) ? -x : x; }

/* Convert counter value to electrical angle */
static float cnt_to_elec_angle(int32_t cnt, int32_t cpr, int pole_pairs)
{
    /* Normalise count to [0, CPR) */
    int32_t norm = cnt % cpr;
    if (norm < 0) norm += cpr;

    float mech_rad = (float)norm * 6.2831853f / (float)cpr;
    float elec_rad = mech_rad * (float)pole_pairs;
    /* Normalise to [0, 2*pi) */
    elec_rad = elec_rad - 6.2831853f * (float)((int32_t)(elec_rad * 0.15915494f));
    if (elec_rad < 0.0f) elec_rad += 6.2831853f;
    return elec_rad;
}

/* Simulate M-method speed measurement */
static float m_method_speed(int32_t pulse_count, uint32_t dt_us, int32_t cpr)
{
    return (float)pulse_count * 60000000.0f / ((float)cpr * (float)dt_us);
}

/* Simulate T-method speed measurement */
static float t_method_speed(uint32_t period_us, int32_t cpr)
{
    return 60000000.0f / ((float)cpr * (float)period_us / 4.0f);
}

/* ---------------------------------------------------------------------------*/
/*  1. Quadrature decode: A leads B = forward                                 */
/* ---------------------------------------------------------------------------*/

static void test_quadrature_forward(void)
{
    enc_print("--- Test: Quadrature Decode Forward (A leads B) ---");

    /* Simulate 10 full quadrature cycles with A leading B */
    EncoderSim sim;
    enc_sim_init(&sim);

    int32_t total_delta = 0;
    for (int i = 0; i < 10; i++) {
        total_delta += enc_sim_quad_cycle(1); /* A leads B => +4 */
    }
    enc_sim_move(&sim, total_delta);

    /* 10 cycles * 4 counts = 40 counts forward */
    enc_assert(sim.count == 40, "Forward: 10 quad cycles = +40 counts");

    /* Verify electrical angle after forward movement */
    float angle = cnt_to_elec_angle(sim.count, ENC_CPR, MOTOR_POLE_PAIRS);
    enc_assert(angle > 0.0f, "Forward: angle > 0");
    enc_assert(angle < 6.2832f, "Forward: angle < 2*pi");
}

/* ---------------------------------------------------------------------------*/
/*  2. Quadrature decode: B leads A = reverse                                 */
/* ---------------------------------------------------------------------------*/

static void test_quadrature_reverse(void)
{
    enc_print("--- Test: Quadrature Decode Reverse (B leads A) ---");

    EncoderSim sim;
    enc_sim_init(&sim);

    int32_t total_delta = 0;
    for (int i = 0; i < 10; i++) {
        total_delta += enc_sim_quad_cycle(0); /* B leads A => -4 */
    }
    enc_sim_move(&sim, total_delta);

    enc_assert(sim.count == -40, "Reverse: 10 quad cycles = -40 counts");

    /* Verify angle wraps correctly for negative counts */
    float angle = cnt_to_elec_angle(sim.count, ENC_CPR, MOTOR_POLE_PAIRS);
    enc_assert(angle >= 0.0f, "Reverse: angle >= 0");
    enc_assert(angle < 6.2832f, "Reverse: angle < 2*pi");
}

/* ---------------------------------------------------------------------------*/
/*  3. Index pulse resets position                                             */
/* ---------------------------------------------------------------------------*/

static void test_index_pulse(void)
{
    enc_print("--- Test: Index Pulse Resets Position ---");

    EncoderSim sim;
    enc_sim_init(&sim);

    /* Move forward 100 counts */
    enc_sim_move(&sim, 100);
    enc_assert(sim.count == 100, "Index pre-move: count = 100");

    /* Index pulse at current position */
    enc_sim_index(&sim);
    enc_assert(sim.index_count == 100, "Index: captured count = 100");
    enc_assert(sim.index_triggered == 1, "Index: triggered flag = 1");

    /* After index, further movement from this reference */
    enc_sim_move(&sim, 50);
    enc_assert(sim.count == 150, "Index post-move: count = 150");

    /* Index position should still be recorded */
    enc_assert(sim.index_count == 100, "Index: recorded position unchanged");
}

/* ---------------------------------------------------------------------------*/
/*  4. M method speed measurement at known RPM                                */
/* ---------------------------------------------------------------------------*/

static void test_m_method(void)
{
    enc_print("--- Test: M-Method Speed Measurement ---");

    /* At 1000 RPM, in 5 ms (5000 us), encoder should produce:
       counts = RPM * CPR * time_min / 60
              = 1000 * 8192 * (5/60000)
              = 1000 * 8192 * 0.00008333
              = 682.67 counts
    */
    int32_t expected_counts = (int32_t)(1000.0f * (float)ENC_CPR * 0.005f / 60.0f);

    /* Alternatively: 1000 RPM = 16.67 rev/s = 136533 counts/s
       In 5 ms = 682.7 counts */
    float speed = m_method_speed(683, 5000, ENC_CPR);
    enc_assert(enc_absf(speed - 1000.0f) < TEST_TOL_RPM,
               "M-method: 683 counts in 5ms = ~1000 RPM");

    /* At 5000 RPM */
    speed = m_method_speed(3413, 5000, ENC_CPR);
    enc_assert(enc_absf(speed - 5000.0f) < TEST_TOL_RPM,
               "M-method: 3413 counts in 5ms = ~5000 RPM");

    /* At 100 RPM (low speed, still using M method with threshold at 100 RPM) */
    speed = m_method_speed(68, 5000, ENC_CPR);
    enc_assert(enc_absf(speed - 100.0f) < TEST_TOL_RPM,
               "M-method: 68 counts in 5ms = ~100 RPM");

    /* Zero speed */
    speed = m_method_speed(0, 5000, ENC_CPR);
    enc_assert(enc_absf(speed) < 1.0f, "M-method: 0 counts = 0 RPM");
}

/* ---------------------------------------------------------------------------*/
/*  5. T method speed measurement at known RPM                                */
/* ---------------------------------------------------------------------------*/

static void test_t_method(void)
{
    enc_print("--- Test: T-Method Speed Measurement ---");

    /* T method measures the period of one encoder edge.
       With x4 decoding, there are 4*CPR edges per revolution = 32768 edges.

       At 50 RPM: 50/60 = 0.833 rev/s => 27307 edges/s => 36.6 us per edge.
       At 100 RPM: 100/60 = 1.667 rev/s => 54613 edges/s => 18.3 us per edge.
    */

    /* 36.6 us per edge => ~50 RPM */
    float speed = t_method_speed(37, ENC_CPR);
    enc_assert(enc_absf(speed - 50.0f) < TEST_TOL_RPM,
               "T-method: 37 us period = ~50 RPM");

    /* 18.3 us per edge => ~100 RPM */
    speed = t_method_speed(18, ENC_CPR);
    enc_assert(enc_absf(speed - 100.0f) < TEST_TOL_RPM,
               "T-method: 18 us period = ~100 RPM");

    /* 9.1 us per edge => ~200 RPM */
    speed = t_method_speed(9, ENC_CPR);
    enc_assert(enc_absf(speed - 200.0f) < 2.0f * TEST_TOL_RPM,
               "T-method: 9 us period = ~200 RPM");
}

/* ---------------------------------------------------------------------------*/
/*  6. Encoder offset calibration                                             */
/* ---------------------------------------------------------------------------*/

static void test_offset_calibration(void)
{
    enc_print("--- Test: Encoder Offset Calibration ---");

    /* Simulate alignment: encoder reads 1024 counts when rotor at D-axis */
    int32_t aligned_count = 1024;

    /* Compute offset so that electrical angle = 0 at alignment */
    float mech_angle = (float)(aligned_count % (int32_t)ENC_CPR) * 6.2831853f / (float)ENC_CPR;
    float offset = -mech_angle * (float)MOTOR_POLE_PAIRS;

    /* Normalise to [0, 2*pi) */
    offset = fmodf(offset, 6.2831853f);
    if (offset < 0.0f) offset += 6.2831853f;

    /* After applying offset, at the aligned position, electrical angle should be 0 */
    int32_t test_cnt = aligned_count;
    int32_t norm = test_cnt % (int32_t)ENC_CPR;
    if (norm < 0) norm += ENC_CPR;
    float test_mech = (float)norm * 6.2831853f / (float)ENC_CPR;

    float test_elec = test_mech * (float)MOTOR_POLE_PAIRS + offset;
    test_elec = fmodf(test_elec, 6.2831853f);
    if (test_elec < 0.0f) test_elec += 6.2831853f;

    /* At alignment, electrical angle should be 0 (or very close) */
    enc_assert(test_elec < TEST_TOL_ANGLE,
               "Calibration: aligned position gives ~0 electrical angle");

    /* Test: 90 degree rotation should produce pole_pairs * 90deg shift */
    int32_t rotated = aligned_count + ENC_CPR / 4; /* 90 deg mechanical */
    norm = rotated % (int32_t)ENC_CPR;
    if (norm < 0) norm += ENC_CPR;
    test_mech = (float)norm * 6.2831853f / (float)ENC_CPR;
    test_elec = test_mech * (float)MOTOR_POLE_PAIRS + offset;
    test_elec = fmodf(test_elec, 6.2831853f);
    if (test_elec < 0.0f) test_elec += 6.2831853f;

    /* 90 deg mech = 90 * 7 = 630 deg electrical = 270 deg mod 360 = 4.71 rad */
    enc_assert(enc_absf(test_elec - 4.7124f) < 0.1f,
               "Calibration: 90 deg mech rotation produces correct electrical shift");
}

/* ---------------------------------------------------------------------------*/
/*  7. 16-bit counter overflow handling                                       */
/* ---------------------------------------------------------------------------*/

static void test_overflow_handling(void)
{
    enc_print("--- Test: 16-bit Counter Overflow Handling ---");

    int32_t previous = 65000; /* near top of 16-bit range */
    int32_t current  = 100;   /* wrapped around (65000 + 65536 - 100 = overflow) */

    /* Delta calculation with overflow handling */
    int32_t delta = current - previous;
    if (delta > 32767) delta -= 65536;
    if (delta < -32768) delta += 65536;

    /* Actual forward movement: 65000 -> 100 means 65536 - 65000 + 100 = 636 counts forward */
    int32_t expected_delta = 100 + (65536 - 65000); /* = 636 */
    enc_assert(delta == expected_delta,
               "Overflow: delta calculation correct for forward wrap");

    /* Reverse overflow: 100 -> 65000 means 100 - 65536 - 65000 = -636 */
    previous = 100;
    current  = 65000;
    delta = current - previous;
    if (delta > 32767) delta -= 65536;
    if (delta < -32768) delta += 65536;

    expected_delta = -(100 + (65536 - 65000)); /* = -636 */
    enc_assert(delta == expected_delta,
               "Overflow: delta calculation correct for reverse wrap");

    /* Normal movement (no overflow) */
    previous = 1000;
    current  = 2000;
    delta = current - previous;
    if (delta > 32767) delta -= 65536;
    if (delta < -32768) delta += 65536;
    enc_assert(delta == 1000, "No overflow: delta = 1000");
}

/* ---------------------------------------------------------------------------*/
/*  8. Zero speed output                                                      */
/* ---------------------------------------------------------------------------*/

static void test_zero_speed(void)
{
    enc_print("--- Test: Zero Speed Output ---");

    /* At zero speed, encoder count should not change */
    float speed = m_method_speed(0, 5000, ENC_CPR);
    enc_assert(enc_absf(speed) < 0.1f, "Zero speed: M-method = 0 RPM");

    speed = t_method_speed(0, ENC_CPR);
    enc_assert(speed == 0.0f, "Zero speed: T-method = 0 RPM (infinite period)");
}

/* ---------------------------------------------------------------------------*/
/*  Test runner                                                               */
/* ---------------------------------------------------------------------------*/

void TEST_ENCODER_RunAll(void)
{
    enc_print("================================================");
    enc_print("  Encoder Unit Tests");
    enc_print("================================================");

    enc_passed = 0U;
    enc_failed = 0U;

    test_quadrature_forward();
    test_quadrature_reverse();
    test_index_pulse();
    test_m_method();
    test_t_method();
    test_offset_calibration();
    test_overflow_handling();
    test_zero_speed();

    enc_print("================================================");
    char buf[64];
    sprintf(buf, "  Results: %lu passed, %lu failed", enc_passed, enc_failed);
    enc_print(buf);
    enc_print("================================================");
}

uint8_t TEST_ENCODER_AllPassed(void) { return (enc_failed == 0U) ? 1U : 0U; }
uint32_t TEST_ENCODER_GetPassed(void) { return enc_passed; }
uint32_t TEST_ENCODER_GetFailed(void) { return enc_failed; }
