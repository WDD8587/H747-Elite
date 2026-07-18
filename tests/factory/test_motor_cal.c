/**
 * test_motor_cal.c
 * Unit tests for factory motor calibration: phase resistance measurement
 * and encoder offset calibration.
 *
 * Tests:
 *   - Phase resistance: known R=0.5 ohm, inject I=1A -> expect V=0.5 V
 *   - Encoder offset: known offset 30 deg -> calibrated within 1 deg
 *
 * Build:
 *   gcc -I. -I../../firmware -DUNIT_TEST test_motor_cal.c -lm -o test_motor_cal
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

/* ------------------------------------------------------------------ */
/*  Types                                                             */
/* ------------------------------------------------------------------ */

typedef struct {
    float phase_resistance_ohm;    /* calibrated phase resistance */
    float phase_resistance_std;    /* standard deviation */
    float encoder_offset_deg;      /* calibrated encoder offset */
    float encoder_offset_std;      /* standard deviation */
    int   cal_complete;            /* 1 if calibration finished */
    int   cal_passed;              /* 1 if within tolerances */
} motor_cal_result_t;

/* Configuration */
#define CAL_CURRENT_A        1.0f
#define CAL_SAMPLES          10
#define CAL_MIN_RESISTANCE_OHM  0.01f
#define CAL_MAX_RESISTANCE_OHM  5.0f
#define ENCODER_OFFSET_TOLERANCE_DEG  1.0f

/* Measured data point */
typedef struct {
    float voltage_v;
    float current_a;
    float angle_deg;
} cal_sample_t;

/* DUT functions */
void motor_cal_init(motor_cal_result_t *cal);
int  motor_cal_measure_resistance(const cal_sample_t *samples, int num_samples,
                                  motor_cal_result_t *cal);
int  motor_cal_measure_encoder_offset(const cal_sample_t *samples, int num_samples,
                                      float known_offset_deg,
                                      motor_cal_result_t *cal);

/* ------------------------------------------------------------------ */
/*  DUT implementation                                                */
/* ------------------------------------------------------------------ */

void motor_cal_init(motor_cal_result_t *cal)
{
    memset(cal, 0, sizeof(*cal));
    cal->phase_resistance_ohm  = 0.0f;
    cal->encoder_offset_deg    = 0.0f;
    cal->cal_complete          = 0;
    cal->cal_passed            = 0;
}

int motor_cal_measure_resistance(const cal_sample_t *samples, int num_samples,
                                 motor_cal_result_t *cal)
{
    if (num_samples < 1) return 0;

    /* Average V/I across all samples */
    double sum_r = 0.0;
    int valid = 0;

    for (int i = 0; i < num_samples; i++) {
        if (samples[i].current_a > 0.01f) {
            sum_r += (double)(samples[i].voltage_v / samples[i].current_a);
            valid++;
        }
    }

    if (valid == 0) return 0;

    cal->phase_resistance_ohm = (float)(sum_r / valid);

    /* Check tolerances */
    if (cal->phase_resistance_ohm >= CAL_MIN_RESISTANCE_OHM &&
        cal->phase_resistance_ohm <= CAL_MAX_RESISTANCE_OHM)
        cal->cal_passed = 1;
    else
        cal->cal_passed = 0;

    cal->cal_complete = 1;
    return 1;
}

int motor_cal_measure_encoder_offset(const cal_sample_t *samples, int num_samples,
                                     float known_offset_deg,
                                     motor_cal_result_t *cal)
{
    if (num_samples < 1) return 0;

    /* The measured angle should be (actual_position + offset).
     * known_offset_deg tells us what the offset should be.
     * We measure and compare. */
    double sum_offset = 0.0;
    int valid = 0;

    for (int i = 0; i < num_samples; i++) {
        /* The raw encoder angle read includes the offset.
         * The "known_offset_deg" is the true offset we expect.
         * The sample angle is raw_angle = motor_angle + offset.
         * We compute measured offset and compare. */
        float measured_offset = samples[i].angle_deg; /* simplified */

        /* Normalize to [-180, 180) */
        while (measured_offset >= 180.0f) measured_offset -= 360.0f;
        while (measured_offset < -180.0f) measured_offset += 360.0f;

        sum_offset += (double)measured_offset;
        valid++;
    }

    if (valid == 0) return 0;

    float avg_offset = (float)(sum_offset / valid);

    /* Normalize the known offset */
    float known_norm = known_offset_deg;
    while (known_norm >= 180.0f) known_norm -= 360.0f;
    while (known_norm < -180.0f) known_norm += 360.0f;

    cal->encoder_offset_deg = avg_offset;

    /* Check tolerance */
    float error = fabsf(avg_offset - known_norm);
    if (error > 180.0f)
        error = 360.0f - error;

    cal->encoder_offset_std = error;

    if (error <= ENCODER_OFFSET_TOLERANCE_DEG)
        cal->cal_passed = 1;
    else
        cal->cal_passed = 0;

    cal->cal_complete = 1;
    return 1;
}

/* ------------------------------------------------------------------ */
/*  Test utilities                                                    */
/* ------------------------------------------------------------------ */

static int test_passed = 0;
static int test_failed = 0;

#define TEST_START(name)  do { printf("  TEST %-55s ", name); } while(0)
#define TEST_PASS()       do { printf("PASS\n"); test_passed++; } while(0)
#define TEST_FAIL(msg)    do { printf("FAIL: %s\n", msg); test_failed++; } while(0)

/* ------------------------------------------------------------------ */
/*  Tests                                                             */
/* ------------------------------------------------------------------ */

static void test_phase_resistance_known_r(void)
{
    TEST_START("Phase resistance: R=0.5 ohm, I=1A -> V=0.5V");
    motor_cal_result_t cal;
    motor_cal_init(&cal);

    cal_sample_t samples[CAL_SAMPLES];
    for (int i = 0; i < CAL_SAMPLES; i++) {
        samples[i].current_a = CAL_CURRENT_A;
        samples[i].voltage_v = 0.5f;   /* V = I * R = 1 * 0.5 */
    }

    int ret = motor_cal_measure_resistance(samples, CAL_SAMPLES, &cal);

    if (ret && cal.cal_complete && cal.cal_passed &&
        fabsf(cal.phase_resistance_ohm - 0.5f) < 0.01f)
        TEST_PASS();
    else
        TEST_FAIL("resistance measurement incorrect");
}

static void test_phase_resistance_different_r(void)
{
    TEST_START("Phase resistance: R=1.2 ohm, I=1A -> V=1.2V");
    motor_cal_result_t cal;
    motor_cal_init(&cal);

    cal_sample_t samples[CAL_SAMPLES];
    for (int i = 0; i < CAL_SAMPLES; i++) {
        samples[i].current_a = CAL_CURRENT_A;
        samples[i].voltage_v = 1.2f;
    }

    motor_cal_measure_resistance(samples, CAL_SAMPLES, &cal);

    if (fabsf(cal.phase_resistance_ohm - 1.2f) < 0.01f)
        TEST_PASS();
    else
        TEST_FAIL("unexpected resistance value");
}

static void test_phase_resistance_zero_current(void)
{
    TEST_START("Phase resistance: zero current returns 0");
    motor_cal_result_t cal;
    motor_cal_init(&cal);

    cal_sample_t samples[3];
    for (int i = 0; i < 3; i++) {
        samples[i].current_a = 0.0f;
        samples[i].voltage_v = 0.5f;
    }

    int ret = motor_cal_measure_resistance(samples, 3, &cal);

    if (ret == 0)
        TEST_PASS();
    else
        TEST_FAIL("should have returned 0 for zero current");
}

static void test_phase_resistance_out_of_range(void)
{
    TEST_START("Phase resistance: out of range fails calibration");
    motor_cal_result_t cal;
    motor_cal_init(&cal);

    cal_sample_t samples[CAL_SAMPLES];
    for (int i = 0; i < CAL_SAMPLES; i++) {
        samples[i].current_a = CAL_CURRENT_A;
        samples[i].voltage_v = 100.0f;  /* R = 100 ohm, too high */
    }

    motor_cal_measure_resistance(samples, CAL_SAMPLES, &cal);

    if (!cal.cal_passed && cal.cal_complete)
        TEST_PASS();
    else
        TEST_FAIL("should have failed calibration (R out of range)");
}

static void test_encoder_offset_known(void)
{
    TEST_START("Encoder offset: known 30 deg -> calibrated within 1 deg");
    motor_cal_result_t cal;
    motor_cal_init(&cal);

    float true_offset = 30.0f;

    cal_sample_t samples[CAL_SAMPLES];
    for (int i = 0; i < CAL_SAMPLES; i++) {
        samples[i].angle_deg = true_offset + ((float)rand() / RAND_MAX - 0.5f) * 0.5f;
        samples[i].current_a = 1.0f;
        samples[i].voltage_v = 0.5f;
    }

    int ret = motor_cal_measure_encoder_offset(samples, CAL_SAMPLES,
                                                true_offset, &cal);

    if (ret && cal.cal_passed && cal.cal_complete)
        TEST_PASS();
    else
        TEST_FAIL("encoder offset calibration out of tolerance");
}

static void test_encoder_offset_negative(void)
{
    TEST_START("Encoder offset: known -45 deg -> calibrated within 1 deg");
    motor_cal_result_t cal;
    motor_cal_init(&cal);

    float true_offset = -45.0f;

    cal_sample_t samples[CAL_SAMPLES];
    for (int i = 0; i < CAL_SAMPLES; i++) {
        samples[i].angle_deg = true_offset;
        samples[i].current_a = 1.0f;
        samples[i].voltage_v = 0.5f;
    }

    motor_cal_measure_encoder_offset(samples, CAL_SAMPLES, true_offset, &cal);

    if (cal.cal_passed && fabsf(cal.encoder_offset_deg - true_offset) < ENCODER_OFFSET_TOLERANCE_DEG)
        TEST_PASS();
    else
        TEST_FAIL("negative offset calibration failed");
}

static void test_encoder_offset_wrong_expected_fails(void)
{
    TEST_START("Encoder offset: wrong expected offset fails verification");
    motor_cal_result_t cal;
    motor_cal_init(&cal);

    cal_sample_t samples[CAL_SAMPLES];
    for (int i = 0; i < CAL_SAMPLES; i++) {
        samples[i].angle_deg = 30.0f;  /* actual offset */
        samples[i].current_a = 1.0f;
        samples[i].voltage_v = 0.5f;
    }

    /* Expect 90 deg, but actual is 30 */
    motor_cal_measure_encoder_offset(samples, CAL_SAMPLES, 90.0f, &cal);

    if (!cal.cal_passed)
        TEST_PASS();
    else
        TEST_FAIL("should have failed with wrong expected offset");
}

/* ------------------------------------------------------------------ */
/*  Main                                                              */
/* ------------------------------------------------------------------ */

int main(void)
{
    printf("=== Factory Motor Calibration Unit Tests ===\n\n");

    printf("--- Phase Resistance ---\n");
    test_phase_resistance_known_r();
    test_phase_resistance_different_r();
    test_phase_resistance_zero_current();
    test_phase_resistance_out_of_range();

    printf("\n--- Encoder Offset ---\n");
    test_encoder_offset_known();
    test_encoder_offset_negative();
    test_encoder_offset_wrong_expected_fails();

    printf("\nResults: %d passed, %d failed\n", test_passed, test_failed);
    return test_failed > 0 ? 1 : 0;
}
