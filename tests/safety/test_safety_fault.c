/**
 * test_safety_fault.c
 * Unit tests for fault detection (overcurrent, encoder disconnect, IMU freeze).
 *
 * Tests:
 *   - Overcurrent: ia=3.1A triggers fault flag within 1 ms
 *   - Encoder disconnect: encoder=0 for 100 ms -> fallback to sensorless
 *   - IMU freeze: gyro=0 for 500 ms -> alert generated
 *
 * Build:
 *   gcc -I. -I../../firmware -DUNIT_TEST test_safety_fault.c -o test_safety_fault
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

/* ------------------------------------------------------------------ */
/*  Types                                                             */
/* ------------------------------------------------------------------ */

#define FAULT_HISTORY_SIZE  10

typedef enum {
    FAULT_NONE            = 0,
    FAULT_OVERCURRENT     = 1,
    FAULT_ENCODER_LOST    = 2,
    FAULT_IMU_FREEZE      = 3,
    FAULT_OVERVOLTAGE     = 4,
    FAULT_UNDERVOLTAGE    = 5
} fault_type_t;

typedef struct {
    uint32_t fault_flags;          /* bitmask of active faults */
    uint32_t fault_history[FAULT_HISTORY_SIZE];
    int      history_head;
    int      sensorless_active;    /* 1 = running in sensorless mode */
    int      imu_alert_active;
} fault_state_t;

typedef struct {
    float ia;              /* phase A current (A) */
    float ib;
    float ic;
    int   encoder_valid;   /* 1 if encoder readings valid */
    float gyro_dps;        /* gyro Z in deg/s */
    float accel_mg;        /* accelerometer in mg */
    uint32_t timestamp_us;
} sensor_data_t;

/* Configuration */
#define OVERCURRENT_THRESHOLD_A     3.0f
#define FAULT_DEBOUNCE_US           1000   /* 1 ms */
#define ENCODER_LOST_TIMEOUT_US     100000 /* 100 ms */
#define IMU_FREEZE_TIMEOUT_US       500000 /* 500 ms */
#define IMU_FREEZE_GYRO_THRESHOLD   1.0f   /* deg/s */

/* DUT functions */
void fault_monitor_init(fault_state_t *fs);
void fault_monitor_process(fault_state_t *fs, const sensor_data_t *sensors, uint32_t now_us);

/* ------------------------------------------------------------------ */
/*  DUT implementation                                                */
/* ------------------------------------------------------------------ */

void fault_monitor_init(fault_state_t *fs)
{
    memset(fs, 0, sizeof(*fs));
    fs->sensorless_active = 0;
}

static void fault_history_push(fault_state_t *fs, uint32_t fault)
{
    fs->fault_history[fs->history_head % FAULT_HISTORY_SIZE] = fault;
    fs->history_head++;
}

static uint32_t overcurrent_start_us = 0;
static uint32_t encoder_lost_start_us = 0;
static uint32_t imu_freeze_start_us = 0;

void fault_monitor_process(fault_state_t *fs, const sensor_data_t *sensors, uint32_t now_us)
{
    uint32_t new_flags = 0;

    /* --- Overcurrent --- */
    float max_i = fmaxf(fmaxf(fabsf(sensors->ia), fabsf(sensors->ib)), fabsf(sensors->ic));
    if (max_i > OVERCURRENT_THRESHOLD_A) {
        if (overcurrent_start_us == 0)
            overcurrent_start_us = now_us;
        else if ((now_us - overcurrent_start_us) >= FAULT_DEBOUNCE_US)
            new_flags |= (1U << FAULT_OVERCURRENT);
    } else {
        overcurrent_start_us = 0;
    }

    /* --- Encoder disconnect --- */
    if (!sensors->encoder_valid) {
        if (encoder_lost_start_us == 0)
            encoder_lost_start_us = now_us;
        else if ((now_us - encoder_lost_start_us) >= ENCODER_LOST_TIMEOUT_US) {
            new_flags |= (1U << FAULT_ENCODER_LOST);
            fs->sensorless_active = 1;
        }
    } else {
        encoder_lost_start_us = 0;
        fs->sensorless_active = 0;
    }

    /* --- IMU freeze --- */
    if (fabsf(sensors->gyro_dps) < IMU_FREEZE_GYRO_THRESHOLD) {
        if (imu_freeze_start_us == 0)
            imu_freeze_start_us = now_us;
        else if ((now_us - imu_freeze_start_us) >= IMU_FREEZE_TIMEOUT_US) {
            new_flags |= (1U << FAULT_IMU_FREEZE);
            fs->imu_alert_active = 1;
        }
    } else {
        imu_freeze_start_us = 0;
    }

    /* Log new faults */
    if (new_flags != fs->fault_flags) {
        uint32_t changed = new_flags & ~fs->fault_flags;
        for (int f = 0; f < 32; f++) {
            if (changed & (1U << f))
                fault_history_push(fs, (uint32_t)f);
        }
        fs->fault_flags = new_flags;
    }
}

/* ------------------------------------------------------------------ */
/*  Test utilities                                                    */
/* ------------------------------------------------------------------ */

static int test_passed = 0;
static int test_failed = 0;

#define TEST_START(name)  do { printf("  TEST %-50s ", name); } while(0)
#define TEST_PASS()       do { printf("PASS\n"); test_passed++; } while(0)
#define TEST_FAIL(msg)    do { printf("FAIL: %s\n", msg); test_failed++; } while(0)

static sensor_data_t make_sensors(float ia, float ib, float ic,
                                  int encoder_valid, float gyro_dps)
{
    sensor_data_t sd;
    memset(&sd, 0, sizeof(sd));
    sd.ia = ia;
    sd.ib = ib;
    sd.ic = ic;
    sd.encoder_valid = encoder_valid;
    sd.gyro_dps      = gyro_dps;
    sd.timestamp_us  = 0;
    return sd;
}

/* ------------------------------------------------------------------ */
/*  Tests                                                             */
/* ------------------------------------------------------------------ */

static void test_overcurrent_detection(void)
{
    TEST_START("Overcurrent ia=3.1A triggers fault within 1ms");
    fault_state_t fs;
    fault_monitor_init(&fs);
    overcurrent_start_us = 0;

    sensor_data_t sd = make_sensors(3.1f, 0.5f, 0.5f, 1, 10.0f);

    /* At t=0, should NOT trigger yet (debounce) */
    fault_monitor_process(&fs, &sd, 0);
    if (fs.fault_flags & (1U << FAULT_OVERCURRENT)) { TEST_FAIL("triggered too early"); return; }

    /* At t=1500 us, should trigger */
    fault_monitor_process(&fs, &sd, 1500);
    if (fs.fault_flags & (1U << FAULT_OVERCURRENT))
        TEST_PASS();
    else
        TEST_FAIL("overcurrent fault not set after debounce");
}

static void test_overcurrent_clears_when_current_drops(void)
{
    TEST_START("Overcurrent flag clears when current drops below threshold");
    fault_state_t fs;
    fault_monitor_init(&fs);
    overcurrent_start_us = 0;

    sensor_data_t sd = make_sensors(3.1f, 0.5f, 0.5f, 1, 10.0f);
    fault_monitor_process(&fs, &sd, 0);
    fault_monitor_process(&fs, &sd, 1500);

    if (!(fs.fault_flags & (1U << FAULT_OVERCURRENT))) { TEST_FAIL("fault not set"); return; }

    /* Current drops to normal */
    sd.ia = 0.5f;
    overcurrent_start_us = 0;  /* simulated clear */
    fault_monitor_process(&fs, &sd, 2000);

    if (!(fs.fault_flags & (1U << FAULT_OVERCURRENT)))
        TEST_PASS();
    else
        TEST_FAIL("overcurrent flag should have cleared");
}

static void test_encoder_disconnect(void)
{
    TEST_START("Encoder disconnect -> fallback to sensorless");
    fault_state_t fs;
    fault_monitor_init(&fs);
    encoder_lost_start_us = 0;

    sensor_data_t sd = make_sensors(0.5f, 0.5f, 0.5f, 0, 10.0f);

    /* t=0 -> no fault */
    fault_monitor_process(&fs, &sd, 0);
    if (fs.sensorless_active) { TEST_FAIL("premature sensorless fallback"); return; }

    /* t=101ms -> fault and sensorless active */
    fault_monitor_process(&fs, &sd, 101000);
    if (fs.sensorless_active && (fs.fault_flags & (1U << FAULT_ENCODER_LOST)))
        TEST_PASS();
    else
        TEST_FAIL("expected sensorless fallback after 100ms");
}

static void test_encoder_recovers(void)
{
    TEST_START("Encoder recovers -> exit sensorless mode");
    fault_state_t fs;
    fault_monitor_init(&fs);
    encoder_lost_start_us = 0;

    sensor_data_t sd = make_sensors(0.5f, 0.5f, 0.5f, 0, 10.0f);
    fault_monitor_process(&fs, &sd, 0);
    fault_monitor_process(&fs, &sd, 101000);
    if (!fs.sensorless_active) { TEST_FAIL("should be in sensorless"); return; }

    /* Encoder recovers */
    sd.encoder_valid = 1;
    encoder_lost_start_us = 0;
    fault_monitor_process(&fs, &sd, 102000);

    if (!fs.sensorless_active)
        TEST_PASS();
    else
        TEST_FAIL("should have exited sensorless mode");
}

static void test_imu_freeze(void)
{
    TEST_START("IMU freeze (gyro=0 for 500ms) -> alert");
    fault_state_t fs;
    fault_monitor_init(&fs);
    imu_freeze_start_us = 0;

    sensor_data_t sd = make_sensors(0.5f, 0.5f, 0.5f, 1, 0.0f);

    fault_monitor_process(&fs, &sd, 0);
    fault_monitor_process(&fs, &sd, 600000);  /* 600ms */

    if (fs.imu_alert_active && (fs.fault_flags & (1U << FAULT_IMU_FREEZE)))
        TEST_PASS();
    else
        TEST_FAIL("expected IMU freeze alert after 500ms");
}

static void test_imu_freeze_no_false_positive(void)
{
    TEST_START("IMU no false positive with normal gyro readings");
    fault_state_t fs;
    fault_monitor_init(&fs);
    imu_freeze_start_us = 0;

    sensor_data_t sd = make_sensors(0.5f, 0.5f, 0.5f, 1, 25.0f);

    for (uint32_t t = 0; t < 1000000; t += 10000)
        fault_monitor_process(&fs, &sd, t);

    if (!fs.imu_alert_active)
        TEST_PASS();
    else
        TEST_FAIL("false IMU freeze alert with normal gyro");
}

/* ------------------------------------------------------------------ */
/*  Main                                                              */
/* ------------------------------------------------------------------ */

int main(void)
{
    printf("=== Safety Fault Detection Unit Tests ===\n\n");

    test_overcurrent_detection();
    test_overcurrent_clears_when_current_drops();
    test_encoder_disconnect();
    test_encoder_recovers();
    test_imu_freeze();
    test_imu_freeze_no_false_positive();

    printf("\nResults: %d passed, %d failed\n", test_passed, test_failed);
    return test_failed > 0 ? 1 : 0;
}
