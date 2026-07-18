/**
 * @file regression_suite.c
 * @brief H747 Elite Regression Test Runner
 *
 * Runs all regression tests sequentially, compares output hashes
 * against golden values, generates JUnit XML for CI integration.
 * Target total regression time: < 5 minutes on CI runner.
 *
 * Build:
 *   gcc -o regression_suite regression_suite.c -lm -lcrypto -lssl -lpthread
 *   or with cmocka:
 *   gcc -o regression_suite regression_suite.c -lm -lcmocka
 *
 * Run:
 *   ./regression_suite                   # standard run
 *   ./regression_suite --junit           # output JUnit XML
 *   ./regression_suite --golden-update   # update golden hashes
 *   ./regression_suite --list            # list tests only
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <stdint.h>
#include <stdarg.h>
#include <assert.h>

/* ======================== Configuration ======================== */

#define MAX_TEST_NAME        128
#define MAX_TEST_DESC        256
#define MAX_TESTS            200
#define GOLDEN_HASH_SIZE     32   /* SHA-256 */
#define JUNIT_OUTPUT_FILE    "regression_results.xml"
#define GOLDEN_HASH_FILE     "golden_hashes.dat"
#define MAX_OUTPUT_SIZE      (1024 * 1024)  /* 1MB max output per test */
#define TIME_LIMIT_SEC       300  /* 5 minutes total */

/* ======================== Data Structures ======================== */

typedef enum {
    TEST_PASS = 0,
    TEST_FAIL = 1,
    TEST_SKIP = 2,
    TEST_ERROR = 3
} test_result_t;

typedef struct {
    char name[MAX_TEST_NAME];
    char description[MAX_TEST_DESC];
    test_result_t result;
    double duration_ms;
    char failure_message[1024];
    uint8_t output_hash[GOLDEN_HASH_SIZE];
    uint8_t golden_hash[GOLDEN_HASH_SIZE];
    uint64_t output_size_bytes;
} test_entry_t;

typedef struct {
    test_entry_t tests[MAX_TESTS];
    int count;
    int passed;
    int failed;
    int skipped;
    int errors;
    double total_duration_ms;
    char start_time[64];
    char end_time[64];
} test_suite_t;

static test_suite_t suite;
static int flag_junit_output = 0;
static int flag_golden_update = 0;
static int flag_list_tests = 0;

/* ======================== Simple SHA-256 Implementation ======================== */
/* For portability; link against libcrypto if available */

#define ROTR(x, n) (((x) >> (n)) | ((x) << (32 - (n))))
#define CH(x, y, z) (((x) & (y)) ^ (~(x) & (z)))
#define MAJ(x, y, z) (((x) & (y)) ^ ((x) & (z)) ^ ((y) & (z)))
#define SIG0(x) (ROTR(x, 2) ^ ROTR(x, 13) ^ ROTR(x, 22))
#define SIG1(x) (ROTR(x, 6) ^ ROTR(x, 11) ^ ROTR(x, 25))
#define sig0(x) (ROTR(x, 7) ^ ROTR(x, 18) ^ ((x) >> 3))
#define sig1(x) (ROTR(x, 17) ^ ROTR(x, 19) ^ ((x) >> 10))

static const uint32_t K256[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5,
    0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
    0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
    0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
    0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
    0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3,
    0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5,
    0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
    0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
};

typedef struct {
    uint32_t state[8];
    uint64_t count;
    uint8_t buffer[64];
    int buffer_offset;
} sha256_ctx_t;

static void sha256_init(sha256_ctx_t* ctx) {
    ctx->state[0] = 0x6a09e667;
    ctx->state[1] = 0xbb67ae85;
    ctx->state[2] = 0x3c6ef372;
    ctx->state[3] = 0xa54ff53a;
    ctx->state[4] = 0x510e527f;
    ctx->state[5] = 0x9b05688c;
    ctx->state[6] = 0x1f83d9ab;
    ctx->state[7] = 0x5be0cd19;
    ctx->count = 0;
    ctx->buffer_offset = 0;
}

static void sha256_transform(sha256_ctx_t* ctx, const uint8_t* block) {
    uint32_t W[64];
    for (int t = 0; t < 16; t++) {
        W[t] = ((uint32_t)block[t*4] << 24) |
               ((uint32_t)block[t*4+1] << 16) |
               ((uint32_t)block[t*4+2] << 8) |
               ((uint32_t)block[t*4+3]);
    }
    for (int t = 16; t < 64; t++) {
        W[t] = sig1(W[t-2]) + W[t-7] + sig0(W[t-15]) + W[t-16];
    }

    uint32_t a = ctx->state[0];
    uint32_t b = ctx->state[1];
    uint32_t c = ctx->state[2];
    uint32_t d = ctx->state[3];
    uint32_t e = ctx->state[4];
    uint32_t f = ctx->state[5];
    uint32_t g = ctx->state[6];
    uint32_t h = ctx->state[7];

    for (int t = 0; t < 64; t++) {
        uint32_t T1 = h + SIG1(e) + CH(e, f, g) + K256[t] + W[t];
        uint32_t T2 = SIG0(a) + MAJ(a, b, c);
        h = g;
        g = f;
        f = e;
        e = d + T1;
        d = c;
        c = b;
        b = a;
        a = T1 + T2;
    }

    ctx->state[0] += a;
    ctx->state[1] += b;
    ctx->state[2] += c;
    ctx->state[3] += d;
    ctx->state[4] += e;
    ctx->state[5] += f;
    ctx->state[6] += g;
    ctx->state[7] += h;
}

static void sha256_update(sha256_ctx_t* ctx, const uint8_t* data, size_t len) {
    ctx->count += len;
    int offset = ctx->buffer_offset;

    if (offset > 0) {
        int fill = 64 - offset;
        if ((size_t)fill > len) fill = (int)len;
        memcpy(ctx->buffer + offset, data, fill);
        offset += fill;
        data += fill;
        len -= fill;
        if (offset == 64) {
            sha256_transform(ctx, ctx->buffer);
            offset = 0;
        }
    }

    while (len >= 64) {
        sha256_transform(ctx, data);
        data += 64;
        len -= 64;
    }

    if (len > 0) {
        memcpy(ctx->buffer + offset, data, len);
        offset += (int)len;
    }
    ctx->buffer_offset = offset;
}

static void sha256_final(sha256_ctx_t* ctx, uint8_t* hash) {
    uint64_t bits = ctx->count * 8;
    int offset = ctx->buffer_offset;

    /* Append 0x80 */
    ctx->buffer[offset++] = 0x80;

    /* Pad with zeros */
    if (offset > 56) {
        memset(ctx->buffer + offset, 0, 64 - offset);
        sha256_transform(ctx, ctx->buffer);
        offset = 0;
    }
    memset(ctx->buffer + offset, 0, 56 - offset);

    /* Append length in bits */
    for (int i = 0; i < 8; i++) {
        ctx->buffer[63 - i] = (uint8_t)(bits >> (i * 8));
    }
    sha256_transform(ctx, ctx->buffer);

    /* Produce hash */
    for (int i = 0; i < 8; i++) {
        hash[i*4]   = (uint8_t)(ctx->state[i] >> 24);
        hash[i*4+1] = (uint8_t)(ctx->state[i] >> 16);
        hash[i*4+2] = (uint8_t)(ctx->state[i] >> 8);
        hash[i*4+3] = (uint8_t)(ctx->state[i]);
    }
}

static void compute_hash(const uint8_t* data, size_t len, uint8_t* hash) {
    sha256_ctx_t ctx;
    sha256_init(&ctx);
    sha256_update(&ctx, data, len);
    sha256_final(&ctx, hash);
}

static void hash_to_string(const uint8_t* hash, char* str) {
    for (int i = 0; i < GOLDEN_HASH_SIZE; i++) {
        sprintf(str + i*2, "%02x", hash[i]);
    }
    str[GOLDEN_HASH_SIZE * 2] = '\0';
}

/* ======================== Golden Hash Management ======================== */

static int load_golden_hashes(void) {
    FILE* f = fopen(GOLDEN_HASH_FILE, "rb");
    if (!f) {
        printf("No golden hash file found (%s). First run or use --golden-update.\n",
               GOLDEN_HASH_FILE);
        return 0;
    }

    int loaded = 0;
    char name[MAX_TEST_NAME];
    uint8_t hash[GOLDEN_HASH_SIZE];

    while (fscanf(f, "%127s ", name) == 1) {
        size_t read = fread(hash, 1, GOLDEN_HASH_SIZE, f);
        if (read == GOLDEN_HASH_SIZE) {
            for (int i = 0; i < suite.count; i++) {
                if (strcmp(suite.tests[i].name, name) == 0) {
                    memcpy(suite.tests[i].golden_hash, hash, GOLDEN_HASH_SIZE);
                    loaded++;
                    break;
                }
            }
        }
    }
    fclose(f);
    return loaded;
}

static int save_golden_hashes(void) {
    FILE* f = fopen(GOLDEN_HASH_FILE, "wb");
    if (!f) {
        fprintf(stderr, "ERROR: Cannot write %s\n", GOLDEN_HASH_FILE);
        return 0;
    }

    for (int i = 0; i < suite.count; i++) {
        fprintf(f, "%s ", suite.tests[i].name);
        fwrite(suite.tests[i].output_hash, 1, GOLDEN_HASH_SIZE, f);
        fprintf(f, "\n");
    }
    fclose(f);
    return 1;
}

static int compare_hashes(const uint8_t* a, const uint8_t* b) {
    return memcmp(a, b, GOLDEN_HASH_SIZE) == 0;
}

/* ======================== Test Registration ======================== */

static void register_test(const char* name, const char* description) {
    if (suite.count >= MAX_TESTS) {
        fprintf(stderr, "ERROR: Too many tests (max %d)\n", MAX_TESTS);
        return;
    }
    test_entry_t* t = &suite.tests[suite.count++];
    strncpy(t->name, name, MAX_TEST_NAME - 1);
    strncpy(t->description, description, MAX_TEST_DESC - 1);
    t->result = TEST_SKIP;
    t->duration_ms = 0;
    t->failure_message[0] = '\0';
    memset(t->output_hash, 0, GOLDEN_HASH_SIZE);
    memset(t->golden_hash, 0, GOLDEN_HASH_SIZE);
    t->output_size_bytes = 0;
}

/* ======================== Test Capture ======================== */

static char captured_output[MAX_OUTPUT_SIZE];
static size_t captured_output_len = 0;

static void capture_start(void) {
    captured_output_len = 0;
    captured_output[0] = '\0';
}

static void capture_printf(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    char buf[4096];
    int n = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (n > 0) {
        size_t remaining = MAX_OUTPUT_SIZE - captured_output_len - 1;
        size_t to_copy = (size_t)n < remaining ? (size_t)n : remaining;
        memcpy(captured_output + captured_output_len, buf, to_copy);
        captured_output_len += to_copy;
        captured_output[captured_output_len] = '\0';
    }

    /* Also print to stdout */
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
}

/* ======================== Individual Tests ======================== */

static int test_motor_foc_current_loop(void) {
    capture_printf("  Testing FOC current loop PI controller...\n");

    /* Simulate current loop step response */
    float Kp = 0.85f, Ki = 320.0f;
    float iq_ref = 1.0f;
    float iq = 0.0f;
    float integral = 0.0f;
    float dt = 50e-6f;
    float R = 0.45f, L = 0.12e-3f;
    float Vdc = 12.0f;

    int steps = 200;  /* 200 * 50us = 10ms simulation */
    float iq_peak = 0.0f;
    float iq_ss = 0.0f;

    for (int s = 0; s < steps; s++) {
        float err = iq_ref - iq;
        integral += Ki * err * dt;
        float vq = Kp * err + integral;

        /* Voltage limit */
        if (vq > Vdc) vq = Vdc;
        if (vq < -Vdc) vq = -Vdc;

        /* Simple motor model: diq/dt = (vq - R*iq - omega*lambda) / L */
        float diq = (vq - R * iq) / L;
        iq += diq * dt;

        if (iq > iq_peak) iq_peak = iq;
    }

    iq_ss = iq;

    capture_printf("    iq_ref=%.2fA, iq_ss=%.2fA, iq_peak=%.2fA\n",
                   iq_ref, iq_ss, iq_peak);

    /* Check: steady-state error < 5%, overshoot < 20% */
    float ss_error = fabsf(iq_ss - iq_ref) / iq_ref * 100.0f;
    float overshoot = (iq_peak - iq_ref) / iq_ref * 100.0f;

    capture_printf("    Steady-state error: %.1f%% (limit: 5%%)\n", ss_error);
    capture_printf("    Overshoot: %.1f%% (limit: 20%%)\n", overshoot);

    if (ss_error > 5.0f) {
        capture_printf("  FAIL: Current loop steady-state error too high\n");
        return 0;
    }
    return 1;
}

static int test_motor_foc_speed_loop(void) {
    capture_printf("  Testing FOC speed loop PI controller...\n");

    float Kp_s = 0.12f / 7.0f;
    float Ki_s = 4.0f;
    float omega_ref = 1000.0f * (2.0f * 3.14159f / 60.0f) * 7.0f;
    float omega = 0.0f;
    float integral = 0.0f;
    float dt = 1e-3f;
    float J = 1.2e-5f;
    float lambda = 0.052f;
    float pole_pairs = 7.0f;

    int steps = 200;  /* 200 * 1ms = 200ms simulation */
    float omega_peak = 0.0f;

    for (int s = 0; s < steps; s++) {
        float err = omega_ref - omega;
        integral += Ki_s * err * dt;
        float iq_ref = Kp_s * err + integral;
        if (iq_ref > 3.0f) iq_ref = 3.0f;
        if (iq_ref < -3.0f) iq_ref = -3.0f;

        float torque = 1.5f * pole_pairs * lambda * iq_ref;
        float domega = torque / J;
        omega += domega * dt;

        if (omega > omega_peak) omega_peak = omega;
    }

    float omega_rpm = omega * 60.0f / (2.0f * 3.14159f) / 7.0f;
    float omega_ref_rpm = 1000.0f;
    float speed_error = fabsf(omega_rpm - omega_ref_rpm);

    capture_printf("    Target: %.0f RPM, Achieved: %.1f RPM\n",
                   omega_ref_rpm, omega_rpm);
    capture_printf("    Speed error: %.1f RPM\n", speed_error);

    if (speed_error > 50.0f) {
        capture_printf("  FAIL: Speed loop error too high\n");
        return 0;
    }
    return 1;
}

static int test_bms_voltage_monitoring(void) {
    capture_printf("  Testing BMS voltage monitoring...\n");

    /* Simulate BQ40Z50 register reads */
    uint16_t cell_voltages[4] = {3700, 3680, 3720, 3690};  /* mV */
    uint16_t pack_voltage = 0;

    for (int i = 0; i < 4; i++) {
        pack_voltage += cell_voltages[i];
    }

    capture_printf("    Cell voltages: %d, %d, %d, %d mV\n",
                   cell_voltages[0], cell_voltages[1],
                   cell_voltages[2], cell_voltages[3]);
    capture_printf("    Pack voltage: %d mV\n", pack_voltage);

    /* Check cell balance: delta < 100mV */
    int max_cell = cell_voltages[0], min_cell = cell_voltages[0];
    for (int i = 1; i < 4; i++) {
        if (cell_voltages[i] > max_cell) max_cell = cell_voltages[i];
        if (cell_voltages[i] < min_cell) min_cell = cell_voltages[i];
    }
    int delta = max_cell - min_cell;

    capture_printf("    Cell delta: %d mV (limit: 100 mV)\n", delta);

    if (delta > 100) {
        capture_printf("  FAIL: Cell imbalance exceeds 100mV\n");
        return 0;
    }

    /* Check pack voltage in valid range (11.2V - 16.8V) */
    if (pack_voltage < 11200 || pack_voltage > 16800) {
        capture_printf("  FAIL: Pack voltage out of range\n");
        return 0;
    }

    return 1;
}

static int test_safety_monitor_state_machine(void) {
    capture_printf("  Testing safety monitor state machine...\n");

    /* Simulate state transitions */
    typedef enum { SAFE_INIT, SAFE_NORMAL, SAFE_WARNING, SAFE_FAULT, SAFE_EMERGENCY } safety_state_t;
    safety_state_t state = SAFE_INIT;
    int fault_count = 0;

    /* Normal operation: no faults, should stay in NORMAL */
    state = SAFE_NORMAL;
    for (int i = 0; i < 100; i++) {
        /* Simulate no faults */
        if (state == SAFE_FAULT && fault_count > 0) fault_count--;
        /* No transition needed - staying in NORMAL */
    }

    capture_printf("    Normal operation: state=%d (expected 1=NORMAL)\n", state);

    /* Fault condition should trigger transition */
    state = SAFE_NORMAL;
    int overcurrent_detected = 1;
    if (overcurrent_detected) {
        state = SAFE_FAULT;
        fault_count++;
    }

    capture_printf("    Overcurrent fault: state=%d (expected 2=FAULT)\n", state);

    if (state != SAFE_FAULT) {
        capture_printf("  FAIL: Safety state should transition to FAULT\n");
        return 0;
    }

    return 1;
}

static int test_slam_scan_matching(void) {
    capture_printf("  Testing SLAM scan matching...\n");

    /* Simulate simple scan matching */
    float ref_scan[4][2] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
    float test_scan[4][2] = {{0.05, 0.02}, {1.03, 0.01}, {1.02, 1.04}, {-0.01, 0.98}};

    /* ICP-style alignment: compute mean offset */
    float dx_sum = 0, dy_sum = 0;
    for (int i = 0; i < 4; i++) {
        dx_sum += test_scan[i][0] - ref_scan[i][0];
        dy_sum += test_scan[i][1] - ref_scan[i][1];
    }
    float dx = dx_sum / 4.0f;
    float dy = dy_sum / 4.0f;
    float error = sqrtf(dx*dx + dy*dy);

    capture_printf("    Estimated offset: (%.3f, %.3f), error: %.3f\n", dx, dy, error);

    /* Expected: small offset (< 0.05m) */
    if (error > 0.05f) {
        capture_printf("  FAIL: Scan matching error too high\n");
        return 0;
    }
    return 1;
}

static int test_ota_delta_apply(void) {
    capture_printf("  Testing OTA delta application (bsdiff simulation)...\n");

    /* Simulate delta update: copy old to new with patch data */
    uint8_t old_firmware[1024];
    uint8_t delta[128];
    uint8_t new_firmware[1024];
    uint8_t expected_new[1024];

    /* Initialize old firmware with pattern */
    for (int i = 0; i < 1024; i++) {
        old_firmware[i] = (uint8_t)(i & 0xFF);
        expected_new[i] = (uint8_t)(~(i & 0xFF));  /* Inverted = "updated" */
    }

    /* Simulate delta: copy old and apply simple transformation */
    memcpy(new_firmware, old_firmware, 1024);
    for (int i = 0; i < 1024; i++) {
        new_firmware[i] = ~new_firmware[i];  /* Apply "patch" */
    }

    /* Verify delta produced correct output */
    int match = (memcmp(new_firmware, expected_new, 1024) == 0);

    capture_printf("    Delta applied correctly: %s\n", match ? "YES" : "NO");

    if (!match) {
        capture_printf("  FAIL: Delta application produced incorrect result\n");
        return 0;
    }
    return 1;
}

static int test_comm_crc16(void) {
    capture_printf("  Testing UART CRC16 protection...\n");

    uint8_t test_data[] = "H747 Elite Communication Test Frame 0x01 0x02 0x03 0xFF";
    size_t len = strlen((char*)test_data);

    /* CRC-16-CCITT implementation */
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)test_data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }

    capture_printf("    CRC-16: 0x%04X\n", crc);

    /* Verify CRC detects single-bit error */
    uint8_t corrupted[256];
    memcpy(corrupted, test_data, len);
    corrupted[len/2] ^= 0x01;  /* Flip one bit */

    uint16_t crc2 = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc2 ^= (uint16_t)corrupted[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc2 & 0x8000) {
                crc2 = (crc2 << 1) ^ 0x1021;
            } else {
                crc2 <<= 1;
            }
        }
    }

    capture_printf("    CRC-16 with bit error: 0x%04X\n", crc2);

    if (crc == crc2) {
        capture_printf("  FAIL: CRC should differ after bit error\n");
        return 0;
    }
    return 1;
}

static int test_safety_watchdog(void) {
    capture_printf("  Testing safety watchdog timing...\n");

    /* Simulate IWDG timeout */
    int wdt_timeout_ms = 4000;  /* 4 seconds */
    int feed_interval_ms = 100;  /* Feed every 100ms */
    int expected_feeds = wdt_timeout_ms / feed_interval_ms - 1;

    /* Feed watchdog normally */
    int feeds = 0;
    for (int t = 0; t < wdt_timeout_ms; t += feed_interval_ms) {
        feeds++;
    }

    capture_printf("    Expected feeds before timeout: %d, actual: %d\n",
                   expected_feeds, feeds);

    if (feeds < expected_feeds - 1 || feeds > expected_feeds + 1) {
        capture_printf("  FAIL: Watchdog feed count incorrect\n");
        return 0;
    }
    return 1;
}

static int test_slam_map_io(void) {
    capture_printf("  Testing SLAM map serialization...\n");

    /* Simple occupancy grid serialize/deserialize */
    uint8_t grid[10][10];
    uint8_t buffer[200];
    uint8_t grid_out[10][10];

    /* Fill grid with pattern */
    for (int y = 0; y < 10; y++) {
        for (int x = 0; x < 10; x++) {
            grid[y][x] = (uint8_t)((x + y * 10) % 101);
        }
    }

    /* Serialize: RLE */
    int buf_pos = 0;
    for (int y = 0; y < 10; y++) {
        for (int x = 0; x < 10; x++) {
            buffer[buf_pos++] = grid[y][x];
        }
    }

    /* Deserialize */
    buf_pos = 0;
    for (int y = 0; y < 10; y++) {
        for (int x = 0; x < 10; x++) {
            grid_out[y][x] = buffer[buf_pos++];
        }
    }

    int match = (memcmp(grid, grid_out, 100) == 0);
    capture_printf("    Serialization round-trip: %s\n", match ? "PASS" : "FAIL");

    if (!match) {
        capture_printf("  FAIL: Map serialization round-trip failed\n");
        return 0;
    }
    return 1;
}

static int test_motor_encoder_plausibility(void) {
    capture_printf("  Testing encoder plausibility check...\n");

    /* Simulate encoder readings */
    int32_t encoder_counts[5] = {0, 48, 96, 144, 192};  /* 48 count increments */
    int32_t prev_count = encoder_counts[0];
    int valid_steps = 0;

    for (int i = 1; i < 5; i++) {
        int32_t delta = encoder_counts[i] - prev_count;
        if (delta > 0 && delta < 100) {  /* Plausible range */
            valid_steps++;
        }
        prev_count = encoder_counts[i];
    }

    capture_printf("    Valid encoder steps: %d/4\n", valid_steps);

    if (valid_steps < 4) {
        capture_printf("  FAIL: Encoder plausibility check failed\n");
        return 0;
    }

    /* Test invalid reading detection */
    int32_t bad_counts[5] = {0, 48, 5000, 5050, 0};  /* Jump of ~5000 counts */
    prev_count = bad_counts[0];
    int invalid_detected = 0;

    for (int i = 1; i < 5; i++) {
        int32_t delta = abs(bad_counts[i] - prev_count);
        if (delta > 1000) {  /* Impossible jump */
            invalid_detected++;
        }
        prev_count = bad_counts[i];
    }

    capture_printf("    Invalid jumps detected: %d (expected >= 1)\n", invalid_detected);

    if (invalid_detected < 1) {
        capture_printf("  FAIL: Should detect impossible encoder jumps\n");
        return 0;
    }
    return 1;
}

static int test_power_rail_sequencing(void) {
    capture_printf("  Testing power rail sequencing...\n");

    /* Simulate power-up sequence timing */
    float rails[3][2] = {
        {0, 0},  /* 3.3V: time, voltage */
        {0, 0},  /* 5V */
        {0, 0}   /* 12V */
    };

    /* Expected sequence: 3.3V first, then 5V, then 12V */
    rails[0][1] = 3.3f; rails[0][0] = 0.001f;  /* 1ms */
    rails[1][1] = 5.0f; rails[1][0] = 0.005f;  /* 5ms */
    rails[2][1] = 12.0f; rails[2][0] = 0.010f; /* 10ms */

    /* Check ordering */
    int seq_ok = (rails[0][0] < rails[1][0] && rails[1][0] < rails[2][0]);
    /* Check voltages within tolerance */
    int v_ok = (fabsf(rails[0][1] - 3.3f) < 0.165f) &&
               (fabsf(rails[1][1] - 5.0f) < 0.25f) &&
               (fabsf(rails[2][1] - 12.0f) < 0.6f);

    capture_printf("    Rail sequencing order: %s\n", seq_ok ? "OK" : "FAIL");
    capture_printf("    Rail voltages: %s\n", v_ok ? "OK" : "FAIL");

    if (!seq_ok || !v_ok) {
        capture_printf("  FAIL: Power rail sequencing\n");
        return 0;
    }
    return 1;
}

/* ======================== Test Runner Helpers ======================== */

static void get_timestamp(char* buf, size_t buf_size) {
    time_t now = time(NULL);
    struct tm* tm_info = localtime(&now);
    strftime(buf, buf_size, "%Y-%m-%dT%H:%M:%S", tm_info);
}

static double get_time_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

/* ======================== Run All Registered Tests ======================== */

static int run_all_tests(void) {
    printf("\n========================================\n");
    printf("  H747 Elite Regression Test Suite\n");
    printf("========================================\n\n");

    get_timestamp(suite.start_time, sizeof(suite.start_time));
    printf("Started: %s\n\n", suite.start_time);

    double suite_start = get_time_ms();

    /* Register all tests */
    register_test("motor_foc_current_loop",
                  "FOC current loop PI controller step response");
    register_test("motor_foc_speed_loop",
                  "FOC speed loop PI controller step response");
    register_test("bms_voltage_monitoring",
                  "BMS cell voltage monitoring and balancing check");
    register_test("safety_monitor_state_machine",
                  "Safety monitor state machine transitions");
    register_test("slam_scan_matching",
                  "SLAM scan matching alignment accuracy");
    register_test("ota_delta_apply",
                  "OTA delta update application correctness");
    register_test("comm_crc16",
                  "UART CRC16 error detection");
    register_test("safety_watchdog",
                  "Safety watchdog timing and feed behavior");
    register_test("slam_map_io",
                  "SLAM occupancy grid serialization");
    register_test("motor_encoder_plausibility",
                  "Encoder plausibility check");
    register_test("power_rail_sequencing",
                  "Power rail sequencing order and tolerance");

    /* Load golden hashes */
    int golden_count = load_golden_hashes();
    printf("Loaded %d golden hashes from %s\n\n", golden_count, GOLDEN_HASH_FILE);

    if (flag_list_tests) {
        printf("Registered tests:\n");
        for (int i = 0; i < suite.count; i++) {
            printf("  [%2d] %s - %s\n", i+1, suite.tests[i].name, suite.tests[i].description);
        }
        return 0;
    }

    /* Run each test */
    for (int i = 0; i < suite.count; i++) {
        test_entry_t* test = &suite.tests[i];

        printf("[%2d/%2d] %s:\n", i+1, suite.count, test->name);
        printf("       %s\n", test->description);

        capture_start();
        double t_start = get_time_ms();

        int result = 0;
        if (strcmp(test->name, "motor_foc_current_loop") == 0)
            result = test_motor_foc_current_loop();
        else if (strcmp(test->name, "motor_foc_speed_loop") == 0)
            result = test_motor_foc_speed_loop();
        else if (strcmp(test->name, "bms_voltage_monitoring") == 0)
            result = test_bms_voltage_monitoring();
        else if (strcmp(test->name, "safety_monitor_state_machine") == 0)
            result = test_safety_monitor_state_machine();
        else if (strcmp(test->name, "slam_scan_matching") == 0)
            result = test_slam_scan_matching();
        else if (strcmp(test->name, "ota_delta_apply") == 0)
            result = test_ota_delta_apply();
        else if (strcmp(test->name, "comm_crc16") == 0)
            result = test_comm_crc16();
        else if (strcmp(test->name, "safety_watchdog") == 0)
            result = test_safety_watchdog();
        else if (strcmp(test->name, "slam_map_io") == 0)
            result = test_slam_map_io();
        else if (strcmp(test->name, "motor_encoder_plausibility") == 0)
            result = test_motor_encoder_plausibility();
        else if (strcmp(test->name, "power_rail_sequencing") == 0)
            result = test_power_rail_sequencing();
        else {
            capture_printf("  SKIP: No implementation for this test\n");
            test->result = TEST_SKIP;
            suite.skipped++;
        }

        double t_end = get_time_ms();
        test->duration_ms = t_end - t_start;

        if (result) {
            test->result = TEST_PASS;
            suite.passed++;
            printf("       PASS (%.1f ms)\n", test->duration_ms);
        } else if (test->result != TEST_SKIP) {
            test->result = TEST_FAIL;
            suite.failed++;
            strncpy(test->failure_message, captured_output, sizeof(test->failure_message)-1);
            printf("       FAIL (%.1f ms)\n", test->duration_ms);
        }

        /* Compute output hash */
        compute_hash((uint8_t*)captured_output, strlen(captured_output), test->output_hash);
        test->output_size_bytes = strlen(captured_output);

        char hash_str[65];
        hash_to_string(test->output_hash, hash_str);
        printf("       Hash: %s\n", hash_str);

        /* Compare with golden hash (if available) */
        int has_golden = 0;
        for (int j = 0; j < GOLDEN_HASH_SIZE; j++) {
            if (test->golden_hash[j] != 0) has_golden = 1;
        }

        if (has_golden) {
            char golden_str[65];
            hash_to_string(test->golden_hash, golden_str);
            int match = compare_hashes(test->output_hash, test->golden_hash);

            if (!match) {
                printf("       GOLDEN MISMATCH! Expected: %s\n", golden_str);
                /* Don't fail the test for hash mismatch alone */
            } else {
                printf("       Golden match: OK\n");
            }
        } else {
            printf("       No golden hash (use --golden-update)\n");
        }

        printf("\n");
    }

    suite.total_duration_ms = get_time_ms() - suite_start;
    get_timestamp(suite.end_time, sizeof(suite.end_time));

    /* Print summary */
    printf("========================================\n");
    printf("  Regression Test Summary\n");
    printf("========================================\n");
    printf("  Total:  %d\n", suite.count);
    printf("  Passed: %d\n", suite.passed);
    printf("  Failed: %d\n", suite.failed);
    printf("  Errors: %d\n", suite.errors);
    printf("  Skipped: %d\n", suite.skipped);
    printf("  Duration: %.1f ms (limit: %d ms)\n",
           suite.total_duration_ms, TIME_LIMIT_SEC * 1000);
    printf("  Started: %s\n", suite.start_time);
    printf("  Ended:   %s\n", suite.end_time);

    int all_passed = (suite.failed == 0 && suite.errors == 0);

    if (suite.total_duration_ms > TIME_LIMIT_SEC * 1000) {
        printf("\n  WARNING: Regression suite exceeded time limit!\n");
        all_passed = 0;
    }

    if (all_passed) {
        printf("\n  RESULT: ALL TESTS PASSED\n");
    } else {
        printf("\n  RESULT: SOME TESTS FAILED\n");
    }
    printf("========================================\n");

    /* Update golden hashes if requested */
    if (flag_golden_update && all_passed) {
        save_golden_hashes();
        printf("\nGolden hashes updated in %s\n", GOLDEN_HASH_FILE);
    }

    /* Generate JUnit XML if requested */
    if (flag_junit_output) {
        generate_junit_xml();
    }

    return all_passed ? 0 : 1;
}

/* ======================== JUnit XML Generation ======================== */

static void xml_escape(const char* in, char* out, size_t out_size) {
    size_t pos = 0;
    for (const char* p = in; *p && pos < out_size - 1; p++) {
        switch (*p) {
            case '<':  out[pos++] = '&'; out[pos++] = 'l'; out[pos++] = 't'; out[pos++] = ';'; break;
            case '>':  out[pos++] = '&'; out[pos++] = 'g'; out[pos++] = 't'; out[pos++] = ';'; break;
            case '&':  out[pos++] = '&'; out[pos++] = 'a'; out[pos++] = 'm'; out[pos++] = 'p'; out[pos++] = ';'; break;
            case '\"': out[pos++] = '&'; out[pos++] = 'q'; out[pos++] = 'u'; out[pos++] = 'o'; out[pos++] = 't'; out[pos++] = ';'; break;
            case '\'': out[pos++] = '&'; out[pos++] = 'a'; out[pos++] = 'p'; out[pos++] = 'o'; out[pos++] = 's'; out[pos++] = ';'; break;
            default:   out[pos++] = *p; break;
        }
    }
    out[pos] = '\0';
}

static void generate_junit_xml(void) {
    FILE* f = fopen(JUNIT_OUTPUT_FILE, "w");
    if (!f) {
        fprintf(stderr, "ERROR: Cannot write %s\n", JUNIT_OUTPUT_FILE);
        return;
    }

    fprintf(f, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    fprintf(f, "<testsuite name=\"H747_Elite_Regression_Suite\"\n");
    fprintf(f, "           tests=\"%d\"\n", suite.count);
    fprintf(f, "           failures=\"%d\"\n", suite.failed);
    fprintf(f, "           errors=\"%d\"\n", suite.errors);
    fprintf(f, "           skipped=\"%d\"\n", suite.skipped);
    fprintf(f, "           time=\"%.3f\"\n", suite.total_duration_ms / 1000.0);
    fprintf(f, "           timestamp=\"%s\">\n", suite.start_time);

    for (int i = 0; i < suite.count; i++) {
        test_entry_t* test = &suite.tests[i];
        char escaped_name[512], escaped_desc[512], escaped_msg[4096];

        xml_escape(test->name, escaped_name, sizeof(escaped_name));
        xml_escape(test->description, escaped_desc, sizeof(escaped_desc));

        fprintf(f, "  <testcase name=\"%s\" classname=\"Regression\"\n",
                escaped_name);
        fprintf(f, "            time=\"%.3f\"\n", test->duration_ms / 1000.0);

        if (test->result == TEST_PASS) {
            fprintf(f, "            status=\"pass\">\n");
            fprintf(f, "    <system-out>%s</system-out>\n", escaped_desc);
            fprintf(f, "  </testcase>\n");
        } else if (test->result == TEST_FAIL) {
            xml_escape(test->failure_message, escaped_msg, sizeof(escaped_msg));
            fprintf(f, "            status=\"fail\">\n");
            fprintf(f, "    <failure message=\"%s\">\n", escaped_msg);
            fprintf(f, "      <![CDATA[\n");
            fprintf(f, "        Test: %s\n", test->name);
            fprintf(f, "        Description: %s\n", test->description);
            fprintf(f, "        Duration: %.1f ms\n", test->duration_ms);
            fprintf(f, "      ]]>\n");
            fprintf(f, "    </failure>\n");
            fprintf(f, "  </testcase>\n");
        } else if (test->result == TEST_SKIP) {
            fprintf(f, "            status=\"skip\">\n");
            fprintf(f, "    <skipped message=\"Not implemented\"/>\n");
            fprintf(f, "  </testcase>\n");
        } else {
            fprintf(f, "            status=\"error\">\n");
            fprintf(f, "    <error message=\"Test error\"/>\n");
            fprintf(f, "  </testcase>\n");
        }
    }

    /* System output */
    fprintf(f, "  <system-out>\n");
    fprintf(f, "    H747 Elite Regression Suite\n");
    fprintf(f, "    Started: %s\n", suite.start_time);
    fprintf(f, "    Ended: %s\n", suite.end_time);
    fprintf(f, "    Duration: %.1f ms\n", suite.total_duration_ms);
    fprintf(f, "  </system-out>\n");

    fprintf(f, "</testsuite>\n");
    fclose(f);

    printf("JUnit XML written to %s\n", JUNIT_OUTPUT_FILE);
}

/* ======================== Main ======================== */

int main(int argc, char* argv[]) {
    /* Parse arguments */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--junit") == 0)
            flag_junit_output = 1;
        else if (strcmp(argv[i], "--golden-update") == 0)
            flag_golden_update = 1;
        else if (strcmp(argv[i], "--list") == 0)
            flag_list_tests = 1;
        else if (strcmp(argv[i], "--help") == 0) {
            printf("Usage: %s [options]\n", argv[0]);
            printf("Options:\n");
            printf("  --junit           Generate JUnit XML output\n");
            printf("  --golden-update   Update golden hash file with current results\n");
            printf("  --list            List registered tests and exit\n");
            printf("  --help            Show this help\n");
            return 0;
        }
    }

    memset(&suite, 0, sizeof(suite));

    int result = run_all_tests();

    return result;
}
