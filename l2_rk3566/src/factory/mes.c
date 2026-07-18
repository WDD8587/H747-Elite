/**
 * mes.c
 * Manufacturing Execution System (MES) interface.
 *
 * HTTP POST test results to factory server.
 * JSON format: {serial, timestamp, test_results: [{name, result, values}], overall_pass}
 * Retry 3x on network failure.
 * Store results to eMMC and upload on next connection.
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>

/* ------------------------------------------------------------------ */
/*  Configuration                                                     */
/* ------------------------------------------------------------------ */

#define MES_SERVER_URL      "http://factory.example.com/api/v1/test_results"
#define MES_MAX_RETRIES     3
#define MES_HTTP_TIMEOUT_S  10
#define MES_MAX_RESULTS     64
#define MES_RESULT_NAME_LEN 48
#define MES_STORAGE_FILE    "/emmc/factory/results.dat"

/* ------------------------------------------------------------------ */
/*  Types                                                             */
/* ------------------------------------------------------------------ */

typedef enum {
    MES_RESULT_PASS = 0,
    MES_RESULT_FAIL = 1
} mes_test_result_t;

typedef struct {
    char              name[MES_RESULT_NAME_LEN];
    mes_test_result_t result;
    float             values[8];       /* measured values */
    int               num_values;
    uint32_t          elapsed_ms;
} mes_test_entry_t;

typedef struct {
    char              serial[64];
    char              timestamp[32];
    mes_test_entry_t  tests[MES_MAX_RESULTS];
    int               num_tests;
    int               overall_pass;
    uint32_t          attempt_count;
    uint32_t          upload_timestamp;
} mes_report_t;

/* HTTP response */
typedef struct {
    int   status_code;       /* 200 = OK */
    char  body[256];
} http_response_t;

/* ------------------------------------------------------------------ */
/*  DUT state                                                         */
/* ------------------------------------------------------------------ */

static mes_report_t g_pending_report;
static int g_has_pending = 0;
static int g_network_available = 0;
static int g_stored_results_count = 0;

/* ------------------------------------------------------------------ */
/*  HTTP client (platform stub - connects to real transport)         */
/* ------------------------------------------------------------------ */

/* Platform HTTP POST function (implemented per platform).
 * Returns 0 on success, -1 on error. */
static int platform_http_post(const char *url, const char *json_body,
                              size_t body_len, http_response_t *resp,
                              uint32_t timeout_s)
{
    (void)url;
    (void)json_body;
    (void)body_len;
    (void)resp;
    (void)timeout_s;

    /* In production, this calls curl or SDK HTTP client.
     * Returns 0 if HTTP 200 received, -1 otherwise. */

    if (!g_network_available)
        return -1;

    resp->status_code = 200;
    snprintf(resp->body, sizeof(resp->body), "{\"status\":\"ok\"}");
    return 0;
}

/* eMMC storage (platform stub) */
static int platform_storage_write(const char *path, const uint8_t *data,
                                  size_t len)
{
    (void)path;
    (void)data;
    (void)len;
    /* Simulate write */
    return 0;
}

static int platform_storage_read(const char *path, uint8_t *buf,
                                 size_t max_len, size_t *out_len)
{
    (void)path;
    (void)buf;
    (void)max_len;
    (void)out_len;
    /* Simulate read */
    return 0;
}

/* ------------------------------------------------------------------ */
/*  JSON serialization                                               */
/* ------------------------------------------------------------------ */

static void json_escape(char *out, size_t out_size, const char *in)
{
    size_t o = 0;
    while (*in && o < out_size - 2) {
        if (*in == '"' || *in == '\\') {
            if (o < out_size - 3) out[o++] = '\\';
        }
        out[o++] = *in++;
    }
    out[o] = '\0';
}

static int mes_report_to_json(const mes_report_t *report,
                              char *buf, size_t buf_size)
{
    int n = 0;
    char esc_serial[128];

    json_escape(esc_serial, sizeof(esc_serial), report->serial);

    n += snprintf(buf + n, buf_size - (size_t)n > 0 ? buf_size - (size_t)n : 0,
                  "{\n"
                  "  \"serial\": \"%s\",\n"
                  "  \"timestamp\": \"%s\",\n"
                  "  \"overall_pass\": %s,\n"
                  "  \"test_results\": [\n",
                  esc_serial,
                  report->timestamp,
                  report->overall_pass ? "true" : "false");

    for (int i = 0; i < report->num_tests; i++) {
        char esc_name[64];
        json_escape(esc_name, sizeof(esc_name), report->tests[i].name);

        n += snprintf(buf + n, buf_size - (size_t)n > 0 ? buf_size - (size_t)n : 0,
                      "    {\n"
                      "      \"name\": \"%s\",\n"
                      "      \"result\": \"%s\",\n"
                      "      \"elapsed_ms\": %u,\n"
                      "      \"values\": [",
                      esc_name,
                      report->tests[i].result == MES_RESULT_PASS ? "PASS" : "FAIL",
                      (unsigned)report->tests[i].elapsed_ms);

        for (int v = 0; v < report->tests[i].num_values; v++) {
            n += snprintf(buf + n, buf_size - (size_t)n > 0 ? buf_size - (size_t)n : 0,
                          "%s%.3f",
                          v > 0 ? ", " : "",
                          (double)report->tests[i].values[v]);
        }

        n += snprintf(buf + n, buf_size - (size_t)n > 0 ? buf_size - (size_t)n : 0,
                      "]\n"
                      "    }%s\n",
                      (i < report->num_tests - 1) ? "," : "");
    }

    n += snprintf(buf + n, buf_size - (size_t)n > 0 ? buf_size - (size_t)n : 0,
                  "  ]\n"
                  "}\n");

    return n;
}

/* ------------------------------------------------------------------ */
/*  MES API                                                           */
/* ------------------------------------------------------------------ */

void mes_init(void)
{
    memset(&g_pending_report, 0, sizeof(g_pending_report));
    g_has_pending = 0;
    g_network_available = 0;
    g_stored_results_count = 0;
}

void mes_set_network_available(int available)
{
    g_network_available = available;
}

void mes_start_report(const char *serial)
{
    memset(&g_pending_report, 0, sizeof(g_pending_report));
    strncpy(g_pending_report.serial, serial, sizeof(g_pending_report.serial) - 1);

    /* Generate timestamp */
    time_t now = time(NULL);
    struct tm *tm = localtime(&now);
    if (tm) {
        strftime(g_pending_report.timestamp, sizeof(g_pending_report.timestamp),
                 "%Y-%m-%dT%H:%M:%S%z", tm);
    } else {
        snprintf(g_pending_report.timestamp, sizeof(g_pending_report.timestamp),
                 "1970-01-01T00:00:00+0000");
    }

    g_pending_report.overall_pass = 1;
    g_pending_report.num_tests    = 0;
    g_pending_report.attempt_count = 0;
    g_has_pending = 1;
}

void mes_add_test_result(const char *name, mes_test_result_t result,
                         const float *values, int num_values,
                         uint32_t elapsed_ms)
{
    if (!g_has_pending) return;
    if (g_pending_report.num_tests >= MES_MAX_RESULTS) return;

    mes_test_entry_t *entry = &g_pending_report.tests[g_pending_report.num_tests];
    strncpy(entry->name, name, sizeof(entry->name) - 1);
    entry->result     = result;
    entry->num_values = (num_values > 8) ? 8 : num_values;
    entry->elapsed_ms = elapsed_ms;
    for (int i = 0; i < entry->num_values; i++)
        entry->values[i] = values[i];

    g_pending_report.num_tests++;

    if (result == MES_RESULT_FAIL)
        g_pending_report.overall_pass = 0;
}

/* Upload report to MES server with retry.
 * Returns 1 if uploaded successfully, 0 if failed. */
int mes_upload_report(void)
{
    if (!g_has_pending) return 0;

    char json_body[4096];
    int body_len = mes_report_to_json(&g_pending_report, json_body, sizeof(json_body));
    if (body_len <= 0) return 0;

    http_response_t resp;

    for (uint32_t retry = 0; retry < MES_MAX_RETRIES; retry++) {
        g_pending_report.attempt_count++;

        int rc = platform_http_post(MES_SERVER_URL, json_body, (size_t)body_len,
                                    &resp, MES_HTTP_TIMEOUT_S);
        if (rc == 0 && resp.status_code == 200) {
            g_pending_report.upload_timestamp = (uint32_t)time(NULL);
            g_has_pending = 0;
            return 1;
        }

        /* Wait before retry (in production, use actual delay) */
        if (retry < MES_MAX_RETRIES - 1) {
            volatile int delay = 0;
            for (int i = 0; i < 1000000; i++) delay++;
            (void)delay;
        }
    }

    return 0;
}

/* Store results to eMMC for later upload */
int mes_store_results(void)
{
    if (!g_has_pending) return 0;

    uint8_t buf[4096];
    int len = mes_report_to_json(&g_pending_report, (char *)buf, sizeof(buf));
    if (len <= 0) return 0;

    int rc = platform_storage_write(MES_STORAGE_FILE, buf, (size_t)len);
    if (rc == 0) {
        g_stored_results_count++;
        return 1;
    }
    return 0;
}

/* Upload all stored results from eMMC */
int mes_upload_stored(void)
{
    uint8_t buf[4096];
    size_t out_len = 0;
    int rc = platform_storage_read(MES_STORAGE_FILE, buf, sizeof(buf), &out_len);
    if (rc != 0 || out_len == 0) return 0;

    http_response_t resp;
    for (uint32_t retry = 0; retry < MES_MAX_RETRIES; retry++) {
        rc = platform_http_post(MES_SERVER_URL, (const char *)buf, out_len,
                                &resp, MES_HTTP_TIMEOUT_S);
        if (rc == 0 && resp.status_code == 200) {
            g_stored_results_count = 0;
            return 1;
        }
    }
    return 0;
}

/* Try to upload everything: pending + stored */
int mes_upload_all(void)
{
    int success = 1;

    if (g_has_pending) {
        if (mes_upload_report())
            g_has_pending = 0;
        else
            success = 0;
    }

    if (g_stored_results_count > 0) {
        if (mes_upload_stored())
            g_stored_results_count = 0;
        else
            success = 0;
    }

    return success;
}

/* Combined: store then attempt upload.
 * If upload fails, data persists on eMMC. */
int mes_submit_results(void)
{
    /* Always store locally first */
    mes_store_results();

    /* Try upload if network available */
    if (g_network_available) {
        return mes_upload_all();
    }

    return 0;  /* stored locally, will retry later */
}

/* ------------------------------------------------------------------ */
/*  Test / example                                                    */
/* ------------------------------------------------------------------ */

#ifdef UNIT_TEST
#include <assert.h>

static int test_passed = 0;
static int test_failed = 0;

#define TEST_START(name)  do { printf("  TEST %-50s ", name); } while(0)
#define TEST_PASS()       do { printf("PASS\n"); test_passed++; } while(0)
#define TEST_FAIL(msg)    do { printf("FAIL: %s\n", msg); test_failed++; } while(0)

int main(void)
{
    printf("=== MES Interface Tests ===\n\n");

    /* Test JSON generation */
    TEST_START("JSON report generation");
    mes_init();
    mes_start_report("H747-ELITE-00001");

    float motor_vals[] = {0.523f, 30.2f};
    mes_add_test_result("Motor Cal", MES_RESULT_PASS, motor_vals, 2, 1520);

    float imu_vals[] = {0.01f, 0.02f, 9.81f};
    mes_add_test_result("IMU Cal", MES_RESULT_PASS, imu_vals, 3, 890);

    float bat_vals[] = {16.8f, 3.2f};
    mes_add_test_result("Battery", MES_RESULT_FAIL, bat_vals, 2, 340);

    char json[4096];
    int len = mes_report_to_json(&g_pending_report, json, sizeof(json));
    if (len > 0 && strstr(json, "H747-ELITE-00001") &&
        strstr(json, "Motor Cal") &&
        strstr(json, "Battery"))
        TEST_PASS();
    else
        TEST_FAIL("JSON missing expected content");

    /* Test overall pass computation */
    TEST_START("Overall pass computation");
    if (!g_pending_report.overall_pass)
        TEST_PASS();
    else
        TEST_FAIL("expected overall_pass=false");

    /* Test network upload with retry */
    TEST_START("Upload with retry (network available)");
    mes_init();
    g_network_available = 1;
    mes_start_report("TEST-001");
    mes_add_test_result("Test1", MES_RESULT_PASS, NULL, 0, 100);
    int rc = mes_upload_report();
    if (rc == 1)
        TEST_PASS();
    else
        TEST_FAIL("upload failed");

    /* Test network failure -> store */
    TEST_START("Store when network unavailable");
    mes_init();
    g_network_available = 0;
    mes_start_report("TEST-002");
    mes_add_test_result("Test1", MES_RESULT_PASS, NULL, 0, 100);
    rc = mes_submit_results();
    if (rc == 0 && g_stored_results_count > 0)
        TEST_PASS();
    else
        TEST_FAIL("expected local storage fallback");

    printf("\nResults: %d passed, %d failed\n", test_passed, test_failed);
    return test_failed > 0 ? 1 : 0;
}
#endif
