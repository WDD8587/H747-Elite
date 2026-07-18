/**
 * analytics.c — Usage analytics (opt-in, GDPR compliant)
 *
 * Tracks: total cleaning time, total area cleaned, average session duration,
 * most used mode, error frequency per component.
 * Weekly upload. Anonymized device ID (hash of serial number).
 * Data stored locally until upload.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <pthread.h>
#include <time.h>
#include <errno.h>
#include <sys/stat.h>
#include <openssl/evp.h>
#include <cjson/cJSON.h>

#include "analytics.h"
#include "cloud_selector.h"
#include "../control/robot_ctrl.h"

/* ---------------------------------------------------------------------------
 * Constants
 * --------------------------------------------------------------------------- */
#define ANALYTICS_STORE_PATH    "/data/analytics/usage.bin"
#define ANALYTICS_UPLOAD_PERIOD (7 * 24 * 3600)  /* weekly (seconds) */
#define ANALYTICS_HASH_HEX_LEN  64
#define ANALYTICS_MAX_SESSIONS  1000
#define ANALYTICS_ERROR_CATEGORIES 16

/* ---------------------------------------------------------------------------
 * Error counter
 * --------------------------------------------------------------------------- */
typedef struct {
    char     component[32];
    uint32_t count;
} error_counter_t;

/* ---------------------------------------------------------------------------
 * Session record
 * --------------------------------------------------------------------------- */
typedef struct {
    uint64_t start_time_unix;
    uint32_t duration_sec;
    uint32_t area_cleaned_cm2;    /* total area during session */
    uint8_t  mode;                 /* clean_mode_t */
    uint8_t  completed;            /* 1=completed, 0=interrupted */
    uint32_t errors_during_session;
} session_record_t;

/* ---------------------------------------------------------------------------
 * Analytics data (persisted)
 * --------------------------------------------------------------------------- */
typedef struct {
    uint64_t     magic;            /* validation constant */
    uint64_t     version;

    /* Accumulated totals */
    uint64_t     total_cleaning_time_sec;
    uint64_t     total_area_cm2;
    uint32_t     total_sessions;

    /* Mode usage */
    uint32_t     mode_use_count[5]; /* indexed by clean_mode_t */

    /* Error frequency */
    error_counter_t error_counts[ANALYTICS_ERROR_CATEGORIES];
    int             error_count;

    /* Recent sessions (circular buffer) */
    session_record_t sessions[ANALYTICS_MAX_SESSIONS];
    int              session_head;
    int              session_tail;

    /* Anonymized device ID */
    char         device_id[ANALYTICS_HASH_HEX_LEN + 1];

    /* Upload tracking */
    uint64_t     last_upload_unix;

    uint64_t     checksum;
} analytics_data_t;

#define ANALYTICS_MAGIC 0x414E414C59544943ULL  /* "ANALYTIC" */

/* ---------------------------------------------------------------------------
 * Context
 * --------------------------------------------------------------------------- */
typedef struct {
    analytics_data_t data;
    pthread_mutex_t  lock;
    int              opt_in;       /* user consent */
    int              dirty;        /* needs persist */
    int              keep_running;

    pthread_t        upload_thread;
    pthread_t        persist_thread;

    /* Current session tracking */
    uint64_t         session_start;
    uint32_t         session_area;
    int              session_active;
    int              current_mode;
} analytics_ctx_t;

static analytics_ctx_t g_ctx;

/* ---------------------------------------------------------------------------
 * Forward declarations
 * --------------------------------------------------------------------------- */
static void analytics_hash_device_id(const char *serial, char *hex_out, size_t hex_cap);
static int  analytics_persist(void);
static int  analytics_restore(void);
static void analytics_upload_loop(void);
static void analytics_persist_loop(void);
static uint64_t analytics_checksum(const analytics_data_t *d);

/* ---------------------------------------------------------------------------
 * Device ID hashing (SHA-256 of serial number)
 * --------------------------------------------------------------------------- */
static void analytics_hash_device_id(const char *serial, char *hex_out, size_t hex_cap) {
    unsigned char hash[EVP_MAX_MD_SIZE];
    unsigned int hash_len = 0;

    EVP_MD_CTX *ctx = EVP_MD_CTX_new();
    if (!ctx) { strncpy(hex_out, "unknown", hex_cap - 1); return; }

    EVP_DigestInit_ex(ctx, EVP_sha256(), NULL);
    EVP_DigestUpdate(ctx, serial, strlen(serial));
    /* Add a fixed pepper */
    EVP_DigestUpdate(ctx, "r0b0t_an4lyt1cs_p3pp3r", 22);
    EVP_DigestFinal_ex(ctx, hash, &hash_len);
    EVP_MD_CTX_free(ctx);

    for (unsigned int i = 0; i < hash_len && i * 2 + 1 < hex_cap; i++) {
        sprintf(hex_out + i * 2, "%02x", hash[i]);
    }
    hex_out[hex_cap - 1] = '\0';
}

/* ---------------------------------------------------------------------------
 * Simple checksum (XOR-based, sufficient for integrity check)
 * --------------------------------------------------------------------------- */
static uint64_t analytics_checksum(const analytics_data_t *d) {
    const uint8_t *bytes = (const uint8_t *)d;
    uint64_t cs = 0;
    for (size_t i = 0; i < sizeof(analytics_data_t) - sizeof(uint64_t); i++) {
        cs ^= ((uint64_t)bytes[i]) << ((i % 8) * 8);
    }
    return cs;
}

/* ---------------------------------------------------------------------------
 * Persist to disk
 * --------------------------------------------------------------------------- */
static int analytics_persist(void) {
    analytics_data_t *d = &g_ctx.data;
    d->checksum = analytics_checksum(d);

    /* Ensure directory exists */
    mkdir("/data/analytics", 0755);

    FILE *fh = fopen(ANALYTICS_STORE_PATH, "wb");
    if (!fh) {
        fprintf(stderr, "[Analytics] Cannot write %s\n", ANALYTICS_STORE_PATH);
        return -1;
    }

    size_t written = fwrite(d, sizeof(*d), 1, fh);
    fclose(fh);

    if (written != 1) return -1;
    return 0;
}

/* ---------------------------------------------------------------------------
 * Restore from disk
 * --------------------------------------------------------------------------- */
static int analytics_restore(void) {
    FILE *fh = fopen(ANALYTICS_STORE_PATH, "rb");
    if (!fh) return -1;

    analytics_data_t d;
    if (fread(&d, sizeof(d), 1, fh) != 1) {
        fclose(fh);
        return -1;
    }
    fclose(fh);

    if (d.magic != ANALYTICS_MAGIC) {
        fprintf(stderr, "[Analytics] Invalid data file\n");
        return -1;
    }

    if (d.checksum != analytics_checksum(&d)) {
        fprintf(stderr, "[Analytics] Data corruption detected\n");
        return -1;
    }

    g_ctx.data = d;
    return 0;
}

/* ---------------------------------------------------------------------------
 * Upload to cloud
 * --------------------------------------------------------------------------- */
static int analytics_upload(void) {
    analytics_ctx_t *ctx = &g_ctx;
    pthread_mutex_lock(&ctx->lock);

    cJSON *root = cJSON_CreateObject();
    if (!root) { pthread_mutex_unlock(&ctx->lock); return -1; }

    cJSON_AddStringToObject(root, "device_id", ctx->data.device_id);
    cJSON_AddNumberToObject(root, "total_time_sec", (double)ctx->data.total_cleaning_time_sec);
    cJSON_AddNumberToObject(root, "total_area_cm2", (double)ctx->data.total_area_cm2);
    cJSON_AddNumberToObject(root, "total_sessions", (double)ctx->data.total_sessions);

    /* Average session duration */
    if (ctx->data.total_sessions > 0) {
        cJSON_AddNumberToObject(root, "avg_session_sec",
            (double)(ctx->data.total_cleaning_time_sec / ctx->data.total_sessions));
    }

    /* Most used mode */
    int best_mode = 0;
    uint32_t best_count = 0;
    const char *mode_names[] = { "unknown", "auto", "edge", "spot", "room" };
    for (int i = 1; i < 5; i++) {
        if (ctx->data.mode_use_count[i] > best_count) {
            best_count = ctx->data.mode_use_count[i];
            best_mode = i;
        }
    }
    cJSON_AddStringToObject(root, "most_used_mode",
                            mode_names[best_mode < 5 ? best_mode : 0]);

    /* Error frequency */
    cJSON *errors = cJSON_AddArrayToObject(root, "error_frequency");
    if (errors) {
        for (int i = 0; i < ctx->data.error_count; i++) {
            cJSON *e = cJSON_CreateObject();
            if (!e) continue;
            cJSON_AddStringToObject(e, "component", ctx->data.error_counts[i].component);
            cJSON_AddNumberToObject(e, "count", (double)ctx->data.error_counts[i].count);
            cJSON_AddItemToArray(errors, e);
        }
    }

    /* Timestamp and version */
    cJSON_AddNumberToObject(root, "timestamp", (double)time(NULL));
    cJSON_AddNumberToObject(root, "version", 2);

    /* GDPR/privacy notice */
    cJSON_AddStringToObject(root, "privacy_notice",
        "Data anonymized and aggregated. No personal or location data transmitted.");

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (!json) { pthread_mutex_unlock(&ctx->lock); return -1; }

    pthread_mutex_unlock(&ctx->lock);

    /* Publish */
    int ret = cloud_publish("analytics/usage", json, strlen(json));
    free(json);

    if (ret == 0) {
        pthread_mutex_lock(&ctx->lock);
        ctx->data.last_upload_unix = (uint64_t)time(NULL);
        ctx->dirty = 1;
        pthread_mutex_unlock(&ctx->lock);
    }

    return ret;
}

/* ---------------------------------------------------------------------------
 * Upload loop (weekly)
 * --------------------------------------------------------------------------- */
static void analytics_upload_loop(void) {
    while (g_ctx.keep_running) {
        sleep(3600); /* check every hour */

        if (!g_ctx.opt_in) continue;

        time_t now = time(NULL);
        uint64_t next_upload = g_ctx.data.last_upload_unix + ANALYTICS_UPLOAD_PERIOD;

        if ((uint64_t)now >= next_upload) {
            fprintf(stdout, "[Analytics] Weekly upload triggered\n");
            analytics_upload();
        }
    }
}

/* ---------------------------------------------------------------------------
 * Persist loop (every 60 seconds)
 * --------------------------------------------------------------------------- */
static void analytics_persist_loop(void) {
    while (g_ctx.keep_running) {
        sleep(60);
        pthread_mutex_lock(&g_ctx.lock);
        if (g_ctx.dirty) {
            analytics_persist();
            g_ctx.dirty = 0;
        }
        pthread_mutex_unlock(&g_ctx.lock);
    }
}

/* ---------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------- */
int analytics_init(const char *serial_number) {
    memset(&g_ctx, 0, sizeof(g_ctx));
    pthread_mutex_init(&g_ctx.lock, NULL);
    g_ctx.opt_in = 0; /* opt-in by default off */
    g_ctx.keep_running = 1;

    /* Restore previous data */
    if (analytics_restore() != 0) {
        /* Initialize fresh */
        analytics_data_t *d = &g_ctx.data;
        memset(d, 0, sizeof(*d));
        d->magic = ANALYTICS_MAGIC;
        d->version = 2;

        if (serial_number)
            analytics_hash_device_id(serial_number, d->device_id, sizeof(d->device_id));
        else
            strncpy(d->device_id, "unknown", sizeof(d->device_id) - 1);
    }

    return 0;
}

int analytics_start(void) {
    g_ctx.keep_running = 1;

    pthread_create(&g_ctx.upload_thread, NULL,
                   (void *(*)(void *))analytics_upload_loop, NULL);
    pthread_create(&g_ctx.persist_thread, NULL,
                   (void *(*)(void *))analytics_persist_loop, NULL);

    fprintf(stdout, "[Analytics] Started (opt-in: %s)\n",
            g_ctx.opt_in ? "yes" : "no");
    return 0;
}

int analytics_stop(void) {
    g_ctx.keep_running = 0;

    /* Flush */
    if (g_ctx.session_active) {
        analytics_end_session(0);
    }

    pthread_mutex_lock(&g_ctx.lock);
    analytics_persist();
    pthread_mutex_unlock(&g_ctx.lock);

    pthread_join(g_ctx.upload_thread, NULL);
    pthread_join(g_ctx.persist_thread, NULL);
    return 0;
}

void analytics_set_opt_in(int enabled) {
    g_ctx.opt_in = enabled;
    fprintf(stdout, "[Analytics] User opt-in: %s\n", enabled ? "yes" : "no");
    if (enabled)
        g_ctx.dirty = 1;
}

int analytics_is_opted_in(void) {
    return g_ctx.opt_in;
}

/* ---------------------------------------------------------------------------
 * Session tracking
 * --------------------------------------------------------------------------- */
void analytics_start_session(int mode) {
    if (!g_ctx.opt_in) return;

    pthread_mutex_lock(&g_ctx.lock);
    g_ctx.session_start = (uint64_t)time(NULL);
    g_ctx.session_area = 0;
    g_ctx.session_active = 1;
    g_ctx.current_mode = mode;
    pthread_mutex_unlock(&g_ctx.lock);
}

void analytics_end_session(int completed) {
    if (!g_ctx.opt_in || !g_ctx.session_active) return;

    pthread_mutex_lock(&g_ctx.lock);

    uint64_t now = (uint64_t)time(NULL);
    uint32_t duration = (uint32_t)(now - g_ctx.session_start);

    /* Update totals */
    g_ctx.data.total_cleaning_time_sec += duration;
    g_ctx.data.total_area_cm2 += g_ctx.session_area;
    g_ctx.data.total_sessions++;

    /* Mode count */
    int mode_idx = g_ctx.current_mode;
    if (mode_idx > 0 && mode_idx < 5)
        g_ctx.data.mode_use_count[mode_idx]++;

    /* Store session */
    session_record_t *sr = &g_ctx.data.sessions[g_ctx.data.session_head];
    sr->start_time_unix = g_ctx.session_start;
    sr->duration_sec = duration;
    sr->area_cleaned_cm2 = g_ctx.session_area;
    sr->mode = (uint8_t)g_ctx.current_mode;
    sr->completed = completed ? 1 : 0;
    sr->errors_during_session = 0;
    g_ctx.data.session_head = (g_ctx.data.session_head + 1) % ANALYTICS_MAX_SESSIONS;
    if (g_ctx.data.session_head == g_ctx.data.session_tail)
        g_ctx.data.session_tail = (g_ctx.data.session_tail + 1) % ANALYTICS_MAX_SESSIONS;

    g_ctx.session_active = 0;
    g_ctx.dirty = 1;

    pthread_mutex_unlock(&g_ctx.lock);
}

void analytics_add_area(uint32_t area_cm2) {
    if (!g_ctx.opt_in || !g_ctx.session_active) return;

    pthread_mutex_lock(&g_ctx.lock);
    g_ctx.session_area += area_cm2;
    g_ctx.dirty = 1;
    pthread_mutex_unlock(&g_ctx.lock);
}

/* ---------------------------------------------------------------------------
 * Error tracking
 * --------------------------------------------------------------------------- */
void analytics_record_error(const char *component) {
    if (!g_ctx.opt_in) return;

    pthread_mutex_lock(&g_ctx.lock);

    /* Find or create error counter */
    int found = -1;
    for (int i = 0; i < g_ctx.data.error_count; i++) {
        if (strcmp(g_ctx.data.error_counts[i].component, component) == 0) {
            found = i;
            break;
        }
    }

    if (found >= 0) {
        g_ctx.data.error_counts[found].count++;
    } else if (g_ctx.data.error_count < ANALYTICS_ERROR_CATEGORIES) {
        strncpy(g_ctx.data.error_counts[g_ctx.data.error_count].component,
                component, sizeof(g_ctx.data.error_counts[0].component) - 1);
        g_ctx.data.error_counts[g_ctx.data.error_count].count = 1;
        g_ctx.data.error_count++;
    }

    g_ctx.dirty = 1;
    pthread_mutex_unlock(&g_ctx.lock);
}

/* ---------------------------------------------------------------------------
 * Force upload
 * --------------------------------------------------------------------------- */
int analytics_force_upload(void) {
    if (!g_ctx.opt_in) return -1;
    return analytics_upload();
}
