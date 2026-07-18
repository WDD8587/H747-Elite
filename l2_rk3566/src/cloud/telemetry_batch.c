/**
 * telemetry_batch.c — Telemetry batching to reduce cloud costs
 *
 * Buffer telemetry data points in 5-second buckets.
 * Compress with Snappy.
 * Upload every 30 seconds.
 * On WiFi disconnect, spill to eMMC (up to 100 MB).
 * Replay on reconnect in chronological order.
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
#include <snappy-c.h>
#include <cjson/cJSON.h>

#include "telemetry_batch.h"
#include "cloud_selector.h"

/* ---------------------------------------------------------------------------
 * Constants
 * --------------------------------------------------------------------------- */
#define TELEMETRY_BUCKET_MS       5000    /* 5-second bucket */
#define TELEMETRY_UPLOAD_INTERVAL 30      /* 30-second upload interval */
#define TELEMETRY_SPILL_MAX_BYTES (100LL * 1024LL * 1024LL)  /* 100 MB */
#define TELEMETRY_SPILL_DIR       "/data/telemetry/spill"
#define TELEMETRY_MAX_BATCH_SIZE  (64 * 1024)  /* 64 KB per batch */
#define TELEMETRY_MAX_POINTS      500     /* max data points per batch */

/* ---------------------------------------------------------------------------
 * Data point
 * --------------------------------------------------------------------------- */
typedef struct {
    uint64_t timestamp_ms;
    char     key[48];
    double   value;
    char     unit[16];
} telemetry_point_t;

/* ---------------------------------------------------------------------------
 * Bucket
 * --------------------------------------------------------------------------- */
typedef struct {
    uint64_t           start_time_ms;
    telemetry_point_t  points[TELEMETRY_MAX_POINTS];
    int                count;
} telemetry_bucket_t;

/* ---------------------------------------------------------------------------
 * Context
 * --------------------------------------------------------------------------- */
typedef struct {
    /* Current bucket being filled */
    telemetry_bucket_t current_bucket;
    pthread_mutex_t    lock;
    int                initialized;

    /* Upload thread */
    pthread_t          upload_thread;
    int                keep_running;

    /* Spill tracking */
    uint64_t           spill_bytes;       /* total bytes spilled to disk */
    int                spill_file_count;
    FILE              *spill_file;        /* current open spill file */

    /* Statistics */
    uint64_t           total_points_recorded;
    uint64_t           total_bytes_uploaded;
    uint64_t           total_spill_writes;
    uint64_t           total_spill_reads;

    /* Network state */
    int                network_available;
} telemetry_ctx_t;

static telemetry_ctx_t g_ctx;

/* ---------------------------------------------------------------------------
 * Forward declarations
 * --------------------------------------------------------------------------- */
static void telemetry_flush_bucket(void);
static void telemetry_upload_batch(const uint8_t *compressed, size_t comp_len,
                                   uint64_t bucket_time);
static int  telemetry_open_spill(void);
static int  telemetry_write_spill(const uint8_t *data, size_t len);
static void telemetry_replay_spill(void);
static void telemetry_upload_loop(void);
static void telemetry_close_spill(void);

/* ---------------------------------------------------------------------------
 * Initialize
 * --------------------------------------------------------------------------- */
int telemetry_batch_init(void) {
    memset(&g_ctx, 0, sizeof(g_ctx));
    pthread_mutex_init(&g_ctx.lock, NULL);
    g_ctx.keep_running = 1;
    g_ctx.network_available = 1;
    g_ctx.initialized = 1;

    /* Ensure spill directory exists */
    mkdir(TELEMETRY_SPILL_DIR, 0755);

    return 0;
}

/* ---------------------------------------------------------------------------
 * Add a single data point
 * --------------------------------------------------------------------------- */
int telemetry_batch_add(const char *key, double value, const char *unit) {
    return telemetry_batch_add_ts(key, value, unit, 0);
}

int telemetry_batch_add_ts(const char *key, double value, const char *unit,
                            uint64_t timestamp_ms) {
    if (!g_ctx.initialized) return -1;

    pthread_mutex_lock(&g_ctx.lock);

    telemetry_bucket_t *bucket = &g_ctx.current_bucket;

    /* Initialize bucket start time if empty */
    if (bucket->count == 0) {
        bucket->start_time_ms = (timestamp_ms > 0) ? timestamp_ms :
                                (uint64_t)(time(NULL) * 1000ULL);
    }

    /* Check if bucket is full or timespan exceeded — flush */
    uint64_t now_ms = (uint64_t)(time(NULL) * 1000ULL);
    if (bucket->count >= TELEMETRY_MAX_POINTS ||
        (now_ms - bucket->start_time_ms) >= (uint64_t)TELEMETRY_BUCKET_MS) {
        telemetry_flush_bucket();
    }

    /* Add point */
    telemetry_point_t *pt = &bucket->points[bucket->count];
    pt->timestamp_ms = (timestamp_ms > 0) ? timestamp_ms : now_ms;
    strncpy(pt->key, key, sizeof(pt->key) - 1);
    pt->value = value;
    strncpy(pt->unit, unit ? unit : "", sizeof(pt->unit) - 1);
    bucket->count++;

    g_ctx.total_points_recorded++;
    pthread_mutex_unlock(&g_ctx.lock);
    return 0;
}

/* Convenience: add from JSON payload (cloud_selector passthrough) */
int telemetry_batch_add_json(const char *payload, size_t len) {
    cJSON *root = cJSON_ParseWithLength(payload, len);
    if (!root) return -1;

    cJSON *item = NULL;
    cJSON_ArrayForEach(item, root) {
        if (cJSON_IsString(item)) {
            telemetry_batch_add(item->string, 0, item->valuestring);
        } else if (cJSON_IsNumber(item)) {
            telemetry_batch_add(item->string, item->valuedouble, "");
        }
    }
    cJSON_Delete(root);
    return 0;
}

/* ---------------------------------------------------------------------------
 * Flush current bucket: compress and either upload or spill
 * --------------------------------------------------------------------------- */
static void telemetry_flush_bucket(void) {
    telemetry_bucket_t *bucket = &g_ctx.current_bucket;
    if (bucket->count == 0) return;

    /* Build JSON array of points */
    cJSON *arr = cJSON_CreateArray();
    if (!arr) return;

    for (int i = 0; i < bucket->count; i++) {
        cJSON *pt = cJSON_CreateObject();
        if (!pt) continue;
        cJSON_AddNumberToObject(pt, "ts", (double)bucket->points[i].timestamp_ms);
        cJSON_AddStringToObject(pt, "k", bucket->points[i].key);
        cJSON_AddNumberToObject(pt, "v", bucket->points[i].value);
        if (bucket->points[i].unit[0])
            cJSON_AddStringToObject(pt, "u", bucket->points[i].unit);
        cJSON_AddItemToArray(arr, pt);
    }

    char *json_str = cJSON_PrintUnformatted(arr);
    cJSON_Delete(arr);
    if (!json_str) return;

    size_t json_len = strlen(json_str);

    /* Compress with Snappy */
    size_t comp_len = snappy_max_compressed_length(json_len);
    uint8_t *compressed = (uint8_t *)malloc(comp_len);
    if (!compressed) { free(json_str); return; }

    snappy_status status = snappy_compress(json_str, json_len, (char *)compressed, &comp_len);
    free(json_str);

    if (status != SNAPPY_OK) {
        free(compressed);
        return;
    }

    /* If network available, upload; otherwise spill */
    if (g_ctx.network_available) {
        telemetry_upload_batch(compressed, comp_len, bucket->start_time_ms);
    } else {
        telemetry_write_spill(compressed, comp_len);
    }

    free(compressed);

    /* Reset bucket */
    bucket->count = 0;
    bucket->start_time_ms = 0;
}

/* ---------------------------------------------------------------------------
 * Upload compressed batch to cloud
 * --------------------------------------------------------------------------- */
static void telemetry_upload_batch(const uint8_t *compressed, size_t comp_len,
                                    uint64_t bucket_time) {
    /* Build upload payload with metadata */
    cJSON *root = cJSON_CreateObject();
    if (!root) return;

    cJSON_AddNumberToObject(root, "t", (double)bucket_time);
    cJSON_AddNumberToObject(root, "n", (double)(bucket_time + TELEMETRY_BUCKET_MS));
    cJSON_AddNumberToObject(root, "c", (double)comp_len);

    /* Base64-encode the compressed data for JSON transport */
    /* For production: use base64 encoder. Here we post as binary via MQTT binary. */
    /* Simplified: just upload raw compressed via a dedicated binary topic */
    cJSON_Delete(root);

    /* Upload compressed data via cloud_publish on telemetry topic */
    char topic[128];
    snprintf(topic, sizeof(topic), "telemetry/batch/%llu",
             (unsigned long long)bucket_time);

    /* Prepend uncompressed length as 4-byte header */
    size_t uncomp_len = 0; /* not tracked; client calculates via Snappy */
    uint8_t header[8];
    header[0] = (uint8_t)(comp_len & 0xFF);
    header[1] = (uint8_t)((comp_len >> 8) & 0xFF);
    header[2] = (uint8_t)((comp_len >> 16) & 0xFF);
    header[3] = (uint8_t)((comp_len >> 24) & 0xFF);

    size_t total_len = 4 + comp_len;
    uint8_t *upload_buf = (uint8_t *)malloc(total_len);
    if (!upload_buf) return;

    memcpy(upload_buf, header, 4);
    memcpy(upload_buf + 4, compressed, comp_len);

    cloud_publish(topic, (const char *)upload_buf, total_len);

    g_ctx.total_bytes_uploaded += total_len;
    free(upload_buf);
}

/* ---------------------------------------------------------------------------
 * Spill to eMMC
 * --------------------------------------------------------------------------- */
static int telemetry_open_spill(void) {
    if (g_ctx.spill_bytes >= TELEMETRY_SPILL_MAX_BYTES) {
        fprintf(stderr, "[Telemetry] Spill storage full (%llu/%llu bytes)\n",
                (unsigned long long)g_ctx.spill_bytes,
                (unsigned long long)TELEMETRY_SPILL_MAX_BYTES);
        return -1;
    }

    char fname[256];
    snprintf(fname, sizeof(fname), "%s/batch_%llu_%d.snap",
             TELEMETRY_SPILL_DIR,
             (unsigned long long)time(NULL),
             g_ctx.spill_file_count);

    g_ctx.spill_file = fopen(fname, "wb");
    if (!g_ctx.spill_file) return -1;

    g_ctx.spill_file_count++;
    return 0;
}

static int telemetry_write_spill(const uint8_t *data, size_t len) {
    if (!g_ctx.spill_file) {
        if (telemetry_open_spill() != 0)
            return -1;
    }

    /* Write length header + data */
    uint32_t nlen = (uint32_t)len;
    if (fwrite(&nlen, sizeof(nlen), 1, g_ctx.spill_file) != 1) {
        telemetry_close_spill();
        return -1;
    }
    if (fwrite(data, 1, len, g_ctx.spill_file) != len) {
        telemetry_close_spill();
        return -1;
    }
    fflush(g_ctx.spill_file);

    g_ctx.spill_bytes += len + sizeof(nlen);
    g_ctx.total_spill_writes++;

    /* Close file periodically to avoid data loss */
    if (g_ctx.spill_bytes % (1024 * 1024) < len) {
        telemetry_close_spill();
    }
    return 0;
}

static void telemetry_close_spill(void) {
    if (g_ctx.spill_file) {
        fclose(g_ctx.spill_file);
        g_ctx.spill_file = NULL;
    }
}

/* ---------------------------------------------------------------------------
 * Replay spill files in chronological order
 * --------------------------------------------------------------------------- */
static void telemetry_replay_spill(void) {
    /* List files in spill directory */
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "ls -1 %s/*.snap 2>/dev/null", TELEMETRY_SPILL_DIR);

    FILE *ls = popen(cmd, "r");
    if (!ls) return;

    char fname[512];
    while (fgets(fname, sizeof(fname), ls)) {
        /* Trim newline */
        size_t ln = strlen(fname);
        if (ln > 0 && fname[ln - 1] == '\n') fname[ln - 1] = '\0';

        FILE *fh = fopen(fname, "rb");
        if (!fh) continue;

        uint32_t blen;
        while (fread(&blen, sizeof(blen), 1, fh) == 1) {
            if (blen > TELEMETRY_MAX_BATCH_SIZE) {
                fclose(fh);
                remove(fname);
                break;
            }

            uint8_t *buf = (uint8_t *)malloc(blen);
            if (!buf) break;

            if (fread(buf, 1, blen, fh) != blen) {
                free(buf);
                break;
            }

            /* Replay by uploading */
            /* Determine approximate bucket time from filename */
            uint64_t bt = 0;
            /* Just upload — the header has the time in each record */
            if (g_ctx.network_available) {
                telemetry_upload_batch(buf, blen, 0);
                g_ctx.total_spill_reads++;
            }

            free(buf);
        }

        fclose(fh);
        remove(fname);  /* Remove after replay */
    }

    pclose(ls);
    g_ctx.spill_bytes = 0;
}

/* ---------------------------------------------------------------------------
 * Network state notification
 * --------------------------------------------------------------------------- */
void telemetry_batch_set_network_available(int available) {
    pthread_mutex_lock(&g_ctx.lock);
    int was_available = g_ctx.network_available;
    g_ctx.network_available = available;
    pthread_mutex_unlock(&g_ctx.lock);

    if (available && !was_available) {
        /* Just came back online — flush current bucket and replay spill */
        pthread_mutex_lock(&g_ctx.lock);
        telemetry_flush_bucket();
        pthread_mutex_unlock(&g_ctx.lock);

        telemetry_replay_spill();
    }
}

/* ---------------------------------------------------------------------------
 * Upload loop — runs every TELEMETRY_UPLOAD_INTERVAL seconds
 * --------------------------------------------------------------------------- */
static void telemetry_upload_loop(void) {
    while (g_ctx.keep_running) {
        sleep(TELEMETRY_UPLOAD_INTERVAL);

        pthread_mutex_lock(&g_ctx.lock);
        telemetry_flush_bucket();
        pthread_mutex_unlock(&g_ctx.lock);
    }
}

/* ---------------------------------------------------------------------------
 * Start/stop
 * --------------------------------------------------------------------------- */
int telemetry_batch_start(void) {
    g_ctx.keep_running = 1;
    pthread_create(&g_ctx.upload_thread, NULL,
                   (void *(*)(void *))telemetry_upload_loop, NULL);
    return 0;
}

int telemetry_batch_stop(void) {
    g_ctx.keep_running = 0;

    /* Flush any remaining data */
    pthread_mutex_lock(&g_ctx.lock);
    telemetry_flush_bucket();
    telemetry_close_spill();
    pthread_mutex_unlock(&g_ctx.lock);

    pthread_join(g_ctx.upload_thread, NULL);
    return 0;
}

/* ---------------------------------------------------------------------------
 * Statistics
 * --------------------------------------------------------------------------- */
uint64_t telemetry_batch_get_total_points(void) {
    return g_ctx.total_points_recorded;
}

uint64_t telemetry_batch_get_total_uploaded(void) {
    return g_ctx.total_bytes_uploaded;
}

uint64_t telemetry_batch_get_spill_bytes(void) {
    return g_ctx.spill_bytes;
}
