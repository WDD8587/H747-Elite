/**
 * @file    trace_buffer.c
 * @brief   In-memory ring buffer for the last 10,000 log events.
 *
 * @details Each event is a compact 64-bit entry:
 *            - timestamp_us (32 bits, low bits of microsecond counter)
 *            - level        (4 bits: TRACE, DEBUG, INFO, WARN, ERROR, FATAL)
 *            - module       (8 bits: up to 256 modules)
 *            - message      (20 bits: message index into string table)
 *
 *          On crash, the buffer contents are dumped to a reserved flash
 *          page for post-mortem analysis. Live debugging is supported
 *          via a UART command that queries the buffer.
 *
 *          This implementation runs on the RK3566 side, receiving trace
 *          events from the STM32H747 via shared memory/UART.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include "trace_buffer.h"

/* ---------------------------------------------------------------------------
 * Configuration
 * -------------------------------------------------------------------------*/

#define TRACE_BUFFER_SIZE      10000    /* Number of events in ring buffer */
#define TRACE_DUMP_FLASH_ADDR  0x08200000UL  /* Flash page for crash dump */
#define TRACE_DUMP_FILE        "/tmp/trace_dump.bin"

/* ---------------------------------------------------------------------------
 * Trace event (64-bit packed)
 * -------------------------------------------------------------------------*/
typedef union {
    struct {
        uint32_t timestamp_us : 32;  /* Microsecond timestamp (low 32) */
        uint32_t level        :  4;  /* Event level                    */
        uint32_t module       :  8;  /* Module ID                      */
        uint32_t message      : 20;  /* Message index in string table  */
    } fields;
    uint64_t raw;
} trace_event_t;

_Static_assert(sizeof(trace_event_t) == 8,
               "trace_event_t must be 8 bytes");

/* ---------------------------------------------------------------------------
 * Module names (for human-readable output)
 * -------------------------------------------------------------------------*/
static const char *module_names[256] = {
    [TRACE_MOD_SYSTEM]  = "SYS",
    [TRACE_MOD_MOTOR]   = "MOT",
    [TRACE_MOD_IMU]     = "IMU",
    [TRACE_MOD_BMS]     = "BMS",
    [TRACE_MOD_SENSOR]  = "SEN",
    [TRACE_MOD_CAN]     = "CAN",
    [TRACE_MOD_SAFETY]  = "SAF",
    [TRACE_MOD_NET]     = "NET",
    [TRACE_MOD_DIAG]    = "DIA",
    [TRACE_MOD_BOOT]    = "BOOT",
};

static const char *level_names[] = {
    [TRACE_LEVEL_ERROR] = "ERROR",
    [TRACE_LEVEL_WARN]  = "WARN",
    [TRACE_LEVEL_INFO]  = "INFO",
    [TRACE_LEVEL_DEBUG] = "DEBUG",
    [TRACE_LEVEL_TRACE] = "TRACE",
    [TRACE_LEVEL_FATAL] = "FATAL",
};

/* ---------------------------------------------------------------------------
 * Ring buffer state
 * -------------------------------------------------------------------------*/
static trace_event_t buffer[TRACE_BUFFER_SIZE];
static uint32_t      write_index = 0;
static uint32_t      event_count = 0;
static int           buffer_initialized = 0;
static uint64_t      base_timestamp = 0;

/* Mutex for thread safety */
static pthread_mutex_t trace_mutex = PTHREAD_MUTEX_INITIALIZER;

/* ---------------------------------------------------------------------------
 * Internal helpers
 * -------------------------------------------------------------------------*/

/**
 * @brief  Get a monotonic microsecond timestamp (low 32 bits).
 */
static uint32_t trace_timestamp_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);

    uint64_t us = (uint64_t)ts.tv_sec * 1000000ULL +
                  (uint64_t)(ts.tv_nsec / 1000);

    if (base_timestamp == 0) {
        base_timestamp = us;
    }

    return (uint32_t)(us - base_timestamp);
}

/**
 * @brief  Ensure the dump directory exists.
 */
static int ensure_dump_dir(void)
{
    struct stat st;
    if (stat("/tmp", &st) == 0 && S_ISDIR(st.st_mode)) {
        return 0;
    }
    return mkdir("/tmp", 0755);
}

/* ---------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------*/

/**
 * @brief  Initialise the trace ring buffer.
 */
void trace_buffer_init(void)
{
    pthread_mutex_lock(&trace_mutex);

    memset(buffer, 0, sizeof(buffer));
    write_index = 0;
    event_count = 0;
    base_timestamp = 0;
    buffer_initialized = 1;

    pthread_mutex_unlock(&trace_mutex);
}

/**
 * @brief  Push a trace event into the ring buffer.
 *
 * @param  level        Event severity level.
 * @param  module       Module identifier (0-255).
 * @param  message      Message index (0-1048575, i.e. 20 bits).
 */
void trace_buffer_push(uint8_t level, uint8_t module, uint32_t message)
{
    if (!buffer_initialized) {
        return;
    }

    if (level > TRACE_LEVEL_FATAL) level = TRACE_LEVEL_FATAL;
    if (message > 0xFFFFF)         message = 0xFFFFF;

    uint32_t ts = trace_timestamp_us();

    trace_event_t ev;
    ev.fields.timestamp_us = ts;
    ev.fields.level        = level;
    ev.fields.module       = module;
    ev.fields.message      = message;

    pthread_mutex_lock(&trace_mutex);

    buffer[write_index] = ev;
    write_index = (write_index + 1) % TRACE_BUFFER_SIZE;
    if (event_count < TRACE_BUFFER_SIZE) {
        event_count++;
    }

    pthread_mutex_unlock(&trace_mutex);
}

/**
 * @brief  Dump the ring buffer to a file for post-mortem analysis.
 *
 * Typically called after a crash or on request.
 *
 * @param  path  Output file path (NULL = default /tmp/trace_dump.bin).
 * @return Number of events dumped, or -1 on error.
 */
int trace_buffer_dump(const char *path)
{
    if (path == NULL) {
        path = TRACE_DUMP_FILE;
    }

    ensure_dump_dir();

    FILE *fp = fopen(path, "wb");
    if (fp == NULL) {
        return -1;
    }

    uint32_t count = 0;

    pthread_mutex_lock(&trace_mutex);

    /* Write header: magic + count */
    uint32_t header[2] = { 0x54524345U, event_count };  /* "TRCE" */
    fwrite(header, sizeof(header), 1, fp);

    /* Write events in chronological order */
    if (event_count < TRACE_BUFFER_SIZE) {
        /* Buffer hasn't wrapped; events are at [0 .. event_count-1] */
        fwrite(buffer, sizeof(trace_event_t), event_count, fp);
        count = event_count;
    } else {
        /* Buffer has wrapped; start at write_index (which is the oldest) */
        fwrite(buffer + write_index, sizeof(trace_event_t),
               TRACE_BUFFER_SIZE - write_index, fp);
        fwrite(buffer, sizeof(trace_event_t),
               write_index, fp);
        count = TRACE_BUFFER_SIZE;
    }

    pthread_mutex_unlock(&trace_mutex);

    fclose(fp);

    /* Also write human-readable dump */
    char txt_path[320];
    snprintf(txt_path, sizeof(txt_path), "%s.txt", path);
    trace_buffer_print(txt_path, 0);

    return (int)count;
}

/**
 * @brief  Print ring buffer contents to a file or stdout.
 *
 * @param  path   Output path (NULL = stdout).
 * @param  filter Level filter: only show events >= this level (0 = all).
 */
void trace_buffer_print(const char *path, uint8_t filter)
{
    FILE *fp = stdout;
    int   close_fp = 0;

    if (path != NULL) {
        fp = fopen(path, "w");
        if (fp == NULL) {
            fp = stdout;
        } else {
            close_fp = 1;
        }
    }

    pthread_mutex_lock(&trace_mutex);

    fprintf(fp, "=== Trace Buffer Dump ===\n");
    fprintf(fp, "Total events: %u / %d\n\n",
            (unsigned int)event_count, TRACE_BUFFER_SIZE);

    /* Iterate in chronological order */
    uint32_t start = 0;
    uint32_t n = event_count;

    if (event_count > TRACE_BUFFER_SIZE) {
        start = write_index;
        n = TRACE_BUFFER_SIZE;
    }

    for (uint32_t i = 0; i < n; i++) {
        uint32_t idx = (start + i) % TRACE_BUFFER_SIZE;
        trace_event_t ev = buffer[idx];

        if (ev.fields.level < filter) continue;

        const char *lvl_str = (ev.fields.level <= TRACE_LEVEL_FATAL)
                                  ? level_names[ev.fields.level]
                                  : "???";
        const char *mod_str = module_names[ev.fields.module];
        if (mod_str == NULL) mod_str = "???";

        fprintf(fp, "[%08u] %-5s %3s msg=%05u\n",
                (unsigned int)ev.fields.timestamp_us,
                lvl_str, mod_str,
                (unsigned int)ev.fields.message);
    }

    fprintf(fp, "=== End of Dump ===\n");

    pthread_mutex_unlock(&trace_mutex);

    if (close_fp) {
        fclose(fp);
    }
}

/**
 * @brief  Query the buffer for events matching a module/level filter.
 *
 * @param  out     Output buffer for matching events.
 * @param  max     Maximum number of events to return.
 * @param  module  Module filter (0xFF = any).
 * @param  level   Min level filter (0 = any).
 * @return Number of matching events written to out.
 */
int trace_buffer_query(trace_entry_t *out, int max,
                        uint8_t module, uint8_t level)
{
    if (out == NULL || max <= 0) return 0;

    int written = 0;

    pthread_mutex_lock(&trace_mutex);

    uint32_t start = 0;
    uint32_t n = event_count;

    if (event_count > TRACE_BUFFER_SIZE) {
        start = write_index;
        n = TRACE_BUFFER_SIZE;
    }

    for (uint32_t i = 0; i < n && written < max; i++) {
        uint32_t idx = (start + i) % TRACE_BUFFER_SIZE;
        trace_event_t ev = buffer[idx];

        if (module != 0xFF && ev.fields.module != module) continue;
        if (ev.fields.level < level) continue;

        out[written].timestamp_us = ev.fields.timestamp_us;
        out[written].level        = ev.fields.level;
        out[written].module       = ev.fields.module;
        out[written].message      = ev.fields.message;
        written++;
    }

    pthread_mutex_unlock(&trace_mutex);

    return written;
}

/**
 * @brief  Get the number of events currently in the buffer.
 *
 * @return Event count.
 */
uint32_t trace_buffer_count(void)
{
    return event_count;
}

/**
 * @brief  Clear all events from the buffer.
 */
void trace_buffer_clear(void)
{
    pthread_mutex_lock(&trace_mutex);

    memset(buffer, 0, sizeof(buffer));
    write_index = 0;
    event_count = 0;

    pthread_mutex_unlock(&trace_mutex);
}
