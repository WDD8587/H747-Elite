/**
 * @file    perf_profiler.c
 * @brief   Runtime performance profiler using FreeRTOS trace hooks.
 *
 * @details Hooks into traceTASK_SWITCHED_IN and traceTASK_SWITCHED_OUT
 *          (or the equivalent trace macros) to measure per-task:
 *            - CPU utilisation (%)
 *            - Stack high-water mark (bytes)
 *            - Maximum single execution time (us)
 *
 *          Outputs CSV every 60 seconds to /var/log/robot/perf.csv.
 *
 *          The profiling data is accumulated in a task-local or global
 *          table and periodically flushed to disk from the RK3566 side
 *          (the measurement capture happens on the STM32H747 via
 *          FreeRTOS trace macros; the data is forwarded via shared
 *          memory or UART to RK3566 for persistent logging).
 *
 *          This file implements the RK3566-side consumer that receives
 *          profiling telemetry and writes CSV output.
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
#include "perf_profiler.h"

/* ---------------------------------------------------------------------------
 * Configuration
 * -------------------------------------------------------------------------*/

#define PERF_LOG_DIR           "/var/log/robot"
#define PERF_LOG_PATH          PERF_LOG_DIR "/perf.csv"

/* Max number of tasks tracked */
#define PERF_MAX_TASKS         32

/* Default flush interval in seconds */
#define PERF_FLUSH_INTERVAL_S  60

/* ---------------------------------------------------------------------------
 * Per-task profiling data
 * -------------------------------------------------------------------------*/
typedef struct {
    char     task_name[configMAX_TASK_NAME_LEN];
    uint32_t total_runtime_us;       /* Accumulated runtime (microseconds) */
    uint32_t last_switch_in_us;      /* Timestamp at last switch-in        */
    uint32_t max_execution_us;       /* Max single execution time          */
    uint32_t prev_execution_us;      /* Last execution time                */
    uint32_t switch_count;           /* Number of context switches         */
    uint16_t stack_high_water;       /* Stack high-water mark (bytes)      */
    uint8_t  active;                 /* 1 if slot is occupied              */
} perf_task_t;

/* ---------------------------------------------------------------------------
 * State
 * -------------------------------------------------------------------------*/
static perf_task_t tasks[PERF_MAX_TASKS];
static int         task_count = 0;
static uint32_t    last_flush_time = 0;
static uint32_t    total_uptime_us = 0;

/* The current task index (-1 if idle) */
static int         current_task_idx = -1;
static uint32_t    current_switch_in_us = 0;

/* Mutex for thread safety (POSIX mutex for RK3566) */
static pthread_mutex_t perf_mutex = PTHREAD_MUTEX_INITIALIZER;

/* ---------------------------------------------------------------------------
 * Internal helpers
 * -------------------------------------------------------------------------*/

/**
 * @brief  Get current timestamp in microseconds (monotonic).
 */
static uint32_t perf_get_timestamp_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000000UL + ts.tv_nsec / 1000UL);
}

/**
 * @brief  Find or allocate a task slot.
 *
 * @param  name  Task name.
 * @return Index into tasks[] or -1 if table full.
 */
static int perf_find_or_add_task(const char *name)
{
    /* Look for existing */
    for (int i = 0; i < task_count; i++) {
        if (strncmp(tasks[i].task_name, name,
                    configMAX_TASK_NAME_LEN) == 0) {
            return i;
        }
    }

    /* Add new */
    if (task_count >= PERF_MAX_TASKS) {
        return -1;
    }

    int idx = task_count++;
    strncpy(tasks[idx].task_name, name, configMAX_TASK_NAME_LEN - 1);
    tasks[idx].task_name[configMAX_TASK_NAME_LEN - 1] = '\0';
    tasks[idx].total_runtime_us   = 0;
    tasks[idx].last_switch_in_us  = 0;
    tasks[idx].max_execution_us   = 0;
    tasks[idx].prev_execution_us  = 0;
    tasks[idx].switch_count       = 0;
    tasks[idx].stack_high_water   = 0;
    tasks[idx].active             = 1;

    return idx;
}

/**
 * @brief  Ensure the log directory exists.
 */
static int ensure_perf_log_dir(void)
{
    struct stat st;
    if (stat(PERF_LOG_DIR, &st) == 0) {
        if (S_ISDIR(st.st_mode)) {
            return 0;
        }
        unlink(PERF_LOG_DIR);
    }
    return mkdir(PERF_LOG_DIR, 0755);
}

/**
 * @brief  Write CSV header if file is empty.
 *
 * @param  fp  Open file handle.
 */
static void write_csv_header(FILE *fp)
{
    /* Check if file is empty */
    fseek(fp, 0, SEEK_END);
    long size = ftell(fp);

    if (size == 0) {
        fprintf(fp, "Timestamp,Task,CPU_Percent,StackHighWater,"
                    "MaxExec_us,AvgExec_us,SwitchCount\n");
    }
}

/**
 * @brief  Flush profiling data to CSV file.
 */
static void flush_csv(void)
{
    ensure_perf_log_dir();

    FILE *fp = fopen(PERF_LOG_PATH, "a");
    if (fp == NULL) {
        fprintf(stderr, "perf_profiler: cannot open %s: %s\n",
                PERF_LOG_PATH, strerror(errno));
        return;
    }

    write_csv_header(fp);

    /* Generate ISO timestamp */
    time_t now = time(NULL);
    struct tm *tm = gmtime(&now);
    char timestamp[32];
    if (tm != NULL) {
        strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%SZ", tm);
    } else {
        snprintf(timestamp, sizeof(timestamp), "1970-01-01T00:00:00Z");
    }

    /* Write one row per active task */
    for (int i = 0; i < task_count; i++) {
        if (!tasks[i].active) continue;

        float cpu_pct = 0.0f;
        if (total_uptime_us > 0) {
            cpu_pct = (float)tasks[i].total_runtime_us * 100.0f /
                      (float)total_uptime_us;
        }

        uint32_t avg_exec_us = 0;
        if (tasks[i].switch_count > 0) {
            avg_exec_us = tasks[i].total_runtime_us / tasks[i].switch_count;
        }

        fprintf(fp, "%s,%s,%.2f,%u,%u,%u,%u\n",
                timestamp,
                tasks[i].task_name,
                (double)cpu_pct,
                (unsigned int)tasks[i].stack_high_water,
                (unsigned int)tasks[i].max_execution_us,
                (unsigned int)avg_exec_us,
                (unsigned int)tasks[i].switch_count);
    }

    fclose(fp);

    /* Reset running totals for next interval */
    for (int i = 0; i < task_count; i++) {
        tasks[i].total_runtime_us  = 0;
        tasks[i].max_execution_us  = 0;
        tasks[i].switch_count      = 0;
    }
    total_uptime_us = 0;

    /* Rotate log if it exceeds 10 MB */
    perf_profiler_rotate();
}

/* ---------------------------------------------------------------------------
 * Public API — called from FreeRTOS trace macros
 * -------------------------------------------------------------------------*/

/**
 * @brief  Called when a task is switched in (from traceTASK_SWITCHED_IN).
 *
 * @param  task_name  Name of the task being switched to.
 */
void perf_profiler_task_switch_in(const char *task_name)
{
    pthread_mutex_lock(&perf_mutex);

    uint32_t now = perf_get_timestamp_us();

    /* Charge previous task's execution time */
    if (current_task_idx >= 0 && current_task_idx < task_count) {
        uint32_t elapsed = now - current_switch_in_us;
        tasks[current_task_idx].total_runtime_us += elapsed;
        if (elapsed > tasks[current_task_idx].max_execution_us) {
            tasks[current_task_idx].max_execution_us = elapsed;
        }
        tasks[current_task_idx].prev_execution_us = elapsed;
        tasks[current_task_idx].switch_count++;
        total_uptime_us += elapsed;
    }

    /* Switch to new task */
    int idx = perf_find_or_add_task(task_name);
    current_task_idx = idx;
    current_switch_in_us = now;

    pthread_mutex_unlock(&perf_mutex);
}

/**
 * @brief  Called when a task is switched out (from traceTASK_SWITCHED_OUT).
 *
 * @param  task_name  Name of the task being switched from.
 */
void perf_profiler_task_switch_out(const char *task_name)
{
    (void)task_name;
    /* Handled in _switch_in; this is provided for symmetry */
}

/**
 * @brief  Update the stack high-water mark for a task.
 *
 * @param  task_name  Task name.
 * @param  hwm        Stack high-water mark in bytes.
 */
void perf_profiler_stack_hwm(const char *task_name, uint16_t hwm)
{
    pthread_mutex_lock(&perf_mutex);

    int idx = perf_find_or_add_task(task_name);
    if (idx >= 0) {
        tasks[idx].stack_high_water = hwm;
    }

    pthread_mutex_unlock(&perf_mutex);
}

/**
 * @brief  Periodic flush function — call from a timer or main loop.
 *
 * Checks if PERF_FLUSH_INTERVAL_S has elapsed and writes CSV.
 *
 * @param  force  If non-zero, flush immediately regardless of interval.
 */
void perf_profiler_flush(int force)
{
    pthread_mutex_lock(&perf_mutex);

    uint32_t now_sec = (uint32_t)time(NULL);
    if (!force && (now_sec - last_flush_time < PERF_FLUSH_INTERVAL_S)) {
        pthread_mutex_unlock(&perf_mutex);
        return;
    }

    last_flush_time = now_sec;
    flush_csv();

    pthread_mutex_unlock(&perf_mutex);
}

/**
 * @brief  Rotate the perf log if it exceeds a maximum size.
 *
 * Keeps up to 3 rotated files (perf.csv, perf.csv.1, perf.csv.2).
 */
void perf_profiler_rotate(void)
{
    struct stat st;
    if (stat(PERF_LOG_PATH, &st) != 0) {
        return;  /* File doesn't exist yet */
    }

    /* Rotate if > 10 MB */
    if (st.st_size < 10 * 1024 * 1024) {
        return;
    }

    /* Shift old logs: .2 -> .3 (drop), .1 -> .2, .csv -> .1 */
    unlink(PERF_LOG_PATH ".2");
    rename(PERF_LOG_PATH ".1", PERF_LOG_PATH ".2");
    rename(PERF_LOG_PATH, PERF_LOG_PATH ".1");
}

/**
 * @brief  Initialise the profiler.
 */
void perf_profiler_init(void)
{
    pthread_mutex_init(&perf_mutex, NULL);
    memset(tasks, 0, sizeof(tasks));
    task_count = 0;
    current_task_idx = -1;
    current_switch_in_us = 0;
    last_flush_time = (uint32_t)time(NULL);
    total_uptime_us = 0;

    ensure_perf_log_dir();
}
