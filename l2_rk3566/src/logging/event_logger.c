/**
 * @file    event_logger.c
 * @brief   Structured event logger for robot system.
 *
 * @details Produces JSON-line event logs for:
 *   - STATE_CHANGE:  old_state, new_state, trigger
 *   - FAULT:         code, severity, context
 *   - SENSOR_READING: sensor, values (key:value pairs)
 *   - MOTOR_CMD:     v (linear), w (angular), timestamp
 *   - BATTERY_EVENT: type, value
 *
 * Log files are written to /var/log/robot/events/ and rotated:
 *   - Keep last 10 files
 *   - Maximum 1 MB per file
 *   - Naming: events_YYYYMMDD_HHMMSS.json
 *
 * The logger runs on the RK3566 side, receiving event data from the
 * STM32H747 via shared memory or UART.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>
#include <time.h>
#include <errno.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include "event_logger.h"

/* ---------------------------------------------------------------------------
 * Configuration
 * -------------------------------------------------------------------------*/

#define EVENTS_LOG_DIR        "/var/log/robot/events"
#define EVENTS_MAX_FILE_SIZE  (1UL * 1024UL * 1024UL)    /* 1 MB */
#define EVENTS_MAX_FILES      10
#define EVENTS_FILENAME_PFX   "events_"
#define EVENTS_FILENAME_PATTERN "events_%04u%02u%02u_%02u%02u%02u.json"

/* ---------------------------------------------------------------------------
 * State
 * -------------------------------------------------------------------------*/
static FILE        *current_file = NULL;
static uint32_t     current_file_size = 0;
static uint32_t     total_events = 0;
static int          initialized = 0;
static pthread_mutex_t log_mutex = PTHREAD_MUTEX_INITIALIZER;

/* ---------------------------------------------------------------------------
 * Internal helpers
 * -------------------------------------------------------------------------*/

/**
 * @brief  Ensure the log directory exists.
 *
 * @return 0 on success, -1 on error.
 */
static int ensure_events_dir(void)
{
    struct stat st;
    if (stat(EVENTS_LOG_DIR, &st) == 0) {
        if (S_ISDIR(st.st_mode)) {
            return 0;
        }
        unlink(EVENTS_LOG_DIR);
    }
    return mkdir(EVENTS_LOG_DIR, 0755);
}

/**
 * @brief  Get current local time components.
 */
static void get_local_time(uint16_t *year, uint8_t *mon, uint8_t *day,
                            uint8_t *hour, uint8_t *min, uint8_t *sec)
{
    time_t now = time(NULL);
    struct tm *tm = localtime(&now);
    if (tm == NULL) {
        *year = 1970; *mon = 1; *day = 1;
        *hour = 0; *min = 0; *sec = 0;
        return;
    }
    *year = (uint16_t)(tm->tm_year + 1900);
    *mon  = (uint8_t)(tm->tm_mon + 1);
    *day  = (uint8_t)tm->tm_mday;
    *hour = (uint8_t)tm->tm_hour;
    *min  = (uint8_t)tm->tm_min;
    *sec  = (uint8_t)tm->tm_sec;
}

/**
 * @brief  Generate a unique log filename based on timestamp.
 *
 * @param  buf   Output buffer (must be >= 64 bytes).
 * @param  len   Buffer length.
 */
static void make_filename(char *buf, size_t len)
{
    uint16_t y; uint8_t mo, d, h, mi, s;
    get_local_time(&y, &mo, &d, &h, &mi, &s);

    /* Ensure uniqueness by incrementing s if file exists */
    for (int attempt = 0; attempt < 100; attempt++) {
        snprintf(buf, len, "%s/" EVENTS_FILENAME_PATTERN,
                 EVENTS_LOG_DIR, y, mo, d, h, mi, s);

        struct stat st;
        if (stat(buf, &st) != 0) {
            return;  /* File does not exist — use this name */
        }
        s++;
        if (s >= 60) { s = 0; mi++; }
        if (mi >= 60) { mi = 0; h++; }
    }

    /* Fallback: use PID */
    snprintf(buf, len, "%s/events_%u.json",
             EVENTS_LOG_DIR, (unsigned int)getpid());
}

/**
 * @brief  Rotate old log files: keep only the newest EVENTS_MAX_FILES.
 *
 * Deletes the oldest files when the count exceeds EVENTS_MAX_FILES.
 */
static void rotate_logs(void)
{
    DIR *dir = opendir(EVENTS_LOG_DIR);
    if (dir == NULL) return;

    /* Collect event log filenames */
    typedef struct {
        char name[256];
        time_t mtime;
    } log_entry_t;

    log_entry_t files[256];
    int count = 0;

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL && count < 256) {
        if (entry->d_type != DT_REG) continue;
        if (strncmp(entry->d_name, EVENTS_FILENAME_PFX,
                    strlen(EVENTS_FILENAME_PFX)) != 0) {
            continue;
        }

        /* Get modification time */
        char fullpath[320];
        snprintf(fullpath, sizeof(fullpath), "%s/%s",
                 EVENTS_LOG_DIR, entry->d_name);

        struct stat st;
        if (stat(fullpath, &st) != 0) continue;

        strncpy(files[count].name, fullpath, sizeof(files[count].name) - 1);
        files[count].mtime = st.st_mtime;
        count++;
    }
    closedir(dir);

    /* Sort by mtime (oldest first) using simple insertion sort */
    for (int i = 1; i < count; i++) {
        log_entry_t key = files[i];
        int j = i - 1;
        while (j >= 0 && files[j].mtime > key.mtime) {
            files[j + 1] = files[j];
            j--;
        }
        files[j + 1] = key;
    }

    /* Delete oldest files that exceed the limit */
    int to_delete = count - EVENTS_MAX_FILES;
    for (int i = 0; i < to_delete; i++) {
        unlink(files[i].name);
    }
}

/**
 * @brief  Open a new log file for writing.
 *
 * @return 0 on success, -1 on error.
 */
static int open_new_file(void)
{
    if (current_file != NULL) {
        fclose(current_file);
        current_file = NULL;
    }

    ensure_events_dir();

    char filename[320];
    make_filename(filename, sizeof(filename));

    current_file = fopen(filename, "w");
    if (current_file == NULL) {
        fprintf(stderr, "event_logger: cannot create %s: %s\n",
                filename, strerror(errno));
        return -1;
    }

    current_file_size = 0;
    setbuf(current_file, NULL);  /* No buffering for crash safety */
    return 0;
}

/**
 * @brief  Write a JSON string to the log file.
 *
 * @param  json  Null-terminated JSON line (newline appended).
 * @return 0 on success, -1 on error.
 */
static int write_json_line(const char *json)
{
    if (current_file == NULL) {
        if (open_new_file() != 0) {
            return -1;
        }
    }

    size_t len = strlen(json);
    if (fwrite(json, 1, len, current_file) != len) {
        /* Write failed — try reopening */
        fclose(current_file);
        current_file = NULL;
        return -1;
    }

    /* Append newline */
    if (fwrite("\n", 1, 1, current_file) != 1) {
        fclose(current_file);
        current_file = NULL;
        return -1;
    }

    current_file_size += (uint32_t)(len + 1);
    total_events++;

    /* Rotate if file exceeds max size */
    if (current_file_size >= EVENTS_MAX_FILE_SIZE) {
        fclose(current_file);
        current_file = NULL;
        rotate_logs();
    }

    return 0;
}

/* ---------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------*/

/**
 * @brief  Initialise the event logger.
 *
 * Creates the log directory, trims old logs, and opens a new file.
 */
void event_logger_init(void)
{
    pthread_mutex_lock(&log_mutex);

    ensure_events_dir();
    rotate_logs();

    current_file = NULL;
    current_file_size = 0;
    total_events = 0;
    initialized = 1;

    open_new_file();

    pthread_mutex_unlock(&log_mutex);
}

/**
 * @brief  Log a state change event.
 *
 * @param  old_state  Previous state name.
 * @param  new_state  New state name.
 * @param  trigger    Trigger that caused the transition.
 */
void event_logger_state_change(const char *old_state,
                                const char *new_state,
                                const char *trigger)
{
    if (!initialized) return;

    char json[512];
    snprintf(json, sizeof(json),
             "{\"type\":\"STATE_CHANGE\",\"old\":\"%s\",\"new\":\"%s\","
             "\"trigger\":\"%s\",\"ts\":%lu}",
             old_state ? old_state : "null",
             new_state ? new_state : "null",
             trigger  ? trigger  : "null",
             (unsigned long)time(NULL));

    pthread_mutex_lock(&log_mutex);
    write_json_line(json);
    pthread_mutex_unlock(&log_mutex);
}

/**
 * @brief  Log a fault event.
 *
 * @param  code      Fault code (application-specific).
 * @param  severity  Severity string (e.g., "WARN", "ERROR", "FATAL").
 * @param  context   Additional context (JSON object string or free text).
 */
void event_logger_fault(uint32_t code, const char *severity,
                         const char *context)
{
    if (!initialized) return;

    char json[512];
    snprintf(json, sizeof(json),
             "{\"type\":\"FAULT\",\"code\":%u,\"severity\":\"%s\","
             "\"context\":\"%s\",\"ts\":%lu}",
             (unsigned int)code,
             severity ? severity : "UNKNOWN",
             context  ? context  : "",
             (unsigned long)time(NULL));

    pthread_mutex_lock(&log_mutex);
    write_json_line(json);
    pthread_mutex_unlock(&log_mutex);
}

/**
 * @brief  Log a sensor reading event.
 *
 * @param  sensor   Sensor name (e.g., "IMU", "BMS", "VL53L5CX").
 * @param  values   JSON object string with key:value pairs.
 */
void event_logger_sensor(const char *sensor, const char *values)
{
    if (!initialized) return;

    char json[512];
    snprintf(json, sizeof(json),
             "{\"type\":\"SENSOR_READING\",\"sensor\":\"%s\","
             "\"values\":%s,\"ts\":%lu}",
             sensor ? sensor : "unknown",
             values ? values : "{}",
             (unsigned long)time(NULL));

    pthread_mutex_lock(&log_mutex);
    write_json_line(json);
    pthread_mutex_unlock(&log_mutex);
}

/**
 * @brief  Log a motor command event.
 *
 * @param  v          Linear velocity (m/s).
 * @param  w          Angular velocity (rad/s).
 * @param  timestamp  Command timestamp (microseconds, monotonic).
 */
void event_logger_motor_cmd(float v, float w, uint64_t timestamp)
{
    if (!initialized) return;

    char json[256];
    snprintf(json, sizeof(json),
             "{\"type\":\"MOTOR_CMD\",\"v\":%.3f,\"w\":%.3f,"
             "\"cmd_ts\":%llu,\"ts\":%lu}",
             (double)v, (double)w,
             (unsigned long long)timestamp,
             (unsigned long)time(NULL));

    pthread_mutex_lock(&log_mutex);
    write_json_line(json);
    pthread_mutex_unlock(&log_mutex);
}

/**
 * @brief  Log a battery event.
 *
 * @param  type   Event type (e.g., "SOC_CHANGE", "CHARGING", "DISCHARGING",
 *                "FAULT", "TEMP_HIGH").
 * @param  value  Numeric value associated with the event.
 */
void event_logger_battery(const char *type, float value)
{
    if (!initialized) return;

    char json[256];
    snprintf(json, sizeof(json),
             "{\"type\":\"BATTERY_EVENT\",\"battery_type\":\"%s\","
             "\"value\":%.2f,\"ts\":%lu}",
             type  ? type : "unknown",
             (double)value,
             (unsigned long)time(NULL));

    pthread_mutex_lock(&log_mutex);
    write_json_line(json);
    pthread_mutex_unlock(&log_mutex);
}

/**
 * @brief  Log a raw custom event (advanced usage).
 *
 * @param  json  Complete JSON object string (must be a single line).
 */
void event_logger_raw(const char *json)
{
    if (!initialized || json == NULL) return;

    pthread_mutex_lock(&log_mutex);
    write_json_line(json);
    pthread_mutex_unlock(&log_mutex);
}

/**
 * @brief  Flush and close the current log file.
 */
void event_logger_close(void)
{
    pthread_mutex_lock(&log_mutex);

    if (current_file != NULL) {
        fclose(current_file);
        current_file = NULL;
    }

    pthread_mutex_unlock(&log_mutex);
}

/**
 * @brief  Get the total number of events logged since init.
 *
 * @return Event count.
 */
uint32_t event_logger_total_events(void)
{
    return total_events;
}
