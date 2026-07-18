/**
 * device_shadow.c — Device shadow implementation (AWS IoT Shadow pattern)
 *
 * Reported state: robot actual position, battery level, operational state, errors.
 * Desired state: cloud commands (start_clean, stop, dock, set_mode).
 * On desired change: apply delta to robot control.
 * Report new state after action completes.
 *
 * Works with all cloud providers via cloud_selector abstraction.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <cjson/cJSON.h>

#include "device_shadow.h"
#include "cloud_selector.h"
#include "../control/robot_ctrl.h"  /* for set_mode, navigate, etc. */

/* ---------------------------------------------------------------------------
 * Constants
 * --------------------------------------------------------------------------- */
#define SHADOW_STATE_VERSION_MAX  64
#define SHADOW_JSON_PAYLOAD_MAX   2048
#define SHADOW_REPORT_INTERVAL_MS 5000  /* periodic full state report */

/* ---------------------------------------------------------------------------
 * Shadow state
 * --------------------------------------------------------------------------- */
typedef struct {
    /* === Reported (actual robot state) === */
    shadow_reported_t reported;

    /* === Desired (from cloud) === */
    shadow_desired_t  desired;

    /* === Metadata === */
    int             version;
    int             dirty;          /* 1 if state changed and needs upload */
    uint64_t        last_report_ms;
    int             delta_pending;  /* 1 if delta waiting for apply */

    /* Lock */
    pthread_mutex_t lock;
    pthread_t       report_thread;
    int             keep_running;

    /* Callback for applying desired state to robot */
    void (*apply_cb)(const shadow_desired_t *desired);
} shadow_ctx_t;

static shadow_ctx_t g_shadow;

/* ---------------------------------------------------------------------------
 * Forward declarations
 * --------------------------------------------------------------------------- */
static void shadow_report_loop(void);
static void shadow_apply_delta(void);
static int  shadow_build_reported_json(char *buf, size_t cap);
static int  shadow_parse_desired_json(const char *json, shadow_desired_t *out);
static void shadow_on_cloud_msg(const char *topic, const char *payload, size_t len);

/* ---------------------------------------------------------------------------
 * Initialize shadow with defaults
 * --------------------------------------------------------------------------- */
int device_shadow_init(void) {
    memset(&g_shadow, 0, sizeof(g_shadow));
    pthread_mutex_init(&g_shadow.lock, NULL);
    g_shadow.reported.battery_pct = 100;
    g_shadow.reported.state = ROBOT_STATE_IDLE;
    g_shadow.reported.mode = CLEAN_MODE_AUTO;
    g_shadow.reported.charge_current_ma = 0;
    g_shadow.reported.linear_velocity_ms = 0.0f;
    g_shadow.reported.angular_velocity_rads = 0.0f;
    g_shadow.desired.command = CMD_NONE;
    g_shadow.desired.mode = CLEAN_MODE_AUTO;
    g_shadow.version = 1;
    g_shadow.last_report_ms = 0;
    g_shadow.keep_running = 1;
    return 0;
}

/* ---------------------------------------------------------------------------
 * Set callback for applying desired state to robot control
 * --------------------------------------------------------------------------- */
void device_shadow_set_apply_callback(void (*cb)(const shadow_desired_t *)) {
    g_shadow.apply_cb = cb;
}

/* ---------------------------------------------------------------------------
 * Cloud message handler — called by cloud_selector when shadow topic received
 * --------------------------------------------------------------------------- */
static void shadow_on_cloud_msg(const char *topic, const char *payload, size_t len) {
    (void)topic;
    cJSON *root = cJSON_ParseWithLength(payload, len);
    if (!root) {
        fprintf(stderr, "[Shadow] Failed to parse cloud message\n");
        return;
    }

    /* Desired state from "state.desired" or direct "desired" */
    cJSON *state = cJSON_GetObjectItem(root, "state");
    cJSON *desired = NULL;
    if (state)
        desired = cJSON_GetObjectItem(state, "desired");
    else
        desired = cJSON_GetObjectItem(root, "desired");

    if (!desired) {
        /* Also check for delta format: {"version": N, "delta": {...}} */
        cJSON *delta = cJSON_GetObjectItem(root, "delta");
        if (delta)
            desired = delta;
    }

    if (!desired || !cJSON_IsObject(desired)) {
        cJSON_Delete(root);
        return;
    }

    /* Convert cJSON desired to char string */
    char *desired_str = cJSON_PrintUnformatted(desired);
    if (!desired_str) { cJSON_Delete(root); return; }

    /* Parse into shadow_desired_t */
    shadow_desired_t new_desired;
    memset(&new_desired, 0, sizeof(new_desired));
    new_desired.command = CMD_NONE;

    if (shadow_parse_desired_json(desired_str, &new_desired) == 0) {
        pthread_mutex_lock(&g_shadow.lock);
        g_shadow.desired = new_desired;
        g_shadow.delta_pending = 1;
        g_shadow.dirty = 1;
        pthread_mutex_unlock(&g_shadow.lock);
    }

    free(desired_str);

    /* Check version */
    cJSON *ver = cJSON_GetObjectItem(root, "version");
    if (ver && cJSON_IsNumber(ver))
        g_shadow.version = ver->valueint + 1;

    cJSON_Delete(root);

    /* Apply delta immediately */
    shadow_apply_delta();
}

/* ---------------------------------------------------------------------------
 * Parse desired JSON into struct
 * --------------------------------------------------------------------------- */
static int shadow_parse_desired_json(const char *json, shadow_desired_t *out) {
    cJSON *root = cJSON_Parse(json);
    if (!root) return -1;

    cJSON *cmd = cJSON_GetObjectItem(root, "command");
    if (cmd && cJSON_IsString(cmd)) {
        const char *s = cmd->valuestring;
        if (strcmp(s, "start_clean") == 0)  out->command = CMD_START_CLEAN;
        else if (strcmp(s, "stop") == 0)    out->command = CMD_STOP;
        else if (strcmp(s, "dock") == 0)    out->command = CMD_DOCK;
        else if (strcmp(s, "pause") == 0)   out->command = CMD_PAUSE;
        else if (strcmp(s, "resume") == 0)  out->command = CMD_RESUME;
        else if (strcmp(s, "home") == 0)    out->command = CMD_DOCK;
        else if (strcmp(s, "set_mode") == 0) out->command = CMD_SET_MODE;
        else out->command = CMD_NONE;
    }

    cJSON *mode = cJSON_GetObjectItem(root, "mode");
    if (mode && cJSON_IsString(mode)) {
        const char *s = mode->valuestring;
        if (strcmp(s, "auto") == 0)          out->mode = CLEAN_MODE_AUTO;
        else if (strcmp(s, "edge") == 0)     out->mode = CLEAN_MODE_EDGE;
        else if (strcmp(s, "spot") == 0)     out->mode = CLEAN_MODE_SPOT;
        else if (strcmp(s, "room") == 0)     out->mode = CLEAN_MODE_ROOM;
        else out->mode = CLEAN_MODE_AUTO;
    }

    cJSON *room = cJSON_GetObjectItem(root, "room");
    if (room && cJSON_IsString(room)) {
        strncpy(out->target_room, room->valuestring, sizeof(out->target_room) - 1);
    }

    cJSON *fan_speed = cJSON_GetObjectItem(root, "fan_speed");
    if (fan_speed && cJSON_IsNumber(fan_speed))
        out->fan_speed_pct = fan_speed->valueint;

    cJSON *repeat = cJSON_GetObjectItem(root, "repeat");
    if (repeat && cJSON_IsNumber(repeat))
        out->repeat_count = repeat->valueint;

    cJSON_Delete(root);
    return 0;
}

/* ---------------------------------------------------------------------------
 * Build reported state JSON
 * --------------------------------------------------------------------------- */
static int shadow_build_reported_json(char *buf, size_t cap) {
    cJSON *root = cJSON_CreateObject();
    if (!root) return -1;

    pthread_mutex_lock(&g_shadow.lock);
    cJSON_AddNumberToObject(root, "battery", g_shadow.reported.battery_pct);
    cJSON_AddNumberToObject(root, "state", g_shadow.reported.state);
    cJSON_AddNumberToObject(root, "mode", g_shadow.reported.mode);
    cJSON_AddNumberToObject(root, "charge_current", g_shadow.reported.charge_current_ma);
    cJSON_AddNumberToObject(root, "version", g_shadow.version);
    cJSON_AddBoolToObject(root, "docked", g_shadow.reported.docked ? 1 : 0);
    cJSON_AddBoolToObject(root, "charging", g_shadow.reported.charging ? 1 : 0);
    cJSON_AddBoolToObject(root, "error", g_shadow.reported.error ? 1 : 0);
    cJSON_AddNumberToObject(root, "error_code", g_shadow.reported.error_code);

    /* Position */
    cJSON *pos = cJSON_AddObjectToObject(root, "position");
    if (pos) {
        cJSON_AddNumberToObject(pos, "x", g_shadow.reported.pos_x_mm);
        cJSON_AddNumberToObject(pos, "y", g_shadow.reported.pos_y_mm);
        cJSON_AddNumberToObject(pos, "theta", g_shadow.reported.pos_theta_deg);
    }

    /* Velocities */
    cJSON_AddNumberToObject(root, "linear_vel", g_shadow.reported.linear_velocity_ms);
    cJSON_AddNumberToObject(root, "angular_vel", g_shadow.reported.angular_velocity_rads);

    /* Timestamp */
    cJSON_AddNumberToObject(root, "timestamp", (double)time(NULL));
    pthread_mutex_unlock(&g_shadow.lock);

    char *str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!str) return -1;

    strncpy(buf, str, cap - 1);
    free(str);
    return 0;
}

/* ---------------------------------------------------------------------------
 * Apply desired state delta to robot control
 * --------------------------------------------------------------------------- */
static void shadow_apply_delta(void) {
    pthread_mutex_lock(&g_shadow.lock);
    if (!g_shadow.delta_pending) {
        pthread_mutex_unlock(&g_shadow.lock);
        return;
    }
    shadow_desired_t desired = g_shadow.desired;
    g_shadow.delta_pending = 0;
    pthread_mutex_unlock(&g_shadow.lock);

    fprintf(stdout, "[Shadow] Applying delta: cmd=%d mode=%d\n",
            desired.command, desired.mode);

    /* Call the higher-layer apply callback if set */
    if (g_shadow.apply_cb) {
        g_shadow.apply_cb(&desired);
    } else {
        /* Default behavior: translate to robot control calls */
        switch (desired.command) {
        case CMD_START_CLEAN:
            robot_set_mode(desired.mode);
            robot_start();
            break;
        case CMD_STOP:
            robot_stop();
            robot_set_mode(CLEAN_MODE_AUTO);
            break;
        case CMD_DOCK:
            robot_navigate_to_dock();
            break;
        case CMD_PAUSE:
            robot_pause();
            break;
        case CMD_RESUME:
            robot_resume();
            break;
        case CMD_SET_MODE:
            robot_set_mode(desired.mode);
            break;
        case CMD_NONE:
        default:
            break;
        }
    }

    /* Mark dirty so that new state is reported */
    pthread_mutex_lock(&g_shadow.lock);
    g_shadow.dirty = 1;
    pthread_mutex_unlock(&g_shadow.lock);

    /* Immediately report state after delta application */
    device_shadow_report();
}

/* ---------------------------------------------------------------------------
 * Periodic state report thread
 * --------------------------------------------------------------------------- */
static void *shadow_report_thread(void *arg) {
    (void)arg;
    while (g_shadow.keep_running) {
        usleep(SHADOW_REPORT_INTERVAL_MS * 1000);
        device_shadow_report();
    }
    return NULL;
}

/* ---------------------------------------------------------------------------
 * Public API — update reported state elements
 * --------------------------------------------------------------------------- */
void device_shadow_update_battery(int pct) {
    pthread_mutex_lock(&g_shadow.lock);
    if (g_shadow.reported.battery_pct != pct) {
        g_shadow.reported.battery_pct = pct;
        g_shadow.dirty = 1;
    }
    pthread_mutex_unlock(&g_shadow.lock);
}

void device_shadow_update_state(int state) {
    pthread_mutex_lock(&g_shadow.lock);
    if (g_shadow.reported.state != state) {
        g_shadow.reported.state = state;
        g_shadow.dirty = 1;
    }
    pthread_mutex_unlock(&g_shadow.lock);
}

void device_shadow_update_position(float x_mm, float y_mm, float theta_deg) {
    pthread_mutex_lock(&g_shadow.lock);
    g_shadow.reported.pos_x_mm = x_mm;
    g_shadow.reported.pos_y_mm = y_mm;
    g_shadow.reported.pos_theta_deg = theta_deg;
    g_shadow.dirty = 1;
    pthread_mutex_unlock(&g_shadow.lock);
}

void device_shadow_update_error(int error_code) {
    pthread_mutex_lock(&g_shadow.lock);
    g_shadow.reported.error = (error_code != 0);
    g_shadow.reported.error_code = error_code;
    g_shadow.dirty = 1;
    pthread_mutex_unlock(&g_shadow.lock);
}

void device_shadow_update_charging(int enabled, int current_ma) {
    pthread_mutex_lock(&g_shadow.lock);
    g_shadow.reported.charging = enabled;
    g_shadow.reported.charge_current_ma = current_ma;
    if (enabled)
        g_shadow.reported.docked = 1;
    g_shadow.dirty = 1;
    pthread_mutex_unlock(&g_shadow.lock);
}

void device_shadow_update_docked(int docked) {
    pthread_mutex_lock(&g_shadow.lock);
    g_shadow.reported.docked = docked;
    g_shadow.dirty = 1;
    pthread_mutex_unlock(&g_shadow.lock);
}

void device_shadow_update_velocity(float linear_ms, float angular_rads) {
    pthread_mutex_lock(&g_shadow.lock);
    g_shadow.reported.linear_velocity_ms = linear_ms;
    g_shadow.reported.angular_velocity_rads = angular_rads;
    g_shadow.dirty = 1;
    pthread_mutex_unlock(&g_shadow.lock);
}

/* ---------------------------------------------------------------------------
 * Report current state to cloud
 * --------------------------------------------------------------------------- */
int device_shadow_report(void) {
    char json[SHADOW_JSON_PAYLOAD_MAX];

    if (shadow_build_reported_json(json, sizeof(json)) < 0)
        return -1;

    pthread_mutex_lock(&g_shadow.lock);
    int was_dirty = g_shadow.dirty;
    g_shadow.dirty = 0;
    int version = g_shadow.version;
    g_shadow.version++;
    pthread_mutex_unlock(&g_shadow.lock);

    /* Publish via cloud selector */
    char topic[128];
    snprintf(topic, sizeof(topic), "$aws/things/%s/shadow/update",
             cloud_selector_get_config()->aws.thing_name);

    return cloud_publish(topic, json, strlen(json));
}

/* ---------------------------------------------------------------------------
 * Start shadow service
 * --------------------------------------------------------------------------- */
int device_shadow_start(void) {
    /* Subscribe to shadow delta topic */
    cloud_subscribe("$aws/things/+/shadow/update/delta",
                    shadow_on_cloud_msg);
    cloud_subscribe("$aws/things/+/shadow/delta",
                    shadow_on_cloud_msg);

    /* Also subscribe to Alibaba Cloud format */
    cloud_subscribe("/sys/+/+/thing/service/*",
                    shadow_on_cloud_msg);

    /* Start periodic report thread */
    g_shadow.keep_running = 1;
    pthread_create(&g_shadow.report_thread, NULL, shadow_report_thread, NULL);

    /* Report initial state */
    device_shadow_report();
    return 0;
}

int device_shadow_stop(void) {
    g_shadow.keep_running = 0;
    pthread_join(g_shadow.report_thread, NULL);
    return 0;
}

/* ---------------------------------------------------------------------------
 * Accessors
 * --------------------------------------------------------------------------- */
shadow_reported_t device_shadow_get_reported(void) {
    shadow_reported_t r;
    pthread_mutex_lock(&g_shadow.lock);
    r = g_shadow.reported;
    pthread_mutex_unlock(&g_shadow.lock);
    return r;
}

shadow_desired_t device_shadow_get_desired(void) {
    shadow_desired_t d;
    pthread_mutex_lock(&g_shadow.lock);
    d = g_shadow.desired;
    pthread_mutex_unlock(&g_shadow.lock);
    return d;
}
