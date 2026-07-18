/**
 * voice_ctrl.c — Voice control integration
 *
 * Dispatch cloud-to-device voice intents to robot commands.
 * Intents: start_cleaning, stop, go_home, clean_kitchen (navigate to room + clean).
 * Integrates with device_shadow desired state and robot_ctrl.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <cjson/cJSON.h>

#include "voice_ctrl.h"
#include "device_shadow.h"
#include "cloud_selector.h"
#include "../control/robot_ctrl.h"

/* ---------------------------------------------------------------------------
 * Constants
 * --------------------------------------------------------------------------- */
#define VOICE_INTENT_TOPIC      "voice/intent"
#define VOICE_CONFIDENCE_MIN    0.60f    /* reject below 60% confidence */
#define VOICE_MAX_TEXT          256

/* ---------------------------------------------------------------------------
 * Intent mapping
 * --------------------------------------------------------------------------- */
typedef struct {
    const char *intent_name;
    shadow_cmd_t command;
    clean_mode_t mode;
    const char *target_room;
} voice_intent_map_t;

static const voice_intent_map_t g_intent_map[] = {
    { "start_cleaning",  CMD_START_CLEAN, CLEAN_MODE_AUTO,  NULL },
    { "start_clean",     CMD_START_CLEAN, CLEAN_MODE_AUTO,  NULL },
    { "clean_all",       CMD_START_CLEAN, CLEAN_MODE_AUTO,  NULL },
    { "stop",            CMD_STOP,        CLEAN_MODE_AUTO,  NULL },
    { "pause",           CMD_PAUSE,       CLEAN_MODE_AUTO,  NULL },
    { "go_home",         CMD_DOCK,        CLEAN_MODE_AUTO,  NULL },
    { "return_to_base",  CMD_DOCK,        CLEAN_MODE_AUTO,  NULL },
    { "dock",            CMD_DOCK,        CLEAN_MODE_AUTO,  NULL },
    { "clean_kitchen",   CMD_START_CLEAN, CLEAN_MODE_ROOM,  "kitchen" },
    { "clean_living_room", CMD_START_CLEAN, CLEAN_MODE_ROOM, "living_room" },
    { "clean_bedroom",   CMD_START_CLEAN, CLEAN_MODE_ROOM,  "bedroom" },
    { "clean_bathroom",  CMD_START_CLEAN, CLEAN_MODE_ROOM,  "bathroom" },
    { "clean_office",    CMD_START_CLEAN, CLEAN_MODE_ROOM,  "office" },
    { "clean_dining",    CMD_START_CLEAN, CLEAN_MODE_ROOM,  "dining_room" },
    { "spot_clean",      CMD_START_CLEAN, CLEAN_MODE_SPOT,  NULL },
    { "edge_clean",      CMD_START_CLEAN, CLEAN_MODE_EDGE,  NULL },
    { "resume",          CMD_RESUME,      CLEAN_MODE_AUTO,  NULL },
    { "set_mode_auto",   CMD_SET_MODE,    CLEAN_MODE_AUTO,  NULL },
    { "set_mode_spot",   CMD_SET_MODE,    CLEAN_MODE_SPOT,  NULL },
    { "set_mode_edge",   CMD_SET_MODE,    CLEAN_MODE_EDGE,  NULL },
    { "set_mode_room",   CMD_SET_MODE,    CLEAN_MODE_ROOM,  NULL },
    { NULL,              CMD_NONE,        CLEAN_MODE_AUTO,  NULL }
};

/* ---------------------------------------------------------------------------
 * Context
 * --------------------------------------------------------------------------- */
typedef struct {
    pthread_mutex_t lock;
    int  enabled;
    int  verbose;            /* print recognized intents to console */
    char last_intent[64];
    char last_text[VOICE_MAX_TEXT];
    float last_confidence;
    uint64_t intent_count;
} voice_ctx_t;

static voice_ctx_t g_voice;

/* ---------------------------------------------------------------------------
 * Handle an incoming voice intent from the cloud
 * --------------------------------------------------------------------------- */
static int voice_handle_intent(const char *intent_name, float confidence,
                                const char *raw_text, const char *target_room_override) {
    voice_ctx_t *ctx = &g_voice;

    if (!ctx->enabled) {
        fprintf(stdout, "[Voice] Voice control disabled, ignoring intent\n");
        return -1;
    }

    if (confidence < VOICE_CONFIDENCE_MIN) {
        fprintf(stdout, "[Voice] Intent '%s' rejected: low confidence (%.2f < %.2f)\n",
                intent_name, confidence, VOICE_CONFIDENCE_MIN);
        return -1;
    }

    /* Look up intent in mapping table */
    const voice_intent_map_t *map = g_intent_map;
    while (map->intent_name) {
        if (strcasecmp(map->intent_name, intent_name) == 0)
            break;
        map++;
    }

    if (!map->intent_name) {
        fprintf(stdout, "[Voice] Unknown intent: %s\n", intent_name);
        return -1;
    }

    /* Record */
    pthread_mutex_lock(&ctx->lock);
    strncpy(ctx->last_intent, intent_name, sizeof(ctx->last_intent) - 1);
    strncpy(ctx->last_text, raw_text ? raw_text : "", sizeof(ctx->last_text) - 1);
    ctx->last_confidence = confidence;
    ctx->intent_count++;
    pthread_mutex_unlock(&ctx->lock);

    fprintf(stdout, "[Voice] Intent: %s (confidence: %.2f) -> cmd=%d mode=%d\n",
            intent_name, confidence, map->command, map->mode);

    /* Build desired state from intent */
    shadow_desired_t desired;
    memset(&desired, 0, sizeof(desired));
    desired.command = map->command;
    desired.mode = map->mode;

    if (target_room_override && target_room_override[0]) {
        strncpy(desired.target_room, target_room_override, sizeof(desired.target_room) - 1);
    } else if (map->target_room) {
        strncpy(desired.target_room, map->target_room, sizeof(desired.target_room) - 1);
    }

    /* Apply via device shadow (which will call robot_ctrl) */
    /* Set desired state and trigger delta application */
    device_shadow_set_desired(desired);

    /* Acknowledge to cloud */
    char ack_json[256];
    snprintf(ack_json, sizeof(ack_json),
             "{\"intent\":\"%s\",\"status\":\"accepted\",\"confidence\":%.2f}",
             intent_name, confidence);
    cloud_publish("voice/ack", ack_json, strlen(ack_json));

    return 0;
}

/* ---------------------------------------------------------------------------
 * Cloud message callback for voice intents
 * Expected JSON format:
 *   { "intent": "start_cleaning", "confidence": 0.95, "text": "start cleaning" }
 * or Alibaba Cloud format:
 *   { "payload": { "intent": "...", "confidence": 0.95 }, ...
 * --------------------------------------------------------------------------- */
static void voice_on_cloud_message(const char *topic, const char *payload, size_t len) {
    (void)topic;
    cJSON *root = cJSON_ParseWithLength(payload, len);
    if (!root) {
        fprintf(stderr, "[Voice] Failed to parse cloud message\n");
        return;
    }

    const char *intent = NULL;
    float confidence = 1.0f;
    const char *text = NULL;
    const char *room = NULL;

    /* Try direct: { "intent": "...", ... } */
    cJSON *jintent = cJSON_GetObjectItem(root, "intent");
    if (jintent && cJSON_IsString(jintent)) {
        intent = jintent->valuestring;
    }

    /* Try nested: { "payload": { "intent": "...", ... } } */
    if (!intent) {
        cJSON *payload_obj = cJSON_GetObjectItem(root, "payload");
        if (payload_obj) {
            jintent = cJSON_GetObjectItem(payload_obj, "intent");
            if (jintent && cJSON_IsString(jintent))
                intent = jintent->valuestring;

            cJSON *jconf = cJSON_GetObjectItem(payload_obj, "confidence");
            if (jconf && cJSON_IsNumber(jconf))
                confidence = (float)jconf->valuedouble;

            cJSON *jtext = cJSON_GetObjectItem(payload_obj, "text");
            if (jtext && cJSON_IsString(jtext))
                text = jtext->valuestring;

            cJSON *jroom = cJSON_GetObjectItem(payload_obj, "room");
            if (jroom && cJSON_IsString(jroom))
                room = jroom->valuestring;
        }
    }

    /* Try flat confidence + text */
    if (!intent) {
        cJSON *jconf = cJSON_GetObjectItem(root, "confidence");
        if (jconf && cJSON_IsNumber(jconf))
            confidence = (float)jconf->valuedouble;

        cJSON *jtext = cJSON_GetObjectItem(root, "text");
        if (jtext && cJSON_IsString(jtext))
            text = jtext->valuestring;

        cJSON *jroom = cJSON_GetObjectItem(root, "room");
        if (jroom && cJSON_IsString(jroom))
            room = jroom->valuestring;
    }

    if (intent) {
        voice_handle_intent(intent, confidence, text, room);
    }

    cJSON_Delete(root);
}

/* ---------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------- */
int voice_ctrl_init(void) {
    memset(&g_voice, 0, sizeof(g_voice));
    pthread_mutex_init(&g_voice.lock, NULL);
    g_voice.enabled = 1;
    g_voice.verbose = 1;
    return 0;
}

int voice_ctrl_start(void) {
    /* Subscribe to voice intent topics */
    cloud_subscribe(VOICE_INTENT_TOPIC, voice_on_cloud_message);

    /* Subscribe to Alibaba Cloud service topics that may carry voice commands */
    cloud_subscribe("/sys/+/+/thing/service/voice", voice_on_cloud_message);

    fprintf(stdout, "[Voice] Voice control started\n");
    return 0;
}

int voice_ctrl_stop(void) {
    g_voice.enabled = 0;
    return 0;
}

void voice_ctrl_enable(int enabled) {
    g_voice.enabled = enabled;
    fprintf(stdout, "[Voice] Voice control %s\n", enabled ? "enabled" : "disabled");
}

int voice_ctrl_is_enabled(void) {
    return g_voice.enabled;
}

/* For local voice recognition (on-device NLP) integration */
int voice_ctrl_process_local(const char *intent_name, float confidence,
                              const char *raw_text) {
    return voice_handle_intent(intent_name, confidence, raw_text, NULL);
}

/* Add custom intent mapping at runtime */
int voice_ctrl_add_intent(const char *intent_name, shadow_cmd_t command,
                           clean_mode_t mode, const char *room) {
    /* Runtime extension not implemented in fixed table; use for future dynamic loading */
    (void)intent_name;
    (void)command;
    (void)mode;
    (void)room;
    fprintf(stdout, "[Voice] Dynamic intent registration not supported in this build\n");
    return -1;
}

const char *voice_ctrl_get_last_intent(void) {
    return g_voice.last_intent;
}

float voice_ctrl_get_last_confidence(void) {
    return g_voice.last_confidence;
}
