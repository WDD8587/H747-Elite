/**
 * cloud_selector.c — Cloud platform auto-selection
 *
 * Reads cloud config from config.robot.yaml to select: aws / aliyun / tuya / local.
 * Provides an abstract interface: cloud_publish, cloud_subscribe, cloud_connect, cloud_disconnect.
 * Factory provisioning writes cloud config via provisioning agent.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <yaml.h>   /* libyaml for parsing config.robot.yaml */

#include "cloud_selector.h"
#include "device_shadow.h"
#include "telemetry_batch.h"

/* ---------------------------------------------------------------------------
 * Cloud provider implementations
 * --------------------------------------------------------------------------- */
#ifdef CLOUD_AWS
#  include "aws_iot.h"
#endif
#ifdef CLOUD_ALIYUN
#  include "aliyun_iot.h"
#endif
#ifdef CLOUD_TUYA
#  include "tuya_iot.h"
#endif

/* ---------------------------------------------------------------------------
 * Config path
 * --------------------------------------------------------------------------- */
#define CONFIG_PATH "/etc/robot/config.robot.yaml"

/* ---------------------------------------------------------------------------
 * Current provider state
 * --------------------------------------------------------------------------- */
static cloud_provider_t g_provider = CLOUD_PROVIDER_UNKNOWN;
static cloud_config_t   g_config;
static int              g_initialized = 0;
static pthread_mutex_t  g_lock = PTHREAD_MUTEX_INITIALIZER;

/* ---------------------------------------------------------------------------
 * Forward declarations
 * --------------------------------------------------------------------------- */
static cloud_provider_t parse_provider(const char *name);
static int              load_config_from_yaml(const char *path, cloud_config_t *cfg);
static void             default_local_config(cloud_config_t *cfg);

/* ---------------------------------------------------------------------------
 * Parse provider string from config
 * --------------------------------------------------------------------------- */
static cloud_provider_t parse_provider(const char *name) {
    if (!name) return CLOUD_PROVIDER_LOCAL;
    if (strcasecmp(name, "aws") == 0)   return CLOUD_PROVIDER_AWS;
    if (strcasecmp(name, "aliyun") == 0) return CLOUD_PROVIDER_ALIYUN;
    if (strcasecmp(name, "tuya") == 0)  return CLOUD_PROVIDER_TUYA;
    if (strcasecmp(name, "local") == 0) return CLOUD_PROVIDER_LOCAL;
    if (strcasecmp(name, "none") == 0)  return CLOUD_PROVIDER_LOCAL;
    return CLOUD_PROVIDER_LOCAL;
}

/* ---------------------------------------------------------------------------
 * Load config.robot.yaml
 * --------------------------------------------------------------------------- */
static int load_config_from_yaml(const char *path, cloud_config_t *cfg) {
    FILE *fh = fopen(path, "r");
    if (!fh) {
        fprintf(stderr, "[CloudSelector] Cannot open %s\n", path);
        return -1;
    }

    yaml_parser_t parser;
    yaml_event_t  event;
    yaml_parser_initialize(&parser);
    yaml_parser_set_input_file(&parser, fh);

    memset(cfg, 0, sizeof(*cfg));
    cfg->provider = CLOUD_PROVIDER_LOCAL;
    cfg->backoff_max_sec = 60;
    cfg->telemetry_interval_sec = 30;

    /* Simple key-value parsing */
    int in_cloud = 0;
    char key[128] = {0};

    while (1) {
        if (!yaml_parser_parse(&parser, &event)) break;
        if (event.type == YAML_SCALAR_EVENT) {
            const char *val = (const char *)event.data.scalar.value;

            if (!in_cloud && strcmp(val, "cloud") == 0) {
                in_cloud = 1;
            } else if (in_cloud) {
                if (key[0] == '\0') {
                    strncpy(key, val, sizeof(key) - 1);
                } else {
                    if (strcmp(key, "provider") == 0) {
                        cfg->provider = parse_provider(val);
                    } else if (strcmp(key, "aws_thing_name") == 0) {
                        strncpy(cfg->aws.thing_name, val, sizeof(cfg->aws.thing_name) - 1);
                    } else if (strcmp(key, "aws_endpoint") == 0) {
                        strncpy(cfg->aws.endpoint, val, sizeof(cfg->aws.endpoint) - 1);
                    } else if (strcmp(key, "aws_root_ca") == 0) {
                        strncpy(cfg->aws.root_ca, val, sizeof(cfg->aws.root_ca) - 1);
                    } else if (strcmp(key, "aws_cert_path") == 0) {
                        strncpy(cfg->aws.cert_path, val, sizeof(cfg->aws.cert_path) - 1);
                    } else if (strcmp(key, "aws_key_path") == 0) {
                        strncpy(cfg->aws.key_path, val, sizeof(cfg->aws.key_path) - 1);
                    } else if (strcmp(key, "aliyun_product_key") == 0) {
                        strncpy(cfg->aliyun.product_key, val, sizeof(cfg->aliyun.product_key) - 1);
                    } else if (strcmp(key, "aliyun_device_name") == 0) {
                        strncpy(cfg->aliyun.device_name, val, sizeof(cfg->aliyun.device_name) - 1);
                    } else if (strcmp(key, "aliyun_device_secret") == 0) {
                        strncpy(cfg->aliyun.device_secret, val, sizeof(cfg->aliyun.device_secret) - 1);
                    } else if (strcmp(key, "tuya_uart_device") == 0) {
                        strncpy(cfg->tuya.uart_device, val, sizeof(cfg->tuya.uart_device) - 1);
                    } else if (strcmp(key, "telemetry_interval") == 0) {
                        cfg->telemetry_interval_sec = atoi(val);
                    } else if (strcmp(key, "backoff_max") == 0) {
                        cfg->backoff_max_sec = atoi(val);
                    }
                    key[0] = '\0';
                }
            }
        }
        if (event.type == YAML_STREAM_END_EVENT) break;
        yaml_event_delete(&event);
    }
    yaml_event_delete(&event);
    yaml_parser_delete(&parser);
    fclose(fh);

    return 0;
}

static void default_local_config(cloud_config_t *cfg) {
    memset(cfg, 0, sizeof(*cfg));
    cfg->provider = CLOUD_PROVIDER_LOCAL;
}

/* ---------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------- */
int cloud_selector_init(const char *config_path) {
    cloud_config_t cfg;
    const char *path = config_path ? config_path : CONFIG_PATH;

    if (load_config_from_yaml(path, &cfg) != 0) {
        fprintf(stdout, "[CloudSelector] No cloud config found, using local-only mode\n");
        default_local_config(&cfg);
    }

    pthread_mutex_lock(&g_lock);
    g_config = cfg;
    g_provider = cfg.provider;
    g_initialized = 1;
    pthread_mutex_unlock(&g_lock);

    const char *names[] = {
        [CLOUD_PROVIDER_AWS]    = "AWS IoT Core",
        [CLOUD_PROVIDER_ALIYUN] = "Alibaba Cloud IoT",
        [CLOUD_PROVIDER_TUYA]   = "Tuya Smart",
        [CLOUD_PROVIDER_LOCAL]  = "Local-only"
    };

    fprintf(stdout, "[CloudSelector] Selected cloud provider: %s\n",
            names[cfg.provider]);

    return 0;
}

cloud_provider_t cloud_selector_get_provider(void) {
    pthread_mutex_lock(&g_lock);
    cloud_provider_t p = g_provider;
    pthread_mutex_unlock(&g_lock);
    return p;
}

const cloud_config_t *cloud_selector_get_config(void) {
    return &g_config;
}

/* ---------------------------------------------------------------------------
 * Abstract interface
 * --------------------------------------------------------------------------- */
int cloud_connect(void) {
    if (!g_initialized) {
        fprintf(stderr, "[CloudSelector] Not initialized\n");
        return -1;
    }

    switch (g_provider) {
#ifdef CLOUD_AWS
    case CLOUD_PROVIDER_AWS:
        aws_iot_init(g_config.aws.thing_name, g_config.aws.endpoint,
                     g_config.aws.root_ca, g_config.aws.cert_path,
                     g_config.aws.key_path);
        return aws_iot_connect();
#endif
#ifdef CLOUD_ALIYUN
    case CLOUD_PROVIDER_ALIYUN:
        aliyun_iot_init(g_config.aliyun.product_key,
                        g_config.aliyun.device_name,
                        g_config.aliyun.device_secret);
        return aliyun_iot_connect();
#endif
#ifdef CLOUD_TUYA
    case CLOUD_PROVIDER_TUYA:
        tuya_iot_init();
        return tuya_iot_connect();
#endif
    case CLOUD_PROVIDER_LOCAL:
        fprintf(stdout, "[CloudSelector] Local mode, no connection needed\n");
        return 0;
    default:
        fprintf(stderr, "[CloudSelector] Unknown provider %d\n", g_provider);
        return -1;
    }
}

int cloud_disconnect(void) {
    if (!g_initialized) return -1;

    switch (g_provider) {
#ifdef CLOUD_AWS
    case CLOUD_PROVIDER_AWS: return aws_iot_disconnect();
#endif
#ifdef CLOUD_ALIYUN
    case CLOUD_PROVIDER_ALIYUN: return aliyun_iot_disconnect();
#endif
#ifdef CLOUD_TUYA
    case CLOUD_PROVIDER_TUYA: return tuya_iot_disconnect();
#endif
    case CLOUD_PROVIDER_LOCAL:
    default: return 0;
    }
}

int cloud_publish(const char *topic, const char *payload, size_t len) {
    if (!g_initialized) return -1;

    /* Also publish to telemetry batch */
    telemetry_batch_add(payload, len);

    switch (g_provider) {
#ifdef CLOUD_AWS
    case CLOUD_PROVIDER_AWS: return aws_iot_publish(topic, payload, len);
#endif
#ifdef CLOUD_ALIYUN
    case CLOUD_PROVIDER_ALIYUN: return aliyun_iot_publish(topic, payload, len);
#endif
#ifdef CLOUD_TUYA
    case CLOUD_PROVIDER_TUYA: return tuya_iot_report_status();
#endif
    case CLOUD_PROVIDER_LOCAL:
        /* In local mode, no-op */
        return 0;
    default: return -1;
    }
}

int cloud_subscribe(const char *topic,
                    void (*callback)(const char *, const char *, size_t)) {
    if (!g_initialized) return -1;

    switch (g_provider) {
#ifdef CLOUD_AWS
    case CLOUD_PROVIDER_AWS:
        return aws_iot_subscribe(topic, callback);
#endif
#ifdef CLOUD_ALIYUN
    case CLOUD_PROVIDER_ALIYUN:
        return aliyun_iot_subscribe(topic, callback);
#endif
#ifdef CLOUD_TUYA
    case CLOUD_PROVIDER_TUYA:
        /* Tuya uses DP callbacks, not MQTT topic subscriptions */
        return 0;
#endif
    case CLOUD_PROVIDER_LOCAL:
    default: return 0;
    }
}

/* ---------------------------------------------------------------------------
 * Factory provisioning support — write cloud config
 * --------------------------------------------------------------------------- */
int cloud_selector_write_config(const cloud_config_t *cfg, const char *path) {
    if (!cfg) return -1;
    const char *out_path = path ? path : CONFIG_PATH;

    FILE *fh = fopen(out_path, "w");
    if (!fh) {
        fprintf(stderr, "[CloudSelector] Cannot write %s\n", out_path);
        return -1;
    }

    fprintf(fh, "# Robot cloud configuration (auto-generated by factory provisioning)\n");
    fprintf(fh, "---\n");
    fprintf(fh, "cloud:\n");
    fprintf(fh, "  provider: ");
    switch (cfg->provider) {
    case CLOUD_PROVIDER_AWS:    fprintf(fh, "aws\n"); break;
    case CLOUD_PROVIDER_ALIYUN: fprintf(fh, "aliyun\n"); break;
    case CLOUD_PROVIDER_TUYA:   fprintf(fh, "tuya\n"); break;
    case CLOUD_PROVIDER_LOCAL:  fprintf(fh, "local\n"); break;
    default:                    fprintf(fh, "local\n"); break;
    }

    if (cfg->provider == CLOUD_PROVIDER_AWS) {
        fprintf(fh, "  aws_thing_name: %s\n", cfg->aws.thing_name);
        fprintf(fh, "  aws_endpoint: %s\n", cfg->aws.endpoint);
        fprintf(fh, "  aws_root_ca: %s\n", cfg->aws.root_ca);
        fprintf(fh, "  aws_cert_path: %s\n", cfg->aws.cert_path);
        fprintf(fh, "  aws_key_path: %s\n", cfg->aws.key_path);
    } else if (cfg->provider == CLOUD_PROVIDER_ALIYUN) {
        fprintf(fh, "  aliyun_product_key: %s\n", cfg->aliyun.product_key);
        fprintf(fh, "  aliyun_device_name: %s\n", cfg->aliyun.device_name);
        fprintf(fh, "  aliyun_device_secret: %s\n", cfg->aliyun.device_secret);
    } else if (cfg->provider == CLOUD_PROVIDER_TUYA) {
        fprintf(fh, "  tuya_uart_device: %s\n", cfg->tuya.uart_device);
    }

    fprintf(fh, "  telemetry_interval: %d\n", cfg->telemetry_interval_sec);
    fprintf(fh, "  backoff_max: %d\n", cfg->backoff_max_sec);

    fclose(fh);
    return 0;
}
