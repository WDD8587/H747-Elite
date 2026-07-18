/**
 * aliyun_iot.c — Alibaba Cloud IoT Platform client
 *
 * Device authentication via triple (productKey + deviceName + deviceSecret).
 * MQTT endpoint: ${productKey}.iot-as-mqtt.cn-shanghai.aliyuncs.com
 * Publish telemetry to /sys/${productKey}/${deviceName}/thing/event/property/post
 * Subscribe to service commands from Alibaba Cloud.
 * Supports dynamic register and HMAC-SHA1 MQTT password generation.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>
#include <errno.h>
#include <sys/socket.h>
#include <netdb.h>
#include <openssl/ssl.h>
#include <openssl/evp.h>
#include <openssl/hmac.h>
#include <openssl/bio.h>
#include <openssl/buffer.h>

#include "aliyun_iot.h"
#include "cloud_selector.h"

/* ---------------------------------------------------------------------------
 * Constants
 * --------------------------------------------------------------------------- */
#define ALIYUN_PORT               8883
#define ALIYUN_KEEPALIVE          120
#define ALIYUN_MAX_BACKOFF        60
#define ALIYUN_BASE_BACKOFF       1
#define ALIYUN_PAYLOAD_MAX        4096
#define ALIYUN_TOPIC_MAX          256
#define ALIYUN_HMAC_KEY_LEN       64
#define ALIYUN_SUB_SLOTS          16

/* ---------------------------------------------------------------------------
 * State machine
 * --------------------------------------------------------------------------- */
typedef enum {
    ALI_STATE_DISCONNECTED,
    ALI_STATE_CONNECTING,
    ALI_STATE_CONNECTED,
    ALI_STATE_RECONNECTING,
    ALI_STATE_DISCONNECTING
} ali_state_t;

typedef struct {
    char  product_key[64];
    char  device_name[64];
    char  device_secret[64];
    char  endpoint[256];
    char  client_id[128];
    char  username[128];
    char  password[128];

    int              sock_fd;
    SSL             *ssl;
    SSL_CTX         *ssl_ctx;
    ali_state_t      state;
    int              reconnect_attempt;
    int              keep_running;

    pthread_t        rx_thread;
    pthread_mutex_t  lock;

    /* Cached topics */
    char topic_post[ALIYUN_TOPIC_MAX];
    char topic_commands[ALIYUN_TOPIC_MAX];

    /* Subscriptions */
    struct {
        char     topic[ALIYUN_TOPIC_MAX];
        void   (*cb)(const char *topic, const char *payload, size_t len);
    } subs[ALIYUN_SUB_SLOTS];
    int sub_count;
} aliyun_ctx_t;

static aliyun_ctx_t g_ctx;

/* ---------------------------------------------------------------------------
 * Forward declarations
 * --------------------------------------------------------------------------- */
static int  generate_password_hmac(void);
static int  tls_connect(void);
static void tls_disconnect(void);
static int  mqtt_connect(void);
static int  mqtt_publish(const char *topic, const char *payload, size_t len);
static int  mqtt_subscribe(const char *topic);
static void rx_loop(void *arg);
static void mqtt_pingreq(void);
static void reconnect_task(void);

/* ---------------------------------------------------------------------------
 * Base64 encode for password token
 * --------------------------------------------------------------------------- */
static char *base64_encode(const unsigned char *input, int length) {
    BIO *bmem, *b64;
    BUF_MEM *bptr;

    b64 = BIO_new(BIO_f_base64());
    bmem = BIO_new(BIO_s_mem());
    b64 = BIO_push(b64, bmem);
    BIO_set_flags(b64, BIO_FLAGS_BASE64_NO_NL);
    BIO_write(b64, input, length);
    BIO_flush(b64);
    BIO_get_mem_ptr(b64, &bptr);

    char *result = (char *)malloc(bptr->length + 1);
    if (result) {
        memcpy(result, bptr->data, bptr->length);
        result[bptr->length] = '\0';
    }
    BIO_free_all(b64);
    return result;
}

/* ---------------------------------------------------------------------------
 * HMAC-SHA1 password generation
 * Alibaba Cloud MQTT password = HMAC-SHA1(clientId_seed, deviceSecret) in Base64
 * clientId = productKey.deviceName|timestamp|signmethod=hmacsha1|
 * --------------------------------------------------------------------------- */
static int generate_password_hmac(void) {
    aliyun_ctx_t *ctx = &g_ctx;
    char raw_key[ALIYUN_HMAC_KEY_LEN];

    /* Build the plaintext: productKey.deviceName */
    snprintf(raw_key, sizeof(raw_key), "%s.%s", ctx->product_key, ctx->device_name);

    unsigned char hmac_result[EVP_MAX_MD_SIZE];
    unsigned int  hmac_len = 0;

    HMAC(EVP_sha1(),
         ctx->device_secret, (int)strlen(ctx->device_secret),
         (const unsigned char *)raw_key, strlen(raw_key),
         hmac_result, &hmac_len);

    if (hmac_len == 0) return -1;

    char *b64 = base64_encode(hmac_result, (int)hmac_len);
    if (!b64) return -1;

    strncpy(ctx->password, b64, sizeof(ctx->password) - 1);
    free(b64);
    return 0;
}

/* ---------------------------------------------------------------------------
 * Build MQTT connect parameters
 * --------------------------------------------------------------------------- */
static int build_credentials(void) {
    aliyun_ctx_t *ctx = &g_ctx;

    /* Endpoint */
    snprintf(ctx->endpoint, sizeof(ctx->endpoint),
             "%s.iot-as-mqtt.cn-shanghai.aliyuncs.com", ctx->product_key);

    /* Client ID: productKey.deviceName|securemode=3,signmethod=hmacsha1,timestamp=123456| */
    time_t now = time(NULL);
    snprintf(ctx->client_id, sizeof(ctx->client_id),
             "%s.%s|securemode=3,signmethod=hmacsha1,timestamp=%ld|",
             ctx->product_key, ctx->device_name, (long)now);

    /* Username: deviceName&productKey */
    snprintf(ctx->username, sizeof(ctx->username),
             "%s&%s", ctx->device_name, ctx->product_key);

    /* Password */
    return generate_password_hmac();
}

/* ---------------------------------------------------------------------------
 * TLS helpers
 * --------------------------------------------------------------------------- */
static int tls_connect(void) {
    aliyun_ctx_t *ctx = &g_ctx;

    ctx->ssl_ctx = SSL_CTX_new(TLS_client_method());
    if (!ctx->ssl_ctx) return -1;

    SSL_CTX_set_min_proto_version(ctx->ssl_ctx, TLS1_2_VERSION);
    SSL_CTX_set_mode(ctx->ssl_ctx, SSL_MODE_AUTO_RETRY);

    /* Set default CA path (system) for Alibaba Cloud root CA */
    if (SSL_CTX_set_default_verify_paths(ctx->ssl_ctx) != 1) {
        fprintf(stderr, "[Aliyun] Failed to set default CA paths\n");
        return -1;
    }

    struct addrinfo hints, *res;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    char port_str[8];
    snprintf(port_str, sizeof(port_str), "%d", ALIYUN_PORT);

    if (getaddrinfo(ctx->endpoint, port_str, &hints, &res) != 0 || !res) {
        fprintf(stderr, "[Aliyun] DNS resolution failed\n");
        return -1;
    }

    ctx->sock_fd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (ctx->sock_fd < 0) { freeaddrinfo(res); return -1; }

    struct timeval tv = { .tv_sec = 10, .tv_usec = 0 };
    setsockopt(ctx->sock_fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    setsockopt(ctx->sock_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    if (connect(ctx->sock_fd, res->ai_addr, res->ai_addrlen) < 0) {
        freeaddrinfo(res); return -1;
    }
    freeaddrinfo(res);

    ctx->ssl = SSL_new(ctx->ssl_ctx);
    if (!ctx->ssl) return -1;

    SSL_set_fd(ctx->ssl, ctx->sock_fd);
    SSL_set_connect_state(ctx->ssl);

    if (SSL_connect(ctx->ssl) != 1) {
        fprintf(stderr, "[Aliyun] TLS handshake failed\n");
        return -1;
    }

    fprintf(stdout, "[Aliyun] TLS connected to %s\n", ctx->endpoint);
    return 0;
}

static void tls_disconnect(void) {
    aliyun_ctx_t *ctx = &g_ctx;
    if (ctx->ssl) { SSL_shutdown(ctx->ssl); SSL_free(ctx->ssl); ctx->ssl = NULL; }
    if (ctx->ssl_ctx) { SSL_CTX_free(ctx->ssl_ctx); ctx->ssl_ctx = NULL; }
    if (ctx->sock_fd >= 0) { close(ctx->sock_fd); ctx->sock_fd = -1; }
}

/* ---------------------------------------------------------------------------
 * MQTT helpers
 * --------------------------------------------------------------------------- */
static int mqtx_write_all(const uint8_t *buf, size_t len) {
    aliyun_ctx_t *ctx = &g_ctx;
    size_t written = 0;
    while (written < len) {
        int ret = SSL_write(ctx->ssl, buf + written, (int)(len - written));
        if (ret <= 0) return -1;
        written += (size_t)ret;
    }
    return 0;
}

static int mqtx_read_all(uint8_t *buf, size_t len) {
    aliyun_ctx_t *ctx = &g_ctx;
    size_t read_bytes = 0;
    while (read_bytes < len) {
        int ret = SSL_read(ctx->ssl, buf + read_bytes, (int)(len - read_bytes));
        if (ret <= 0) {
            int err = SSL_get_error(ctx->ssl, ret);
            if (err == SSL_ERROR_WANT_READ || err == SSL_ERROR_WANT_WRITE)
                continue;
            return -1;
        }
        read_bytes += (size_t)ret;
    }
    return 0;
}

static uint16_t mqtt_enc_remlen(uint8_t *buf, uint32_t len) {
    uint16_t idx = 0;
    do {
        buf[idx] = len % 128;
        len /= 128;
        if (len > 0) buf[idx] |= 0x80;
        idx++;
    } while (len > 0);
    return idx;
}

static int mqtt_connect(void) {
    aliyun_ctx_t *ctx = &g_ctx;
    uint8_t packet[512];
    uint16_t pos = 0;

    packet[pos++] = 0x10; /* CONNECT */

    uint16_t len_pos = pos;
    pos += 1; /* remaining length placeholder (<128) */

    /* Protocol name: MQTT 3.1.1 */
    memcpy(&packet[pos], "\x00\x04MQTT", 6); pos += 6;
    packet[pos++] = 0x04; /* level */
    packet[pos++] = 0xC2; /* flags: Clean Session + Username + Password */
    uint16_t keepalive = ALIYUN_KEEPALIVE;
    packet[pos++] = (keepalive >> 8) & 0xFF;
    packet[pos++] = keepalive & 0xFF;

    /* Client ID */
    uint16_t cid_len = (uint16_t)strlen(ctx->client_id);
    packet[pos++] = (cid_len >> 8) & 0xFF;
    packet[pos++] = cid_len & 0xFF;
    memcpy(&packet[pos], ctx->client_id, cid_len); pos += cid_len;

    /* Username */
    uint16_t un_len = (uint16_t)strlen(ctx->username);
    packet[pos++] = (un_len >> 8) & 0xFF;
    packet[pos++] = un_len & 0xFF;
    memcpy(&packet[pos], ctx->username, un_len); pos += un_len;

    /* Password */
    uint16_t pw_len = (uint16_t)strlen(ctx->password);
    packet[pos++] = (pw_len >> 8) & 0xFF;
    packet[pos++] = pw_len & 0xFF;
    memcpy(&packet[pos], ctx->password, pw_len); pos += pw_len;

    /* Remaining length */
    uint32_t remaining = pos - len_pos - 1;
    packet[len_pos] = remaining & 0x7F;

    if (mqtx_write_all(packet, pos) < 0) return -1;

    /* CONNACK */
    uint8_t connack[4];
    if (mqtx_read_all(connack, 4) < 0) return -1;
    if (connack[0] != 0x20 || connack[3] != 0x00) {
        fprintf(stderr, "[Aliyun] CONNACK error: %02x\n", connack[3]);
        return -1;
    }

    fprintf(stdout, "[Aliyun] MQTT connected (client: %s)\n", ctx->client_id);
    return 0;
}

static int mqtt_publish(const char *topic, const char *payload, size_t len) {
    aliyun_ctx_t *ctx = &g_ctx;
    uint8_t header[512];
    uint16_t pos = 0;
    uint16_t tlen = (uint16_t)strlen(topic);
    uint32_t remaining = 2 + tlen + (uint32_t)len;
    uint8_t rem_enc[4];
    uint16_t rel = mqtt_enc_remlen(rem_enc, remaining);

    header[pos++] = 0x30; /* PUBLISH QoS=0 */
    memcpy(&header[pos], rem_enc, rel); pos += rel;
    header[pos++] = (tlen >> 8) & 0xFF;
    header[pos++] = tlen & 0xFF;
    memcpy(&header[pos], topic, tlen); pos += tlen;

    if (mqtx_write_all(header, pos) < 0) return -1;
    if (mqtx_write_all((const uint8_t *)payload, len) < 0) return -1;
    return 0;
}

static int mqtt_subscribe(const char *topic) {
    aliyun_ctx_t *ctx = &g_ctx;
    uint8_t packet[512];
    uint16_t pos = 0;

    uint16_t tlen = (uint16_t)strlen(topic);
    uint32_t remaining = 2 + 2 + tlen + 1;
    uint8_t rem_enc[4];
    uint16_t rel = mqtt_enc_remlen(rem_enc, remaining);

    packet[pos++] = 0x82;
    memcpy(&packet[pos], rem_enc, rel); pos += rel;

    static uint16_t pkt_id = 1;
    packet[pos++] = (pkt_id >> 8) & 0xFF;
    packet[pos++] = pkt_id & 0xFF;
    pkt_id++;

    packet[pos++] = (tlen >> 8) & 0xFF;
    packet[pos++] = tlen & 0xFF;
    memcpy(&packet[pos], topic, tlen); pos += tlen;
    packet[pos++] = 0x01; /* QoS 1 */

    if (mqtx_write_all(packet, pos) < 0) return -1;

    uint8_t suback[5];
    if (mqtx_read_all(suback, 5) < 0) return -1;
    if ((suback[0] & 0xF0) != 0x90 || suback[4] == 0x80) return -1;

    fprintf(stdout, "[Aliyun] Subscribed to %s\n", topic);
    return 0;
}

static void mqtt_pingreq(void) {
    uint8_t ping[2] = { 0xC0, 0x00 };
    (void)mqtx_write_all(ping, 2);
}

/* ---------------------------------------------------------------------------
 * RX loop
 * --------------------------------------------------------------------------- */
static void rx_loop(void *arg) {
    (void)arg;
    aliyun_ctx_t *ctx = &g_ctx;
    uint8_t buf[ALIYUN_PAYLOAD_MAX + 256];

    while (ctx->keep_running && ctx->state == ALI_STATE_CONNECTED) {
        /* Read first byte */
        int ret = SSL_read(ctx->ssl, buf, 1);
        if (ret <= 0) {
            int err = SSL_get_error(ctx->ssl, ret);
            if (err == SSL_ERROR_WANT_READ || err == SSL_ERROR_WANT_WRITE) {
                usleep(10000); continue;
            }
            goto reconnect;
        }

        uint8_t ptype = buf[0] & 0xF0;

        /* Remaining length */
        uint32_t remaining = 0;
        uint8_t mult = 1;
        int enc_pos = 0;
        do {
            ret = SSL_read(ctx->ssl, buf, 1);
            if (ret <= 0) goto reconnect;
            remaining += (buf[0] & 0x7F) * mult;
            mult *= 128;
            enc_pos++;
        } while ((buf[0] & 0x80) && enc_pos < 4);

        size_t total = remaining + 1 + enc_pos;
        if (total > sizeof(buf)) goto reconnect;

        /* Read remaining body */
        size_t have = 1 + enc_pos;
        while (have < total) {
            ret = SSL_read(ctx->ssl, buf + have, (int)(total - have));
            if (ret <= 0) goto reconnect;
            have += (size_t)ret;
        }

        switch (ptype) {
        case 0xD0: /* PINGRESP */ break;
        case 0x50: /* PUBLISH */
        {
            size_t rh = 0;
            do { rh++; } while ((buf[rh] & 0x80) && rh < 5);
            uint16_t tlen = ((uint16_t)buf[rh] << 8) | buf[rh + 1];
            char topic[ALIYUN_TOPIC_MAX];
            if (tlen >= sizeof(topic)) tlen = (uint16_t)(sizeof(topic) - 1);
            memcpy(topic, &buf[rh + 2], tlen);
            topic[tlen] = '\0';
            size_t pl_start = rh + 2 + tlen;
            size_t pl_len = total - pl_start;
            const char *payload = (const char *)&buf[pl_start];

            pthread_mutex_lock(&ctx->lock);
            for (int i = 0; i < ctx->sub_count; i++) {
                if (strcmp(ctx->subs[i].topic, topic) == 0 && ctx->subs[i].cb)
                    ctx->subs[i].cb(topic, payload, pl_len);
            }
            pthread_mutex_unlock(&ctx->lock);
            break;
        }
        case 0x48: case 0x40: default: break;
        }
        continue;

reconnect:
        pthread_mutex_lock(&ctx->lock);
        if (ctx->state == ALI_STATE_CONNECTED)
            ctx->state = ALI_STATE_RECONNECTING;
        pthread_mutex_unlock(&ctx->lock);
        break;
    }
}

/* ---------------------------------------------------------------------------
 * Reconnect
 * --------------------------------------------------------------------------- */
static void reconnect_task(void) {
    aliyun_ctx_t *ctx = &g_ctx;
    while (ctx->keep_running &&
           (ctx->state == ALI_STATE_RECONNECTING || ctx->state == ALI_STATE_DISCONNECTED)) {
        int delay = ALIYUN_BASE_BACKOFF;
        for (int i = 0; i < ctx->reconnect_attempt; i++) {
            delay *= 2;
            if (delay > ALIYUN_MAX_BACKOFF) { delay = ALIYUN_MAX_BACKOFF; break; }
        }
        fprintf(stdout, "[Aliyun] Reconnect in %ds (attempt %d)\n",
                delay, ctx->reconnect_attempt + 1);
        for (int i = 0; i < delay && ctx->keep_running; i++) sleep(1);
        if (!ctx->keep_running) break;

        ctx->state = ALI_STATE_CONNECTING;
        if (tls_connect() == 0 && mqtt_connect() == 0) {
            pthread_mutex_lock(&ctx->lock);
            for (int i = 0; i < ctx->sub_count; i++)
                mqtt_subscribe(ctx->subs[i].topic);
            ctx->reconnect_attempt = 0;
            ctx->state = ALI_STATE_CONNECTED;
            pthread_mutex_unlock(&ctx->lock);
            fprintf(stdout, "[Aliyun] Reconnected\n");
            pthread_create(&ctx->rx_thread, NULL,
                           (void *(*)(void *))rx_loop, NULL);
            break;
        } else {
            tls_disconnect();
            ctx->reconnect_attempt++;
            ctx->state = ALI_STATE_RECONNECTING;
        }
    }
}

/* ---------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------- */
int aliyun_iot_init(const char *product_key, const char *device_name,
                    const char *device_secret) {
    aliyun_ctx_t *ctx = &g_ctx;
    memset(ctx, 0, sizeof(*ctx));

    strncpy(ctx->product_key, product_key, sizeof(ctx->product_key) - 1);
    strncpy(ctx->device_name, device_name, sizeof(ctx->device_name) - 1);
    strncpy(ctx->device_secret, device_secret, sizeof(ctx->device_secret) - 1);
    ctx->sock_fd = -1;
    ctx->keep_running = 1;
    ctx->state = ALI_STATE_DISCONNECTED;
    pthread_mutex_init(&ctx->lock, NULL);

    if (build_credentials() != 0)
        return -1;

    /* Build telemetry topic */
    snprintf(ctx->topic_post, sizeof(ctx->topic_post),
             "/sys/%s/%s/thing/event/property/post",
             ctx->product_key, ctx->device_name);
    snprintf(ctx->topic_commands, sizeof(ctx->topic_commands),
             "/sys/%s/%s/thing/service/*",
             ctx->product_key, ctx->device_name);

    return 0;
}

int aliyun_iot_connect(void) {
    aliyun_ctx_t *ctx = &g_ctx;
    ctx->state = ALI_STATE_CONNECTING;

    if (tls_connect() != 0 || mqtt_connect() != 0) {
        tls_disconnect();
        ctx->state = ALI_STATE_RECONNECTING;
        pthread_t rt;
        pthread_create(&rt, NULL, (void *(*)(void *))reconnect_task, NULL);
        pthread_detach(rt);
        return -1;
    }

    /* Subscribe to service commands */
    mqtt_subscribe(ctx->topic_commands);

    ctx->reconnect_attempt = 0;
    ctx->state = ALI_STATE_CONNECTED;
    pthread_create(&ctx->rx_thread, NULL,
                   (void *(*)(void *))rx_loop, NULL);
    return 0;
}

int aliyun_iot_disconnect(void) {
    aliyun_ctx_t *ctx = &g_ctx;
    ctx->keep_running = 0;
    ctx->state = ALI_STATE_DISCONNECTING;
    uint8_t disc[2] = { 0xE0, 0x00 };
    (void)mqtx_write_all(disc, 2);
    tls_disconnect();
    ctx->state = ALI_STATE_DISCONNECTED;
    return 0;
}

int aliyun_iot_report_property(const char *property_json) {
    aliyun_ctx_t *ctx = &g_ctx;
    return mqtt_publish(ctx->topic_post, property_json, strlen(property_json));
}

int aliyun_iot_publish(const char *topic, const char *payload, size_t len) {
    return mqtt_publish(topic, payload, len);
}

int aliyun_iot_subscribe(const char *topic,
                         void (*callback)(const char *, const char *, size_t)) {
    aliyun_ctx_t *ctx = &g_ctx;
    pthread_mutex_lock(&ctx->lock);
    if (ctx->sub_count >= ALIYUN_SUB_SLOTS) {
        pthread_mutex_unlock(&ctx->lock);
        return -1;
    }
    strncpy(ctx->subs[ctx->sub_count].topic, topic,
            sizeof(ctx->subs[0].topic) - 1);
    ctx->subs[ctx->sub_count].cb = callback;
    ctx->sub_count++;
    pthread_mutex_unlock(&ctx->lock);

    if (ctx->state == ALI_STATE_CONNECTED)
        return mqtt_subscribe(topic);
    return 0;
}

void aliyun_iot_keepalive(void) {
    mqtt_pingreq();
}

const char *aliyun_iot_get_topic_post(void) {
    return g_ctx.topic_post;
}
