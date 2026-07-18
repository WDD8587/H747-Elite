/**
 * aws_iot.c — AWS IoT Core MQTT client
 *
 * X.509 certificate authentication. TLS 1.2 via OpenSSL.
 * Device shadow topics: $aws/things/robot_XXXX/shadow/update,
 * $aws/things/robot_XXXX/shadow/get/accepted, $aws/things/robot_XXXX/shadow/delta.
 * Reconnect with exponential backoff (1s, 2s, 4s, ..., max 60s).
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
#include <openssl/err.h>
#include <openssl/x509.h>
#include <openssl/pem.h>

#include "aws_iot.h"
#include "cloud_selector.h"
#include "device_shadow.h"
#include "telemetry_batch.h"

/* ---------------------------------------------------------------------------
 * Constants
 * --------------------------------------------------------------------------- */
#define AWS_IOT_PORT             8883
#define AWS_IOT_KEEPALIVE        60
#define AWS_IOT_MAX_BACKOFF      60           /* seconds */
#define AWS_IOT_BASE_BACKOFF     1            /* seconds */
#define AWS_IOT_THING_PREFIX     "robot_"
#define AWS_IOT_SHADOW_UPDATE    "$aws/things/%s/shadow/update"
#define AWS_IOT_SHADOW_GET       "$aws/things/%s/shadow/get"
#define AWS_IOT_SHADOW_GET_ACC   "$aws/things/%s/shadow/get/accepted"
#define AWS_IOT_SHADOW_DELTA     "$aws/things/%s/shadow/delta"
#define AWS_IOT_TOPIC_MAX        256
#define AWS_IOT_PAYLOAD_MAX      4096

/* ---------------------------------------------------------------------------
 * Internal state
 * --------------------------------------------------------------------------- */
typedef enum {
    AWS_STATE_DISCONNECTED,
    AWS_STATE_CONNECTING,
    AWS_STATE_CONNECTED,
    AWS_STATE_RECONNECTING,
    AWS_STATE_DISCONNECTING
} aws_state_t;

typedef struct {
    char             thing_name[64];
    char             endpoint[256];
    char             root_ca_path[512];
    char             cert_path[512];
    char             key_path[512];

    int              sock_fd;
    SSL             *ssl;
    SSL_CTX         *ssl_ctx;
    aws_state_t      state;

    int              reconnect_attempt;
    int              keep_running;

    pthread_t        rx_thread;
    pthread_mutex_t  lock;

    /* shadow topics (cached) */
    char             topic_update[AWS_IOT_TOPIC_MAX];
    char             topic_get[AWS_IOT_TOPIC_MAX];
    char             topic_get_acc[AWS_IOT_TOPIC_MAX];
    char             topic_delta[AWS_IOT_TOPIC_MAX];

    /* registered callbacks */
    struct {
        char     topic[AWS_IOT_TOPIC_MAX];
        void   (*cb)(const char *topic, const char *payload, size_t len);
    } subs[16];
    int              sub_count;
} aws_iot_ctx_t;

static aws_iot_ctx_t g_ctx;

/* ---------------------------------------------------------------------------
 * Forward declarations
 * --------------------------------------------------------------------------- */
static int  tls_connect(void);
static void tls_disconnect(void);
static int  mqtt_connect(void);
static int  mqtt_publish(const char *topic, const char *payload, size_t len);
static int  mqtt_subscribe(const char *topic);
static int  mqtt_read_packet(uint8_t *buf, size_t cap, size_t *out_len);
static void rx_loop(void *arg);
static void mqtt_pingreq(void);
static void reconnect_task(void);

/* ---------------------------------------------------------------------------
 * Utility
 * --------------------------------------------------------------------------- */
static void build_topics(aws_iot_ctx_t *ctx) {
    snprintf(ctx->topic_update, sizeof(ctx->topic_update),
             AWS_IOT_SHADOW_UPDATE, ctx->thing_name);
    snprintf(ctx->topic_get, sizeof(ctx->topic_get),
             AWS_IOT_SHADOW_GET, ctx->thing_name);
    snprintf(ctx->topic_get_acc, sizeof(ctx->topic_get_acc),
             AWS_IOT_SHADOW_GET_ACC, ctx->thing_name);
    snprintf(ctx->topic_delta, sizeof(ctx->topic_delta),
             AWS_IOT_SHADOW_DELTA, ctx->thing_name);
}

static uint16_t mqtt_encode_remaining_length(uint8_t *buf, uint32_t len) {
    uint16_t idx = 0;
    do {
        buf[idx] = (len % 128);
        len /= 128;
        if (len > 0) buf[idx] |= 0x80;
        idx++;
    } while (len > 0);
    return idx;
}

/* ---------------------------------------------------------------------------
 * TLS helpers
 * --------------------------------------------------------------------------- */
static int tls_connect(void) {
    aws_iot_ctx_t *ctx = &g_ctx;
    int ret;

    /* Create SSL context */
    ctx->ssl_ctx = SSL_CTX_new(TLS_client_method());
    if (!ctx->ssl_ctx) {
        fprintf(stderr, "[AWS] SSL_CTX_new failed\n");
        return -1;
    }

    SSL_CTX_set_min_proto_version(ctx->ssl_ctx, TLS1_2_VERSION);
    SSL_CTX_set_mode(ctx->ssl_ctx, SSL_MODE_AUTO_RETRY);

    /* Load CA certificate */
    if (SSL_CTX_load_verify_locations(ctx->ssl_ctx, ctx->root_ca_path, NULL) != 1) {
        fprintf(stderr, "[AWS] Failed to load CA cert: %s\n", ctx->root_ca_path);
        ERR_print_errors_fp(stderr);
        return -1;
    }

    /* Load client certificate */
    if (SSL_CTX_use_certificate_file(ctx->ssl_ctx, ctx->cert_path, SSL_FILETYPE_PEM) != 1) {
        fprintf(stderr, "[AWS] Failed to load client cert: %s\n", ctx->cert_path);
        ERR_print_errors_fp(stderr);
        return -1;
    }

    /* Load private key */
    if (SSL_CTX_use_PrivateKey_file(ctx->ssl_ctx, ctx->key_path, SSL_FILETYPE_PEM) != 1) {
        fprintf(stderr, "[AWS] Failed to load private key: %s\n", ctx->key_path);
        ERR_print_errors_fp(stderr);
        return -1;
    }

    /* Verify private key matches certificate */
    if (SSL_CTX_check_private_key(ctx->ssl_ctx) != 1) {
        fprintf(stderr, "[AWS] Private key does not match certificate\n");
        return -1;
    }

    /* Resolve endpoint */
    struct addrinfo hints, *res;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    char port_str[8];
    snprintf(port_str, sizeof(port_str), "%d", AWS_IOT_PORT);

    ret = getaddrinfo(ctx->endpoint, port_str, &hints, &res);
    if (ret != 0 || !res) {
        fprintf(stderr, "[AWS] getaddrinfo failed: %s\n", gai_strerror(ret));
        return -1;
    }

    /* Create socket */
    ctx->sock_fd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (ctx->sock_fd < 0) {
        perror("[AWS] socket");
        freeaddrinfo(res);
        return -1;
    }

    /* Connect */
    struct timeval tv = { .tv_sec = 10, .tv_usec = 0 };
    setsockopt(ctx->sock_fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    setsockopt(ctx->sock_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    ret = connect(ctx->sock_fd, res->ai_addr, res->ai_addrlen);
    freeaddrinfo(res);
    if (ret < 0) {
        perror("[AWS] connect");
        return -1;
    }

    /* Create SSL object and perform handshake */
    ctx->ssl = SSL_new(ctx->ssl_ctx);
    if (!ctx->ssl) {
        fprintf(stderr, "[AWS] SSL_new failed\n");
        return -1;
    }

    SSL_set_fd(ctx->ssl, ctx->sock_fd);
    SSL_set_connect_state(ctx->ssl);

    ret = SSL_connect(ctx->ssl);
    if (ret != 1) {
        fprintf(stderr, "[AWS] SSL_connect failed: %d\n", SSL_get_error(ctx->ssl, ret));
        ERR_print_errors_fp(stderr);
        return -1;
    }

    fprintf(stdout, "[AWS] TLS connected to %s\n", ctx->endpoint);
    return 0;
}

static void tls_disconnect(void) {
    aws_iot_ctx_t *ctx = &g_ctx;
    if (ctx->ssl) {
        SSL_shutdown(ctx->ssl);
        SSL_free(ctx->ssl);
        ctx->ssl = NULL;
    }
    if (ctx->ssl_ctx) {
        SSL_CTX_free(ctx->ssl_ctx);
        ctx->ssl_ctx = NULL;
    }
    if (ctx->sock_fd >= 0) {
        close(ctx->sock_fd);
        ctx->sock_fd = -1;
    }
}

/* ---------------------------------------------------------------------------
 * MQTT helpers
 * --------------------------------------------------------------------------- */
static int mqtt_write_all(const uint8_t *buf, size_t len) {
    aws_iot_ctx_t *ctx = &g_ctx;
    size_t written = 0;
    while (written < len) {
        int ret = SSL_write(ctx->ssl, buf + written, (int)(len - written));
        if (ret <= 0) {
            fprintf(stderr, "[AWS] SSL_write error: %d\n", SSL_get_error(ctx->ssl, ret));
            return -1;
        }
        written += (size_t)ret;
    }
    return 0;
}

static int mqtt_read_all(uint8_t *buf, size_t len) {
    aws_iot_ctx_t *ctx = &g_ctx;
    size_t read_bytes = 0;
    while (read_bytes < len) {
        int ret = SSL_read(ctx->ssl, buf + read_bytes, (int)(len - read_bytes));
        if (ret <= 0) {
            int err = SSL_get_error(ctx->ssl, ret);
            if (err == SSL_ERROR_WANT_READ || err == SSL_ERROR_WANT_WRITE)
                continue;
            fprintf(stderr, "[AWS] SSL_read error: %d\n", err);
            return -1;
        }
        read_bytes += (size_t)ret;
    }
    return 0;
}

static int mqtt_connect(void) {
    aws_iot_ctx_t *ctx = &g_ctx;
    uint8_t packet[256];
    uint16_t pos = 0;

    /* Fixed header: CONNECT (0x10) */
    packet[pos++] = 0x10;

    /* Variable header + payload length placeholder */
    uint8_t len_enc[4];
    uint16_t len_pos = pos;
    pos += 1; /* placeholder for remaining length, assume <128 */

    /* Protocol name */
    const char proto_name[] = "\x00\x04MQTT";
    memcpy(&packet[pos], proto_name, 6); pos += 6;

    /* Protocol level (MQTT 3.1.1 = 4) */
    packet[pos++] = 0x04;

    /* Connect flags: Clean Session = 1, Will = 0, Username = 0, Password = 0 */
    packet[pos++] = 0x02;

    /* Keepalive (big-endian) */
    uint16_t keepalive = AWS_IOT_KEEPALIVE;
    packet[pos++] = (keepalive >> 8) & 0xFF;
    packet[pos++] = keepalive & 0xFF;

    /* Client ID = thing_name */
    uint16_t client_id_len = (uint16_t)strlen(ctx->thing_name);
    packet[pos++] = (client_id_len >> 8) & 0xFF;
    packet[pos++] = client_id_len & 0xFF;
    memcpy(&packet[pos], ctx->thing_name, client_id_len); pos += client_id_len;

    /* Fill in remaining length */
    uint32_t remaining = pos - len_pos - 1;
    len_enc[0] = remaining & 0x7F;
    packet[len_pos] = len_enc[0];

    /* Send CONNECT */
    if (mqtt_write_all(packet, pos) < 0)
        return -1;

    /* Read CONNACK */
    uint8_t connack[4];
    if (mqtt_read_all(connack, 4) < 0)
        return -1;

    if (connack[0] != 0x20 || connack[3] != 0x00) {
        fprintf(stderr, "[AWS] CONNACK rejected, reason=%02x\n", connack[3]);
        return -1;
    }

    fprintf(stdout, "[AWS] MQTT connected as %s\n", ctx->thing_name);
    return 0;
}

static int mqtt_publish(const char *topic, const char *payload, size_t len) {
    aws_iot_ctx_t *ctx = &g_ctx;
    uint8_t header[256];
    uint16_t pos = 0;

    uint16_t topic_len = (uint16_t)strlen(topic);
    uint32_t remaining = (uint32_t)(2 + topic_len + len);
    uint8_t remaining_enc[4];
    uint16_t rem_enc_len;

    /* PUBLISH packet (0x30, QoS=0) */
    header[pos++] = 0x30;
    rem_enc_len = mqtt_encode_remaining_length(remaining_enc, remaining);
    memcpy(&header[pos], remaining_enc, rem_enc_len); pos += rem_enc_len;

    /* Topic */
    header[pos++] = (topic_len >> 8) & 0xFF;
    header[pos++] = topic_len & 0xFF;
    memcpy(&header[pos], topic, topic_len); pos += topic_len;

    if (mqtt_write_all(header, pos) < 0)
        return -1;
    if (mqtt_write_all((const uint8_t *)payload, len) < 0)
        return -1;

    return 0;
}

static int mqtt_subscribe(const char *topic) {
    aws_iot_ctx_t *ctx = &g_ctx;
    uint8_t packet[512];
    uint16_t pos = 0;

    uint16_t topic_len = (uint16_t)strlen(topic);
    uint32_t remaining = (uint32_t)(2 + 2 + topic_len + 1); /* packet_id + topic_len + topic + QoS */
    uint8_t remaining_enc[4];
    uint16_t rem_enc_len;

    /* SUBSCRIBE (0x82) */
    packet[pos++] = 0x82;
    rem_enc_len = mqtt_encode_remaining_length(remaining_enc, remaining);
    memcpy(&packet[pos], remaining_enc, rem_enc_len); pos += rem_enc_len;

    /* Packet ID */
    static uint16_t packet_id = 1;
    packet[pos++] = (packet_id >> 8) & 0xFF;
    packet[pos++] = packet_id & 0xFF;
    packet_id++;

    /* Topic filter */
    packet[pos++] = (topic_len >> 8) & 0xFF;
    packet[pos++] = topic_len & 0xFF;
    memcpy(&packet[pos], topic, topic_len); pos += topic_len;

    /* QoS = 1 */
    packet[pos++] = 0x01;

    if (mqtt_write_all(packet, pos) < 0)
        return -1;

    /* Read SUBACK */
    uint8_t suback[5];
    if (mqtt_read_all(suback, 5) < 0)
        return -1;

    if ((suback[0] & 0xF0) != 0x90) {
        fprintf(stderr, "[AWS] SUBACK unexpected header 0x%02x\n", suback[0]);
        return -1;
    }

    uint8_t return_code = suback[4];
    if (return_code == 0x80) {
        fprintf(stderr, "[AWS] SUBACK failure for topic %s\n", topic);
        return -1;
    }

    fprintf(stdout, "[AWS] Subscribed to %s\n", topic);
    return 0;
}

static void mqtt_pingreq(void) {
    uint8_t pingreq[2] = { 0xC0, 0x00 };
    (void)mqtt_write_all(pingreq, 2);
}

/* ---------------------------------------------------------------------------
 * MQTT receive — parse one inbound packet
 * --------------------------------------------------------------------------- */
static int mqtt_read_packet(uint8_t *buf, size_t cap, size_t *out_len) {
    int ret;

    /* Read fixed header first byte */
    ret = SSL_read(g_ctx.ssl, buf, 1);
    if (ret <= 0) {
        int err = SSL_get_error(g_ctx.ssl, ret);
        if (err == SSL_ERROR_WANT_READ || err == SSL_ERROR_WANT_WRITE)
            return 0; /* no data yet */
        return -1;
    }

    uint8_t packet_type = buf[0] & 0xF0;

    /* Read remaining length (variable-length encoding) */
    uint32_t remaining = 0;
    uint8_t enc;
    uint8_t multiplier = 1;
    int enc_pos = 0;
    do {
        ret = SSL_read(g_ctx.ssl, &enc, 1);
        if (ret <= 0) return -1;
        remaining += (enc & 0x7F) * multiplier;
        multiplier *= 128;
        enc_pos++;
    } while ((enc & 0x80) && enc_pos < 4);

    /* Skip variable header size for SUBACK/UNSUBACK — consume remaining */
    if (remaining + 1 + enc_pos > cap) {
        fprintf(stderr, "[AWS] Packet too large (%u bytes)\n", remaining + 1 + enc_pos);
        return -1;
    }

    /* Copy the first byte into buf */
    buf[0] = packet_type;
    size_t idx = 1 + enc_pos;
    size_t to_read = remaining;
    while (to_read > 0) {
        ret = SSL_read(g_ctx.ssl, buf + idx, (int)to_read);
        if (ret <= 0) return -1;
        to_read -= (size_t)ret;
        idx += (size_t)ret;
    }

    *out_len = idx;
    return (int)packet_type;
}

/* ---------------------------------------------------------------------------
 * Receive loop — background thread
 * --------------------------------------------------------------------------- */
static void rx_loop(void *arg) {
    (void)arg;
    aws_iot_ctx_t *ctx = &g_ctx;
    uint8_t buf[AWS_IOT_PAYLOAD_MAX + 256];

    while (ctx->keep_running && ctx->state == AWS_STATE_CONNECTED) {
        size_t pkt_len = 0;
        int ptype = mqtt_read_packet(buf, sizeof(buf), &pkt_len);
        if (ptype < 0) {
            fprintf(stderr, "[AWS] RX error, triggering reconnect\n");
            pthread_mutex_lock(&ctx->lock);
            if (ctx->state == AWS_STATE_CONNECTED)
                ctx->state = AWS_STATE_RECONNECTING;
            pthread_mutex_unlock(&ctx->lock);
            break;
        }

        if (ptype == 0) {
            /* No data yet */
            usleep(10000);
            continue;
        }

        switch (ptype) {
        case 0xD0: /* PINGRESP */
            break;

        case 0x50: /* PUBLISH */
        {
            /* Parse topic from variable header */
            size_t idx = 1; /* skip past remaining length (already handled) */
            /* find actual variable header start — we need to account for remaining length encoding
               but buf has raw bytes from packet start; let's re-derive */
            /* Instead, use pkt_len and parse from known structure.
               After fixed header, remaining length bytes follow, then variable header. */

            /* Quick re-parse: find where remaining length ends */
            size_t rh_len = 0;
            do {
                rh_len++;
            } while ((buf[rh_len] & 0x80) && rh_len < 5);

            uint16_t tlen = ((uint16_t)buf[rh_len] << 8) | buf[rh_len + 1];
            size_t payload_start = rh_len + 2 + tlen;

            char topic[AWS_IOT_TOPIC_MAX];
            if (tlen >= sizeof(topic)) tlen = (uint16_t)(sizeof(topic) - 1);
            memcpy(topic, &buf[rh_len + 2], tlen);
            topic[tlen] = '\0';

            size_t payload_len = pkt_len - payload_start;
            const char *payload = (const char *)&buf[payload_start];

            /* Dispatch to registered callbacks */
            pthread_mutex_lock(&ctx->lock);
            for (int i = 0; i < ctx->sub_count; i++) {
                if (strcmp(ctx->subs[i].topic, topic) == 0) {
                    if (ctx->subs[i].cb)
                        ctx->subs[i].cb(topic, payload, payload_len);
                }
            }
            pthread_mutex_unlock(&ctx->lock);
            break;
        }

        case 0x48: /* UNSUBACK */
        case 0x40: /* PUBACK for QoS 1 */
        default:
            break;
        }
    }
}

/* ---------------------------------------------------------------------------
 * Reconnect with exponential backoff
 * --------------------------------------------------------------------------- */
static void reconnect_task(void) {
    aws_iot_ctx_t *ctx = &g_ctx;

    while (ctx->keep_running &&
           (ctx->state == AWS_STATE_RECONNECTING || ctx->state == AWS_STATE_DISCONNECTED)) {

        int delay = AWS_IOT_BASE_BACKOFF;
        for (int i = 0; i < ctx->reconnect_attempt; i++) {
            delay *= 2;
            if (delay > AWS_IOT_MAX_BACKOFF) {
                delay = AWS_IOT_MAX_BACKOFF;
                break;
            }
        }

        fprintf(stdout, "[AWS] Reconnecting in %ds (attempt %d)\n",
                delay, ctx->reconnect_attempt + 1);

        for (int i = 0; i < delay && ctx->keep_running; i++)
            sleep(1);

        if (!ctx->keep_running) break;

        pthread_mutex_lock(&ctx->lock);
        ctx->state = AWS_STATE_CONNECTING;
        pthread_mutex_unlock(&ctx->lock);

        if (tls_connect() == 0 && mqtt_connect() == 0) {
            /* Re-subscribe */
            pthread_mutex_lock(&ctx->lock);
            for (int i = 0; i < ctx->sub_count; i++) {
                mqtt_subscribe(ctx->subs[i].topic);
            }
            ctx->reconnect_attempt = 0;
            ctx->state = AWS_STATE_CONNECTED;
            pthread_mutex_unlock(&ctx->lock);

            fprintf(stdout, "[AWS] Reconnected successfully\n");

            /* Spawn new RX thread */
            pthread_create(&ctx->rx_thread, NULL,
                           (void *(*)(void *))rx_loop, NULL);
            break;
        } else {
            tls_disconnect();
            pthread_mutex_lock(&ctx->lock);
            ctx->reconnect_attempt++;
            ctx->state = AWS_STATE_RECONNECTING;
            pthread_mutex_unlock(&ctx->lock);
        }
    }
}

/* ---------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------- */
int aws_iot_init(const char *thing_name, const char *endpoint,
                 const char *root_ca, const char *cert, const char *key) {
    aws_iot_ctx_t *ctx = &g_ctx;
    memset(ctx, 0, sizeof(*ctx));

    strncpy(ctx->thing_name, thing_name, sizeof(ctx->thing_name) - 1);
    strncpy(ctx->endpoint, endpoint, sizeof(ctx->endpoint) - 1);
    strncpy(ctx->root_ca_path, root_ca, sizeof(ctx->root_ca_path) - 1);
    strncpy(ctx->cert_path, cert, sizeof(ctx->cert_path) - 1);
    strncpy(ctx->key_path, key, sizeof(ctx->key_path) - 1);
    ctx->sock_fd = -1;
    ctx->keep_running = 1;
    ctx->state = AWS_STATE_DISCONNECTED;

    pthread_mutex_init(&ctx->lock, NULL);
    build_topics(ctx);

    return 0;
}

int aws_iot_connect(void) {
    aws_iot_ctx_t *ctx = &g_ctx;

    pthread_mutex_lock(&ctx->lock);
    ctx->state = AWS_STATE_CONNECTING;
    pthread_mutex_unlock(&ctx->lock);

    if (tls_connect() != 0) {
        tls_disconnect();
        ctx->state = AWS_STATE_RECONNECTING;
        goto start_reconnect;
    }

    if (mqtt_connect() != 0) {
        tls_disconnect();
        ctx->state = AWS_STATE_RECONNECTING;
        goto start_reconnect;
    }

    /* Subscribe to shadow delta */
    mqtt_subscribe(ctx->topic_delta);

    pthread_mutex_lock(&ctx->lock);
    ctx->reconnect_attempt = 0;
    ctx->state = AWS_STATE_CONNECTED;
    pthread_mutex_unlock(&ctx->lock);

    /* Start RX thread */
    pthread_create(&ctx->rx_thread, NULL,
                   (void *(*)(void *))rx_loop, NULL);
    return 0;

start_reconnect:
    /* Start reconnect task in a separate thread */
    pthread_t reconn_thread;
    pthread_create(&reconn_thread, NULL,
                   (void *(*)(void *))reconnect_task, NULL);
    pthread_detach(reconn_thread);
    return -1;
}

int aws_iot_disconnect(void) {
    aws_iot_ctx_t *ctx = &g_ctx;
    ctx->keep_running = 0;
    ctx->state = AWS_STATE_DISCONNECTING;

    /* Send DISCONNECT */
    uint8_t disconnect[2] = { 0xE0, 0x00 };
    (void)mqtt_write_all(disconnect, 2);

    tls_disconnect();
    ctx->state = AWS_STATE_DISCONNECTED;
    return 0;
}

int aws_iot_publish_shadow(const char *state_json) {
    aws_iot_ctx_t *ctx = &g_ctx;
    char payload[AWS_IOT_PAYLOAD_MAX];
    snprintf(payload, sizeof(payload),
             "{\"state\":{\"reported\":%s}}", state_json);
    return mqtt_publish(ctx->topic_update, payload, strlen(payload));
}

int aws_iot_publish(const char *topic, const char *payload, size_t len) {
    return mqtt_publish(topic, payload, len);
}

int aws_iot_subscribe(const char *topic, void (*callback)(const char *, const char *, size_t)) {
    aws_iot_ctx_t *ctx = &g_ctx;

    pthread_mutex_lock(&ctx->lock);
    if (ctx->sub_count >= 16) {
        pthread_mutex_unlock(&ctx->lock);
        return -1;
    }
    strncpy(ctx->subs[ctx->sub_count].topic, topic, sizeof(ctx->subs[0].topic) - 1);
    ctx->subs[ctx->sub_count].cb = callback;
    ctx->sub_count++;
    pthread_mutex_unlock(&ctx->lock);

    /* Subscribe on wire if connected */
    if (ctx->state == AWS_STATE_CONNECTED)
        return mqtt_subscribe(topic);
    return 0;
}

const char *aws_iot_get_shadow_topic(int type) {
    aws_iot_ctx_t *ctx = &g_ctx;
    switch (type) {
    case AWS_SHADOW_TOPIC_UPDATE: return ctx->topic_update;
    case AWS_SHADOW_TOPIC_GET:    return ctx->topic_get;
    case AWS_SHADOW_TOPIC_DELTA:  return ctx->topic_delta;
    default: return NULL;
    }
}

void aws_iot_keepalive(void) {
    mqtt_pingreq();
}
