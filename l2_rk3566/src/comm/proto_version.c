/**
 * proto_version.c — Protocol version negotiation
 *
 * When M7 connects to RK3566 (or vice versa), exchange VERSION frame with
 * supported protocol versions. Negotiate highest common version.
 * If incompatible → alert + log. Supports downgrade compatibility.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <pthread.h>

#include "proto_version.h"

/* ---------------------------------------------------------------------------
 * Constants
 * --------------------------------------------------------------------------- */
#define PROTO_MAX_VERSIONS        16
#define PROTO_FRAME_HEADER        0xAA55
#define PROTO_FRAME_MAX_SIZE      256
#define PROTO_NEGOTIATE_TIMEOUT_MS 5000

/* Frame types */
#define FRAME_TYPE_VERSION_REQ    0x01
#define FRAME_TYPE_VERSION_RESP   0x02
#define FRAME_TYPE_VERSION_ACK    0x03
#define FRAME_TYPE_VERSION_NAK    0x04

/* ---------------------------------------------------------------------------
 * Frame format
 * --------------------------------------------------------------------------- */
#pragma pack(push, 1)
typedef struct {
    uint16_t header;         /* 0xAA55 */
    uint8_t  frame_type;
    uint8_t  length;         /* payload length */
    uint8_t  payload[240];
    uint16_t checksum;
} version_frame_t;

/* Version request/response payload */
typedef struct {
    uint8_t  num_versions;
    uint8_t  versions[PROTO_MAX_VERSIONS]; /* each version = major << 4 | minor */
} version_payload_t;

/* Version ack payload */
typedef struct {
    uint8_t  negotiated_version;
    uint8_t  reserved[3];
} version_ack_payload_t;
#pragma pack(pop)

/* ---------------------------------------------------------------------------
 * Supported protocol versions (firmware capabilities)
 * --------------------------------------------------------------------------- */
static const uint8_t g_supported_versions[] = {
    0x20,  /* v2.0 — current */
    0x13,  /* v1.3 */
    0x12,  /* v1.2 */
    0x11,  /* v1.1 */
};

static const int g_supported_count = sizeof(g_supported_versions) / sizeof(g_supported_versions[0]);

/* ---------------------------------------------------------------------------
 * Context
/* --------------------------------------------------------------------------- */
typedef struct {
    uint8_t  local_version;      /* best local version */
    uint8_t  negotiated_version; /* agreed version (0 = none) */
    int      compatible;

    /* Transport callbacks (set by higher layer) */
    int (*send_fn)(const uint8_t *data, size_t len);
    int (*recv_fn)(uint8_t *buf, size_t cap, size_t *out_len, int timeout_ms);

    pthread_mutex_t lock;
    int             negotiated;

    /* Negotiation status */
    uint64_t        last_request_ms;
    int             retry_count;
    int             max_retries;
} proto_ctx_t;

static proto_ctx_t g_ctx;

/* ---------------------------------------------------------------------------
 * Forward declarations
/* --------------------------------------------------------------------------- */
static uint16_t frame_checksum(const version_frame_t *frame, size_t len);
static int      send_frame(uint8_t frame_type, const uint8_t *payload, uint8_t plen);
static int      recv_frame(version_frame_t *frame, int timeout_ms);
static int      negotiate_as_initiator(void);
static int      negotiate_as_responder(void);

/* ---------------------------------------------------------------------------
 * Checksum
 * --------------------------------------------------------------------------- */
static uint16_t frame_checksum(const version_frame_t *frame, size_t len) {
    uint16_t sum = 0;
    const uint8_t *bytes = (const uint8_t *)frame;
    for (size_t i = 0; i < len && i < sizeof(version_frame_t); i++) {
        sum += bytes[i];
    }
    return sum;
}

/* ---------------------------------------------------------------------------
 * Send a version frame
 * --------------------------------------------------------------------------- */
static int send_frame(uint8_t frame_type, const uint8_t *payload, uint8_t plen) {
    if (!g_ctx.send_fn) return -1;

    version_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.header = PROTO_FRAME_HEADER;
    frame.frame_type = frame_type;
    frame.length = plen;
    if (payload && plen > 0)
        memcpy(frame.payload, payload, plen > 240 ? 240 : plen);

    size_t frame_size = 4 + plen; /* header(2) + type(1) + length(1) + payload(N) — cs separate */
    /* Actually: header(2) + type(1) + len(1) + payload(N) + cs(2) = 4+N+2 */
    frame_size = 4 + plen + 2;
    frame.checksum = frame_checksum(&frame, frame_size - 2);

    return g_ctx.send_fn((const uint8_t *)&frame, frame_size);
}

/* ---------------------------------------------------------------------------
 * Receive a version frame
 * --------------------------------------------------------------------------- */
static int recv_frame(version_frame_t *frame, int timeout_ms) {
    if (!g_ctx.recv_fn) return -1;

    uint8_t buf[sizeof(version_frame_t)];
    size_t len = 0;

    if (g_ctx.recv_fn(buf, sizeof(buf), &len, timeout_ms) < 0)
        return -1;

    if (len < 6) return -1; /* minimum frame */

    memcpy(frame, buf, len < sizeof(*frame) ? len : sizeof(*frame));

    /* Validate header */
    if (frame->header != PROTO_FRAME_HEADER) return -1;

    /* Verify checksum */
    uint16_t cs = frame_checksum(frame, len - 2);
    if (cs != frame->checksum) return -1;

    return 0;
}

/* ---------------------------------------------------------------------------
 * Find highest common version
 * --------------------------------------------------------------------------- */
static uint8_t find_common_version(const uint8_t *remote, int remote_count) {
    /* Highest version that exists in both sets */
    for (int i = 0; i < g_supported_count; i++) {
        for (int j = 0; j < remote_count; j++) {
            if (g_supported_versions[i] == remote[j])
                return g_supported_versions[i];
        }
    }
    return 0; /* no common version */
}

/* ---------------------------------------------------------------------------
 * Negotiator: send VERSION_REQ, wait for VERSION_RESP, send VERSION_ACK/NAK
 * --------------------------------------------------------------------------- */
static int negotiate_as_initiator(void) {
    /* Build version request with local supported versions */
    version_payload_t req_payload;
    req_payload.num_versions = (uint8_t)g_supported_count;
    memcpy(req_payload.versions, g_supported_versions, g_supported_count);

    if (send_frame(FRAME_TYPE_VERSION_REQ, (const uint8_t *)&req_payload,
                   1 + g_supported_count) < 0) {
        fprintf(stderr, "[ProtoVer] Failed to send VERSION_REQ\n");
        return -1;
    }

    /* Wait for VERSION_RESP */
    version_frame_t resp;
    if (recv_frame(&resp, PROTO_NEGOTIATE_TIMEOUT_MS) < 0) {
        fprintf(stderr, "[ProtoVer] No VERSION_RESP received\n");
        return -1;
    }

    if (resp.frame_type != FRAME_TYPE_VERSION_RESP) {
        fprintf(stderr, "[ProtoVer] Expected VERSION_RESP, got 0x%02x\n",
                resp.frame_type);
        return -1;
    }

    /* Parse remote versions */
    version_payload_t *remote = (version_payload_t *)resp.payload;
    int remote_count = remote->num_versions;
    if (remote_count > PROTO_MAX_VERSIONS) remote_count = PROTO_MAX_VERSIONS;

    fprintf(stdout, "[ProtoVer] Remote supports %d versions\n", remote_count);

    /* Negotiate: find highest common version */
    uint8_t common = find_common_version(remote->versions, remote_count);

    if (common == 0) {
        /* No compatible version — send NAK */
        fprintf(stderr, "[ProtoVer] No compatible protocol version!\n");
        version_ack_payload_t nak;
        memset(&nak, 0, sizeof(nak));
        send_frame(FRAME_TYPE_VERSION_NAK, (const uint8_t *)&nak, sizeof(nak));

        pthread_mutex_lock(&g_ctx.lock);
        g_ctx.compatible = 0;
        g_ctx.negotiated_version = 0;
        g_ctx.negotiated = 1;
        pthread_mutex_unlock(&g_ctx.lock);

        return -1;
    }

    /* Send ACK with negotiated version */
    version_ack_payload_t ack;
    memset(&ack, 0, sizeof(ack));
    ack.negotiated_version = common;

    if (send_frame(FRAME_TYPE_VERSION_ACK, (const uint8_t *)&ack, sizeof(ack)) < 0) {
        fprintf(stderr, "[ProtoVer] Failed to send VERSION_ACK\n");
        return -1;
    }

    pthread_mutex_lock(&g_ctx.lock);
    g_ctx.negotiated_version = common;
    g_ctx.compatible = 1;
    g_ctx.negotiated = 1;
    pthread_mutex_unlock(&g_ctx.lock);

    fprintf(stdout, "[ProtoVer] Negotiated version: %d.%d (0x%02x)\n",
            (common >> 4) & 0x0F, common & 0x0F, common);
    return 0;
}

/* ---------------------------------------------------------------------------
 * Responder: receive VERSION_REQ, compare, send VERSION_RESP, wait for ACK/NAK
 * --------------------------------------------------------------------------- */
static int negotiate_as_responder(void) {
    /* Wait for VERSION_REQ */
    version_frame_t req;
    if (recv_frame(&req, PROTO_NEGOTIATE_TIMEOUT_MS) < 0) {
        fprintf(stderr, "[ProtoVer] No VERSION_REQ received (responder)\n");
        return -1;
    }

    if (req.frame_type != FRAME_TYPE_VERSION_REQ) {
        fprintf(stderr, "[ProtoVer] Expected VERSION_REQ, got 0x%02x\n",
                req.frame_type);
        return -1;
    }

    /* Parse remote versions */
    version_payload_t *remote = (version_payload_t *)req.payload;
    int remote_count = remote->num_versions;
    if (remote_count > PROTO_MAX_VERSIONS) remote_count = PROTO_MAX_VERSIONS;

    /* Check if we share at least one version */
    uint8_t common = find_common_version(remote->versions, remote_count);

    /* Send our supported versions as response */
    version_payload_t resp_payload;
    resp_payload.num_versions = (uint8_t)g_supported_count;
    memcpy(resp_payload.versions, g_supported_versions, g_supported_count);

    if (send_frame(FRAME_TYPE_VERSION_RESP, (const uint8_t *)&resp_payload,
                   1 + g_supported_count) < 0) {
        return -1;
    }

    /* Wait for ACK or NAK */
    version_frame_t ack_frame;
    if (recv_frame(&ack_frame, PROTO_NEGOTIATE_TIMEOUT_MS) < 0) {
        fprintf(stderr, "[ProtoVer] No ACK/NAK received\n");
        return -1;
    }

    if (ack_frame.frame_type == FRAME_TYPE_VERSION_NAK) {
        fprintf(stderr, "[ProtoVer] Remote rejected negotiation (incompatible)\n");
        pthread_mutex_lock(&g_ctx.lock);
        g_ctx.compatible = 0;
        g_ctx.negotiated_version = 0;
        g_ctx.negotiated = 1;
        pthread_mutex_unlock(&g_ctx.lock);
        return -1;
    }

    if (ack_frame.frame_type == FRAME_TYPE_VERSION_ACK) {
        version_ack_payload_t *ack = (version_ack_payload_t *)ack_frame.payload;
        uint8_t version = ack->negotiated_version;

        /* Verify it's in our list */
        int valid = 0;
        for (int i = 0; i < g_supported_count; i++) {
            if (g_supported_versions[i] == version) { valid = 1; break; }
        }

        if (!valid) {
            fprintf(stderr, "[ProtoVer] Remote negotiated invalid version 0x%02x\n", version);
            return -1;
        }

        pthread_mutex_lock(&g_ctx.lock);
        g_ctx.negotiated_version = version;
        g_ctx.compatible = 1;
        g_ctx.negotiated = 1;
        pthread_mutex_unlock(&g_ctx.lock);

        fprintf(stdout, "[ProtoVer] Negotiated version: %d.%d (0x%02x)\n",
                (version >> 4) & 0x0F, version & 0x0F, version);
        return 0;
    }

    return -1;
}

/* ---------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------- */
int proto_version_init(void) {
    memset(&g_ctx, 0, sizeof(g_ctx));
    pthread_mutex_init(&g_ctx.lock, NULL);

    /* Local best version = first in list */
    g_ctx.local_version = g_supported_versions[0];
    g_ctx.negotiated_version = 0;
    g_ctx.compatible = 0;
    g_ctx.negotiated = 0;
    g_ctx.max_retries = 3;

    return 0;
}

int proto_version_set_transport(int (*send_fn)(const uint8_t *, size_t),
                                 int (*recv_fn)(uint8_t *, size_t, size_t *, int)) {
    g_ctx.send_fn = send_fn;
    g_ctx.recv_fn = recv_fn;
    return 0;
}

int proto_version_negotiate(int as_initiator) {
    int ret;

    if (as_initiator) {
        ret = negotiate_as_initiator();
    } else {
        ret = negotiate_as_responder();
    }

    /* Retry if failed */
    if (ret < 0 && g_ctx.retry_count < g_ctx.max_retries) {
        g_ctx.retry_count++;
        fprintf(stdout, "[ProtoVer] Retry %d/%d\n",
                g_ctx.retry_count, g_ctx.max_retries);
        sleep(1);
        return proto_version_negotiate(as_initiator);
    }

    return ret;
}

/* ---------------------------------------------------------------------------
 * Queries
 * --------------------------------------------------------------------------- */
int proto_version_is_compatible(void) {
    pthread_mutex_lock(&g_ctx.lock);
    int c = g_ctx.compatible;
    pthread_mutex_unlock(&g_ctx.lock);
    return c;
}

uint8_t proto_version_get_negotiated(void) {
    pthread_mutex_lock(&g_ctx.lock);
    uint8_t v = g_ctx.negotiated_version;
    pthread_mutex_unlock(&g_ctx.lock);
    return v;
}

uint8_t proto_version_get_local(void) {
    return g_ctx.local_version;
}

int proto_version_get_supported_list(uint8_t *buf, int max) {
    int count = g_supported_count < max ? g_supported_count : max;
    memcpy(buf, g_supported_versions, count);
    return count;
}

int proto_version_check_compatibility(uint8_t remote_version) {
    for (int i = 0; i < g_supported_count; i++) {
        if (g_supported_versions[i] == remote_version)
            return 1;
    }
    return 0;
}

const char *proto_version_string(uint8_t version) {
    static char buf[16];
    snprintf(buf, sizeof(buf), "%d.%d",
             (version >> 4) & 0x0F, version & 0x0F);
    return buf;
}
