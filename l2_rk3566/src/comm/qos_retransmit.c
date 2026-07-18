/**
 * @file    qos_retransmit.c
 * @brief   Reliable transmission layer over UART IPC
 * @details Provides sequenced delivery with ACK/NACK, retransmit timer
 *          (100ms RTO), sliding window (max 4 unacknowledged frames),
 *          duplicate detection, and CRC16 per frame.
 *
 *          Protocol:
 *            Frame: [SYNC(2)] [SEQ(2)] [FLAGS(1)] [LEN(2)] [PAYLOAD(N)] [CRC16(2)]
 *            Flags: bit0=ACK, bit1=NACK, bit2=POLL, bit3=RESEND
 *            Window: sender can have up to 4 outstanding frames
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>

/* ------------------------------------------------------------------ */
/*  Constants                                                          */
/* ------------------------------------------------------------------ */
#define QOS_SYNC_WORD           0xAA55u
#define QOS_MAX_PAYLOAD         1024
#define QOS_WINDOW_SIZE         4
#define QOS_RTO_MS              100
#define QOS_MAX_RETRIES         5
#define QOS_MAX_OUTSTANDING     4

#define QOS_FLAG_ACK            0x01
#define QOS_FLAG_NACK           0x02
#define QOS_FLAG_POLL           0x04
#define QOS_FLAG_RESEND         0x08
#define QOS_FLAG_FIN            0x10

#define QOS_HEADER_SIZE         9   /* sync(2)+seq(2)+flags(1)+len(2)+crc(2) */
#define QOS_FRAME_OVERHEAD      QOS_HEADER_SIZE

/* ------------------------------------------------------------------ */
/*  Frame format (on-wire)                                             */
/* ------------------------------------------------------------------ */
typedef struct __attribute__((packed)) {
    uint16_t sync;             /* 0xAA55 */
    uint16_t seq;              /* sequence number (mod 65536) */
    uint8_t  flags;            /* ACK, NACK, POLL, RESEND, FIN */
    uint16_t len;              /* payload length */
    /* payload follows (len bytes) */
    /* CRC16 follows payload (2 bytes) */
} qos_header_t;

/* ------------------------------------------------------------------ */
/*  Internal frame buffer                                              */
/* ------------------------------------------------------------------ */
typedef struct {
    uint16_t  seq;
    uint8_t  *payload;
    uint16_t  len;
    uint8_t   flags;
    uint32_t  sent_time_ms;
    uint8_t   retries;
    bool      in_use;
} qos_slot_t;

/* ------------------------------------------------------------------ */
/*  QoS context                                                        */
/* ------------------------------------------------------------------ */
typedef struct {
    /* Send state */
    uint16_t    next_seq;           /* next sequence number to assign */
    uint16_t    window_start;       /* oldest unacknowledged seq */
    qos_slot_t  window[QOS_WINDOW_SIZE];
    int         outstanding;        /* number of unacknowledged frames */

    /* Receive state */
    uint16_t    expected_seq;       /* next expected sequence number */
    uint16_t    last_acked_seq;     /* last sequence we ACKed */

    /* Statistics */
    uint32_t    frames_sent;
    uint32_t    frames_received;
    uint32_t    frames_acked;
    uint32_t    frames_nacked;
    uint32_t    retransmits;
    uint32_t    duplicates;

    /* Config */
    uint32_t    rto_ms;
    uint8_t     max_retries;

    /* UART IO callbacks */
    int (*uart_send)(const uint8_t *data, uint16_t len);
    int (*uart_recv)(uint8_t *data, uint16_t max_len, int timeout_ms);

    bool        initialized;
} qos_ctx_t;

static qos_ctx_t g_qos;

/* ------------------------------------------------------------------ */
/*  CRC16                                                              */
/* ------------------------------------------------------------------ */
static uint16_t qos_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

/* ------------------------------------------------------------------ */
/*  Time helper                                                        */
/* ------------------------------------------------------------------ */
static uint32_t qos_get_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

/* ------------------------------------------------------------------ */
/*  Sliding window helpers                                             */
/* ------------------------------------------------------------------ */

static int window_index_for_seq(uint16_t seq)
{
    for (int i = 0; i < QOS_WINDOW_SIZE; i++) {
        if (g_qos.window[i].in_use && g_qos.window[i].seq == seq)
            return i;
    }
    return -1;
}

static int window_alloc_slot(void)
{
    for (int i = 0; i < QOS_WINDOW_SIZE; i++) {
        if (!g_qos.window[i].in_use)
            return i;
    }
    return -1;
}

static void window_free(int idx)
{
    if (idx >= 0 && idx < QOS_WINDOW_SIZE) {
        if (g_qos.window[idx].payload) {
            free(g_qos.window[idx].payload);
            g_qos.window[idx].payload = NULL;
        }
        g_qos.window[idx].in_use = false;
        g_qos.outstanding--;
    }
}

/* ------------------------------------------------------------------ */
/*  Frame encode / decode                                              */
/* ------------------------------------------------------------------ */

static int encode_frame(uint16_t seq, uint8_t flags,
                         const uint8_t *payload, uint16_t len,
                         uint8_t *buf, uint16_t *buf_len)
{
    uint16_t total = QOS_HEADER_SIZE + len;
    if (total > QOS_MAX_PAYLOAD + QOS_HEADER_SIZE)
        return -ENOSPC;

    qos_header_t *hdr = (qos_header_t *)buf;
    hdr->sync  = QOS_SYNC_WORD;
    hdr->seq   = seq;
    hdr->flags = flags;
    hdr->len   = len;

    if (payload && len > 0)
        memcpy(buf + sizeof(qos_header_t), payload, len);

    /* CRC covers header + payload */
    uint16_t crc = qos_crc16(buf, sizeof(qos_header_t) + len);
    buf[sizeof(qos_header_t) + len]     = (uint8_t)(crc >> 0);
    buf[sizeof(qos_header_t) + len + 1] = (uint8_t)(crc >> 8);

    *buf_len = total + 2; /* +2 for CRC */
    return 0;
}

static int decode_frame(const uint8_t *buf, uint16_t buf_len,
                         uint16_t *seq, uint8_t *flags,
                         uint8_t *payload, uint16_t *payload_len)
{
    if (buf_len < QOS_HEADER_SIZE + 2)
        return -EINVAL;

    const qos_header_t *hdr = (const qos_header_t *)buf;

    if (hdr->sync != QOS_SYNC_WORD)
        return -EINVAL;

    *seq   = hdr->seq;
    *flags = hdr->flags;
    *payload_len = hdr->len;

    uint16_t total_frame = QOS_HEADER_SIZE + hdr->len + 2;
    if (total_frame > buf_len)
        return -EINVAL;

    /* Verify CRC */
    uint16_t expected_crc = qos_crc16(buf, total_frame - 2);
    uint16_t stored_crc   = (uint16_t)buf[total_frame - 2] |
                            ((uint16_t)buf[total_frame - 1] << 8);

    if (expected_crc != stored_crc)
        return -EINVAL;

    if (payload && hdr->len > 0)
        memcpy(payload, buf + sizeof(qos_header_t), hdr->len);

    return 0;
}

/* ------------------------------------------------------------------ */
/*  Send / receive raw UART frames (via callbacks)                     */
/* ------------------------------------------------------------------ */

static int send_raw(uint16_t seq, uint8_t flags,
                     const uint8_t *payload, uint16_t len)
{
    uint8_t buf[QOS_HEADER_SIZE + QOS_MAX_PAYLOAD + 2];
    uint16_t buf_len;

    int rc = encode_frame(seq, flags, payload, len, buf, &buf_len);
    if (rc != 0) return rc;

    if (g_qos.uart_send)
        return g_qos.uart_send(buf, buf_len);

    /* Stub direct write */
    (void)buf; (void)buf_len;
    return 0;
}

static int recv_raw(uint16_t *seq, uint8_t *flags,
                     uint8_t *payload, uint16_t *len)
{
    uint8_t buf[QOS_HEADER_SIZE + QOS_MAX_PAYLOAD + 2];

    if (g_qos.uart_recv) {
        int n = g_qos.uart_recv(buf, sizeof(buf), 0);
        if (n <= 0) return -EAGAIN;
        return decode_frame(buf, (uint16_t)n, seq, flags, payload, len);
    }

    /* Stub: no data */
    (void)buf;
    return -EAGAIN;
}

/* ------------------------------------------------------------------ */
/*  ACK/NACK processing                                                */
/* ------------------------------------------------------------------ */

static void process_ack(uint16_t seq)
{
    /* Acknowledge all frames up to and including seq */
    for (int i = 0; i < QOS_WINDOW_SIZE; i++) {
        if (g_qos.window[i].in_use &&
            (int16_t)(g_qos.window[i].seq - seq) <= 0) {
            window_free(i);
            g_qos.frames_acked++;
        }
    }
}

static void process_nack(uint16_t seq)
{
    g_qos.frames_nacked++;

    /* Find and resend the NACKed frame immediately */
    for (int i = 0; i < QOS_WINDOW_SIZE; i++) {
        if (g_qos.window[i].in_use && g_qos.window[i].seq == seq) {
            g_qos.window[i].retries++;
            if (g_qos.window[i].retries > g_qos.max_retries) {
                fprintf(stderr, "[QOS] frame %u max retries exceeded, dropping\n", seq);
                window_free(i);
            } else {
                g_qos.retransmits++;
                g_qos.window[i].sent_time_ms = qos_get_ms();
                send_raw(seq, 0, g_qos.window[i].payload, g_qos.window[i].len);
            }
            break;
        }
    }
}

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

int qos_init(int (*tx)(const uint8_t *, uint16_t),
              int (*rx)(uint8_t *, uint16_t, int))
{
    memset(&g_qos, 0, sizeof(g_qos));
    g_qos.uart_send    = tx;
    g_qos.uart_recv    = rx;
    g_qos.rto_ms       = QOS_RTO_MS;
    g_qos.max_retries  = QOS_MAX_RETRIES;
    g_qos.expected_seq = 0;
    g_qos.next_seq     = 0;
    g_qos.window_start = 0;

    for (int i = 0; i < QOS_WINDOW_SIZE; i++)
        g_qos.window[i].in_use = false;

    g_qos.initialized = true;
    return 0;
}

/**
 * qos_send - send data reliably with retransmit
 * @data: payload bytes
 * @len:  payload length
 *
 * Returns 0 on success (frame queued), negative on error.
 * If window is full, returns -EAGAIN (caller should retry).
 */
int qos_send(const uint8_t *data, uint16_t len)
{
    if (!g_qos.initialized) return -EPERM;
    if (len > QOS_MAX_PAYLOAD) return -EMSGSIZE;

    if (g_qos.outstanding >= QOS_MAX_OUTSTANDING)
        return -EAGAIN;

    int slot = window_alloc_slot();
    if (slot < 0) return -EAGAIN;

    uint16_t seq = g_qos.next_seq++;

    /* Store in window */
    g_qos.window[slot].in_use = true;
    g_qos.window[slot].seq    = seq;
    g_qos.window[slot].len    = len;
    g_qos.window[slot].flags  = 0;
    g_qos.window[slot].retries = 0;
    g_qos.window[slot].sent_time_ms = qos_get_ms();
    g_qos.window[slot].payload = (uint8_t *)malloc(len);
    if (!g_qos.window[slot].payload) {
        g_qos.window[slot].in_use = false;
        return -ENOMEM;
    }
    memcpy(g_qos.window[slot].payload, data, len);
    g_qos.outstanding++;

    /* Send the frame */
    int rc = send_raw(seq, 0, data, len);
    if (rc == 0)
        g_qos.frames_sent++;

    return rc;
}

/**
 * qos_recv - receive next reliable data frame
 * @buf:    output buffer
 * @buf_len: input: buffer capacity; output: received length
 *
 * Returns 0 on success, -EAGAIN if no data available.
 */
int qos_recv(uint8_t *buf, uint16_t *buf_len)
{
    if (!g_qos.initialized) return -EPERM;
    if (!buf || !buf_len) return -EINVAL;

    uint16_t seq, len;
    uint8_t  flags;

    int rc = recv_raw(&seq, &flags, buf, &len);
    if (rc != 0) return rc;

    g_qos.frames_received++;

    /* Process ACK/NACK */
    if (flags & QOS_FLAG_ACK) {
        process_ack(seq);
        return -EAGAIN; /* not a data frame */
    }
    if (flags & QOS_FLAG_NACK) {
        process_nack(seq);
        return -EAGAIN;
    }
    if (flags & QOS_FLAG_POLL) {
        /* Poll request — send ACK for our last received */
        send_raw(g_qos.last_acked_seq, QOS_FLAG_ACK, NULL, 0);
        return -EAGAIN;
    }
    if (flags & QOS_FLAG_RESEND) {
        /* Remote requests a resend of a specific frame */
        int idx = window_index_for_seq(seq);
        if (idx >= 0) {
            send_raw(seq, 0, g_qos.window[idx].payload, g_qos.window[idx].len);
            g_qos.retransmits++;
        }
        return -EAGAIN;
    }

    /* Duplicate detection */
    if ((int16_t)(seq - g_qos.expected_seq) < 0) {
        g_qos.duplicates++;
        /* Send ACK for the duplicate (don't process) */
        send_raw(seq, QOS_FLAG_ACK, NULL, 0);
        return -EAGAIN;
    }

    /* Check for gap */
    if (seq != g_qos.expected_seq) {
        /* Gap detected — send NACK */
        send_raw(g_qos.expected_seq, QOS_FLAG_NACK, NULL, 0);
        return -EAGAIN;
    }

    /* Deliver the frame */
    *buf_len = len;
    g_qos.expected_seq = seq + 1;
    g_qos.last_acked_seq = seq;

    /* Send ACK */
    send_raw(seq, QOS_FLAG_ACK, NULL, 0);

    return 0;
}

/**
 * qos_poll - periodic maintenance: retransmit timer, window cleanup
 *            Call at least every 50ms.
 */
int qos_poll(void)
{
    if (!g_qos.initialized) return -EPERM;

    uint32_t now = qos_get_ms();

    for (int i = 0; i < QOS_WINDOW_SIZE; i++) {
        if (!g_qos.window[i].in_use) continue;

        uint32_t elapsed = now - g_qos.window[i].sent_time_ms;

        if (elapsed >= g_qos.rto_ms) {
            /* Timeout — retransmit */
            g_qos.window[i].retries++;
            if (g_qos.window[i].retries > g_qos.max_retries) {
                fprintf(stderr, "[QOS] frame %u timeout, max retries\n",
                        g_qos.window[i].seq);
                window_free(i);
            } else {
                g_qos.retransmits++;
                g_qos.window[i].sent_time_ms = now;
                send_raw(g_qos.window[i].seq, 0,
                         g_qos.window[i].payload, g_qos.window[i].len);
            }
        }
    }

    return 0;
}

/**
 * qos_flush - wait for all outstanding frames to be ACKed
 * @timeout_ms: maximum wait time in ms
 *
 * Returns 0 if all frames flushed, -ETIMEDOUT if timeout.
 */
int qos_flush(uint32_t timeout_ms)
{
    uint32_t deadline = qos_get_ms() + timeout_ms;

    while (g_qos.outstanding > 0 && qos_get_ms() < deadline) {
        qos_poll();
        usleep(10000); /* 10ms */
    }

    return (g_qos.outstanding == 0) ? 0 : -ETIMEDOUT;
}

/**
 * qos_get_stats - copy runtime statistics
 */
void qos_get_stats(uint32_t *sent, uint32_t *received,
                    uint32_t *acked, uint32_t *nacked,
                    uint32_t *retransmits, uint32_t *duplicates)
{
    if (sent)        *sent        = g_qos.frames_sent;
    if (received)    *received    = g_qos.frames_received;
    if (acked)       *acked       = g_qos.frames_acked;
    if (nacked)      *nacked      = g_qos.frames_nacked;
    if (retransmits) *retransmits = g_qos.retransmits;
    if (duplicates)  *duplicates  = g_qos.duplicates;
}

void qos_shutdown(void)
{
    for (int i = 0; i < QOS_WINDOW_SIZE; i++)
        window_free(i);
    g_qos.initialized = false;
}

/* ------------------------------------------------------------------ */
/*  Self-test                                                          */
/* ------------------------------------------------------------------ */
#ifdef TEST_QOS
int main(void)
{
    qos_init(NULL, NULL);

    uint8_t payload[] = "Hello, reliable world!";
    int rc = qos_send(payload, (uint16_t)sizeof(payload));
    printf("qos_send: %d\n", rc);

    qos_poll();
    qos_flush(1000);

    qos_shutdown();
    return 0;
}
#endif /* TEST_QOS */
