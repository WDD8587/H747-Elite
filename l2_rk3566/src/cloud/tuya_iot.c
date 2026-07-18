/**
 * tuya_iot.c — Tuya Smart IoT platform client (OEM/ODM)
 *
 * Tuya MCU SDK protocol over UART (not MQTT).
 * Data points (DP): DP101=battery, DP102=state, DP103=cleaning_area, DP104=error_code.
 * Heartbeat every 10 seconds.
 * Tuya standard frame format: header(0x55AA) + length + cmd + seq + payload + checksum.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <pthread.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/stat.h>
#include <time.h>

#include "tuya_iot.h"
#include "cloud_selector.h"

/* ---------------------------------------------------------------------------
 * Constants
 * --------------------------------------------------------------------------- */
#define TUYA_UART_BAUD           115200
#define TUYA_UART_DEVICE         "/dev/ttyS3"   /* Typical RK3566 UART for Tuya MCU */

#define TUYA_FRAME_HEADER        0x55AA
#define TUYA_MAX_PAYLOAD         256
#define TUYA_RX_BUF_SIZE         512

#define TUYA_HEARTBEAT_INTERVAL  10  /* seconds */
#define TUYA_HEARTBEAT_CMD       0x00
#define TUYA_QUERY_PRODUCT       0x01
#define TUYA_MCU_DP_REPORT       0x02
#define TUYA_MCU_DP_QUERY        0x03
#define TUYA_MCU_STATE_UPDATE    0x04
#define TUYA_WIFI_STATE          0x05
#define TUYA_RESET               0x06
#define TUYA_MCU_DP_QUERY_ACK    0x07

/* DP IDs */
#define TUYA_DP_BATTERY          101
#define TUYA_DP_STATE             102
#define TUYA_DP_CLEANING_AREA     103
#define TUYA_DP_ERROR_CODE        104

/* ---------------------------------------------------------------------------
 * Tuya frame format (UART MCU protocol)
 *
 * Bytes    | Field
 * -------- | --------------------
 * 2        | Header (0x55AA)
 * 1        | Length of remaining (cmd+seq+payload+checksum)
 * 1        | Command
 * 2        | Sequence number (little-endian)
 * N        | Payload (cmd-specific)
 * 1        | Checksum (XOR of all bytes after header)
 * --------------------------------------------------------------------------- */
#pragma pack(push, 1)
typedef struct {
    uint16_t header;      /* 0x55AA */
    uint8_t  length;      /* remaining packet length (excl. header+length) */
    uint8_t  cmd;
    uint16_t seq;         /* little-endian */
    uint8_t  payload[TUYA_MAX_PAYLOAD];
    /* Checksum appended at end of payload */
} tuya_frame_t;
#pragma pack(pop)

/* ---------------------------------------------------------------------------
 * DP value types
 * --------------------------------------------------------------------------- */
typedef enum {
    TUYA_DP_TYPE_BOOL    = 0x01,
    TUYA_DP_TYPE_VALUE   = 0x02,
    TUYA_DP_TYPE_STRING  = 0x03,
    TUYA_DP_TYPE_ENUM    = 0x04,
    TUYA_DP_TYPE_BITMAP  = 0x05
} tuya_dp_type_t;

#pragma pack(push, 1)
typedef struct {
    uint8_t  dp_id;
    uint8_t  dp_type;
    uint16_t dp_len;      /* little-endian */
    uint8_t  dp_data[64]; /* variable length */
} tuya_dp_t;
#pragma pack(pop)

/* ---------------------------------------------------------------------------
 * Internal context
 * --------------------------------------------------------------------------- */
typedef enum {
    TUYA_STATE_DISCONNECTED,
    TUYA_STATE_CONNECTED,
    TUYA_STATE_ERROR
} tuya_state_t;

typedef struct {
    int              uart_fd;
    tuya_state_t     state;
    int              keep_running;
    uint16_t         seq_counter;

    pthread_t        rx_thread;
    pthread_mutex_t  lock;

    /* Last reported DP values */
    int  dp_battery;         /* DP101: 0-100 */
    int  dp_state;           /* DP102: 0=idle, 1=cleaning, 2=docking, 3=charging, 4=error */
    int  dp_cleaning_area;   /* DP103: cm^2 */
    int  dp_error_code;      /* DP104 */

    /* Callback for incoming commands from cloud -> MCU -> here */
    void (*command_cb)(int dp_id, int dp_value);

    time_t last_heartbeat;
} tuya_ctx_t;

static tuya_ctx_t g_ctx;

/* ---------------------------------------------------------------------------
 * Forward declarations
 * --------------------------------------------------------------------------- */
static int  uart_open(const char *device, int baud);
static void uart_close(void);
static int  uart_send(const uint8_t *data, size_t len);
static int  uart_read(uint8_t *buf, size_t cap, size_t *out_len);
static int  send_frame(uint8_t cmd, const uint8_t *payload, uint8_t plen);
static void send_heartbeat(void);
static void send_dp_report(uint8_t dp_id, uint8_t dp_type,
                           const uint8_t *value, uint16_t value_len);
static void rx_process(void);
static uint8_t compute_checksum(const uint8_t *data, size_t len);

/* ---------------------------------------------------------------------------
 * UART helpers
 * --------------------------------------------------------------------------- */
static int uart_open(const char *device, int baud) {
    int fd = open(device, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        fprintf(stderr, "[Tuya] Failed to open %s: %s\n", device, strerror(errno));
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
        fprintf(stderr, "[Tuya] tcgetattr: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag |= CRTSCTS; /* hardware flow control */

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1; /* 100ms timeout */

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "[Tuya] tcsetattr: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    /* Flush buffers */
    tcflush(fd, TCIOFLUSH);
    return fd;
}

static void uart_close(void) {
    if (g_ctx.uart_fd >= 0) {
        close(g_ctx.uart_fd);
        g_ctx.uart_fd = -1;
    }
}

static int uart_send(const uint8_t *data, size_t len) {
    if (g_ctx.uart_fd < 0) return -1;
    size_t written = 0;
    while (written < len) {
        ssize_t n = write(g_ctx.uart_fd, data + written, len - written);
        if (n < 0) return -1;
        written += (size_t)n;
    }
    return 0;
}

static int uart_read(uint8_t *buf, size_t cap, size_t *out_len) {
    if (g_ctx.uart_fd < 0) return -1;
    ssize_t n = read(g_ctx.uart_fd, buf, cap);
    if (n < 0) return -1;
    *out_len = (size_t)n;
    return 0;
}

/* ---------------------------------------------------------------------------
 * Checksum (XOR of all bytes after header)
 * --------------------------------------------------------------------------- */
static uint8_t compute_checksum(const uint8_t *data, size_t len) {
    uint8_t xor = 0;
    for (size_t i = 0; i < len; i++)
        xor ^= data[i];
    return xor;
}

/* ---------------------------------------------------------------------------
 * Frame send
 * --------------------------------------------------------------------------- */
static int send_frame(uint8_t cmd, const uint8_t *payload, uint8_t plen) {
    tuya_ctx_t *ctx = &g_ctx;
    uint8_t frame[4 + 1 + 2 + TUYA_MAX_PAYLOAD + 1]; /* hdr+len+cmd+seq+payload+cs */
    size_t pos = 0;

    /* Header */
    frame[pos++] = 0x55;
    frame[pos++] = 0xAA;

    /* Length = cmd(1) + seq(2) + payload(N) + cs(1) */
    uint8_t length = 1 + 2 + plen + 1;
    frame[pos++] = length;

    /* Command */
    frame[pos++] = cmd;

    /* Sequence (little-endian) */
    ctx->seq_counter++;
    frame[pos++] = ctx->seq_counter & 0xFF;
    frame[pos++] = (ctx->seq_counter >> 8) & 0xFF;

    /* Payload */
    if (payload && plen > 0) {
        memcpy(&frame[pos], payload, plen);
        pos += plen;
    }

    /* Checksum (XOR from length byte through end of payload) */
    frame[pos] = compute_checksum(&frame[2], length - 1); /* from 'length' byte */
    pos++;

    return uart_send(frame, pos);
}

/* ---------------------------------------------------------------------------
 * DP report
 * --------------------------------------------------------------------------- */
static void send_dp_report(uint8_t dp_id, uint8_t dp_type,
                           const uint8_t *value, uint16_t value_len) {
    /* DP payload: dp_id(1) + dp_type(1) + dp_len(2 LE) + dp_data(N) */
    uint8_t payload[4 + 64];
    payload[0] = dp_id;
    payload[1] = dp_type;
    payload[2] = value_len & 0xFF;
    payload[3] = (value_len >> 8) & 0xFF;
    if (value_len > 64) value_len = 64;
    memcpy(&payload[4], value, value_len);

    send_frame(TUYA_MCU_DP_REPORT, payload, (uint8_t)(4 + value_len));
}

/* ---------------------------------------------------------------------------
 * Heartbeat
 * --------------------------------------------------------------------------- */
static void send_heartbeat(void) {
    uint8_t payload[2];
    payload[0] = 0x00; /* heartbeat sub-command: MCU status */
    payload[1] = 0x01; /* MCU working status: normal */
    send_frame(TUYA_HEARTBEAT_CMD, payload, 2);
    fprintf(stdout, "[Tuya] Heartbeat sent\n");
}

/* ---------------------------------------------------------------------------
 * Process incoming frame
 * --------------------------------------------------------------------------- */
static void process_frame(tuya_frame_t *frame, size_t frame_len) {
    tuya_ctx_t *ctx = &g_ctx;

    switch (frame->cmd) {
    case TUYA_HEARTBEAT_CMD: {
        /* Heartbeat response (0x00): echo back */
        send_heartbeat();
        break;
    }
    case TUYA_QUERY_PRODUCT: {
        /* Product query (0x01): respond with product info */
        /* Standard response: MCU version, etc. */
        uint8_t resp[4] = { 0x01, 0x00, 0x01, 0x00 }; /* version 1.0 */
        send_frame(TUYA_QUERY_PRODUCT, resp, sizeof(resp));
        break;
    }
    case TUYA_MCU_DP_REPORT: {
        /* MCU reports DP changes (cloud->MCU commands arrive as DP writes) */
        tuya_dp_t *dp = (tuya_dp_t *)frame->payload;
        uint16_t dplen = (uint16_t)dp->dp_len;
        uint32_t val = 0;
        for (uint16_t i = 0; i < dplen && i < 4; i++)
            val |= ((uint32_t)dp->dp_data[i]) << (i * 8);

        fprintf(stdout, "[Tuya] DP report: id=%d type=%d val=%u\n",
                dp->dp_id, dp->dp_type, (unsigned)val);

        /* Update local DP cache */
        pthread_mutex_lock(&ctx->lock);
        switch (dp->dp_id) {
        case TUYA_DP_BATTERY:
            ctx->dp_battery = (int)val;
            break;
        case TUYA_DP_STATE:
            ctx->dp_state = (int)val;
            break;
        case TUYA_DP_CLEANING_AREA:
            ctx->dp_cleaning_area = (int)val;
            break;
        case TUYA_DP_ERROR_CODE:
            ctx->dp_error_code = (int)val;
            break;
        default:
            break;
        }
        pthread_mutex_unlock(&ctx->lock);

        /* Notify callback */
        if (ctx->command_cb)
            ctx->command_cb(dp->dp_id, (int)val);
        break;
    }
    case TUYA_MCU_DP_QUERY: {
        /* DP query: respond with current DP values */
        uint8_t battery_val[4] = { (uint8_t)(ctx->dp_battery & 0xFF), 0, 0, 0 };
        uint8_t state_val[4]   = { (uint8_t)(ctx->dp_state & 0xFF), 0, 0, 0 };
        send_dp_report(TUYA_DP_BATTERY, TUYA_DP_TYPE_VALUE, battery_val, 1);
        send_dp_report(TUYA_DP_STATE, TUYA_DP_TYPE_ENUM, state_val, 1);
        break;
    }
    case TUYA_WIFI_STATE: {
        /* WiFi state query */
        uint8_t resp = 0x03; /* 0: no WiFi; 1: config; 2: connecting; 3: connected; 4: error */
        send_frame(TUYA_WIFI_STATE, &resp, 1);
        break;
    }
    case TUYA_RESET: {
        /* Factory reset command */
        fprintf(stdout, "[Tuya] Factory reset requested\n");
        /* Signal higher layers */
        if (ctx->command_cb)
            ctx->command_cb(0xFF, 0 /* reset */);
        break;
    }
    default:
        fprintf(stderr, "[Tuya] Unknown cmd: 0x%02x\n", frame->cmd);
        break;
    }
}

/* ---------------------------------------------------------------------------
 * RX processing
 * --------------------------------------------------------------------------- */
static void rx_process(void) {
    tuya_ctx_t *ctx = &g_ctx;
    uint8_t buf[TUYA_RX_BUF_SIZE];
    size_t len = 0;

    if (uart_read(buf, sizeof(buf), &len) < 0 || len == 0)
        return;

    /* Parse frames from buffer */
    size_t pos = 0;
    while (pos + 4 <= len) {
        /* Find header */
        if (buf[pos] != 0x55 || buf[pos + 1] != 0xAA) {
            pos++;
            continue;
        }

        uint8_t  length   = buf[pos + 2];
        uint8_t  cmd      = buf[pos + 3];

        /* Validate length */
        if (length < 4 || pos + 3 + length > len) {
            pos += 2;
            continue;
        }

        /* Verify checksum */
        uint8_t cs = compute_checksum(&buf[pos + 2], length - 1);
        uint8_t frame_cs = buf[pos + 3 + length - 1];
        if (cs != frame_cs) {
            fprintf(stderr, "[Tuya] Checksum error: calc=%02x got=%02x\n", cs, frame_cs);
            pos += 2;
            continue;
        }

        /* Build frame */
        tuya_frame_t frame;
        frame.header = 0x55AA;
        frame.length = length;
        frame.cmd    = cmd;
        frame.seq    = (uint16_t)(buf[pos + 4] | ((uint16_t)buf[pos + 5] << 8));
        uint8_t plen = length - 4; /* cmd(1) + seq(2) + cs(1) = 4 */
        if (plen > 0)
            memcpy(frame.payload, &buf[pos + 6], plen > TUYA_MAX_PAYLOAD ? TUYA_MAX_PAYLOAD : plen);

        process_frame(&frame, plen + 8);

        pos += 3 + length;
    }
}

/* ---------------------------------------------------------------------------
 * RX loop thread
 * --------------------------------------------------------------------------- */
static void rx_loop(void *arg) {
    (void)arg;
    tuya_ctx_t *ctx = &g_ctx;
    time_t now;

    while (ctx->keep_running && ctx->state == TUYA_STATE_CONNECTED) {
        rx_process();

        /* Heartbeat timer */
        now = time(NULL);
        if (now - ctx->last_heartbeat >= TUYA_HEARTBEAT_INTERVAL) {
            send_heartbeat();
            ctx->last_heartbeat = now;
        }

        usleep(50000); /* 50ms poll */
    }
}

/* ---------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------- */
int tuya_iot_init(void) {
    tuya_ctx_t *ctx = &g_ctx;
    memset(ctx, 0, sizeof(*ctx));
    ctx->uart_fd = -1;
    ctx->keep_running = 1;
    ctx->state = TUYA_STATE_DISCONNECTED;
    ctx->dp_battery = 100;
    ctx->dp_state = 0;
    ctx->dp_cleaning_area = 0;
    ctx->dp_error_code = 0;
    pthread_mutex_init(&ctx->lock, NULL);
    return 0;
}

int tuya_iot_connect(void) {
    tuya_ctx_t *ctx = &g_ctx;

    ctx->uart_fd = uart_open(TUYA_UART_DEVICE, TUYA_UART_BAUD);
    if (ctx->uart_fd < 0) {
        ctx->state = TUYA_STATE_ERROR;
        return -1;
    }

    ctx->state = TUYA_STATE_CONNECTED;
    ctx->last_heartbeat = time(NULL);

    /* Send initial heartbeat and product info */
    send_heartbeat();

    /* Start RX thread */
    pthread_create(&ctx->rx_thread, NULL,
                   (void *(*)(void *))rx_loop, NULL);
    return 0;
}

int tuya_iot_disconnect(void) {
    tuya_ctx_t *ctx = &g_ctx;
    ctx->keep_running = 0;
    uart_close();
    ctx->state = TUYA_STATE_DISCONNECTED;
    return 0;
}

int tuya_iot_report_status(void) {
    tuya_ctx_t *ctx = &g_ctx;
    uint8_t val;

    pthread_mutex_lock(&ctx->lock);
    val = (uint8_t)(ctx->dp_battery & 0xFF);
    send_dp_report(TUYA_DP_BATTERY, TUYA_DP_TYPE_VALUE, &val, 1);

    val = (uint8_t)(ctx->dp_state & 0xFF);
    send_dp_report(TUYA_DP_STATE, TUYA_DP_TYPE_ENUM, &val, 1);
    pthread_mutex_unlock(&ctx->lock);
    return 0;
}

int tuya_iot_send_dp(int dp_id, int dp_type, const uint8_t *value, uint16_t len) {
    send_dp_report((uint8_t)dp_id, (uint8_t)dp_type, value, len);
    return 0;
}

void tuya_iot_set_command_callback(void (*cb)(int dp_id, int dp_value)) {
    g_ctx.command_cb = cb;
}

int tuya_iot_get_battery(void) {
    int val;
    pthread_mutex_lock(&g_ctx.lock);
    val = g_ctx.dp_battery;
    pthread_mutex_unlock(&g_ctx.lock);
    return val;
}

int tuya_iot_get_state(void) {
    int val;
    pthread_mutex_lock(&g_ctx.lock);
    val = g_ctx.dp_state;
    pthread_mutex_unlock(&g_ctx.lock);
    return val;
}
