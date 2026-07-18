/**
 * @file    reconnect.c
 * @brief   Connection management with exponential backoff
 * @details Detects link down (no ACK for 500ms), then attempts reconnect
 *          with exponential backoff: 100ms, 200ms, 400ms, 800ms, 1600ms,
 *          5s, 10s, 30s (cap).  On reconnect, resyncs sequence numbers
 *          and requests missing frames.
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
#define RECONNECT_ACK_TIMEOUT_MS    500    /* link considered down after no ACK */
#define RECONNECT_BACKOFF_BASE_MS   100    /* initial backoff */
#define RECONNECT_BACKOFF_CAP_MS    30000  /* max backoff (30s) */
#define RECONNECT_BACKOFF_STEPS     8
#define RECONNECT_HEARTBEAT_INTERVAL_MS 1000
#define RECONNECT_SYNC_RETRIES      3

/* ------------------------------------------------------------------ */
/*  States                                                             */
/* ------------------------------------------------------------------ */
typedef enum {
    LINK_STATE_CONNECTED    = 0,
    LINK_STATE_DISCONNECTED = 1,
    LINK_STATE_RECONNECTING = 2,
    LINK_STATE_SYNCING      = 3,
} link_state_t;

static const char *link_state_name(link_state_t s)
{
    switch (s) {
    case LINK_STATE_CONNECTED:    return "CONNECTED";
    case LINK_STATE_DISCONNECTED: return "DISCONNECTED";
    case LINK_STATE_RECONNECTING: return "RECONNECTING";
    case LINK_STATE_SYNCING:      return "SYNCING";
    default:                      return "UNKNOWN";
    }
}

/* ------------------------------------------------------------------ */
/*  Backoff schedule                                                   */
/* ------------------------------------------------------------------ */
static const uint32_t g_backoff_schedule[RECONNECT_BACKOFF_STEPS] = {
    100,    /* 100ms */
    200,    /* 200ms */
    400,    /* 400ms */
    800,    /* 800ms */
    1600,   /* 1.6s */
    5000,   /* 5s */
    10000,  /* 10s */
    30000   /* 30s (cap) */
};

/* ------------------------------------------------------------------ */
/*  Reconnect context                                                  */
/* ------------------------------------------------------------------ */
typedef struct {
    link_state_t state;
    uint32_t     last_ack_ms;
    uint32_t     last_heartbeat_ms;
    int          backoff_step;
    uint32_t     next_reconnect_ms;
    uint32_t     disconnect_time_ms;    /* when link was lost */
    uint32_t     total_disconnected_ms; /* cumulative time disconnected */
    uint32_t     reconnect_attempts;
    uint32_t     successful_reconnects;

    /* Sequence resync */
    uint16_t     local_seq;
    uint16_t     remote_seq;
    bool         sync_pending;

    /* Flags */
    bool         initialized;

    /* Callback: send a frame (returns 0 on success) */
    int (*send_fn)(const uint8_t *data, uint16_t len);
} reconnect_ctx_t;

static reconnect_ctx_t g_recon;

/* ------------------------------------------------------------------ */
/*  Time helper                                                        */
/* ------------------------------------------------------------------ */
static uint32_t recon_get_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

/* ------------------------------------------------------------------ */
/*  Notification callback (stub)                                       */
/* ------------------------------------------------------------------ */
static void recon_notify(const char *event, const char *detail)
{
    fprintf(stderr, "[RECONNECT] %s: %s\n", event, detail ? detail : "");
}

/* ------------------------------------------------------------------ */
/*  Link state transitions                                             */
/* ------------------------------------------------------------------ */

static void enter_disconnected(void)
{
    if (g_recon.state == LINK_STATE_DISCONNECTED) return;

    g_recon.state = LINK_STATE_DISCONNECTED;
    g_recon.disconnect_time_ms = recon_get_ms();
    g_recon.backoff_step = 0;
    g_recon.next_reconnect_ms = recon_get_ms() + g_backoff_schedule[0];
    g_recon.reconnect_attempts = 0;

    recon_notify("link_state", "DISCONNECTED");
}

static void enter_reconnecting(void)
{
    g_recon.state = LINK_STATE_RECONNECTING;
    g_recon.reconnect_attempts++;

    uint32_t delay = g_backoff_schedule[g_recon.backoff_step];
    g_recon.next_reconnect_ms = recon_get_ms() + delay;

    /* Advance backoff step (with cap) */
    if (g_recon.backoff_step < RECONNECT_BACKOFF_STEPS - 1)
        g_recon.backoff_step++;

    recon_notify("reconnecting", "attempt %u with %ums delay",
                 g_recon.reconnect_attempts, delay);
}

static void enter_syncing(void)
{
    g_recon.state = LINK_STATE_SYNCING;
    g_recon.sync_pending = true;
    recon_notify("link_state", "SYNCING");
}

static void enter_connected(void)
{
    g_recon.state = LINK_STATE_CONNECTED;
    g_recon.last_ack_ms = recon_get_ms();
    g_recon.last_heartbeat_ms = recon_get_ms();
    g_recon.backoff_step = 0;
    g_recon.successful_reconnects++;

    if (g_recon.disconnect_time_ms > 0) {
        g_recon.total_disconnected_ms +=
            recon_get_ms() - g_recon.disconnect_time_ms;
        g_recon.disconnect_time_ms = 0;
    }

    recon_notify("link_state", "CONNECTED");
}

/* ------------------------------------------------------------------ */
/*  Sequence synchronization                                           */
/* ------------------------------------------------------------------ */

static int send_sync_frame(void)
{
    /* SYNC frame: flags byte = 0xFF, payload = local seq number */
    uint8_t payload[2];
    payload[0] = (uint8_t)(g_recon.local_seq >> 0);
    payload[1] = (uint8_t)(g_recon.local_seq >> 8);

    if (g_recon.send_fn)
        return g_recon.send_fn(payload, 2);
    return 0;
}

static int process_sync_response(const uint8_t *data, uint16_t len)
{
    if (len < 2) return -EINVAL;

    uint16_t remote_seq = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    g_recon.remote_seq = remote_seq;

    /* Our local_seq is past what we last sent successfully */
    g_recon.sync_pending = false;

    recon_notify("sync", "remote seq=%u", remote_seq);
    return 0;
}

/* ------------------------------------------------------------------ */
/*  Heartbeat / keepalive                                              */
/* ------------------------------------------------------------------ */

static int send_heartbeat(void)
{
    uint8_t hb = 0x00;
    if (g_recon.send_fn)
        return g_recon.send_fn(&hb, 1);
    return 0;
}

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

int recon_init(int (*send_fn)(const uint8_t *, uint16_t))
{
    memset(&g_recon, 0, sizeof(g_recon));
    g_recon.state = LINK_STATE_CONNECTED;
    g_recon.last_ack_ms = recon_get_ms();
    g_recon.last_heartbeat_ms = recon_get_ms();
    g_recon.send_fn = send_fn;
    g_recon.backoff_step = 0;
    g_recon.local_seq = 0;
    g_recon.remote_seq = 0;
    g_recon.initialized = true;

    recon_notify("init", "connected");
    return 0;
}

/**
 * recon_mark_connected - called when external logic confirms link is up
 */
void recon_mark_connected(void)
{
    if (g_recon.state != LINK_STATE_CONNECTED) {
        enter_connected();
    }
    g_recon.last_ack_ms = recon_get_ms();
}

/**
 * recon_mark_disconnected - called when link failure is detected externally
 */
void recon_mark_disconnected(void)
{
    enter_disconnected();
}

/**
 * recon_on_ack - called when an ACK frame is received
 */
void recon_on_ack(void)
{
    g_recon.last_ack_ms = recon_get_ms();

    if (g_recon.state == LINK_STATE_SYNCING) {
        enter_connected();
    } else if (g_recon.state == LINK_STATE_DISCONNECTED ||
               g_recon.state == LINK_STATE_RECONNECTING) {
        enter_connected();
    }
}

/**
 * recon_on_data_received - called when any data frame is received
 */
void recon_on_data_received(void)
{
    g_recon.last_ack_ms = recon_get_ms();

    if (g_recon.state != LINK_STATE_CONNECTED)
        enter_connected();
}

/**
 * recon_process_sync - process an incoming sync frame
 * @data: sync frame payload
 * @len:  payload length
 */
int recon_process_sync(const uint8_t *data, uint16_t len)
{
    return process_sync_response(data, len);
}

/**
 * recon_get_state - get current link state
 */
link_state_t recon_get_state(void)
{
    return g_recon.state;
}

/**
 * recon_is_connected - check if link is connected
 */
bool recon_is_connected(void)
{
    return g_recon.state == LINK_STATE_CONNECTED;
}

/**
 * recon_poll - periodic maintenance (call every ~10-20ms)
 *
 * Handles:
 *   - Link loss detection (no ACK for 500ms)
 *   - Exponential backoff reconnect attempts
 *   - Sequence resync on reconnect
 *   - Heartbeat transmissions when connected
 */
int recon_poll(void)
{
    if (!g_recon.initialized) return -EPERM;

    uint32_t now = recon_get_ms();

    switch (g_recon.state) {

    case LINK_STATE_CONNECTED: {
        /* Check for link loss */
        uint32_t elapsed = now - g_recon.last_ack_ms;
        if (elapsed >= RECONNECT_ACK_TIMEOUT_MS) {
            recon_notify("ack_timeout", "%u ms since last ACK", elapsed);
            enter_disconnected();
        }

        /* Periodic heartbeat */
        if (now - g_recon.last_heartbeat_ms >= RECONNECT_HEARTBEAT_INTERVAL_MS) {
            send_heartbeat();
            g_recon.last_heartbeat_ms = now;
        }
        break;
    }

    case LINK_STATE_DISCONNECTED: {
        /* Wait for backoff timer, then attempt reconnect */
        if (now >= g_recon.next_reconnect_ms) {
            enter_reconnecting();
        }
        break;
    }

    case LINK_STATE_RECONNECTING: {
        /* Send a sync frame to initiate reconnection */
        int rc = send_sync_frame();
        if (rc == 0) {
            enter_syncing();
        } else {
            /* Send failed, back off and retry */
            g_recon.next_reconnect_ms = now + g_backoff_schedule[g_recon.backoff_step];
            if (g_recon.backoff_step < RECONNECT_BACKOFF_STEPS - 1)
                g_recon.backoff_step++;
        }
        break;
    }

    case LINK_STATE_SYNCING: {
        /* Check if sync timed out */
        static uint32_t sync_start = 0;
        if (sync_start == 0) sync_start = now;

        if (now - sync_start > 2000) { /* 2s sync timeout */
            recon_notify("sync_timeout", "retrying");
            sync_start = 0;
            enter_reconnecting();
        }

        if (!g_recon.sync_pending) {
            sync_start = 0;
            enter_connected();
        }
        break;
    }
    }

    return 0;
}

/**
 * recon_get_backoff_step - get current backoff step index
 */
int recon_get_backoff_step(void)
{
    return g_recon.backoff_step;
}

/**
 * recon_get_stats - get reconnect statistics
 */
void recon_get_stats(uint32_t *attempts, uint32_t *successes,
                      uint32_t *total_disconnected_ms)
{
    if (attempts)              *attempts              = g_recon.reconnect_attempts;
    if (successes)             *successes             = g_recon.successful_reconnects;
    if (total_disconnected_ms) *total_disconnected_ms = g_recon.total_disconnected_ms;
}

void recon_shutdown(void)
{
    g_recon.initialized = false;
}
