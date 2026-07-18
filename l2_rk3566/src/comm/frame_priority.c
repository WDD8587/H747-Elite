/**
 * @file    frame_priority.c
 * @brief   Priority queuing for IPC frames
 * @details 3 priority levels:
 *          - HIGH:   safety, ESTOP, heartbeat
 *          - MEDIUM: velocity command, odometry
 *          - LOW:    telemetry, log, configuration
 *
 *          HIGH preempts MEDIUM preempts LOW.  Each level has its own
 *          FIFO queue.  Starvation prevention: LOW frames are guaranteed
 *          at least one transmission every 500ms.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <time.h>

/* ------------------------------------------------------------------ */
/*  Constants                                                          */
/* ------------------------------------------------------------------ */
#define FP_MAX_QUEUE_LENGTH     32
#define FP_MAX_FRAME_SIZE       2048
#define FP_STARVATION_TIMEOUT_MS 500   /* max time before LOW gets a slot */
#define FP_PRIORITY_LEVELS      3

/* ------------------------------------------------------------------ */
/*  Priority levels                                                    */
/* ------------------------------------------------------------------ */
typedef enum {
    FP_PRIORITY_LOW    = 0,
    FP_PRIORITY_MEDIUM = 1,
    FP_PRIORITY_HIGH   = 2,
    FP_PRIORITY_COUNT  = 3
} fp_priority_t;

static const char *fp_priority_name(fp_priority_t p)
{
    switch (p) {
    case FP_PRIORITY_LOW:    return "LOW";
    case FP_PRIORITY_MEDIUM: return "MEDIUM";
    case FP_PRIORITY_HIGH:   return "HIGH";
    default:                 return "?";
    }
}

/* ------------------------------------------------------------------ */
/*  Frame                                                              */
/* ------------------------------------------------------------------ */
typedef struct {
    uint8_t  *data;
    uint16_t  len;
    fp_priority_t priority;
    uint32_t  enqueue_ms;
} fp_frame_t;

/* ------------------------------------------------------------------ */
/*  Queue (FIFO per priority)                                          */
/* ------------------------------------------------------------------ */
typedef struct {
    fp_frame_t frames[FP_MAX_QUEUE_LENGTH];
    int        head;
    int        tail;
    int        count;
    uint32_t   last_dequeue_ms;   /* last time a frame was dequeued from this queue */
    uint32_t   total_enqueued;
    uint32_t   total_dequeued;
    uint32_t   total_dropped;
} fp_queue_t;

/* ------------------------------------------------------------------ */
/*  Priority manager                                                   */
/* ------------------------------------------------------------------ */
typedef struct {
    fp_queue_t queues[FP_PRIORITY_COUNT];
    uint32_t   last_low_dequeue_ms;  /* last time LOW was serviced */
    uint32_t   starve_check_ms;      /* next time to force LOW */

    /* Tx callback */
    int (*tx_fn)(const uint8_t *data, uint16_t len);

    bool initialized;
} fp_manager_t;

static fp_manager_t g_fp;

/* ------------------------------------------------------------------ */
/*  Time helper                                                        */
/* ------------------------------------------------------------------ */
static uint32_t fp_get_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

/* ------------------------------------------------------------------ */
/*  Queue internals                                                    */
/* ------------------------------------------------------------------ */

static void queue_init(fp_queue_t *q)
{
    memset(q, 0, sizeof(fp_queue_t));
}

static bool queue_is_full(const fp_queue_t *q)
{
    return q->count >= FP_MAX_QUEUE_LENGTH;
}

static bool queue_is_empty(const fp_queue_t *q)
{
    return q->count == 0;
}

static int queue_enqueue(fp_queue_t *q, const uint8_t *data, uint16_t len,
                          fp_priority_t priority)
{
    if (queue_is_full(q))
        return -ENOSPC;

    fp_frame_t *f = &q->frames[q->tail];
    f->data = (uint8_t *)malloc(len);
    if (!f->data) return -ENOMEM;

    memcpy(f->data, data, len);
    f->len = len;
    f->priority = priority;
    f->enqueue_ms = fp_get_ms();

    q->tail = (q->tail + 1) % FP_MAX_QUEUE_LENGTH;
    q->count++;
    q->total_enqueued++;
    return 0;
}

static int queue_dequeue(fp_queue_t *q, fp_frame_t *out)
{
    if (queue_is_empty(q))
        return -EAGAIN;

    fp_frame_t *f = &q->frames[q->head];
    out->data     = f->data;
    out->len      = f->len;
    out->priority = f->priority;
    out->enqueue_ms = f->enqueue_ms;

    f->data = NULL; /* ownership transferred */

    q->head = (q->head + 1) % FP_MAX_QUEUE_LENGTH;
    q->count--;
    q->total_dequeued++;
    q->last_dequeue_ms = fp_get_ms();
    return 0;
}

static int queue_peek(const fp_queue_t *q, fp_frame_t *out)
{
    if (queue_is_empty(q))
        return -EAGAIN;

    const fp_frame_t *f = &q->frames[q->head];
    out->data     = f->data;
    out->len      = f->len;
    out->priority = f->priority;
    out->enqueue_ms = f->enqueue_ms;
    return 0;
}

static void queue_drop_oldest(fp_queue_t *q)
{
    if (queue_is_empty(q)) return;

    fp_frame_t *f = &q->frames[q->head];
    if (f->data) {
        free(f->data);
        f->data = NULL;
    }
    q->head = (q->head + 1) % FP_MAX_QUEUE_LENGTH;
    q->count--;
    q->total_dropped++;
}

static void queue_flush(fp_queue_t *q)
{
    while (!queue_is_empty(q)) {
        fp_frame_t f;
        if (queue_dequeue(q, &f) == 0 && f.data) {
            free(f.data);
        }
    }
}

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

int fp_init(int (*tx_callback)(const uint8_t *, uint16_t))
{
    memset(&g_fp, 0, sizeof(g_fp));
    for (int i = 0; i < FP_PRIORITY_COUNT; i++)
        queue_init(&g_fp.queues[i]);

    g_fp.tx_fn = tx_callback;
    g_fp.last_low_dequeue_ms = fp_get_ms();
    g_fp.starve_check_ms = fp_get_ms() + FP_STARVATION_TIMEOUT_MS;
    g_fp.initialized = true;
    return 0;
}

/**
 * fp_send - queue a frame at the specified priority
 * @data:     frame payload
 * @len:      payload length
 * @priority: priority level
 *
 * Returns 0 on success (queued), negative on error.
 * Drops oldest frame from the target queue if full.
 */
int fp_send(const uint8_t *data, uint16_t len, fp_priority_t priority)
{
    if (!g_fp.initialized) return -EPERM;
    if (len > FP_MAX_FRAME_SIZE) return -EMSGSIZE;
    if (priority < 0 || priority >= FP_PRIORITY_COUNT)
        return -EINVAL;

    fp_queue_t *q = &g_fp.queues[priority];

    /* If queue is full, drop oldest to make room */
    if (queue_is_full(q)) {
        queue_drop_oldest(q);
    }

    return queue_enqueue(q, data, len, priority);
}

/**
 * fp_send_high - convenience wrapper for HIGH priority
 */
int fp_send_high(const uint8_t *data, uint16_t len)
{
    return fp_send(data, len, FP_PRIORITY_HIGH);
}

/**
 * fp_send_medium - convenience wrapper for MEDIUM priority
 */
int fp_send_medium(const uint8_t *data, uint16_t len)
{
    return fp_send(data, len, FP_PRIORITY_MEDIUM);
}

/**
 * fp_send_low - convenience wrapper for LOW priority
 */
int fp_send_low(const uint8_t *data, uint16_t len)
{
    return fp_send(data, len, FP_PRIORITY_LOW);
}

/**
 * fp_dequeue - dequeue the highest priority frame for transmission
 * @out: output frame (caller must free out->data)
 *
 * Priority order: HIGH > MEDIUM > LOW.
 * Starvation prevention: if LOW hasn't been serviced in 500ms,
 * it gets priority.
 *
 * Returns 0 on success, -EAGAIN if all queues empty.
 */
int fp_dequeue(fp_frame_t *out)
{
    if (!g_fp.initialized) return -EPERM;
    if (!out) return -EINVAL;

    uint32_t now = fp_get_ms();
    bool force_low = (now >= g_fp.starve_check_ms);

    /* Determine which queue to service */
    int priority_order[FP_PRIORITY_COUNT];

    if (force_low && !queue_is_empty(&g_fp.queues[FP_PRIORITY_LOW])) {
        /* Starvation avoidance: service LOW first */
        priority_order[0] = FP_PRIORITY_LOW;
        priority_order[1] = FP_PRIORITY_HIGH;
        priority_order[2] = FP_PRIORITY_MEDIUM;
        g_fp.starve_check_ms = now + FP_STARVATION_TIMEOUT_MS;
    } else {
        /* Normal priority order */
        priority_order[0] = FP_PRIORITY_HIGH;
        priority_order[1] = FP_PRIORITY_MEDIUM;
        priority_order[2] = FP_PRIORITY_LOW;
    }

    for (int i = 0; i < FP_PRIORITY_COUNT; i++) {
        fp_queue_t *q = &g_fp.queues[priority_order[i]];
        if (queue_dequeue(q, out) == 0) {
            if (priority_order[i] == FP_PRIORITY_LOW)
                g_fp.last_low_dequeue_ms = now;
            return 0;
        }
    }

    /* All queues empty — reset starvation timer */
    g_fp.starve_check_ms = now + FP_STARVATION_TIMEOUT_MS;
    return -EAGAIN;
}

/**
 * fp_transmit - dequeue and transmit one frame via the callback
 *               (non-blocking)
 *
 * Returns 1 if a frame was transmitted, 0 if nothing to send, negative on err.
 */
int fp_transmit(void)
{
    if (!g_fp.initialized) return -EPERM;

    fp_frame_t frame;
    int rc = fp_dequeue(&frame);
    if (rc != 0) return 0;

    int tx_rc = -1;
    if (g_fp.tx_fn)
        tx_rc = g_fp.tx_fn(frame.data, frame.len);

    free(frame.data);

    return (tx_rc == 0) ? 1 : tx_rc;
}

/**
 * fp_get_queue_info - get current queue occupancy
 */
void fp_get_queue_info(int *high_count, int *med_count, int *low_count)
{
    if (high_count) *high_count = g_fp.queues[FP_PRIORITY_HIGH].count;
    if (med_count)  *med_count  = g_fp.queues[FP_PRIORITY_MEDIUM].count;
    if (low_count)  *low_count  = g_fp.queues[FP_PRIORITY_LOW].count;
}

/**
 * fp_get_stats - get per-queue statistics
 */
void fp_get_stats(uint32_t *high_enq, uint32_t *high_deq, uint32_t *high_drop,
                   uint32_t *med_enq,  uint32_t *med_deq,  uint32_t *med_drop,
                   uint32_t *low_enq,  uint32_t *low_deq,  uint32_t *low_drop)
{
    if (high_enq) *high_enq = g_fp.queues[FP_PRIORITY_HIGH].total_enqueued;
    if (high_deq) *high_deq = g_fp.queues[FP_PRIORITY_HIGH].total_dequeued;
    if (high_drop)*high_drop = g_fp.queues[FP_PRIORITY_HIGH].total_dropped;
    if (med_enq)  *med_enq  = g_fp.queues[FP_PRIORITY_MEDIUM].total_enqueued;
    if (med_deq)  *med_deq  = g_fp.queues[FP_PRIORITY_MEDIUM].total_dequeued;
    if (med_drop) *med_drop = g_fp.queues[FP_PRIORITY_MEDIUM].total_dropped;
    if (low_enq)  *low_enq  = g_fp.queues[FP_PRIORITY_LOW].total_enqueued;
    if (low_deq)  *low_deq  = g_fp.queues[FP_PRIORITY_LOW].total_dequeued;
    if (low_drop) *low_drop = g_fp.queues[FP_PRIORITY_LOW].total_dropped;
}

void fp_flush(void)
{
    for (int i = 0; i < FP_PRIORITY_COUNT; i++)
        queue_flush(&g_fp.queues[i]);
}

void fp_shutdown(void)
{
    fp_flush();
    g_fp.initialized = false;
}

/* ------------------------------------------------------------------ */
/*  Self-test                                                          */
/* ------------------------------------------------------------------ */
#ifdef TEST_FP
int main(void)
{
    fp_init(NULL);

    uint8_t safety[] = "ESTOP!";
    uint8_t velo[]   = "VELOCITY_CMD";
    uint8_t log[]    = "LOG: temperature=42.5";

    fp_send_high(safety, sizeof(safety));
    fp_send_medium(velo, sizeof(velo));
    fp_send_low(log, sizeof(log));

    fp_frame_t frame;
    while (fp_dequeue(&frame) == 0) {
        printf("Dequeued [%s]: %.*s\n",
               fp_priority_name(frame.priority),
               frame.len, (const char *)frame.data);
        free(frame.data);
    }

    fp_shutdown();
    return 0;
}
#endif /* TEST_FP */
