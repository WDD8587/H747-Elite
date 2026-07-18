/**
 * test_qos.c
 * Unit tests for communication QoS layer: retransmit, dedup, reorder.
 *
 * Tests:
 *   - Retransmit: drop frame 3 -> expect retransmit within 150 ms
 *   - Duplicate: send frame twice -> expect deduplication
 *   - Out-of-order: send 1, 3, 2 -> expect reorder to 1, 2, 3
 *
 * Build:
 *   gcc -I. -I../../firmware -DUNIT_TEST test_qos.c -o test_qos
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

/* ------------------------------------------------------------------ */
/*  Types                                                             */
/* ------------------------------------------------------------------ */

#define MAX_PACKET_SIZE      128
#define SEQ_WINDOW_SIZE      32
#define MAX_PENDING_FRAMES   16
#define RETRANSMIT_TIMEOUT_MS  150

typedef enum {
    QOS_RELIABLE    = 0,
    QOS_BEST_EFFORT = 1
} qos_mode_t;

typedef struct {
    uint16_t seq;               /* sequence number */
    uint8_t  data[MAX_PACKET_SIZE];
    uint16_t len;
    uint32_t timestamp_ms;
    uint8_t  retransmitted;     /* 1 if this is a retransmission */
} qos_frame_t;

/* Pending frame for retransmit tracking */
typedef struct {
    uint16_t  seq;
    uint32_t  last_sent_ms;
    uint8_t   data[MAX_PACKET_SIZE];
    uint16_t  len;
    int       acked;
    int       retry_count;
    int       active;
} pending_frame_t;

/* Receive window for dedup and reorder */
typedef struct {
    uint16_t  expected_seq;          /* next expected sequence */
    qos_frame_t window[SEQ_WINDOW_SIZE];
    int       have[SEQ_WINDOW_SIZE]; /* 1 = slot filled */
    uint16_t  window_start;          /* seq of window[0] */
    int       count;
} recv_window_t;

/* QoS sender state */
typedef struct {
    pending_frame_t pending[MAX_PENDING_FRAMES];
    uint16_t        next_tx_seq;
    uint32_t        now_ms;
} qos_sender_t;

/* QoS receiver state */
typedef struct {
    recv_window_t   window;
    uint16_t        last_delivered_seq;
    qos_frame_t     delivered[SEQ_WINDOW_SIZE];
    int             delivered_count;
} qos_receiver_t;

/* DUT function prototypes */
void qos_sender_init(qos_sender_t *s);
void qos_sender_send(qos_sender_t *s, const uint8_t *data, uint16_t len,
                     qos_frame_t *out);
int  qos_sender_check_retransmit(qos_sender_t *s, qos_frame_t *out,
                                 uint32_t timeout_ms);
void qos_sender_ack(qos_sender_t *s, uint16_t seq);

void qos_receiver_init(qos_receiver_t *r);
int  qos_receiver_process(qos_receiver_t *r, const qos_frame_t *frame,
                          qos_frame_t *delivered);

/* ------------------------------------------------------------------ */
/*  DUT implementation                                                */
/* ------------------------------------------------------------------ */

void qos_sender_init(qos_sender_t *s)
{
    memset(s, 0, sizeof(*s));
    s->next_tx_seq = 1;
}

void qos_sender_send(qos_sender_t *s, const uint8_t *data, uint16_t len,
                     qos_frame_t *out)
{
    uint16_t seq = s->next_tx_seq++;

    out->seq = seq;
    out->len = (len > MAX_PACKET_SIZE) ? MAX_PACKET_SIZE : len;
    memcpy(out->data, data, out->len);
    out->timestamp_ms = s->now_ms;
    out->retransmitted = 0;

    /* Add to pending for retransmit tracking */
    for (int i = 0; i < MAX_PENDING_FRAMES; i++) {
        if (!s->pending[i].active) {
            s->pending[i].seq          = seq;
            s->pending[i].last_sent_ms = s->now_ms;
            s->pending[i].len          = out->len;
            memcpy(s->pending[i].data, data, out->len);
            s->pending[i].acked      = 0;
            s->pending[i].retry_count = 0;
            s->pending[i].active     = 1;
            break;
        }
    }
}

int qos_sender_check_retransmit(qos_sender_t *s, qos_frame_t *out,
                                uint32_t timeout_ms)
{
    for (int i = 0; i < MAX_PENDING_FRAMES; i++) {
        if (!s->pending[i].active || s->pending[i].acked)
            continue;
        uint32_t elapsed = s->now_ms - s->pending[i].last_sent_ms;
        if (elapsed >= timeout_ms && s->pending[i].retry_count < 3) {
            out->seq            = s->pending[i].seq;
            out->len            = s->pending[i].len;
            memcpy(out->data, s->pending[i].data, out->len);
            out->timestamp_ms   = s->now_ms;
            out->retransmitted  = 1;

            s->pending[i].last_sent_ms = s->now_ms;
            s->pending[i].retry_count++;
            return 1;
        }
    }
    return 0;
}

void qos_sender_ack(qos_sender_t *s, uint16_t seq)
{
    for (int i = 0; i < MAX_PENDING_FRAMES; i++) {
        if (s->pending[i].active && s->pending[i].seq == seq) {
            s->pending[i].acked = 1;
            s->pending[i].active = 0;
            break;
        }
    }
}

void qos_receiver_init(qos_receiver_t *r)
{
    memset(r, 0, sizeof(*r));
    r->window.expected_seq = 1;
    r->window.window_start = 1;
}

int qos_receiver_process(qos_receiver_t *r, const qos_frame_t *frame,
                         qos_frame_t *delivered)
{
    uint16_t seq = frame->seq;
    recv_window_t *win = &r->window;

    /* Dup detection: already delivered */
    if (seq <= r->last_delivered_seq) {
        return 0;  /* duplicate */
    }

    /* Check if already in window */
    if (seq >= win->window_start &&
        seq < win->window_start + SEQ_WINDOW_SIZE) {
        int idx = (seq - win->window_start) % SEQ_WINDOW_SIZE;
        if (win->have[idx]) {
            return 0;  /* duplicate in window */
        }
        win->window[idx] = *frame;
        win->have[idx]   = 1;
        win->count++;
    } else if (seq >= win->window_start + SEQ_WINDOW_SIZE) {
        /* Sequence too far ahead: advance window */
        /* For simplicity, reset window */
        memset(win->have, 0, sizeof(win->have));
        win->window_start = seq;
        win->count = 0;
        win->expected_seq = seq;
        int idx = 0;
        win->window[idx] = *frame;
        win->have[idx]   = 1;
        win->count       = 1;
    } else {
        /* Old sequence, discard */
        return 0;
    }

    /* Deliver in-order frames */
    int delivered_any = 0;
    while (win->have[win->expected_seq - win->window_start]) {
        int idx = win->expected_seq - win->window_start;
        if (idx < 0 || idx >= SEQ_WINDOW_SIZE)
            break;

        *delivered = win->window[idx];
        r->last_delivered_seq = win->expected_seq;
        win->have[idx] = 0;
        win->count--;
        win->expected_seq++;
        delivered_any = 1;
        break;  /* deliver one per call */
    }

    return delivered_any;
}

/* ------------------------------------------------------------------ */
/*  Test utilities                                                    */
/* ------------------------------------------------------------------ */

static int test_passed = 0;
static int test_failed = 0;

#define TEST_START(name)  do { printf("  TEST %-55s ", name); } while(0)
#define TEST_PASS()       do { printf("PASS\n"); test_passed++; } while(0)
#define TEST_FAIL(msg)    do { printf("FAIL: %s\n", msg); test_failed++; } while(0)

static uint8_t make_data[8];

/* ------------------------------------------------------------------ */
/*  Tests                                                             */
/* ------------------------------------------------------------------ */

static void test_retransmit_drop_frame_3(void)
{
    TEST_START("Retransmit: drop frame 3 -> retransmit within 150ms");
    qos_sender_t sender;
    qos_sender_init(&sender);

    qos_frame_t tx;
    uint8_t data[] = "hello";

    /* Send frames 1, 2, 3, 4 */
    sender.now_ms = 0;
    qos_sender_send(&sender, data, 5, &tx);
    sender.now_ms = 10;
    qos_sender_send(&sender, data, 5, &tx);
    sender.now_ms = 20;
    qos_sender_send(&sender, data, 5, &tx);  /* frame 3 */
    sender.now_ms = 30;
    qos_sender_send(&sender, data, 5, &tx);

    /* Ack frames 1, 2, 4 but not 3 */
    qos_sender_ack(&sender, 1);
    qos_sender_ack(&sender, 2);
    qos_sender_ack(&sender, 4);

    /* Advance time past retransmit timeout */
    sender.now_ms = 200;

    int ret = qos_sender_check_retransmit(&sender, &tx, RETRANSMIT_TIMEOUT_MS);
    if (ret && tx.seq == 3 && tx.retransmitted)
        TEST_PASS();
    else
        TEST_FAIL("frame 3 was not retransmitted");
}

static void test_retransmit_no_false_positive(void)
{
    TEST_START("Retransmit: no retransmit when all ACKed");
    qos_sender_t sender;
    qos_sender_init(&sender);

    qos_frame_t tx;
    uint8_t data[] = "test";

    sender.now_ms = 0;
    qos_sender_send(&sender, data, 4, &tx);
    sender.now_ms = 10;
    qos_sender_send(&sender, data, 4, &tx);

    /* Ack all */
    qos_sender_ack(&sender, 1);
    qos_sender_ack(&sender, 2);

    sender.now_ms = 500;
    int ret = qos_sender_check_retransmit(&sender, &tx, RETRANSMIT_TIMEOUT_MS);
    if (!ret)
        TEST_PASS();
    else
        TEST_FAIL("unexpected retransmit after all ACKed");
}

static void test_dedup_same_frame_twice(void)
{
    TEST_START("Duplicate: send frame twice -> dedup");
    qos_receiver_t recv;
    qos_receiver_init(&recv);

    qos_frame_t f1, f2, delivered;

    /* Frame 1 */
    memset(&f1, 0, sizeof(f1));
    f1.seq = 1;
    f1.len = 4;
    memcpy(f1.data, "data", 4);

    /* Process first time */
    int r1 = qos_receiver_process(&recv, &f1, &delivered);
    if (!r1) { TEST_FAIL("first copy not delivered"); return; }

    /* Process same frame again */
    int r2 = qos_receiver_process(&recv, &f1, &delivered);
    if (r2 == 0 && recv.last_delivered_seq == 1)
        TEST_PASS();
    else
        TEST_FAIL("duplicate was not suppressed");
}

static void test_out_of_order(void)
{
    TEST_START("Out-of-order: send 1, 3, 2 -> reorder to 1, 2, 3");
    qos_receiver_t recv;
    qos_receiver_init(&recv);

    qos_frame_t frames[4], delivered;
    memset(frames, 0, sizeof(frames));

    for (int i = 1; i <= 3; i++) {
        frames[i].seq = (uint16_t)i;
        frames[i].len = 4;
        memcpy(frames[i].data, "data", 4);
    }

    /* Deliver frame 1 (in order) */
    int r = qos_receiver_process(&recv, &frames[1], &delivered);
    if (!r || delivered.seq != 1) { TEST_FAIL("frame 1 not delivered"); return; }

    /* Deliver frame 3 (out of order, buffered) */
    r = qos_receiver_process(&recv, &frames[3], &delivered);
    if (r != 0) { TEST_FAIL("frame 3 should have been buffered"); return; }

    /* Deliver frame 2 (now 2 and 3 should come in order) */
    r = qos_receiver_process(&recv, &frames[2], &delivered);
    if (!r || delivered.seq != 2) { TEST_FAIL("frame 2 not delivered"); return; }

    /* Next call should deliver frame 3 */
    r = qos_receiver_process(&recv, &frames[3], &delivered);
    if (!r || delivered.seq != 3) { TEST_FAIL("frame 3 not delivered after reorder"); return; }

    TEST_PASS();
}

static void test_seq_number_wraparound(void)
{
    TEST_START("Sequence number wraparound handling");
    qos_sender_t sender;
    qos_sender_init(&sender);

    qos_frame_t tx;
    uint8_t data[] = "wrap";

    /* Send many frames to test wraparound */
    sender.next_tx_seq = 65530;  /* near wraparound */
    for (uint32_t i = 0; i < 10; i++) {
        sender.now_ms = i * 10;
        qos_sender_send(&sender, data, 4, &tx);
    }

    /* Verify sequences */
    if (tx.seq == 4)  /* 65530, 65531, 65532, 65533, ... -> wraps to 0,1,2,3,4 */
        TEST_PASS();
    else
        TEST_FAIL("sequence wraparound not correct");
}

/* ------------------------------------------------------------------ */
/*  Main                                                              */
/* ------------------------------------------------------------------ */

int main(void)
{
    printf("=== Communication QoS Unit Tests ===\n\n");

    printf("--- Retransmit ---\n");
    test_retransmit_drop_frame_3();
    test_retransmit_no_false_positive();

    printf("\n--- Deduplication ---\n");
    test_dedup_same_frame_twice();

    printf("\n--- Reorder ---\n");
    test_out_of_order();

    printf("\n--- Sequence Numbers ---\n");
    test_seq_number_wraparound();

    printf("\nResults: %d passed, %d failed\n", test_passed, test_failed);
    return test_failed > 0 ? 1 : 0;
}
