/**
 * test_canopen.c
 * Unit tests for CANopen NMT, PDO, and heartbeat protocol handling.
 *
 * Tests:
 *   - NMT: send start -> node becomes operational
 *   - NMT: send stop -> node becomes stopped
 *   - PDO: send RPDO1 -> read TPDO1 matches expected data
 *   - Heartbeat: stop receiving -> timeout alert
 *
 * Build:
 *   gcc -I. -I../../firmware -DUNIT_TEST test_canopen.c -o test_canopen
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

/* ------------------------------------------------------------------ */
/*  CANopen types                                                     */
/* ------------------------------------------------------------------ */

/* CANopen NMT states */
typedef enum {
    NMT_BOOTUP    = 0,
    NMT_STOPPED   = 4,
    NMT_OPERATIONAL = 5,
    NMT_PRE_OPERATIONAL = 127
} nmt_state_t;

/* CANopen object dictionary entry */
typedef struct {
    uint16_t index;
    uint8_t  subindex;
    uint32_t value;
    size_t   size;      /* 1, 2, or 4 bytes */
} od_entry_t;

/* CAN frame */
typedef struct {
    uint32_t cob_id;    /* CAN identifier */
    uint8_t  data[8];
    uint8_t  dlc;
    uint32_t timestamp_ms;
} can_frame_t;

/* Node state */
typedef struct {
    nmt_state_t nmt_state;
    uint32_t    heartbeat_producer_ms;
    uint32_t    heartbeat_consumer_ms;
    uint32_t    last_heartbeat_ms;
    int         heartbeat_timeout;
    uint32_t    tpdo_data[4];   /* 4 TPDOs, each 4 bytes */
    uint32_t    rpdo_data[4];   /* 4 RPDOs, each 4 bytes */
    uint32_t    error_register;
} canopen_node_t;

/* CANopen bus simulator */
typedef struct {
    can_frame_t tx_queue[64];
    int         tx_count;
    can_frame_t rx_queue[64];
    int         rx_count;
    uint32_t    now_ms;
} canopen_bus_t;

/* ------------------------------------------------------------------ */
/*  DUT functions                                                     */
/* ------------------------------------------------------------------ */

void canopen_nmt_process(canopen_node_t *node, const can_frame_t *frame);
void canopen_pdo_process(canopen_node_t *node, const can_frame_t *frame,
                         canopen_bus_t *bus);
void canopen_heartbeat_check(canopen_node_t *node, uint32_t now_ms);
void canopen_send_nmt(canopen_bus_t *bus, nmt_state_t target_state);
void canopen_send_heartbeat(canopen_bus_t *bus, canopen_node_t *node);
void canopen_node_init(canopen_node_t *node);

/* ------------------------------------------------------------------ */
/*  DUT implementation                                                */
/* ------------------------------------------------------------------ */

/* CANopen COB-ID assignment */
#define COB_NMT           0x000
#define COB_NMT_NODE      0x700  /* + node ID */
#define COB_HEARTBEAT     0x700
#define COB_TPDO1         0x180
#define COB_RPDO1         0x200
#define COB_SYNC          0x080
#define COB_EMERGENCY     0x080

#define NODE_ID           0x01
#define HEARTBEAT_TIMEOUT_MS  500

void canopen_node_init(canopen_node_t *node)
{
    memset(node, 0, sizeof(*node));
    node->nmt_state = NMT_BOOTUP;
    node->heartbeat_producer_ms = 250;
    node->heartbeat_consumer_ms = HEARTBEAT_TIMEOUT_MS;
}

void canopen_send_nmt(canopen_bus_t *bus, nmt_state_t target_state)
{
    can_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.cob_id = COB_NMT;
    frame.dlc    = 2;
    frame.data[0] = (uint8_t)target_state;  /* command specifier */
    frame.data[1] = NODE_ID;                 /* node ID (0 = all) */
    frame.timestamp_ms = bus->now_ms;

    if (bus->rx_count < 64)
        bus->rx_queue[bus->rx_count++] = frame;
}

void canopen_send_heartbeat(canopen_bus_t *bus, canopen_node_t *node)
{
    can_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.cob_id = COB_HEARTBEAT | NODE_ID;
    frame.dlc    = 1;
    frame.data[0] = (uint8_t)node->nmt_state;
    frame.timestamp_ms = bus->now_ms;

    if (bus->tx_count < 64)
        bus->tx_queue[bus->tx_count++] = frame;
}

void canopen_nmt_process(canopen_node_t *node, const can_frame_t *frame)
{
    if (frame->cob_id != COB_NMT)
        return;
    if (frame->dlc < 2)
        return;
    if (frame->data[1] != 0 && frame->data[1] != NODE_ID)
        return;  /* not addressed to us */

    uint8_t cmd = frame->data[0];
    switch (cmd) {
    case 0x01: /* Start */
        node->nmt_state = NMT_OPERATIONAL;
        break;
    case 0x02: /* Stop */
        node->nmt_state = NMT_STOPPED;
        break;
    case 0x80: /* Enter pre-operational */
        node->nmt_state = NMT_PRE_OPERATIONAL;
        break;
    case 0x81: /* Reset node */
        node->nmt_state = NMT_BOOTUP;
        break;
    default:
        break;
    }
}

void canopen_pdo_process(canopen_node_t *node, const can_frame_t *frame,
                         canopen_bus_t *bus)
{
    (void)bus;

    /* Check if this is an RPDO addressed to us */
    uint32_t rpdo_base = COB_RPDO1 + NODE_ID;
    if (frame->cob_id == rpdo_base) {
        /* RPDO1 received */
        uint32_t val = 0;
        for (int i = 0; i < 4 && i < frame->dlc; i++)
            val |= (uint32_t)frame->data[i] << (i * 8);
        node->rpdo_data[0] = val;

        /* Echo back as TPDO1 */
        if (bus->tx_count < 64) {
            can_frame_t tx;
            memset(&tx, 0, sizeof(tx));
            tx.cob_id = COB_TPDO1 | NODE_ID;
            tx.dlc    = 4;
            for (int i = 0; i < 4; i++)
                tx.data[i] = (uint8_t)(val >> (i * 8));
            tx.timestamp_ms = bus->now_ms;
            bus->tx_queue[bus->tx_count++] = tx;
        }
        return;
    }

    /* Check if this is a TPDO from another node */
    uint32_t tpdo_base = COB_TPDO1;
    if ((frame->cob_id & 0x7F) != NODE_ID &&
        (frame->cob_id & 0x780) == tpdo_base) {
        /* Store received TPDO */
        uint32_t val = 0;
        for (int i = 0; i < 4 && i < frame->dlc; i++)
            val |= (uint32_t)frame->data[i] << (i * 8);
        node->tpdo_data[0] = val;
    }
}

void canopen_heartbeat_check(canopen_node_t *node, uint32_t now_ms)
{
    if (node->heartbeat_consumer_ms == 0)
        return;  /* not monitoring */

    uint32_t elapsed = now_ms - node->last_heartbeat_ms;
    if (elapsed >= node->heartbeat_consumer_ms) {
        node->heartbeat_timeout = 1;
    } else {
        node->heartbeat_timeout = 0;
    }
}

/* Process received heartbeat frame */
void canopen_heartbeat_receive(canopen_node_t *node, const can_frame_t *frame)
{
    if (frame->cob_id != (COB_HEARTBEAT | NODE_ID))
        return;
    if (frame->dlc < 1)
        return;

    node->nmt_state = (nmt_state_t)frame->data[0];
    node->last_heartbeat_ms = frame->timestamp_ms;
}

/* ------------------------------------------------------------------ */
/*  Test utilities                                                    */
/* ------------------------------------------------------------------ */

static int test_passed = 0;
static int test_failed = 0;

#define TEST_START(name)  do { printf("  TEST %-55s ", name); } while(0)
#define TEST_PASS()       do { printf("PASS\n"); test_passed++; } while(0)
#define TEST_FAIL(msg)    do { printf("FAIL: %s\n", msg); test_failed++; } while(0)

/* ------------------------------------------------------------------ */
/*  Tests                                                             */
/* ------------------------------------------------------------------ */

static void test_nmt_start(void)
{
    TEST_START("NMT: send start -> node becomes operational");
    canopen_node_t node;
    canopen_bus_t bus;
    memset(&bus, 0, sizeof(bus));

    canopen_node_init(&node);
    node.nmt_state = NMT_PRE_OPERATIONAL;

    canopen_send_nmt(&bus, NMT_OPERATIONAL);
    if (bus.rx_count < 1) { TEST_FAIL("no NMT frame"); return; }

    canopen_nmt_process(&node, &bus.rx_queue[0]);

    if (node.nmt_state == NMT_OPERATIONAL)
        TEST_PASS();
    else
        TEST_FAIL("expected NMT_OPERATIONAL");
}

static void test_nmt_stop(void)
{
    TEST_START("NMT: send stop -> node becomes stopped");
    canopen_node_t node;
    canopen_bus_t bus;
    memset(&bus, 0, sizeof(bus));

    canopen_node_init(&node);
    node.nmt_state = NMT_OPERATIONAL;

    canopen_send_nmt(&bus, NMT_STOPPED);
    if (bus.rx_count < 1) { TEST_FAIL("no NMT frame"); return; }

    canopen_nmt_process(&node, &bus.rx_queue[0]);

    if (node.nmt_state == NMT_STOPPED)
        TEST_PASS();
    else
        TEST_FAIL("expected NMT_STOPPED");
}

static void test_nmt_reset(void)
{
    TEST_START("NMT: send reset -> node returns to bootup");
    canopen_node_t node;
    canopen_bus_t bus;
    memset(&bus, 0, sizeof(bus));

    canopen_node_init(&node);
    node.nmt_state = NMT_OPERATIONAL;

    can_frame_t reset_frame;
    memset(&reset_frame, 0, sizeof(reset_frame));
    reset_frame.cob_id = COB_NMT;
    reset_frame.dlc    = 2;
    reset_frame.data[0] = 0x81;  /* reset command */
    reset_frame.data[1] = NODE_ID;

    canopen_nmt_process(&node, &reset_frame);

    if (node.nmt_state == NMT_BOOTUP)
        TEST_PASS();
    else
        TEST_FAIL("expected NMT_BOOTUP");
}

static void test_pdo_rpdo1_tpdo1(void)
{
    TEST_START("PDO: send RPDO1 -> read TPDO1 matches");
    canopen_node_t node;
    canopen_bus_t bus;
    memset(&bus, 0, sizeof(bus));

    canopen_node_init(&node);
    bus.now_ms = 1000;

    /* Construct RPDO1 frame */
    can_frame_t rpdo;
    memset(&rpdo, 0, sizeof(rpdo));
    rpdo.cob_id = COB_RPDO1 | NODE_ID;  /* 0x201 */
    rpdo.dlc    = 4;
    uint32_t test_val = 0xDEADBEEF;
    for (int i = 0; i < 4; i++)
        rpdo.data[i] = (uint8_t)(test_val >> (i * 8));
    rpdo.timestamp_ms = 1000;

    canopen_pdo_process(&node, &rpdo, &bus);

    /* Check RPDO stored */
    if (node.rpdo_data[0] != test_val)
        TEST_FAIL("RPDO data mismatch");

    /* Check TPDO was sent in response */
    if (bus.tx_count < 1)
        TEST_FAIL("no TPDO sent");

    can_frame_t *tpdo = &bus.tx_queue[0];
    uint32_t tpdo_val = 0;
    for (int i = 0; i < 4 && i < tpdo->dlc; i++)
        tpdo_val |= (uint32_t)tpdo->data[i] << (i * 8);

    if (tpdo_val == test_val)
        TEST_PASS();
    else
        TEST_FAIL("TPDO data does not match RPDO input");
}

static void test_pdo_rpdo_different_node(void)
{
    TEST_START("PDO: RPDO addressed to different node is ignored");
    canopen_node_t node;
    canopen_bus_t bus;
    memset(&bus, 0, sizeof(bus));

    canopen_node_init(&node);

    /* RPDO addressed to node 2 */
    can_frame_t rpdo;
    memset(&rpdo, 0, sizeof(rpdo));
    rpdo.cob_id = COB_RPDO1 | 0x02;
    rpdo.dlc    = 4;
    rpdo.data[0] = 0x42;

    canopen_pdo_process(&node, &rpdo, &bus);

    if (node.rpdo_data[0] == 0 && bus.tx_count == 0)
        TEST_PASS();
    else
        TEST_FAIL("RPDO not ignored");
}

static void test_heartbeat_timeout(void)
{
    TEST_START("Heartbeat: stop receiving -> timeout alert");
    canopen_node_t node;
    canopen_bus_t bus;
    memset(&bus, 0, sizeof(bus));

    canopen_node_init(&node);
    node.last_heartbeat_ms = 0;

    /* Receive one heartbeat */
    can_frame_t hb;
    memset(&hb, 0, sizeof(hb));
    hb.cob_id = COB_HEARTBEAT | NODE_ID;
    hb.dlc    = 1;
    hb.data[0] = NMT_OPERATIONAL;
    hb.timestamp_ms = 100;
    canopen_heartbeat_receive(&node, &hb);

    canopen_heartbeat_check(&node, 100);
    if (node.heartbeat_timeout) { TEST_FAIL("premature timeout"); return; }

    /* Advance time beyond timeout */
    canopen_heartbeat_check(&node, 100 + HEARTBEAT_TIMEOUT_MS + 100);

    if (node.heartbeat_timeout)
        TEST_PASS();
    else
        TEST_FAIL("heartbeat timeout not detected");
}

static void test_heartbeat_normal_operation(void)
{
    TEST_START("Heartbeat: regular updates prevent timeout");
    canopen_node_t node;
    memset(&node, 0, sizeof(node));

    canopen_node_init(&node);
    node.last_heartbeat_ms = 0;

    /* Periodic heartbeats */
    for (uint32_t t = 0; t <= 2000; t += 200) {
        can_frame_t hb;
        memset(&hb, 0, sizeof(hb));
        hb.cob_id = COB_HEARTBEAT | NODE_ID;
        hb.dlc    = 1;
        hb.data[0] = NMT_OPERATIONAL;
        hb.timestamp_ms = t;
        canopen_heartbeat_receive(&node, &hb);
        canopen_heartbeat_check(&node, t);
    }

    if (!node.heartbeat_timeout)
        TEST_PASS();
    else
        TEST_FAIL("unexpected heartbeat timeout");
}

/* ------------------------------------------------------------------ */
/*  Main                                                              */
/* ------------------------------------------------------------------ */

int main(void)
{
    printf("=== CANopen Protocol Unit Tests ===\n\n");

    printf("--- NMT ---\n");
    test_nmt_start();
    test_nmt_stop();
    test_nmt_reset();

    printf("\n--- PDO ---\n");
    test_pdo_rpdo1_tpdo1();
    test_pdo_rpdo_different_node();

    printf("\n--- Heartbeat ---\n");
    test_heartbeat_timeout();
    test_heartbeat_normal_operation();

    printf("\nResults: %d passed, %d failed\n", test_passed, test_failed);
    return test_failed > 0 ? 1 : 0;
}
