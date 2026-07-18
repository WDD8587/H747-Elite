/**
 * @file    canopen.c
 * @brief   CANopen protocol stack for motor drives (NMT master)
 * @details Implements:
 *          - NMT master (start/stop/reset nodes)
 *          - PDO mapping (RPDO1: target velocity, TPDO1: actual velocity+current)
 *          - SDO for configuration (PID gains, limits)
 *          - Heartbeat consumer (100ms interval, 300ms timeout -> alert)
 *          - Emergency message handler
 *
 *          CAN bus: 250 kbps (default), 11-bit CAN ID (CAN 2.0A)
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
/*  CANopen constants                                                  */
/* ------------------------------------------------------------------ */
#define CO_NMT_START_NODE       0x01
#define CO_NMT_STOP_NODE        0x02
#define CO_NMT_PREOP            0x80
#define CO_NMT_RESET_NODE       0x81
#define CO_NMT_RESET_COMM       0x82

#define CO_NMT_CS_PREFIX        0x0000

#define CO_SDO_REQUEST          0x600
#define CO_SDO_RESPONSE         0x580

#define CO_NODE_ID_DEFAULT      0x01

/* Heartbeat */
#define CO_HEARTBEAT_TIME_MS    100
#define CO_HEARTBEAT_TIMEOUT_MS 300

/* PDO */
#define CO_RPDO1_COBID          0x200       /* receive PDO 1 */
#define CO_TPDO1_COBID          0x180       /* transmit PDO 1 */
#define CO_EMCY_COBID           0x080       /* emergency */

/* SDO expedited upload/download */
#define CO_SDO_CMD_UPLOAD_REQ   0x40
#define CO_SDO_CMD_DOWNLOAD_REQ 0x22
#define CO_SDO_CMD_UPLOAD_RESP  0x42
#define CO_SDO_CMD_DOWNLOAD_RESP 0x60
#define CO_SDO_CMD_ABORT        0x80

#define CO_BAUD_RATE            250000

/* Motor drive object dictionary entries */
#define OD_INDEX_DEVICE_TYPE        0x1000
#define OD_INDEX_ERROR_REGISTER     0x1001
#define OD_INDEX_MANUFACTURER       0x1002
#define OD_INDEX_DEVICE_NAME        0x1008
#define OD_INDEX_HARDWARE_VERSION   0x1009
#define OD_INDEX_SOFTWARE_VERSION   0x100A
#define OD_INDEX_IDENTITY           0x1018

#define OD_INDEX_GUARD_TIME         0x100C
#define OD_INDEX_LIFE_TIME_FACTOR   0x100D

#define OD_INDEX_RPDO1_COMM         0x1400
#define OD_INDEX_RPDO1_MAP          0x1600
#define OD_INDEX_TPDO1_COMM         0x1800
#define OD_INDEX_TPDO1_MAP          0x1A00

#define OD_INDEX_SYNC_COUNTER       0x1006

/* Motor drive specific (0x2000-0x5FFF manufacturer-specific) */
#define OD_INDEX_TARGET_VELOCITY    0x2001
#define OD_INDEX_ACTUAL_VELOCITY    0x2002
#define OD_INDEX_ACTUAL_CURRENT     0x2003
#define OD_INDEX_PID_POS_KP         0x2100
#define OD_INDEX_PID_POS_KI         0x2101
#define OD_INDEX_PID_POS_KD         0x2102
#define OD_INDEX_PID_VEL_KP         0x2103
#define OD_INDEX_PID_VEL_KI         0x2104
#define OD_INDEX_PID_VEL_KD         0x2105
#define OD_INDEX_VELOCITY_LIMIT     0x2200
#define OD_INDEX_ACCEL_LIMIT        0x2201
#define OD_INDEX_DECEL_LIMIT        0x2202
#define OD_INDEX_CURRENT_LIMIT      0x2203

/* Error codes */
#define CO_ERR_NO_ERROR             0x0000
#define CO_ERR_GENERIC              0x1000
#define CO_ERR_CURRENT              0x2000
#define CO_ERR_VOLTAGE              0x3000
#define CO_ERR_TEMPERATURE          0x4000
#define CO_ERR_COMMUNICATION        0x5000
#define CO_ERR_PROFILE              0x6000

/* ------------------------------------------------------------------ */
/*  NMT states                                                        */
/* ------------------------------------------------------------------ */
typedef enum {
    NMT_STATE_INIT     = 0,
    NMT_STATE_PREOP    = 127,
    NMT_STATE_OPERATIONAL = 5,
    NMT_STATE_STOPPED  = 4
} nmt_state_t;

/* ------------------------------------------------------------------ */
/*  CAN frame                                                          */
/* ------------------------------------------------------------------ */
typedef struct __attribute__((packed)) {
    uint32_t can_id;         /* 11-bit identifier */
    uint8_t  dlc;            /* data length (0-8) */
    uint8_t  data[8];
    bool     is_extended;    /* false = CAN 2.0A */
} can_frame_t;

/* ------------------------------------------------------------------ */
/*  CAN bus abstraction (stub — replace with socketcan / HAL)          */
/* ------------------------------------------------------------------ */
static int can_send(const can_frame_t *frame)
{
    /* STUB: write to CAN controller.
     * On Linux: write(socket_fd, frame, sizeof(struct can_frame));
     * On STM32: FDCAN_Tx FIFO push.
     */
    (void)frame;
    return 0;
}

static int can_recv(can_frame_t *frame, int timeout_ms)
{
    /* STUB: poll CAN controller with timeout.
     * Returns 1 if frame received, 0 on timeout, -1 on error.
     */
    (void)frame; (void)timeout_ms;
    return 0; /* no data */
}

/* ------------------------------------------------------------------ */
/*  Object Dictionary                                                  */
/* ------------------------------------------------------------------ */
typedef struct {
    uint16_t index;
    uint8_t  subindex;
    uint32_t value;
    uint8_t  size;     /* 1, 2, or 4 bytes */
    bool     readonly;
    const char *name;
} od_entry_t;

/* Simplified object dictionary for the motor drive */
static od_entry_t g_od[] = {
    {OD_INDEX_DEVICE_TYPE,      0, 0x00020192, 4, true,  "Device Type"},
    {OD_INDEX_ERROR_REGISTER,   0, 0x00,        1, true,  "Error Register"},
    {OD_INDEX_MANUFACTURER,     0, 0x486F6E00,  4, true,  "Manufacturer"},
    {OD_INDEX_DEVICE_NAME,      0, 0x4D6F746F,  4, true,  "Device Name"},
    {OD_INDEX_HARDWARE_VERSION, 0, 0x01000000,  4, true,  "Hardware Version"},
    {OD_INDEX_SOFTWARE_VERSION, 0, 0x01000000,  4, true,  "Software Version"},
    {OD_INDEX_TARGET_VELOCITY,  0, 0,           4, false, "Target Velocity"},
    {OD_INDEX_ACTUAL_VELOCITY,  0, 0,           4, true,  "Actual Velocity"},
    {OD_INDEX_ACTUAL_CURRENT,   0, 0,           2, true,  "Actual Current"},
    {OD_INDEX_PID_VEL_KP,       0, 1000,        2, false, "Velocity Kp"},
    {OD_INDEX_PID_VEL_KI,       0, 100,         2, false, "Velocity Ki"},
    {OD_INDEX_PID_VEL_KD,       0, 10,          2, false, "Velocity Kd"},
    {OD_INDEX_VELOCITY_LIMIT,   0, 3000,        2, false, "Velocity Limit"},
    {OD_INDEX_ACCEL_LIMIT,      0, 1000,        2, false, "Accel Limit"},
    {OD_INDEX_CURRENT_LIMIT,    0, 5000,        2, false, "Current Limit"},
};

#define OD_ENTRIES (sizeof(g_od) / sizeof(g_od[0]))

static od_entry_t *od_find(uint16_t index, uint8_t subindex)
{
    for (size_t i = 0; i < OD_ENTRIES; i++) {
        if (g_od[i].index == index && g_od[i].subindex == subindex)
            return &g_od[i];
    }
    return NULL;
}

/* ------------------------------------------------------------------ */
/*  CANopen node context                                               */
/* ------------------------------------------------------------------ */
typedef struct {
    uint8_t      node_id;
    nmt_state_t  nmt_state;
    bool         heartbeat_active;
    uint32_t     heartbeat_last_ms;
    uint32_t     emergency_error_code;
    uint8_t      error_register;

    /* PDO data */
    int32_t      target_velocity;   /* RPDO1 */
    int32_t      actual_velocity;   /* TPDO1 */
    int16_t      actual_current;    /* TPDO1 */
} co_node_t;

static co_node_t g_node;
static bool g_initialized = false;

/* ------------------------------------------------------------------ */
/*  Time helper (stub — platform specific)                             */
/* ------------------------------------------------------------------ */
static uint32_t co_get_ms(void)
{
    /* In production: HAL_GetTick() or clock_gettime(CLOCK_MONOTONIC) */
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

/* ------------------------------------------------------------------ */
/*  NMT                                                               */
/* ------------------------------------------------------------------ */

int co_nmt_send_command(uint8_t node_id, uint8_t command)
{
    can_frame_t frame;
    frame.can_id = 0x000; /* NMT is always on COB-ID 0 */
    frame.dlc = 2;
    frame.data[0] = command;
    frame.data[1] = node_id;
    frame.is_extended = false;

    int rc = can_send(&frame);
    if (rc == 0) {
        switch (command) {
        case CO_NMT_START_NODE:
            g_node.nmt_state = NMT_STATE_OPERATIONAL;
            break;
        case CO_NMT_STOP_NODE:
            g_node.nmt_state = NMT_STATE_STOPPED;
            break;
        case CO_NMT_PREOP:
            g_node.nmt_state = NMT_STATE_PREOP;
            break;
        case CO_NMT_RESET_NODE:
        case CO_NMT_RESET_COMM:
            g_node.nmt_state = NMT_STATE_INIT;
            break;
        }
    }
    return rc;
}

int co_nmt_start_node(uint8_t node_id)
{
    return co_nmt_send_command(node_id, CO_NMT_START_NODE);
}

int co_nmt_stop_node(uint8_t node_id)
{
    return co_nmt_send_command(node_id, CO_NMT_STOP_NODE);
}

int co_nmt_reset_node(uint8_t node_id)
{
    return co_nmt_send_command(node_id, CO_NMT_RESET_NODE);
}

/* ------------------------------------------------------------------ */
/*  SDO                                                               */
/* ------------------------------------------------------------------ */

int co_sdo_upload(uint8_t node_id, uint16_t index, uint8_t subindex,
                   uint32_t *value)
{
    /* Build SDO upload request (expedited) */
    can_frame_t tx;
    tx.can_id = CO_SDO_REQUEST | node_id;
    tx.dlc = 8;
    tx.data[0] = CO_SDO_CMD_UPLOAD_REQ;
    tx.data[1] = (uint8_t)(index >> 0);
    tx.data[2] = (uint8_t)(index >> 8);
    tx.data[3] = subindex;
    tx.data[4] = 0;
    tx.data[5] = 0;
    tx.data[6] = 0;
    tx.data[7] = 0;
    tx.is_extended = false;

    int rc = can_send(&tx);
    if (rc != 0) return rc;

    /* Wait for response */
    can_frame_t rx;
    int retries = 50;
    while (retries--) {
        if (can_recv(&rx, 10) == 1 &&
            rx.can_id == (CO_SDO_RESPONSE | node_id)) {
            if ((rx.data[0] & 0xE0) == CO_SDO_CMD_UPLOAD_RESP) {
                *value = (uint32_t)rx.data[4] |
                         ((uint32_t)rx.data[5] << 8) |
                         ((uint32_t)rx.data[6] << 16) |
                         ((uint32_t)rx.data[7] << 24);
                return 0;
            } else if ((rx.data[0] & 0xE0) == CO_SDO_CMD_ABORT) {
                uint32_t abort = (uint32_t)rx.data[4] |
                                 ((uint32_t)rx.data[5] << 8) |
                                 ((uint32_t)rx.data[6] << 16) |
                                 ((uint32_t)rx.data[7] << 24);
                fprintf(stderr, "[CANopen] SDO abort code 0x%08X\n", abort);
                return -EIO;
            }
        }
    }
    return -ETIMEDOUT;
}

int co_sdo_download(uint8_t node_id, uint16_t index, uint8_t subindex,
                     uint32_t value, uint8_t size)
{
    can_frame_t tx;
    tx.can_id = CO_SDO_REQUEST | node_id;
    tx.dlc = 8;
    tx.data[0] = CO_SDO_CMD_DOWNLOAD_REQ | (size == 1 ? 0x00 :
                                            size == 2 ? 0x10 :
                                            size == 4 ? 0x20 : 0x20);
    tx.data[1] = (uint8_t)(index >> 0);
    tx.data[2] = (uint8_t)(index >> 8);
    tx.data[3] = subindex;
    tx.data[4] = (uint8_t)(value >> 0);
    tx.data[5] = (uint8_t)(value >> 8);
    tx.data[6] = (uint8_t)(value >> 16);
    tx.data[7] = (uint8_t)(value >> 24);
    tx.is_extended = false;

    int rc = can_send(&tx);
    if (rc != 0) return rc;

    /* Wait for response */
    can_frame_t rx;
    int retries = 50;
    while (retries--) {
        if (can_recv(&rx, 10) == 1 &&
            rx.can_id == (CO_SDO_RESPONSE | node_id)) {
            if ((rx.data[0] & 0xE0) == CO_SDO_CMD_DOWNLOAD_RESP) {
                return 0;
            } else if ((rx.data[0] & 0xE0) == CO_SDO_CMD_ABORT) {
                uint32_t abort = (uint32_t)rx.data[4] |
                                 ((uint32_t)rx.data[5] << 8) |
                                 ((uint32_t)rx.data[6] << 16) |
                                 ((uint32_t)rx.data[7] << 24);
                fprintf(stderr, "[CANopen] SDO abort code 0x%08X\n", abort);
                return -EIO;
            }
        }
    }
    return -ETIMEDOUT;
}

/* Convenience wrappers */
int co_sdo_write_u32(uint8_t node_id, uint16_t idx, uint8_t sub, uint32_t val)
{
    return co_sdo_download(node_id, idx, sub, val, 4);
}

int co_sdo_write_u16(uint8_t node_id, uint16_t idx, uint8_t sub, uint16_t val)
{
    return co_sdo_download(node_id, idx, sub, val, 2);
}

int co_sdo_read_u32(uint8_t node_id, uint16_t idx, uint8_t sub, uint32_t *val)
{
    return co_sdo_upload(node_id, idx, sub, val);
}

/* ------------------------------------------------------------------ */
/*  PDO                                                               */
/* ------------------------------------------------------------------ */

/**
 * co_pdo_send_tpdo1 - send TPDO1: actual velocity + actual current
 */
int co_pdo_send_tpdo1(int32_t actual_velocity, int16_t actual_current)
{
    can_frame_t frame;
    frame.can_id = CO_TPDO1_COBID | g_node.node_id;
    frame.dlc = 6;
    frame.data[0] = (uint8_t)(actual_velocity >> 0);
    frame.data[1] = (uint8_t)(actual_velocity >> 8);
    frame.data[2] = (uint8_t)(actual_velocity >> 16);
    frame.data[3] = (uint8_t)(actual_velocity >> 24);
    frame.data[4] = (uint8_t)(actual_current >> 0);
    frame.data[5] = (uint8_t)(actual_current >> 8);
    frame.is_extended = false;

    g_node.actual_velocity = actual_velocity;
    g_node.actual_current  = actual_current;

    return can_send(&frame);
}

/**
 * co_pdo_process_rpdo1 - process RPDO1: target velocity + control word
 */
int co_pdo_process_rpdo1(const can_frame_t *frame)
{
    if (!frame) return -EINVAL;

    int32_t target_vel = (int32_t)(frame->data[0] |
                                   ((uint32_t)frame->data[1] << 8) |
                                   ((uint32_t)frame->data[2] << 16) |
                                   ((uint32_t)frame->data[3] << 24));

    g_node.target_velocity = target_vel;
    return 0;
}

/* ------------------------------------------------------------------ */
/*  Heartbeat                                                          */
/* ------------------------------------------------------------------ */

/**
 * co_heartbeat_send - transmit heartbeat message
 */
int co_heartbeat_send(void)
{
    can_frame_t frame;
    frame.can_id = 0x700 | g_node.node_id;
    frame.dlc = 1;
    frame.data[0] = (uint8_t)g_node.nmt_state;
    frame.is_extended = false;

    return can_send(&frame);
}

/**
 * co_heartbeat_process - process received heartbeat
 */
int co_heartbeat_process(const can_frame_t *frame)
{
    if (!frame) return -EINVAL;

    uint8_t node_state = frame->data[0];
    g_node.heartbeat_last_ms = co_get_ms();
    g_node.heartbeat_active = true;

    /* Update known NMT state */
    switch (node_state) {
    case NMT_STATE_OPERATIONAL:
    case NMT_STATE_STOPPED:
    case NMT_STATE_PREOP:
        g_node.nmt_state = (nmt_state_t)node_state;
        break;
    }

    return 0;
}

/**
 * co_heartbeat_check - call periodically (every ~50ms) to check for timeout
 *                      Returns 0 if OK, 1 if heartbeat timeout (aler)
 */
int co_heartbeat_check(void)
{
    if (!g_node.heartbeat_active) return 0;

    uint32_t now = co_get_ms();
    uint32_t elapsed = now - g_node.heartbeat_last_ms;

    if (elapsed > CO_HEARTBEAT_TIMEOUT_MS) {
        fprintf(stderr, "[CANopen] HEARTBEAT TIMEOUT: %u ms since last\n", elapsed);
        g_node.emergency_error_code = CO_ERR_COMMUNICATION;
        return 1;
    }

    return 0;
}

/* ------------------------------------------------------------------ */
/*  Emergency                                                          */
/* ------------------------------------------------------------------ */

/**
 * co_emcy_send - transmit emergency message
 */
int co_emcy_send(uint16_t error_code, uint8_t error_register,
                  uint8_t *manufacturer_data)
{
    can_frame_t frame;
    frame.can_id = CO_EMCY_COBID | g_node.node_id;
    frame.dlc = 8;
    frame.data[0] = (uint8_t)(error_code >> 0);
    frame.data[1] = (uint8_t)(error_code >> 8);
    frame.data[2] = error_register;
    frame.data[3] = manufacturer_data ? manufacturer_data[0] : 0;
    frame.data[4] = manufacturer_data ? manufacturer_data[1] : 0;
    frame.data[5] = manufacturer_data ? manufacturer_data[2] : 0;
    frame.data[6] = manufacturer_data ? manufacturer_data[3] : 0;
    frame.data[7] = manufacturer_data ? manufacturer_data[4] : 0;
    frame.is_extended = false;

    return can_send(&frame);
}

/**
 * co_emcy_process - handle received emergency message
 */
int co_emcy_process(const can_frame_t *frame)
{
    if (!frame || frame->dlc < 4) return -EINVAL;

    uint16_t error_code = (uint16_t)(frame->data[0] | (frame->data[1] << 8));
    uint8_t  error_reg  = frame->data[2];

    g_node.emergency_error_code = error_code;
    g_node.error_register |= error_reg;

    fprintf(stderr, "[CANopen] EMCY from node 0x%02X: code=0x%04X reg=0x%02X\n",
            (unsigned)(frame->can_id & 0x7F), error_code, error_reg);
    return 0;
}

/* ------------------------------------------------------------------ */
/*  Configuration helpers                                              */
/* ------------------------------------------------------------------ */

int co_set_pid_gains(uint8_t node_id,
                      int16_t vel_kp, int16_t vel_ki, int16_t vel_kd)
{
    int rc;
    rc = co_sdo_write_u16(node_id, OD_INDEX_PID_VEL_KP, 0, (uint16_t)vel_kp);
    if (rc) return rc;
    rc = co_sdo_write_u16(node_id, OD_INDEX_PID_VEL_KI, 0, (uint16_t)vel_ki);
    if (rc) return rc;
    rc = co_sdo_write_u16(node_id, OD_INDEX_PID_VEL_KD, 0, (uint16_t)vel_kd);
    return rc;
}

int co_set_limits(uint8_t node_id,
                   uint16_t velocity_max, uint16_t current_max)
{
    int rc;
    rc = co_sdo_write_u16(node_id, OD_INDEX_VELOCITY_LIMIT, 0, velocity_max);
    if (rc) return rc;
    rc = co_sdo_write_u16(node_id, OD_INDEX_CURRENT_LIMIT, 0, current_max);
    return rc;
}

/* ------------------------------------------------------------------ */
/*  Initialization / Poll                                              */
/* ------------------------------------------------------------------ */

int co_init(uint8_t node_id)
{
    memset(&g_node, 0, sizeof(g_node));
    g_node.node_id = node_id;
    g_node.nmt_state = NMT_STATE_INIT;
    g_node.heartbeat_active = false;

    /* Initialize CAN hardware */
    /* STUB: HAL_CAN_Init() or socketcan socket() */

    g_initialized = true;
    return 0;
}

/**
 * co_poll - main CANopen processing loop (call periodically)
 *
 * Handles:
 *   - Incoming PDO processing
 *   - Heartbeat monitoring
 *   - Emergency message handling
 *   - Periodic heartbeat transmission (if operational)
 */
int co_poll(void)
{
    if (!g_initialized) return -EPERM;

    can_frame_t frame;
    int rc;

    /* Process all pending frames */
    while ((rc = can_recv(&frame, 0)) == 1) {
        uint32_t cob_id = frame.can_id & 0x7F8; /* mask node ID */
        uint8_t  node   = frame.can_id & 0x7F;

        switch (cob_id) {
        case 0x180: /* TPDO1 from remote */
            /* Remote node sent its velocity/current */
            break;

        case 0x200: /* RPDO1 addressed to us */
            co_pdo_process_rpdo1(&frame);
            break;

        case 0x580: /* SDO response */
            /* Handled in co_sdo_upload/download context */
            break;

        case 0x700: /* Heartbeat */
            co_heartbeat_process(&frame);
            break;

        case 0x080: /* Emergency */
            co_emcy_process(&frame);
            break;

        default:
            break;
        }
    }

    /* Send heartbeat if operational */
    if (g_node.nmt_state == NMT_STATE_OPERATIONAL) {
        static uint32_t last_heartbeat = 0;
        uint32_t now = co_get_ms();
        if (now - last_heartbeat >= CO_HEARTBEAT_TIME_MS) {
            co_heartbeat_send();
            last_heartbeat = now;
        }
    }

    /* Check heartbeats */
    co_heartbeat_check();

    return 0;
}

void co_shutdown(void)
{
    co_nmt_stop_node(g_node.node_id);
    g_initialized = false;
}
