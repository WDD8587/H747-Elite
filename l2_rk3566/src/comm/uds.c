/**
 * @file    uds.c
 * @brief   UDS (Unified Diagnostic Services) over CAN for ECU diagnostics
 * @details Implements ISO 14229 services:
 *          0x10 (Diagnostic Session Control)
 *          0x22 (ReadDataByIdentifier)
 *          0x2E (WriteDataByIdentifier)
 *          0x11 (ECU Reset)
 *          0x14 (Clear Diagnostic Information)
 *          0x19 (Read DTC Information)
 *
 *          DTC format: 3-byte code per ISO 14229
 *          Transport: CAN bus with ISO-TP (single-frame, < 8 bytes)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

/* ------------------------------------------------------------------ */
/*  UDS constants                                                      */
/* ------------------------------------------------------------------ */
#define UDS_SESSION_DEFAULT     0x01
#define UDS_SESSION_PROGRAMMING 0x02
#define UDS_SESSION_EXTENDED    0x03
#define UDS_SESSION_SAFETY      0x04

#define UDS_SERVICE_DIAG_SESSION_CTRL    0x10
#define UDS_SERVICE_ECU_RESET            0x11
#define UDS_SERVICE_CLEAR_DTC            0x14
#define UDS_SERVICE_READ_DTC             0x19
#define UDS_SERVICE_READ_DATA_BY_ID      0x22
#define UDS_SERVICE_WRITE_DATA_BY_ID     0x2E

#define UDS_RESP_POSITIVE               0x40  /* service + 0x40 */
#define UDS_RESP_NEGATIVE               0x7F

/* Negative response codes (NRC) */
#define UDS_NRC_GENERAL_REJECT          0x10
#define UDS_NRC_SERVICE_NOT_SUPPORTED    0x11
#define UDS_NRC_SUBFUNC_NOT_SUPPORTED    0x12
#define UDS_NRC_INCORRECT_MSG_LENGTH     0x13
#define UDS_NRC_CONDITIONS_NOT_CORRECT   0x22
#define UDS_NRC_REQUEST_OUT_OF_RANGE     0x31
#define UDS_NRC_SECURITY_ACCESS_DENIED   0x33
#define UDS_NRC_UPLOAD_DOWNLOAD_NOT_ACCEPTED 0x70

#define UDS_DTC_FORMAT_LEN              3     /* 3-byte DTC */

#define UDS_MAX_DATA_LEN                128
#define UDS_CAN_ID_REQUEST              0x7E0  /* physical request */
#define UDS_CAN_ID_RESPONSE             0x7E8  /* physical response */

/* ------------------------------------------------------------------ */
/*  DTC format helpers                                                 */
/* ------------------------------------------------------------------ */
#define DTC(high, mid, low)  ((uint32_t)(((high) & 0xFF) << 16) | \
                              ((uint32_t)((mid) & 0xFF) << 8)  | \
                              ((uint32_t)((low) & 0xFF)))

#define DTC_HIGH(dtc)  ((uint8_t)((dtc) >> 16))
#define DTC_MID(dtc)   ((uint8_t)((dtc) >> 8))
#define DTC_LOW(dtc)   ((uint8_t)(dtc))

/* Predefined DTC codes */
#define DTC_NO_ERROR                 DTC(0, 0, 0)
#define DTC_POWER_SUPPLY             DTC(0x10, 0x01, 0x00)
#define DTC_COMM_LOST                DTC(0x20, 0x01, 0x00)
#define DTC_OVERCURRENT              DTC(0x30, 0x01, 0x00)
#define DTC_OVERTEMPERATURE          DTC(0x40, 0x01, 0x00)
#define DTC_SENSOR_FAILURE           DTC(0x50, 0x01, 0x00)
#define DTC_ACTUATOR_FAILURE         DTC(0x60, 0x01, 0x00)
#define DTC_SOFTWARE_ERROR           DTC(0x70, 0x01, 0x00)
#define DTC_CALIBRATION_ERROR        DTC(0x80, 0x01, 0x00)
#define DTC_MEMORY_ERROR             DTC(0x90, 0x01, 0x00)

/* ------------------------------------------------------------------ */
/*  Data structure                                                     */
/* ------------------------------------------------------------------ */

typedef struct {
    uint8_t service_id;
    uint8_t subfunction;
    uint8_t data[UDS_MAX_DATA_LEN];
    uint16_t data_len;
} uds_request_t;

typedef struct {
    uint8_t service_id;      /* response service (= request + 0x40) */
    uint8_t data[UDS_MAX_DATA_LEN];
    uint16_t data_len;
} uds_response_t;

/* DTC entry */
typedef struct {
    uint32_t  dtc;            /* 3-byte DTC */
    uint8_t   status;         /* status byte per ISO 14229 */
    uint8_t   severity;       /* 0=no, 1=maintenance, 2=check, 3=immediate */
    uint16_t  occurrence_count;
    uint32_t  first_occurred_ms;
    uint32_t  last_occurred_ms;
} dtc_entry_t;

/* ------------------------------------------------------------------ */
/*  Global state                                                       */
/* ------------------------------------------------------------------ */
static uint8_t  g_current_session = UDS_SESSION_DEFAULT;
static dtc_entry_t g_dtc_table[16];
static uint8_t  g_dtc_count = 0;
static bool     g_initialized = false;

/* DTC status byte bits */
#define DTC_STATUS_TEST_FAILED          0x01
#define DTC_STATUS_TEST_FAILED_THIS_OP  0x02
#define DTC_STATUS_PENDING              0x04
#define DTC_STATUS_CONFIRMED            0x08
#define DTC_STATUS_TEST_NOT_COMPLETED   0x10
#define DTC_STATUS_PREVIOUSLY_CONFIRMED 0x20
#define DTC_STATUS_WARNING_INDICATOR    0x40

/* ------------------------------------------------------------------ */
/*  CAN abstraction (stub)                                             */
/* ------------------------------------------------------------------ */
typedef struct {
    uint32_t can_id;
    uint8_t  dlc;
    uint8_t  data[8];
} can_message_t;

static int can_write(const can_message_t *msg)
{
    (void)msg;
    return 0;
}

static int can_read(can_message_t *msg, int timeout_ms)
{
    (void)msg; (void)timeout_ms;
    return 0; /* no data */
}

/* ------------------------------------------------------------------ */
/*  DTC management                                                     */
/* ------------------------------------------------------------------ */

int uds_dtc_set(uint32_t dtc, uint8_t severity)
{
    /* Find existing or first empty slot */
    dtc_entry_t *entry = NULL;
    for (int i = 0; i < g_dtc_count; i++) {
        if (g_dtc_table[i].dtc == dtc) {
            entry = &g_dtc_table[i];
            break;
        }
    }

    if (!entry && g_dtc_count < (int)(sizeof(g_dtc_table)/sizeof(g_dtc_table[0]))) {
        entry = &g_dtc_table[g_dtc_count++];
        memset(entry, 0, sizeof(*entry));
        entry->dtc = dtc;
        entry->first_occurred_ms = 0; /* set proper timestamp in production */
    }

    if (!entry) return -ENOSPC;

    entry->status |= DTC_STATUS_TEST_FAILED | DTC_STATUS_PENDING | DTC_STATUS_CONFIRMED;
    entry->severity = severity;
    entry->occurrence_count++;
    entry->last_occurred_ms = 0; /* set proper timestamp */

    return 0;
}

int uds_dtc_clear(void)
{
    memset(g_dtc_table, 0, sizeof(g_dtc_table));
    g_dtc_count = 0;
    return 0;
}

uint8_t uds_dtc_get_count(void)
{
    return g_dtc_count;
}

/* ------------------------------------------------------------------ */
/*  Service handlers                                                    */
/* ------------------------------------------------------------------ */

static int handle_session_control(const uds_request_t *req, uds_response_t *resp)
{
    if (req->data_len < 1) {
        resp->data[0] = UDS_NRC_INCORRECT_MSG_LENGTH;
        resp->data_len = 1;
        resp->service_id = UDS_RESP_NEGATIVE;
        return -1;
    }

    uint8_t session = req->data[0];

    /* Validate session */
    if (session != UDS_SESSION_DEFAULT &&
        session != UDS_SESSION_PROGRAMMING &&
        session != UDS_SESSION_EXTENDED &&
        session != UDS_SESSION_SAFETY) {
        resp->data[0] = UDS_NRC_SUBFUNC_NOT_SUPPORTED;
        resp->data_len = 1;
        resp->service_id = UDS_RESP_NEGATIVE;
        return -1;
    }

    /* If programming, require security access */
    if (session == UDS_SESSION_PROGRAMMING &&
        g_current_session != UDS_SESSION_PROGRAMMING) {
        /* In production: verify security access */
    }

    g_current_session = session;

    resp->data[0] = session;
    resp->data[1] = 0x00; /* session parameter record (none) */
    resp->data_len = 2;
    return 0;
}

static int handle_ecu_reset(const uds_request_t *req, uds_response_t *resp)
{
    if (req->data_len < 1) {
        resp->data[0] = UDS_NRC_INCORRECT_MSG_LENGTH;
        resp->data_len = 1;
        resp->service_id = UDS_RESP_NEGATIVE;
        return -1;
    }

    uint8_t reset_type = req->data[0];

    if (reset_type != 0x01 && reset_type != 0x02 && reset_type != 0x03) {
        resp->data[0] = UDS_NRC_SUBFUNC_NOT_SUPPORTED;
        resp->data_len = 1;
        resp->service_id = UDS_RESP_NEGATIVE;
        return -1;
    }

    /* Send positive response before actually resetting */
    resp->data[0] = reset_type;
    resp->data_len = 1;

    /* Schedule reset (set a flag for main loop to act on) */
    fprintf(stderr, "[UDS] ECU Reset type 0x%02X scheduled\n", reset_type);
    return 0;
}

static int handle_read_data_by_id(const uds_request_t *req, uds_response_t *resp)
{
    if (req->data_len < 2) {
        resp->data[0] = UDS_NRC_INCORRECT_MSG_LENGTH;
        resp->data_len = 1;
        resp->service_id = UDS_RESP_NEGATIVE;
        return -1;
    }

    uint16_t did = (uint16_t)(req->data[0] << 8) | req->data[1];

    /* Echo DID in response */
    resp->data[0] = (uint8_t)(did >> 8);
    resp->data[1] = (uint8_t)(did);

    switch (did) {
    case 0xF190: /* Software version */
    {
        const char *sw_ver = "H747_Elite v2.1.0";
        uint8_t len = (uint8_t)strlen(sw_ver);
        resp->data[2] = len;
        memcpy(resp->data + 3, sw_ver, len);
        resp->data_len = 3 + len;
        break;
    }
    case 0xF191: /* VIN / serial number */
    {
        const char *vin = "SN-H747E-2025-00001";
        uint8_t len = (uint8_t)strlen(vin);
        resp->data[2] = len;
        memcpy(resp->data + 3, vin, len);
        resp->data_len = 3 + len;
        break;
    }
    case 0xF192: /* Hardware version */
    {
        const char *hw = "REV-B";
        resp->data[2] = (uint8_t)strlen(hw);
        memcpy(resp->data + 3, hw, strlen(hw));
        resp->data_len = 3 + (uint8_t)strlen(hw);
        break;
    }
    case 0xF193: /* Bootloader version */
        resp->data[2] = 0x01;
        resp->data[3] = 0x03;
        resp->data_len = 4;
        break;
    case 0xF194: /* Current session */
        resp->data[2] = g_current_session;
        resp->data_len = 3;
        break;
    case 0xF195: /* System supply voltage (mV) */
    {
        uint16_t mv = 12000; /* 12.0V */
        resp->data[2] = (uint8_t)(mv >> 8);
        resp->data[3] = (uint8_t)(mv);
        resp->data_len = 4;
        break;
    }
    default:
        resp->data[0] = UDS_NRC_REQUEST_OUT_OF_RANGE;
        resp->data_len = 1;
        resp->service_id = UDS_RESP_NEGATIVE;
        return -1;
    }

    return 0;
}

static int handle_write_data_by_id(const uds_request_t *req, uds_response_t *resp)
{
    if (req->data_len < 2) {
        resp->data[0] = UDS_NRC_INCORRECT_MSG_LENGTH;
        resp->data_len = 1;
        resp->service_id = UDS_RESP_NEGATIVE;
        return -1;
    }

    uint16_t did = (uint16_t)(req->data[0] << 8) | req->data[1];

    /* Check session — writing typically requires extended or programming */
    if (g_current_session < UDS_SESSION_EXTENDED) {
        resp->data[0] = UDS_NRC_CONDITIONS_NOT_CORRECT;
        resp->data_len = 1;
        resp->service_id = UDS_RESP_NEGATIVE;
        return -1;
    }

    /* Echo DID in response */
    resp->data[0] = (uint8_t)(did >> 8);
    resp->data[1] = (uint8_t)(did);
    resp->data_len = 2;

    switch (did) {
    case 0xF194: /* Write session (override) */
        if (req->data_len >= 3) {
            g_current_session = req->data[2];
        }
        break;
    default:
        resp->data[0] = UDS_NRC_REQUEST_OUT_OF_RANGE;
        resp->data_len = 1;
        resp->service_id = UDS_RESP_NEGATIVE;
        return -1;
    }

    return 0;
}

static int handle_clear_dtc(const uds_request_t *req, uds_response_t *resp)
{
    /* Optional: check group of DTC (byte 0 = 0xFF means all) */
    (void)req;

    uds_dtc_clear();
    resp->data_len = 0;
    return 0;
}

static int handle_read_dtc(const uds_request_t *req, uds_response_t *resp)
{
    if (req->data_len < 1) {
        resp->data[0] = UDS_NRC_INCORRECT_MSG_LENGTH;
        resp->data_len = 1;
        resp->service_id = UDS_RESP_NEGATIVE;
        return -1;
    }

    uint8_t dtc_mask = req->data[0]; /* report type */

    resp->data[0] = dtc_mask;        /* echo subfunction */
    resp->data[1] = g_dtc_count;     /* number of DTCs */
    resp->data[2] = 0x00;            /* DTC status availability mask */

    uint16_t offset = 3;

    for (int i = 0; i < g_dtc_count && (offset + 4) <= UDS_MAX_DATA_LEN; i++) {
        resp->data[offset + 0] = DTC_HIGH(g_dtc_table[i].dtc);
        resp->data[offset + 1] = DTC_MID(g_dtc_table[i].dtc);
        resp->data[offset + 2] = DTC_LOW(g_dtc_table[i].dtc);
        resp->data[offset + 3] = g_dtc_table[i].status;
        offset += 4;
    }

    resp->data_len = offset;
    return 0;
}

/* ------------------------------------------------------------------ */
/*  Main dispatcher                                                    */
/* ------------------------------------------------------------------ */

int uds_dispatch(const uds_request_t *req, uds_response_t *resp)
{
    if (!req || !resp) return -EINVAL;

    memset(resp, 0, sizeof(*resp));
    resp->service_id = req->service_id + UDS_RESP_POSITIVE;

    switch (req->service_id) {
    case UDS_SERVICE_DIAG_SESSION_CTRL:
        return handle_session_control(req, resp);
    case UDS_SERVICE_ECU_RESET:
        return handle_ecu_reset(req, resp);
    case UDS_SERVICE_READ_DATA_BY_ID:
        return handle_read_data_by_id(req, resp);
    case UDS_SERVICE_WRITE_DATA_BY_ID:
        return handle_write_data_by_id(req, resp);
    case UDS_SERVICE_CLEAR_DTC:
        return handle_clear_dtc(req, resp);
    case UDS_SERVICE_READ_DTC:
        return handle_read_dtc(req, resp);
    default:
        resp->data[0] = UDS_NRC_SERVICE_NOT_SUPPORTED;
        resp->data_len = 1;
        resp->service_id = UDS_RESP_NEGATIVE;
        return -1;
    }
}

/* ------------------------------------------------------------------ */
/*  CAN transport layer (ISO-TP single frame)                          */
/* ------------------------------------------------------------------ */

static int uds_can_send_response(const uds_response_t *resp)
{
    can_message_t msg;
    msg.can_id = UDS_CAN_ID_RESPONSE;
    msg.dlc = (resp->data_len + 1) > 8 ? 8 : (resp->data_len + 1);

    msg.data[0] = resp->service_id;
    uint16_t copy_len = msg.dlc - 1;
    if (copy_len > resp->data_len) copy_len = resp->data_len;
    memcpy(msg.data + 1, resp->data, copy_len);

    return can_write(&msg);
}

static int uds_can_parse_request(const can_message_t *msg, uds_request_t *req)
{
    if (!msg || !req || msg->dlc < 1) return -EINVAL;

    memset(req, 0, sizeof(*req));
    req->service_id = msg->data[0];
    req->data_len = msg->dlc - 1;

    if (req->data_len > UDS_MAX_DATA_LEN)
        req->data_len = UDS_MAX_DATA_LEN;

    memcpy(req->data, msg->data + 1, req->data_len);
    return 0;
}

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

int uds_init(void)
{
    g_current_session = UDS_SESSION_DEFAULT;
    uds_dtc_clear();
    g_initialized = true;
    return 0;
}

int uds_poll(void)
{
    if (!g_initialized) return -EPERM;

    can_message_t msg;
    if (can_read(&msg, 0) != 1)
        return 0;

    /* Only process messages addressed to us */
    if (msg.can_id != UDS_CAN_ID_REQUEST)
        return 0;

    uds_request_t  req;
    uds_response_t resp;

    int rc = uds_can_parse_request(&msg, &req);
    if (rc != 0) return rc;

    rc = uds_dispatch(&req, &resp);
    /* Send response even on negative */
    uds_can_send_response(&resp);

    return rc;
}

void uds_shutdown(void)
{
    g_initialized = false;
}

uint8_t uds_get_session(void)
{
    return g_current_session;
}

/* ------------------------------------------------------------------ */
/*  Self-test                                                          */
/* ------------------------------------------------------------------ */
#ifdef TEST_UDS
int main(void)
{
    uds_init();

    /* Test: read software version */
    uint8_t req_data[] = {0xF1, 0x90};
    uds_request_t req = {
        .service_id = UDS_SERVICE_READ_DATA_BY_ID,
        .data_len = 2,
    };
    memcpy(req.data, req_data, 2);

    uds_response_t resp;
    int rc = uds_dispatch(&req, &resp);
    printf("ReadDataById 0xF190: rc=%d service=0x%02X len=%d\n",
           rc, resp.service_id, resp.data_len);
    for (int i = 0; i < resp.data_len; i++)
        printf("  [%d] 0x%02X\n", i, resp.data[i]);

    /* Test: set a DTC */
    uds_dtc_set(DTC_OVERCURRENT, 3);
    uds_dtc_set(DTC_OVERTEMPERATURE, 2);

    req.service_id = UDS_SERVICE_READ_DTC;
    req.data[0] = 0x01; /* report all */
    req.data_len = 1;
    memset(&resp, 0, sizeof(resp));
    rc = uds_dispatch(&req, &resp);
    printf("ReadDTC: rc=%d len=%d count=%d\n", rc, resp.data_len, resp.data[1]);

    uds_shutdown();
    return 0;
}
#endif /* TEST_UDS */
