/**
 * qrcode.c
 * QR code binding for factory production.
 *
 * Scans QR code on robot base (contains MAC address + serial number).
 * Writes bound data to OTP flash.
 * Verifies by reading back.
 * Cross-checks QR serial against MES serial.
 * Mismatch -> reject, re-scan.
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>

/* ------------------------------------------------------------------ */
/*  Configuration                                                     */
/* ------------------------------------------------------------------ */

#define QR_MAX_DATA_LEN         128
#define QR_SERIAL_LEN           32
#define QR_MAC_STR_LEN          18   /* "XX:XX:XX:XX:XX:XX" */
#define OTP_FLASH_SIZE          512
#define OTP_BINDING_OFFSET      0
#define OTP_BINDING_MAX_LEN     256

/* ------------------------------------------------------------------ */
/*  Types                                                             */
/* ------------------------------------------------------------------ */

typedef struct {
    char serial[QR_SERIAL_LEN];      /* product serial number */
    char mac_address[QR_MAC_STR_LEN]; /* MAC address */
    int  valid;                       /* 1 if data parsed OK */
} qr_data_t;

typedef enum {
    BIND_OK = 0,
    BIND_ERR_PARSE,
    BIND_ERR_OTP_WRITE,
    BIND_ERR_OTP_VERIFY,
    BIND_ERR_SERIAL_MISMATCH,
    BIND_ERR_READBACK_MISMATCH
} bind_result_t;

/* DUT function prototypes */
int        qr_parse(const char *raw_data, size_t raw_len, qr_data_t *out);
bind_result_t qr_bind_to_otp(const qr_data_t *qr, const char *mes_serial);
int        qr_verify_otp(const qr_data_t *expected);
int        qr_read_otp(qr_data_t *out);

/* ------------------------------------------------------------------ */
/*  OTP flash simulation (platform stub)                              */
/* ------------------------------------------------------------------ */

static uint8_t otp_sim[OTP_FLASH_SIZE];
static int otp_write_protected = 0;

/* Initialize simulated OTP */
void otp_sim_init(void)
{
    memset(otp_sim, 0xFF, sizeof(otp_sim));
    otp_write_protected = 0;
}

/* Simulated OTP write. In production, writes to one-time programmable fuse array. */
static int platform_otp_write(uint32_t offset, const uint8_t *data, size_t len)
{
    if (otp_write_protected) return -1;
    if (offset + len > OTP_FLASH_SIZE) return -1;

    for (size_t i = 0; i < len; i++)
        otp_sim[offset + i] &= data[i];  /* OTP: can only clear bits */

    otp_write_protected = 1;  /* Once written, lock */
    return 0;
}

/* Simulated OTP read */
static int platform_otp_read(uint32_t offset, uint8_t *data, size_t len)
{
    if (offset + len > OTP_FLASH_SIZE) return -1;
    memcpy(data, otp_sim + offset, len);
    return 0;
}

/* ------------------------------------------------------------------ */
/*  QR data format                                                    */
/* ------------------------------------------------------------------ */

/*
 * Expected QR format:
 *   H747-ELITE,<SERIAL>,<MAC>
 *
 * Example:
 *   H747-ELITE,SN20240101-001,00:1A:2B:3C:4D:5E
 *
 * The format includes a product prefix, serial number, and MAC address
 * separated by commas.
 */

#define QR_PREFIX "H747-ELITE"

int qr_parse(const char *raw_data, size_t raw_len, qr_data_t *out)
{
    memset(out, 0, sizeof(*out));

    if (!raw_data || raw_len < 10) return 0;

    /* Check prefix */
    size_t prefix_len = strlen(QR_PREFIX);
    if (raw_len < prefix_len + 3) return 0;
    if (strncmp(raw_data, QR_PREFIX, prefix_len) != 0) return 0;

    /* Find first comma after prefix */
    const char *p = raw_data + prefix_len;
    if (*p != ',') return 0;
    p++;

    /* Extract serial (up to next comma or end) */
    const char *serial_start = p;
    while (*p && *p != ',' && (size_t)(p - serial_start) < (size_t)(QR_SERIAL_LEN - 1))
        p++;

    if (p == serial_start) return 0;

    size_t serial_len = (size_t)(p - serial_start);
    memcpy(out->serial, serial_start, serial_len);
    out->serial[serial_len] = '\0';

    /* Expect comma */
    if (*p != ',') return 0;
    p++;

    /* Extract MAC address */
    const char *mac_start = p;
    while (*p && *p != '\r' && *p != '\n' && (size_t)(p - mac_start) < (size_t)(QR_MAC_STR_LEN - 1))
        p++;

    size_t mac_len = (size_t)(p - mac_start);
    memcpy(out->mac_address, mac_start, mac_len);
    out->mac_address[mac_len] = '\0';

    /* Validate MAC format: XX:XX:XX:XX:XX:XX */
    if (mac_len != 17) return 0;
    for (int i = 0; i < 17; i++) {
        if (i % 3 == 2) {
            if (out->mac_address[i] != ':') return 0;
        } else {
            if (!isxdigit((unsigned char)out->mac_address[i])) return 0;
        }
    }

    out->valid = 1;
    return 1;
}

/* ------------------------------------------------------------------ */
/*  OTP binding                                                       */
/* ------------------------------------------------------------------ */

/*
 * OTP storage format:
 *   [1 byte]  version (0x01)
 *   [1 byte]  flags
 *   [2 bytes] serial length (big-endian)
 *   [N bytes] serial string
 *   [6 bytes] MAC address (binary)
 *   [2 bytes] CRC-16 of preceding data
 *   [padding to 4-byte alignment]
 */

#define OTP_VERSION      0x01
#define OTP_FLAG_BOUND   0x01

static uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            if (crc & 0x8000)
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            else
                crc <<= 1;
        }
    }
    return crc;
}

/* Pack MAC string to binary */
static int mac_str_to_bin(const char *mac_str, uint8_t mac_bin[6])
{
    unsigned int b[6];
    if (sscanf(mac_str, "%02x:%02x:%02x:%02x:%02x:%02x",
               &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6)
        return 0;
    for (int i = 0; i < 6; i++)
        mac_bin[i] = (uint8_t)b[i];
    return 1;
}

/* Pack binding data for OTP storage */
static int pack_binding(const qr_data_t *qr, uint8_t *out, size_t max_out)
{
    size_t serial_len = strlen(qr->serial);
    if (serial_len > 64) return 0;

    size_t total = 1 + 1 + 2 + serial_len + 6 + 2;
    if (total > max_out) return 0;

    size_t pos = 0;
    out[pos++] = OTP_VERSION;
    out[pos++] = OTP_FLAG_BOUND;
    out[pos++] = (uint8_t)((serial_len >> 8) & 0xFF);
    out[pos++] = (uint8_t)(serial_len & 0xFF);
    memcpy(out + pos, qr->serial, serial_len);
    pos += serial_len;

    uint8_t mac_bin[6];
    if (!mac_str_to_bin(qr->mac_address, mac_bin))
        return 0;
    memcpy(out + pos, mac_bin, 6);
    pos += 6;

    uint16_t crc = crc16_ccitt(out, pos);
    out[pos++] = (uint8_t)((crc >> 8) & 0xFF);
    out[pos++] = (uint8_t)(crc & 0xFF);

    return (int)pos;
}

/* Unpack binding data from OTP */
static int unpack_binding(const uint8_t *data, size_t len, qr_data_t *out)
{
    memset(out, 0, sizeof(*out));

    if (len < 4) return 0;
    size_t pos = 0;

    uint8_t version = data[pos++];
    if (version != OTP_VERSION) return 0;

    uint8_t flags = data[pos++];
    (void)flags;

    size_t serial_len = ((size_t)data[pos] << 8) | data[pos + 1];
    pos += 2;

    if (pos + serial_len + 6 + 2 > len) return 0;
    if (serial_len >= QR_SERIAL_LEN) return 0;

    memcpy(out->serial, data + pos, serial_len);
    out->serial[serial_len] = '\0';
    pos += serial_len;

    /* Convert MAC binary to string */
    snprintf(out->mac_address, QR_MAC_STR_LEN,
             "%02X:%02X:%02X:%02X:%02X:%02X",
             data[pos], data[pos + 1], data[pos + 2],
             data[pos + 3], data[pos + 4], data[pos + 5]);
    pos += 6;

    /* Verify CRC */
    uint16_t stored_crc = ((uint16_t)data[pos] << 8) | data[pos + 1];
    uint16_t computed_crc = crc16_ccitt(data, pos);
    if (stored_crc != computed_crc) return 0;

    out->valid = 1;
    return 1;
}

/* ------------------------------------------------------------------ */
/*  High-level binding API                                            */
/* ------------------------------------------------------------------ */

/* Bind QR data to OTP.
 * Checks that QR serial matches MES serial.
 * Returns bind result code. */
bind_result_t qr_bind_to_otp(const qr_data_t *qr, const char *mes_serial)
{
    if (!qr || !qr->valid)
        return BIND_ERR_PARSE;

    if (!mes_serial || strlen(mes_serial) == 0)
        return BIND_ERR_PARSE;

    /* Cross-check: QR serial must match MES serial */
    if (strcmp(qr->serial, mes_serial) != 0)
        return BIND_ERR_SERIAL_MISMATCH;

    /* Pack binding data */
    uint8_t binding[OTP_BINDING_MAX_LEN];
    int packed_len = pack_binding(qr, binding, sizeof(binding));
    if (packed_len <= 0)
        return BIND_ERR_PARSE;

    /* Write to OTP */
    int rc = platform_otp_write(OTP_BINDING_OFFSET, binding, (size_t)packed_len);
    if (rc != 0)
        return BIND_ERR_OTP_WRITE;

    /* Verify by reading back */
    if (!qr_verify_otp(qr))
        return BIND_ERR_OTP_VERIFY;

    return BIND_OK;
}

/* Verify OTP contains the expected binding */
int qr_verify_otp(const qr_data_t *expected)
{
    uint8_t readback[OTP_BINDING_MAX_LEN];
    if (platform_otp_read(OTP_BINDING_OFFSET, readback, sizeof(readback)) != 0)
        return 0;

    qr_data_t read_qr;
    if (!unpack_binding(readback, sizeof(readback), &read_qr))
        return 0;

    if (!read_qr.valid)
        return 0;

    if (strcmp(read_qr.serial, expected->serial) != 0)
        return 0;

    if (strcmp(read_qr.mac_address, expected->mac_address) != 0)
        return 0;

    return 1;
}

/* Read binding from OTP */
int qr_read_otp(qr_data_t *out)
{
    uint8_t readback[OTP_BINDING_MAX_LEN];
    if (platform_otp_read(OTP_BINDING_OFFSET, readback, sizeof(readback)) != 0)
        return 0;

    return unpack_binding(readback, sizeof(readback), out);
}

/* Check if OTP has already been programmed */
int qr_is_bound(void)
{
    uint8_t header[2];
    if (platform_otp_read(OTP_BINDING_OFFSET, header, 2) != 0)
        return 0;

    return (header[0] == OTP_VERSION && (header[1] & OTP_FLAG_BOUND));
}

/* ------------------------------------------------------------------ */
/*  Test / example                                                    */
/* ------------------------------------------------------------------ */

#ifdef UNIT_TEST
static int test_passed = 0;
static int test_failed = 0;

#define TEST_START(name)  do { printf("  TEST %-55s ", name); } while(0)
#define TEST_PASS()       do { printf("PASS\n"); test_passed++; } while(0)
#define TEST_FAIL(msg)    do { printf("FAIL: %s\n", msg); test_failed++; } while(0)

int main(void)
{
    printf("=== QR Code Binding Tests ===\n\n");

    /* Test QR parsing */
    TEST_START("QR parse valid data");
    const char *qr_raw = "H747-ELITE,SN20240101-001,00:1A:2B:3C:4D:5E";
    qr_data_t qr;
    int rc = qr_parse(qr_raw, strlen(qr_raw), &qr);
    if (rc && qr.valid &&
        strcmp(qr.serial, "SN20240101-001") == 0 &&
        strcmp(qr.mac_address, "00:1A:2B:3C:4D:5E") == 0)
        TEST_PASS();
    else
        TEST_FAIL("parse failed");

    /* Test invalid prefix */
    TEST_START("QR parse invalid prefix");
    rc = qr_parse("OTHER,SN001,00:00:00:00:00:00", 30, &qr);
    if (!rc)
        TEST_PASS();
    else
        TEST_FAIL("should have rejected unknown prefix");

    /* Test invalid MAC */
    TEST_START("QR parse invalid MAC format");
    rc = qr_parse("H747-ELITE,SN001,00:00:00:00:00:GG", 32, &qr);
    if (!rc)
        TEST_PASS();
    else
        TEST_FAIL("should have rejected bad MAC");

    /* Test bind to OTP */
    TEST_START("QR bind to OTP with matching serials");
    otp_sim_init();
    const char *mes_serial = "SN20240101-001";
    memset(&qr, 0, sizeof(qr));
    qr_parse(qr_raw, strlen(qr_raw), &qr);

    bind_result_t br = qr_bind_to_otp(&qr, mes_serial);
    if (br == BIND_OK)
        TEST_PASS();
    else
        TEST_FAIL("bind failed");

    /* Test OTP verify */
    TEST_START("OTP verify after bind");
    int verified = qr_verify_otp(&qr);
    if (verified)
        TEST_PASS();
    else
        TEST_FAIL("verify failed");

    /* Test OTP readback */
    TEST_START("OTP readback matches original");
    qr_data_t readback;
    memset(&readback, 0, sizeof(readback));
    int rd = qr_read_otp(&readback);
    if (rd && readback.valid &&
        strcmp(readback.serial, qr.serial) == 0 &&
        strcmp(readback.mac_address, qr.mac_address) == 0)
        TEST_PASS();
    else
        TEST_FAIL("readback mismatch");

    /* Test serial mismatch rejection */
    TEST_START("Serial mismatch rejection");
    otp_sim_init();
    br = qr_bind_to_otp(&qr, "WRONG-SERIAL");
    if (br == BIND_ERR_SERIAL_MISMATCH)
        TEST_PASS();
    else
        TEST_FAIL("expected serial mismatch error");

    /* Test is_bound */
    TEST_START("is_bound returns 1 after binding");
    otp_sim_init();
    qr_bind_to_otp(&qr, mes_serial);
    int bound = qr_is_bound();
    if (bound)
        TEST_PASS();
    else
        TEST_FAIL("expected bound=true");

    printf("\nResults: %d passed, %d failed\n", test_passed, test_failed);
    return test_failed > 0 ? 1 : 0;
}
#endif
