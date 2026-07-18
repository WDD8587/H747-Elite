/**
 * @file    test_spi.c
 * @brief   Unit tests for SPI IPC transport (protocol-level, no HW).
 *
 * Tests the SPI frame packing/unpacking, CRC16 validation, and
 * double-buffer logic in isolation from STM32 HAL.
 *
 * Run: gcc -std=gnu99 -I.. -I../unity test_spi.c ../unity/unity.c -lm
 */

#include "../unity/unity.h"
#include "../../shared/inc/ipc_proto.h"
#include <string.h>
#include <stdint.h>

#define FRAME 64

/* ---- CRC16 table (same as production) ---- */
static const uint16_t tbl[256] = {
    0x0000,0xC0C1,0xC181,0x0140,0xC301,0x03C0,0x0280,0xC241,
    0xC601,0x06C0,0x0780,0xC741,0x0500,0xC5C1,0xC481,0x0440,
    0xCC01,0x0CC0,0x0D80,0xCD41,0x0F00,0xCFC1,0xCE81,0x0E40,
    0x0A00,0xCAC1,0xCB81,0x0B40,0xC901,0x09C0,0x0880,0xC841,
    0xD801,0x18C0,0x1980,0xD941,0x1B00,0xDBC1,0xDA81,0x1A40,
    0x1E00,0xDEC1,0xDF81,0x1F40,0xDD01,0x1DC0,0x1C80,0xDC41,
    0x1400,0xD4C1,0xD581,0x1540,0xD701,0x17C0,0x1680,0xD641,
    0xD201,0x12C0,0x1380,0xD341,0x1100,0xD1C1,0xD081,0x1040,
    0xF001,0x30C0,0x3180,0xF141,0x3300,0xF3C1,0xF281,0x3240,
    0x3600,0xF6C1,0xF781,0x3740,0xF501,0x35C0,0x3480,0xF441,
    0x3C00,0xFCC1,0xFD81,0x3D40,0xFF01,0x3FC0,0x3E80,0xFE41,
    0xFA01,0x3AC0,0x3B80,0xFB41,0x3900,0xF9C1,0xF881,0x3840,
    0x2800,0xE8C1,0xE981,0x2940,0xEB01,0x2BC0,0x2A80,0xEA41,
    0xEE01,0x2EC0,0x2F80,0xEF41,0x2D00,0xEDC1,0xEC81,0x2C40,
    0xE401,0x24C0,0x2580,0xE541,0x2700,0xE7C1,0xE681,0x2640,
    0x2200,0xE2C1,0xE381,0x2340,0xE101,0x21C0,0x2080,0xE041,
    0xA001,0x60C0,0x6180,0xA141,0x6300,0xA3C1,0xA281,0x6240,
    0x6600,0xA6C1,0xA781,0x6740,0xA501,0x65C0,0x6480,0xA441,
    0x6C00,0xACC1,0xAD81,0x6D40,0xAF01,0x6FC0,0x6E80,0xAE41,
    0xAA01,0x6AC0,0x6B80,0xAB41,0x6900,0xA9C1,0xA881,0x6840,
    0x7800,0xB8C1,0xB981,0x7940,0xBB01,0x7BC0,0x7A80,0xBA41,
    0xBE01,0x7EC0,0x7F80,0xBF41,0x7D00,0xBDC1,0xBC81,0x7C40,
    0xB401,0x74C0,0x7580,0xB541,0x7700,0xB7C1,0xB681,0x7640,
    0x7200,0xB2C1,0xB381,0x7340,0xB101,0x71C0,0x7080,0xB041,
    0x5000,0x90C1,0x9181,0x5140,0x9301,0x53C0,0x5280,0x9241,
    0x9601,0x56C0,0x5780,0x9741,0x5500,0x95C1,0x9481,0x5440,
    0x9C01,0x5CC0,0x5D80,0x9D41,0x5F00,0x9FC1,0x9E81,0x5E40,
    0x5A00,0x9AC1,0x9B81,0x5B40,0x9901,0x59C0,0x5880,0x9841,
    0x8801,0x48C0,0x4980,0x8941,0x4B00,0x8BC1,0x8A81,0x4A40,
    0x4E00,0x8EC1,0x8F81,0x4F40,0x8D01,0x4DC0,0x4C80,0x8C41,
    0x4400,0x84C1,0x8581,0x4540,0x8701,0x47C0,0x4680,0x8641,
    0x8201,0x42C0,0x4380,0x8341,0x4100,0x81C1,0x8081,0x4040
};

static uint16_t crc16(const uint8_t *d, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    while (len--) crc = (crc >> 8) ^ tbl[(crc ^ *d++) & 0xFF];
    return crc;
}

/* Simulate SPI full-duplex frame packing/unpacking */
static void spi_frame_pack(const l1_report_t *rpt, uint8_t *tx)
{
    memset(tx, 0, FRAME);
    memcpy(tx, rpt, sizeof(l1_report_t));
}

static int spi_frame_unpack_cmd(const uint8_t *rx, l2_cmd_t *cmd)
{
    if (rx[0] != IPC_HEAD_RK) return -1;
    uint16_t c = crc16(rx, sizeof(l2_cmd_t) - 2);
    uint16_t r;
    memcpy(&r, rx + sizeof(l2_cmd_t) - 2, 2);
    if (c != r) return -2;
    memcpy(cmd, rx, sizeof(l2_cmd_t));
    return 0;
}

static int spi_frame_unpack_report(const uint8_t *rx, l1_report_t *rpt)
{
    if (rx[0] != IPC_HEAD_M7) return -1;
    uint16_t c = crc16(rx, sizeof(l1_report_t) - 2);
    uint16_t r;
    memcpy(&r, rx + sizeof(l1_report_t) - 2, 2);
    if (c != r) return -2;
    memcpy(rpt, rx, sizeof(l1_report_t));
    return 0;
}

/* ---- Tests ---- */

void test_spi_frame_size(void)
{
    unity_assert_true("l1_report_t fits in 64B",
        sizeof(l1_report_t) <= FRAME);
    unity_assert_true("l2_cmd_t fits in 64B",
        sizeof(l2_cmd_t) <= FRAME);
}

void test_spi_pack_roundtrip(void)
{
    l1_report_t orig;
    memset(&orig, 0, sizeof(orig));
    orig.head      = IPC_HEAD_M7;
    orig.seq       = 42;
    orig.odom.magic  = 0x4C314C31;
    orig.odom.enc_l  = 12345;
    orig.odom.enc_r  = 67890;
    orig.odom.imu_yaw = 90.0f;
    orig.crc = crc16((uint8_t *)&orig, sizeof(orig) - 2);

    uint8_t tx[FRAME];
    spi_frame_pack(&orig, tx);

    l1_report_t recovered;
    unity_assert_equal_int("Unpack OK", 0,
        spi_frame_unpack_report(tx, &recovered));
    unity_assert_equal_int("Seq roundtrip", 42, (int)recovered.seq);
    unity_assert_equal_int("enc_l roundtrip", 12345, (int)recovered.odom.enc_l);
    unity_assert_equal_float("yaw roundtrip", 90.0f, recovered.odom.imu_yaw, 0.1f);
}

void test_spi_corrupted_frame_rejected(void)
{
    l1_report_t rpt;
    memset(&rpt, 0, sizeof(rpt));
    rpt.head = IPC_HEAD_M7;
    rpt.crc  = crc16((uint8_t *)&rpt, sizeof(rpt) - 2);

    uint8_t tx[FRAME];
    spi_frame_pack(&rpt, tx);

    /* Corrupt byte 3 */
    tx[3] ^= 0xFF;

    l1_report_t recovered;
    int err = spi_frame_unpack_report(tx, &recovered);
    unity_assert_true("Corrupted CRC rejected", err != 0);
}

void test_spi_cmd_pack_roundtrip(void)
{
    l2_cmd_t orig;
    memset(&orig, 0, sizeof(orig));
    orig.head  = IPC_HEAD_RK;
    orig.v     = 250.0f;
    orig.w     = -30.0f;
    orig.flags = FLAG_DOCK;
    orig.crc   = crc16((uint8_t *)&orig, sizeof(orig) - 2);

    uint8_t tx[FRAME];
    memset(tx, 0, FRAME);
    memcpy(tx, &orig, sizeof(orig));

    l2_cmd_t recovered;
    int ret = spi_frame_unpack_cmd(tx, &recovered);
    unity_assert_equal_int("Cmd unpack OK", 0, ret);
    unity_assert_equal_float("Cmd v", 250.0f, recovered.v, 0.01f);
    unity_assert_equal_float("Cmd w", -30.0f, recovered.w, 0.01f);
    unity_assert_equal_int("Cmd flags", FLAG_DOCK, (int)recovered.flags);
}

void test_spi_double_buffer_pingpong(void)
{
    /* Simulate ping/pong: two buffers, alternate writes */
    uint8_t buf[2][FRAME];
    memset(buf, 0, sizeof(buf));

    l1_report_t rpt[2];
    for (int i = 0; i < 2; i++) {
        memset(&rpt[i], 0, sizeof(rpt[i]));
        rpt[i].head = IPC_HEAD_M7;
        rpt[i].seq  = (uint8_t)(i + 1);
        rpt[i].odom.enc_l = i * 1000;
        rpt[i].crc = crc16((uint8_t *)&rpt[i], sizeof(rpt[i]) - 2);
    }

    memcpy(buf[0], &rpt[0], sizeof(l1_report_t));
    memcpy(buf[1], &rpt[1], sizeof(l1_report_t));

    /* Validate both buffers survived without corruption */
    l1_report_t check;
    unity_assert_equal_int("Buf0 unpack", 0,
        spi_frame_unpack_report(buf[0], &check));
    unity_assert_equal_int("Buf0 seq", 1, (int)check.seq);
    unity_assert_equal_int("Buf1 unpack", 0,
        spi_frame_unpack_report(buf[1], &check));
    unity_assert_equal_int("Buf1 seq", 2, (int)check.seq);
}

void test_spi_bad_header_rejected(void)
{
    uint8_t rx[FRAME];
    memset(rx, 0, FRAME);
    rx[0] = 0xFF;  /* invalid header */

    l1_report_t rpt;
    unity_assert_true("Bad rpt header rejected",
        spi_frame_unpack_report(rx, &rpt) != 0);

    l2_cmd_t cmd;
    unity_assert_true("Bad cmd header rejected",
        spi_frame_unpack_cmd(rx, &cmd) != 0);
}
