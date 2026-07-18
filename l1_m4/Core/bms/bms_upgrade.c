/**
 * @file    bms_upgrade.c
 * @brief   BMS firmware upgrade via SMBus.
 *          Sequence:
 *            1. Enter ROM mode (send 0x0F00 to ManufacturerAccess)
 *            2. Flash new firmware in 128-byte blocks
 *            3. Verify CRC per block
 *            4. Exit ROM mode
 *          Timeout: 30 s total.
 *
 * @note    Part of STM32H747 BMS subsystem.
 */

#include "bms_upgrade.h"
#include "bms_smbus.h"
#include "bms_flash.h"
#include "bms_timer.h"

#include <string.h>

/* ---------------------------------------------------------------------------
 * SMBus command codes (BQ76952 / BMS bootloader convention)
 * ------------------------------------------------------------------------- */
#define UPGRADE_MANUFACTURER_ACCESS     0x00
#define UPGRADE_ROM_MODE_KEY            0x0F00
#define UPGRADE_ROM_EXIT_KEY            0x0001

#define UPGRADE_BLOCK_SIZE              128
#define UPGRADE_TIMEOUT_MS             30000   /* 30 s total              */
#define UPGRADE_MAX_BLOCKS             1024    /* Max firmware size       */
#define UPGRADE_CRC_INIT               0xFFFF

#define UPGRADE_SMBUS_ADDR             0x16    /* BMS SMBus address       */

/* ---------------------------------------------------------------------------
 * Local state
 * ------------------------------------------------------------------------- */

typedef enum {
    UPGRADE_IDLE,
    UPGRADE_ENTER_ROM,
    UPGRADE_TRANSFER,
    UPGRADE_VERIFY,
    UPGRADE_EXIT_ROM,
    UPGRADE_COMPLETE,
    UPGRADE_FAILED
} Upgrade_Phase;

typedef struct {
    Upgrade_Phase   phase;
    uint32_t        start_tick;
    uint32_t        total_bytes;
    uint32_t        transferred_bytes;
    uint16_t        expected_crc[UPGRADE_MAX_BLOCKS];
    uint16_t        computed_crc[UPGRADE_MAX_BLOCKS];
    uint8_t         block_buf[UPGRADE_BLOCK_SIZE];
    uint8_t         retry_count;
    bool            initialized;
} Upgrade_State;

static Upgrade_State upg_;

/* ---------------------------------------------------------------------------
 * CRC-16-IBM for block verification
 * ------------------------------------------------------------------------- */
static uint16_t upgrade_crc16(const uint8_t *data, size_t len, uint16_t crc)
{
    static const uint16_t poly = 0x8005;
    for (size_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i]) << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000U) ? ((crc << 1) ^ poly) : (crc << 1);
        }
    }
    return crc;
}

/* ---------------------------------------------------------------------------
 * SMBus block write / read helpers
 * ------------------------------------------------------------------------- */
static bool upgrade_smbus_write_block(uint8_t cmd, const uint8_t *data, size_t len)
{
    return SMBUS_WriteBlock(UPGRADE_SMBUS_ADDR, cmd, data, len);
}

static bool upgrade_smbus_read_block(uint8_t cmd, uint8_t *data, size_t *len)
{
    return SMBUS_ReadBlock(UPGRADE_SMBUS_ADDR, cmd, data, len);
}

static bool upgrade_smbus_write_word(uint8_t cmd, uint16_t word)
{
    return SMBUS_WriteWord(UPGRADE_SMBUS_ADDR, cmd, word);
}

static bool upgrade_smbus_read_word(uint8_t cmd, uint16_t *word)
{
    return SMBUS_ReadWord(UPGRADE_SMBUS_ADDR, cmd, word);
}

/* ---------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

bool UPGRADE_Init(void)
{
    memset(&upg_, 0, sizeof(upg_));
    upg_.phase       = UPGRADE_IDLE;
    upg_.initialized = true;
    return true;
}

bool UPGRADE_Start(const uint8_t *firmware, uint32_t length)
{
    if (!upg_.initialized) return false;
    if (upg_.phase != UPGRADE_IDLE) return false;
    if (length == 0 || length > UPGRADE_MAX_BLOCKS * UPGRADE_BLOCK_SIZE) return false;

    upg_.phase            = UPGRADE_ENTER_ROM;
    upg_.start_tick       = TIMER_GetTick();
    upg_.total_bytes      = length;
    upg_.transferred_bytes = 0;
    upg_.retry_count      = 0;

    /* Pre-compute expected CRC per block */
    for (uint32_t offset = 0; offset < length; offset += UPGRADE_BLOCK_SIZE) {
        size_t blk_len = (offset + UPGRADE_BLOCK_SIZE <= length) ? UPGRADE_BLOCK_SIZE
                                                                 : (length - offset);
        upg_.expected_crc[offset / UPGRADE_BLOCK_SIZE] =
            upgrade_crc16(firmware + offset, blk_len, UPGRADE_CRC_INIT);
    }

    return true;
}

UPGRADE_Result UPGRADE_Task(const uint8_t *firmware, uint32_t length)
{
    if (!upg_.initialized) return UPGRADE_RESULT_FAILED;

    uint32_t elapsed = TIMER_GetTick() - upg_.start_tick;
    if (elapsed >= UPGRADE_TIMEOUT_MS && upg_.phase != UPGRADE_COMPLETE) {
        upg_.phase = UPGRADE_FAILED;
        return UPGRADE_RESULT_TIMEOUT;
    }

    size_t  blk_len;
    uint16_t status;
    uint8_t  resp[UPGRADE_BLOCK_SIZE + 4];

    switch (upg_.phase) {

    case UPGRADE_ENTER_ROM:
        /* Send ROM mode key to ManufacturerAccess */
        if (!upgrade_smbus_write_word(UPGRADE_MANUFACTURER_ACCESS,
                                       UPGRADE_ROM_MODE_KEY)) {
            if (++upg_.retry_count > 3) {
                upg_.phase = UPGRADE_FAILED;
                return UPGRADE_RESULT_FAILED;
            }
            TIMER_DelayMs(10);
            return UPGRADE_RESULT_BUSY;
        }
        upg_.retry_count = 0;
        upg_.phase = UPGRADE_TRANSFER;
        upg_.start_tick = TIMER_GetTick();  /* Reset timeout for transfer */
        return UPGRADE_RESULT_BUSY;

    case UPGRADE_TRANSFER:
        if (upg_.transferred_bytes >= length) {
            upg_.phase = UPGRADE_VERIFY;
            return UPGRADE_RESULT_BUSY;
        }

        /* Prepare block */
        blk_len = (upg_.transferred_bytes + UPGRADE_BLOCK_SIZE <= length)
                  ? UPGRADE_BLOCK_SIZE
                  : (length - upg_.transferred_bytes);

        memcpy(upg_.block_buf, firmware + upg_.transferred_bytes, blk_len);
        if (blk_len < UPGRADE_BLOCK_SIZE) {
            memset(upg_.block_buf + blk_len, 0xFF, UPGRADE_BLOCK_SIZE - blk_len);
        }

        /* Send block */
        if (!upgrade_smbus_write_block(0x01, upg_.block_buf, UPGRADE_BLOCK_SIZE)) {
            if (++upg_.retry_count > 3) {
                upg_.phase = UPGRADE_FAILED;
                return UPGRADE_RESULT_FAILED;
            }
            TIMER_DelayMs(5);
            return UPGRADE_RESULT_BUSY;
        }

        /* Request CRC verification */
        if (!upgrade_smbus_read_word(0x02, &status)) {
            if (++upg_.retry_count > 3) {
                upg_.phase = UPGRADE_FAILED;
                return UPGRADE_RESULT_FAILED;
            }
            TIMER_DelayMs(5);
            return UPGRADE_RESULT_BUSY;
        }

        /* Store firmware CRC (status is firmware CRC check result) */
        upg_.computed_crc[upg_.transferred_bytes / UPGRADE_BLOCK_SIZE] = status;
        upg_.retry_count = 0;
        upg_.transferred_bytes += blk_len;

        return UPGRADE_RESULT_BUSY;

    case UPGRADE_VERIFY:
        /* Verify all blocks */
        for (uint32_t i = 0; i < (length + UPGRADE_BLOCK_SIZE - 1) / UPGRADE_BLOCK_SIZE; i++) {
            if (upg_.expected_crc[i] != upg_.computed_crc[i]) {
                upg_.phase = UPGRADE_FAILED;
                return UPGRADE_RESULT_CRC_MISMATCH;
            }
        }
        upg_.phase = UPGRADE_EXIT_ROM;
        return UPGRADE_RESULT_BUSY;

    case UPGRADE_EXIT_ROM:
        if (!upgrade_smbus_write_word(UPGRADE_MANUFACTURER_ACCESS,
                                       UPGRADE_ROM_EXIT_KEY)) {
            if (++upg_.retry_count > 3) {
                upg_.phase = UPGRADE_FAILED;
                return UPGRADE_RESULT_FAILED;
            }
            TIMER_DelayMs(10);
            return UPGRADE_RESULT_BUSY;
        }

        upg_.phase = UPGRADE_COMPLETE;
        return UPGRADE_RESULT_SUCCESS;

    case UPGRADE_COMPLETE:
        return UPGRADE_RESULT_SUCCESS;

    case UPGRADE_FAILED:
    default:
        return UPGRADE_RESULT_FAILED;
    }
}

void UPGRADE_Reset(void)
{
    upg_.phase            = UPGRADE_IDLE;
    upg_.transferred_bytes = 0;
    upg_.retry_count      = 0;
}

bool UPGRADE_IsBusy(void)
{
    return (upg_.phase == UPGRADE_ENTER_ROM ||
            upg_.phase == UPGRADE_TRANSFER ||
            upg_.phase == UPGRADE_VERIFY ||
            upg_.phase == UPGRADE_EXIT_ROM);
}

bool UPGRADE_IsComplete(void)
{
    return (upg_.phase == UPGRADE_COMPLETE);
}

bool UPGRADE_IsFailed(void)
{
    return (upg_.phase == UPGRADE_FAILED);
}
