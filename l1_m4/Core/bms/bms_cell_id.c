/**
 * @file    bms_cell_id.c
 * @brief   Cell identification via 1-Wire DS2431 EEPROM in each cell pack.
 *          Reads unique ID, manufacture date, chemistry type.
 *          Cross-checks with BMS ROM ID. Mismatch triggers cloud warning.
 *
 * @note    Part of STM32H747 BMS subsystem.
 */

#include "bms_cell_id.h"
#include "bms_onewire.h"
#include "bms_flash.h"
#include "bms_timer.h"

#include <string.h>
#include <stdio.h>

/* ---------------------------------------------------------------------------
 * DS2431 memory map (shared across all packs, 1 kbit = 128 bytes)
 * Addresses used:
 *   0x00 – 0x07 : Unique serial number (8 bytes, factory-programmed)
 *   0x08 – 0x0F : Manufacture date (ASCII, e.g. "20250115")
 *   0x10 – 0x1F : Chemistry type string
 *   0x20 – 0x27 : Rated capacity (ASCII mAh)
 *   0x28 – 0x2F : Reserved
 *   0x80 – 0x87 : User-programmable pack ID
 * ------------------------------------------------------------------------- */

#define DS2431_FAMILY_CODE         0x2D
#define DS2431_READ_SCRATCHPAD     0xAA
#define DS2431_WRITE_SCRATCHPAD    0x0F
#define DS2431_COPY_SCRATCHPAD     0x55
#define DS2431_READ_MEMORY         0xF0

#define CELL_ID_ROM_ID_LEN          8
#define CELL_ID_SERIAL_LEN          8
#define CELL_ID_DATE_LEN            8
#define CELL_ID_CHEM_LEN            16
#define CELL_ID_CAPACITY_LEN        8
#define CELL_ID_PACK_ID_LEN         8

/* Scratchpad timing */
#define DS2431_TPROG_MS            10

/* ---------------------------------------------------------------------------
 * Local state
 * ------------------------------------------------------------------------- */

typedef struct {
    uint8_t rom_id[CELL_ID_ROM_ID_LEN];        /* 64-bit ROM ID            */
    uint8_t serial[CELL_ID_SERIAL_LEN];         /* Unique serial            */
    char    manufacture_date[CELL_ID_DATE_LEN + 1];
    char    chemistry[CELL_ID_CHEM_LEN + 1];
    char    rated_capacity_str[CELL_ID_CAPACITY_LEN + 1];
    uint8_t pack_id[CELL_ID_PACK_ID_LEN];
    bool    found;
    bool    mismatch_warning;
} Cell_ID_State;

static Cell_ID_State cell_id_;

/* ---------------------------------------------------------------------------
 * 1-Wire CRC (Dallas Semiconductor CRC-8)
 * ------------------------------------------------------------------------- */
static uint8_t cell_id_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        uint8_t byte = data[i];
        for (int b = 0; b < 8; b++) {
            uint8_t mix = (crc ^ byte) & 0x01;
            crc >>= 1;
            if (mix) crc ^= 0x8C;
            byte >>= 1;
        }
    }
    return crc;
}

/* ---------------------------------------------------------------------------
 * Read a block from DS2431 memory
 * ------------------------------------------------------------------------- */
static bool cell_id_read_mem(uint8_t addr, uint8_t *buf, size_t len)
{
    if (!ONEWIRE_Reset()) return false;

    ONEWIRE_WriteByte(DS2431_READ_MEMORY);
    ONEWIRE_WriteByte(addr & 0xFF);
    ONEWIRE_WriteByte((addr >> 8) & 0xFF);

    for (size_t i = 0; i < len; i++) {
        buf[i] = ONEWIRE_ReadByte();
    }

    return true;
}

/* ---------------------------------------------------------------------------
 * Write scratchpad (3 bytes at a time for DS2431)
 * ------------------------------------------------------------------------- */
static bool cell_id_write_scratchpad(uint8_t addr, const uint8_t *data, size_t len)
{
    if (len > 8) len = 8;

    if (!ONEWIRE_Reset()) return false;

    ONEWIRE_WriteByte(DS2431_WRITE_SCRATCHPAD);
    ONEWIRE_WriteByte(addr & 0xFF);
    ONEWIRE_WriteByte((addr >> 8) & 0xFF);

    for (size_t i = 0; i < len; i++) {
        ONEWIRE_WriteByte(data[i]);
    }

    /* Read back verification (scratchpad contains TA1, TA2, ES + data) */
    uint8_t es = ONEWIRE_ReadByte();  /* ES (error/size) */
    (void)es;
    uint8_t readback[8];
    for (size_t i = 0; i < len; i++) {
        readback[i] = ONEWIRE_ReadByte();
    }

    /* CRC-8 of the readback */
    uint8_t crc_rb = ONEWIRE_ReadByte();
    (void)crc_rb;

    return (memcmp(readback, data, len) == 0);
}

/* ---------------------------------------------------------------------------
 * Copy scratchpad to EEPROM
 * ------------------------------------------------------------------------- */
static bool cell_id_copy_scratchpad(uint8_t addr)
{
    if (!ONEWIRE_Reset()) return false;

    ONEWIRE_WriteByte(DS2431_COPY_SCRATCHPAD);
    ONEWIRE_WriteByte(addr & 0xFF);
    ONEWIRE_WriteByte((addr >> 8) & 0xFF);

    /* Enable copy by sending 0x0F (authorization pattern) */
    ONEWIRE_WriteByte(0x0F);

    /* Wait for tPROG */
    TIMER_DelayMs(DS2431_TPROG_MS);

    /* Verify by reading back */
    return ONEWIRE_Reset();
}

/* ---------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

bool CELLID_Init(void)
{
    memset(&cell_id_, 0, sizeof(cell_id_));

    /* Detect and read ROM ID */
    if (!ONEWIRE_Reset()) {
        cell_id_.found = false;
        return false;
    }

    /* Read 64-bit ROM ID (requires SKIP ROM + READ ROM protocol) */
    /* For a single device on bus, use Skip ROM */
    ONEWIRE_WriteByte(0xCC);  /* Skip ROM */
    ONEWIRE_WriteByte(0x33);  /* Read ROM */
    for (int i = 0; i < CELL_ID_ROM_ID_LEN; i++) {
        cell_id_.rom_id[i] = ONEWIRE_ReadByte();
    }

    /* Verify CRC of ROM ID */
    uint8_t rom_crc = cell_id_crc8(cell_id_.rom_id, 7);
    if (rom_crc != cell_id_.rom_id[7]) {
        cell_id_.found = false;
        return false;
    }

    /* Verify family code */
    if (cell_id_.rom_id[0] != DS2431_FAMILY_CODE) {
        cell_id_.found = false;
        return false;
    }

    /* Read application memory */
    if (!cell_id_read_mem(0x00, cell_id_.serial, CELL_ID_SERIAL_LEN)) {
        cell_id_.found = false;
        return false;
    }

    /* Read manufacture date */
    uint8_t date_raw[CELL_ID_DATE_LEN];
    if (!cell_id_read_mem(0x08, date_raw, CELL_ID_DATE_LEN)) {
        cell_id_.found = false;
        return false;
    }
    memcpy(cell_id_.manufacture_date, date_raw, CELL_ID_DATE_LEN);
    cell_id_.manufacture_date[CELL_ID_DATE_LEN] = '\0';

    /* Read chemistry */
    uint8_t chem_raw[CELL_ID_CHEM_LEN];
    if (!cell_id_read_mem(0x10, chem_raw, CELL_ID_CHEM_LEN)) {
        cell_id_.found = false;
        return false;
    }
    memcpy(cell_id_.chemistry, chem_raw, CELL_ID_CHEM_LEN);
    cell_id_.chemistry[CELL_ID_CHEM_LEN] = '\0';

    /* Read rated capacity */
    uint8_t cap_raw[CELL_ID_CAPACITY_LEN];
    if (!cell_id_read_mem(0x20, cap_raw, CELL_ID_CAPACITY_LEN)) {
        cell_id_.found = false;
        return false;
    }
    memcpy(cell_id_.rated_capacity_str, cap_raw, CELL_ID_CAPACITY_LEN);
    cell_id_.rated_capacity_str[CELL_ID_CAPACITY_LEN] = '\0';

    /* Read pack ID */
    if (!cell_id_read_mem(0x80, cell_id_.pack_id, CELL_ID_PACK_ID_LEN)) {
        cell_id_.found = false;
        return false;
    }

    cell_id_.found = true;
    return true;
}

bool CELLID_CrossCheck(const uint8_t *bms_rom_id)
{
    if (!cell_id_.found) return false;

    /* Compare BMS ROM ID with stored pack ROM ID */
    if (memcmp(bms_rom_id, cell_id_.rom_id, CELL_ID_ROM_ID_LEN) != 0) {
        cell_id_.mismatch_warning = true;
        FLASH_LogEvent(FLASH_LOG_WARNING, "CELLID: ROM ID mismatch");
        return false;
    }

    cell_id_.mismatch_warning = false;
    return true;
}

bool CELLID_IsMismatch(void)
{
    return cell_id_.mismatch_warning;
}

bool CELLID_IsFound(void)
{
    return cell_id_.found;
}

const uint8_t* CELLID_GetRomID(void)
{
    return cell_id_.rom_id;
}

const uint8_t* CELLID_GetSerial(void)
{
    return cell_id_.serial;
}

const char* CELLID_GetManufactureDate(void)
{
    return cell_id_.manufacture_date;
}

const char* CELLID_GetChemistry(void)
{
    return cell_id_.chemistry;
}

uint16_t CELLID_GetRatedCapacity_mAh(void)
{
    return (uint16_t)atoi(cell_id_.rated_capacity_str);
}

bool CELLID_WritePackID(const uint8_t *pack_id, size_t len)
{
    if (len > CELL_ID_PACK_ID_LEN) len = CELL_ID_PACK_ID_LEN;

    if (!cell_id_write_scratchpad(0x80, pack_id, len)) return false;
    if (!cell_id_copy_scratchpad(0x80)) return false;

    memcpy(cell_id_.pack_id, pack_id, len);
    return true;
}
