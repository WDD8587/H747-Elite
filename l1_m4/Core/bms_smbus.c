/**
  ******************************************************************************
  * @file    bms_smbus.c
  * @author  H747 Elite Team
  * @brief   SMBus Smart Battery 4S driver (BQ40Z50 or compatible gas gauge).
  *
  *          Translates ITE EC SB1.1 experience to STM32H7:
  *            - SMBus Read Word / Read Block with PEC (CRC-8, poly 0x07)
  *            - Retry logic (3 attempts)
  *            - Smart Battery command set: Voltage (0x09), Current (0x0A),
  *              RSOC (0x0D), Temperature (0x08), Cell Voltages (0x3F)
  *            - Results written into caller-provided BmsData_t
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include <string.h>
#include <stdint.h>

/* ---------------------------------------------------------------------------
 * BQ40Z50 SMBus address (7-bit, left-aligned)
 * --------------------------------------------------------------------------- */
#define BQ40Z50_ADDR_7BIT   0x0BU   /* 0x16 >> 1 */

/* SMBus command codes -- Smart Battery specification v1.1 */
#define SB_CMD_TEMP          0x08U   /* Temperature                   */
#define SB_CMD_VOLTAGE       0x09U   /* Voltage                       */
#define SB_CMD_CURRENT       0x0AU   /* Current (signed)              */
#define SB_CMD_RSOC          0x0DU   /* Relative State Of Charge      */
#define SB_CMD_CELL_VOLTAGES 0x3FU   /* Cell Voltages (block read)    */
#define SB_CMD_MANUF_ACCESS  0x00U   /* ManufacturerAccess            */

/* SMBus timeouts (ms) */
#define SMBUS_TIMEOUT_MS     50U
#define SMBUS_RETRY_MAX      3U

/* ---------------------------------------------------------------------------
 * BmsData_t -- public data structure
 * --------------------------------------------------------------------------- */
typedef struct {
    uint16_t voltage_mV;       /* pack voltage, mV                     */
    int16_t  current_mA;       /* pack current, mA (neg = discharge)   */
    uint8_t  rsoc;             /* Relative State-Of-Charge 0-100       */
    int16_t  temp_dK;          /* temperature, deci-Kelvin             */
    uint16_t cell_mV[4];       /* individual cell voltages, mV         */
    uint8_t  valid;            /* 1 if all reads succeeded             */
} BmsData_t;

/* ---------------------------------------------------------------------------
 * Module state
 * --------------------------------------------------------------------------- */
static uint8_t  s_last_pec_error = 0;
static uint32_t s_bus_error_cnt  = 0;

/*============================================================================
 *  SMBus CRC-8 (PEC byte) -- polynomial 0x07 (x^8 + x^2 + x + 1)
 *
 *  SMBus v1.1 Specification:
 *    - Initial value: 0x00
 *    - No final XOR
 *============================================================================*/
static uint8_t SMBus_CRC8(const uint8_t *data, uint32_t len)
{
    uint8_t crc = 0x00;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint32_t j = 0; j < 8; j++) {
            if (crc & 0x80U) {
                crc = (uint8_t)((crc << 1) ^ 0x07U);
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }
    return crc;
}

/*============================================================================
 *  SMBus_ReadWord -- read a 16-bit word with PEC verification
 *
 *  Protocol:
 *    START, Addr+W, Cmd, REPEATED_START, Addr+R,
 *    DataLow, DataHigh, PEC, NACK, STOP
 *
 *  Returns 0 on success, -1 on I2C error or PEC mismatch.
 *============================================================================*/
static int SMBus_ReadWord(I2C_HandleTypeDef *hi2c,
                          uint8_t  slave_addr,
                          uint8_t  cmd,
                          uint16_t *data)
{
    uint8_t tx_buf[2];
    uint8_t rx_buf[3];   /* data low, data high, PEC */
    uint8_t pec_buf[5];  /* Addr+W, Cmd, Addr+R, DataLow, DataHigh */
    uint8_t pec_calc;
    int     retry;

    for (retry = 0; retry < SMBUS_RETRY_MAX; retry++) {
        /* Send command byte (START -> Addr+W -> Cmd) */
        tx_buf[0] = cmd;
        if (HAL_I2C_Master_Seq_Transmit(hi2c,
                (uint16_t)(slave_addr << 1) | 0x00U,
                tx_buf, 1,
                I2C_FIRST_FRAME,
                SMBUS_TIMEOUT_MS) != HAL_OK) {
            continue;
        }

        /* Receive data + PEC (REP_START -> Addr+R -> DataLo -> DataHi -> PEC) */
        if (HAL_I2C_Master_Seq_Receive(hi2c,
                (uint16_t)(slave_addr << 1) | 0x01U,
                rx_buf, 3,
                I2C_LAST_FRAME,
                SMBUS_TIMEOUT_MS) != HAL_OK) {
            continue;
        }

        /* Build PEC input buffer and verify */
        pec_buf[0] = (uint8_t)(slave_addr << 1);         /* Addr+W */
        pec_buf[1] = cmd;                                 /* Cmd    */
        pec_buf[2] = (uint8_t)(slave_addr << 1) | 0x01U; /* Addr+R */
        pec_buf[3] = rx_buf[0];                           /* Lo     */
        pec_buf[4] = rx_buf[1];                           /* Hi     */

        pec_calc = SMBus_CRC8(pec_buf, 5);

        if (pec_calc != rx_buf[2]) {
            s_last_pec_error++;
            continue;
        }

        *data = (uint16_t)(rx_buf[1] << 8) | rx_buf[0];
        return 0;
    }

    s_bus_error_cnt++;
    return -1;
}

/*============================================================================
 *  SMBus_ReadBlock -- read a block of bytes with PEC verification
 *
 *  Protocol:
 *    START, Addr+W, Cmd, REPEATED_START, Addr+R,
 *    ByteCount, Data[0..N-1], PEC, NACK, STOP
 *
 *  Returns 0 on success, -1 on error.
 *============================================================================*/
static int SMBus_ReadBlock(I2C_HandleTypeDef *hi2c,
                           uint8_t  slave_addr,
                           uint8_t  cmd,
                           uint8_t *data,
                           uint8_t  len)
{
    uint8_t tx_buf[2];
    uint8_t rx_buf[64];
    uint8_t pec_buf[64];
    uint8_t pec_calc;
    uint8_t byte_count;
    uint32_t pec_len;
    int retry;

    if (len > 60) return -1;

    for (retry = 0; retry < SMBUS_RETRY_MAX; retry++) {
        tx_buf[0] = cmd;
        if (HAL_I2C_Master_Seq_Transmit(hi2c,
                (uint16_t)(slave_addr << 1),
                tx_buf, 1,
                I2C_FIRST_FRAME,
                SMBUS_TIMEOUT_MS) != HAL_OK) {
            continue;
        }

        if (HAL_I2C_Master_Seq_Receive(hi2c,
                (uint16_t)(slave_addr << 1) | 0x01U,
                rx_buf, (uint16_t)(len + 2),
                I2C_LAST_FRAME,
                SMBUS_TIMEOUT_MS) != HAL_OK) {
            continue;
        }

        byte_count = rx_buf[0];
        if (byte_count > len) continue;

        memcpy(data, &rx_buf[1], byte_count);

        pec_buf[0] = (uint8_t)(slave_addr << 1);
        pec_buf[1] = cmd;
        pec_buf[2] = (uint8_t)(slave_addr << 1) | 0x01U;
        pec_buf[3] = byte_count;
        memcpy(&pec_buf[4], data, byte_count);
        pec_len = (uint32_t)byte_count + 4U;
        pec_calc = SMBus_CRC8(pec_buf, pec_len);

        if (pec_calc != rx_buf[len + 1]) {
            s_last_pec_error++;
            continue;
        }

        return 0;
    }

    s_bus_error_cnt++;
    return -1;
}

/*============================================================================
 *  BMS_Init -- initialise BMS communication
 *============================================================================*/
void BMS_Init(I2C_HandleTypeDef *i2c)
{
    uint16_t manuf_data = 0;
    (void)i2c;

    s_last_pec_error = 0;
    s_bus_error_cnt  = 0;

    /* Verify slave presence -- read ManufacturerAccess (0x00) */
    if (SMBus_ReadWord(i2c, BQ40Z50_ADDR_7BIT,
                       SB_CMD_MANUF_ACCESS, &manuf_data) != 0) {
        HAL_I2C_Reset(i2c);
        HAL_Delay(10);
        SMBus_ReadWord(i2c, BQ40Z50_ADDR_7BIT,
                       SB_CMD_MANUF_ACCESS, &manuf_data);
    }
}

/*============================================================================
 *  BMS_ReadAll -- read all BMS parameters in one call
 *
 *  Returns 0 on success, -1 if any read failed after all retries.
 *============================================================================*/
int BMS_ReadAll(I2C_HandleTypeDef *i2c, BmsData_t *d)
{
    uint16_t tmp;
    uint8_t  cell_block[12];
    int      result = 0;

    if (d == NULL) return -1;
    memset(d, 0, sizeof(BmsData_t));

    /* Temperature (0x08) -- word, units of 0.1 K */
    if (SMBus_ReadWord(i2c, BQ40Z50_ADDR_7BIT, SB_CMD_TEMP, &tmp) == 0) {
        d->temp_dK = (int16_t)tmp;
    } else { result = -1; }

    /* Voltage (0x09) -- word, mV */
    if (SMBus_ReadWord(i2c, BQ40Z50_ADDR_7BIT, SB_CMD_VOLTAGE, &tmp) == 0) {
        d->voltage_mV = tmp;
    } else { result = -1; }

    /* Current (0x0A) -- word, signed mA */
    if (SMBus_ReadWord(i2c, BQ40Z50_ADDR_7BIT, SB_CMD_CURRENT, &tmp) == 0) {
        d->current_mA = (int16_t)tmp;
    } else { result = -1; }

    /* RSOC (0x0D) -- word, percent */
    if (SMBus_ReadWord(i2c, BQ40Z50_ADDR_7BIT, SB_CMD_RSOC, &tmp) == 0) {
        d->rsoc = (uint8_t)(tmp & 0xFFU);
    } else { result = -1; }

    /* Cell Voltages (0x3F) -- block read, 8 bytes for 4 cells */
    if (SMBus_ReadBlock(i2c, BQ40Z50_ADDR_7BIT,
                        SB_CMD_CELL_VOLTAGES,
                        cell_block, 8) == 0) {
        for (int i = 0; i < 4; i++) {
            d->cell_mV[i] = (uint16_t)(cell_block[2U*i + 1U] << 8)
                          | (uint16_t)(cell_block[2U*i]);
        }
    } else { result = -1; }

    d->valid = (result == 0) ? 1 : 0;
    return result;
}
