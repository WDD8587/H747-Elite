/**
  ******************************************************************************
  * @file    tof_vl53l5cx.c
  * @author  H747 Elite Team
  * @brief   VL53L5CX Time-of-Flight sensor driver in 4x4 fast mode.
  *
  *          - I2C register access with 16-bit register addresses
  *          - 4x4 resolution (16 zones) continuous ranging
  *          - EXTI interrupt on data ready (GPIO1 from sensor)
  *          - Results written to caller-provided buffer
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include <string.h>
#include <stdint.h>

/* ---------------------------------------------------------------------------
 * VL53L5CX I2C address (7-bit)
 * --------------------------------------------------------------------------- */
#define VL53L5CX_ADDR_7BIT    0x29U

/* ---------------------------------------------------------------------------
 * Register map (key registers for VL53L5CX)
 * --------------------------------------------------------------------------- */
#define REG_SOFT_RESET                0x0000U
#define REG_FW_SYSTEM_STATUS          0x0014U
#define REG_PAD_I2C_HV_CFG           0x01F6U
#define REG_GPIO_HV_MUX              0x01F8U
#define REG_DCI_CMD                  0x02FFU
#define REG_DCI_CMD_MSB              0x0300U
#define REG_DCI_CMD_LEN              8U
#define REG_INTERRUPT_CFG            0x0300U
#define REG_RANGE_DATA               0x2000U
#define REG_RANGE_DATA_SIZE          64U       /* 16 zones * 4 bytes    */
#define REG_STATUS                   0x002CU
#define REG_INTERRUPT_CLEAR          0x001AU

#define DCI_CMD_START                0x0001U
#define RESOLUTION_4X4               0x10U
#define TIMING_BUDGET_MS             20U
#define TOF_I2C_TIMEOUT_MS           50U

/* ---------------------------------------------------------------------------
 * Module state
 * --------------------------------------------------------------------------- */
static uint8_t  s_sensor_ready = 0;
static uint32_t s_read_count   = 0;
static uint32_t s_error_count  = 0;

/*============================================================================
 *  I2C register-level helpers
 *============================================================================*/

static int ToF_WriteReg(I2C_HandleTypeDef *i2c, uint16_t reg, uint8_t val)
{
    uint8_t buf[3];
    buf[0] = (uint8_t)(reg >> 8);
    buf[1] = (uint8_t)(reg & 0xFF);
    buf[2] = val;
    if (HAL_I2C_Master_Transmit(i2c,
            (uint16_t)(VL53L5CX_ADDR_7BIT << 1),
            buf, 3, TOF_I2C_TIMEOUT_MS) != HAL_OK) {
        s_error_count++;
        return -1;
    }
    return 0;
}

static int ToF_WriteMulti(I2C_HandleTypeDef *i2c, uint16_t reg,
                          const uint8_t *data, uint16_t len)
{
    uint8_t buf[66];
    if (len > 64) return -1;
    buf[0] = (uint8_t)(reg >> 8);
    buf[1] = (uint8_t)(reg & 0xFF);
    memcpy(&buf[2], data, len);
    if (HAL_I2C_Master_Transmit(i2c,
            (uint16_t)(VL53L5CX_ADDR_7BIT << 1),
            buf, (uint16_t)(len + 2),
            TOF_I2C_TIMEOUT_MS) != HAL_OK) {
        s_error_count++;
        return -1;
    }
    return 0;
}

static int ToF_ReadReg(I2C_HandleTypeDef *i2c, uint16_t reg, uint8_t *val)
{
    uint8_t addr[2];
    addr[0] = (uint8_t)(reg >> 8);
    addr[1] = (uint8_t)(reg & 0xFF);
    if (HAL_I2C_Master_Transmit(i2c,
            (uint16_t)(VL53L5CX_ADDR_7BIT << 1),
            addr, 2, TOF_I2C_TIMEOUT_MS) != HAL_OK) return -1;
    if (HAL_I2C_Master_Receive(i2c,
            (uint16_t)(VL53L5CX_ADDR_7BIT << 1),
            val, 1, TOF_I2C_TIMEOUT_MS) != HAL_OK) return -1;
    return 0;
}

static int ToF_ReadMulti(I2C_HandleTypeDef *i2c, uint16_t reg,
                          uint8_t *data, uint16_t len)
{
    uint8_t addr[2];
    addr[0] = (uint8_t)(reg >> 8);
    addr[1] = (uint8_t)(reg & 0xFF);
    if (HAL_I2C_Master_Transmit(i2c,
            (uint16_t)(VL53L5CX_ADDR_7BIT << 1),
            addr, 2, TOF_I2C_TIMEOUT_MS) != HAL_OK) return -1;
    if (HAL_I2C_Master_Receive(i2c,
            (uint16_t)(VL53L5CX_ADDR_7BIT << 1),
            data, len, TOF_I2C_TIMEOUT_MS) != HAL_OK) return -1;
    return 0;
}

/*============================================================================
 *  ToF_Init -- initialise the VL53L5CX in 4x4 continuous mode
 *============================================================================*/
void ToF_Init(I2C_HandleTypeDef *i2c)
{
    uint8_t status, cfg;
    int retry;

    s_sensor_ready = 0;
    s_read_count   = 0;
    s_error_count  = 0;

    ToF_WriteReg(i2c, REG_SOFT_RESET, 0x01);
    HAL_Delay(10);

    for (retry = 0; retry < 50; retry++) {
        if (ToF_ReadReg(i2c, REG_FW_SYSTEM_STATUS, &status) == 0) {
            if (status & 0x01) break;
        }
        HAL_Delay(10);
    }
    if (retry >= 50) { s_error_count++; return; }

    /* I2C mode (not MIPI) */
    if (ToF_ReadReg(i2c, REG_PAD_I2C_HV_CFG, &cfg) == 0) {
        cfg &= ~(1U << 4);
        ToF_WriteReg(i2c, REG_PAD_I2C_HV_CFG, cfg);
    }

    /* Set 4x4 resolution */
    {
        uint8_t dci_cmd[8] = {0};
        dci_cmd[0] = (uint8_t)(RESOLUTION_4X4);
        ToF_WriteMulti(i2c, REG_DCI_CMD_MSB, dci_cmd, REG_DCI_CMD_LEN);
    }

    /* GPIO1 = data ready interrupt */
    ToF_WriteReg(i2c, REG_GPIO_HV_MUX, 0x01);

    /* Start continuous ranging */
    {
        uint8_t start_cmd[8] = {0};
        start_cmd[0] = 0x01;
        start_cmd[1] = 0x01;
        start_cmd[2] = (uint8_t)(TIMING_BUDGET_MS & 0xFF);
        start_cmd[3] = (uint8_t)((TIMING_BUDGET_MS >> 8) & 0xFF);
        ToF_WriteMulti(i2c, REG_DCI_CMD, start_cmd, 4);
    }

    s_sensor_ready = 1;
}

/*============================================================================
 *  ToF_Read -- read 4x4 range results
 *
 *  4 bytes per zone: [status][distLSB][distMSB][sigma]
 *  Returns 0 on success, -1 on error.
 *============================================================================*/
int ToF_Read(I2C_HandleTypeDef *i2c, uint16_t *zones_mm)
{
    uint8_t  raw[REG_RANGE_DATA_SIZE];
    uint32_t offset;
    uint16_t dist;
    uint8_t  dr;

    if (!s_sensor_ready || zones_mm == NULL) return -1;
    memset(raw, 0, sizeof(raw));

    /* Check data ready */
    if (ToF_ReadReg(i2c, REG_STATUS, &dr) == 0) {
        if (!(dr & 0x01)) return -1;
    }

    /* Read 64 bytes of range results */
    if (ToF_ReadMulti(i2c, REG_RANGE_DATA, raw, REG_RANGE_DATA_SIZE) != 0) {
        s_error_count++;
        return -1;
    }

    /* Parse 16 zones */
    for (uint32_t zone = 0; zone < 16; zone++) {
        offset = zone * 4U;
        if (raw[offset] == 0) {   /* status = 0 means valid */
            dist = (uint16_t)raw[offset + 1]
                 | (uint16_t)(raw[offset + 2] << 8);
            zones_mm[zone] = dist;
        } else {
            zones_mm[zone] = 0xFFFFU;
        }
    }

    /* Clear interrupt */
    ToF_WriteReg(i2c, REG_INTERRUPT_CLEAR, 0x01);

    s_read_count++;
    return 0;
}

/*============================================================================
 *  ToF_Stop
 *============================================================================*/
void ToF_Stop(I2C_HandleTypeDef *i2c)
{
    uint8_t stop_cmd[8] = {0};
    stop_cmd[0] = 0x01;
    ToF_WriteMulti(i2c, REG_DCI_CMD, stop_cmd, 1);
    s_sensor_ready = 0;
}

uint32_t ToF_GetReadCount(void)  { return s_read_count; }
uint32_t ToF_GetErrorCount(void) { return s_error_count; }
uint8_t  ToF_IsReady(void)       { return s_sensor_ready; }
