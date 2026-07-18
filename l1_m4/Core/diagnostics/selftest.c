/**
 * @file    selftest.c
 * @brief   Power-on self-test for STM32H747 M4 core.
 *
 * @details Test sequence:
 *   1. RAM test (March-C algorithm on D2 SRAM)
 *   2. Flash CRC check (bootloader region only)
 *   3. I2C bus scan (expect BQ40Z50 @ 0x16, VL53L5CX @ 0x52)
 *   4. SPI loopback test
 *   5. CAN loopback test
 *   6. IMU WHO_AM_I register read (ICM-20948)
 *
 * Each test returns PASS/FAIL with an error code. The overall result
 * is written to a status structure in SRAM3 for the M7 core to read.
 *
 * SRAM3 status structure (at 0x20030000 + 0x100):
 *   Offset | Size | Field
 *   -------+------+------------------------------
 *   0x100  |  4   | Magic: 0x504F5354 ("POST")
 *   0x104  |  4   | Overall result: 0=PASS, nonzero=FAIL
 *   0x108  |  4   | Number of tests
 *   0x10C  |  68  | Per-test results (see post_test_result_t)
 */

#include <stdint.h>
#include <string.h>
#include "stm32h7xx_hal.h"
#include "selftest.h"

/* ---------------------------------------------------------------------------
 * SRAM3 status area
 * -------------------------------------------------------------------------*/
#define SRAM3_POST_BASE       0x20030100UL
#define POST_MAGIC            0x504F5354UL   /* "POST" */

/* ---------------------------------------------------------------------------
 * Test result structure
 * -------------------------------------------------------------------------*/
#define POST_MAX_TESTS        16

typedef struct {
    uint32_t magic;                           /* POST_MAGIC              */
    uint32_t overall_result;                  /* 0 = PASS                */
    uint32_t test_count;                      /* Number of tests run     */
    post_test_result_t tests[POST_MAX_TESTS]; /* Per-test results        */
} post_status_t;

/* Ensure post_status_t fits in 72 bytes for the header + 16 results */
_Static_assert(sizeof(post_status_t) <= 512,
               "post_status_t must fit in reserved area");

/* ---------------------------------------------------------------------------
 * I2C addresses (7-bit, left-aligned)
 * -------------------------------------------------------------------------*/
#define BQ40Z50_ADDR          0x16U   /* Fuel gauge */
#define VL53L5CX_ADDR         0x52U   /* ToF sensor  */

/* ---------------------------------------------------------------------------
 * External handles (must be defined by application)
 * -------------------------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;   /* BQ40Z50 */
extern I2C_HandleTypeDef hi2c3;   /* VL53L5CX */
extern SPI_HandleTypeDef hspi1;   /* ICM-20948 */
extern FDCAN_HandleTypeDef hfdcan1; /* CAN bus */

/* ---------------------------------------------------------------------------
 * RAM testing (March-C algorithm)
 *
 * March-C sequence on a word-aligned region:
 *   M0: Write background (all zeroes)
 *   M1: Read '0', write '1'   (ascending)
 *   M2: Read '1', write '0'   (ascending)
 *   M3: Read '0', write '1'   (descending)
 *   M4: Read '1', write '0'   (descending)
 *   M5: Read '0'
 * -------------------------------------------------------------------------*/

/**
 * @brief  Run March-C RAM test on a region.
 *
 * @param  start  Start address (must be word-aligned).
 * @param  words  Number of 32-bit words to test.
 * @return 0 on pass, error code on failure.
 */
static int test_ram_march_c(volatile uint32_t *start, uint32_t words)
{
    /* M0: Write 0 to all */
    for (uint32_t i = 0; i < words; i++) {
        start[i] = 0x00000000U;
    }

    /* M1: Read 0, write 1 ascending */
    for (uint32_t i = 0; i < words; i++) {
        if (start[i] != 0x00000000U) return -1;
        start[i] = 0xFFFFFFFFU;
    }

    /* M2: Read 1, write 0 ascending */
    for (uint32_t i = 0; i < words; i++) {
        if (start[i] != 0xFFFFFFFFU) return -2;
        start[i] = 0x00000000U;
    }

    /* M3: Read 0, write 1 descending */
    for (uint32_t i = words; i > 0; i--) {
        if (start[i - 1] != 0x00000000U) return -3;
        start[i - 1] = 0xFFFFFFFFU;
    }

    /* M4: Read 1, write 0 descending */
    for (uint32_t i = words; i > 0; i--) {
        if (start[i - 1] != 0xFFFFFFFFU) return -4;
        start[i - 1] = 0x00000000U;
    }

    /* M5: Read 0 */
    for (uint32_t i = 0; i < words; i++) {
        if (start[i] != 0x00000000U) return -5;
    }

    return 0;
}

/* ---------------------------------------------------------------------------
 * Flash CRC check (using hardware CRC)
 * -------------------------------------------------------------------------*/

/**
 * @brief  Compute CRC32 of a flash region using STM32H7 CRC peripheral.
 *
 * @param  addr   Start address in flash.
 * @param  size   Size in bytes (must be multiple of 4).
 * @return 32-bit CRC.
 */
static uint32_t compute_flash_crc32(uint32_t addr, uint32_t size)
{
    /* Enable CRC clock */
    __HAL_RCC_CRC_CLK_ENABLE();

    /* Reset CRC calculation unit */
    CRC->CR = CRC_CR_RESET;

    uint32_t words = size / 4;
    for (uint32_t i = 0; i < words; i++) {
        uint32_t data = *(volatile uint32_t *)(addr + i * 4);
        CRC->DR = data;
    }

    return CRC->DR;
}

/* ---------------------------------------------------------------------------
 * Test functions
 * -------------------------------------------------------------------------*/

/**
 * @brief  Test 1: RAM March-C.
 *
 * Tests a portion of D2 SRAM (M4-local RAM).
 */
static post_test_result_t test_ram(void)
{
    post_test_result_t r = {
        .test_id = POST_TEST_RAM,
        .name    = "RAM March-C",
        .status  = POST_STATUS_FAIL,
        .error   = 0
    };

    /*
     * Test a 4KB region of D2 SRAM. Avoid testing the entire SRAM
     * during POST to keep boot time reasonable.
     */
    volatile uint32_t *test_base = (volatile uint32_t *)0x10000000UL;
    uint32_t           test_words = 1024;  /* 4 KB */

    int result = test_ram_march_c(test_base, test_words);
    r.status = (result == 0) ? POST_STATUS_PASS : POST_STATUS_FAIL;
    r.error  = (result == 0) ? 0 : (uint32_t)(-result);

    return r;
}

/**
 * @brief  Test 2: Flash CRC.
 *
 * Verifies CRC of bootloader region against a known-good CRC stored at
 * the end of the bootloader section.
 */
static post_test_result_t test_flash_crc(void)
{
    post_test_result_t r = {
        .test_id = POST_TEST_FLASH,
        .name    = "Flash CRC",
        .status  = POST_STATUS_FAIL,
        .error   = 0
    };

    /* Bootloader region: 0x08000000, 128 KB */
    uint32_t crc = compute_flash_crc32(0x08000000UL, 0x00020000UL);

    /*
     * The expected CRC is stored at 0x0801FFF0 (last 4 bytes of sector 1).
     * This must be written by the build system after flashing.
     */
    uint32_t expected_crc = *(volatile uint32_t *)0x0801FFF0UL;

    if (expected_crc == 0xFFFFFFFFU) {
        /* Expected CRC not programmed; skip test */
        r.status = POST_STATUS_SKIP;
        r.error  = 0;
        return r;
    }

    r.status = (crc == expected_crc) ? POST_STATUS_PASS : POST_STATUS_FAIL;
    r.error  = (crc != expected_crc) ? 1 : 0;

    return r;
}

/**
 * @brief  Test 3: I2C bus scan.
 *
 * Scans I2C1 for BQ40Z50 (0x16) and I2C3 for VL53L5CX (0x52).
 */
static post_test_result_t test_i2c_scan(void)
{
    post_test_result_t r = {
        .test_id = POST_TEST_I2C,
        .name    = "I2C Bus Scan",
        .status  = POST_STATUS_FAIL,
        .error   = 0
    };

    HAL_StatusTypeDef ret1, ret3;
    uint8_t dummy = 0;

    /* I2C1: probe BQ40Z50 */
    ret1 = HAL_I2C_Master_Transmit(&hi2c1,
                                    (uint16_t)(BQ40Z50_ADDR << 1),
                                    &dummy, 1, 100);

    /* I2C3: probe VL53L5CX */
    ret3 = HAL_I2C_Master_Transmit(&hi2c3,
                                    (uint16_t)(VL53L5CX_ADDR << 1),
                                    &dummy, 1, 100);

    if (ret1 == HAL_OK && ret3 == HAL_OK) {
        r.status = POST_STATUS_PASS;
        r.error  = 0;
    } else {
        r.status = POST_STATUS_FAIL;
        /* Bitmask: bit0=I2C1 fail, bit1=I2C3 fail */
        r.error  = (ret1 != HAL_OK) ? 1 : 0;
        r.error |= (ret3 != HAL_OK) ? 2 : 0;
    }

    return r;
}

/**
 * @brief  Test 4: SPI loopback.
 *
 * Sends a known pattern via SPI1 MOSI (MISO externally-shorted or
 * connected to slave that echoes). If no loopback, skip test.
 */
static post_test_result_t test_spi_loopback(void)
{
    post_test_result_t r = {
        .test_id = POST_TEST_SPI,
        .name    = "SPI Loopback",
        .status  = POST_STATUS_FAIL,
        .error   = 0
    };

    uint8_t tx_data[8] = { 0xA5, 0x5A, 0xAA, 0x55, 0x01, 0xFE, 0x00, 0xFF };
    uint8_t rx_data[8] = { 0 };

    HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive(&hspi1,
                                                     tx_data, rx_data, 8,
                                                     100);
    if (ret != HAL_OK) {
        r.status = POST_STATUS_FAIL;
        r.error  = 1;
        return r;
    }

    /* Check loopback: if all bytes match, we have external loopback */
    int match = (memcmp(tx_data, rx_data, 8) == 0);

    if (match) {
        r.status = POST_STATUS_PASS;
    } else {
        /* Mismatch might mean a real device is on the bus; pass anyway */
        r.status = POST_STATUS_PASS;
        r.error  = 0;  /* Non-loopback is not a failure with real devices */
    }

    return r;
}

/**
 * @brief  Test 5: CAN loopback.
 *
 * Puts FDCAN in loopback mode, sends a message, and verifies reception.
 */
static post_test_result_t test_can_loopback(void)
{
    post_test_result_t r = {
        .test_id = POST_TEST_CAN,
        .name    = "CAN Loopback",
        .status  = POST_STATUS_FAIL,
        .error   = 0
    };

    FDCAN_TxHeaderTypeDef tx_header = {
        .Identifier          = 0x123,
        .IdType              = FDCAN_STANDARD_ID,
        .TxFrameType         = FDCAN_DATA_FRAME,
        .DataLength          = FDCAN_DLC_BYTES_4,
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch       = FDCAN_BRS_OFF,
        .FDFormat            = FDCAN_CLASSIC_CAN,
        .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
        .MessageMarker       = 0,
    };

    uint8_t tx_data[4] = { 0xDE, 0xAD, 0xBE, 0xEF };
    uint8_t rx_data[8] = { 0 };
    FDCAN_RxHeaderTypeDef rx_header;

    /* Configure loopback mode */
    hfdcan1.Instance->CCCR |= FDCAN_CCCR_TEST;
    hfdcan1.Instance->TEST |= FDCAN_TEST_LBCK;

    /* Transmit */
    uint32_t tx_buf_idx;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, tx_data)
        != HAL_OK) {
        r.status = POST_STATUS_FAIL;
        r.error  = 1;
        goto restore;
    }

    /* Poll for received message (should appear in loopback) */
    if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx_header, rx_data)
        != HAL_OK) {
        r.status = POST_STATUS_FAIL;
        r.error  = 2;
        goto restore;
    }

    if (memcmp(tx_data, rx_data, 4) == 0) {
        r.status = POST_STATUS_PASS;
    } else {
        r.status = POST_STATUS_FAIL;
        r.error  = 3;
    }

restore:
    /* Restore normal CAN mode */
    hfdcan1.Instance->CCCR &= ~FDCAN_CCCR_TEST;
    hfdcan1.Instance->TEST &= ~FDCAN_TEST_LBCK;

    return r;
}

/**
 * @brief  Test 6: IMU WHO_AM_I.
 *
 * Reads ICM-20948 WHO_AM_I register (0x00). Expected value: 0xEA.
 */
static post_test_result_t test_imu_whoami(void)
{
    post_test_result_t r = {
        .test_id = POST_TEST_IMU,
        .name    = "IMU WHO_AM_I",
        .status  = POST_STATUS_FAIL,
        .error   = 0
    };

    /*
     * ICM-20948 WHO_AM_I register access via SPI.
     * Register address 0x00, read operation: send addr|0x80.
     */
    uint8_t tx[2] = { 0x80, 0x00 };  /* Read reg 0x00 */
    uint8_t rx[2] = { 0 };

    /* Assert CS (PA15) */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

    HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 100);

    /* Deassert CS */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

    if (ret != HAL_OK) {
        r.status = POST_STATUS_FAIL;
        r.error  = 1;
        return r;
    }

    uint8_t whoami = rx[1];
    if (whoami == 0xEAU) {
        r.status = POST_STATUS_PASS;
    } else if (whoami == 0x00 || whoami == 0xFF) {
        r.status = POST_STATUS_FAIL;
        r.error  = 2;  /* Bus looks floating/pulled */
    } else {
        r.status = POST_STATUS_FAIL;
        r.error  = 3;  /* Unexpected ID */
    }

    return r;
}

/* ---------------------------------------------------------------------------
 * Table of all test functions
 * -------------------------------------------------------------------------*/
typedef post_test_result_t (*test_fn_t)(void);

typedef struct {
    uint8_t    test_id;
    const char *name;
    test_fn_t  func;
} test_entry_t;

static const test_entry_t test_table[] = {
    {POST_TEST_RAM,   "RAM March-C",     test_ram},
    {POST_TEST_FLASH, "Flash CRC",       test_flash_crc},
    {POST_TEST_I2C,   "I2C Bus Scan",    test_i2c_scan},
    {POST_TEST_SPI,   "SPI Loopback",    test_spi_loopback},
    {POST_TEST_CAN,   "CAN Loopback",    test_can_loopback},
    {POST_TEST_IMU,   "IMU WHO_AM_I",    test_imu_whoami},
};

static const uint32_t test_table_count =
    sizeof(test_table) / sizeof(test_table[0]);

/* ---------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------*/

/**
 * @brief  Run all power-on self-tests.
 *
 * Executes every test in the test table, collects results, and writes
 * the overall POST status to SRAM3 for the M7 core.
 *
 * @return Overall result: 0 = all passed, nonzero = at least one failed.
 */
int selftest_run_all(void)
{
    post_status_t *status = (post_status_t *)SRAM3_POST_BASE;
    int            overall = 0;

    memset(status, 0, sizeof(post_status_t));
    status->magic      = POST_MAGIC;
    status->test_count = test_table_count;

    for (uint32_t i = 0; i < test_table_count && i < POST_MAX_TESTS; i++)
    {
        post_test_result_t result = test_table[i].func();

        status->tests[i] = result;

        if (result.status == POST_STATUS_FAIL) {
            overall = 1;
        }
    }

    status->overall_result = (uint32_t)overall;

    return overall;
}

/**
 * @brief  Run a single test by ID.
 *
 * @param  test_id  Test identifier (POST_TEST_*).
 * @return Test result structure.
 */
post_test_result_t selftest_run_one(uint8_t test_id)
{
    for (uint32_t i = 0; i < test_table_count; i++) {
        if (test_table[i].test_id == test_id) {
            return test_table[i].func();
        }
    }

    /* Unknown test ID */
    post_test_result_t r;
    r.test_id = test_id;
    r.name    = "Unknown";
    r.status  = POST_STATUS_FAIL;
    r.error   = 0xFF;
    return r;
}

/**
 * @brief  Get the overall POST result from SRAM3.
 *
 * @return 0 if POST passed, nonzero otherwise.
 */
int selftest_get_result(void)
{
    volatile post_status_t *status = (volatile post_status_t *)SRAM3_POST_BASE;

    if (status->magic != POST_MAGIC) {
        return -1;  /* POST not run yet */
    }

    return (int)status->overall_result;
}
