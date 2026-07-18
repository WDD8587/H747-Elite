/**
 * @file    selftest.h
 * @brief   Power-on self-test API.
 */
#ifndef SELFTEST_H
#define SELFTEST_H

#include <stdint.h>

/* Test IDs */
#define POST_TEST_RAM     0
#define POST_TEST_FLASH   1
#define POST_TEST_I2C     2
#define POST_TEST_SPI     3
#define POST_TEST_CAN     4
#define POST_TEST_IMU     5

/* Status values */
#define POST_STATUS_PASS  0
#define POST_STATUS_FAIL  1
#define POST_STATUS_SKIP  2

/* Per-test result */
typedef struct {
    uint8_t     test_id;
    const char *name;
    uint8_t     status;   /* POST_STATUS_* */
    uint32_t    error;    /* Test-specific error code */
} post_test_result_t;

int                selftest_run_all(void);
post_test_result_t selftest_run_one(uint8_t test_id);
int                selftest_get_result(void);

#endif /* SELFTEST_H */
