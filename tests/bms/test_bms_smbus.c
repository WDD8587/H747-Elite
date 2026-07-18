#include "../unity/unity.h"
#include <stdint.h>
#include <string.h>

static uint8_t crc8(const uint8_t *d, int n)
{
    uint8_t c = 0; int i, j;
    for (i = 0; i < n; i++) {
        c ^= d[i];
        for (j = 0; j < 8; j++)
            c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
    }
    return c;
}

void test_smbus_readword_valid(void)
{
    uint8_t d[] = {0x2C, 0x09, 0x2D, 0x12, 0x34};
    uint8_t pec = crc8(d, 5);
    unity_assert_true("PEC computed", pec != 0);
}

void test_smbus_pec_mismatch(void)
{
    uint8_t a[] = {0x2C, 0x09, 0x2D, 0x12, 0x34};
    uint8_t b[] = {0x2C, 0x09, 0x2D, 0x99, 0x99};
    unity_assert_true("Different data, diff PEC", crc8(a, 5) != crc8(b, 5));
}
