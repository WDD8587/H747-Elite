#include "../unity/unity.h"
#include <string.h>
#include <stdint.h>

static int diff(const uint8_t *o, int ol, const uint8_t *n, int nl, uint8_t *d, int *dl)
{
    if (ol == nl && memcmp(o, n, ol) == 0) { *dl = 0; return 0; }
    memcpy(d, n, nl); *dl = nl; return 0;
}

static int patch(const uint8_t *o, int ol, const uint8_t *d, int dl, uint8_t *n, int *nl)
{
    if (dl == 0) { memcpy(n, o, ol); *nl = ol; return 0; }
    memcpy(n, d, dl); *nl = dl; return 0;
}

void test_diff_identical_empty(void)
{
    const char *a = "firmware_v1.0";
    uint8_t d[256]; int dl;
    diff((const uint8_t*)a, (int)strlen(a), (const uint8_t*)a, (int)strlen(a), d, &dl);
    unity_assert_equal_int("Identical->empty", 0, dl);
}

void test_diff_roundtrip(void)
{
    const char *o = "firmware_v1.0", *n = "firmware_v2.0";
    uint8_t d[256], r[256]; int dl, rl;
    diff((const uint8_t*)o, (int)strlen(o), (const uint8_t*)n, (int)strlen(n), d, &dl);
    unity_assert_true("Delta has content", dl > 0);
    patch((const uint8_t*)o, (int)strlen(o), d, dl, r, &rl);
    r[rl] = 0;
    unity_assert_true("Roundtrip matches", strcmp(n, (char*)r) == 0);
}
