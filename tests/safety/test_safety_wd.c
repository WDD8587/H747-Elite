#include "../unity/unity.h"
#include <stdint.h>

static uint8_t hb = 0, cnt = 0, fs = 0;

static void monitor(void)
{
    if (hb) { cnt = 0; hb = 0; fs = 0; return; }
    if (++cnt > 5) fs = 1;
}

void test_safety_m7_alive(void)
{
    cnt = 0; fs = 0; hb = 1; monitor();
    unity_assert_equal_int("Alive->NORMAL", 0, (int)fs);
    unity_assert_equal_int("Cnt reset", 0, (int)cnt);
    for (int i = 0; i < 3; i++) monitor();
    unity_assert_equal_int("3miss still OK", 0, (int)fs);
}

void test_safety_m7_dead(void)
{
    cnt = 0; fs = 0; hb = 0;
    for (int i = 0; i < 7; i++) monitor();
    unity_assert_equal_int("7miss->FAILSAFE", 1, (int)fs);
}
