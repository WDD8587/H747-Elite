#include "../unity/unity.h"
#include <stdint.h>

typedef enum { TRICKLE, CC_0_5C, CC_1C, CV_16_8V, DONE, FAULT } Chg_t;

static void fsm(Chg_t *s, uint16_t mv, float *i)
{
    switch (*s) {
    case TRICKLE:  *i = 0.1f; if (mv > 12000) *s = CC_0_5C;  break;
    case CC_0_5C:  *i = 0.5f; if (mv > 14000) *s = CC_1C;    break;
    case CC_1C:    *i = 1.0f; if (mv > 16400) *s = CV_16_8V;  break;
    case CV_16_8V: *i = 0.2f; if (*i < 0.05f) *s = DONE;      break;
    case DONE:     *i = 0.0f; break;
    case FAULT:    *i = 0.0f; break;
    }
}

void test_charge_trickle_to_cc(void)
{
    Chg_t s = TRICKLE; float i;
    fsm(&s, 10000, &i);
    unity_assert_true("Still TRICKLE", s == TRICKLE);
    fsm(&s, 12500, &i); fsm(&s, 12500, &i);
    unity_assert_true("Switch CC", s == CC_0_5C);
    unity_assert_equal_float("0.5A", 0.5f, i, 0.01f);
}

void test_charge_fault_ovp(void)
{
    Chg_t s = FAULT; float i;
    fsm(&s, 17000, &i);
    unity_assert_equal_float("Fault zero", 0.0f, i, 0.01f);
}
