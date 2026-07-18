#include "../unity/unity.h"
#include <math.h>

static void clarke(float a, float b, float c, float *alpha, float *beta)
{
    *alpha = a;
    *beta  = (a + 2.0f * b) * 0.577350269f;
}

static void park(float alpha, float beta, float theta, float *d, float *q)
{
    float ct = cosf(theta), st = sinf(theta);
    *d =  alpha * ct + beta * st;
    *q = -alpha * st + beta * ct;
}

static int svpwm_sector(float va, float vb)
{
    float t = atan2f(vb, va);
    if (t < 0) t += 2.0f * 3.14159f;
    return (int)(t / 1.0472f + 0.001f) + 1;
}

void test_clarke_park_balanced(void)
{
    float a, b, d, q;
    clarke(1.0f, -0.5f, -0.5f, &a, &b);
    unity_assert_equal_float("Clarke alpha", 1.0f, a, 0.01f);
    unity_assert_equal_float("Clarke beta",  0.0f, b, 0.05f);
    park(a, b, 0.0f, &d, &q);
    unity_assert_equal_float("Park id@0", 1.0f, d, 0.01f);
    unity_assert_equal_float("Park iq@0", 0.0f, q, 0.01f);
}

void test_park_at_0(void)
{
    float d, q;
    park(1.0f, 0.0f, 0.0f, &d, &q);
    unity_assert_equal_float("id", 1.0f, d, 0.01f);
    park(1.0f, 0.0f, 1.5708f, &d, &q);
    unity_assert_equal_float("iq@90", -1.0f, q, 0.01f);
}

void test_svpwm_sectors(void)
{
    unity_assert_equal_int("S1", 1, svpwm_sector( 1.0f,  0.3f));
    unity_assert_equal_int("S2", 2, svpwm_sector( 0.0f,  1.0f));
    unity_assert_equal_int("S3", 3, svpwm_sector(-0.5f,  0.5f));
    unity_assert_equal_int("S4", 4, svpwm_sector(-1.0f,  0.0f));
    unity_assert_equal_int("S4b", 4, svpwm_sector(-0.3f, -0.5f));
    unity_assert_equal_int("S6", 6, svpwm_sector( 0.3f, -0.5f));
}
