#include "stm32h7xx_hal.h"
#include "ipc_proto.h"
#include <math.h>
#include <string.h>

typedef struct {
    float q0, q1, q2, q3;
    float gyr_bias[3];
    float beta;
} Madgwick_t;

static Madgwick_t gMadg = {1, 0, 0, 0, {0, 0, 0}, 0.1f};
static l1_odom_t *gOdom = (l1_odom_t *)SRAM3_BASE;

void IMU_ReadAccelGyro(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
{
    *ax = 0; *ay = 0; *az = 9.81f;
    *gx = 0; *gy = 0; *gz = 0;
}

void MadgwickUpdate(float ax, float ay, float az, float gx, float gy, float gz, float dt)
{
    float q0 = gMadg.q0, q1 = gMadg.q1, q2 = gMadg.q2, q3 = gMadg.q3;
    float recip = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
    ax *= recip; ay *= recip; az *= recip;

    float s0, s1, s2, s3;
    float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
    float _4q0 = 4.0f * q0, _4q1 = 4.0f * q1, _4q2 = 4.0f * q2;
    float _8q1 = 8.0f * q1, _8q2 = 8.0f * q2;

    s0 = _4q0 * q2 * q2 + _2q2 * ax + _4q0 * q1 * q1 - _2q1 * ay;
    s1 = _4q1 * q3 * q3 - _2q3 * ax + 4.0f * q0 * q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1 * q1 + _8q1 * q2 * q2 + _4q1 * az;
    s2 = 4.0f * q0 * q0 * q2 + _2q0 * ax + _4q2 * q3 * q3 - _2q3 * ay - _4q2 + _8q2 * q1 * q1 + _8q2 * q2 * q2 + _4q2 * az;
    s3 = 4.0f * q1 * q1 * q3 - _2q1 * ax + 4.0f * q2 * q2 * q3 - _2q2 * ay;

    recip = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recip; s1 *= recip; s2 *= recip; s3 *= recip;

    float beta = gMadg.beta;
    q0 -= beta * s0; q1 -= beta * s1; q2 -= beta * s2; q3 -= beta * s3;

    float hx = (gx - gMadg.gyr_bias[0]) * 0.5f * dt;
    float hy = (gy - gMadg.gyr_bias[1]) * 0.5f * dt;
    float hz = (gz - gMadg.gyr_bias[2]) * 0.5f * dt;
    float qa = q0, qb = q1, qc = q2;
    q0 += -qb * hx - qc * hy - q3 * hz;
    q1 += qa * hx + qc * hz - q3 * hy;
    q2 += qa * hy - qb * hz + q3 * hx;
    q3 += qa * hz + qb * hy - qc * hx;

    recip = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    gMadg.q0 = q0 * recip; gMadg.q1 = q1 * recip;
    gMadg.q2 = q2 * recip; gMadg.q3 = q3 * recip;
}

void IMU_Process(void)
{
    float ax, ay, az, gx, gy, gz;
    IMU_ReadAccelGyro(&ax, &ay, &az, &gx, &gy, &gz);
    MadgwickUpdate(ax, ay, az, gx, gy, gz, 0.001f);

    gOdom->imu_yaw = atan2f(2.0f * (gMadg.q0 * gMadg.q3 + gMadg.q1 * gMadg.q2),
                           1.0f - 2.0f * (gMadg.q2 * gMadg.q2 + gMadg.q3 * gMadg.q3)) * 57.2958f;
    gOdom->imu_gyro_z = gz - gMadg.gyr_bias[2];
}
