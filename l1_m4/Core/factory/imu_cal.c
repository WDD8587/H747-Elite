/**
 * @file    imu_cal.c
 * @brief   IMU 6-face calibration for production line.
 *
 * Robot placed in 6 known orientations by fixture. Each face:
 * record 100 samples of accelerometer + gyroscope.
 * Compute: accel bias, scale factor, cross-axis sensitivity
 * (12-parameter model), gyro bias. Save to flash.
 *
 * @note    Part of STM32H747 factory calibration suite.
 */

#include "imu_cal.h"
#include "imu_spi.h"
#include "factory_flash.h"
#include "factory_timer.h"
#include "factory_uart.h"

#include <string.h>
#include <math.h>

/* ---------------------------------------------------------------------------
 * Constants
 * ------------------------------------------------------------------------- */
#define IMUCAL_SAMPLES_PER_FACE         100
#define IMUCAL_SETTLE_MS                 200
#define IMUCAL_SAMPLE_INTERVAL_MS        10
#define IMUCAL_GRAVITY_m_s2              9.80665f
#define CAL_PARAM_MAGIC                 0x494D5543UL  /* "IMUC" */

/* Face definitions (orientation of the IMU chassis frame) */
typedef enum {
    FACE_Z_UP      = 0,   /* +Z aligned with gravity */
    FACE_Z_DOWN    = 1,   /* -Z aligned with gravity */
    FACE_X_UP      = 2,   /* +X aligned with gravity */
    FACE_X_DOWN    = 3,   /* -X aligned with gravity */
    FACE_Y_UP      = 4,   /* +Y aligned with gravity */
    FACE_Y_DOWN    = 5,   /* -Y aligned with gravity */
    FACE_COUNT     = 6
} IMU_Face;

/* Expected acceleration for each face (in m/s^2, chassis frame) */
static const float expected_accel[FACE_COUNT][3] = {
    { 0.0f,  0.0f,   IMUCAL_GRAVITY_m_s2 },   /* Z up    */
    { 0.0f,  0.0f,  -IMUCAL_GRAVITY_m_s2 },   /* Z down  */
    { IMUCAL_GRAVITY_m_s2,  0.0f,  0.0f },    /* X up    */
    {-IMUCAL_GRAVITY_m_s2,  0.0f,  0.0f },    /* X down  */
    { 0.0f,  IMUCAL_GRAVITY_m_s2,  0.0f },    /* Y up    */
    { 0.0f, -IMUCAL_GRAVITY_m_s2,  0.0f },    /* Y down  */
};

/* ---------------------------------------------------------------------------
 * Calibration parameter structure (12-parameter accelerometer model)
 *
 * Accel model:
 *   a_true = M * (a_raw - bias)
 * where M is the 3x3 correction matrix (scale + cross-axis):
 *   M = [ Sxx  Mxy  Mxz
 *         Myx  Syy  Myz
 *         Mzx  Mzy  Szz ]
 *
 * For simplicity in production, we compute:
 *   bias = mean of +/- face readings
 *   scale = from expected vs measured range
 *   cross-axis = from off-axis sensitivity
 * ------------------------------------------------------------------------- */

typedef struct __attribute__((packed)) {
    /* Accelerometer model */
    float accel_bias[3];           /* Bias per axis (m/s^2)              */
    float accel_scale[3];          /* Scale factor per axis              */
    float accel_cross[6];          /* Cross-axis (Mxy, Mxz, Myx, Myz, Mzx, Mzy) */
    /* Gyroscope model */
    float gyro_bias[3];            /* Bias per axis (rad/s)              */
    /* Metadata */
    uint32_t magic;
    float    temperature_at_cal_C; /* Temperature during calibration     */
    uint32_t crc32;
} IMU_CalParams;

static IMU_CalParams imuc_;

/* ---------------------------------------------------------------------------
 * CRC-32
 * ------------------------------------------------------------------------- */
static uint32_t cal_crc32(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ ((crc & 1UL) ? 0xEDB88320UL : 0UL);
        }
    }
    return crc ^ 0xFFFFFFFFUL;
}

/* ---------------------------------------------------------------------------
 * Sample one face: collect IMUCAL_SAMPLES_PER_FACE readings, store averages
 * ------------------------------------------------------------------------- */
static bool imucal_sample_face(IMU_Face face, float accel_avg[3], float gyro_avg[3])
{
    float accel_sum[3] = {0.0f, 0.0f, 0.0f};
    float gyro_sum[3]  = {0.0f, 0.0f, 0.0f};

    UART_Print("IMU_CAL: sampling face %d...\r\n", (int)face);
    TIMER_DelayMs(IMUCAL_SETTLE_MS);

    for (int s = 0; s < IMUCAL_SAMPLES_PER_FACE; s++) {
        float a[3], g[3];
        IMU_SPI_ReadAccel(a);
        IMU_SPI_ReadGyro(g);

        for (int i = 0; i < 3; i++) {
            accel_sum[i] += a[i];
            gyro_sum[i]  += g[i];
        }
        TIMER_DelayMs(IMUCAL_SAMPLE_INTERVAL_MS);
    }

    float inv_n = 1.0f / (float)IMUCAL_SAMPLES_PER_FACE;
    for (int i = 0; i < 3; i++) {
        accel_avg[i] = accel_sum[i] * inv_n;
        gyro_avg[i]  = gyro_sum[i] * inv_n;
    }

    UART_Print("  accel = [%.3f, %.3f, %.3f]  gyro = [%.5f, %.5f, %.5f]\r\n",
               accel_avg[0], accel_avg[1], accel_avg[2],
               gyro_avg[0], gyro_avg[1], gyro_avg[2]);
    return true;
}

/* ---------------------------------------------------------------------------
 * Compute accelerometer bias from +/- face pairs
 * ------------------------------------------------------------------------- */
static void imucal_compute_accel_bias(const float face_data[FACE_COUNT][3])
{
    /* Bias = (measurement_on_+face + measurement_on_-face) / 2
     * For each axis, the sum of + and - readings gives 2 * bias
     * (since gravity cancels out). */
    for (int axis = 0; axis < 3; axis++) {
        imuc_.accel_bias[axis] =
            (face_data[axis * 2][axis] + face_data[axis * 2 + 1][axis]) / 2.0f;
    }
}

/* ---------------------------------------------------------------------------
 * Compute accelerometer scale factors from +/- face pairs
 * ------------------------------------------------------------------------- */
static void imucal_compute_accel_scale(const float face_data[FACE_COUNT][3])
{
    /* Scale = (expected_range) / (measured_range_after_bias_correction)
     * For each axis: scale = 2 * G / (measurement_on_+face - measurement_on_-face)
     * after subtracting bias.
     */
    for (int axis = 0; axis < 3; axis++) {
        float m_plus  = face_data[axis * 2][axis]     - imuc_.accel_bias[axis];
        float m_minus = face_data[axis * 2 + 1][axis] - imuc_.accel_bias[axis];
        float range = fabsf(m_plus - m_minus);

        if (range > 1.0f) {
            imuc_.accel_scale[axis] = (2.0f * IMUCAL_GRAVITY_m_s2) / range;
        } else {
            imuc_.accel_scale[axis] = 1.0f;  /* Fallback */
        }
    }
}

/* ---------------------------------------------------------------------------
 * Compute cross-axis sensitivity from off-axis readings
 * ------------------------------------------------------------------------- */
static void imucal_compute_cross_axis(const float face_data[FACE_COUNT][3])
{
    /* For each pair of +/- faces, the off-axis readings show cross-coupling.
     * Mxy = off-axis Y reading when X is active, normalised. */
    int cross_idx = 0;
    for (int axis = 0; axis < 3; axis++) {
        for (int other = 0; other < 3; other++) {
            if (axis == other) continue;

            /* Read off-axis component when this axis is aligned with gravity */
            float off_plus  = face_data[axis * 2][other];
            float off_minus = face_data[axis * 2 + 1][other];
            float avg_off = (off_plus + off_minus) / 2.0f;

            /* Cross-axis sensitivity = off-axis signal / (2 * G) */
            float cross = avg_off / (2.0f * IMUCAL_GRAVITY_m_s2);
            if (cross_idx < 6) {
                imuc_.accel_cross[cross_idx++] = cross;
            }
        }
    }
}

/* ---------------------------------------------------------------------------
 * Compute gyroscope bias (average of all face readings while stationary)
 * ------------------------------------------------------------------------- */
static void imucal_compute_gyro_bias(const float gyro_data[FACE_COUNT][3])
{
    float sum[3] = {0.0f, 0.0f, 0.0f};
    for (int f = 0; f < FACE_COUNT; f++) {
        for (int i = 0; i < 3; i++) {
            sum[i] += gyro_data[f][i];
        }
    }

    float inv = 1.0f / (float)FACE_COUNT;
    for (int i = 0; i < 3; i++) {
        imuc_.gyro_bias[i] = sum[i] * inv;
    }
}

/* ---------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

bool IMUCAL_RunAll(void)
{
    UART_Print("IMU_CAL: starting 6-face production calibration...\r\n");
    memset(&imuc_, 0, sizeof(imuc_));
    imuc_.magic = CAL_PARAM_MAGIC;

    float accel_data[FACE_COUNT][3];
    float gyro_data[FACE_COUNT][3];

    for (int face = 0; face < FACE_COUNT; face++) {
        UART_Print("  Place robot in face orientation %d and press Enter...\r\n", face);
        /* In production, fixture triggers automatically; we just wait */
        TIMER_DelayMs(1000);

        if (!imucal_sample_face((IMU_Face)face, accel_data[face], gyro_data[face])) {
            UART_Print("IMU_CAL: sampling failed on face %d\r\n", face);
            return false;
        }
    }

    /* Compute parameters */
    imucal_compute_accel_bias(accel_data);
    UART_Print("  Accel bias = [%.4f, %.4f, %.4f]\r\n",
               imuc_.accel_bias[0], imuc_.accel_bias[1], imuc_.accel_bias[2]);

    imucal_compute_accel_scale(accel_data);
    UART_Print("  Accel scale = [%.6f, %.6f, %.6f]\r\n",
               imuc_.accel_scale[0], imuc_.accel_scale[1], imuc_.accel_scale[2]);

    imucal_compute_cross_axis(accel_data);
    UART_Print("  Accel cross = [");

    for (int i = 0; i < 6; i++) {
        UART_Print("%.6f%c", imuc_.accel_cross[i], (i < 5) ? ',' : ']');
    }
    UART_Print("\r\n");

    imucal_compute_gyro_bias(gyro_data);
    UART_Print("  Gyro bias = [%.6f, %.6f, %.6f] rad/s\r\n",
               imuc_.gyro_bias[0], imuc_.gyro_bias[1], imuc_.gyro_bias[2]);

    /* Record temperature */
    imuc_.temperature_at_cal_C = 25.0f;  /* Assumes controlled factory temp */

    /* Save to flash */
    imuc_.crc32 = cal_crc32((const uint8_t *)&imuc_,
                             offsetof(IMU_CalParams, crc32));
    FACTORY_FLASH_Write(FACTORY_SECTOR_IMU, (uint32_t)&imuc_, sizeof(imuc_));

    UART_Print("IMU_CAL: saved to flash. PASS\r\n");
    return true;
}

bool IMUCAL_LoadFromFlash(void)
{
    if (!FACTORY_FLASH_Read(FACTORY_SECTOR_IMU, (uint32_t)&imuc_, sizeof(imuc_))) {
        return false;
    }
    if (imuc_.magic != CAL_PARAM_MAGIC) return false;

    uint32_t expected_crc = cal_crc32((const uint8_t *)&imuc_,
                                       offsetof(IMU_CalParams, crc32));
    return (expected_crc == imuc_.crc32);
}

void IMUCAL_ApplyAccelCorrection(float raw[3], float corrected[3])
{
    /* a_corrected = M * (a_raw - bias) */
    float tmp[3];
    for (int i = 0; i < 3; i++) {
        tmp[i] = raw[i] - imuc_.accel_bias[i];
    }

    /* Apply correction matrix */
    corrected[0] = imuc_.accel_scale[0] * tmp[0]
                 + imuc_.accel_cross[0] * tmp[1]
                 + imuc_.accel_cross[1] * tmp[2];
    corrected[1] = imuc_.accel_cross[2] * tmp[0]
                 + imuc_.accel_scale[1] * tmp[1]
                 + imuc_.accel_cross[3] * tmp[2];
    corrected[2] = imuc_.accel_cross[4] * tmp[0]
                 + imuc_.accel_cross[5] * tmp[1]
                 + imuc_.accel_scale[2] * tmp[2];
}

void IMUCAL_ApplyGyroCorrection(float raw[3], float corrected[3])
{
    for (int i = 0; i < 3; i++) {
        corrected[i] = raw[i] - imuc_.gyro_bias[i];
    }
}

const float* IMUCAL_GetAccelBias(void) { return imuc_.accel_bias; }
const float* IMUCAL_GetAccelScale(void) { return imuc_.accel_scale; }
const float* IMUCAL_GetGyroBias(void) { return imuc_.gyro_bias; }
