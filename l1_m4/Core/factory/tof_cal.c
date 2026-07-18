/**
 * @file    tof_cal.c
 * @brief   Time-of-Flight sensor calibration for production line.
 *
 * Robot placed in calibration box with known target distances:
 *   10 cm, 30 cm, 50 cm, 100 cm, 200 cm.
 * For each distance: 100 samples per zone.
 * Compute per-zone offset and crosstalk compensation.
 * Save calibration to flash.
 *
 * @note    Part of STM32H747 factory calibration suite.
 */

#include "tof_cal.h"
#include "tof_i2c.h"
#include "factory_flash.h"
#include "factory_timer.h"
#include "factory_uart.h"

#include <string.h>
#include <math.h>

/* ---------------------------------------------------------------------------
 * Constants
 * ------------------------------------------------------------------------- */
#define TOFCAL_TARGET_DISTANCES        5
#define TOFCAL_SAMPLES_PER_DISTANCE   100
#define TOFCAL_SETTLE_MS              100
#define TOFCAL_ZONES                    4   /* Multi-zone ToF sensor        */
#define CAL_PARAM_MAGIC               0x544F4631UL  /* "TOF1" */

/* Known target distances (mm) */
static const float target_distances_mm[TOFCAL_TARGET_DISTANCES] = {
    100.0f,    /* 10 cm */
    300.0f,    /* 30 cm */
    500.0f,    /* 50 cm */
    1000.0f,   /* 100 cm */
    2000.0f    /* 200 cm */
};

/* ---------------------------------------------------------------------------
 * Calibration parameter structure
 * ------------------------------------------------------------------------- */
typedef struct __attribute__((packed)) {
    /* Per-zone offset (mm) for each of the TOFCAL_TARGET_DISTANCES distances */
    float zone_offset[TOFCAL_ZONES][TOFCAL_TARGET_DISTANCES];
    /* Crosstalk compensation per zone */
    float crosstalk_comp[TOFCAL_ZONES];
    /* Polynomial fit coefficients for each zone (2nd order: a*x^2 + b*x + c) */
    float zone_poly_a[TOFCAL_ZONES];
    float zone_poly_b[TOFCAL_ZONES];
    float zone_poly_c[TOFCAL_ZONES];
    /* Metadata */
    uint32_t magic;
    uint32_t crc32;
} ToF_CalParams;

static ToF_CalParams tofc_;

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
 * Sample one distance target for all zones
 * ------------------------------------------------------------------------- */
static bool tofcal_sample_distance(int dist_idx, float zone_avg[TOFCAL_ZONES])
{
    float sum[TOFCAL_ZONES] = {0.0f};

    UART_Print("TOF_CAL: target %d = %.0f mm, sampling %d readings...\r\n",
               dist_idx, target_distances_mm[dist_idx], TOFCAL_SAMPLES_PER_DISTANCE);

    TIMER_DelayMs(TOFCAL_SETTLE_MS);

    for (int s = 0; s < TOFCAL_SAMPLES_PER_DISTANCE; s++) {
        float zone_readings[TOFCAL_ZONES];
        if (!TOF_I2C_ReadAllZones(zone_readings)) {
            return false;
        }

        for (int z = 0; z < TOFCAL_ZONES; z++) {
            sum[z] += zone_readings[z];
        }
        TIMER_DelayMs(10);
    }

    float inv_n = 1.0f / (float)TOFCAL_SAMPLES_PER_DISTANCE;
    for (int z = 0; z < TOFCAL_ZONES; z++) {
        zone_avg[z] = sum[z] * inv_n;
    }

    UART_Print("  zone averages: [");
    for (int z = 0; z < TOFCAL_ZONES; z++) {
        UART_Print("%.1f%c", zone_avg[z], (z < TOFCAL_ZONES - 1) ? ", " : "]\r\n");
    }

    return true;
}

/* ---------------------------------------------------------------------------
 * Compute per-zone offsets (observed - true)
 * ------------------------------------------------------------------------- */
static void tofcal_compute_offsets(float samples[TOFCAL_TARGET_DISTANCES][TOFCAL_ZONES])
{
    for (int z = 0; z < TOFCAL_ZONES; z++) {
        for (int d = 0; d < TOFCAL_TARGET_DISTANCES; d++) {
            tofc_.zone_offset[z][d] = samples[d][z] - target_distances_mm[d];
        }
    }
}

/* ---------------------------------------------------------------------------
 * Polynomial fit (2nd order) via least squares for each zone
 * Fits: y = a*x^2 + b*x + c where y = measured, x = true distance
 * We want the inverse: corrected = measured - offset(measured)
 * For simplicity we fit offset vs measured.
 * ------------------------------------------------------------------------- */
static void tofcal_fit_polynomial(float samples[TOFCAL_TARGET_DISTANCES][TOFCAL_ZONES])
{
    for (int z = 0; z < TOFCAL_ZONES; z++) {
        /* Collect data points: x = measured, y = offset = measured - true */
        float x[TOFCAL_TARGET_DISTANCES];
        float y[TOFCAL_TARGET_DISTANCES];

        for (int d = 0; d < TOFCAL_TARGET_DISTANCES; d++) {
            x[d] = samples[d][z];
            y[d] = samples[d][z] - target_distances_mm[d];
        }

        /* Least squares quadratic fit using normal equations.
         * Solve: [n      sum(x)   sum(x^2) ] [c]   [sum(y)   ]
         *        [sum(x) sum(x^2) sum(x^3) ] [b] = [sum(x*y) ]
         *        [sum(x^2) sum(x^3) sum(x^4)] [a]   [sum(x^2*y)]
         *
         * For TOFCAL_TARGET_DISTANCES = 5, system is overdetermined.
         */

        float s_x  = 0.0f, s_x2 = 0.0f, s_x3 = 0.0f, s_x4 = 0.0f;
        float s_y  = 0.0f, s_xy = 0.0f, s_x2y = 0.0f;
        float n = (float)TOFCAL_TARGET_DISTANCES;

        for (int d = 0; d < TOFCAL_TARGET_DISTANCES; d++) {
            float xi = x[d];
            float yi = y[d];
            float xi2 = xi * xi;
            float xi3 = xi2 * xi;
            float xi4 = xi3 * xi;

            s_x   += xi;
            s_x2  += xi2;
            s_x3  += xi3;
            s_x4  += xi4;
            s_y   += yi;
            s_xy  += xi * yi;
            s_x2y += xi2 * yi;
        }

        /* Solve 3x3 system using Cramer's rule */
        float det = n * (s_x2 * s_x4 - s_x3 * s_x3)
                  - s_x * (s_x * s_x4 - s_x2 * s_x3)
                  + s_x2 * (s_x * s_x3 - s_x2 * s_x2);

        if (fabsf(det) < 1e-15f) {
            /* Degenerate; fall back to linear fit */
            tofc_.zone_poly_a[z] = 0.0f;
            tofc_.zone_poly_b[z] = 1.0f;
            tofc_.zone_poly_c[z] = 0.0f;
            continue;
        }

        float det_a = s_y * (s_x2 * s_x4 - s_x3 * s_x3)
                    - s_xy * (s_x * s_x4 - s_x2 * s_x3)
                    + s_x2y * (s_x * s_x3 - s_x2 * s_x2);

        float det_b = n * (s_xy * s_x4 - s_x3 * s_x2y)
                    - s_x * (s_y * s_x4 - s_x2 * s_x2y)
                    + s_x2 * (s_y * s_x3 - s_x2 * s_xy);

        float det_c = n * (s_x2 * s_x2y - s_x3 * s_xy)
                    - s_x * (s_x * s_x2y - s_x3 * s_y)
                    + s_x2 * (s_x * s_xy - s_x2 * s_y);

        tofc_.zone_poly_a[z] = det_a / det;
        tofc_.zone_poly_b[z] = det_b / det;
        tofc_.zone_poly_c[z] = det_c / det;

        UART_Print("  Zone %d poly: offset = %.4f*x^2 + %.4f*x + %.4f\r\n",
                   z, tofc_.zone_poly_a[z], tofc_.zone_poly_b[z], tofc_.zone_poly_c[z]);
    }
}

/* ---------------------------------------------------------------------------
 * Estimate crosstalk compensation per zone
 * Crosstalk appears as a constant offset independent of distance.
 * Estimated from the offset at the shortest distance, after removing
 * the distance-dependent part.
 * ------------------------------------------------------------------------- */
static void tofcal_compute_crosstalk(float samples[TOFCAL_TARGET_DISTANCES][TOFCAL_ZONES])
{
    for (int z = 0; z < TOFCAL_ZONES; z++) {
        /* Use the shortest distance (10 cm) to estimate crosstalk */
        float offset_at_short = samples[0][z] - target_distances_mm[0];
        /* Remove the polynomial-predicted offset at this distance */
        float x = samples[0][z];
        float predicted_offset = tofc_.zone_poly_a[z] * x * x
                               + tofc_.zone_poly_b[z] * x
                               + tofc_.zone_poly_c[z];
        tofc_.crosstalk_comp[z] = offset_at_short - predicted_offset;
        if (tofc_.crosstalk_comp[z] < 0.0f) {
            tofc_.crosstalk_comp[z] = 0.0f;  /* Non-negative crosstalk */
        }
        UART_Print("  Zone %d crosstalk comp = %.2f mm\r\n", z, tofc_.crosstalk_comp[z]);
    }
}

/* ---------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

bool TOFCAL_RunAll(void)
{
    UART_Print("TOF_CAL: starting production calibration...\r\n");
    memset(&tofc_, 0, sizeof(tofc_));
    tofc_.magic = CAL_PARAM_MAGIC;

    /* Storage: [distance_index][zone] */
    float samples[TOFCAL_TARGET_DISTANCES][TOFCAL_ZONES];

    for (int d = 0; d < TOFCAL_TARGET_DISTANCES; d++) {
        UART_Print("  Place target at %.0f mm...\r\n", target_distances_mm[d]);
        /* In production, the calibration box positions are fixed */
        if (!tofcal_sample_distance(d, samples[d])) {
            UART_Print("TOF_CAL: sampling failed at distance %d\r\n", d);
            return false;
        }
    }

    /* Compute calibration parameters */
    tofcal_compute_offsets(samples);
    tofcal_fit_polynomial(samples);
    tofcal_compute_crosstalk(samples);

    /* Save to flash */
    tofc_.crc32 = cal_crc32((const uint8_t *)&tofc_,
                             offsetof(ToF_CalParams, crc32));
    FACTORY_FLASH_Write(FACTORY_SECTOR_TOF, (uint32_t)&tofc_, sizeof(tofc_));

    UART_Print("TOF_CAL: saved to flash. PASS\r\n");
    return true;
}

bool TOFCAL_LoadFromFlash(void)
{
    if (!FACTORY_FLASH_Read(FACTORY_SECTOR_TOF, (uint32_t)&tofc_, sizeof(tofc_))) {
        return false;
    }
    if (tofc_.magic != CAL_PARAM_MAGIC) return false;

    uint32_t expected_crc = cal_crc32((const uint8_t *)&tofc_,
                                       offsetof(ToF_CalParams, crc32));
    return (expected_crc == tofc_.crc32);
}

float TOFCAL_CorrectMeasurement(uint8_t zone, float raw_distance_mm)
{
    if (zone >= TOFCAL_ZONES) return raw_distance_mm;

    /* Apply polynomial offset correction */
    float x = raw_distance_mm;
    float offset = tofc_.zone_poly_a[zone] * x * x
                 + tofc_.zone_poly_b[zone] * x
                 + tofc_.zone_poly_c[zone];

    float corrected = raw_distance_mm - offset - tofc_.crosstalk_comp[zone];
    if (corrected < 0.0f) corrected = 0.0f;

    return corrected;
}

float TOFCAL_GetZoneOffset(uint8_t zone, int distance_idx)
{
    if (zone >= TOFCAL_ZONES) return 0.0f;
    if (distance_idx >= TOFCAL_TARGET_DISTANCES) return 0.0f;
    return tofc_.zone_offset[zone][distance_idx];
}
