/**
  ******************************************************************************
  * @file    foc_params.c
  * @author  M7 FOC Team
  * @brief   Runtime parameter tables for FOC control.
  *
  *          Contains:
  *            1. Speed-dependent PID gain table (Kp, Ki for low vs high speed)
  *            2. Current-limit vs temperature derating table
  *            3. Motor-specific electrical parameters
  *            4. Lookup helpers with linear interpolation
  *
  *          Gain scheduling: at low speed we use higher gains for tight
  *          tracking; at high speed we reduce gains for stability (avoiding
  *          excitation of mechanical resonances).
  *
  *          Temperature derating: as motor temperature rises, the maximum
  *          allowable current is reduced to protect the windings.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include <string.h>
#include <math.h>

/* Private defines -----------------------------------------------------------*/
#define KP_TABLE_SIZE           16U
#define KI_TABLE_SIZE           16U
#define DERATE_TABLE_SIZE       12U

/* Default motor parameters (example: custom 120 W BLDC) */
#define MOTOR_RATED_CURRENT_A   8.0f
#define MOTOR_RATED_VOLTAGE_V   24.0f
#define MOTOR_RATED_SPEED_RPM   3500.0f
#define MOTOR_POLE_PAIRS        7U
#define MOTOR_PHASE_RESISTANCE  0.050f
#define MOTOR_PHASE_INDUCTANCE  0.000015f
#define MOTOR_MAX_TEMP_C        120.0f

/* ---------------------------------------------------------------------------*/
/*  Speed-dependent PI gain table                                             */
/* ---------------------------------------------------------------------------*/
typedef struct {
    float speed_rpm;    /* mechanical speed [RPM] at this breakpoint */
    float Kp;           /* proportional gain */
    float Ki;           /* integral gain */
} PID_GainEntry;

/* Gain schedule: low speed -> high Kp/Ki for tracking; high speed -> lower */
static const PID_GainEntry kp_table[KP_TABLE_SIZE] = {
    {    0.0f,   3.500f,   0.120f },
    {  100.0f,   3.500f,   0.120f },
    {  250.0f,   3.200f,   0.110f },
    {  500.0f,   2.800f,   0.100f },
    {  750.0f,   2.500f,   0.090f },
    { 1000.0f,   2.200f,   0.080f },
    { 1500.0f,   1.800f,   0.065f },
    { 2000.0f,   1.500f,   0.055f },
    { 2500.0f,   1.200f,   0.045f },
    { 3000.0f,   1.000f,   0.035f },
    { 3500.0f,   0.800f,   0.030f },
    { 4000.0f,   0.650f,   0.025f },
    { 5000.0f,   0.500f,   0.020f },
    { 6000.0f,   0.400f,   0.015f },
    { 8000.0f,   0.300f,   0.010f },
    {10000.0f,   0.250f,   0.008f },
};

/* ---------------------------------------------------------------------------*/
/*  Current-limit vs temperature derating table                                */
/* ---------------------------------------------------------------------------*/
typedef struct {
    float temp_c;       /* motor temperature [deg C] */
    float derate_pu;    /* current limit multiplier (1.0 = no derating) */
} DerateEntry;

static const DerateEntry derate_table[DERATE_TABLE_SIZE] = {
    {  25.0f,   1.000f },
    {  40.0f,   1.000f },
    {  50.0f,   0.950f },
    {  60.0f,   0.900f },
    {  70.0f,   0.820f },
    {  80.0f,   0.720f },
    {  90.0f,   0.600f },
    { 100.0f,   0.450f },
    { 110.0f,   0.300f },
    { 120.0f,   0.150f },
    { 130.0f,   0.050f },
    { 140.0f,   0.000f },
};

/* ---------------------------------------------------------------------------*/
/*  Motor parameters structure                                                */
/* ---------------------------------------------------------------------------*/
typedef struct {
    /* Electrical */
    float Rs;                   /* phase resistance [Ohm] */
    float Ld;                   /* D-axis inductance [H] */
    float Lq;                   /* Q-axis inductance [H] */
    float flux_permanent;       /* permanent magnet flux linkage [Wb] */

    /* Ratings */
    float rated_current;        /* RMS rated current [A] */
    float rated_voltage;        /* rated voltage [V] */
    float rated_speed_rpm;      /* rated speed [RPM] */
    uint16_t pole_pairs;

    /* Thermal */
    float max_temp_c;           /* maximum winding temperature */
    float current_temp_c;       /* current winding temperature estimate */

    /* Current limits */
    float i_max_continuous;     /* continuous max current [A] */
    float i_max_peak;           /* peak current (10 s) [A] */
    float i_max_derated;        /* derated limit at current temp [A] */

    /* Status */
    uint8_t params_loaded;
    uint8_t derating_active;
} MotorParamsTypeDef;

static MotorParamsTypeDef hmotor;

/* ---------------------------------------------------------------------------*/
/*  Lookup helpers                                                            */
/* ---------------------------------------------------------------------------*/

/** @brief  Linear interpolation in a gain table.
  * @param  x        Input value (speed).
  * @param  table    Pointer to table entries.
  * @param  size     Number of entries.
  * @return Interpolated Kp value.
  */
static float interpolate_gain_table(float x, const PID_GainEntry *table, uint32_t size)
{
    if (size == 0U) return 0.0f;

    /* Bounds */
    if (x <= table[0].speed_rpm) return table[0].Kp;
    if (x >= table[size - 1U].speed_rpm) return table[size - 1U].Kp;

    /* Binary search */
    uint32_t lo = 0U;
    uint32_t hi = size - 1U;
    while (hi - lo > 1U) {
        uint32_t mid = (lo + hi) >> 1U;
        if (table[mid].speed_rpm <= x) {
            lo = mid;
        } else {
            hi = mid;
        }
    }

    float x_lo = table[lo].speed_rpm;
    float x_hi = table[hi].speed_rpm;
    float y_lo = table[lo].Kp;
    float y_hi = table[hi].Kp;

    if (x_hi > x_lo) {
        float frac = (x - x_lo) / (x_hi - x_lo);
        return y_lo + frac * (y_hi - y_lo);
    }
    return y_lo;
}

/** @brief  Linear interpolation in a derating table.
  * @param  temp_c   Motor temperature [deg C].
  * @return Derating factor (0.0 to 1.0).
  */
static float interpolate_derate_table(float temp_c)
{
    if (temp_c <= derate_table[0].temp_c) return derate_table[0].derate_pu;
    if (temp_c >= derate_table[DERATE_TABLE_SIZE - 1U].temp_c)
        return derate_table[DERATE_TABLE_SIZE - 1U].derate_pu;

    uint32_t lo = 0U;
    uint32_t hi = DERATE_TABLE_SIZE - 1U;
    while (hi - lo > 1U) {
        uint32_t mid = (lo + hi) >> 1U;
        if (derate_table[mid].temp_c <= temp_c) {
            lo = mid;
        } else {
            hi = mid;
        }
    }

    float t_lo = derate_table[lo].temp_c;
    float t_hi = derate_table[hi].temp_c;
    float d_lo = derate_table[lo].derate_pu;
    float d_hi = derate_table[hi].derate_pu;

    if (t_hi > t_lo) {
        float frac = (temp_c - t_lo) / (t_hi - t_lo);
        return d_lo + frac * (d_hi - d_lo);
    }
    return d_lo;
}

/* ---------------------------------------------------------------------------*/
/*  Public API                                                                */
/* ---------------------------------------------------------------------------*/

/** @brief  Initialise motor parameters with defaults. */
void PARAMS_Init(void)
{
    memset(&hmotor, 0, sizeof(hmotor));

    hmotor.Rs               = MOTOR_PHASE_RESISTANCE;
    hmotor.Ld               = MOTOR_PHASE_INDUCTANCE;
    hmotor.Lq               = MOTOR_PHASE_INDUCTANCE;
    hmotor.flux_permanent   = 0.0025f;         /* 2.5 mWb */
    hmotor.rated_current    = MOTOR_RATED_CURRENT_A;
    hmotor.rated_voltage    = MOTOR_RATED_VOLTAGE_V;
    hmotor.rated_speed_rpm  = MOTOR_RATED_SPEED_RPM;
    hmotor.pole_pairs       = MOTOR_POLE_PAIRS;
    hmotor.max_temp_c       = MOTOR_MAX_TEMP_C;
    hmotor.current_temp_c   = 25.0f;            /* ambient startup */
    hmotor.i_max_continuous = MOTOR_RATED_CURRENT_A;
    hmotor.i_max_peak       = MOTOR_RATED_CURRENT_A * 1.5f;
    hmotor.i_max_derated    = MOTOR_RATED_CURRENT_A;
    hmotor.params_loaded    = 1U;
    hmotor.derating_active  = 0U;
}

/** @brief  Look up proportional gain for a given speed.
  * @param  speed_rpm  Current mechanical speed [RPM].
  * @return Kp value.
  */
float PARAMS_GetKp(float speed_rpm)
{
    return interpolate_gain_table(fabsf(speed_rpm), kp_table, KP_TABLE_SIZE);
}

/** @brief  Look up integral gain for a given speed.
  * @param  speed_rpm  Current mechanical speed [RPM].
  * @return Ki value.
  */
float PARAMS_GetKi(float speed_rpm)
{
    /* Ki uses same structure but we extract Ki from the table */
    return interpolate_gain_table(fabsf(speed_rpm), kp_table, KP_TABLE_SIZE)
           * 0.04f; /* Ki ~ 4% of Kp as default ratio; actual stored in table is unused */
}

/* Alternative: use the actual Ki values from the table */
static float interpolate_ki_table(float x)
{
    if (x <= kp_table[0].speed_rpm) return kp_table[0].Ki;
    if (x >= kp_table[KP_TABLE_SIZE - 1U].speed_rpm) return kp_table[KP_TABLE_SIZE - 1U].Ki;

    uint32_t lo = 0U;
    uint32_t hi = KP_TABLE_SIZE - 1U;
    while (hi - lo > 1U) {
        uint32_t mid = (lo + hi) >> 1U;
        if (kp_table[mid].speed_rpm <= x) {
            lo = mid;
        } else {
            hi = mid;
        }
    }

    float frac = (x - kp_table[lo].speed_rpm) / (kp_table[hi].speed_rpm - kp_table[lo].speed_rpm);
    return kp_table[lo].Ki + frac * (kp_table[hi].Ki - kp_table[lo].Ki);
}

/** @brief  Get both Kp and Ki in one call (more efficient). */
void PARAMS_GetGains(float speed_rpm, float *pKp, float *pKi)
{
    float abs_spd = fabsf(speed_rpm);
    *pKp = interpolate_gain_table(abs_spd, kp_table, KP_TABLE_SIZE);
    *pKi = interpolate_ki_table(abs_spd);
}

/** @brief  Get derated current limit for a given temperature.
  * @param  temp_c  Winding temperature [deg C].
  * @return Maximum allowed current [A].
  */
float PARAMS_GetDeratedCurrent(float temp_c)
{
    float derate = interpolate_derate_table(temp_c);
    float limit = hmotor.i_max_continuous * derate;

    hmotor.current_temp_c  = temp_c;
    hmotor.i_max_derated   = limit;
    hmotor.derating_active = (derate < 0.99f) ? 1U : 0U;

    return limit;
}

/** @brief  Update winding temperature estimate (call from thermal monitoring). */
void PARAMS_SetTemperature(float temp_c)
{
    hmotor.current_temp_c = temp_c;
    PARAMS_GetDeratedCurrent(temp_c);
}

/** @brief  Get current temperature estimate. */
float PARAMS_GetTemperature(void)
{
    return hmotor.current_temp_c;
}

/** @brief  Get rated current. */
float PARAMS_GetRatedCurrent(void)
{
    return hmotor.rated_current;
}

/** @brief  Get pole pairs. */
uint16_t PARAMS_GetPolePairs(void)
{
    return hmotor.pole_pairs;
}

/** @brief  Get phase inductance (average of Ld and Lq for sensorless observer). */
float PARAMS_GetInductance(void)
{
    return (hmotor.Ld + hmotor.Lq) * 0.5f;
}

/** @brief  Get D-axis inductance. */
float PARAMS_GetLd(void) { return hmotor.Ld; }

/** @brief  Get Q-axis inductance. */
float PARAMS_GetLq(void) { return hmotor.Lq; }

/** @brief  Get stator resistance. */
float PARAMS_GetResistance(void) { return hmotor.Rs; }

/** @brief  Override motor parameters at runtime (e.g., from NVM config). */
void PARAMS_SetMotorParams(float Rs, float Ld, float Lq, float flux,
                            float rated_current, uint16_t pole_pairs)
{
    if (Rs > 0.0f) hmotor.Rs = Rs;
    if (Ld > 0.0f) hmotor.Ld = Ld;
    if (Lq > 0.0f) hmotor.Lq = Lq;
    if (flux > 0.0f) hmotor.flux_permanent = flux;
    if (rated_current > 0.0f) {
        hmotor.rated_current = rated_current;
        hmotor.i_max_continuous = rated_current;
        hmotor.i_max_peak = rated_current * 1.5f;
    }
    if (pole_pairs > 0U) hmotor.pole_pairs = pole_pairs;
    hmotor.params_loaded = 1U;
}

/** @brief  Check if parameters have been loaded. */
uint8_t PARAMS_IsLoaded(void)
{
    return hmotor.params_loaded;
}

/** @brief  Check if derating is currently active. */
uint8_t PARAMS_IsDeratingActive(void)
{
    return hmotor.derating_active;
}

/** @brief  Get the peak current limit [A]. */
float PARAMS_GetPeakCurrent(void)
{
    return hmotor.i_max_peak;
}

/** @brief  Get the continuous current limit [A] (derated). */
float PARAMS_GetContinuousCurrent(void)
{
    return hmotor.i_max_derated;
}
