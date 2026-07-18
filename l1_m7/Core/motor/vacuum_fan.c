/**
  ******************************************************************************
  * @file    vacuum_fan.c
  * @author  M7 FOC Team
  * @brief   Vacuum fan BLDC FOC control with constant power mode.
  *
  *          The vacuum fan creates suction for debris pickup.  Key features:
  *            - FOC-based sinusoidal drive for quiet, efficient operation
  *            - Constant power mode (P = V * I): automatically regulates
  *              speed to maintain target suction power regardless of load
  *            - Speed range: 8000 - 25000 RPM (high-speed motor)
  *            - Dust-bin-full detection via motor current increase at
  *              constant power (when bin is full, airflow drops, load changes)
  *            - Soft-start to limit inrush current
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include <string.h>
#include <math.h>

/* Private defines -----------------------------------------------------------*/
#define FAN_SPEED_MIN_RPM           8000U
#define FAN_SPEED_MAX_RPM           25000U
#define FAN_SPEED_DEFAULT_RPM       18000U

#define FAN_CURRENT_LIMIT_A         5.0f
#define FAN_POWER_MAX_W             120.0f
#define FAN_POWER_DEFAULT_W         50.0f

/* Constant power control */
#define FAN_POWER_KP                0.001f
#define FAN_POWER_KI                0.0001f
#define FAN_POWER_INTEGRAL_MAX      5000.0f

/* Dust-bin-full detection */
#define FAN_DUST_FULL_CURRENT_RISE  0.20f   /* 20% current rise at const power */
#define FAN_DUST_FULL_DEBOUNCE_MS   2000U   /* 2 s debounce */

/* Soft-start */
#define FAN_SOFTSTART_RAMP_RPMPS    50000.0f  /* RPM per second */
#define FAN_SOFTSTART_CURRENT_LIMIT 2.0f       /* limit during startup */

/* ---------------------------------------------------------------------------*/
/*  Fan state                                                                 */
/* ---------------------------------------------------------------------------*/
typedef enum {
    FAN_STATE_IDLE       = 0U,
    FAN_STATE_SOFTSTART  = 1U,
    FAN_STATE_RUN        = 2U,
    FAN_STATE_FAULT      = 3U,
} FanState;

typedef struct {
    /* State machine */
    FanState   state;
    FanState   state_prev;

    /* Speed */
    float      speed_ref_rpm;
    float      speed_meas_rpm;
    float      speed_target_rpm;

    /* Current / power */
    float      iq_ref;
    float      iq_meas;
    float      power_target_w;       /* target suction power [W] */
    float      power_meas_w;         /* measured electrical power [W] */

    /* Constant power controller */
    float      cp_kp;
    float      cp_ki;
    float      cp_integral;

    /* Dust-bin detection */
    float      iq_baseline;          /* baseline current at clean bin */
    float      iq_current_avg;       /* running average of current */
    uint8_t    dust_bin_full;
    uint32_t   dust_full_start_ms;

    /* Soft-start */
    uint32_t   softstart_start_ms;
    float      softstart_target_rpm;

    /* General */
    uint8_t    enabled;
    uint8_t    fault;
    uint32_t   last_update_ms;

    /* Diagnostics */
    uint16_t   run_hours;            /* cumulative run time */
} FanHandleTypeDef;

static FanHandleTypeDef hfan;

/* ---------------------------------------------------------------------------*/
/*  Initialisation                                                            */
/* ---------------------------------------------------------------------------*/

/** @brief  Initialise vacuum fan module. */
void FAN_Init(void)
{
    memset(&hfan, 0, sizeof(hfan));

    hfan.state              = FAN_STATE_IDLE;
    hfan.speed_ref_rpm      = 0.0f;
    hfan.speed_target_rpm   = 0.0f;
    hfan.power_target_w     = FAN_POWER_DEFAULT_W;
    hfan.cp_kp              = FAN_POWER_KP;
    hfan.cp_ki              = FAN_POWER_KI;
    hfan.cp_integral        = 0.0f;
    hfan.enabled            = 0U;
    hfan.fault              = 0U;
    hfan.dust_bin_full      = 0U;
    hfan.iq_baseline        = 0.0f;
    hfan.last_update_ms     = HAL_GetTick();
    hfan.run_hours          = 0U;
}

/** @brief  Enable fan (starts soft-start sequence). */
void FAN_SetEnabled(uint8_t enabled)
{
    if (enabled && !hfan.enabled) {
        /* Transition to soft-start */
        hfan.state = FAN_STATE_SOFTSTART;
        hfan.softstart_start_ms = HAL_GetTick();
        hfan.speed_target_rpm = FAN_SPEED_DEFAULT_RPM;
        hfan.cp_integral = 0.0f;
    } else if (!enabled && hfan.enabled) {
        hfan.state = FAN_STATE_IDLE;
        hfan.iq_ref = 0.0f;
        hfan.speed_ref_rpm = 0.0f;
        hfan.cp_integral = 0.0f;
    }
    hfan.enabled = enabled;
}

/** @brief  Get enabled status. */
uint8_t FAN_IsEnabled(void)
{
    return hfan.enabled;
}

/* ---------------------------------------------------------------------------*/
/*  Constant power controller                                                 */
/* ---------------------------------------------------------------------------*/

/** @brief  Run constant power controller.
  *
  *         Measures electrical power (Vdc * Iq) and adjusts speed reference
  *         to maintain constant power.
  *
  *         If bin is empty: fan moves free air, lower current at given speed.
  *         If bin is full: airflow restricted, load changes -> current changes
  *         -> power controller adjusts speed.
  *
  * @param  v_dc     DC bus voltage [V].
  * @return Speed reference to maintain target power [RPM].
  */
static float fan_power_controller(float v_dc)
{
    if (v_dc < 1.0f) v_dc = 24.0f;

    /* Measured power */
    hfan.power_meas_w = v_dc * fabsf(hfan.iq_meas) * 0.85f; /* efficiency factor */

    /* Power error */
    float err = hfan.power_target_w - hfan.power_meas_w;

    /* PI control */
    float p_term = hfan.cp_kp * err;
    hfan.cp_integral += hfan.cp_ki * err;

    /* Anti-windup */
    if (hfan.cp_integral > FAN_POWER_INTEGRAL_MAX) hfan.cp_integral = FAN_POWER_INTEGRAL_MAX;
    if (hfan.cp_integral < -FAN_POWER_INTEGRAL_MAX) hfan.cp_integral = -FAN_POWER_INTEGRAL_MAX;

    float speed_adjust = p_term + hfan.cp_integral;

    /* Apply to base speed */
    float base_speed = FAN_SPEED_DEFAULT_RPM;
    float new_target = base_speed + speed_adjust;

    /* Clamp */
    if (new_target > FAN_SPEED_MAX_RPM) new_target = (float)FAN_SPEED_MAX_RPM;
    if (new_target < FAN_SPEED_MIN_RPM) new_target = (float)FAN_SPEED_MIN_RPM;

    return new_target;
}

/* ---------------------------------------------------------------------------*/
/*  Dust-bin-full detection                                                   */
/* ---------------------------------------------------------------------------*/

/** @brief  Detect dust bin full condition from motor current behaviour.
  *
  *          When the bin is full, airflow is restricted.  At constant power,
  *          the fan speed controller adjusts speed, and the resulting
  *          equilibrium current changes compared to the empty-bin baseline.
  *
  *          Method: At constant power mode, if the average Iq deviates from
  *          baseline by > FAN_DUST_FULL_CURRENT_RISE for > debounce period,
  *          declare bin full.
  */
static void fan_detect_dust_bin(void)
{
    /* Update running average of Iq */
    const float alpha = 0.99f;
    hfan.iq_current_avg = alpha * hfan.iq_current_avg + (1.0f - alpha) * fabsf(hfan.iq_meas);

    /* Establish baseline after first 5 seconds of stable operation */
    static uint8_t baseline_established = 0U;
    if (!baseline_established && (hfan.state == FAN_STATE_RUN)) {
        if ((HAL_GetTick() - hfan.softstart_start_ms) > 5000U) {
            hfan.iq_baseline = hfan.iq_current_avg;
            baseline_established = 1U;
        }
    }

    if (!baseline_established) return;

    /* Check deviation */
    float iq_diff = (hfan.iq_current_avg - hfan.iq_baseline) / (hfan.iq_baseline + 0.001f);

    if (iq_diff > FAN_DUST_FULL_CURRENT_RISE) {
        /* Current increased -> possible bin full */
        if (hfan.dust_full_start_ms == 0U) {
            hfan.dust_full_start_ms = HAL_GetTick();
        }
        if ((HAL_GetTick() - hfan.dust_full_start_ms) > FAN_DUST_FULL_DEBOUNCE_MS) {
            hfan.dust_bin_full = 1U;
        }
    } else {
        hfan.dust_full_start_ms = 0U;
        hfan.dust_bin_full = 0U;
    }
}

/* ---------------------------------------------------------------------------*/
/*  Main update                                                               */
/* ---------------------------------------------------------------------------*/

/** @brief  Run fan control update.
  *         Call at 1 kHz.
  *
  * @param  iq_meas   Measured Q-axis current [A].
  * @param  speed_rpm Measured speed [RPM].
  * @param  v_dc      DC bus voltage [V].
  */
void FAN_Update(float iq_meas, float speed_rpm, float v_dc)
{
    hfan.iq_meas        = iq_meas;
    hfan.speed_meas_rpm = speed_rpm;

    uint32_t now_ms = HAL_GetTick();
    float dt_s = (float)(now_ms - hfan.last_update_ms) * 0.001f;
    if (dt_s < 0.001f) dt_s = 0.001f;
    hfan.last_update_ms = now_ms;

    /* ---- State machine ---- */
    switch (hfan.state) {

        case FAN_STATE_IDLE:
            hfan.iq_ref = 0.0f;
            hfan.speed_ref_rpm = 0.0f;
            break;

        case FAN_STATE_SOFTSTART:
            /* Ramp speed from 0 to target */
            if (hfan.speed_ref_rpm < hfan.speed_target_rpm) {
                hfan.speed_ref_rpm += FAN_SOFTSTART_RAMP_RPMPS * dt_s;
                if (hfan.speed_ref_rpm > hfan.speed_target_rpm) {
                    hfan.speed_ref_rpm = hfan.speed_target_rpm;
                }
            }

            /* Limit current during soft-start */
            if (fabsf(iq_meas) > FAN_SOFTSTART_CURRENT_LIMIT) {
                hfan.speed_ref_rpm *= 0.99f; /* back off if overcurrent */
            }

            /* Transition to RUN when speed reaches 90% of target */
            if (hfan.speed_ref_rpm > (hfan.speed_target_rpm * 0.9f)) {
                hfan.state = FAN_STATE_RUN;
            }
            break;

        case FAN_STATE_RUN:
            /* Constant power control */
            hfan.speed_target_rpm = fan_power_controller(v_dc);

            /* Ramp to target */
            float ramp = FAN_SOFTSTART_RAMP_RPMPS * dt_s;
            float diff = hfan.speed_target_rpm - hfan.speed_ref_rpm;
            if (fabsf(diff) <= ramp) {
                hfan.speed_ref_rpm = hfan.speed_target_rpm;
            } else if (diff > 0.0f) {
                hfan.speed_ref_rpm += ramp;
            } else {
                hfan.speed_ref_rpm -= ramp;
            }

            /* Clamp */
            if (hfan.speed_ref_rpm > FAN_SPEED_MAX_RPM) hfan.speed_ref_rpm = (float)FAN_SPEED_MAX_RPM;
            if (hfan.speed_ref_rpm < FAN_SPEED_MIN_RPM) hfan.speed_ref_rpm = (float)FAN_SPEED_MIN_RPM;

            /* Dust bin detection */
            fan_detect_dust_bin();
            break;

        case FAN_STATE_FAULT:
            hfan.iq_ref = 0.0f;
            hfan.speed_ref_rpm = 0.0f;
            hfan.enabled = 0U;
            break;

        default:
            break;
    }

    /* Update run hours */
    static uint32_t last_hour_tick = 0U;
    if (hfan.state == FAN_STATE_RUN) {
        if ((now_ms - last_hour_tick) > 3600000U) {
            hfan.run_hours++;
            last_hour_tick = now_ms;
        }
    }
}

/* ---------------------------------------------------------------------------*/
/*  Public API                                                                */
/* ---------------------------------------------------------------------------*/

/** @brief  Get speed reference [RPM]. */
float FAN_GetSpeedRef(void)
{
    return hfan.speed_ref_rpm;
}

/** @brief  Get IQ reference for FOC [A]. */
float FAN_GetIqRef(void)
{
    return hfan.iq_ref;
}

/** @brief  Set IQ reference directly (torque override). */
void FAN_SetIqRef(float iq)
{
    if (hfan.fault) return;
    if (iq > FAN_CURRENT_LIMIT_A) iq = FAN_CURRENT_LIMIT_A;
    if (iq < -FAN_CURRENT_LIMIT_A) iq = -FAN_CURRENT_LIMIT_A;
    hfan.iq_ref = iq;
}

/** @brief  Set constant power target [W] (typically 30-120 W). */
void FAN_SetPowerTarget(float watts)
{
    if (watts < 0.0f) watts = 0.0f;
    if (watts > FAN_POWER_MAX_W) watts = FAN_POWER_MAX_W;
    hfan.power_target_w = watts;
}

/** @brief  Get target power [W]. */
float FAN_GetPowerTarget(void)
{
    return hfan.power_target_w;
}

/** @brief  Get measured electrical power [W]. */
float FAN_GetPowerMeasured(void)
{
    return hfan.power_meas_w;
}

/** @brief  Get dust bin full status. */
uint8_t FAN_IsDustBinFull(void)
{
    return hfan.dust_bin_full;
}

/** @brief  Clear dust bin full flag (after bin emptied). */
void FAN_ClearDustBinFull(void)
{
    hfan.dust_bin_full = 0U;
    hfan.dust_full_start_ms = 0U;
    hfan.iq_baseline = 0.0f;
}

/** @brief  Get stall/fault status. */
uint8_t FAN_GetFault(void)
{
    return hfan.fault;
}

/** @brief  Clear fault. */
void FAN_ClearFault(void)
{
    hfan.fault = 0U;
    hfan.state = FAN_STATE_IDLE;
}

/** @brief  Get current fan state. */
uint8_t FAN_GetState(void)
{
    return (uint8_t)hfan.state;
}

/** @brief  Get fan state as string. */
const char* FAN_GetStateStr(void)
{
    switch (hfan.state) {
        case FAN_STATE_IDLE:      return "IDLE";
        case FAN_STATE_SOFTSTART: return "SOFTSTART";
        case FAN_STATE_RUN:       return "RUN";
        case FAN_STATE_FAULT:     return "FAULT";
        default:                  return "UNKNOWN";
    }
}

/** @brief  Get cumulative run hours. */
uint16_t FAN_GetRunHours(void)
{
    return hfan.run_hours;
}

/** @brief  Get running average current (for dust bin diagnostics). */
float FAN_GetAvgCurrent(void)
{
    return hfan.iq_current_avg;
}

/** @brief  Get the baseline current (clean bin). */
float FAN_GetBaselineCurrent(void)
{
    return hfan.iq_baseline;
}
