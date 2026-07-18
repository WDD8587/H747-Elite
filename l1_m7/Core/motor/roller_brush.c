/**
  ******************************************************************************
  * @file    roller_brush.c
  * @author  M7 FOC Team
  * @brief   Roller brush FOC motor control with carpet detection.
  *
  *          The roller brush sweeps debris into the robot's path for the
  *          vacuum intake.  Key features:
  *            - FOC-based current-limited speed control
  *            - Carpet detection via RMS current monitoring (carpet = higher
  *              mechanical load -> higher current)
  *            - Automatic speed boost on carpet to maintain cleaning
  *              effectiveness
  *            - Stall protection (tangled debris)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include <string.h>
#include <math.h>

/* Private defines -----------------------------------------------------------*/
#define ROLLER_SPEED_DEFAULT_RPM    1500.0f
#define ROLLER_SPEED_CARPET_RPM     2000.0f
#define ROLLER_SPEED_MAX_RPM        3000.0f
#define ROLLER_SPEED_MIN_RPM        500.0f

#define ROLLER_CURRENT_LIMIT_A      8.0f
#define ROLLER_CARPET_CURRENT_THRESH 4.5f   /* current above this = carpet */
#define ROLLER_STALL_CURRENT_A      6.0f    /* stall threshold */
#define ROLLER_STALL_TIME_MS        500U    /* 500 ms stall = fault */

#define ROLLER_CARPET_DEBOUNCE_MS   200U    /* 200 ms debounce for carpet */
#define ROLLER_CARPET_HYSTERESIS_A  0.5f    /* hysteresis on threshold */

#define ROLLER_RMS_WINDOW_SIZE      32U     /* RMS averaging window */

/* ---------------------------------------------------------------------------*/
/*  Roller brush state                                                        */
/* ---------------------------------------------------------------------------*/
typedef struct {
    /* Speed control */
    float    speed_ref_rpm;
    float    speed_meas_rpm;
    float    iq_ref;
    float    iq_meas;
    float    current_rms;           /* RMS of Iq over window */

    /* Carpet detection */
    uint8_t  on_carpet;             /* 1 = currently on carpet */
    uint8_t  carpet_prev;           /* previous carpet state */
    uint32_t carpet_change_ms;      /* timestamp of last state change */

    /* Current RMS buffer */
    float    rms_buffer[ROLLER_RMS_WINDOW_SIZE];
    uint32_t rms_index;

    /* Stall detection */
    uint8_t  stall_detected;
    uint32_t stall_start_ms;
    uint32_t stall_current_ms;

    /* State */
    uint8_t  enabled;
    uint8_t  fault;

    /* Derived */
    float    power_estimate_w;      /* estimated mechanical power [W] */
    float    speed_boost_factor;    /* 1.0 = normal, >1.0 = carpet boost */
} RollerBrushHandleTypeDef;

static RollerBrushHandleTypeDef hroller;

/* ---------------------------------------------------------------------------*/
/*  Initialisation                                                            */
/* ---------------------------------------------------------------------------*/

/** @brief  Initialise roller brush module. */
void ROLLER_Init(void)
{
    memset(&hroller, 0, sizeof(hroller));
    hroller.speed_ref_rpm    = ROLLER_SPEED_DEFAULT_RPM;
    hroller.speed_boost_factor = 1.0f;
    hroller.enabled          = 0U;
    hroller.fault            = 0U;
    hroller.stall_detected   = 0U;
    hroller.on_carpet        = 0U;
    hroller.carpet_prev      = 0U;
    hroller.rms_index        = 0U;
}

/** @brief  Enable/disable roller brush. */
void ROLLER_SetEnabled(uint8_t enabled)
{
    hroller.enabled = enabled;
    if (!enabled) {
        hroller.iq_ref = 0.0f;
        hroller.speed_ref_rpm = 0.0f;
    }
}

/** @brief  Get enabled status. */
uint8_t ROLLER_IsEnabled(void)
{
    return hroller.enabled;
}

/* ---------------------------------------------------------------------------*/
/*  RMS current calculator                                                    */
/* ---------------------------------------------------------------------------*/

/** @brief  Update RMS current from latest Iq sample.
  *         Sliding window RMS over ROLLER_RMS_WINDOW_SIZE samples.
  */
static void roller_update_rms(float iq)
{
    hroller.rms_buffer[hroller.rms_index] = iq * iq;
    hroller.rms_index++;
    if (hroller.rms_index >= ROLLER_RMS_WINDOW_SIZE) {
        hroller.rms_index = 0U;
    }

    float sum_sq = 0.0f;
    for (uint32_t i = 0; i < ROLLER_RMS_WINDOW_SIZE; i++) {
        sum_sq += hroller.rms_buffer[i];
    }
    hroller.current_rms = sqrtf(sum_sq / (float)ROLLER_RMS_WINDOW_SIZE);
}

/* ---------------------------------------------------------------------------*/
/*  Carpet detection                                                          */
/* ---------------------------------------------------------------------------*/

/** @brief  Detect carpet from RMS current with hysteresis and debounce.
  *         Carpet increases mechanical load -> higher motor current.
  */
static void roller_detect_carpet(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint8_t carpet_now = 0U;

    /* Threshold with hysteresis */
    if (hroller.current_rms > (ROLLER_CARPET_CURRENT_THRESH + ROLLER_CARPET_HYSTERESIS_A)) {
        carpet_now = 1U;
    } else if (hroller.current_rms < (ROLLER_CARPET_CURRENT_THRESH - ROLLER_CARPET_HYSTERESIS_A)) {
        carpet_now = 0U;
    } else {
        carpet_now = hroller.on_carpet; /* stay in current state */
    }

    /* Debounce */
    if (carpet_now != hroller.carpet_prev) {
        hroller.carpet_change_ms = now_ms;
        hroller.carpet_prev = carpet_now;
    }

    if ((now_ms - hroller.carpet_change_ms) > ROLLER_CARPET_DEBOUNCE_MS) {
        if (hroller.on_carpet != carpet_now) {
            hroller.on_carpet = carpet_now;
            /* Adjust speed target */
            if (carpet_now) {
                hroller.speed_boost_factor = ROLLER_SPEED_CARPET_RPM / ROLLER_SPEED_DEFAULT_RPM;
                if (hroller.speed_boost_factor < 1.0f) hroller.speed_boost_factor = 1.0f;
            } else {
                hroller.speed_boost_factor = 1.0f;
            }
        }
    }
}

/* ---------------------------------------------------------------------------*/
/*  Stall detection                                                           */
/* ---------------------------------------------------------------------------*/

/** @brief  Detect roller stall (blocked by tangled debris).
  *         Condition: high current + low speed for > 500 ms.
  */
static void roller_detect_stall(void)
{
    uint32_t now_ms = HAL_GetTick();

    if ((hroller.iq_meas > ROLLER_STALL_CURRENT_A) &&
        (fabsf(hroller.speed_meas_rpm) < 50.0f)) {
        /* Possible stall */
        if (hroller.stall_start_ms == 0U) {
            hroller.stall_start_ms = now_ms;
        }
        hroller.stall_current_ms = now_ms;
    } else {
        /* Not stalling */
        if ((now_ms - hroller.stall_current_ms) > 200U) {
            /* 200 ms of normal operation resets stall timer */
            hroller.stall_start_ms = 0U;
        }
    }

    if ((hroller.stall_start_ms > 0U) &&
        ((now_ms - hroller.stall_start_ms) > ROLLER_STALL_TIME_MS)) {
        hroller.stall_detected = 1U;
        hroller.fault = 1U;
        hroller.enabled = 0U;
        hroller.iq_ref = 0.0f;
    }
}

/* ---------------------------------------------------------------------------*/
/*  Main control loop                                                         */
/* ---------------------------------------------------------------------------*/

/** @brief  Run roller brush control update.
  *         Call at 1 kHz (or from outer control loop).
  *         Updates carpet detection, stall detection, and speed reference.
  */
void ROLLER_Update(float iq_meas, float speed_rpm)
{
    if (!hroller.enabled || hroller.fault) return;

    hroller.iq_meas = iq_meas;
    hroller.speed_meas_rpm = speed_rpm;

    /* Update RMS current */
    roller_update_rms(iq_meas);

    /* Carpet detection */
    roller_detect_carpet();

    /* Stall detection */
    roller_detect_stall();

    /* Compute boosted speed reference */
    float base_ref = ROLLER_SPEED_DEFAULT_RPM;
    float boosted_ref = base_ref * hroller.speed_boost_factor;

    /* Apply min/max limits */
    if (boosted_ref > ROLLER_SPEED_MAX_RPM) boosted_ref = ROLLER_SPEED_MAX_RPM;
    if (boosted_ref < ROLLER_SPEED_MIN_RPM) boosted_ref = ROLLER_SPEED_MIN_RPM;

    hroller.speed_ref_rpm = boosted_ref;

    /* Power estimate */
    hroller.power_estimate_w = 24.0f * fabsf(iq_meas) * 0.7f;
}

/* ---------------------------------------------------------------------------*/
/*  Public API — getters / setters                                            */
/* ---------------------------------------------------------------------------*/

/** @brief  Get roller speed reference [RPM]. */
float ROLLER_GetSpeedRef(void)
{
    return hroller.speed_ref_rpm;
}

/** @brief  Get IQ reference for FOC [A]. */
float ROLLER_GetIqRef(void)
{
    return hroller.iq_ref;
}

/** @brief  Set IQ reference directly (torque control mode). */
void ROLLER_SetIqRef(float iq)
{
    if (hroller.fault) return;
    if (iq > ROLLER_CURRENT_LIMIT_A) iq = ROLLER_CURRENT_LIMIT_A;
    if (iq < -ROLLER_CURRENT_LIMIT_A) iq = -ROLLER_CURRENT_LIMIT_A;
    hroller.iq_ref = iq;
}

/** @brief  Get carpet detection status. */
uint8_t ROLLER_IsOnCarpet(void)
{
    return hroller.on_carpet;
}

/** @brief  Get stall status. */
uint8_t ROLLER_IsStalled(void)
{
    return hroller.stall_detected;
}

/** @brief  Clear stall fault. */
void ROLLER_ClearStall(void)
{
    hroller.stall_detected = 0U;
    hroller.stall_start_ms = 0U;
    hroller.fault = 0U;
    hroller.enabled = 1U;
}

/** @brief  Get RMS current [A]. */
float ROLLER_GetCurrentRMS(void)
{
    return hroller.current_rms;
}

/** @brief  Get speed boost factor. */
float ROLLER_GetBoostFactor(void)
{
    return hroller.speed_boost_factor;
}

/** @brief  Get estimated power [W]. */
float ROLLER_GetPower(void)
{
    return hroller.power_estimate_w;
}

/** @brief  Get fault flag. */
uint8_t ROLLER_GetFault(void)
{
    return hroller.fault;
}

/** @brief  Override carpet detection state (e.g., from external sensor). */
void ROLLER_ForceCarpetState(uint8_t on_carpet)
{
    hroller.on_carpet = on_carpet;
    hroller.speed_boost_factor = on_carpet ?
        (ROLLER_SPEED_CARPET_RPM / ROLLER_SPEED_DEFAULT_RPM) : 1.0f;
}
