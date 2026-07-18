/**
  ******************************************************************************
  * @file    side_brush.c
  * @author  M7 FOC Team
  * @brief   Side brush speed control with edge-detection correlation.
  *
  *          The side brush extends beyond the robot's footprint to sweep
  *          debris from edges and corners into the cleaning path.
  *
  *          Features:
  *            - FOC-based speed control with current limit
  *            - Speed modulation based on robot state (wall following,
  *              open area, cornering)
  *            - Edge-detection correlation: slow down near walls for thorough
  *              edge cleaning; speed up in open areas for wider coverage
  *            - Stall protection (tangled cables, debris)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include <string.h>
#include <math.h>

/* Private defines -----------------------------------------------------------*/
#define SIDE_SPEED_NORMAL_RPM       2500.0f
#define SIDE_SPEED_WALL_RPM         1800.0f   /* slower near walls */
#define SIDE_SPEED_OPEN_RPM         3000.0f   /* faster in open area */
#define SIDE_SPEED_CORNER_RPM       1500.0f   /* slow in tight corners */
#define SIDE_SPEED_MAX_RPM          3500.0f
#define SIDE_SPEED_MIN_RPM          800.0f

#define SIDE_CURRENT_LIMIT_A        3.0f
#define SIDE_STALL_CURRENT_A        2.5f
#define SIDE_STALL_TIME_MS          400U

/* Edge detection thresholds (distance from ToF side sensor) */
#define SIDE_WALL_DISTANCE_MM       150.0f    /* within 150 mm = near wall */
#define SIDE_OPEN_DISTANCE_MM       500.0f    /* > 500 mm = open area */
#define SIDE_CORNER_DISTANCE_MM     80.0f     /* < 80 mm = corner */

/* Hysteresis */
#define SIDE_DIST_HYSTERESIS_MM     20.0f

/* Speed ramp rate [RPM/s] for smooth transitions */
#define SIDE_RAMP_RATE_RPMPS        5000.0f

/* ---------------------------------------------------------------------------*/
/*  Robot driving context (from navigation)                                    */
/* ---------------------------------------------------------------------------*/
typedef enum {
    SIDE_CONTEXT_NORMAL    = 0U,
    SIDE_CONTEXT_WALL      = 1U,
    SIDE_CONTEXT_OPEN      = 2U,
    SIDE_CONTEXT_CORNER    = 3U,
} SideBrushContext;

/* ---------------------------------------------------------------------------*/
/*  Side brush state                                                          */
/* ---------------------------------------------------------------------------*/
typedef struct {
    /* Speed control */
    float    speed_ref_rpm;
    float    speed_meas_rpm;
    float    speed_target_rpm;      /* target before ramping */
    float    iq_ref;
    float    iq_meas;

    /* Context */
    SideBrushContext context;
    SideBrushContext context_prev;
    float    side_distance_mm;      /* distance to left/right wall [mm] */
    uint8_t  wall_following_active; /* set by navigation */

    /* Stall */
    uint8_t  stall_detected;
    uint32_t stall_start_ms;

    /* State */
    uint8_t  enabled;
    uint8_t  fault;
    uint32_t last_update_ms;

    /* Derived */
    float    power_estimate_w;
} SideBrushHandleTypeDef;

static SideBrushHandleTypeDef hside;

/* ---------------------------------------------------------------------------*/
/*  Initialisation                                                            */
/* ---------------------------------------------------------------------------*/

/** @brief  Initialise side brush module. */
void SIDE_Init(void)
{
    memset(&hside, 0, sizeof(hside));
    hside.speed_ref_rpm     = SIDE_SPEED_NORMAL_RPM;
    hside.speed_target_rpm  = SIDE_SPEED_NORMAL_RPM;
    hside.context           = SIDE_CONTEXT_NORMAL;
    hside.context_prev      = SIDE_CONTEXT_NORMAL;
    hside.enabled           = 0U;
    hside.fault             = 0U;
    hside.stall_detected    = 0U;
    hside.side_distance_mm  = 999.0f; /* default: open area */
    hside.last_update_ms    = HAL_GetTick();
}

/** @brief  Enable/disable side brush. */
void SIDE_SetEnabled(uint8_t enabled)
{
    hside.enabled = enabled;
    if (!enabled) {
        hside.iq_ref = 0.0f;
        hside.speed_ref_rpm = 0.0f;
    }
}

/** @brief  Get enabled status. */
uint8_t SIDE_IsEnabled(void)
{
    return hside.enabled;
}

/* ---------------------------------------------------------------------------*/
/*  Context determination from side distance sensor                            */
/* ---------------------------------------------------------------------------*/

/** @brief  Determine the current cleaning context based on side distance.
  *         Uses hysteresis to prevent oscillation at boundaries.
  */
static SideBrushContext side_determine_context(float dist_mm)
{
    SideBrushContext ctx = hside.context; /* default: stay in current */

    if (dist_mm < (SIDE_CORNER_DISTANCE_MM - SIDE_DIST_HYSTERESIS_MM)) {
        ctx = SIDE_CONTEXT_CORNER;
    } else if (dist_mm > (SIDE_OPEN_DISTANCE_MM + SIDE_DIST_HYSTERESIS_MM)) {
        ctx = SIDE_CONTEXT_OPEN;
    } else if (dist_mm < (SIDE_WALL_DISTANCE_MM + SIDE_DIST_HYSTERESIS_MM)) {
        ctx = SIDE_CONTEXT_WALL;
    } else {
        ctx = SIDE_CONTEXT_NORMAL;
    }

    return ctx;
}

/** @brief  Get target speed for current context. */
static float side_get_target_speed(SideBrushContext ctx)
{
    switch (ctx) {
        case SIDE_CONTEXT_WALL:   return SIDE_SPEED_WALL_RPM;
        case SIDE_CONTEXT_OPEN:   return SIDE_SPEED_OPEN_RPM;
        case SIDE_CONTEXT_CORNER: return SIDE_SPEED_CORNER_RPM;
        case SIDE_CONTEXT_NORMAL:
        default:                  return SIDE_SPEED_NORMAL_RPM;
    }
}

/* ---------------------------------------------------------------------------*/
/*  Stall detection                                                           */
/* ---------------------------------------------------------------------------*/

static void side_detect_stall(void)
{
    uint32_t now_ms = HAL_GetTick();

    if ((fabsf(hside.iq_meas) > SIDE_STALL_CURRENT_A) &&
        (fabsf(hside.speed_meas_rpm) < 30.0f)) {
        if (hside.stall_start_ms == 0U) {
            hside.stall_start_ms = now_ms;
        }
        if ((now_ms - hside.stall_start_ms) > SIDE_STALL_TIME_MS) {
            hside.stall_detected = 1U;
            hside.fault = 1U;
            hside.enabled = 0U;
            hside.iq_ref = 0.0f;
        }
    } else {
        if ((now_ms - hside.stall_start_ms) > 200U) {
            hside.stall_start_ms = 0U;
        }
    }
}

/* ---------------------------------------------------------------------------*/
/*  Main update                                                               */
/* ---------------------------------------------------------------------------*/

/** @brief  Run side brush update.
  *         Call at 1 kHz.
  *
  * @param  iq_meas      Measured Q-axis current [A].
  * @param  speed_rpm    Measured speed [RPM].
  * @param  side_dist_mm Distance to side wall from ToF sensor [mm].
  *                       Set to a large value (e.g. 999) if no wall detected.
  */
void SIDE_Update(float iq_meas, float speed_rpm, float side_dist_mm)
{
    if (!hside.enabled || hside.fault) return;

    hside.iq_meas        = iq_meas;
    hside.speed_meas_rpm = speed_rpm;
    hside.side_distance_mm = side_dist_mm;

    uint32_t now_ms = HAL_GetTick();
    float dt_s = (float)(now_ms - hside.last_update_ms) * 0.001f;
    if (dt_s < 0.001f) dt_s = 0.001f;
    if (dt_s > 0.1f) dt_s = 0.1f; /* cap at 100 ms */
    hside.last_update_ms = now_ms;

    /* ---- 1. Determine context ---- */
    hside.context_prev = hside.context;
    hside.context = side_determine_context(side_dist_mm);

    /* ---- 2. Set target speed from context ---- */
    hside.speed_target_rpm = side_get_target_speed(hside.context);

    /* If wall-following is active and we are near a wall, use wall speed */
    if (hside.wall_following_active && (hside.context == SIDE_CONTEXT_WALL)) {
        hside.speed_target_rpm = SIDE_SPEED_WALL_RPM;
    }

    /* ---- 3. Ramp speed reference (smooth transitions) ---- */
    float ramp_step = SIDE_RAMP_RATE_RPMPS * dt_s;
    float diff = hside.speed_target_rpm - hside.speed_ref_rpm;

    if (fabsf(diff) <= ramp_step) {
        hside.speed_ref_rpm = hside.speed_target_rpm;
    } else if (diff > 0.0f) {
        hside.speed_ref_rpm += ramp_step;
    } else {
        hside.speed_ref_rpm -= ramp_step;
    }

    /* Clamp */
    if (hside.speed_ref_rpm > SIDE_SPEED_MAX_RPM) hside.speed_ref_rpm = SIDE_SPEED_MAX_RPM;
    if (hside.speed_ref_rpm < SIDE_SPEED_MIN_RPM) hside.speed_ref_rpm = SIDE_SPEED_MIN_RPM;

    /* ---- 4. Stall detection ---- */
    side_detect_stall();

    /* ---- 5. Power estimate ---- */
    hside.power_estimate_w = 24.0f * fabsf(iq_meas) * 0.65f;
}

/* ---------------------------------------------------------------------------*/
/*  Public API                                                                */
/* ---------------------------------------------------------------------------*/

/** @brief  Get current speed reference [RPM]. */
float SIDE_GetSpeedRef(void)
{
    return hside.speed_ref_rpm;
}

/** @brief  Get IQ reference for FOC [A]. */
float SIDE_GetIqRef(void)
{
    return hside.iq_ref;
}

/** @brief  Set IQ reference directly (torque control mode). */
void SIDE_SetIqRef(float iq)
{
    if (hside.fault) return;
    if (iq > SIDE_CURRENT_LIMIT_A) iq = SIDE_CURRENT_LIMIT_A;
    if (iq < -SIDE_CURRENT_LIMIT_A) iq = -SIDE_CURRENT_LIMIT_A;
    hside.iq_ref = iq;
}

/** @brief  Get current context. */
SideBrushContext SIDE_GetContext(void)
{
    return hside.context;
}

/** @brief  Get context as human-readable string. */
const char* SIDE_GetContextStr(void)
{
    switch (hside.context) {
        case SIDE_CONTEXT_WALL:   return "WALL";
        case SIDE_CONTEXT_OPEN:   return "OPEN";
        case SIDE_CONTEXT_CORNER: return "CORNER";
        case SIDE_CONTEXT_NORMAL:
        default:                  return "NORMAL";
    }
}

/** @brief  Get stall status. */
uint8_t SIDE_IsStalled(void)
{
    return hside.stall_detected;
}

/** @brief  Clear stall fault. */
void SIDE_ClearStall(void)
{
    hside.stall_detected = 0U;
    hside.stall_start_ms = 0U;
    hside.fault = 0U;
    hside.enabled = 1U;
}

/** @brief  Get fault flag. */
uint8_t SIDE_GetFault(void)
{
    return hside.fault;
}

/** @brief  Get estimated power [W]. */
float SIDE_GetPower(void)
{
    return hside.power_estimate_w;
}

/** @brief  Notify side brush that wall-following mode is active. */
void SIDE_SetWallFollowingActive(uint8_t active)
{
    hside.wall_following_active = active;
}

/** @brief  Force set context (e.g., from high-level planner). */
void SIDE_ForceContext(uint8_t context)
{
    if (context <= SIDE_CONTEXT_CORNER) {
        hside.context = (SideBrushContext)context;
    }
}
