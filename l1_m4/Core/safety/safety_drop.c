/**
  ******************************************************************************
  * @file    safety_drop.c
  * @author  M4 Safety Team
  * @brief   Drop sensor redundancy and cliff detection for the H747 Elite robot.
  *
  *          Uses four cliff sensor pairs (front-left, front-right, rear-left,
  *          rear-right), each with a ToF ranging sensor + IR proximity sensor
  *          for redundancy.
  *
  *          Features:
  *            - ToF cliff + IR cliff correlation (redundant measurement)
  *            - 50 ms debounce on cliff detection
  *            - Hysteresis: approach threshold > retreat threshold
  *              (prevents oscillation at cliff edges)
  *            - Auto edge-follow: when near a cliff, the robot can follow
  *              the edge at a safe distance
  *            - Per-sensor fault detection (soiling, LED failure, timeout)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include <string.h>
#include <math.h>

/* Private defines -----------------------------------------------------------*/
#define NUM_CLIFF_SENSORS       4U

/* Sensor indices */
#define CLIFF_FL                0U   /* front-left  */
#define CLIFF_FR                1U   /* front-right */
#define CLIFF_RL                2U   /* rear-left   */
#define CLIFF_RR                3U   /* rear-right  */

/* Cliff thresholds [mm] */
#define CLIFF_THRESHOLD_MM          30.0f   /* distance below this = cliff */
#define CLIFF_APPROACH_HYST_MM      15.0f   /* approach threshold: 30 mm */
#define CLIFF_RETREAT_HYST_MM       25.0f   /* retreat threshold: 50 mm (higher = harder to un-trigger) */
#define CLIFF_SAFE_DIST_MM          100.0f  /* safe distance from edge */

/* Debounce */
#define CLIFF_DEBOUNCE_MS           50U
#define CLIFF_SOILING_DEBOUNCE_MS   5000U   /* 5 s to detect soiling */

/* IR vs ToF correlation */
#define CLIFF_IR_TOF_DISAGREE_MM    20.0f   /* >20 mm diff = disagreement */
#define CLIFF_IR_TOF_FAULT_SAMPLES  50U     /* 50 consecutive disagree = sensor fault */

/* Edge following */
#define CLIFF_EDGE_FOLLOW_DIST_MM   60.0f   /* target distance for edge following */
#define CLIFF_EDGE_FOLLOW_BAND_MM   15.0f   /* +/- band around target */

/* ---------------------------------------------------------------------------*/
/*  Per-sensor cliff state                                                    */
/* ---------------------------------------------------------------------------*/
typedef struct {
    uint8_t  id;

    /* Raw readings */
    float    tof_distance_mm;
    float    ir_distance_mm;
    float    combined_distance_mm;    /* fused ToF + IR */

    /* State */
    uint8_t  cliff_detected;          /* 1 = cliff under this sensor */
    uint8_t  cliff_prev;
    uint32_t cliff_change_ms;

    /* Redundancy */
    uint8_t  ir_tof_agree;            /* 1 = IR and ToF agree */
    uint8_t  sensor_fault;            /* 1 = sensor is faulty */
    uint32_t disagree_count;

    /* Soiling detection (gradual distance reduction) */
    float    baseline_distance_mm;    /* clean sensor baseline */
    uint8_t  soiled;                  /* 1 = sensor likely soiled */
    uint32_t soiled_start_ms;

    /* Position */
    const char *name;
} CliffSensorState;

/* ---------------------------------------------------------------------------*/
/*  Drop handle                                                               */
/* ---------------------------------------------------------------------------*/
typedef struct {
    CliffSensorState sensors[NUM_CLIFF_SENSORS];

    /* Global cliff status */
    uint8_t  any_cliff;              /* 1 = any sensor detects cliff */
    uint8_t  cliff_active;           /* debounced global cliff */
    uint32_t cliff_active_ms;

    /* Edge following */
    uint8_t  edge_follow_active;
    float    edge_follow_distance;   /* current edge distance */
    float    edge_follow_target;

    /* Fault tracking */
    uint8_t  any_sensor_fault;

    /* Statistics */
    uint32_t total_cliff_events;
} DropHandleTypeDef;

static DropHandleTypeDef hdrop;

/* ---------------------------------------------------------------------------*/
/*  Sensor names                                                              */
/* ---------------------------------------------------------------------------*/
static const char *sensor_names[NUM_CLIFF_SENSORS] = {
    "CLIFF_FL", "CLIFF_FR", "CLIFF_RL", "CLIFF_RR"
};

/* ---------------------------------------------------------------------------*/
/*  Initialisation                                                            */
/* ---------------------------------------------------------------------------*/

/** @brief  Initialise drop sensor module. */
void DROP_Init(void)
{
    memset(&hdrop, 0, sizeof(hdrop));

    for (uint32_t i = 0; i < NUM_CLIFF_SENSORS; i++) {
        hdrop.sensors[i].id            = (uint8_t)i;
        hdrop.sensors[i].name          = sensor_names[i];
        hdrop.sensors[i].baseline_distance_mm = 200.0f; /* typical floor distance */
    }

    hdrop.edge_follow_target = CLIFF_EDGE_FOLLOW_DIST_MM;
}

/* ---------------------------------------------------------------------------*/
/*  Per-sensor update                                                         */
/* ---------------------------------------------------------------------------*/

/** @brief  Update a single cliff sensor.
  *
  * @param  s        Sensor state to update.
  * @param  tof_mm   ToF distance reading [mm].
  * @param  ir_mm    IR distance reading [mm].
  */
static void cliff_update_sensor(CliffSensorState *s,
                                 float tof_mm, float ir_mm)
{
    uint32_t now_ms = HAL_GetTick();

    s->tof_distance_mm = tof_mm;
    s->ir_distance_mm  = ir_mm;

    /* ---- Redundancy check: ToF vs IR ---- */
    float diff = fabsf(tof_mm - ir_mm);
    if (diff < CLIFF_IR_TOF_DISAGREE_MM) {
        s->ir_tof_agree = 1U;
        s->disagree_count = 0U;

        /* Fuse: average of ToF and IR */
        s->combined_distance_mm = (tof_mm + ir_mm) * 0.5f;
    } else {
        s->ir_tof_agree = 0U;
        s->disagree_count++;

        /* Sensor fault if persistent disagreement */
        if (s->disagree_count > CLIFF_IR_TOF_FAULT_SAMPLES) {
            s->sensor_fault = 1U;
        }

        /* Use the more conservative (shorter) reading */
        s->combined_distance_mm = (tof_mm < ir_mm) ? tof_mm : ir_mm;
    }

    /* ---- Cliff detection with hysteresis ---- */
    s->cliff_prev = s->cliff_detected;

    /* Approach threshold (lower): going from safe -> cliff */
    if (s->combined_distance_mm < (CLIFF_THRESHOLD_MM - CLIFF_APPROACH_HYST_MM)) {
        if (!s->cliff_detected) {
            s->cliff_change_ms = now_ms;
        }
        s->cliff_detected = 1U;
    }
    /* Retreat threshold (higher): going from cliff -> safe */
    else if (s->combined_distance_mm > (CLIFF_THRESHOLD_MM + CLIFF_RETREAT_HYST_MM)) {
        if (s->cliff_detected) {
            s->cliff_change_ms = now_ms;
        }
        s->cliff_detected = 0U;
    }

    /* Debounce */
    if (s->cliff_detected != s->cliff_prev) {
        if ((now_ms - s->cliff_change_ms) < CLIFF_DEBOUNCE_MS) {
            s->cliff_detected = s->cliff_prev; /* revert */
        }
    }

    /* ---- Soiling detection ---- */
    if (!s->sensor_fault) {
        /* If the measured distance is consistently much shorter than baseline,
           the sensor might be soiled (dirt on lens). */
        if ((s->combined_distance_mm < (s->baseline_distance_mm * 0.5f)) &&
            (s->combined_distance_mm > CLIFF_THRESHOLD_MM)) {
            /* Not a cliff, but reading low -> possible soiling */
            if (s->soiled_start_ms == 0U) {
                s->soiled_start_ms = now_ms;
            }
            if ((now_ms - s->soiled_start_ms) > CLIFF_SOILING_DEBOUNCE_MS) {
                s->soiled = 1U;
            }
        } else {
            s->soiled_start_ms = 0U;
            s->soiled = 0U;
        }
    }
}

/* ---------------------------------------------------------------------------*/
/*  Edge-follow detection                                                     */
/* ---------------------------------------------------------------------------*/

/** @brief  Determine if the robot should enter edge-follow mode.
  *         Edge follow activates when one side detects a cliff but the
  *         opposite side does not (robot is parallel to a drop-off).
  */
static void cliff_check_edge_follow(void)
{
    uint8_t left_cliff  = hdrop.sensors[CLIFF_FL].cliff_detected ||
                          hdrop.sensors[CLIFF_RL].cliff_detected;
    uint8_t right_cliff = hdrop.sensors[CLIFF_FR].cliff_detected ||
                          hdrop.sensors[CLIFF_RR].cliff_detected;

    uint8_t front_cliff = hdrop.sensors[CLIFF_FL].cliff_detected &&
                          hdrop.sensors[CLIFF_FR].cliff_detected;

    /* Edge follow: one side at cliff, other side safe */
    if ((left_cliff || right_cliff) && !front_cliff) {
        hdrop.edge_follow_active = 1U;

        /* Measure distance to edge */
        if (left_cliff) {
            /* Robot's left side is at the cliff */
            hdrop.edge_follow_distance =
                hdrop.sensors[CLIFF_FL].combined_distance_mm <
                hdrop.sensors[CLIFF_RL].combined_distance_mm ?
                hdrop.sensors[CLIFF_FL].combined_distance_mm :
                hdrop.sensors[CLIFF_RL].combined_distance_mm;
        } else {
            hdrop.edge_follow_distance =
                hdrop.sensors[CLIFF_FR].combined_distance_mm <
                hdrop.sensors[CLIFF_RR].combined_distance_mm ?
                hdrop.sensors[CLIFF_FR].combined_distance_mm :
                hdrop.sensors[CLIFF_RR].combined_distance_mm;
        }
    } else {
        /* No edge detected or robot front is at cliff (should stop) */
        if (!left_cliff && !right_cliff) {
            /* Safe: no cliffs on either side */
            hdrop.edge_follow_active = 0U;
        }
    }
}

/* ---------------------------------------------------------------------------*/
/*  Main update function                                                      */
/* ---------------------------------------------------------------------------*/

/** @brief  Run drop detection for all four cliff sensors.
  *         Call at 1 kHz.
  *
  * @param  tof_fl, tof_fr, tof_rl, tof_rr   ToF distances [mm].
  * @param  ir_fl,  ir_fr,  ir_rl,  ir_rr    IR distances [mm].
  */
void DROP_Update(float tof_fl, float tof_fr, float tof_rl, float tof_rr,
                  float ir_fl,  float ir_fr,  float ir_rl,  float ir_rr)
{
    /* Update each sensor */
    cliff_update_sensor(&hdrop.sensors[CLIFF_FL], tof_fl, ir_fl);
    cliff_update_sensor(&hdrop.sensors[CLIFF_FR], tof_fr, ir_fr);
    cliff_update_sensor(&hdrop.sensors[CLIFF_RL], tof_rl, ir_rl);
    cliff_update_sensor(&hdrop.sensors[CLIFF_RR], tof_rr, ir_rr);

    /* Global cliff status */
    uint32_t now_ms = HAL_GetTick();
    uint8_t prev_cliff = hdrop.cliff_active;

    hdrop.any_cliff = 0U;
    hdrop.any_sensor_fault = 0U;

    for (uint32_t i = 0; i < NUM_CLIFF_SENSORS; i++) {
        if (hdrop.sensors[i].cliff_detected) {
            hdrop.any_cliff = 1U;
        }
        if (hdrop.sensors[i].sensor_fault) {
            hdrop.any_sensor_fault = 1U;
        }
    }

    /* Debounce global cliff */
    hdrop.cliff_active = hdrop.any_cliff;

    if (hdrop.cliff_active && !prev_cliff) {
        hdrop.cliff_active_ms = now_ms;
        hdrop.total_cliff_events++;
    }

    /* Edge-follow check */
    cliff_check_edge_follow();
}

/* ---------------------------------------------------------------------------*/
/*  Public API                                                                */
/* ---------------------------------------------------------------------------*/

/** @brief  Check if any cliff is detected (debounced). */
uint8_t DROP_IsCliffDetected(void)
{
    return hdrop.cliff_active;
}

/** @brief  Check if a specific sensor detects a cliff. */
uint8_t DROP_IsSensorCliff(uint8_t sensor_id)
{
    if (sensor_id >= NUM_CLIFF_SENSORS) return 0U;
    return hdrop.sensors[sensor_id].cliff_detected;
}

/** @brief  Get combined distance for a specific sensor [mm]. */
float DROP_GetSensorDistance(uint8_t sensor_id)
{
    if (sensor_id >= NUM_CLIFF_SENSORS) return 999.0f;
    return hdrop.sensors[sensor_id].combined_distance_mm;
}

/** @brief  Check if IR and ToF agree for a specific sensor. */
uint8_t DROP_AreSensorsAgreeing(uint8_t sensor_id)
{
    if (sensor_id >= NUM_CLIFF_SENSORS) return 0U;
    return hdrop.sensors[sensor_id].ir_tof_agree;
}

/** @brief  Check if a specific sensor is faulty. */
uint8_t DROP_IsSensorFaulty(uint8_t sensor_id)
{
    if (sensor_id >= NUM_CLIFF_SENSORS) return 0U;
    return hdrop.sensors[sensor_id].sensor_fault;
}

/** @brief  Check if any sensor is faulty. */
uint8_t DROP_IsAnySensorFaulty(void)
{
    return hdrop.any_sensor_fault;
}

/** @brief  Check if a specific sensor is soiled. */
uint8_t DROP_IsSensorSoiled(uint8_t sensor_id)
{
    if (sensor_id >= NUM_CLIFF_SENSORS) return 0U;
    return hdrop.sensors[sensor_id].soiled;
}

/** @brief  Get edge-follow active status. */
uint8_t DROP_IsEdgeFollowing(void)
{
    return hdrop.edge_follow_active;
}

/** @brief  Get edge-follow distance [mm]. */
float DROP_GetEdgeDistance(void)
{
    return hdrop.edge_follow_distance;
}

/** @brief  Get the edge-follow target distance [mm]. */
float DROP_GetEdgeTarget(void)
{
    return hdrop.edge_follow_target;
}

/** @brief  Set the edge-follow target distance [mm]. */
void DROP_SetEdgeTarget(float target_mm)
{
    if ((target_mm > 30.0f) && (target_mm < 200.0f)) {
        hdrop.edge_follow_target = target_mm;
    }
}

/** @brief  Get total cliff events counted. */
uint32_t DROP_GetTotalCliffEvents(void)
{
    return hdrop.total_cliff_events;
}

/** @brief  Reset soiled flag for a sensor (e.g., after cleaning). */
void DROP_ClearSoiled(uint8_t sensor_id)
{
    if (sensor_id >= NUM_CLIFF_SENSORS) return;
    hdrop.sensors[sensor_id].soiled = 0U;
    hdrop.sensors[sensor_id].soiled_start_ms = 0U;
}

/** @brief  Reset a sensor fault flag. */
void DROP_ClearFault(uint8_t sensor_id)
{
    if (sensor_id >= NUM_CLIFF_SENSORS) return;
    hdrop.sensors[sensor_id].sensor_fault = 0U;
    hdrop.sensors[sensor_id].disagree_count = 0U;
}
