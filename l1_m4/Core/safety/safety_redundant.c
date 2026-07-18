/**
  ******************************************************************************
  * @file    safety_redundant.c
  * @author  M4 Safety Team
  * @brief   Redundant sensor cross-check and decision matrix.
  *
  *          Implements N-version sensor fusion for safety-critical inputs:
  *            - IMU tilt vs ToF cliff correlation (both should agree on "tilted")
  *            - Motor current vs wheel encoder speed consistency
  *            - Battery voltage vs BMS reported voltage
  *
  *          Redundancy decision matrix:
  *            Each sensor pair produces a "disagreement" metric.
  *            If any 2-of-3 pairs disagree beyond threshold, the system enters
  *            DEGRADED mode (reduce speed, increase monitoring).
  *            If all 3 disagree, enter SAFE STOP mode.
  *
  *          Runs on the M4 core at 1 kHz, independent of the M7 FOC loop.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include <string.h>
#include <math.h>

/* Private defines -----------------------------------------------------------*/
#define NUM_SENSOR_PAIRS        3U

/* Disagreement thresholds */
#define TILT_CLIFF_DISAGREE_THRESH  5.0f    /* tilt [deg] vs cliff-predicted tilt */
#define CURRENT_SPEED_DISAGREE_THRESH 0.30f /* 30% deviation */
#define VOLTAGE_DISAGREE_THRESH_V   1.0f    /* 1 V difference */

/* Debounce */
#define DISAGREE_DEBOUNCE_MS     50U
#define DEGRADED_DEBOUNCE_MS     200U
#define SAFESTOP_DEBOUNCE_MS     100U

/* ---------------------------------------------------------------------------*/
/*  Sensor pair structure                                                     */
/* ---------------------------------------------------------------------------*/
typedef struct {
    const char *name_a;       /* sensor A name */
    const char *name_b;       /* sensor B name */
    float value_a;            /* latest reading from sensor A */
    float value_b;            /* latest reading from sensor B */
    float disagreement;       /* normalised disagreement metric */
    float threshold;          /* threshold above which disagreement is flagged */
    uint8_t flagged;          /* 1 = disagreement detected */
    uint8_t prev_flagged;
    uint32_t flagged_start_ms;
} SensorPair;

/* ---------------------------------------------------------------------------*/
/*  Redundancy decision matrix                                                */
/* ---------------------------------------------------------------------------*/
typedef struct {
    SensorPair pairs[NUM_SENSOR_PAIRS];

    /* System-level status */
    uint8_t  degraded_mode;           /* 1 = degraded (reduce speed) */
    uint8_t  safe_stop;               /* 1 = safe stop required */
    uint8_t  any_disagreement;        /* 1 = at least one pair disagrees */

    uint32_t degraded_start_ms;
    uint32_t num_disagree_pairs;

    /* Cross-check counters */
    uint32_t total_checks;
    uint32_t total_disagreements;
} RedundancyHandleTypeDef;

static RedundancyHandleTypeDef hred;

/* ---------------------------------------------------------------------------*/
/*  Initialisation                                                            */
/* ---------------------------------------------------------------------------*/

/** @brief  Initialise redundancy cross-check module. */
void RED_Init(void)
{
    memset(&hred, 0, sizeof(hred));

    /* Pair 0: IMU tilt vs ToF cliff */
    hred.pairs[0].name_a    = "IMU_TILT";
    hred.pairs[0].name_b    = "TOF_CLIFF";
    hred.pairs[0].threshold = TILT_CLIFF_DISAGREE_THRESH;

    /* Pair 1: Motor current vs encoder speed */
    hred.pairs[1].name_a    = "MOTOR_CURRENT";
    hred.pairs[1].name_b    = "ENC_SPEED";
    hred.pairs[1].threshold = CURRENT_SPEED_DISAGREE_THRESH;

    /* Pair 2: Battery voltage vs BMS voltage */
    hred.pairs[2].name_a    = "BATTERY_VOLTAGE";
    hred.pairs[2].name_b    = "BMS_VOLTAGE";
    hred.pairs[2].threshold = VOLTAGE_DISAGREE_THRESH_V;

    hred.degraded_mode      = 0U;
    hred.safe_stop          = 0U;
    hred.total_checks       = 0U;
    hred.total_disagreements = 0U;
}

/* ---------------------------------------------------------------------------*/
/*  Per-pair agreement checks                                                 */
/* ---------------------------------------------------------------------------*/

/** @brief  Check IMU tilt vs ToF cliff agreement.
  *
  *         ToF cliff sensors can estimate expected tilt from floor distance
  *         differences.  If IMU says tilted but ToF says level (or vice versa),
  *         there is a disagreement.
  *
  *         For simplicity, disagreement = |tilt_angle - cliff_derived_tilt|.
  */
static void check_tilt_cliff_pair(SensorPair *p)
{
    /* p->value_a = IMU pitch angle [deg] (abs)
       p->value_b = ToF cliff derived tilt [deg] (abs) */
    float diff = fabsf(p->value_a - p->value_b);
    p->disagreement = diff;

    if (diff > p->threshold) {
        if (!p->flagged) {
            p->flagged_start_ms = HAL_GetTick();
        }
        p->prev_flagged = p->flagged;
        p->flagged = 1U;

        /* Debounce */
        if ((HAL_GetTick() - p->flagged_start_ms) < DISAGREE_DEBOUNCE_MS) {
            p->flagged = 0U;
        }
    } else {
        p->flagged = 0U;
    }
}

/** @brief  Check motor current vs encoder speed consistency.
  *
  *         Current should correlate with acceleration (via F = ma).
  *         Disagreement = |actual_current - expected_current| / expected_current
  *         Or simpler: if current is high but speed is near zero (stall without
  *         detection), flag disagreement.
  */
static void check_current_speed_pair(SensorPair *p)
{
    /* p->value_a = motor current magnitude [A]
       p->value_b = motor speed magnitude [RPM] */
    float current = fabsf(p->value_a);
    float speed   = fabsf(p->value_b);

    /* Expected current at given speed (simplified linear model: I = k * speed + I0) */
    float k = 0.002f;    /* 2 mA per RPM (very approximate) */
    float I0 = 0.5f;     /* no-load current 0.5 A */
    float expected_current = k * speed + I0;

    float diff;
    if (expected_current > 0.1f) {
        diff = fabsf(current - expected_current) / expected_current;
    } else {
        diff = current; /* at zero speed, any current is unexpected */
    }

    p->disagreement = diff;

    if ((diff > p->threshold) && (current > 1.0f)) {
        if (!p->flagged) {
            p->flagged_start_ms = HAL_GetTick();
        }
        p->prev_flagged = p->flagged;
        p->flagged = 1U;

        if ((HAL_GetTick() - p->flagged_start_ms) < DISAGREE_DEBOUNCE_MS) {
            p->flagged = 0U;
        }
    } else {
        p->flagged = 0U;
    }
}

/** @brief  Check battery voltage vs BMS reported voltage agreement. */
static void check_voltage_pair(SensorPair *p)
{
    /* p->value_a = battery voltage from ADC [V]
       p->value_b = BMS reported voltage [V] */
    float diff = fabsf(p->value_a - p->value_b);
    p->disagreement = diff;

    if (diff > p->threshold) {
        if (!p->flagged) {
            p->flagged_start_ms = HAL_GetTick();
        }
        p->prev_flagged = p->flagged;
        p->flagged = 1U;

        if ((HAL_GetTick() - p->flagged_start_ms) < DISAGREE_DEBOUNCE_MS) {
            p->flagged = 0U;
        }
    } else {
        p->flagged = 0U;
    }
}

/* ---------------------------------------------------------------------------*/
/*  Decision matrix                                                           */
/* ---------------------------------------------------------------------------*/

/** @brief  Evaluate the redundancy decision matrix.
  *
  *         0 pairs disagree: NORMAL operation.
  *         1 pair disagrees: MONITOR (log, no action).
  *         2 pairs disagree: DEGRADED mode (reduce speed, increase monitoring).
  *         3 pairs disagree: SAFE STOP (emergency stop).
  */
static void evaluate_decision_matrix(void)
{
    uint32_t count = 0U;
    for (uint32_t i = 0; i < NUM_SENSOR_PAIRS; i++) {
        if (hred.pairs[i].flagged) count++;
    }
    hred.num_disagree_pairs = count;
    hred.any_disagreement = (count > 0U) ? 1U : 0U;

    uint32_t now_ms = HAL_GetTick();

    switch (count) {
        case 0U:
            /* Normal - recover from degraded mode if stable */
            if (hred.degraded_mode) {
                /* Only recover if stable for 1 second */
                static uint32_t recovery_start = 0U;
                if (recovery_start == 0U) recovery_start = now_ms;
                if ((now_ms - recovery_start) > 1000U) {
                    hred.degraded_mode = 0U;
                    recovery_start = 0U;
                }
            }
            hred.safe_stop = 0U;
            break;

        case 1U:
            /* Monitor only */
            break;

        case 2U:
            /* Degraded mode */
            if (!hred.degraded_mode) {
                hred.degraded_start_ms = now_ms;
            }
            if ((now_ms - hred.degraded_start_ms) > DEGRADED_DEBOUNCE_MS) {
                hred.degraded_mode = 1U;
            }
            break;

        default: /* 3 or more = safe stop */
            if ((now_ms - hred.degraded_start_ms) > SAFESTOP_DEBOUNCE_MS) {
                hred.safe_stop = 1U;
                hred.degraded_mode = 1U;
            }
            break;
    }
}

/* ---------------------------------------------------------------------------*/
/*  Main update function                                                      */
/* ---------------------------------------------------------------------------*/

/** @brief  Run redundancy cross-check for all sensor pairs.
  *         Call at 1 kHz.
  *
  * @param  imu_tilt_deg     IMU pitch/roll absolute tilt [deg].
  * @param  cliff_tilt_deg   ToF-cliff-derived expected tilt [deg].
  * @param  motor_current_a  Motor current magnitude [A].
  * @param  motor_speed_rpm  Motor speed magnitude [RPM].
  * @param  battery_v        Battery voltage from ADC [V].
  * @param  bms_v            BMS-reported voltage [V].
  */
void RED_Update(float imu_tilt_deg, float cliff_tilt_deg,
                float motor_current_a, float motor_speed_rpm,
                float battery_v, float bms_v)
{
    /* Update sensor values */
    hred.pairs[0].value_a = imu_tilt_deg;
    hred.pairs[0].value_b = cliff_tilt_deg;

    hred.pairs[1].value_a = motor_current_a;
    hred.pairs[1].value_b = motor_speed_rpm;

    hred.pairs[2].value_a = battery_v;
    hred.pairs[2].value_b = bms_v;

    /* Run each pair check */
    check_tilt_cliff_pair(&hred.pairs[0]);
    check_current_speed_pair(&hred.pairs[1]);
    check_voltage_pair(&hred.pairs[2]);

    /* Evaluate decision matrix */
    evaluate_decision_matrix();

    hred.total_checks++;
    if (hred.any_disagreement) {
        hred.total_disagreements++;
    }
}

/* ---------------------------------------------------------------------------*/
/*  Public API                                                                */
/* ---------------------------------------------------------------------------*/

/** @brief  Get degraded mode status. */
uint8_t RED_IsDegraded(void)
{
    return hred.degraded_mode;
}

/** @brief  Get safe stop status. */
uint8_t RED_IsSafeStop(void)
{
    return hred.safe_stop;
}

/** @brief  Get number of currently disagreeing pairs. */
uint32_t RED_GetDisagreeCount(void)
{
    return hred.num_disagree_pairs;
}

/** @brief  Get total checks performed. */
uint32_t RED_GetTotalChecks(void)
{
    return hred.total_checks;
}

/** @brief  Get total disagreements detected. */
uint32_t RED_GetTotalDisagreements(void)
{
    return hred.total_disagreements;
}

/** @brief  Get disagreement metric for a specific pair (0, 1, 2). */
float RED_GetDisagreementMetric(uint32_t pair_idx)
{
    if (pair_idx >= NUM_SENSOR_PAIRS) return 0.0f;
    return hred.pairs[pair_idx].disagreement;
}

/** @brief  Get flagged status for a specific pair. */
uint8_t RED_IsPairFlagged(uint32_t pair_idx)
{
    if (pair_idx >= NUM_SENSOR_PAIRS) return 0U;
    return hred.pairs[pair_idx].flagged;
}

/** @brief  Get pair name as string. */
const char* RED_GetPairName(uint32_t pair_idx)
{
    if (pair_idx >= NUM_SENSOR_PAIRS) return "INVALID";
    return hred.pairs[pair_idx].name_a;
}

/** @brief  Clear safe stop condition (manual override). */
void RED_ClearSafeStop(void)
{
    hred.safe_stop = 0U;
    hred.degraded_mode = 0U;
    for (uint32_t i = 0; i < NUM_SENSOR_PAIRS; i++) {
        hred.pairs[i].flagged = 0U;
    }
}
