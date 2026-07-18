/**
  ******************************************************************************
  * @file    safety_tilt.c
  * @author  M4 Safety Team
  * @brief   Tilt detection and recovery for the H747 Elite robot.
  *
  *          Monitors IMU pitch and roll angles.  When the tilt exceeds a
  *          speed-dependent threshold, the robot slows down or stops to
  *          prevent tipping over.
  *
  *          Features:
  *            - Dynamic tilt threshold: lower threshold at high speed
  *              (more cautious when moving fast), higher threshold at low
  *              speed (allows climbing over obstacles).
  *            - Tilt + cliff correlation: if both tilt and cliff sensors
  *              indicate instability, confidence is higher.
  *            - Tilt recovery: if tilted beyond recovery angle, execute
  *              stop -> reverse -> re-orient sequence.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include <string.h>
#include <math.h>

/* Private defines -----------------------------------------------------------*/
/* Static thresholds [deg] */
#define TILT_THRESHOLD_WARN_DEG     15.0f    /* warning tilt angle */
#define TILT_THRESHOLD_CRIT_DEG     25.0f    /* critical — stop immediately */
#define TILT_THRESHOLD_RECOVER_DEG  30.0f    /* beyond this, need recovery */
#define TILT_MAX_SAFE_DEG           40.0f    /* absolute max before tip */

/* Speed-dependent scaling */
#define TILT_SPEED_THRESH_RPM       500.0f   /* speed above which we scale down threshold */
#define TILT_SPEED_SCALE_FACTOR     0.5f     /* reduce threshold by 50% at high speed */

/* Cliff correlation */
#define TILT_CLIFF_CORR_THRESH_DEG  8.0f     /* if tilt + cliff agree > this -> high confidence */

/* Recovery */
#define TILT_RECOVERY_REVERSE_MS    500U     /* reverse duration [ms] */
#define TILT_RECOVERY_SETTLE_MS     1000U    /* settle duration [ms] */
#define TILT_RECOVERY_DEBOUNCE_MS   200U     /* debounce before declaring tilt */

/* IMU filter */
#define TILT_FILTER_ALPHA           0.92f    /* low-pass for tilt angle */

/* ---------------------------------------------------------------------------*/
/*  Tilt states                                                               */
/* ---------------------------------------------------------------------------*/
typedef enum {
    TILT_STATE_OK         = 0U,
    TILT_STATE_WARNING    = 1U,   /* tilted, reduce speed */
    TILT_STATE_CRITICAL   = 2U,   /* tilted, stop now */
    TILT_STATE_RECOVERING = 3U,   /* recovery sequence active */
    TILT_STATE_TIPPED     = 4U,   /* beyond recovery angle */
} TiltState;

/* ---------------------------------------------------------------------------*/
/*  Tilt handle                                                               */
/* ---------------------------------------------------------------------------*/
typedef struct {
    /* Raw IMU data */
    float pitch_deg;              /* filtered pitch [deg] */
    float roll_deg;               /* filtered roll [deg] */
    float tilt_magnitude_deg;     /* combined tilt = max(|pitch|, |roll|) */

    /* Thresholds */
    float threshold_warn_deg;     /* dynamic warning threshold */
    float threshold_crit_deg;     /* dynamic critical threshold */

    /* State */
    TiltState state;
    TiltState prev_state;
    uint32_t  state_change_ms;

    /* Cliff correlation */
    float     cliff_derived_tilt; /* expected tilt from cliff sensors */
    uint8_t   cliff_agreement;    /* 1 = cliff data consistent with tilt */
    float     correlation_conf;   /* confidence in tilt measurement */

    /* Recovery */
    uint32_t  recovery_start_ms;
    uint32_t  recovery_phase;     /* 0 = stop, 1 = reverse, 2 = reorient, 3 = settle */

    /* Speed context */
    float     current_speed_rpm;

    /* Filtered raw angle (before magnitude) */
    float     filtered_pitch;
    float     filtered_roll;

} TiltHandleTypeDef;

static TiltHandleTypeDef htilt;

/* ---------------------------------------------------------------------------*/
/*  Initialisation                                                            */
/* ---------------------------------------------------------------------------*/

/** @brief  Initialise tilt detection module. */
void TILT_Init(void)
{
    memset(&htilt, 0, sizeof(htilt));

    htilt.threshold_warn_deg  = TILT_THRESHOLD_WARN_DEG;
    htilt.threshold_crit_deg  = TILT_THRESHOLD_CRIT_DEG;
    htilt.state               = TILT_STATE_OK;
    htilt.filtered_pitch      = 0.0f;
    htilt.filtered_roll       = 0.0f;
    htilt.correlation_conf    = 1.0f; /* start with full confidence */
}

/* ---------------------------------------------------------------------------*/
/*  Dynamic threshold calculation                                             */
/* ---------------------------------------------------------------------------*/

/** @brief  Calculate speed-dependent tilt threshold.
  *         At high speed, we lower the threshold to avoid tipping during
  *         turns.  At low speed, we allow more tilt for obstacle climbing.
  */
static void tilt_update_thresholds(void)
{
    float speed_factor = 1.0f;
    float abs_speed = fabsf(htilt.current_speed_rpm);

    if (abs_speed > TILT_SPEED_THRESH_RPM) {
        float excess = (abs_speed - TILT_SPEED_THRESH_RPM) / TILT_SPEED_THRESH_RPM;
        speed_factor = 1.0f - excess * (1.0f - TILT_SPEED_SCALE_FACTOR);
        if (speed_factor < TILT_SPEED_SCALE_FACTOR) speed_factor = TILT_SPEED_SCALE_FACTOR;
    }

    htilt.threshold_warn_deg = TILT_THRESHOLD_WARN_DEG * speed_factor;
    htilt.threshold_crit_deg = TILT_THRESHOLD_CRIT_DEG * speed_factor;

    /* Always ensure at least minimum threshold */
    if (htilt.threshold_warn_deg < 5.0f) htilt.threshold_warn_deg = 5.0f;
    if (htilt.threshold_crit_deg < 10.0f) htilt.threshold_crit_deg = 10.0f;
}

/* ---------------------------------------------------------------------------*/
/*  Cliff correlation                                                         */
/* ---------------------------------------------------------------------------*/

/** @brief  Correlate tilt measurement with cliff sensor data.
  *         If the cliff sensors show a floor distance gradient consistent with
  *         the measured tilt, confidence increases.  If they disagree,
  *         confidence decreases (possible sensor fault).
  *
  * @param  cliff_tilt_deg  Expected tilt derived from cliff distance deltas.
  */
static void tilt_correlate_with_cliff(float cliff_tilt_deg)
{
    float diff = fabsf(htilt.tilt_magnitude_deg - cliff_tilt_deg);

    if (diff < TILT_CLIFF_CORR_THRESH_DEG) {
        /* Agreement: increase confidence */
        htilt.correlation_conf += 0.1f;
        htilt.cliff_agreement = 1U;
    } else {
        /* Disagreement: decrease confidence */
        htilt.correlation_conf -= 0.2f;
        htilt.cliff_agreement = 0U;
    }

    /* Clamp confidence */
    if (htilt.correlation_conf < 0.0f) htilt.correlation_conf = 0.0f;
    if (htilt.correlation_conf > 1.0f) htilt.correlation_conf = 1.0f;
}

/* ---------------------------------------------------------------------------*/
/*  Recovery sequence                                                         */
/* ---------------------------------------------------------------------------*/

/** @brief  Run tilt recovery state machine.
  *         Phases: STOP -> REVERSE -> REORIENT -> SETTLE -> OK.
  */
static void tilt_recovery_sequence(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint32_t elapsed = now_ms - htilt.recovery_start_ms;

    switch (htilt.recovery_phase) {
        case 0: /* STOP — already stopped by critical tilt */
            htilt.recovery_phase = 1U;
            htilt.recovery_start_ms = now_ms;
            /* FALLTHROUGH */

        case 1: /* REVERSE — back away from hazard */
            if (elapsed > TILT_RECOVERY_REVERSE_MS) {
                htilt.recovery_phase = 2U;
                htilt.recovery_start_ms = now_ms;
            }
            break;

        case 2: /* REORIENT — turn to safe direction */
            if (elapsed > TILT_RECOVERY_REVERSE_MS) {
                htilt.recovery_phase = 3U;
                htilt.recovery_start_ms = now_ms;
            }
            break;

        case 3: /* SETTLE — wait for stability */
            if (elapsed > TILT_RECOVERY_SETTLE_MS) {
                /* Check if we're now within safe tilt */
                if (htilt.tilt_magnitude_deg < TILT_THRESHOLD_WARN_DEG) {
                    htilt.state = TILT_STATE_OK;
                    htilt.recovery_phase = 0U;
                } else {
                    /* Still tilted: loop back to reverse */
                    htilt.recovery_phase = 1U;
                    htilt.recovery_start_ms = now_ms;
                }
            }
            break;

        default:
            break;
    }
}

/* ---------------------------------------------------------------------------*/
/*  Main update                                                               */
/* ---------------------------------------------------------------------------*/

/** @brief  Run tilt detection.
  *         Call at 1 kHz.
  *
  * @param  pitch_deg   IMU pitch angle [deg] (positive = forward tilt).
  * @param  roll_deg    IMU roll angle [deg] (positive = right tilt).
  * @param  speed_rpm   Current robot speed [RPM] (from wheel encoder).
  * @param  cliff_tilt_deg  Expected tilt from cliff sensors [deg].
  *                         Set to tilt_magnitude if not available.
  */
void TILT_Update(float pitch_deg, float roll_deg,
                  float speed_rpm, float cliff_tilt_deg)
{
    /* Low-pass filter raw angles */
    htilt.filtered_pitch = TILT_FILTER_ALPHA * htilt.filtered_pitch
                         + (1.0f - TILT_FILTER_ALPHA) * pitch_deg;
    htilt.filtered_roll  = TILT_FILTER_ALPHA * htilt.filtered_roll
                         + (1.0f - TILT_FILTER_ALPHA) * roll_deg;

    htilt.pitch_deg = htilt.filtered_pitch;
    htilt.roll_deg  = htilt.filtered_roll;
    htilt.current_speed_rpm = speed_rpm;

    /* Combined tilt magnitude */
    htilt.tilt_magnitude_deg = fmaxf(fabsf(htilt.pitch_deg), fabsf(htilt.roll_deg));

    /* Update dynamic thresholds */
    tilt_update_thresholds();

    /* Cliff correlation */
    tilt_correlate_with_cliff(cliff_tilt_deg);

    /* State machine */
    uint32_t now_ms = HAL_GetTick();
    htilt.prev_state = htilt.state;

    switch (htilt.state) {
        case TILT_STATE_OK:
            if (htilt.tilt_magnitude_deg > htilt.threshold_crit_deg) {
                if ((now_ms - htilt.state_change_ms) > TILT_RECOVERY_DEBOUNCE_MS) {
                    htilt.state = TILT_STATE_CRITICAL;
                    htilt.state_change_ms = now_ms;
                }
            } else if (htilt.tilt_magnitude_deg > htilt.threshold_warn_deg) {
                if ((now_ms - htilt.state_change_ms) > TILT_RECOVERY_DEBOUNCE_MS) {
                    htilt.state = TILT_STATE_WARNING;
                    htilt.state_change_ms = now_ms;
                }
            }
            break;

        case TILT_STATE_WARNING:
            if (htilt.tilt_magnitude_deg < (htilt.threshold_warn_deg * 0.8f)) {
                htilt.state = TILT_STATE_OK;
                htilt.state_change_ms = now_ms;
            } else if (htilt.tilt_magnitude_deg > htilt.threshold_crit_deg) {
                htilt.state = TILT_STATE_CRITICAL;
                htilt.state_change_ms = now_ms;
            }
            break;

        case TILT_STATE_CRITICAL:
            if (htilt.tilt_magnitude_deg < TILT_THRESHOLD_RECOVER_DEG) {
                /* Recovery possible */
                htilt.state = TILT_STATE_RECOVERING;
                htilt.recovery_start_ms = now_ms;
                htilt.recovery_phase = 0U;
                htilt.state_change_ms = now_ms;
            } else if (htilt.tilt_magnitude_deg > TILT_MAX_SAFE_DEG) {
                htilt.state = TILT_STATE_TIPPED;
                htilt.state_change_ms = now_ms;
            }
            break;

        case TILT_STATE_RECOVERING:
            tilt_recovery_sequence();
            break;

        case TILT_STATE_TIPPED:
            /* Robot has tipped over — requires manual intervention */
            break;

        default:
            break;
    }
}

/* ---------------------------------------------------------------------------*/
/*  Public API                                                                */
/* ---------------------------------------------------------------------------*/

/** @brief  Get current tilt state. */
TiltState TILT_GetState(void)
{
    return htilt.state;
}

/** @brief  Get tilt state as human-readable string. */
const char* TILT_GetStateStr(void)
{
    switch (htilt.state) {
        case TILT_STATE_OK:         return "OK";
        case TILT_STATE_WARNING:    return "WARNING";
        case TILT_STATE_CRITICAL:   return "CRITICAL";
        case TILT_STATE_RECOVERING: return "RECOVERING";
        case TILT_STATE_TIPPED:     return "TIPPED";
        default:                    return "UNKNOWN";
    }
}

/** @brief  Get tilt magnitude [deg]. */
float TILT_GetMagnitude(void)
{
    return htilt.tilt_magnitude_deg;
}

/** @brief  Get filtered pitch [deg]. */
float TILT_GetPitch(void) { return htilt.pitch_deg; }

/** @brief  Get filtered roll [deg]. */
float TILT_GetRoll(void) { return htilt.roll_deg; }

/** @brief  Get dynamic warning threshold [deg]. */
float TILT_GetWarnThreshold(void) { return htilt.threshold_warn_deg; }

/** @brief  Get dynamic critical threshold [deg]. */
float TILT_GetCritThreshold(void) { return htilt.threshold_crit_deg; }

/** @brief  Get cliff agreement status. */
uint8_t TILT_GetCliffAgreement(void) { return htilt.cliff_agreement; }

/** @brief  Get correlation confidence (0.0 to 1.0). */
float TILT_GetConfidence(void) { return htilt.correlation_conf; }

/** @brief  Get recovery phase (0=stop,1=reverse,2=reorient,3=settle). */
uint32_t TILT_GetRecoveryPhase(void) { return htilt.recovery_phase; }

/** @brief  Check if tilt recovery is active. */
uint8_t TILT_IsRecovering(void)
{
    return (htilt.state == TILT_STATE_RECOVERING) ? 1U : 0U;
}

/** @brief  Check if robot has tipped over. */
uint8_t TILT_IsTipped(void)
{
    return (htilt.state == TILT_STATE_TIPPED) ? 1U : 0U;
}

/** @brief  Reset tilt state (after manual recovery). */
void TILT_Reset(void)
{
    htilt.state = TILT_STATE_OK;
    htilt.state_change_ms = HAL_GetTick();
    htilt.recovery_phase = 0U;
    htilt.correlation_conf = 1.0f;
}
