/**
  ******************************************************************************
  * @file    safety_collision.c
  * @author  M4 Safety Team
  * @brief   Collision detection and response for the H747 Elite robot.
  *
  *          Uses three redundant detection methods:
  *            1. IMU jerk detection (> 3 g in 10 ms)
  *            2. Motor current spike (> 200% in 50 ms)
  *            3. Bumper switch (mechanical contact)
  *
  *          Any one of the three can trigger collision detection.  The
  *          response is:
  *            1. Immediate stop (PWM disable)
  *            2. Retreat 10 cm (reverse)
  *            3. Turn 30 degrees (avoidance)
  *            4. Resume normal operation
  *
  *          Bumper switch serves as the backup / confirmation signal.
  *          If IMU jerk + current spike agree but bumper is not triggered,
  *          it suggests a soft collision (e.g., bumping into a curtain).
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include <string.h>
#include <math.h>

/* Private defines -----------------------------------------------------------*/
/* Jerk detection */
#define COLLISION_JERK_THRESH_G    3.0f       /* 3 g */
#define COLLISION_JERK_WINDOW_MS   10U        /* within 10 ms */
#define COLLISION_JERK_SAMPLES     3U         /* consecutive samples */

/* Current spike */
#define COLLISION_CURRENT_THRESH_PU 2.0f      /* 200% of rated */
#define COLLISION_CURRENT_WINDOW_MS 50U
#define COLLISION_CURRENT_SAMPLES  5U

/* Bumper */
#define COLLISION_BUMPER_DEBOUNCE_MS 20U

/* Response sequence */
#define COLLISION_RETREAT_MM       100U       /* retreat 10 cm */
#define COLLISION_RETREAT_SPEED    0.3f       /* speed factor of max */
#define COLLISION_TURN_DEG         30.0f      /* turn 30 degrees */
#define COLLISION_TURN_SPEED       0.2f       /* turn speed factor */
#define COLLISION_RESUME_DELAY_MS  300U       /* pause before resume */

/* Collision state machine */
typedef enum {
    COLL_STATE_OK        = 0U,
    COLL_STATE_DETECTED  = 1U,   /* collision just detected */
    COLL_STATE_STOPPED   = 2U,   /* motors disabled */
    COLL_STATE_RETREAT   = 3U,   /* reversing */
    COLL_STATE_TURN      = 4U,   /* turning away */
    COLL_STATE_RESUME    = 5U,   /* resuming normal operation */
} CollisionState;

/* ---------------------------------------------------------------------------*/
/*  Collision handle                                                          */
/* ---------------------------------------------------------------------------*/
typedef struct {
    /* State */
    CollisionState state;
    CollisionState prev_state;
    uint32_t       state_start_ms;

    /* Detection methods */
    uint8_t  jerk_triggered;         /* IMU jerk detected */
    uint8_t  current_triggered;      /* current spike detected */
    uint8_t  bumper_triggered;       /* bumper switch activated */
    uint8_t  any_triggered;

    /* IMU jerk tracking */
    float    prev_accel[3];          /* previous accel sample (x, y, z) [g] */
    float    jerk_magnitude;         /* max jerk in window [g/s] */
    uint32_t jerk_count;
    uint32_t jerk_window_start;

    /* Current spike tracking */
    float    current_baseline;       /* running average current [A] */
    float    current_instant;        /* instantaneous current [A] */
    uint32_t current_count;
    uint32_t current_window_start;

    /* Bumper */
    uint8_t  bumper_state;           /* raw bumper state (1 = pressed) */
    uint8_t  bumper_debounced;
    uint32_t bumper_change_ms;

    /* Response */
    uint32_t retreat_start_ms;
    uint32_t turn_start_ms;
    float    retreat_distance_mm;    /* accumulated retreat distance */
    float    turn_angle_deg;         /* accumulated turn angle */

    /* Statistics */
    uint32_t total_collisions;
    uint32_t total_bumper_events;

} CollisionHandleTypeDef;

static CollisionHandleTypeDef hcoll;

/* ---------------------------------------------------------------------------*/
/*  Initialisation                                                            */
/* ---------------------------------------------------------------------------*/

/** @brief  Initialise collision detection module. */
void COLL_Init(void)
{
    memset(&hcoll, 0, sizeof(hcoll));
    hcoll.state = COLL_STATE_OK;
    hcoll.current_baseline = 0.5f; /* assume 0.5 A no-load */
}

/* ---------------------------------------------------------------------------*/
/*  Detection methods                                                         */
/* ---------------------------------------------------------------------------*/

/** @brief  Detect collision from IMU jerk.
  *         Jerk = derivative of acceleration.  A sudden impact produces high jerk.
  *
  * @param  accel_x, accel_y, accel_z  Current acceleration [g].
  */
static void detect_jerk(float accel_x, float accel_y, float accel_z)
{
    uint32_t now_ms = HAL_GetTick();

    /* Compute jerk as delta acceleration */
    float dj_x = fabsf(accel_x - hcoll.prev_accel[0]);
    float dj_y = fabsf(accel_y - hcoll.prev_accel[1]);
    float dj_z = fabsf(accel_z - hcoll.prev_accel[2]);
    float jerk = dj_x + dj_y + dj_z; /* sum of absolute deltas */

    hcoll.jerk_magnitude = jerk;
    hcoll.prev_accel[0] = accel_x;
    hcoll.prev_accel[1] = accel_y;
    hcoll.prev_accel[2] = accel_z;

    /* Reset window if too old */
    if ((now_ms - hcoll.jerk_window_start) > COLLISION_JERK_WINDOW_MS) {
        hcoll.jerk_count = 0U;
        hcoll.jerk_window_start = now_ms;
    }

    if (jerk > COLLISION_JERK_THRESH_G) {
        hcoll.jerk_count++;
        if (hcoll.jerk_count >= COLLISION_JERK_SAMPLES) {
            hcoll.jerk_triggered = 1U;
        }
    }
}

/** @brief  Detect collision from motor current spike.
  *         A sudden impact causes the wheels to decelerate, increasing current.
  *
  * @param  motor_current_a  Total motor current [A].
  */
static void detect_current_spike(float motor_current_a)
{
    uint32_t now_ms = HAL_GetTick();

    /* Running baseline of motor current */
    const float alpha = 0.98f;
    hcoll.current_baseline = alpha * hcoll.current_baseline
                           + (1.0f - alpha) * fabsf(motor_current_a);
    hcoll.current_instant = fabsf(motor_current_a);

    /* Reset window if too old */
    if ((now_ms - hcoll.current_window_start) > COLLISION_CURRENT_WINDOW_MS) {
        hcoll.current_count = 0U;
        hcoll.current_window_start = now_ms;
    }

    /* Check for spike */
    if ((hcoll.current_baseline > 0.1f) &&
        (hcoll.current_instant > (hcoll.current_baseline * COLLISION_CURRENT_THRESH_PU)))
    {
        hcoll.current_count++;
        if (hcoll.current_count >= COLLISION_CURRENT_SAMPLES) {
            hcoll.current_triggered = 1U;
        }
    }
}

/** @brief  Detect bumper switch press with debounce.
  *
  * @param  bumper_pressed  Raw bumper switch state (1 = pressed).
  */
static void detect_bumper(uint8_t bumper_pressed)
{
    uint32_t now_ms = HAL_GetTick();

    hcoll.bumper_state = bumper_pressed;

    if (bumper_pressed != hcoll.bumper_debounced) {
        if (hcoll.bumper_change_ms == 0U) {
            hcoll.bumper_change_ms = now_ms;
        }
        if ((now_ms - hcoll.bumper_change_ms) > COLLISION_BUMPER_DEBOUNCE_MS) {
            hcoll.bumper_debounced = bumper_pressed;
            hcoll.bumper_change_ms = 0U;
        }
    } else {
        hcoll.bumper_change_ms = 0U;
    }

    if (hcoll.bumper_debounced) {
        hcoll.bumper_triggered = 1U;
        hcoll.total_bumper_events++;
    }
}

/* ---------------------------------------------------------------------------*/
/*  Collision response sequence                                               */
/* ---------------------------------------------------------------------------*/

/** @brief  Execute the collision response state machine.
  *         STOP -> RETREAT -> TURN -> RESUME -> OK.
  */
static void collision_response_sequence(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint32_t elapsed = now_ms - hcoll.state_start_ms;

    switch (hcoll.state) {
        case COLL_STATE_DETECTED:
            /* Transition to STOPPED — motors will be disabled externally */
            hcoll.state = COLL_STATE_STOPPED;
            hcoll.state_start_ms = now_ms;
            break;

        case COLL_STATE_STOPPED:
            /* Wait 100 ms, then begin retreat */
            if (elapsed > 100U) {
                hcoll.state = COLL_STATE_RETREAT;
                hcoll.state_start_ms = now_ms;
                hcoll.retreat_start_ms = now_ms;
                hcoll.retreat_distance_mm = 0.0f;
            }
            break;

        case COLL_STATE_RETREAT:
            /* Retreat at 30% speed for ~300 ms (approx 10 cm) */
            if (elapsed > 300U) {
                hcoll.state = COLL_STATE_TURN;
                hcoll.state_start_ms = now_ms;
                hcoll.turn_start_ms = now_ms;
                hcoll.turn_angle_deg = 0.0f;
            }
            break;

        case COLL_STATE_TURN:
            /* Turn 30 degrees at 20% speed for ~500 ms */
            if (elapsed > 500U) {
                hcoll.state = COLL_STATE_RESUME;
                hcoll.state_start_ms = now_ms;
            }
            break;

        case COLL_STATE_RESUME:
            /* Pause then resume */
            if (elapsed > COLLISION_RESUME_DELAY_MS) {
                hcoll.state = COLL_STATE_OK;
                hcoll.state_start_ms = now_ms;
                /* Clear all trigger flags */
                hcoll.jerk_triggered = 0U;
                hcoll.current_triggered = 0U;
                hcoll.bumper_triggered = 0U;
                hcoll.any_triggered = 0U;
            }
            break;

        default:
            break;
    }
}

/* ---------------------------------------------------------------------------*/
/*  Main update function                                                      */
/* ---------------------------------------------------------------------------*/

/** @brief  Run collision detection and response.
  *         Call at 1 kHz.
  *
  * @param  accel_g[3]       IMU acceleration [g] (x, y, z).
  * @param  motor_current_a  Total motor current [A].
  * @param  bumper_1         Bumper switch 1 (front-left).
  * @param  bumper_2         Bumper switch 2 (front-right).
  */
void COLL_Update(const float accel_g[3], float motor_current_a,
                  uint8_t bumper_1, uint8_t bumper_2)
{
    uint32_t now_ms = HAL_GetTick();

    /* ---- Detection ---- */
    detect_jerk(accel_g[0], accel_g[1], accel_g[2]);
    detect_current_spike(motor_current_a);

    /* Bumper detection (OR of both bumpers) */
    detect_bumper(bumper_1 || bumper_2);

    /* ---- Trigger logic (any one triggers) ---- */
    if (hcoll.jerk_triggered || hcoll.current_triggered || hcoll.bumper_triggered) {
        if (!hcoll.any_triggered) {
            hcoll.total_collisions++;
        }
        hcoll.any_triggered = 1U;
    }

    /* ---- State machine ---- */
    hcoll.prev_state = hcoll.state;

    if (hcoll.any_triggered && (hcoll.state == COLL_STATE_OK)) {
        hcoll.state = COLL_STATE_DETECTED;
        hcoll.state_start_ms = now_ms;
    }

    if (hcoll.state > COLL_STATE_OK) {
        collision_response_sequence();
    }
}

/* ---------------------------------------------------------------------------*/
/*  Public API                                                                */
/* ---------------------------------------------------------------------------*/

/** @brief  Get current collision state. */
CollisionState COLL_GetState(void)
{
    return hcoll.state;
}

/** @brief  Get collision state as string. */
const char* COLL_GetStateStr(void)
{
    switch (hcoll.state) {
        case COLL_STATE_OK:       return "OK";
        case COLL_STATE_DETECTED: return "DETECTED";
        case COLL_STATE_STOPPED:  return "STOPPED";
        case COLL_STATE_RETREAT:  return "RETREAT";
        case COLL_STATE_TURN:     return "TURN";
        case COLL_STATE_RESUME:   return "RESUME";
        default:                  return "UNKNOWN";
    }
}

/** @brief  Check if collision is currently active (any state except OK). */
uint8_t COLL_IsActive(void)
{
    return (hcoll.state != COLL_STATE_OK) ? 1U : 0U;
}

/** @brief  Check if collision was detected by a specific method. */
uint8_t COLL_WasJerkTriggered(void)    { return hcoll.jerk_triggered; }
uint8_t COLL_WasCurrentTriggered(void) { return hcoll.current_triggered; }
uint8_t COLL_WasBumperTriggered(void)  { return hcoll.bumper_triggered; }

/** @brief  Get the jerk magnitude from the last detection [g]. */
float COLL_GetJerkMagnitude(void) { return hcoll.jerk_magnitude; }

/** @brief  Get the current spike magnitude [A]. */
float COLL_GetCurrentSpike(void)
{
    return hcoll.current_instant;
}

/** @brief  Get the current baseline [A]. */
float COLL_GetCurrentBaseline(void)
{
    return hcoll.current_baseline;
}

/** @brief  Get total collision count. */
uint32_t COLL_GetTotalCollisions(void) { return hcoll.total_collisions; }

/** @brief  Get total bumper events. */
uint32_t COLL_GetTotalBumperEvents(void) { return hcoll.total_bumper_events; }

/** @brief  Get debounced bumper state. */
uint8_t COLL_GetBumperState(void) { return hcoll.bumper_debounced; }

/** @brief  Manually clear collision state (e.g., after operator override). */
void COLL_Clear(void)
{
    hcoll.state = COLL_STATE_OK;
    hcoll.state_start_ms = HAL_GetTick();
    hcoll.jerk_triggered = 0U;
    hcoll.current_triggered = 0U;
    hcoll.bumper_triggered = 0U;
    hcoll.any_triggered = 0U;
    hcoll.jerk_count = 0U;
    hcoll.current_count = 0U;
}
