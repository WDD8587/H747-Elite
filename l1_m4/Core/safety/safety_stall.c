/**
  ******************************************************************************
  * @file    safety_stall.c
  * @author  M4 Safety Team
  * @brief   Motor stall detection and classification for the H747 Elite robot.
  *
  *          Detects three types of stall conditions:
  *            1. HARD STALL (blocked) — high current, zero speed, abrupt onset.
  *               Recovery: immediate stop, brief reverse, retry.
  *            2. SOFT STALL (carpet tangle) — high current, very low speed,
  *               gradual onset.  Recovery: speed reduction, reverse pulse,
  *               self-clean cycle.
  *            3. WHEEL SLIP (speed mismatch L/R) — one wheel current low while
  *               the other is high, differential speed large.
  *               Recovery: reduce speed, adjust steering.
  *
  *          Runs on M4 core at 1 kHz.  Interfaces with M7 FOC via shared memory.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include <string.h>
#include <math.h>

/* Private defines -----------------------------------------------------------*/
/* Common */
#define STALL_CURRENT_THRESH_A      8.0f
#define STALL_SPEED_THRESH_RPM      50.0f    /* below this = effectively zero */
#define STALL_TIME_MS               500U     /* condition must persist this long */

/* Hard stall */
#define HARD_STALL_DI_CURRENT       3.0f     /* delta current for abrupt detection [A per sample] */
#define HARD_STALL_TIME_MS          200U     /* shorter timeout for hard stall */

/* Soft stall */
#define SOFT_STALL_TIME_MS          800U     /* longer timeout for gradual onset */
#define SOFT_STALL_SPEED_THRESH_RPM 200.0f   /* low but not zero */

/* Wheel slip */
#define SLIP_CURRENT_RATIO          3.0f     /* L/R current ratio > 3x */
#define SLIP_SPEED_DIFF_RPM         1000.0f  /* speed diff > 1000 RPM */
#define SLIP_TIME_MS                300U

/* Recovery */
#define HARD_STALL_RETRIES          3U       /* max retries before permanent fault */
#define SOFT_STALL_REVERSE_MS       1000U
#define SLIP_REDUCE_FACTOR          0.7f

/* ---------------------------------------------------------------------------*/
/*  Stall types                                                               */
/* ---------------------------------------------------------------------------*/
typedef enum {
    STALL_NONE       = 0U,
    STALL_HARD       = 1U,
    STALL_SOFT       = 2U,
    STALL_WHEEL_SLIP = 3U,
} StallType;

/* ---------------------------------------------------------------------------*/
/*  Per-motor stall state                                                     */
/* ---------------------------------------------------------------------------*/
typedef struct {
    uint8_t  motor_id;
    float    current_a;           /* measured Q-axis current [A] */
    float    speed_rpm;           /* measured speed [RPM] */
    float    current_prev;        /* previous sample for delta */

    /* Hard stall */
    uint8_t  hard_stall_triggered;
    uint32_t hard_stall_start_ms;
    uint32_t hard_stall_retries;

    /* Soft stall */
    uint8_t  soft_stall_triggered;
    uint32_t soft_stall_start_ms;

    /* General stall detection */
    uint8_t  stall_active;
    StallType stall_type;
    uint32_t stall_time_ms;
    uint32_t stall_start_ms;
    uint32_t last_recovery_ms;

    /* Recovery */
    uint8_t  recovery_active;
    uint32_t recovery_start_ms;
} MotorStallState;

/* ---------------------------------------------------------------------------*/
/*  Stall handle                                                              */
/* ---------------------------------------------------------------------------*/
typedef struct {
    MotorStallState wheel_l;
    MotorStallState wheel_r;
    MotorStallState roller;
    MotorStallState side;
    MotorStallState fan;

    /* Wheel slip (cross-motor) */
    float    wheel_l_current;
    float    wheel_r_current;
    float    wheel_l_speed;
    float    wheel_r_speed;
    uint8_t  slip_detected;
    uint32_t slip_start_ms;

    /* Global */
    uint32_t total_stalls;
    uint32_t total_recoveries;
} StallHandleTypeDef;

static StallHandleTypeDef hstall;

/* ---------------------------------------------------------------------------*/
/*  Initialisation                                                            */
/* ---------------------------------------------------------------------------*/

/** @brief  Initialise stall detection module. */
void STALL_Init(void)
{
    memset(&hstall, 0, sizeof(hstall));
}

/* ---------------------------------------------------------------------------*/
/*  Per-motor stall detection                                                 */
/* ---------------------------------------------------------------------------*/

/** @brief  Detect hard stall: abrupt current spike + zero speed.
  *         Returns true if hard stall is confirmed.
  */
static uint8_t detect_hard_stall(MotorStallState *m, uint32_t now_ms)
{
    float dI = fabsf(m->current_a - m->current_prev);

    if ((dI > HARD_STALL_DI_CURRENT) &&
        (fabsf(m->speed_rpm) < STALL_SPEED_THRESH_RPM) &&
        (fabsf(m->current_a) > STALL_CURRENT_THRESH_A * 0.8f))
    {
        if (m->hard_stall_start_ms == 0U) {
            m->hard_stall_start_ms = now_ms;
        }
        if ((now_ms - m->hard_stall_start_ms) > HARD_STALL_TIME_MS) {
            return 1U;
        }
    } else {
        /* Reset if current drops or speed rises */
        if (fabsf(m->current_a) < STALL_CURRENT_THRESH_A * 0.5f ||
            fabsf(m->speed_rpm) > STALL_SPEED_THRESH_RPM * 2.0f) {
            m->hard_stall_start_ms = 0U;
        }
    }
    return 0U;
}

/** @brief  Detect soft stall: moderate current + very low speed (gradual).
  *         Returns true if soft stall is confirmed.
  */
static uint8_t detect_soft_stall(MotorStallState *m, uint32_t now_ms)
{
    if ((fabsf(m->current_a) > STALL_CURRENT_THRESH_A * 0.6f) &&
        (fabsf(m->speed_rpm) < SOFT_STALL_SPEED_THRESH_RPM) &&
        (fabsf(m->speed_rpm) > STALL_SPEED_THRESH_RPM)) /* not zero, but very low */
    {
        if (m->soft_stall_start_ms == 0U) {
            m->soft_stall_start_ms = now_ms;
        }
        if ((now_ms - m->soft_stall_start_ms) > SOFT_STALL_TIME_MS) {
            return 1U;
        }
    } else {
        if (fabsf(m->current_a) < STALL_CURRENT_THRESH_A * 0.3f) {
            m->soft_stall_start_ms = 0U;
        }
    }
    return 0U;
}

/* ---------------------------------------------------------------------------*/
/*  Wheel slip detection (across left and right wheels)                       */
/* ---------------------------------------------------------------------------*/

/** @brief  Detect wheel slip: one wheel spinning faster with lower current.
  *         Indicates loss of traction.
  */
static uint8_t detect_wheel_slip(uint32_t now_ms)
{
    float i_l = fabsf(hstall.wheel_l.current_a);
    float i_r = fabsf(hstall.wheel_r.current_a);
    float s_l = fabsf(hstall.wheel_l.speed_rpm);
    float s_r = fabsf(hstall.wheel_r.speed_rpm);

    /* Both wheels should have similar currents and speeds in normal operation.
       Slip: one wheel has low current but high speed (spinning freely). */
    uint8_t slip_condition = 0U;

    if ((i_l > 0.5f) && (i_r > 0.5f)) {
        float current_ratio = (i_l > i_r) ? (i_l / i_r) : (i_r / i_l);
        float speed_diff = fabsf(s_l - s_r);

        if ((current_ratio > SLIP_CURRENT_RATIO) && (speed_diff > SLIP_SPEED_DIFF_RPM)) {
            slip_condition = 1U;
        }
    }

    if (slip_condition) {
        if (hstall.slip_start_ms == 0U) {
            hstall.slip_start_ms = now_ms;
        }
        if ((now_ms - hstall.slip_start_ms) > SLIP_TIME_MS) {
            return 1U;
        }
    } else {
        if ((now_ms - hstall.slip_start_ms) > 500U) {
            hstall.slip_start_ms = 0U;
        }
    }

    return 0U;
}

/* ---------------------------------------------------------------------------*/
/*  Stall recovery                                                            */
/* ---------------------------------------------------------------------------*/

/** @brief  Execute recovery action based on stall type.
  *         Returns 1 if recovery is still in progress.
  */
static uint8_t execute_recovery(MotorStallState *m, uint32_t now_ms)
{
    if (!m->recovery_active) {
        /* Start recovery */
        m->recovery_active = 1U;
        m->recovery_start_ms = now_ms;
        m->last_recovery_ms = now_ms;
        hstall.total_recoveries++;
    }

    uint32_t elapsed = now_ms - m->recovery_start_ms;

    switch (m->stall_type) {
        case STALL_HARD:
            /* Stop -> brief reverse -> wait -> retry */
            if (elapsed < 200U) {
                /* Phase 1: stop (do nothing, PWM disabled) */
            } else if (elapsed < 500U) {
                /* Phase 2: reverse */
            } else if (elapsed < 800U) {
                /* Phase 3: wait */
            } else {
                /* Phase 4: retry */
                m->recovery_active = 0U;
                m->stall_active = 0U;
                m->hard_stall_retries++;

                if (m->hard_stall_retries >= HARD_STALL_RETRIES) {
                    /* Permanent fault after max retries */
                    return 0U; /* recovery failed */
                }
            }
            return 1U; /* still recovering */

        case STALL_SOFT:
            /* Reduce speed, apply reverse burst */
            if (elapsed < SOFT_STALL_REVERSE_MS) {
                /* Reverse burst */
            } else {
                m->recovery_active = 0U;
                m->stall_active = 0U;
            }
            return 1U;

        case STALL_WHEEL_SLIP:
            /* Reduce speed on both wheels */
            if (elapsed < 500U) {
                /* Speed reduction active */
            } else {
                m->recovery_active = 0U;
                m->stall_active = 0U;
                hstall.slip_detected = 0U;
            }
            return 1U;

        default:
            m->recovery_active = 0U;
            return 0U;
    }
}

/* ---------------------------------------------------------------------------*/
/*  Public update function                                                    */
/* ---------------------------------------------------------------------------*/

/** @brief  Update stall detection for all motors.
  *         Call at 1 kHz.
  *
  * @param  wheel_l_cur   Left wheel current [A].
  * @param  wheel_l_spd   Left wheel speed [RPM].
  * @param  wheel_r_cur   Right wheel current [A].
  * @param  wheel_r_spd   Right wheel speed [RPM].
  * @param  roller_cur    Roller brush current [A].
  * @param  roller_spd    Roller brush speed [RPM].
  * @param  side_cur      Side brush current [A].
  * @param  side_spd      Side brush speed [RPM].
  * @param  fan_cur       Vacuum fan current [A].
  * @param  fan_spd       Vacuum fan speed [RPM].
  */
void STALL_Update(float wheel_l_cur, float wheel_l_spd,
                   float wheel_r_cur, float wheel_r_spd,
                   float roller_cur, float roller_spd,
                   float side_cur, float side_spd,
                   float fan_cur, float fan_spd)
{
    uint32_t now_ms = HAL_GetTick();

    /* ---- Store current samples ---- */
    hstall.wheel_l.current_prev = hstall.wheel_l.current_a;
    hstall.wheel_l.current_a    = wheel_l_cur;
    hstall.wheel_l.speed_rpm    = wheel_l_spd;

    hstall.wheel_r.current_prev = hstall.wheel_r.current_a;
    hstall.wheel_r.current_a    = wheel_r_cur;
    hstall.wheel_r.speed_rpm    = wheel_r_spd;

    hstall.roller.current_prev  = hstall.roller.current_a;
    hstall.roller.current_a     = roller_cur;
    hstall.roller.speed_rpm     = roller_spd;

    hstall.side.current_prev    = hstall.side.current_a;
    hstall.side.current_a       = side_cur;
    hstall.side.speed_rpm       = side_spd;

    hstall.fan.current_prev     = hstall.fan.current_a;
    hstall.fan.current_a        = fan_cur;
    hstall.fan.speed_rpm        = fan_spd;

    /* ---- Hard stall checks ---- */
    if (detect_hard_stall(&hstall.wheel_l, now_ms)) {
        hstall.wheel_l.stall_type = STALL_HARD;
        hstall.wheel_l.stall_active = 1U;
    }
    if (detect_hard_stall(&hstall.wheel_r, now_ms)) {
        hstall.wheel_r.stall_type = STALL_HARD;
        hstall.wheel_r.stall_active = 1U;
    }
    if (detect_hard_stall(&hstall.roller, now_ms)) {
        hstall.roller.stall_type = STALL_HARD;
        hstall.roller.stall_active = 1U;
    }
    if (detect_hard_stall(&hstall.side, now_ms)) {
        hstall.side.stall_type = STALL_HARD;
        hstall.side.stall_active = 1U;
    }
    if (detect_hard_stall(&hstall.fan, now_ms)) {
        hstall.fan.stall_type = STALL_HARD;
        hstall.fan.stall_active = 1U;
    }

    /* ---- Soft stall checks ---- */
    if (!hstall.wheel_l.hard_stall_triggered &&
        detect_soft_stall(&hstall.wheel_l, now_ms)) {
        hstall.wheel_l.stall_type = STALL_SOFT;
        hstall.wheel_l.stall_active = 1U;
    }
    if (!hstall.roller.hard_stall_triggered &&
        detect_soft_stall(&hstall.roller, now_ms)) {
        hstall.roller.stall_type = STALL_SOFT;
        hstall.roller.stall_active = 1U;
    }
    if (!hstall.side.hard_stall_triggered &&
        detect_soft_stall(&hstall.side, now_ms)) {
        hstall.side.stall_type = STALL_SOFT;
        hstall.side.stall_active = 1U;
    }

    /* ---- Wheel slip check ---- */
    if (detect_wheel_slip(now_ms)) {
        hstall.slip_detected = 1U;
        hstall.wheel_l.stall_type = STALL_WHEEL_SLIP;
        hstall.wheel_r.stall_type = STALL_WHEEL_SLIP;
        hstall.wheel_l.stall_active = 1U;
        hstall.wheel_r.stall_active = 1U;
    }

    /* ---- Execute recovery ---- */
    MotorStallState *motors[] = {
        &hstall.wheel_l, &hstall.wheel_r,
        &hstall.roller, &hstall.side, &hstall.fan
    };

    for (uint32_t i = 0; i < 5U; i++) {
        if (motors[i]->stall_active) {
            if (!motors[i]->recovery_active) {
                hstall.total_stalls++;
            }
            uint8_t still_recovering = execute_recovery(motors[i], now_ms);
            if (!still_recovering && !motors[i]->recovery_active) {
                motors[i]->stall_active = 0U;
            }
        }
    }

    /* Update previous currents */
    hstall.wheel_l.current_prev = wheel_l_cur;
    hstall.wheel_r.current_prev = wheel_r_cur;
    hstall.roller.current_prev  = roller_cur;
    hstall.side.current_prev    = side_cur;
    hstall.fan.current_prev     = fan_cur;
}

/* ---------------------------------------------------------------------------*/
/*  Public API                                                                */
/* ---------------------------------------------------------------------------*/

/** @brief  Check if any motor is currently stalled. */
uint8_t STALL_IsAnyStalled(void)
{
    return (hstall.wheel_l.stall_active || hstall.wheel_r.stall_active ||
            hstall.roller.stall_active || hstall.side.stall_active ||
            hstall.fan.stall_active) ? 1U : 0U;
}

/** @brief  Get stall type for a specific motor (0=wheel_l, 1=wheel_r, 2=roller, 3=side, 4=fan). */
StallType STALL_GetType(uint8_t motor_idx)
{
    MotorStallState *motors[] = {
        &hstall.wheel_l, &hstall.wheel_r,
        &hstall.roller, &hstall.side, &hstall.fan
    };
    if (motor_idx > 4U) return STALL_NONE;
    return motors[motor_idx]->stall_type;
}

/** @brief  Get stall type as string. */
const char* STALL_GetTypeStr(uint8_t motor_idx)
{
    switch (STALL_GetType(motor_idx)) {
        case STALL_HARD:       return "HARD";
        case STALL_SOFT:       return "SOFT";
        case STALL_WHEEL_SLIP: return "SLIP";
        default:               return "NONE";
    }
}

/** @brief  Check if wheel slip is detected. */
uint8_t STALL_IsSlipDetected(void)
{
    return hstall.slip_detected;
}

/** @brief  Get total stall count. */
uint32_t STALL_GetTotalStalls(void)
{
    return hstall.total_stalls;
}

/** @brief  Get total recovery count. */
uint32_t STALL_GetTotalRecoveries(void)
{
    return hstall.total_recoveries;
}

/** @brief  Clear stall condition manually. */
void STALL_Clear(uint8_t motor_idx)
{
    MotorStallState *motors[] = {
        &hstall.wheel_l, &hstall.wheel_r,
        &hstall.roller, &hstall.side, &hstall.fan
    };
    if (motor_idx > 4U) return;
    motors[motor_idx]->stall_active = 0U;
    motors[motor_idx]->recovery_active = 0U;
    motors[motor_idx]->hard_stall_start_ms = 0U;
    motors[motor_idx]->soft_stall_start_ms = 0U;
    motors[motor_idx]->hard_stall_retries = 0U;
}

/** @brief  Clear all stall conditions. */
void STALL_ClearAll(void)
{
    for (uint8_t i = 0; i < 5U; i++) STALL_Clear(i);
    hstall.slip_detected = 0U;
    hstall.slip_start_ms = 0U;
}
