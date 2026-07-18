/**
  ******************************************************************************
  * @file    multi_motor.c
  * @author  M7 FOC Team
  * @brief   Multi-motor coordinator for H747 Elite robot platform.
  *
  *          Manages five motors sharing the same TIM1+TIM8 and HRTIM resources:
  *            - Wheel L (left drive)   — TIM1_CH1
  *            - Wheel R (right drive)  — TIM1_CH2
  *            - Roller brush           — TIM1_CH3
  *            - Side brush             — TIM8_CH1
  *            - Vacuum fan             — TIM8_CH2
  *
  *          Torque allocation priority (highest to lowest):
  *            1. SAFETY   — current budget reserved for emergency manoeuvres
  *            2. WHEEL    — traction (L + R)
  *            3. ROLLER   — roller brush
  *            4. SIDE     — side brush
  *            5. FAN      — vacuum fan
  *
  *          Each motor has its own FOC handle with independent Id/Iq refs.
  *          Coordinator distributes total power budget across motors, respecting
  *          priority and individual current limits.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include <string.h>
#include <math.h>

/* Private defines -----------------------------------------------------------*/
#define NUM_MOTORS              5U

/* Motor indices */
#define MOTOR_WHEEL_L           0U
#define MOTOR_WHEEL_R           1U
#define MOTOR_ROLLER            2U
#define MOTOR_SIDE              3U
#define MOTOR_FAN               4U

/* Total system current budget [A] (from PSU capability, e.g. 80 A peak) */
#define TOTAL_CURRENT_BUDGET_A  80.0f
#define SAFETY_RESERVE_A        10.0f    /* reserved for emergency */

/* Individual current limits */
#define WHEEL_L_CURRENT_MAX_A   20.0f
#define WHEEL_R_CURRENT_MAX_A   20.0f
#define ROLLER_CURRENT_MAX_A    8.0f
#define SIDE_CURRENT_MAX_A      3.0f
#define FAN_CURRENT_MAX_A       5.0f

/* Priority levels (lower number = higher priority) */
#define PRIORITY_SAFETY         0U
#define PRIORITY_WHEEL          1U
#define PRIORITY_ROLLER         2U
#define PRIORITY_SIDE           3U
#define PRIORITY_FAN            4U

/* Control modes */
typedef enum {
    MODE_STOP      = 0U,
    MODE_SPEED     = 1U,    /* closed-loop speed control */
    MODE_TORQUE    = 2U,    /* closed-loop torque (current) control */
    MODE_POWER     = 3U,    /* constant power mode (fan) */
    MODE_OPENLOOP  = 4U,    /* open-loop voltage control (startup) */
} MotorControlMode;

/* ---------------------------------------------------------------------------*/
/*  Per-motor structure                                                       */
/* ---------------------------------------------------------------------------*/
typedef struct {
    /* Identification */
    uint8_t  id;
    uint8_t  enabled;
    const char *name;

    /* Control mode */
    MotorControlMode mode;

    /* References */
    float    speed_ref_rpm;      /* speed target [RPM] */
    float    torque_ref_nm;      /* torque target [Nm] */
    float    iq_ref;             /* Q-axis current ref [A] */
    float    id_ref;             /* D-axis current ref [A] */

    /* Feedback */
    float    speed_meas_rpm;     /* measured speed [RPM] */
    float    iq_meas;            /* measured Q-axis current [A] */
    float    id_meas;            /* measured D-axis current [A] */
    float    torque_estimate;    /* estimated torque [Nm] */
    float    power_estimate;     /* estimated power [W] */

    /* Limits */
    float    iq_max;             /* Q-axis current limit [A] */
    float    iq_min;
    float    power_max_w;        /* power limit [W] */

    /* Priority */
    uint8_t  priority;

    /* Status */
    uint8_t  fault;              /* 1 = motor in fault */
    uint8_t  current_limited;    /* 1 = being current-limited by coordinator */
    float    allocated_current;  /* current allocated by coordinator [A] */

    /* Speed PI controller (outer loop) */
    float    sp_kp;
    float    sp_ki;
    float    sp_integral;
    float    sp_output_max;
    float    sp_output_min;

} MotorHandleTypeDef;

/* ---------------------------------------------------------------------------*/
/*  Static motor instances                                                    */
/* ---------------------------------------------------------------------------*/
static MotorHandleTypeDef motors[NUM_MOTORS];

static const float motor_iq_max[NUM_MOTORS] = {
    WHEEL_L_CURRENT_MAX_A,
    WHEEL_R_CURRENT_MAX_A,
    ROLLER_CURRENT_MAX_A,
    SIDE_CURRENT_MAX_A,
    FAN_CURRENT_MAX_A,
};

static const uint8_t motor_priority[NUM_MOTORS] = {
    PRIORITY_WHEEL,
    PRIORITY_WHEEL,
    PRIORITY_ROLLER,
    PRIORITY_SIDE,
    PRIORITY_FAN,
};

/* Total available current after reserving safety margin */
static float g_total_available_a = TOTAL_CURRENT_BUDGET_A - SAFETY_RESERVE_A;
static float g_total_allocated_a = 0.0f;
static uint8_t g_coordinator_active = 0U;

/* ---------------------------------------------------------------------------*/
/*  Initialisation                                                            */
/* ---------------------------------------------------------------------------*/

/** @brief  Initialise all motor instances with default parameters. */
void MULTI_Init(void)
{
    const char *motor_names[NUM_MOTORS] = {
        "WHEEL_L", "WHEEL_R", "ROLLER", "SIDE", "FAN"
    };

    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        MotorHandleTypeDef *m = &motors[i];
        memset(m, 0, sizeof(MotorHandleTypeDef));
        m->id         = i;
        m->enabled    = 0U;
        m->name       = motor_names[i];
        m->mode       = MODE_STOP;
        m->priority   = motor_priority[i];
        m->iq_max     = motor_iq_max[i];
        m->iq_min     = -motor_iq_max[i];
        m->power_max_w = 200.0f;
        m->fault      = 0U;
        m->current_limited = 0U;
        m->allocated_current = 0.0f;

        /* Default speed PI gains */
        m->sp_kp        = 0.5f;
        m->sp_ki        = 0.02f;
        m->sp_integral  = 0.0f;
        m->sp_output_max = m->iq_max;
        m->sp_output_min = m->iq_min;
    }

    /* Override fan for constant power mode */
    motors[MOTOR_FAN].mode = MODE_POWER;

    g_total_available_a = TOTAL_CURRENT_BUDGET_A - SAFETY_RESERVE_A;
    g_total_allocated_a = 0.0f;
    g_coordinator_active = 0U;
}

/** @brief  Enable a specific motor (turn on its PWM outputs). */
void MULTI_EnableMotor(uint8_t motor_id)
{
    if (motor_id >= NUM_MOTORS) return;
    motors[motor_id].enabled = 1U;
}

/** @brief  Disable a specific motor (safe stop). */
void MULTI_DisableMotor(uint8_t motor_id)
{
    if (motor_id >= NUM_MOTORS) return;
    motors[motor_id].enabled = 0U;
    motors[motor_id].iq_ref  = 0.0f;
    motors[motor_id].speed_ref_rpm = 0.0f;
}

/* ---------------------------------------------------------------------------*/
/*  Torque allocation (budgeting)                                              */
/* ---------------------------------------------------------------------------*/

/** @brief  Distribute total current budget across motors respecting priority.
  *
  *          Algorithm:
  *            1. Sort motors by priority (already in priority order array).
  *            2. Allocate Iq_max to each motor in priority order until budget exhausted.
  *            3. If budget runs out, remaining motors get 0.
  *            4. SAFETY_RESERVE is never touched.
  *
  *          Call after any motor reference changes, before FOC update.
  */
void MULTI_AllocateTorque(void)
{
    if (!g_coordinator_active) return;

    float remaining = g_total_available_a;
    g_total_allocated_a = 0.0f;

    /* Pass 1: allocate full Iq_max to higher-priority motors */
    for (uint8_t prio = 0; prio <= PRIORITY_FAN; prio++) {
        for (uint8_t i = 0; i < NUM_MOTORS; i++) {
            if (motors[i].priority != prio) continue;
            if (!motors[i].enabled || motors[i].fault) {
                motors[i].allocated_current = 0.0f;
                continue;
            }

            float required = fabsf(motors[i].iq_ref);
            if (required > motors[i].iq_max) required = motors[i].iq_max;

            if (required <= remaining) {
                motors[i].allocated_current = required;
                remaining -= required;
            } else {
                motors[i].allocated_current = remaining;
                motors[i].current_limited = 1U;
                remaining = 0.0f;
            }
            g_total_allocated_a += motors[i].allocated_current;
        }
    }

    /* Pass 2: clamp Iq references to allocated values */
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        if (!motors[i].enabled) continue;
        float allocated = motors[i].allocated_current;
        if (motors[i].iq_ref > allocated) {
            motors[i].iq_ref = allocated;
            motors[i].current_limited = 1U;
        } else if (motors[i].iq_ref < -allocated) {
            motors[i].iq_ref = -allocated;
            motors[i].current_limited = 1U;
        } else {
            motors[i].current_limited = 0U;
        }
    }
}

/** @brief  Enable or disable the coordinator (allows torque allocation). */
void MULTI_SetCoordinatorActive(uint8_t active)
{
    g_coordinator_active = active;
}

/** @brief  Get the total allocated current [A]. */
float MULTI_GetTotalAllocatedCurrent(void)
{
    return g_total_allocated_a;
}

/* ---------------------------------------------------------------------------*/
/*  Speed controller (outer loop, run at 1 kHz)                               */
/* ---------------------------------------------------------------------------*/

/** @brief  Run speed PI controller for a motor, producing Iq reference.
  *         Call at 1 kHz (every 20th FOC cycle).
  */
static void motor_speed_controller(MotorHandleTypeDef *m)
{
    if (m->mode != MODE_SPEED || !m->enabled) return;

    float err = m->speed_ref_rpm - m->speed_meas_rpm;
    float p_term = m->sp_kp * err;

    /* Integrate with anti-windup */
    float u_pre = p_term + m->sp_integral;
    if (u_pre > m->sp_output_max || u_pre < m->sp_output_min) {
        /* Saturated: do not integrate */
    } else {
        m->sp_integral += m->sp_ki * err;
        if (m->sp_integral > m->sp_output_max) m->sp_integral = m->sp_output_max;
        if (m->sp_integral < m->sp_output_min) m->sp_integral = m->sp_output_min;
    }

    float output = p_term + m->sp_integral;
    if (output > m->sp_output_max) output = m->sp_output_max;
    if (output < m->sp_output_min) output = m->sp_output_min;

    m->iq_ref = output;
}

/** @brief  Run all speed controllers. */
void MULTI_SpeedControlLoop(void)
{
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        motor_speed_controller(&motors[i]);
    }
}

/* ---------------------------------------------------------------------------*/
/*  Set references                                                            */
/* ---------------------------------------------------------------------------*/

/** @brief  Set speed reference for a motor [RPM]. */
void MULTI_SetSpeedRef(uint8_t motor_id, float speed_rpm)
{
    if (motor_id >= NUM_MOTORS) return;
    motors[motor_id].mode = MODE_SPEED;
    motors[motor_id].speed_ref_rpm = speed_rpm;
}

/** @brief  Set torque (Iq) reference for a motor [A]. */
void MULTI_SetTorqueRef(uint8_t motor_id, float iq_a)
{
    if (motor_id >= NUM_MOTORS) return;
    motors[motor_id].mode = MODE_TORQUE;
    motors[motor_id].iq_ref = iq_a;
}

/** @brief  Set wheel speeds differentially for robot motion.
  * @param  left_rpm   Left wheel target [RPM]
  * @param  right_rpm  Right wheel target [RPM]
  */
void MULTI_SetWheelSpeeds(float left_rpm, float right_rpm)
{
    MULTI_SetSpeedRef(MOTOR_WHEEL_L, left_rpm);
    MULTI_SetSpeedRef(MOTOR_WHEEL_R, right_rpm);
}

/** @brief  Set roller brush speed [RPM]. */
void MULTI_SetRollerSpeed(float rpm)
{
    MULTI_SetSpeedRef(MOTOR_ROLLER, rpm);
}

/** @brief  Set side brush speed [RPM]. */
void MULTI_SetSideSpeed(float rpm)
{
    MULTI_SetSpeedRef(MOTOR_SIDE, rpm);
}

/** @brief  Set fan power (constant power mode) [W]. */
void MULTI_SetFanPower(float watts)
{
    if (watts < 0.0f) watts = 0.0f;
    motors[MOTOR_FAN].mode = MODE_POWER;
    motors[MOTOR_FAN].power_max_w = watts;
}

/* ---------------------------------------------------------------------------*/
/*  Feedback                                                                  */
/* ---------------------------------------------------------------------------*/

/** @brief  Update measured speed for a motor (call from encoder ISR). */
void MULTI_UpdateSpeed(uint8_t motor_id, float speed_rpm)
{
    if (motor_id >= NUM_MOTORS) return;
    motors[motor_id].speed_meas_rpm = speed_rpm;
}

/** @brief  Update measured currents for a motor (call from FOC ISR). */
void MULTI_UpdateCurrents(uint8_t motor_id, float iq, float id)
{
    if (motor_id >= NUM_MOTORS) return;
    motors[motor_id].iq_meas = iq;
    motors[motor_id].id_meas = id;

    /* Estimate torque: T = 1.5 * pole_pairs * (flux * iq + (Ld-Lq)*id*iq)
       Simplified: T = Kt * iq */
    extern uint16_t PARAMS_GetPolePairs(void);
    extern float PARAMS_GetLd(void);
    extern float PARAMS_GetLq(void);
    float flux = 0.0025f; /* 2.5 mWb default */
    float Kt = 1.5f * (float)PARAMS_GetPolePairs() * flux;
    motors[motor_id].torque_estimate = Kt * iq;

    /* Power = V * I (estimated from DC bus) */
    extern float FOC_GetIq(void);
    motors[motor_id].power_estimate = 24.0f * iq * 0.8f; /* rough efficiency */
}

/** @brief  Get motor data for external monitoring. */
MotorHandleTypeDef* MULTI_GetMotor(uint8_t motor_id)
{
    if (motor_id >= NUM_MOTORS) return NULL;
    return &motors[motor_id];
}

/** @brief  Get number of managed motors. */
uint8_t MULTI_GetMotorCount(void)
{
    return NUM_MOTORS;
}

/* ---------------------------------------------------------------------------*/
/*  Fault handling                                                            */
/* ---------------------------------------------------------------------------*/

/** @brief  Set fault flag for a motor (coordinator will deprioritise it). */
void MULTI_SetFault(uint8_t motor_id, uint8_t fault)
{
    if (motor_id >= NUM_MOTORS) return;
    motors[motor_id].fault = fault;
    if (fault) {
        motors[motor_id].iq_ref = 0.0f;
        motors[motor_id].speed_ref_rpm = 0.0f;
    }
}

/** @brief  Emergency stop all motors. */
void MULTI_EmergencyStop(void)
{
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        motors[i].iq_ref = 0.0f;
        motors[i].speed_ref_rpm = 0.0f;
        motors[i].sp_integral = 0.0f;
    }
}

/** @brief  Resume all motors from emergency stop (restore previous refs). */
void MULTI_EmergencyResume(void)
{
    /* References need to be re-set by application */
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        motors[i].sp_integral = 0.0f;
        motors[i].fault = 0U;
    }
}
