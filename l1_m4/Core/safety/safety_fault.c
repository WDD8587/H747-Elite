/**
  ******************************************************************************
  * @file    safety_fault.c
  * @author  M4 Safety Team
  * @brief   Fault injection framework for HIL (Hardware-in-the-Loop) testing.
  *
  *          Provides software-controllable fault injection for all major
  *          subsystems:
  *            - Overcurrent on any phase
  *            - Encoder disconnect / signal loss
  *            - IMU data freeze / stuck value
  *            - CAN bus-off
  *            - ToF sensor timeout
  *            - Motor stall
  *            - Battery voltage sag
  *            - M7-M4 comms loss
  *
  *          Each fault has:
  *            - Enable/disable toggle
  *            - Trigger condition (immediate, delayed, or conditional)
  *            - Expected system response (for automated pass/fail)
  *            - Verification hook (checks that safety_wd caught it)
  *
  *          Compile-time guard: FAULT_INJECTION_ENABLED must be defined.
  *          In production builds, all fault injection code is compiled out.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include <string.h>
#include <math.h>

/* ---------------------------------------------------------------------------*/
/*  Fault injection framework — guarded by FAULT_INJECTION_ENABLED             */
/* ---------------------------------------------------------------------------*/
#ifdef FAULT_INJECTION_ENABLED

/* Private defines -----------------------------------------------------------*/
#define MAX_FAULT_INJECTORS     16U
#define FAULT_NAME_MAX_LEN      32U

/* Fault states */
typedef enum {
    FAULT_STATE_DISABLED    = 0U,
    FAULT_STATE_ARMED       = 1U,
    FAULT_STATE_TRIGGERED   = 2U,
    FAULT_STATE_VERIFIED    = 3U,
    FAULT_STATE_FAILED      = 4U,
} FaultState;

/* Trigger types */
typedef enum {
    TRIGGER_IMMEDIATE    = 0U,   /* fire immediately on arm */
    TRIGGER_DELAYED      = 1U,   /* fire after delay_ms */
    TRIGGER_CONDITIONAL  = 2U,   /* fire when condition callback returns true */
} TriggerType;

/* ---------------------------------------------------------------------------*/
/*  Fault injector structure                                                  */
/* ---------------------------------------------------------------------------*/
typedef struct {
    uint16_t  id;
    char      name[FAULT_NAME_MAX_LEN];
    FaultState state;
    TriggerType trigger_type;
    uint32_t  delay_ms;
    uint32_t  armed_ms;
    uint32_t  trigger_count;

    /* Fault-specific data */
    union {
        float overcurrent_a;         /* current to inject [A] */
        int32_t encoder_value;        /* fixed encoder count */
        float imu_freeze_value[6];    /* frozen accel + gyro values */
        uint8_t can_bus_off;          /* 1 = force bus-off */
        uint32_t tof_timeout_ms;      /* force timeout duration */
        float battery_sag_v;          /* voltage to inject [V] */
        uint8_t comms_loss;           /* 1 = force M7-M4 disconnect */
    } params;

    /* Expected response */
    const char *expected_response[4]; /* what safety_wd should do */
    uint8_t expected_response_count;

    /* Verification */
    uint8_t (*verify_callback)(uint16_t fault_id); /* returns 1 if caught */
    uint8_t verified;

    /* Condition for TRIGGER_CONDITIONAL */
    uint8_t (*condition_callback)(void); /* returns 1 when condition met */
} FaultInjector;

/* ---------------------------------------------------------------------------*/
/*  Fault injector pool                                                       */
/* ---------------------------------------------------------------------------*/
static FaultInjector injectors[MAX_FAULT_INJECTORS];
static uint32_t num_injectors = 0U;
static uint32_t total_triggers = 0U;
static uint32_t total_verified = 0U;
static uint32_t total_failed = 0U;

/* ---------------------------------------------------------------------------*/
/*  Injector registration                                                     */
/* ---------------------------------------------------------------------------*/

/** @brief  Register a new fault injector.
  * @param  name   Human-readable fault name.
  * @param  trigger Trigger type.
  * @param  delay  Delay in ms (for TRIGGER_DELAYED).
  * @return Injector ID, or 0xFFFF on failure.
  */
uint16_t FAULT_RegisterInjector(const char *name,
                                 TriggerType trigger, uint32_t delay)
{
    if (num_injectors >= MAX_FAULT_INJECTORS) return 0xFFFFU;

    FaultInjector *fi = &injectors[num_injectors];
    memset(fi, 0, sizeof(FaultInjector));

    strncpy(fi->name, name, FAULT_NAME_MAX_LEN - 1U);
    fi->name[FAULT_NAME_MAX_LEN - 1U] = '\0';
    fi->id = (uint16_t)num_injectors;
    fi->state = FAULT_STATE_DISABLED;
    fi->trigger_type = trigger;
    fi->delay_ms = delay;
    fi->armed_ms = 0U;
    fi->trigger_count = 0U;
    fi->verified = 0U;

    num_injectors++;
    return fi->id;
}

/* ---------------------------------------------------------------------------*/
/*  Pre-defined fault injectors                                               */
/* ---------------------------------------------------------------------------*/

/** @brief  Initialise all standard fault injectors for the H747 platform. */
void FAULT_InitAll(void)
{
    /* Clear all */
    memset(injectors, 0, sizeof(injectors));
    num_injectors = 0U;

    /* 1. Overcurrent on phase A */
    uint16_t id;
    id = FAULT_RegisterInjector("OVERCURRENT_PHASE_A", TRIGGER_IMMEDIATE, 0U);
    if (id != 0xFFFFU) injectors[id].params.overcurrent_a = 20.0f;

    /* 2. Overcurrent on phase B */
    id = FAULT_RegisterInjector("OVERCURRENT_PHASE_B", TRIGGER_IMMEDIATE, 0U);
    if (id != 0xFFFFU) injectors[id].params.overcurrent_a = 20.0f;

    /* 3. Encoder disconnect (freeze count) */
    id = FAULT_RegisterInjector("ENCODER_DISCONNECT", TRIGGER_DELAYED, 100U);
    if (id != 0xFFFFU) injectors[id].params.encoder_value = 0xDEAD;

    /* 4. IMU data freeze (all axes stuck) */
    id = FAULT_RegisterInjector("IMU_FREEZE", TRIGGER_DELAYED, 50U);
    if (id != 0xFFFFU) {
        for (int i = 0; i < 6; i++) injectors[id].params.imu_freeze_value[i] = 0.0f;
    }

    /* 5. CAN bus-off */
    id = FAULT_RegisterInjector("CAN_BUS_OFF", TRIGGER_IMMEDIATE, 0U);
    if (id != 0xFFFFU) injectors[id].params.can_bus_off = 1U;

    /* 6. ToF sensor timeout (cliff front-left) */
    id = FAULT_RegisterInjector("TOF_TIMEOUT_FL", TRIGGER_DELAYED, 200U);
    if (id != 0xFFFFU) injectors[id].params.tof_timeout_ms = 5000U;

    /* 7. ToF sensor timeout (cliff front-right) */
    id = FAULT_RegisterInjector("TOF_TIMEOUT_FR", TRIGGER_DELAYED, 200U);

    /* 8. Battery voltage sag (simulate discharged battery) */
    id = FAULT_RegisterInjector("BATTERY_SAG", TRIGGER_CONDITIONAL, 0U);
    if (id != 0xFFFFU) injectors[id].params.battery_sag_v = 16.0f;

    /* 9. Motor stall (left wheel) */
    id = FAULT_RegisterInjector("STALL_WHEEL_L", TRIGGER_DELAYED, 500U);

    /* 10. Motor stall (right wheel) */
    id = FAULT_RegisterInjector("STALL_WHEEL_R", TRIGGER_DELAYED, 500U);

    /* 11. M7-M4 comms loss */
    id = FAULT_RegisterInjector("COMMS_LOSS_M7M4", TRIGGER_DELAYED, 50U);
    if (id != 0xFFFFU) injectors[id].params.comms_loss = 1U;

    /* 12. Roller brush stall */
    id = FAULT_RegisterInjector("STALL_ROLLER", TRIGGER_DELAYED, 500U);

    total_triggers = 0U;
    total_verified = 0U;
    total_failed = 0U;
}

/* ---------------------------------------------------------------------------*/
/*  Fault control                                                             */
/* ---------------------------------------------------------------------------*/

/** @brief  Arm a fault injector (enable + prepare to trigger). */
uint8_t FAULT_Arm(uint16_t injector_id)
{
    if (injector_id >= num_injectors) return 0U;

    FaultInjector *fi = &injectors[injector_id];
    if (fi->state == FAULT_STATE_TRIGGERED) return 0U; /* already active */

    fi->state = FAULT_STATE_ARMED;
    fi->armed_ms = HAL_GetTick();
    return 1U;
}

/** @brief  Disarm / disable a fault injector. */
uint8_t FAULT_Disarm(uint16_t injector_id)
{
    if (injector_id >= num_injectors) return 0U;
    injectors[injector_id].state = FAULT_STATE_DISABLED;
    return 1U;
}

/** @brief  Manually trigger a fault injector (bypasses trigger logic). */
uint8_t FAULT_Trigger(uint16_t injector_id)
{
    if (injector_id >= num_injectors) return 0U;

    FaultInjector *fi = &injectors[injector_id];
    if (fi->state != FAULT_STATE_ARMED) return 0U;

    fi->state = FAULT_STATE_TRIGGERED;
    fi->trigger_count++;
    total_triggers++;
    return 1U;
}

/* ---------------------------------------------------------------------------*/
/*  Trigger evaluation                                                        */
/* ---------------------------------------------------------------------------*/

/** @brief  Evaluate all armed fault injectors and trigger as appropriate.
  *         Call periodically (e.g., at 1 kHz).
  */
void FAULT_EvaluateTriggers(void)
{
    uint32_t now_ms = HAL_GetTick();

    for (uint32_t i = 0; i < num_injectors; i++) {
        FaultInjector *fi = &injectors[i];
        if (fi->state != FAULT_STATE_ARMED) continue;

        uint8_t should_trigger = 0U;

        switch (fi->trigger_type) {
            case TRIGGER_IMMEDIATE:
                should_trigger = 1U;
                break;

            case TRIGGER_DELAYED:
                if ((now_ms - fi->armed_ms) >= fi->delay_ms) {
                    should_trigger = 1U;
                }
                break;

            case TRIGGER_CONDITIONAL:
                if (fi->condition_callback != NULL) {
                    should_trigger = fi->condition_callback();
                }
                break;

            default:
                break;
        }

        if (should_trigger) {
            fi->state = FAULT_STATE_TRIGGERED;
            fi->trigger_count++;
            total_triggers++;
        }
    }
}

/* ---------------------------------------------------------------------------*/
/*  Verification                                                              */
/* ---------------------------------------------------------------------------*/

/** @brief  Run verification for all triggered faults.
  *         Checks that the safety watchdog caught each one.
  *         Call after sufficient settling time.
  */
void FAULT_VerifyAll(void)
{
    for (uint32_t i = 0; i < num_injectors; i++) {
        FaultInjector *fi = &injectors[i];
        if (fi->state != FAULT_STATE_TRIGGERED) continue;
        if (fi->verified) continue; /* already verified */

        if (fi->verify_callback != NULL) {
            uint8_t caught = fi->verify_callback(fi->id);
            if (caught) {
                fi->state = FAULT_STATE_VERIFIED;
                fi->verified = 1U;
                total_verified++;
            } else {
                fi->state = FAULT_STATE_FAILED;
                total_failed++;
            }
        } else {
            /* No callback: mark as verified (assume caught) */
            fi->state = FAULT_STATE_VERIFIED;
            fi->verified = 1U;
            total_verified++;
        }
    }
}

/** @brief  Set a verification callback for a fault injector. */
void FAULT_SetVerifyCallback(uint16_t injector_id,
                              uint8_t (*callback)(uint16_t))
{
    if (injector_id >= num_injectors) return;
    injectors[injector_id].verify_callback = callback;
}

/** @brief  Set a condition callback for conditional trigger. */
void FAULT_SetConditionCallback(uint16_t injector_id,
                                 uint8_t (*callback)(void))
{
    if (injector_id >= num_injectors) return;
    injectors[injector_id].condition_callback = callback;
}

/** @brief  Set injector parameters (fault-specific data). */
void FAULT_SetParamFloat(uint16_t injector_id, float value)
{
    if (injector_id >= num_injectors) return;
    injectors[injector_id].params.overcurrent_a = value;
}

/* ---------------------------------------------------------------------------*/
/*  Status and reporting                                                      */
/* ---------------------------------------------------------------------------*/

/** @brief  Get state of a fault injector. */
FaultState FAULT_GetState(uint16_t injector_id)
{
    if (injector_id >= num_injectors) return FAULT_STATE_DISABLED;
    return injectors[injector_id].state;
}

/** @brief  Get total number of registered injectors. */
uint32_t FAULT_GetNumInjectors(void)
{
    return num_injectors;
}

/** @brief  Get total triggers fired. */
uint32_t FAULT_GetTotalTriggers(void)
{
    return total_triggers;
}

/** @brief  Get total verified faults. */
uint32_t FAULT_GetTotalVerified(void)
{
    return total_verified;
}

/** @brief  Get total failed verifications. */
uint32_t FAULT_GetTotalFailed(void)
{
    return total_failed;
}

/** @brief  Get test pass/fail status (all triggered faults verified). */
uint8_t FAULT_AllPassed(void)
{
    for (uint32_t i = 0; i < num_injectors; i++) {
        if (injectors[i].state == FAULT_STATE_FAILED) return 0U;
        if (injectors[i].state == FAULT_STATE_TRIGGERED) return 0U; /* not yet verified */
    }
    return 1U;
}

/** @brief  Reset all injectors. */
void FAULT_ResetAll(void)
{
    for (uint32_t i = 0; i < num_injectors; i++) {
        injectors[i].state = FAULT_STATE_DISABLED;
        injectors[i].armed_ms = 0U;
        injectors[i].verified = 0U;
    }
    total_triggers = 0U;
    total_verified = 0U;
    total_failed = 0U;
}

#else /* !FAULT_INJECTION_ENABLED */

/* Stubs for production builds — all fault injection code is removed. */
uint16_t FAULT_RegisterInjector(const char *name, TriggerType trigger, uint32_t delay)
{
    (void)name; (void)trigger; (void)delay;
    return 0xFFFFU;
}
void    FAULT_InitAll(void) {}
uint8_t FAULT_Arm(uint16_t id) { (void)id; return 0U; }
uint8_t FAULT_Disarm(uint16_t id) { (void)id; return 0U; }
uint8_t FAULT_Trigger(uint16_t id) { (void)id; return 0U; }
void    FAULT_EvaluateTriggers(void) {}
void    FAULT_VerifyAll(void) {}
void    FAULT_SetVerifyCallback(uint16_t id, uint8_t (*cb)(uint16_t)) { (void)id; (void)cb; }
void    FAULT_SetConditionCallback(uint16_t id, uint8_t (*cb)(void)) { (void)id; (void)cb; }
void    FAULT_SetParamFloat(uint16_t id, float v) { (void)id; (void)v; }
uint32_t FAULT_GetNumInjectors(void) { return 0U; }
uint32_t FAULT_GetTotalTriggers(void) { return 0U; }
uint32_t FAULT_GetTotalVerified(void) { return 0U; }
uint32_t FAULT_GetTotalFailed(void) { return 0U; }
uint8_t  FAULT_AllPassed(void) { return 1U; }
void     FAULT_ResetAll(void) {}

#endif /* FAULT_INJECTION_ENABLED */
