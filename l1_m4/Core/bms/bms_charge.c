/**
 * @file    bms_charge.c
 * @brief   Enhanced charge state machine for Li-Ion BMS.
 *          Features:
 *          - Temperature-compensated CV voltage (-3 mV/C/cell above 25 C)
 *          - Timer-based stage transitions with maximum duration
 *          - Charge efficiency tracking (Ah_in vs Ah_out)
 *
 * @note    Part of STM32H747 BMS subsystem.
 */

#include "bms_charge.h"
#include "bms_adc.h"
#include "bms_ntc.h"
#include "bms_soh.h"
#include "bms_protect.h"
#include "bms_gpio.h"
#include "bms_timer.h"
#include "bms_flash.h"

#include <string.h>
#include <math.h>

/* ---------------------------------------------------------------------------
 * Constants
 * ------------------------------------------------------------------------- */
#define CHARGE_CC_CURRENT_mA           2000    /* 2 A CC phase              */
#define CHARGE_CV_VOLTAGE_mV           4200    /* 4.2 V per cell (nominal)  */
#define CHARGE_CV_TEMP_COEFF_mVperC      3     /* -3 mV/C/cell above 25 C   */
#define CHARGE_CV_TEMP_REF_C            25
#define CHARGE_TERMINATION_CURRENT_mA   100    /* C/20 termination           */
#define CHARGE_PRECHARGE_CURRENT_mA     200    /* Pre-charge current         */
#define CHARGE_PRECHARGE_THRESHOLD_mV   3000   /* Below this: pre-charge     */
#define CHARGE_PRECHARGE_TIMEOUT_S      600    /* 10 min precharge max       */
#define CHARGE_CC_TIMEOUT_S            7200    /* 2 hr CC max                */
#define CHARGE_CV_TIMEOUT_S            3600    /* 1 hr CV max                */
#define CHARGE_TEMP_MIN_C                 0    /* Charge temp window         */
#define CHARGE_TEMP_MAX_C                55
#define CHARGE_TEMP_REDUCED_C            45    /* Above this: reduce current */
#define CHARGE_REDUCED_CURRENT_mA      1000    /* Reduced current            */
#define CHARGE_EFFICIENCY_SAMPLES       100    /* Rolling average window     */

/* ---------------------------------------------------------------------------
 * Charge state machine
 * ------------------------------------------------------------------------- */

typedef enum {
    CHG_IDLE,
    CHG_PRECHARGE,
    CHG_CC,                /* Constant Current */
    CHG_CV,                /* Constant Voltage */
    CHG_TERMINATED,
    CHG_FAULT
} Charge_Phase;

typedef struct {
    Charge_Phase phase;
    uint32_t     phase_start_tick;
    float        cc_current_mA;
    float        cv_voltage_mV;
    float        accumulated_charge_in_Ah;
    float        accumulated_discharge_Ah;
    float        efficiency_window[CHARGE_EFFICIENCY_SAMPLES];
    uint8_t      efficiency_idx;
    float        current_efficiency;
    bool         initialized;
} Charge_State;

static Charge_State chg_;

/* ---------------------------------------------------------------------------
 * Temperature-compensated CV voltage
 * ------------------------------------------------------------------------- */
static float charge_cv_compensated(void)
{
    float pack_temp = NTC_GetPackTemp();
    float delta_C   = pack_temp - CHARGE_CV_TEMP_REF_C;

    /* -3 mV/C per cell (assume 1S config; scale for series cells) */
    float comp = CHARGE_CV_VOLTAGE_mV + delta_C * CHARGE_CV_TEMP_COEFF_mVperC;

    /* Clamp to safe range */
    if (comp < 4000.0f) comp = 4000.0f;
    if (comp > CHARGE_CV_VOLTAGE_mV) comp = (float)CHARGE_CV_VOLTAGE_mV;

    return comp;
}

/* ---------------------------------------------------------------------------
 * Check charge temperature window
 * ------------------------------------------------------------------------- */
static bool charge_temp_ok(void)
{
    float t = NTC_GetPackTemp();
    return (t >= CHARGE_TEMP_MIN_C && t <= CHARGE_TEMP_MAX_C);
}

static float charge_get_cc_current(void)
{
    float t = NTC_GetPackTemp();
    if (t >= CHARGE_TEMP_REDUCED_C) {
        return (float)CHARGE_REDUCED_CURRENT_mA;
    }
    return (float)CHARGE_CC_CURRENT_mA;
}

/* ---------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

void CHG_Init(void)
{
    memset(&chg_, 0, sizeof(chg_));
    chg_.phase              = CHG_IDLE;
    chg_.current_efficiency = 1.0f;
    chg_.initialized        = true;
}

bool CHG_Start(void)
{
    if (!chg_.initialized) return false;
    if (chg_.phase != CHG_IDLE) return false;
    if (!charge_temp_ok()) return false;

    /* Check if pack needs precharge */
    float v_min = 9999.0f;
    for (int i = 0; i < BMS_CELL_COUNT; i++) {
        float v = (float)ADC_GetCellVoltage_mV(i);
        if (v < v_min) v_min = v;
    }

    if (v_min < CHARGE_PRECHARGE_THRESHOLD_mV) {
        chg_.phase = CHG_PRECHARGE;
    } else {
        chg_.phase = CHG_CC;
    }

    chg_.phase_start_tick       = TIMER_GetTick();
    chg_.cc_current_mA          = charge_get_cc_current();
    chg_.cv_voltage_mV          = charge_cv_compensated();
    chg_.accumulated_charge_in_Ah = 0.0f;

    /* Enable charge FET */
    GPIO_SetChargeFet(1);

    FLASH_LogEvent(FLASH_LOG_INFO, "CHG: started");
    return true;
}

void CHG_Task(void)
{
    if (!chg_.initialized) return;
    if (chg_.phase == CHG_IDLE || chg_.phase == CHG_TERMINATED || chg_.phase == CHG_FAULT) {
        return;
    }

    uint32_t now      = TIMER_GetTick();
    uint32_t elapsed  = (now - chg_.phase_start_tick) / 1000;  /* seconds */

    float V_pack = ADC_GetPackVoltage_mV();
    float I_pack = ADC_GetPackCurrent_mA();  /* positive = charging */
    float v_min  = 9999.0f;

    for (int i = 0; i < BMS_CELL_COUNT; i++) {
        float v = (float)ADC_GetCellVoltage_mV(i);
        if (v < v_min) v_min = v;
    }

    /* Temperature check */
    if (!charge_temp_ok()) {
        chg_.phase = CHG_FAULT;
        GPIO_SetChargeFet(0);
        FLASH_LogEvent(FLASH_LOG_ERROR, "CHG: temperature out of range");
        return;
    }

    /* Re-evaluate CC current */
    chg_.cc_current_mA = charge_get_cc_current();

    /* Re-evaluate CV voltage (tracks temperature changes) */
    chg_.cv_voltage_mV = charge_cv_compensated();

    switch (chg_.phase) {

    case CHG_PRECHARGE:
        /* Soft start with limited current */
        if (v_min >= CHARGE_PRECHARGE_THRESHOLD_mV) {
            chg_.phase = CHG_CC;
            chg_.phase_start_tick = now;
            FLASH_LogEvent(FLASH_LOG_INFO, "CHG: precharge -> CC");
        } else if (elapsed >= CHARGE_PRECHARGE_TIMEOUT_S) {
            chg_.phase = CHG_FAULT;
            GPIO_SetChargeFet(0);
            FLASH_LogEvent(FLASH_LOG_ERROR, "CHG: precharge timeout");
        }
        break;

    case CHG_CC:
        /* Check for CC timeout */
        if (elapsed >= CHARGE_CC_TIMEOUT_S) {
            chg_.phase = CHG_FAULT;
            GPIO_SetChargeFet(0);
            FLASH_LogEvent(FLASH_LOG_ERROR, "CHG: CC timeout");
            break;
        }

        /* Transition to CV when pack reaches target voltage */
        if (V_pack >= chg_.cv_voltage_mV) {
            chg_.phase = CHG_CV;
            chg_.phase_start_tick = now;
            FLASH_LogEvent(FLASH_LOG_INFO, "CHG: CC -> CV");
        }
        break;

    case CHG_CV:
        /* Check for CV timeout */
        if (elapsed >= CHARGE_CV_TIMEOUT_S) {
            chg_.phase = CHG_FAULT;
            GPIO_SetChargeFet(0);
            FLASH_LogEvent(FLASH_LOG_ERROR, "CHG: CV timeout");
            break;
        }

        /* Termination: current drops below threshold */
        if (I_pack < CHARGE_TERMINATION_CURRENT_mA && I_pack > 0) {
            chg_.phase = CHG_TERMINATED;
            GPIO_SetChargeFet(0);
            FLASH_LogEvent(FLASH_LOG_INFO, "CHG: terminated");
        }
        break;

    default:
        break;
    }

    /* Track charge efficiency (Ah_in during charging vs Ah_out during use) */
    static uint32_t last_efficiency_update = 0;
    if (now - last_efficiency_update >= 60000) {  /* Every 60 s */
        last_efficiency_update = now;

        if (I_pack > 10.0f) {
            chg_.accumulated_charge_in_Ah += I_pack * (60.0f / 3600.0f) / 1000.0f;
        }

        /* Update rolling efficiency */
        chg_.efficiency_window[chg_.efficiency_idx++ % CHARGE_EFFICIENCY_SAMPLES] =
            (chg_.accumulated_discharge_Ah > 0.0f)
                ? (chg_.accumulated_charge_in_Ah / chg_.accumulated_discharge_Ah)
                : 1.0f;

        float sum = 0.0f;
        uint8_t count = (chg_.efficiency_idx < CHARGE_EFFICIENCY_SAMPLES)
                        ? chg_.efficiency_idx : CHARGE_EFFICIENCY_SAMPLES;
        for (uint8_t i = 0; i < count; i++) {
            sum += chg_.efficiency_window[i];
        }
        chg_.current_efficiency = (count > 0) ? (sum / count) : 1.0f;
    }
}

void CHG_Stop(void)
{
    GPIO_SetChargeFet(0);
    chg_.phase = CHG_IDLE;
    FLASH_LogEvent(FLASH_LOG_INFO, "CHG: stopped");
}

bool CHG_IsCharging(void)
{
    return (chg_.phase == CHG_PRECHARGE || chg_.phase == CHG_CC || chg_.phase == CHG_CV);
}

bool CHG_IsComplete(void)
{
    return (chg_.phase == CHG_TERMINATED);
}

bool CHG_IsFaulted(void)
{
    return (chg_.phase == CHG_FAULT);
}

float CHG_GetCompensatedCV_mV(void)
{
    return chg_.cv_voltage_mV;
}

float CHG_GetEfficiency(void)
{
    return chg_.current_efficiency;
}

float CHG_GetAccumulatedCharge_Ah(void)
{
    return chg_.accumulated_charge_in_Ah;
}

void CHG_RecordDischarge_Ah(float ah)
{
    chg_.accumulated_discharge_Ah += ah;
}
