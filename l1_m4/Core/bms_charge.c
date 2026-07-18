/**
  ******************************************************************************
  * @file    bms_charge.c
  * @author  H747 Elite Team
  * @brief   4S lithium charging profiles with OVP/OCP/OTP protection.
  *
  *          State machine:
  *            TRICKLE -> CC_0_5C -> CC_1C -> CC_2C -> CV_16_8V -> DONE
  *            Any state -> FAULT (on protection event)
  *            FAULT -> TRICKLE (on recovery)
  *
  *          Motor stall derate: when stall signal asserted, charge target
  *          current is multiplied by 0.7 to reduce electrical load.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>

/* ---------------------------------------------------------------------------
 * Charge states
 * --------------------------------------------------------------------------- */
typedef enum {
    CC_0_5C    = 0,
    CC_1C      = 1,
    CC_2C      = 2,
    CV_16_8V   = 3,
    TRICKLE    = 4,
    DONE       = 5,
    FAULT      = 6
} ChargeState_t;

/* ---------------------------------------------------------------------------
 * BmsData_t
 * --------------------------------------------------------------------------- */
typedef struct {
    uint16_t voltage_mV;       /* pack voltage, mV                     */
    int16_t  current_mA;       /* pack current, mA (neg = discharge)   */
    uint8_t  rsoc;             /* Relative State-Of-Charge 0-100       */
    int16_t  temp_dK;          /* temperature, deci-Kelvin             */
    uint16_t cell_mV[4];       /* individual cell voltages, mV         */
    uint8_t  valid;            /* 1 if all reads succeeded             */
} BmsData_t;

/* ---------------------------------------------------------------------------
 * 4S LiPo / Li-ion constants
 * --------------------------------------------------------------------------- */
#define CELLS_SERIAL            4U

#define CELL_CV_mV             4200U   /* 4.20 V per cell (full)       */
#define CELL_TRICKLE_mV        2900U   /* 2.90 V -- minimum for CC     */
#define CELL_MIN_mV            3000U   /* 3.00 V -- normal operating   */
#define CELL_OVP_mV            4250U   /* 4.25 V -- overvoltage        */
#define CELL_RECHARGE_mV       4100U   /* 4.10 V -- restart from DONE  */

#define PACK_CV_mV             16800U  /* 4 x 4.20 V                   */
#define PACK_OVP_mV            17000U  /* 4 x 4.25 V                   */
#define PACK_UVLO_mV           12000U  /* 3.00 V * 4 -- under-voltage  */
#define PACK_RECHARGE_mV       16400U  /* restart charging             */

#define CAPACITY_MAH           5000U   /* nominal pack capacity        */

/* Charge currents at various rates */
#define I_TRICKLE_mA           250U    /* 0.05C                        */
#define I_0_5C_mA              2500U   /* 0.5C                         */
#define I_1C_mA                5000U   /* 1C                           */
#define I_2C_mA                10000U  /* 2C                           */
#define I_TERM_mA              500U    /* termination (C/10)           */

/* Protection thresholds */
#define TEMP_OTP_dK            3180U   /* 45.0 C = 318.0 K             */
#define TEMP_CHG_MIN_dK        2730U   /* 0.0 C                        */
#define CELL_DELTA_MAX_mV      300U    /* max imbalance                */

/* Derate factor on motor stall */
#define STALL_DERATE           0.70f

/* Timeout for CC stages (seconds) */
#define CC0_TIMEOUT_S          600U    /* 10 min                       */
#define CC1_TIMEOUT_S          900U    /* 15 min                       */
#define CC2_TIMEOUT_S          600U    /* 10 min                       */
#define CV_TIMEOUT_S           1800U   /* 30 min                       */

/* ---------------------------------------------------------------------------
 * Module state
 * --------------------------------------------------------------------------- */
static uint32_t s_stage_start_tick = 0;
static uint8_t  s_stall_flag       = 0;
static uint8_t  s_fault_reason     = 0;

static uint32_t ticks_s(void) { return HAL_GetTick() / 1000U; }

/*============================================================================
 *  ChargeFSM -- run the charging state machine
 *
 *  Parameters:
 *    state    -- current state (in/out)
 *    bms      -- BMS data snapshot
 *    i_target -- output: target charge current in A
 *============================================================================*/
void ChargeFSM(ChargeState_t *state, BmsData_t *bms, float *i_target)
{
    uint8_t  cell_ovp = 0, cell_uv = 0, cell_imb = 0;
    uint16_t cell_min = 9999, cell_max = 0;
    uint32_t elapsed  = 0;

    if (state == NULL || bms == NULL || i_target == NULL) return;

    *i_target = 0.0f;

    /* Derive per-cell statistics */
    for (uint32_t i = 0; i < CELLS_SERIAL; i++) {
        if (bms->cell_mV[i] > cell_max) cell_max = bms->cell_mV[i];
        if (bms->cell_mV[i] < cell_min) cell_min = bms->cell_mV[i];
        if (bms->cell_mV[i] >= CELL_OVP_mV)  cell_ovp = 1;
        if (bms->cell_mV[i] < CELL_TRICKLE_mV &&
            bms->cell_mV[i] > 100) cell_uv = 1;
    }

    if ((cell_max > cell_min) &&
        ((uint16_t)(cell_max - cell_min) > CELL_DELTA_MAX_mV)) {
        cell_imb = 1;
    }

    if (s_stage_start_tick == 0) s_stage_start_tick = ticks_s();
    elapsed = ticks_s() - s_stage_start_tick;

    /* ---- Protection checks (run in all states) ---- */
    if (cell_ovp || bms->voltage_mV >= PACK_OVP_mV) {
        *state = FAULT; s_fault_reason = 1;
    }
    if (bms->temp_dK >= TEMP_OTP_dK && bms->valid) {
        *state = FAULT; s_fault_reason = 3;
    }
    if (bms->temp_dK <= TEMP_CHG_MIN_dK && bms->valid) {
        *state = FAULT; s_fault_reason = 4;
    }

    /* ---- State machine ---- */
    switch (*state) {

    case TRICKLE:
        *i_target = (float)I_TRICKLE_mA;
        if (cell_min >= CELL_MIN_mV &&
            bms->voltage_mV >= PACK_UVLO_mV && !cell_ovp) {
            *state = CC_0_5C;
            s_stage_start_tick = ticks_s();
        }
        break;

    case CC_0_5C:
        *i_target = (float)I_0_5C_mA;
        if (elapsed >= CC0_TIMEOUT_S || bms->voltage_mV >= 14000U) {
            *state = CC_1C;
            s_stage_start_tick = ticks_s();
        }
        if (cell_min < CELL_TRICKLE_mV) {
            *state = TRICKLE;
            s_stage_start_tick = ticks_s();
        }
        break;

    case CC_1C:
        *i_target = (float)I_1C_mA;
        if (cell_max >= 4150U || elapsed >= CC1_TIMEOUT_S) {
            *state = CV_16_8V;
            s_stage_start_tick = ticks_s();
        }
        if (cell_min < CELL_TRICKLE_mV) {
            *state = TRICKLE;
            s_stage_start_tick = ticks_s();
        }
        break;

    case CC_2C:
        *i_target = (float)I_2C_mA;
        if (cell_max >= 4150U || elapsed >= CC2_TIMEOUT_S) {
            *state = CV_16_8V;
            s_stage_start_tick = ticks_s();
        }
        if (cell_imb) *i_target = (float)I_1C_mA;
        break;

    case CV_16_8V:
        *i_target = (float)I_1C_mA;
        if (bms->voltage_mV >= PACK_CV_mV) {
            float v_err = (float)(bms->voltage_mV - PACK_CV_mV);
            if (v_err > 0.0f) {
                *i_target = (float)I_1C_mA * (1.0f - v_err / 500.0f);
                if (*i_target < (float)I_TERM_mA) *i_target = 0.0f;
            }
        }
        if (bms->current_mA > 0 && bms->current_mA <= (int16_t)I_TERM_mA) {
            *state = DONE;
            s_stage_start_tick = ticks_s();
        }
        if (elapsed >= CV_TIMEOUT_S) {
            *state = DONE;
            s_stage_start_tick = ticks_s();
        }
        break;

    case DONE:
        *i_target = 0.0f;
        if (bms->voltage_mV <= PACK_RECHARGE_mV && bms->rsoc < 95) {
            *state = CC_0_5C;
            s_stage_start_tick = ticks_s();
        }
        break;

    case FAULT:
        *i_target = 0.0f;
        if (!cell_ovp &&
            bms->voltage_mV < PACK_OVP_mV &&
            bms->temp_dK < (int16_t)(TEMP_OTP_dK - 50) &&
            bms->temp_dK > (int16_t)(TEMP_CHG_MIN_dK + 20)) {
            *state = TRICKLE;
            s_stage_start_tick = ticks_s();
            s_fault_reason = 0;
        }
        break;

    default:
        *state = FAULT;
        break;
    }

    /* Stall derate */
    if (s_stall_flag && *i_target > 0.0f) {
        *i_target *= STALL_DERATE;
    }
    /* Cell imbalance derate */
    if (cell_imb && *i_target > (float)I_1C_mA) {
        *i_target = (float)I_1C_mA;
    }
}

/*============================================================================
 *  Charge_SetStall
 *============================================================================*/
void Charge_SetStall(uint8_t active)
{
    s_stall_flag = (active) ? 1 : 0;
}

/*============================================================================
 *  Charge_GetFaultReason
 *============================================================================*/
uint8_t Charge_GetFaultReason(void)
{
    return s_fault_reason;
}

/*============================================================================
 *  Charge_Init
 *============================================================================*/
void Charge_Init(void)
{
    s_stage_start_tick = 0;
    s_stall_flag       = 0;
    s_fault_reason     = 0;
}

/*============================================================================
 *  Charge_GetTargetCurrentIq
 *============================================================================*/
float Charge_GetTargetCurrentIq(float i_target_mA)
{
    const float IQ_PER_AMP = 0.10f;
    return i_target_mA * IQ_PER_AMP / 1000.0f;
}
