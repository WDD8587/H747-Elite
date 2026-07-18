/**
 * @file    bms_protect.c
 * @brief   Hardware / firmware protection layer for Li-Ion BMS.
 *
 * Protections (all with auto-recovery hysteresis):
 *   SC  – Short circuit:   I > 20 A    → MOSFET off in < 200 us (comparator)
 *   OC  – Overcurrent:     I > 10 A, 5s persist → FET off
 *   OV  – Overvoltage:     cell > 4.25 V → charge FET off
 *   UV  – Undervoltage:    cell < 2.80 V → discharge FET off
 *   OT  – Overtemp:        NTC > 65 C  → both FETs off
 *   UT  – Undertemp:       NTC < 0 C   → charge FET off
 *
 * @note   Part of STM32H747 BMS subsystem.
 */

#include "bms_protect.h"
#include "bms_adc.h"
#include "bms_ntc.h"
#include "bms_timer.h"
#include "bms_gpio.h"
#include "bms_flash.h"

#include <string.h>

/* ---------------------------------------------------------------------------
 * Thresholds
 * ------------------------------------------------------------------------- */
#define PROT_SC_CURRENT_mA          20000   /* 20 A short-circuit              */
#define PROT_SC_DEBOUNCE_us           150   /* < 200 us response               */
#define PROT_OC_CURRENT_mA          10000   /* 10 A overcurrent                */
#define PROT_OC_TRIP_TIME_MS         5000   /* 5 s persist                     */
#define PROT_OC_HYSTERESIS_mA        2000   /* Recovery below 8 A              */
#define PROT_OV_CELL_mV              4250   /* Overvoltage per cell            */
#define PROT_OV_HYSTERESIS_mV         100   /* Recovery below 4.15 V           */
#define PROT_UV_CELL_mV              2800   /* Undervoltage per cell           */
#define PROT_UV_HYSTERESIS_mV         100   /* Recovery above 2.90 V           */
#define PROT_OT_TEMP_C                 65   /* Overtemp                        */
#define PROT_OT_HYSTERESIS_C            5   /* Recovery below 60 C             */
#define PROT_UT_TEMP_C                  0   /* Undertemp                       */
#define PROT_UT_HYSTERESIS_C            5   /* Recovery above 5 C              */
#define PROT_FAULT_LOG_MAX             50

/* ---------------------------------------------------------------------------
 * Protection state flags
 * ------------------------------------------------------------------------- */
typedef struct {
    bool charge_fet_on;
    bool discharge_fet_on;
    bool precharge_active;

    /* Trip flags (latched until auto-recovery conditions met) */
    bool sc_tripped;
    bool oc_tripped;
    bool ov_tripped;
    bool uv_tripped;
    bool ot_tripped;
    bool ut_tripped;

    /* Debounce / persist timers */
    uint32_t oc_start_ms;
    uint32_t sc_assert_time_us;

    /* Logging */
    uint32_t fault_log_index;

    bool initialized;
} Protect_State;

typedef struct __attribute__((packed)) {
    uint32_t timestamp;
    uint8_t  fault_type;    /* 0=SC, 1=OC, 2=OV, 3=UV, 4=OT, 5=UT */
    uint16_t cell_voltage_mV;
    int16_t  pack_current_mA;
    int8_t   pack_temp_C;
    uint8_t  fet_state;     /* bit0=charge, bit1=discharge */
} Protect_FaultLog;

static Protect_State prot_;

/* ---------------------------------------------------------------------------
 * FET control helpers
 * ------------------------------------------------------------------------- */
static void protect_charge_fet_off(void)
{
    GPIO_SetChargeFet(0);
    prot_.charge_fet_on = false;
}

static void protect_discharge_fet_off(void)
{
    GPIO_SetDischargeFet(0);
    prot_.discharge_fet_on = false;
}

static void protect_charge_fet_on(void)
{
    GPIO_SetChargeFet(1);
    prot_.charge_fet_on = true;
}

static void protect_discharge_fet_on(void)
{
    GPIO_SetDischargeFet(1);
    prot_.discharge_fet_on = true;
}

static void protect_both_fets_off(void)
{
    protect_charge_fet_off();
    protect_discharge_fet_off();
}

/* ---------------------------------------------------------------------------
 * Fault logging
 * ------------------------------------------------------------------------- */
static void protect_log_fault(uint8_t fault_type, uint16_t v_cell, int16_t i_pack, int8_t t_pack)
{
    Protect_FaultLog entry;
    entry.timestamp       = TIMER_GetTick();
    entry.fault_type      = fault_type;
    entry.cell_voltage_mV = v_cell;
    entry.pack_current_mA = i_pack;
    entry.pack_temp_C     = t_pack;
    entry.fet_state       = (prot_.charge_fet_on ? 1 : 0) | (prot_.discharge_fet_on ? 2 : 0);

    FLASH_LogWrite(FLASH_LOG_PROTECT, prot_.fault_log_index,
                   (const uint8_t *)&entry, sizeof(entry));
    prot_.fault_log_index = (prot_.fault_log_index + 1) % PROT_FAULT_LOG_MAX;
}

/* ---------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

void PROT_Init(void)
{
    memset(&prot_, 0, sizeof(prot_));
    prot_.fault_log_index = FLASH_LogGetNextIndex(FLASH_LOG_PROTECT, PROT_FAULT_LOG_MAX);
    prot_.initialized     = true;

    /* Start with FETs off. Caller will run precharge / normal start. */
    protect_both_fets_off();
}

void PROT_ShortCircuitHandler(void)
{
    /* Called from comparator ISR – must be within 200 us */
    float I_pack = ADC_GetPackCurrent_mA();

    if (I_pack > PROT_SC_CURRENT_mA) {
        protect_both_fets_off();
        prot_.sc_tripped = true;
        prot_.sc_assert_time_us = TIMER_GetMicros();
        protect_log_fault(0, ADC_GetCellVoltage_mV(0), (int16_t)I_pack, (int8_t)NTC_GetPackTemp());
    }
}

void PROT_Task(void)
{
    if (!prot_.initialized) return;

    float I_pack       = ADC_GetPackCurrent_mA();
    float pack_temp    = NTC_GetPackTemp();
    uint16_t v_min     = 0xFFFF;
    uint16_t v_max     = 0;

    for (int i = 0; i < BMS_CELL_COUNT; i++) {
        uint16_t v = ADC_GetCellVoltage_mV(i);
        if (v < v_min) v_min = v;
        if (v > v_max) v_max = v;
    }

    /* -----------------------------------------------------------------------
     * SC – Short Circuit (debounced)
     * ----------------------------------------------------------------------- */
    if (prot_.sc_tripped) {
        /* Auto-recovery: wait for current to drop to safe level, then re-enable */
        if (I_pack < (PROT_SC_CURRENT_mA / 2)) {
            uint32_t elapsed_us = TIMER_GetMicros() - prot_.sc_assert_time_us;
            if (elapsed_us > 1000000UL) {  /* 1 s cool-off */
                prot_.sc_tripped = false;
                /* Discharge FET re-enabled by state machine */
            }
        }
    } else {
        if (I_pack > PROT_SC_CURRENT_mA) {
            protect_both_fets_off();
            prot_.sc_tripped = true;
            prot_.sc_assert_time_us = TIMER_GetMicros();
            protect_log_fault(0, v_max, (int16_t)I_pack, (int8_t)pack_temp);
        }
    }

    /* -----------------------------------------------------------------------
     * OC – Overcurrent (5 s persistent)
     * ----------------------------------------------------------------------- */
    if (prot_.oc_tripped) {
        if (I_pack < (PROT_OC_CURRENT_mA - PROT_OC_HYSTERESIS_mA)) {
            prot_.oc_tripped = false;
            protect_discharge_fet_on();
        }
    } else {
        if (I_pack > PROT_OC_CURRENT_mA) {
            if (prot_.oc_start_ms == 0) {
                prot_.oc_start_ms = TIMER_GetTick();
            } else if ((TIMER_GetTick() - prot_.oc_start_ms) >= (uint32_t)PROT_OC_TRIP_TIME_MS) {
                protect_discharge_fet_off();
                prot_.oc_tripped = true;
                prot_.oc_start_ms = 0;
                protect_log_fault(1, v_max, (int16_t)I_pack, (int8_t)pack_temp);
            }
        } else {
            prot_.oc_start_ms = 0;  /* Reset timer if current subsides */
        }
    }

    /* -----------------------------------------------------------------------
     * OV – Overvoltage (any cell > 4.25 V)
     * ----------------------------------------------------------------------- */
    if (prot_.ov_tripped) {
        if (v_max <= (PROT_OV_CELL_mV - PROT_OV_HYSTERESIS_mV)) {
            prot_.ov_tripped = false;
            protect_charge_fet_on();
        }
    } else {
        if (v_max >= PROT_OV_CELL_mV) {
            protect_charge_fet_off();
            prot_.ov_tripped = true;
            protect_log_fault(2, v_max, (int16_t)I_pack, (int8_t)pack_temp);
        }
    }

    /* -----------------------------------------------------------------------
     * UV – Undervoltage (any cell < 2.80 V)
     * ----------------------------------------------------------------------- */
    if (prot_.uv_tripped) {
        if (v_min >= (PROT_UV_CELL_mV + PROT_UV_HYSTERESIS_mV)) {
            prot_.uv_tripped = false;
            protect_discharge_fet_on();
        }
    } else {
        if (v_min <= PROT_UV_CELL_mV) {
            protect_discharge_fet_off();
            prot_.uv_tripped = true;
            protect_log_fault(3, v_min, (int16_t)I_pack, (int8_t)pack_temp);
        }
    }

    /* -----------------------------------------------------------------------
     * OT – Overtemp (NTC > 65 C)
     * ----------------------------------------------------------------------- */
    if (prot_.ot_tripped) {
        if (pack_temp <= (PROT_OT_TEMP_C - PROT_OT_HYSTERESIS_C)) {
            prot_.ot_tripped = false;
            protect_charge_fet_on();
            protect_discharge_fet_on();
        }
    } else {
        if (pack_temp >= PROT_OT_TEMP_C) {
            protect_both_fets_off();
            prot_.ot_tripped = true;
            protect_log_fault(4, v_max, (int16_t)I_pack, (int8_t)pack_temp);
        }
    }

    /* -----------------------------------------------------------------------
     * UT – Undertemp (NTC < 0 C)
     * ----------------------------------------------------------------------- */
    if (prot_.ut_tripped) {
        if (pack_temp >= (PROT_UT_TEMP_C + PROT_UT_HYSTERESIS_C)) {
            prot_.ut_tripped = false;
            protect_charge_fet_on();
        }
    } else {
        if (pack_temp <= PROT_UT_TEMP_C) {
            protect_charge_fet_off();
            prot_.ut_tripped = true;
            protect_log_fault(5, v_max, (int16_t)I_pack, (int8_t)pack_temp);
        }
    }
}

/* ---------------------------------------------------------------------------
 * Status query
 * ------------------------------------------------------------------------- */

bool PROT_IsChargeFetOn(void)  { return prot_.charge_fet_on; }
bool PROT_IsDischargeFetOn(void) { return prot_.discharge_fet_on; }
bool PROT_IsFaulted(void)
{
    return prot_.sc_tripped || prot_.oc_tripped || prot_.ov_tripped ||
           prot_.uv_tripped || prot_.ot_tripped || prot_.ut_tripped;
}

uint8_t PROT_GetFaultFlags(void)
{
    uint8_t flags = 0;
    if (prot_.sc_tripped) flags |= (1 << 0);
    if (prot_.oc_tripped) flags |= (1 << 1);
    if (prot_.ov_tripped) flags |= (1 << 2);
    if (prot_.uv_tripped) flags |= (1 << 3);
    if (prot_.ot_tripped) flags |= (1 << 4);
    if (prot_.ut_tripped) flags |= (1 << 5);
    return flags;
}
