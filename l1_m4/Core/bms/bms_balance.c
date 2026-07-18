/**
 * @file    bms_balance.c
 * @brief   Passive cell balancing for Li-Ion battery pack.
 *          Balancing threshold: delta > 30 mV between any cells.
 *          Balance current: 100 mA via external MOSFET + resistor per cell.
 *          Balancing only active during charging near CV phase.
 *          Thermal limit: pause if MOSFET temperature > 85 C.
 *          Balance events logged to flash.
 *
 * @note    Part of STM32H747 BMS subsystem.
 */

#include "bms_balance.h"
#include "bms_adc.h"
#include "bms_ntc.h"
#include "bms_flash.h"
#include "bms_timer.h"
#include "bms_gpio.h"

#include <string.h>
#include <stdio.h>

/* ---------------------------------------------------------------------------
 * Constants
 * ------------------------------------------------------------------------- */
#define BALANCE_THRESHOLD_mV           30U     /* Cell delta trigger          */
#define BALANCE_HYSTERESIS_mV          5U      /* Stop delta after balance    */
#define BALANCE_CURRENT_mA           100       /* Per-cell bleed current      */
#define BALANCE_CV_VOLTAGE_THRESH_mV 4050     /* Near-CV voltage threshold   */
#define BALANCE_CHARGE_CURRENT_MIN_mA 300     /* Min charge current to allow */
#define BALANCE_TEMP_MAX_C            85      /* MOSFET thermal limit        */
#define BALANCE_TEMP_HYSTERESIS_C      3      /* Restart after cooldown      */
#define BALANCE_CYCLE_INTERVAL_MS    1000     /* Balance cycle period        */
#define BALANCE_MAX_DUTY_CYCLES       600     /* Max 10 min per cell/cycle   */
#define LOG_ENTRIES_MAX                100     /* Flash log ring size         */

/* ---------------------------------------------------------------------------
 * Local state
 * ------------------------------------------------------------------------- */

typedef struct {
    uint16_t cell_voltage_mV[BMS_CELL_COUNT];
    uint16_t cell_delta_max_mV;
    uint8_t  balancing_active[BMS_CELL_COUNT];  /* 0/1 per cell MOSFET        */
    uint8_t  duty_counter[BMS_CELL_COUNT];
    uint32_t log_write_index;
    bool     paused_thermal;
    bool     initialized;
} Balance_State;

typedef struct __attribute__((packed)) {
    uint32_t timestamp;
    uint16_t cell_voltages_mV[BMS_CELL_COUNT];
    uint16_t pack_current_mA;
    uint8_t  balancing_mask;
    int8_t   mosfet_temp_C;
} Balance_LogEntry;

static Balance_State bal_;

/* ---------------------------------------------------------------------------
 * Local helpers
 * ------------------------------------------------------------------------- */

static void balance_set_mosfet(uint8_t cell, uint8_t on)
{
    if (cell >= BMS_CELL_COUNT) return;
    if (on) {
        GPIO_SetCellBalance(cell, 1);
        bal_.balancing_active[cell] = 1;
    } else {
        GPIO_SetCellBalance(cell, 0);
        bal_.balancing_active[cell] = 0;
        bal_.duty_counter[cell] = 0;
    }
}

static uint8_t balance_get_mosfet_temp_overlimit(void)
{
    float mosfet_temp = NTC_GetBalanceMosfetTemp();
    if (bal_.paused_thermal) {
        return (mosfet_temp <= (BALANCE_TEMP_MAX_C - BALANCE_TEMP_HYSTERESIS_C)) ? 0 : 1;
    }
    return (mosfet_temp > BALANCE_TEMP_MAX_C) ? 1 : 0;
}

static void balance_write_log(uint8_t mask)
{
    Balance_LogEntry entry;
    entry.timestamp = TIMER_GetTick();
    for (int i = 0; i < BMS_CELL_COUNT; i++) {
        entry.cell_voltages_mV[i] = bal_.cell_voltage_mV[i];
    }
    entry.pack_current_mA = (uint16_t)ADC_GetPackCurrent_mA();
    entry.balancing_mask  = mask;
    entry.mosfet_temp_C   = (int8_t)NTC_GetBalanceMosfetTemp();

    FLASH_LogWrite(FLASH_LOG_BALANCE, bal_.log_write_index,
                   (const uint8_t *)&entry, sizeof(entry));
    bal_.log_write_index = (bal_.log_write_index + 1) % LOG_ENTRIES_MAX;
}

/* ---------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

void BAL_Init(void)
{
    memset(&bal_, 0, sizeof(bal_));

    for (int i = 0; i < BMS_CELL_COUNT; i++) {
        GPIO_SetCellBalance(i, 0);
    }

    bal_.log_write_index = FLASH_LogGetNextIndex(FLASH_LOG_BALANCE, LOG_ENTRIES_MAX);
    bal_.initialized     = true;
}

void BAL_Task(void)
{
    if (!bal_.initialized) return;

    uint32_t now = TIMER_GetTick();
    static uint32_t last_run = 0;
    if (now - last_run < BALANCE_CYCLE_INTERVAL_MS) return;
    last_run = now;

    /* 1. Read all cell voltages */
    for (int i = 0; i < BMS_CELL_COUNT; i++) {
        bal_.cell_voltage_mV[i] = ADC_GetCellVoltage_mV(i);
    }

    /* 2. Compute max delta */
    uint16_t v_min = 0xFFFF, v_max = 0;
    for (int i = 0; i < BMS_CELL_COUNT; i++) {
        if (bal_.cell_voltage_mV[i] < v_min) v_min = bal_.cell_voltage_mV[i];
        if (bal_.cell_voltage_mV[i] > v_max) v_max = bal_.cell_voltage_mV[i];
    }
    bal_.cell_delta_max_mV = (v_max > v_min) ? (v_max - v_min) : 0;

    /* 3. Check thermal limit */
    if (balance_get_mosfet_temp_overlimit()) {
        if (!bal_.paused_thermal) {
            bal_.paused_thermal = true;
            /* Turn off all balancing */
            for (int i = 0; i < BMS_CELL_COUNT; i++) {
                balance_set_mosfet(i, 0);
            }
        }
        return;  /* Stay paused */
    }
    bal_.paused_thermal = false;

    /* 4. Only balance during charging near CV */
    float pack_current = ADC_GetPackCurrent_mA();
    float pack_voltage = ADC_GetPackVoltage_mV();
    bool  charging     = (pack_current > BALANCE_CHARGE_CURRENT_MIN_mA);
    bool  near_cv      = (pack_voltage > BALANCE_CV_VOLTAGE_THRESH_mV);

    if (!charging || !near_cv) {
        /* Not in balance window – turn all off */
        for (int i = 0; i < BMS_CELL_COUNT; i++) {
            balance_set_mosfet(i, 0);
        }
        return;
    }

    /* 5. Determine which cells to balance (highest-voltage cells above threshold) */
    uint8_t balance_mask = 0;
    int     cells_to_balance = 0;

    for (int i = 0; i < BMS_CELL_COUNT; i++) {
        uint16_t delta = bal_.cell_voltage_mV[i] - v_min;

        if (delta >= BALANCE_THRESHOLD_mV &&
            bal_.duty_counter[i] < BALANCE_MAX_DUTY_CYCLES) {
            balance_mask |= (1U << i);
            cells_to_balance++;
        }
    }

    /* 6. Apply balancing – limit by thermal budget */
    const uint8_t max_balance_cells = 4;  /* Thermal design constraint */
    if (cells_to_balance > max_balance_cells) {
        /* Prioritise highest-voltage cells */
        uint8_t priority[BMS_CELL_COUNT];
        for (int i = 0; i < BMS_CELL_COUNT; i++) {
            priority[i] = i;
        }
        /* Bubble-sort by voltage descending */
        for (int i = 0; i < BMS_CELL_COUNT - 1; i++) {
            for (int j = i + 1; j < BMS_CELL_COUNT; j++) {
                if (bal_.cell_voltage_mV[priority[j]] > bal_.cell_voltage_mV[priority[i]]) {
                    uint8_t tmp = priority[i];
                    priority[i] = priority[j];
                    priority[j] = tmp;
                }
            }
        }
        balance_mask = 0;
        for (int i = 0; i < max_balance_cells; i++) {
            uint16_t delta = bal_.cell_voltage_mV[priority[i]] - v_min;
            if (delta >= BALANCE_THRESHOLD_mV) {
                balance_mask |= (1U << priority[i]);
            }
        }
    }

    /* 7. Apply mask */
    for (int i = 0; i < BMS_CELL_COUNT; i++) {
        uint8_t should_balance = (balance_mask >> i) & 1U;
        if (should_balance && !bal_.balancing_active[i]) {
            balance_set_mosfet(i, 1);
            bal_.duty_counter[i] = 0;
        } else if (!should_balance && bal_.balancing_active[i]) {
            /* Check hysteresis */
            uint16_t delta = bal_.cell_voltage_mV[i] - v_min;
            if (delta < (BALANCE_THRESHOLD_mV - BALANCE_HYSTERESIS_mV)) {
                balance_set_mosfet(i, 0);
            }
        }
        if (bal_.balancing_active[i]) {
            bal_.duty_counter[i]++;
        }
    }

    /* 8. Log if any balancing is active */
    if (balance_mask) {
        balance_write_log(balance_mask);
    }
}

uint16_t BAL_GetCellDeltaMax_mV(void)
{
    return bal_.cell_delta_max_mV;
}

uint8_t BAL_GetActiveMask(void)
{
    uint8_t mask = 0;
    for (int i = 0; i < BMS_CELL_COUNT; i++) {
        if (bal_.balancing_active[i]) mask |= (1U << i);
    }
    return mask;
}

bool BAL_IsThermallyPaused(void)
{
    return bal_.paused_thermal;
}
