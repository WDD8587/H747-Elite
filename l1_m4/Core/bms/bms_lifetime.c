/**
 * @file    bms_lifetime.c
 * @brief   Cycle count tracking for Li-Ion battery pack.
 *          Increment cycle when cumulative discharge > 80 % FCC.
 *          Store cycle count in DS2431 EEPROM.
 *          Log max / min cell voltage per cycle.
 *          Estimate remaining cycles from SOH trend.
 *
 * @note    Part of STM32H747 BMS subsystem.
 */

#include "bms_lifetime.h"
#include "bms_adc.h"
#include "bms_soh.h"
#include "bms_cell_id.h"
#include "bms_flash.h"
#include "bms_timer.h"

#include <string.h>
#include <math.h>

/* ---------------------------------------------------------------------------
 * Constants
 * ------------------------------------------------------------------------- */
#define LIFETIME_CYCLE_THRESHOLD_FRAC     0.80f   /* 80 % of FCC             */
#define LIFETIME_EOL_SOH_PERCENT          60.0f   /* End of life threshold   */
#define LIFETIME_CYCLE_STORE_INTERVAL      10     /* Store every N cycles    */
#define LIFETIME_LOG_SIZE                  200    /* Max stored cycles       */

/* ---------------------------------------------------------------------------
 * Local state
 * ------------------------------------------------------------------------- */

typedef struct {
    uint32_t total_cycles;
    float    discharge_accumulator_mAh;   /* Running sum towards next cycle */
    float    cycle_max_voltage_mV;
    float    cycle_min_voltage_mV;
    float    rated_capacity_mAh;
    float    soh_trend_slope;             /* SOH % loss per cycle          */
    float    estimated_remaining_cycles;
    uint32_t last_stored_cycle;
    bool     initialized;
} Lifetime_State;

static Lifetime_State life_;

/* ---------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

void LIFETIME_Init(void)
{
    memset(&life_, 0, sizeof(life_));
    life_.total_cycles            = SOH_GetCycleCount();
    life_.cycle_max_voltage_mV    = 0.0f;
    life_.cycle_min_voltage_mV    = 9999.0f;
    life_.rated_capacity_mAh      = 5000.0f;  /* Default, override via CELLID */
    life_.soh_trend_slope         = -0.04f;   /* ~4 % loss per 100 cycles, initial estimate */
    life_.last_stored_cycle       = life_.total_cycles;
    life_.initialized             = true;

    /* Try to read rated capacity from cell ID */
    uint16_t cap = CELLID_GetRatedCapacity_mAh();
    if (cap > 0) {
        life_.rated_capacity_mAh = (float)cap;
    }

    /* Compute remaining cycle estimate */
    LIFETIME_UpdateRemainingEstimate();
}

void LIFETIME_Task(void)
{
    if (!life_.initialized) return;

    static uint32_t last_sample = 0;
    uint32_t now = TIMER_GetTick();
    if (now - last_sample < 1000) return;   /* 1 Hz */
    last_sample = now;

    float I_pack = ADC_GetPackCurrent_mA();  /* Negative = discharge */

    /* Track min/max cell voltages during this cycle */
    for (int i = 0; i < BMS_CELL_COUNT; i++) {
        float v = (float)ADC_GetCellVoltage_mV(i);
        if (v > life_.cycle_max_voltage_mV) life_.cycle_max_voltage_mV = v;
        if (v < life_.cycle_min_voltage_mV) life_.cycle_min_voltage_mV = v;
    }

    /* Accumulate discharge */
    if (I_pack < -50.0f) {  /* > 50 mA discharge */
        float dt_h = 1.0f / 3600.0f;
        float dq_mAh = fabsf(I_pack) * dt_h * 1000.0f;
        life_.discharge_accumulator_mAh += dq_mAh;
    }

    /* Check for cycle completion */
    float fcc = SOH_GetPercent() * 0.01f * life_.rated_capacity_mAh;
    if (fcc < 100.0f) fcc = 100.0f;

    if (life_.discharge_accumulator_mAh >= fcc * LIFETIME_CYCLE_THRESHOLD_FRAC) {
        /* Cycle completed */
        life_.total_cycles++;
        life_.discharge_accumulator_mAh = 0.0f;

        /* Log cycle data */
        LIFETIME_LogCycle();

        /* Reset min/max for next cycle */
        life_.cycle_max_voltage_mV = 0.0f;
        life_.cycle_min_voltage_mV = 9999.0f;

        /* Store to EEPROM periodically */
        if ((life_.total_cycles - life_.last_stored_cycle) >= LIFETIME_CYCLE_STORE_INTERVAL) {
            LIFETIME_SaveCycleCount();
            life_.last_stored_cycle = life_.total_cycles;
        }

        /* Update remaining estimate */
        LIFETIME_UpdateRemainingEstimate();
    }
}

uint32_t LIFETIME_GetTotalCycles(void)
{
    return life_.total_cycles;
}

void LIFETIME_LogCycle(void)
{
    typedef struct __attribute__((packed)) {
        uint32_t cycle_number;
        float    max_voltage_mV;
        float    min_voltage_mV;
        float    soh_at_cycle;
        uint32_t timestamp;
    } Cycle_LogEntry;

    Cycle_LogEntry entry;
    entry.cycle_number   = life_.total_cycles;
    entry.max_voltage_mV = life_.cycle_max_voltage_mV;
    entry.min_voltage_mV = life_.cycle_min_voltage_mV;
    entry.soh_at_cycle   = SOH_GetPercent();
    entry.timestamp      = TIMER_GetTick();

    uint32_t idx = (life_.total_cycles - 1) % LIFETIME_LOG_SIZE;
    FLASH_LogWrite(FLASH_LOG_CYCLE, idx, (const uint8_t *)&entry, sizeof(entry));
}

void LIFETIME_SaveCycleCount(void)
{
    SOH_SetCycleCount(life_.total_cycles);

    /* Also save to DS2431 EEPROM */
    uint8_t cycle_buf[4];
    cycle_buf[0] = (uint8_t)(life_.total_cycles >> 24);
    cycle_buf[1] = (uint8_t)(life_.total_cycles >> 16);
    cycle_buf[2] = (uint8_t)(life_.total_cycles >> 8);
    cycle_buf[3] = (uint8_t)(life_.total_cycles & 0xFF);
    CELLID_WritePackID(cycle_buf, 4);
}

void LIFETIME_UpdateRemainingEstimate(void)
{
    if (life_.total_cycles < 10) {
        /* Too few cycles for a reliable trend; use nominal estimate */
        life_.estimated_remaining_cycles = 500.0f;
        return;
    }

    float current_soh = SOH_GetPercent();
    if (current_soh <= LIFETIME_EOL_SOH_PERCENT) {
        life_.estimated_remaining_cycles = 0.0f;
        return;
    }

    /* SOH trend: loss per cycle */
    float soh_lost = 100.0f - current_soh;
    float soh_per_cycle = soh_lost / (float)life_.total_cycles;
    if (soh_per_cycle < 0.001f) soh_per_cycle = 0.001f;  /* Avoid div-by-zero */

    /* How many more cycles until SOH hits EOL threshold? */
    float remaining_soh = current_soh - LIFETIME_EOL_SOH_PERCENT;
    life_.estimated_remaining_cycles = remaining_soh / soh_per_cycle;

    life_.soh_trend_slope = -soh_per_cycle;
}

float LIFETIME_GetEstimatedRemainingCycles(void)
{
    return life_.estimated_remaining_cycles;
}

float LIFETIME_GetSOHTrendSlope(void)
{
    return life_.soh_trend_slope;
}

bool LIFETIME_IsEndOfLife(void)
{
    return (SOH_GetPercent() <= LIFETIME_EOL_SOH_PERCENT);
}
