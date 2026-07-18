/**
 * @file    bms_soh.c
 * @brief   State-of-Health estimation for Li-Ion battery pack.
 *          Coulomb counting with temperature compensation.
 *          Internal resistance tracking via delta-V / delta-I.
 *          SOH = current_capacity / rated_capacity * 100.
 *          Saves retained parameters to SRAM2 (Stop2 retained).
 *
 * @note    Part of STM32H747 BMS subsystem.
 */

#include "bms_soh.h"
#include "bms_ntc.h"
#include "bms_adc.h"
#include "bms_flash.h"
#include "bms_timer.h"

#include <string.h>
#include <math.h>

/* ---------------------------------------------------------------------------
 * Constants
 * ------------------------------------------------------------------------- */
#define SOH_RATED_CAPACITY_mAh      5000U       /* Nominal pack capacity       */
#define SOH_CURRENT_THRESHOLD_mA     200        /* Minimum current to count    */
#define SOH_SAMPLE_INTERVAL_MS        10        /* 100 Hz Coulomb counter      */
#define SOH_TEMP_REF_C                25         /* Reference temperature       */
#define SOH_TEMP_COEFF_ppm_perC     4000         /* 0.4 % / C capacity change   */
#define SOH_IR_CALIB_PULSE_A        2.0f         /* Discharge pulse amplitude   */
#define SOH_IR_CALIB_PULSE_MS       100          /* Pulse duration              */
#define SOH_IR_MAX_mOhm             200           /* End-of-life threshold       */
#define SOH_SRAM2_BACKUP_MAGIC    0xB055A5A5UL   /* Validity marker             */

/* ---------------------------------------------------------------------------
 * Local state
 * ------------------------------------------------------------------------- */

typedef struct {
    float    coulomb_count_Ah;          /* Accumulated charge               */
    float    temperature_comp_capacity;  /* Temp-compensated capacity [mAh]  */
    float    internal_resistance_mOhm;   /* Last measured Ri                 */
    float    soh_percent;                /* 100.0 = like new                */
    uint32_t sample_count;
    uint32_t last_save_tick;
    bool     valid;
} SOH_State;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    float    soh_percent;
    float    internal_resistance_mOhm;
    float    coulomb_count_Ah;
    uint32_t cycle_count;
    uint32_t crc32;
} SOH_SRAM2_Backup;

static SOH_State          soh_;
static SOH_SRAM2_Backup  *soh_backup_ = (SOH_SRAM2_Backup *)BMS_SRAM2_BACKUP_ADDR;

/* ---------------------------------------------------------------------------
 * Local helpers
 * ------------------------------------------------------------------------- */

/**
 * @brief  Simple CRC-32 over the backup structure (excluding the CRC field).
 */
static uint32_t soh_crc32(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ ((crc & 1U) ? 0xEDB88320UL : 0UL);
        }
    }
    return crc ^ 0xFFFFFFFFU;
}

/**
 * @brief  Temperature compensation factor.
 *         Capacity fades ~0.4 % per deg-C below reference.
 *         Returns multiplier (1.0 at 25 C).
 */
static inline float soh_temp_factor(float pack_temp_C)
{
    float delta = SOH_TEMP_REF_C - pack_temp_C;
    /* 4000 ppm / C -> 0.004 per C */
    return 1.0f + delta * (SOH_TEMP_COEFF_ppm_perC * 1e-6f);
}

/* ---------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

void SOH_Init(void)
{
    memset(&soh_, 0, sizeof(soh_));

    /* Attempt restore from SRAM2 */
    if (soh_backup_->magic == SOH_SRAM2_BACKUP_MAGIC) {
        uint32_t expected_crc = soh_crc32((const uint8_t *)soh_backup_,
                                          offsetof(SOH_SRAM2_Backup, crc32));
        if (expected_crc == soh_backup_->crc32) {
            soh_.soh_percent           = soh_backup_->soh_percent;
            soh_.internal_resistance_mOhm = soh_backup_->internal_resistance_mOhm;
            soh_.coulomb_count_Ah      = soh_backup_->coulomb_count_Ah;
            soh_.valid                 = true;
        }
    }

    if (!soh_.valid) {
        /* First boot / corrupt backup – default to 100 % */
        soh_.soh_percent              = 100.0f;
        soh_.internal_resistance_mOhm = 30.0f;   /* typical fresh Ri */
        soh_.coulomb_count_Ah         = 0.0f;
        soh_.valid                    = true;
    }

    soh_.temperature_comp_capacity = SOH_RATED_CAPACITY_mAh * soh_temp_factor(NTC_GetPackTemp());
    soh_.last_save_tick            = TIMER_GetTick();
}

void SOH_Update(void)
{
    if (!soh_.valid) return;

    uint32_t now = TIMER_GetTick();
    if (now - soh_.sample_count * SOH_SAMPLE_INTERVAL_MS < SOH_SAMPLE_INTERVAL_MS) {
        return;  /* Not yet time for a new sample */
    }

    float I_mA = ADC_GetPackCurrent_mA();   /* negative = discharge */

    /* Coulomb counting – integrate current over dt */
    float dt_h = (float)SOH_SAMPLE_INTERVAL_MS / 3600000.0f;
    soh_.coulomb_count_Ah += fabsf(I_mA) * dt_h / 1000.0f;  /* mA * h / 1000 => Ah */

    /* Update temperature-compensated capacity */
    float pack_temp = NTC_GetPackTemp();
    soh_.temperature_comp_capacity = SOH_RATED_CAPACITY_mAh * soh_temp_factor(pack_temp);

    /* SOH estimation based on accumulated capacity through full cycles */
    if (soh_.temperature_comp_capacity > 0.0f) {
        /* Capacity throughput method: SOH decays with cumulative Ah */
        float throughput_loss = soh_.coulomb_count_Ah / (SOH_RATED_CAPACITY_mAh / 1000.0f * 500.0f);
        /* 500 full equivalent cycles before reaching 80 % */
        float estimated = 100.0f - throughput_loss * 20.0f;  /* 20 % loss over 500 cycles */
        if (estimated < 0.0f) estimated = 0.0f;
        /* Blend with previous value (low-pass) */
        soh_.soh_percent = 0.95f * soh_.soh_percent + 0.05f * estimated;
    }

    soh_.sample_count++;
}

bool SOH_MeasureInternalResistance(void)
{
    /* Measure Ri during a discharge pulse: R = deltaV / deltaI */
    float I_before = ADC_GetPackCurrent_mA();
    float V_before = ADC_GetPackVoltage_mV();

    /* Request discharge pulse (caller must ensure load is enabled) */
    if (!ADC_RequestDischargePulse(SOH_IR_CALIB_PULSE_A, SOH_IR_CALIB_PULSE_MS)) {
        return false;
    }

    TIMER_DelayMs(SOH_IR_CALIB_PULSE_MS / 2);   /* settle */

    float I_after  = ADC_GetPackCurrent_mA();
    float V_after  = ADC_GetPackVoltage_mV();

    float deltaI = fabsf(I_after - I_before);
    float deltaV = fabsf(V_after - V_before);

    if (deltaI < 10.0f) {
        return false;  /* Not enough current change to measure reliably */
    }

    soh_.internal_resistance_mOhm = (deltaV / deltaI) * 1000.0f;

    /* Temperature-normalise to 25 C */
    float pack_temp = NTC_GetPackTemp();
    float temp_factor = 1.0f + (pack_temp - SOH_TEMP_REF_C) * 0.003f; /* ~0.3 % / C */
    soh_.internal_resistance_mOhm /= temp_factor;

    return true;
}

float SOH_GetPercent(void)
{
    return soh_.soh_percent;
}

float SOH_GetInternalResistance_mOhm(void)
{
    return soh_.internal_resistance_mOhm;
}

float SOH_GetAccumulatedCapacity_Ah(void)
{
    return soh_.coulomb_count_Ah;
}

void SOH_SaveToSRAM2(void)
{
    soh_backup_->magic           = SOH_SRAM2_BACKUP_MAGIC;
    soh_backup_->soh_percent     = soh_.soh_percent;
    soh_backup_->internal_resistance_mOhm = soh_.internal_resistance_mOhm;
    soh_backup_->coulomb_count_Ah        = soh_.coulomb_count_Ah;
    /* cycle_count is written by bms_lifetime */
    soh_backup_->crc32 = soh_crc32((const uint8_t *)soh_backup_,
                                    offsetof(SOH_SRAM2_Backup, crc32));
}

void SOH_SaveToFlash(void)
{
    SOH_SRAM2_Backup copy;
    memcpy(&copy, soh_backup_, sizeof(copy));
    FLASH_Write(FLASH_SECTOR_BMS, (uint32_t)&copy, sizeof(copy));
}

void SOH_RestoreFromFlash(void)
{
    SOH_SRAM2_Backup copy;
    if (FLASH_Read(FLASH_SECTOR_BMS, (uint32_t)&copy, sizeof(copy))) {
        uint32_t expected_crc = soh_crc32((const uint8_t *)&copy,
                                          offsetof(SOH_SRAM2_Backup, crc32));
        if (copy.magic == SOH_SRAM2_BACKUP_MAGIC && expected_crc == copy.crc32) {
            soh_.soh_percent           = copy.soh_percent;
            soh_.internal_resistance_mOhm = copy.internal_resistance_mOhm;
            soh_.coulomb_count_Ah      = copy.coulomb_count_Ah;
        }
    }
}

void SOH_SetCycleCount(uint32_t cycles)
{
    soh_backup_->cycle_count = cycles;
}

uint32_t SOH_GetCycleCount(void)
{
    return soh_backup_->cycle_count;
}
