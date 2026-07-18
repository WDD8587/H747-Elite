/**
 * @file    bms_discharge.c
 * @brief   Discharge monitoring for Li-Ion battery pack.
 *          Tracks available capacity (FCC - passed_charge).
 *          Low battery warning at 20 % -> return to dock.
 *          Critical at 5 % -> immediate stop + save state to flash.
 *
 * @note    Part of STM32H747 BMS subsystem.
 */

#include "bms_discharge.h"
#include "bms_adc.h"
#include "bms_soh.h"
#include "bms_protect.h"
#include "bms_gpio.h"
#include "bms_timer.h"
#include "bms_flash.h"

#include <string.h>

/* ---------------------------------------------------------------------------
 * Constants
 * ------------------------------------------------------------------------- */
#define DISCHARGE_SAMPLE_INTERVAL_MS     100     /* 10 Hz         */
#define DISCHARGE_LOW_THRESHOLD_PERCENT   20     /* Low warning   */
#define DISCHARGE_CRITICAL_THRESHOLD_PERCENT  5  /* Critical stop */
#define DISCHARGE_HYSTERESIS_PERCENT       3     /* Recovery hyst */

/* ---------------------------------------------------------------------------
 * Local state
 * ------------------------------------------------------------------------- */

typedef struct {
    float   full_charge_capacity_mAh;   /* FCC from SOH               */
    float   passed_charge_mAh;           /* Discharge since full charge*/
    float   state_of_charge_percent;     /* 0.0 - 100.0                */
    bool    low_warning;                 /* Below 20 %                 */
    bool    critical_stop;               /* Below 5 %                  */
    bool    initialized;
} Discharge_State;

static Discharge_State dis_;

/* ---------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

void DISCH_Init(void)
{
    memset(&dis_, 0, sizeof(dis_));
    dis_.full_charge_capacity_mAh = SOH_GetPercent() * 50.0f;  /* 50 mAh per % */
    dis_.state_of_charge_percent  = 100.0f;
    dis_.passed_charge_mAh        = 0.0f;
    dis_.initialized              = true;
}

void DISCH_Task(void)
{
    if (!dis_.initialized) return;

    static uint32_t last_sample = 0;
    uint32_t now = TIMER_GetTick();
    if (now - last_sample < DISCHARGE_SAMPLE_INTERVAL_MS) return;
    last_sample = now;

    /* Update FCC from SOH (capacity may change with temperature/age) */
    dis_.full_charge_capacity_mAh = SOH_GetPercent() * 50.0f;  /* Scale to mAh */

    float I_pack = ADC_GetPackCurrent_mA();  /* Negative = discharging */

    if (I_pack < -50.0f) {   /* >50 mA discharge current */
        float dt_h = (float)DISCHARGE_SAMPLE_INTERVAL_MS / 3600000.0f;
        float dq_mAh = fabsf(I_pack) * dt_h * 1000.0f;  /* mA * h => mAh */
        dis_.passed_charge_mAh += dq_mAh;

        /* Also report discharge to charge module for efficiency tracking */
        extern void CHG_RecordDischarge_Ah(float ah);
        CHG_RecordDischarge_Ah(dq_mAh / 1000.0f);
    }

    /* Compute SoC */
    if (dis_.full_charge_capacity_mAh > 0.0f) {
        float remaining = dis_.full_charge_capacity_mAh - dis_.passed_charge_mAh;
        if (remaining < 0.0f) remaining = 0.0f;
        dis_.state_of_charge_percent = (remaining / dis_.full_charge_capacity_mAh) * 100.0f;
    } else {
        dis_.state_of_charge_percent = 0.0f;
    }

    /* Clamp */
    if (dis_.state_of_charge_percent > 100.0f) dis_.state_of_charge_percent = 100.0f;

    /* Check thresholds with hysteresis */
    if (dis_.critical_stop) {
        /* In critical stop, wait for recharge above hysteresis */
        if (dis_.state_of_charge_percent >
            (DISCHARGE_CRITICAL_THRESHOLD_PERCENT + DISCHARGE_HYSTERESIS_PERCENT)) {
            dis_.critical_stop = false;
            GPIO_SetDischargeFet(1);
            FLASH_LogEvent(FLASH_LOG_INFO, "DISCH: critical cleared, FET on");
        }
    } else if (dis_.state_of_charge_percent <= DISCHARGE_CRITICAL_THRESHOLD_PERCENT) {
        /* Critical stop */
        GPIO_SetDischargeFet(0);
        dis_.critical_stop = true;
        DISCH_SaveState();
        FLASH_LogEvent(FLASH_LOG_CRITICAL, "DISCH: critical stop, SoC < 5%%");
    }

    if (dis_.low_warning) {
        if (dis_.state_of_charge_percent >
            (DISCHARGE_LOW_THRESHOLD_PERCENT + DISCHARGE_HYSTERESIS_PERCENT)) {
            dis_.low_warning = false;
        }
    } else if (dis_.state_of_charge_percent <= DISCHARGE_LOW_THRESHOLD_PERCENT) {
        if (!dis_.critical_stop) {
            dis_.low_warning = true;
            FLASH_LogEvent(FLASH_LOG_WARNING, "DISCH: low battery, return to dock");
        }
    }
}

float DISCH_GetStateOfCharge(void)
{
    return dis_.state_of_charge_percent;
}

float DISCH_GetPassedCharge_mAh(void)
{
    return dis_.passed_charge_mAh;
}

float DISCH_GetRemainingCapacity_mAh(void)
{
    float rem = dis_.full_charge_capacity_mAh - dis_.passed_charge_mAh;
    return (rem > 0.0f) ? rem : 0.0f;
}

bool DISCH_IsLow(void)
{
    return dis_.low_warning;
}

bool DISCH_IsCritical(void)
{
    return dis_.critical_stop;
}

void DISCH_ResetState(void)
{
    dis_.passed_charge_mAh        = 0.0f;
    dis_.state_of_charge_percent  = 100.0f;
    dis_.low_warning              = false;
    dis_.critical_stop            = false;
    dis_.full_charge_capacity_mAh = SOH_GetPercent() * 50.0f;
}

void DISCH_SaveState(void)
{
    /* Save discharge state to flash for recovery after power loss */
    typedef struct __attribute__((packed)) {
        uint32_t magic;
        float    passed_charge_mAh;
        float    state_of_charge_percent;
        float    full_charge_capacity_mAh;
        uint32_t crc32;
    } Discharge_FlashRecord;

    Discharge_FlashRecord rec;
    rec.magic                  = 0xD15C5000UL;
    rec.passed_charge_mAh      = dis_.passed_charge_mAh;
    rec.state_of_charge_percent = dis_.state_of_charge_percent;
    rec.full_charge_capacity_mAh = dis_.full_charge_capacity_mAh;

    /* Compute CRC */
    uint32_t crc = 0xFFFFFFFFUL;
    const uint8_t *p = (const uint8_t *)&rec;
    for (size_t i = 0; i < offsetof(Discharge_FlashRecord, crc32); i++) {
        crc ^= p[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ ((crc & 1U) ? 0xEDB88320UL : 0UL);
        }
    }
    rec.crc32 = crc ^ 0xFFFFFFFFUL;

    FLASH_Write(FLASH_SECTOR_DISCHARGE, (uint32_t)&rec, sizeof(rec));
}

bool DISCH_RestoreState(void)
{
    typedef struct __attribute__((packed)) {
        uint32_t magic;
        float    passed_charge_mAh;
        float    state_of_charge_percent;
        float    full_charge_capacity_mAh;
        uint32_t crc32;
    } Discharge_FlashRecord;

    Discharge_FlashRecord rec;
    if (!FLASH_Read(FLASH_SECTOR_DISCHARGE, (uint32_t)&rec, sizeof(rec))) {
        return false;
    }

    if (rec.magic != 0xD15C5000UL) return false;

    /* Verify CRC */
    uint32_t crc = 0xFFFFFFFFUL;
    const uint8_t *p = (const uint8_t *)&rec;
    for (size_t i = 0; i < offsetof(Discharge_FlashRecord, crc32); i++) {
        crc ^= p[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ ((crc & 1U) ? 0xEDB88320UL : 0UL);
        }
    }
    if ((crc ^ 0xFFFFFFFFUL) != rec.crc32) return false;

    dis_.passed_charge_mAh       = rec.passed_charge_mAh;
    dis_.state_of_charge_percent = rec.state_of_charge_percent;
    dis_.full_charge_capacity_mAh = rec.full_charge_capacity_mAh;

    return true;
}
