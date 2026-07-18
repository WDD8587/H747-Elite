/**
 * @file    bms_precharge.c
 * @brief   Precharge sequence before closing main discharge FET.
 *          Procedure:
 *            1. Close precharge FET (with 10R series resistor)
 *            2. Wait for Vload > 90 % Vbat
 *            3. Close main discharge FET
 *            4. Open precharge FET
 *          Timeout: 100 ms per attempt.
 *          Retry 3x on failure, then enter FAULT state.
 *
 * @note    Part of STM32H747 BMS subsystem.
 */

#include "bms_precharge.h"
#include "bms_gpio.h"
#include "bms_adc.h"
#include "bms_timer.h"
#include "bms_flash.h"

#include <string.h>

/* ---------------------------------------------------------------------------
 * Constants
 * ------------------------------------------------------------------------- */
#define PRECHARGE_TIMEOUT_MS               100U
#define PRECHARGE_MAX_RETRIES                3U
#define PRECHARGE_THRESHOLD_RATIO          0.90f   /* 90 % of Vbat           */
#define PRECHARGE_SAMPLE_INTERVAL_MS         2U    /* 500 Hz check rate       */
#define PRECHARGE_RETRY_COOLDOWN_MS        500U    /* Cool-off between retries*/

/* ---------------------------------------------------------------------------
 * Local state
 * ------------------------------------------------------------------------- */
typedef enum {
    PRECHARGE_IDLE,
    PRECHARGE_ACTIVE,
    PRECHARGE_COMPLETE,
    PRECHARGE_FAILED
} Precharge_Phase;

typedef struct {
    Precharge_Phase phase;
    uint8_t         retry_count;
    uint32_t        start_tick;
    bool            initialized;
} Precharge_State;

static Precharge_State pre_;

/* ---------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

void PRECHARGE_Init(void)
{
    memset(&pre_, 0, sizeof(pre_));
    pre_.phase       = PRECHARGE_IDLE;
    pre_.retry_count = 0;
    pre_.initialized = true;

    /* Ensure both FETs start off */
    GPIO_SetPrechargeFet(0);
    GPIO_SetDischargeFet(0);
}

bool PRECHARGE_Start(void)
{
    if (!pre_.initialized) return false;
    if (pre_.phase == PRECHARGE_ACTIVE) return false; /* Already in progress */

    pre_.phase       = PRECHARGE_ACTIVE;
    pre_.retry_count = 0;
    pre_.start_tick  = TIMER_GetTick();

    /* Close precharge FET */
    GPIO_SetPrechargeFet(1);
    GPIO_SetDischargeFet(0);

    return true;
}

PRECHARGE_Result PRECHARGE_Task(void)
{
    if (!pre_.initialized) return PRECHARGE_RESULT_BUSY;

    if (pre_.phase != PRECHARGE_ACTIVE) {
        return (pre_.phase == PRECHARGE_COMPLETE) ? PRECHARGE_RESULT_SUCCESS
                                                   : PRECHARGE_RESULT_BUSY;
    }

    uint32_t elapsed = TIMER_GetTick() - pre_.start_tick;

    /* Check timeout */
    if (elapsed >= PRECHARGE_TIMEOUT_MS) {
        /* Turn off precharge FET */
        GPIO_SetPrechargeFet(0);
        pre_.retry_count++;

        if (pre_.retry_count >= PRECHARGE_MAX_RETRIES) {
            pre_.phase = PRECHARGE_FAILED;
            FLASH_LogEvent(FLASH_LOG_ERROR, "PRECHARGE: max retries exceeded");
            return PRECHARGE_RESULT_FAILED;
        }

        /* Wait cooldown then auto-retry */
        pre_.start_tick = TIMER_GetTick();  /* Reset timer for cooldown */
        /* The caller must keep calling Task; after cooldown we re-close FET */
        return PRECHARGE_RESULT_RETRY;
    }

    /* Check if cooldown period is active (between retries) */
    if (pre_.retry_count > 0 && elapsed < PRECHARGE_RETRY_COOLDOWN_MS) {
        /* Still in cooldown – precharge FET remains off */
        return PRECHARGE_RESULT_RETRY;
    }

    /* Re-close precharge FET if in cooldown */
    if (pre_.retry_count > 0 && elapsed >= PRECHARGE_RETRY_COOLDOWN_MS) {
        GPIO_SetPrechargeFet(1);
        /* Transition back to active monitoring */
        /* We use a trick: subtract cooldown from start_tick to keep timeout check */
        pre_.start_tick = TIMER_GetTick() - PRECHARGE_RETRY_COOLDOWN_MS;
        pre_.retry_count = 0;  /* Clear retry flag for this attempt */
    }

    /* Sample load voltage */
    float V_bat  = ADC_GetPackVoltage_mV();
    float V_load = ADC_GetLoadVoltage_mV();

    if (V_bat > 100.0f && (V_load / V_bat) >= PRECHARGE_THRESHOLD_RATIO) {
        /* Success: close main FET, open precharge FET */
        GPIO_SetDischargeFet(1);
        /* Small delay to let main FET stabilise */
        TIMER_DelayUs(500);
        GPIO_SetPrechargeFet(0);

        pre_.phase = PRECHARGE_COMPLETE;
        FLASH_LogEvent(FLASH_LOG_INFO, "PRECHARGE: completed successfully");
        return PRECHARGE_RESULT_SUCCESS;
    }

    return PRECHARGE_RESULT_BUSY;
}

bool PRECHARGE_IsComplete(void)
{
    return (pre_.phase == PRECHARGE_COMPLETE);
}

bool PRECHARGE_IsFaulted(void)
{
    return (pre_.phase == PRECHARGE_FAILED);
}

void PRECHARGE_Reset(void)
{
    GPIO_SetPrechargeFet(0);
    GPIO_SetDischargeFet(0);
    pre_.phase       = PRECHARGE_IDLE;
    pre_.retry_count = 0;
}
