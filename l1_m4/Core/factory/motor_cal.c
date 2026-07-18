/**
 * @file    motor_cal.c
 * @brief   Motor calibration sequence for production line.
 *
 * Procedures:
 *   - Phase resistance: inject small DC, measure I.
 *   - Phase inductance: inject high-freq AC, measure L.
 *   - Back-EMF constant: spin motor externally, measure V.
 *   - Encoder offset: lock rotor to phase A, read encoder.
 *
 * All calibration parameters saved to flash.
 *
 * @note    Part of STM32H747 factory calibration suite.
 */

#include "motor_cal.h"
#include "motor_pwm.h"
#include "motor_adc.h"
#include "motor_encoder.h"
#include "factory_flash.h"
#include "factory_timer.h"
#include "factory_uart.h"

#include <string.h>
#include <math.h>

/* ---------------------------------------------------------------------------
 * Constants
 * ------------------------------------------------------------------------- */
#define CAL_ADC_SAMPLES                 64
#define CAL_DC_INJECT_CURRENT_A        0.5f   /* Small DC for R measurement  */
#define CAL_DC_INJECT_DURATION_MS      100
#define CAL_AC_INJECT_FREQ_HZ         1000.0f /* 1 kHz for L measurement    */
#define CAL_AC_INJECT_AMPLITUDE_V      1.0f
#define CAL_AC_INJECT_CYCLES            10
#define CAL_BEMF_SPEED_RPM            500.0f  /* External spin speed        */
#define CAL_BEMF_SETTLE_TIME_MS       2000
#define CAL_ENCODER_LOCK_CURRENT_A    1.0f
#define CAL_ENCODER_LOCK_DURATION_MS  500
#define CAL_PARAM_MAGIC              0x4D4F544FUL  /* "MOTO" */

/* ---------------------------------------------------------------------------
 * Calibration parameter structure (saved to flash)
 * ------------------------------------------------------------------------- */
typedef struct __attribute__((packed)) {
    uint32_t magic;
    float    phase_R_mOhm;
    float    phase_L_uH;
    float    bemf_const_Vpk_per_krpm;
    float    encoder_offset_deg;
    float    encoder_offset_rad;
    uint32_t crc32;
    uint8_t  reserved[32];
} Motor_CalParams;

static Motor_CalParams mc_;

/* ---------------------------------------------------------------------------
 * CRC-32 helper
 * ------------------------------------------------------------------------- */
static uint32_t cal_crc32(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ ((crc & 1UL) ? 0xEDB88320UL : 0UL);
        }
    }
    return crc ^ 0xFFFFFFFFUL;
}

/* ---------------------------------------------------------------------------
 * Measurement: phase resistance
 * Inject small DC current between two phases, measure voltage -> R
 * ------------------------------------------------------------------------- */
static bool cal_measure_phase_R(void)
{
    float v_sum = 0.0f;
    float i_sum = 0.0f;

    /* Enable PWM on phase A high, phase B low (DC injection) */
    PWM_SetDuty(PWM_PHASE_A_HIGH, 0.2f);      /* 20 % duty */
    PWM_SetDuty(PWM_PHASE_A_LOW,  0.0f);
    PWM_SetDuty(PWM_PHASE_B_HIGH, 0.0f);
    PWM_SetDuty(PWM_PHASE_B_LOW,  0.2f);
    PWM_SetDuty(PWM_PHASE_C_HIGH, 0.0f);
    PWM_SetDuty(PWM_PHASE_C_LOW,  0.0f);
    PWM_Enable();

    TIMER_DelayMs(CAL_DC_INJECT_DURATION_MS);

    for (int i = 0; i < CAL_ADC_SAMPLES; i++) {
        v_sum += ADC_GetPhaseVoltage_V(0);  /* Phase A-B */
        i_sum += ADC_GetPhaseCurrent_A(0);
        TIMER_DelayUs(100);
    }

    PWM_Disable();
    TIMER_DelayMs(50);  /* Cooldown */

    float v_avg = v_sum / (float)CAL_ADC_SAMPLES;
    float i_avg = i_sum / (float)CAL_ADC_SAMPLES;

    if (i_avg < 0.01f) return false;

    /* R = V / I. For phase-to-phase measurement, divide by 2 for per-phase */
    mc_.phase_R_mOhm = (v_avg / i_avg) * 1000.0f / 2.0f;

    return (mc_.phase_R_mOhm > 0.1f && mc_.phase_R_mOhm < 500.0f);
}

/* ---------------------------------------------------------------------------
 * Measurement: phase inductance
 * Inject AC voltage, measure phase current amplitude -> L = V / (2*pi*f * I)
 * ------------------------------------------------------------------------- */
static bool cal_measure_phase_L(void)
{
    float freq_rad = 2.0f * 3.14159265f * CAL_AC_INJECT_FREQ_HZ;

    /* Sinusoidal voltage injection on phase A-B */
    PWM_EnableSineInjection(PWM_PHASE_AB, CAL_AC_INJECT_FREQ_HZ,
                            CAL_AC_INJECT_AMPLITUDE_V);
    TIMER_DelayMs(50);  /* Settle */

    float i_peak = 0.0f;
    for (int cyc = 0; cyc < CAL_AC_INJECT_CYCLES; cyc++) {
        for (int s = 0; s < 20; s++) {
            float i = ADC_GetPhaseCurrent_A(0);
            if (fabsf(i) > i_peak) i_peak = fabsf(i);
            TIMER_DelayUs(50);
        }
    }

    PWM_Disable();
    TIMER_DelayMs(50);

    if (i_peak < 0.001f) return false;

    /* X_L = V_peak / I_peak = 2*pi*f*L */
    float XL = CAL_AC_INJECT_AMPLITUDE_V / i_peak;
    mc_.phase_L_uH = (XL / freq_rad) * 1e6f;  /* Henry to uH */

    return (mc_.phase_L_uH > 0.1f && mc_.phase_L_uH < 1000.0f);
}

/* ---------------------------------------------------------------------------
 * Measurement: back-EMF constant
 * Spin motor externally at known speed, measure generated voltage
 * ------------------------------------------------------------------------- */
static bool cal_measure_bemf(void)
{
    UART_Print("MOTOR_CAL: spinning motor externally at %.0f RPM...\r\n",
               CAL_BEMF_SPEED_RPM);
    TIMER_DelayMs(CAL_BEMF_SETTLE_TIME_MS);  /* Wait for stable spin */

    float v_sum = 0.0f;
    for (int i = 0; i < CAL_ADC_SAMPLES; i++) {
        v_sum += ADC_GetBemfVoltage_V();
        TIMER_DelayMs(10);
    }
    float v_avg = v_sum / (float)CAL_ADC_SAMPLES;

    /* Ke = V_line_to_line_peak / (electrical speed in krpm)
     * For a PMSM, Ke (Vpk / krpm) = V_measured / (rpm / 1000) */
    mc_.bemf_const_Vpk_per_krpm = v_avg / (CAL_BEMF_SPEED_RPM / 1000.0f);

    return (mc_.bemf_const_Vpk_per_krpm > 0.1f && mc_.bemf_const_Vpk_per_krpm < 500.0f);
}

/* ---------------------------------------------------------------------------
 * Measurement: encoder offset
 * Lock rotor to phase A (align d-axis with phase A), read encoder angle
 * ------------------------------------------------------------------------- */
static bool cal_measure_encoder_offset(void)
{
    /* Apply DC current to phase A (align rotor magnetic axis to phase A) */
    PWM_SetDuty(PWM_PHASE_A_HIGH, 0.1f);
    PWM_SetDuty(PWM_PHASE_A_LOW,  0.0f);
    PWM_SetDuty(PWM_PHASE_B_HIGH, 0.0f);
    PWM_SetDuty(PWM_PHASE_B_LOW,  0.0f);
    PWM_SetDuty(PWM_PHASE_C_HIGH, 0.0f);
    PWM_SetDuty(PWM_PHASE_C_LOW,  0.0f);
    PWM_Enable();

    TIMER_DelayMs(CAL_ENCODER_LOCK_DURATION_MS);

    /* Read encoder multiple times and average */
    float enc_sum = 0.0f;
    for (int i = 0; i < 20; i++) {
        enc_sum += ENCODER_GetPosition_deg();
        TIMER_DelayMs(10);
    }
    PWM_Disable();

    float enc_avg = enc_sum / 20.0f;

    /* When rotor is aligned with phase A, encoder should read 0 (electrical).
     * The offset is the actual encoder reading (assumes pole-pair = 1 for simplicity). */
    mc_.encoder_offset_deg = enc_avg;
    mc_.encoder_offset_rad = enc_avg * (3.14159265f / 180.0f);

    /* Normalise to 0-360 */
    while (mc_.encoder_offset_deg < 0.0f)  mc_.encoder_offset_deg += 360.0f;
    while (mc_.encoder_offset_deg >= 360.0f) mc_.encoder_offset_deg -= 360.0f;

    return true;
}

/* ---------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

bool MOTORCAL_RunAll(void)
{
    UART_Print("MOTOR_CAL: starting production calibration...\r\n");
    memset(&mc_, 0, sizeof(mc_));
    mc_.magic = CAL_PARAM_MAGIC;

    bool ok = true;

    ok &= cal_measure_phase_R();
    UART_Print("  Phase R = %.3f mOhm  [%s]\r\n",
               mc_.phase_R_mOhm, ok ? "PASS" : "FAIL");

    ok &= cal_measure_phase_L();
    UART_Print("  Phase L = %.3f uH    [%s]\r\n",
               mc_.phase_L_uH, ok ? "PASS" : "FAIL");

    ok &= cal_measure_bemf();
    UART_Print("  Ke = %.3f Vpk/krpm    [%s]\r\n",
               mc_.bemf_const_Vpk_per_krpm, ok ? "PASS" : "FAIL");

    ok &= cal_measure_encoder_offset();
    UART_Print("  Encoder offset = %.1f deg [%s]\r\n",
               mc_.encoder_offset_deg, ok ? "PASS" : "FAIL");

    if (!ok) {
        UART_Print("MOTOR_CAL: FAILED\r\n");
        return false;
    }

    /* Save to flash */
    mc_.crc32 = cal_crc32((const uint8_t *)&mc_,
                           offsetof(Motor_CalParams, crc32));
    FACTORY_FLASH_Write(FACTORY_SECTOR_MOTOR, (uint32_t)&mc_, sizeof(mc_));

    UART_Print("MOTOR_CAL: saved to flash. PASS\r\n");
    return true;
}

bool MOTORCAL_LoadFromFlash(void)
{
    if (!FACTORY_FLASH_Read(FACTORY_SECTOR_MOTOR, (uint32_t)&mc_, sizeof(mc_))) {
        return false;
    }
    if (mc_.magic != CAL_PARAM_MAGIC) return false;

    uint32_t expected_crc = cal_crc32((const uint8_t *)&mc_,
                                       offsetof(Motor_CalParams, crc32));
    return (expected_crc == mc_.crc32);
}

float MOTORCAL_GetPhaseResistance_mOhm(void)
{
    return mc_.phase_R_mOhm;
}

float MOTORCAL_GetPhaseInductance_uH(void)
{
    return mc_.phase_L_uH;
}

float MOTORCAL_GetBemfConstant_Vpk_per_krpm(void)
{
    return mc_.bemf_const_Vpk_per_krpm;
}

float MOTORCAL_GetEncoderOffset_deg(void)
{
    return mc_.encoder_offset_deg;
}

float MOTORCAL_GetEncoderOffset_rad(void)
{
    return mc_.encoder_offset_rad;
}
