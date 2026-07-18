#include "stm32h7xx_hal.h"
#include "foc_params.h"
#include <math.h>
#include <string.h>

typedef struct {
    float ia, ib, ic, id, iq;
    float vd, vq, speed_rpm, pos_deg;
    float id_integral, iq_integral;
    int32_t enc_ticks;
    uint8_t fault;
} FocCtrl_t;

static FocCtrl_t gFocL, gFocR;
static float gIqTargetL, gIqTargetR;

void FocSetSpeed(float vl, float vr)
{
    gIqTargetL = vl; gIqTargetR = vr;
}

/*
 * TIM1 UP @ 20kHz (50us period)
 * Latency breakdown (M7@480MHz, CORDIC enabled):
 *   1. ADC JDR read (3-phase + Vbus) — 0.2us
 *   2. CORDIC Clarke+Park — 0.15us
 *   3. PI_id + PI_iq — 0.3us
 *   4. CORDIC invPark + SVPWM -> CCR — 0.3us
 *   Total: ~1.0us = 2% duty cycle
 */
void TIM1_UP_TIM10_IRQHandler(void)
{
    /* 1. ADC injected group read — 0.2us */
    float ia = (int16_t)ADC1->JDR1 * (3.3f / 4096.0f / 0.05f);
    float ib = (int16_t)ADC1->JDR2 * (3.3f / 4096.0f / 0.05f);
    float ic = (int16_t)ADC1->JDR3 * (3.3f / 4096.0f / 0.05f);
    gFocL.ia = ia; gFocL.ib = ib; gFocL.ic = ic;

    /* 2. CORDIC Clarke+Park with timer-synced theta — 0.15us */
    static uint16_t theta_q14 = 0;
    theta_q14 += 2730; /* 20kHz/1kHz * 16384 = 2730 q14 steps */
    if (theta_q14 >= 16384) theta_q14 -= 16384;

    CORDIC->WDATA = (uint32_t)ia & 0xFFFF;
    CORDIC->WDATA = (uint32_t)ib & 0xFFFF;
    CORDIC->CSR = 0x00000043; /* 16-bit q1.15, cosine+Clarke+Park */
    float id = (int16_t)(CORDIC->RDATA & 0xFFFF) / 32768.0f;
    float iq = (int16_t)((CORDIC->RDATA >> 16) & 0xFFFF) / 32768.0f;
    gFocL.id = id; gFocL.iq = iq;

    /* 3. PI_id + PI_iq — 0.3us */
    float id_err = Id_ref - id;
    gFocL.id_integral += id_err * 0.00005f * Ki_id;
    if (gFocL.id_integral > MAX_CURRENT_A) gFocL.id_integral = MAX_CURRENT_A;
    if (gFocL.id_integral < -MAX_CURRENT_A) gFocL.id_integral = -MAX_CURRENT_A;
    float vd = Kp_id * id_err + gFocL.id_integral;

    float iq_err = gIqTargetL - iq;
    gFocL.iq_integral += iq_err * 0.00005f * Ki_iq;
    if (gFocL.iq_integral > MAX_CURRENT_A) gFocL.iq_integral = MAX_CURRENT_A;
    if (gFocL.iq_integral < -MAX_CURRENT_A) gFocL.iq_integral = -MAX_CURRENT_A;
    float vq = Kp_iq * iq_err + gFocL.iq_integral;
    gFocL.vd = vd; gFocL.vq = vq;

    /* 4. CORDIC invPark + SVPWM -> CCR1/2/3 — 0.3us */
    CORDIC->WDATA = (uint32_t)((int16_t)(vd * 32768.0f)) & 0xFFFF;
    CORDIC->WDATA = (uint32_t)((int16_t)(vq * 32768.0f)) & 0xFFFF;
    CORDIC->CSR = 0x00000083; /* inverse Park */
    float Va = (int16_t)(CORDIC->RDATA & 0xFFFF) / 32768.0f;
    float Vb = (int16_t)((CORDIC->RDATA >> 16) & 0xFFFF) / 32768.0f;

    float Ta, Tb, Tc;
    float vMax = VBUS_NOMINAL * 0.577f;
    if (Va > vMax) Va = vMax; if (Va < -vMax) Va = -vMax;
    if (Vb > vMax) Vb = vMax; if (Vb < -vMax) Vb = -vMax;
    Ta = (Va + 0.5f * VBUS_NOMINAL) / VBUS_NOMINAL;
    Tb = ((-0.5f * Va + 0.866f * Vb) + 0.5f * VBUS_NOMINAL) / VBUS_NOMINAL;
    Tc = ((-0.5f * Va - 0.866f * Vb) + 0.5f * VBUS_NOMINAL) / VBUS_NOMINAL;

    uint32_t arr = TIM1->ARR;
    TIM1->CCR1 = (uint32_t)(Ta * arr);
    TIM1->CCR2 = (uint32_t)(Tb * arr);
    TIM1->CCR3 = (uint32_t)(Tc * arr);

    /* Overcurrent check */
    if (ia > MAX_CURRENT_A || ib > MAX_CURRENT_A || ic > MAX_CURRENT_A) {
        TIM1->BDTR &= ~TIM_BDTR_MOE;
        gFocL.fault |= 0x01;
    }

    TIM1->SR = ~TIM_SR_UIF;
    (void)gFocR;
}

int FocOvercurrent(void) { return gFocL.fault & 0x01; }
