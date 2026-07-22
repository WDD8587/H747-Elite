/**
  ******************************************************************************
  * @file    foc_field_weaken.c
  * @author  M7 FOC Team
  * @brief   Field-weakening and MTPA (Maximum Torque Per Ampere) control for
  *          interior permanent-magnet synchronous motors (IPMSM) and surface-
  *          mount PMSM (SPMSM).
  *
  *          Field weakening: when Vq approaches the bus-voltage limit,
  *          inject negative Id (demagnetising current) to reduce the back-EMF
  *          and extend the speed range above base speed.
  *
  *          MTPA: lookup-table-based Id optimisation as a function of Iq to
  *          achieve maximum torque per ampere at low-to-medium speeds where
  *          reluctance torque contributes (IPMSM).
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "foc_params.h"
#include <string.h>
#include <math.h>

/* Private defines -----------------------------------------------------------*/
#define FW_TABLE_SIZE           32U    /* entries per table                    */
#define MTPA_TABLE_SIZE         64U

/* Voltage margin [V] — how close to Vdc/sqrt(3) we allow before FW kicks in */
#define FW_VOLTAGE_MARGIN       1.5f

/* FW PI gains (slow outer loop) */
#define FW_KP                   0.01f
#define FW_KI                   0.001f
#define FW_INTEGRAL_MAX         10.0f

/* Minimum Id in field weakening (per-unit of rated current) */
#define FW_ID_MIN_PU           -1.0f

/* ---------------------------------------------------------------------------*/
/*  Field-weakening controller state                                          */
/* ---------------------------------------------------------------------------*/
typedef struct {
    /* Voltage controller (PI that regulates Vq magnitude) */
    float fw_kp;
    float fw_ki;
    float fw_integral;
    float fw_output;         /* Id_ref_fw (negative value) */

    /* Reference voltage magnitude (from DQ voltage commands) */
    float v_q_mag;           /* magnitude of Vq (filtered) */
    float v_dc;              /* DC bus voltage */

    /* Limits */
    float v_max;             /* Vdc / sqrt(3)  (max linear voltage) */
    float id_min;            /* minimum allowed Id (negative, per-unit) */

    /* Enable / status */
    uint8_t  fw_active;      /* 1 when field weakening is active */
    uint8_t  fw_enabled;     /* master enable */

} FieldWeakenState;

static FieldWeakenState hfw;

/* ---------------------------------------------------------------------------*/
/*  MTPA lookup table                                                         */
/* ---------------------------------------------------------------------------*/
typedef struct {
    float iq_pu;             /* Iq per-unit (0 .. 1) */
    float id_mtpa_pu;        /* optimal Id per-unit for MTPA (negative) */
} MTPA_Entry;

/* Pre-computed MTPA table for a typical IPMSM with saliency ratio Lq/Ld ~ 2.5.
   For SPMSM (Lq == Ld), MTPA reduces to Id = 0, so this table would be all 0. */
static const MTPA_Entry mtpa_table[MTPA_TABLE_SIZE] = {
    /* Iq_pu,   Id_mtpa_pu */
    { 0.0000f,  0.0000f },
    { 0.0159f, -0.0002f },
    { 0.0317f, -0.0008f },
    { 0.0476f, -0.0018f },
    { 0.0635f, -0.0032f },
    { 0.0794f, -0.0050f },
    { 0.0952f, -0.0072f },
    { 0.1111f, -0.0098f },
    { 0.1270f, -0.0127f },
    { 0.1429f, -0.0161f },
    { 0.1587f, -0.0198f },
    { 0.1746f, -0.0240f },
    { 0.1905f, -0.0285f },
    { 0.2063f, -0.0335f },
    { 0.2222f, -0.0389f },
    { 0.2381f, -0.0448f },
    { 0.2540f, -0.0511f },
    { 0.2698f, -0.0579f },
    { 0.2857f, -0.0651f },
    { 0.3016f, -0.0728f },
    { 0.3175f, -0.0810f },
    { 0.3333f, -0.0897f },
    { 0.3492f, -0.0989f },
    { 0.3651f, -0.1087f },
    { 0.3810f, -0.1190f },
    { 0.3968f, -0.1298f },
    { 0.4127f, -0.1413f },
    { 0.4286f, -0.1534f },
    { 0.4444f, -0.1661f },
    { 0.4603f, -0.1795f },
    { 0.4762f, -0.1936f },
    { 0.4921f, -0.2084f },
    { 0.5079f, -0.2240f },
    { 0.5238f, -0.2404f },
    { 0.5397f, -0.2577f },
    { 0.5556f, -0.2758f },
    { 0.5714f, -0.2949f },
    { 0.5873f, -0.3149f },
    { 0.6032f, -0.3359f },
    { 0.6190f, -0.3580f },
    { 0.6349f, -0.3812f },
    { 0.6508f, -0.4056f },
    { 0.6667f, -0.4312f },
    { 0.6825f, -0.4581f },
    { 0.6984f, -0.4863f },
    { 0.7143f, -0.5159f },
    { 0.7302f, -0.5470f },
    { 0.7460f, -0.5797f },
    { 0.7619f, -0.6141f },
    { 0.7778f, -0.6503f },
    { 0.7937f, -0.6884f },
    { 0.8095f, -0.7285f },
    { 0.8254f, -0.7708f },
    { 0.8413f, -0.8155f },
    { 0.8571f, -0.8627f },
    { 0.8730f, -0.9127f },
    { 0.8889f, -0.9657f },
    { 0.9048f, -1.0000f },
    { 0.9206f, -1.0000f },
    { 0.9365f, -1.0000f },
    { 0.9524f, -1.0000f },
    { 0.9683f, -1.0000f },
    { 0.9841f, -1.0000f },
    { 1.0000f, -1.0000f },
};

/* ---------------------------------------------------------------------------*/
/*  Initialisation                                                            */
/* ---------------------------------------------------------------------------*/

/** @brief  Initialise field weakening and MTPA modules.
  * @param  v_dc_nominal  Nominal DC bus voltage [V].
  */
void FOC_FW_Init(float v_dc_nominal)
{
    memset(&hfw, 0, sizeof(hfw));

    hfw.fw_kp           = FW_KP;
    hfw.fw_ki           = FW_KI;
    hfw.fw_integral     = 0.0f;
    hfw.fw_output       = 0.0f;
    hfw.v_dc            = v_dc_nominal;
    hfw.v_max           = v_dc_nominal * INV_SQRT3;
    hfw.id_min          = FW_ID_MIN_PU; /* per-unit, scaled later */
    hfw.fw_enabled      = 1U;
    hfw.fw_active       = 0U;
    hfw.v_q_mag         = 0.0f;
}

/** @brief  Update DC bus voltage used for Vmax calculation. */
void FOC_FW_UpdateBusVoltage(float v_dc)
{
    hfw.v_dc  = v_dc;
    hfw.v_max = v_dc * INV_SQRT3;
}

/* ---------------------------------------------------------------------------*/
/*  Field-weakening PI controller                                             */
/* ---------------------------------------------------------------------------*/

/** @brief  Run field weakening controller.
  *
  *          Monitors the magnitude of the Q-axis voltage reference.
  *          When Vq approaches Vmax (DC bus limit), a PI controller
  *          generates a negative Id reference to weaken the rotor field,
  *          allowing higher speed operation.
  *
  * @param  v_q   Q-axis voltage reference from current controller [V].
  * @param  v_d   D-axis voltage reference [V] (used for combined magnitude).
  * @param  iq    Measured Q-axis current [A].
  * @param  omega Electrical speed [rad/s] (used for feed-forward).
  * @return Id reference [A] (negative in field-weakening region).
  */
float FOC_FW_VoltageController(float v_q, float v_d, float iq, float omega)
{
    if (!hfw.fw_enabled) {
        hfw.fw_active  = 0U;
        hfw.fw_output  = 0.0f;
        return 0.0f;
    }

    /* Compute voltage magnitude (low-pass filtered) */
    float v_mag = sqrtf(v_q * v_q + v_d * v_d);
    const float alpha = 0.95f;
    hfw.v_q_mag = alpha * hfw.v_q_mag + (1.0f - alpha) * v_q; /* filtered Vq */

    /* Voltage error (positive when approaching limit) */
    float v_err = v_mag - (hfw.v_max - FW_VOLTAGE_MARGIN);

    if (v_err > 0.0f) {
        hfw.fw_active = 1U;

        /* PI control */
        float p_term = hfw.fw_kp * v_err;
        hfw.fw_integral += hfw.fw_ki * v_err;

        /* Integrator anti-windup */
        if (hfw.fw_integral > FW_INTEGRAL_MAX)  hfw.fw_integral = FW_INTEGRAL_MAX;
        if (hfw.fw_integral < -FW_INTEGRAL_MAX) hfw.fw_integral = -FW_INTEGRAL_MAX;

        hfw.fw_output = -(p_term + hfw.fw_integral); /* negative Id */

        /* Apply minimum Id limit */
        float id_min_scaled = hfw.id_min; /* scaled elsewhere with rated current */
        if (hfw.fw_output < id_min_scaled) {
            hfw.fw_output = id_min_scaled;
        }
        if (hfw.fw_output > 0.0f) {
            hfw.fw_output = 0.0f; /* never inject positive Id via FW */
        }
    } else {
        /* Below voltage limit: reset integrator slowly */
        hfw.fw_integral *= 0.995f;
        hfw.fw_output    = 0.0f;
        hfw.fw_active    = 0U;
    }

    return hfw.fw_output;
}

/* ---------------------------------------------------------------------------*/
/*  MTPA lookup                                                               */
/* ---------------------------------------------------------------------------*/

/** @brief  Look up optimal Id for MTPA (maximum torque per ampere) as a
  *         function of commanded Iq.
  *
  *         For SPMSM (surface-mount PMSM, Ld == Lq), MTPA is simply Id = 0.
  *         For IPMSM (interior PMSM, Lq > Ld), injecting negative Id exploits
  *         reluctance torque to achieve higher torque for the same stator
  *         current magnitude.
  *
  *         Linear interpolation between table entries.
  *
  * @param  iq_ref  Q-axis current reference [A] (signed).
  * @param  i_rated Rated current of the motor [A] (for per-unit conversion).
  * @return Optimal Id reference [A] for MTPA operation.
  */
float FOC_FW_MTPA(float iq_ref, float i_rated)
{
    if (i_rated < 0.001f) return 0.0f;

    /* Convert to per-unit (absolute value) */
    float iq_pu = fabsf(iq_ref) / i_rated;
    if (iq_pu > 1.0f) iq_pu = 1.0f;

    /* Linear interpolation in mtpa_table */
    uint32_t idx_low = 0U;
    uint32_t idx_high = MTPA_TABLE_SIZE - 1U;

    if (iq_pu <= mtpa_table[0].iq_pu) {
        idx_low = idx_high = 0U;
    } else if (iq_pu >= mtpa_table[MTPA_TABLE_SIZE - 1U].iq_pu) {
        idx_low = idx_high = MTPA_TABLE_SIZE - 1U;
    } else {
        /* Binary search */
        uint32_t lo = 0U;
        uint32_t hi = MTPA_TABLE_SIZE - 1U;
        while (hi - lo > 1U) {
            uint32_t mid = (lo + hi) >> 1U;
            if (mtpa_table[mid].iq_pu <= iq_pu) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        idx_low  = lo;
        idx_high = lo + 1U;
    }

    float iq_lo = mtpa_table[idx_low].iq_pu;
    float iq_hi = mtpa_table[idx_high].iq_pu;
    float id_lo = mtpa_table[idx_low].id_mtpa_pu;
    float id_hi = mtpa_table[idx_high].id_mtpa_pu;

    float id_pu;
    if (iq_hi > iq_lo) {
        float frac = (iq_pu - iq_lo) / (iq_hi - iq_lo);
        id_pu = id_lo + frac * (id_hi - id_lo);
    } else {
        id_pu = id_lo;
    }

    /* Convert back to actual current, preserving sign of iq_ref */
    float id_ref = id_pu * i_rated;
    if (iq_ref < 0.0f) {
        id_ref = -id_ref; /* mirror for reverse rotation */
    }

    return id_ref;
}

/* ---------------------------------------------------------------------------*/
/*  Combined Id reference generator                                           */
/* ---------------------------------------------------------------------------*/

/** @brief  Produce the final Id reference by combining MTPA and field-weakening
  *         contributions.
  *
  *         Id_final = Id_mtpa + Id_fw
  *
  *         At low speed: Id_mtpa dominates (optimises torque per amp).
  *         At high speed: Id_fw dominates (extends speed range via demagnetisation).
  *
  * @param  iq_ref    Torque-producing current reference [A].
  * @param  v_q       Q-axis voltage from current controller [V].
  * @param  v_d       D-axis voltage from current controller [V].
  * @param  omega     Electrical speed [rad/s].
  * @param  i_rated   Motor rated current [A].
  * @return Combined Id reference [A].
  */
float FOC_FW_GetIdReference(float iq_ref, float v_q, float v_d,
                             float omega, float i_rated)
{
    /* 1. MTPA contribution (dominant at low speed) */
    float id_mtpa = FOC_FW_MTPA(iq_ref, i_rated);

    /* 2. Field-weakening contribution (dominant at high speed) */
    float id_fw = FOC_FW_VoltageController(v_q, v_d, iq_ref, omega);

    /* 3. Combine — select the more negative of the two */
    float id_ref = (id_mtpa < id_fw) ? id_mtpa : id_fw;

    /* 4. Absolute lower limit */
    float id_abs_min = -FW_ID_MIN_PU * i_rated;
    if (id_ref < id_abs_min) id_ref = id_abs_min;

    return id_ref;
}

/* ---------------------------------------------------------------------------*/
/*  Status and diagnostics                                                    */
/* ---------------------------------------------------------------------------*/

/** @brief  Returns 1 if field weakening is currently active. */
uint8_t FOC_FW_IsActive(void)
{
    return hfw.fw_active;
}

/** @brief  Enable or disable field weakening. */
void FOC_FW_Enable(uint8_t enable)
{
    hfw.fw_enabled = enable ? 1U : 0U;
    if (!hfw.fw_enabled) {
        hfw.fw_integral = 0.0f;
        hfw.fw_output   = 0.0f;
        hfw.fw_active   = 0U;
    }
}

/** @brief  Get the current FW integral term (for debugging). */
float FOC_FW_GetIntegral(void)
{
    return hfw.fw_integral;
}

/** @brief  Get the current FW Id output. */
float FOC_FW_GetOutput(void)
{
    return hfw.fw_output;
}

/** @brief  Override FW PI gains (runtime tuning). */
void FOC_FW_SetGain(float kp, float ki)
{
    if (kp < 0.0f) kp = 0.0f;
    if (ki < 0.0f) ki = 0.0f;
    hfw.fw_kp = kp;
    hfw.fw_ki = ki;
}
