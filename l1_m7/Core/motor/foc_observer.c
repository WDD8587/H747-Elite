/**
  ******************************************************************************
  * @file    foc_observer.c
  * @author  M7 FOC Team
  * @brief   Sliding-mode observer (SMO) for sensorless rotor angle and speed
  *          estimation of PMSM.
  *
  *          Estimates back-EMF (Ealpha, Ebeta) from measured currents and
  *          known voltages using a sliding-mode current observer.  The
  *          estimated back-EMF is fed through a PLL to extract rotor angle
  *          and speed.
  *
  *          Used as backup when the quadrature encoder signal is lost or
  *          invalid (encoder fault detection).
  *
  *          Observer model (PMSM in alpha-beta frame):
  *            d(i_hat)/dt = (1/Ls) * (V - Rs*i_hat - E_hat) - K_smo * sign(i_hat - i)
  *            E_hat = low-pass filtered switching signal
  *            theta_hat = atan2(-Ealpha_hat, Ebeta_hat)
  *            omega_hat = d(theta_hat)/dt (via PLL)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include <string.h>
#include <math.h>

/* Private defines -----------------------------------------------------------*/
#define SMO_LS                  0.000015f     /* 15 uH — motor stator inductance */
#define SMO_RS                  0.050f        /* 50 mOhm — motor stator resistance */
#define SMO_TS                  0.000050f     /* FOC period = 50 us */
#define SMO_ALPHA               0.5f          /* SMO gain */
#define SMO_K_SLIDE             10.0f         /* sliding-mode gain */

/* LPF cutoff for back-EMF filtering */
#define SMO_LPF_CUTOFF          1000.0f       /* 1 kHz */
#define SMO_LPF_ALPHA           (SMO_TS * 6283.185f / (1.0f + SMO_TS * 6283.185f))

/* PLL gains for angle tracking */
#define SMO_PLL_KP              200.0f
#define SMO_PLL_KI              50.0f
#define SMO_PLL_KD              0.0f

/* Status */
#define SMO_STATUS_INIT         0U
#define SMO_STATUS_CONVERGING   1U
#define SMO_STATUS_LOCKED       2U
#define SMO_STATUS_FAULT        3U

/* Convergence thresholds */
#define SMO_CONV_ERR_THRESH     0.5f          /* 0.5 A current error threshold */
#define SMO_CONV_SAMPLES        500U          /* 500 samples = 25 ms */
#define SMO_LOCK_SAMPLES        2000U         /* 2000 samples = 100 ms */

/* Encoder fault: switch threshold */
#define SMO_ENC_FAULT_THRESH    0.3f          /* angle diff > 0.3 rad = fault */

/* ---------------------------------------------------------------------------*/
/*  Observer state                                                            */
/* ---------------------------------------------------------------------------*/
typedef struct {
    /* Estimated currents (alpha-beta frame) */
    float i_alpha_hat;
    float i_beta_hat;

    /* Current error (measured - estimated) */
    float err_alpha;
    float err_beta;

    /* Switching function (sign of current error) */
    float z_alpha;
    float z_beta;

    /* Estimated back-EMF (low-pass filtered switching function) */
    float e_alpha_hat;
    float e_beta_hat;

    /* Estimated angle and speed */
    float theta_hat;      /* electrical angle [rad] */
    float omega_hat;      /* electrical speed [rad/s] */
    float omega_filtered; /* low-pass filtered speed [rad/s] */

    /* PLL state */
    float pll_integral;
    float pll_theta_err;

    /* Convergence state */
    uint8_t status;
    uint32_t converge_count;
    uint32_t lock_count;

    /* Motor parameters */
    float Rs;   /* stator resistance */
    float Ls;   /* stator inductance */
    float Ts;   /* sampling period */
    float Kslide; /* sliding gain */
    float lpf_alpha; /* LPF coefficient */

    /* Enable */
    uint8_t enabled;
    uint8_t encoder_valid; /* set externally by encoder module */
} SMO_HandleTypeDef;

static SMO_HandleTypeDef hsmo;

/* ---------------------------------------------------------------------------*/
/*  Initialisation                                                            */
/* ---------------------------------------------------------------------------*/

/** @brief  Initialise the sliding-mode observer.
  * @param  Rs  Stator resistance [Ohm].
  * @param  Ls  Stator inductance [H].
  * @param  Ts  Sampling period [s] (typically 50 us for 20 kHz FOC).
  */
void SMO_Init(float Rs, float Ls, float Ts)
{
    memset(&hsmo, 0, sizeof(hsmo));

    hsmo.Rs       = (Rs > 0.0f) ? Rs : SMO_RS;
    hsmo.Ls       = (Ls > 0.0f) ? Ls : SMO_LS;
    hsmo.Ts       = (Ts > 0.0f) ? Ts : SMO_TS;
    hsmo.Kslide   = SMO_K_SLIDE;
    hsmo.lpf_alpha = SMO_LPF_ALPHA;

    hsmo.status       = SMO_STATUS_INIT;
    hsmo.enabled      = 1U;
    hsmo.encoder_valid = 1U; /* assume encoder valid until proven otherwise */
}

/** @brief  Set sliding-mode gain (higher = faster convergence but more chatter). */
void SMO_SetGain(float kslide)
{
    hsmo.Kslide = kslide;
}

/* ---------------------------------------------------------------------------*/
/*  Core observer update (called at FOC rate, 20 kHz)                         */
/* ---------------------------------------------------------------------------*/

/** @brief  Run one step of the sliding-mode observer.
  *
  *         Estimates:
  *           i_hat[k+1] = i_hat[k] + Ts/Ls * (V[k] - Rs*i_hat[k] - E_hat[k])
  *                        - Kslide * sign(i_hat[k] - i_meas[k])
  *
  *           z[k] = sign(i_hat[k] - i_meas[k])
  *           E_hat[k+1] = E_hat[k] + LPF_alpha * (z[k] - E_hat[k])
  *
  * @param  v_alpha  Alpha-axis voltage [V].
  * @param  v_beta   Beta-axis voltage [V].
  * @param  i_alpha  Measured alpha-axis current [A].
  * @param  i_beta   Measured beta-axis current [A].
  * @note   Called from FOC ISR or from background at FOC rate.
  */
void SMO_Run(float v_alpha, float v_beta,
             float i_alpha, float i_beta)
{
    if (!hsmo.enabled) return;

    float Ts_Ls = hsmo.Ts / hsmo.Ls;
    float Rs_Ts_Ls = hsmo.Rs * Ts_Ls;

    /* ---- Step 1: Current error ---- */
    hsmo.err_alpha = i_alpha - hsmo.i_alpha_hat;
    hsmo.err_beta  = i_beta  - hsmo.i_beta_hat;

    /* ---- Step 2: Switching function (sign of error) ---- */
    hsmo.z_alpha = (hsmo.err_alpha > 0.0f) ? 1.0f : ((hsmo.err_alpha < 0.0f) ? -1.0f : 0.0f);
    hsmo.z_beta  = (hsmo.err_beta  > 0.0f) ? 1.0f : ((hsmo.err_beta  < 0.0f) ? -1.0f : 0.0f);

    /* ---- Step 3: Update estimated currents ---- */
    hsmo.i_alpha_hat = hsmo.i_alpha_hat
                     + Ts_Ls * (v_alpha - hsmo.Rs * hsmo.i_alpha_hat - hsmo.e_alpha_hat)
                     - hsmo.Kslide * hsmo.z_alpha;

    hsmo.i_beta_hat = hsmo.i_beta_hat
                    + Ts_Ls * (v_beta - hsmo.Rs * hsmo.i_beta_hat - hsmo.e_beta_hat)
                    - hsmo.Kslide * hsmo.z_beta;

    /* ---- Step 4: Update estimated back-EMF (low-pass filtered switching) ---- */
    hsmo.e_alpha_hat = hsmo.e_alpha_hat
                     + hsmo.lpf_alpha * (hsmo.z_alpha - hsmo.e_alpha_hat);
    hsmo.e_beta_hat = hsmo.e_beta_hat
                    + hsmo.lpf_alpha * (hsmo.z_beta - hsmo.e_beta_hat);

    /* ---- Step 5: Compute estimated angle ---- */
    /* theta_hat = atan2(-e_alpha_hat, e_beta_hat) */
    float e_alpha = hsmo.e_alpha_hat;
    float e_beta  = hsmo.e_beta_hat;
    float e_mag = sqrtf(e_alpha * e_alpha + e_beta * e_beta);

    if (e_mag > 0.01f) {
        hsmo.theta_hat = atan2f(-e_alpha, e_beta);
        if (hsmo.theta_hat < 0.0f) hsmo.theta_hat += 6.2831853f;
    }

    /* ---- Step 6: Update PLL for speed estimation ---- */
    /* PLL error: sin(theta_hat - theta_pll) approximated for small errors */
    static float pll_angle = 0.0f;
    float delta = hsmo.theta_hat - pll_angle;
    /* Normalise to [-pi, pi) */
    if (delta > 3.14159265f) delta -= 6.2831853f;
    if (delta < -3.14159265f) delta += 6.2831853f;

    hsmo.pll_theta_err = delta;

    /* PI controller for PLL */
    float pll_p = SMO_PLL_KP * delta;
    hsmo.pll_integral += SMO_PLL_KI * delta * hsmo.Ts;

    /* Limit integrator */
    if (hsmo.pll_integral > 10000.0f) hsmo.pll_integral = 10000.0f;
    if (hsmo.pll_integral < -10000.0f) hsmo.pll_integral = -10000.0f;

    float omega_pll = pll_p + hsmo.pll_integral;
    pll_angle += omega_pll * hsmo.Ts;
    if (pll_angle > 6.2831853f) pll_angle -= 6.2831853f;
    if (pll_angle < 0.0f) pll_angle += 6.2831853f;

    hsmo.omega_hat = omega_pll;

    /* Low-pass filter speed estimate */
    const float omega_lpf = 0.99f;
    hsmo.omega_filtered = omega_lpf * hsmo.omega_filtered
                        + (1.0f - omega_lpf) * hsmo.omega_hat;

    /* ---- Step 7: Convergence state machine ---- */
    float err_mag = fabsf(hsmo.err_alpha) + fabsf(hsmo.err_beta);
    switch (hsmo.status) {
        case SMO_STATUS_INIT:
            if (err_mag < SMO_CONV_ERR_THRESH) {
                hsmo.converge_count++;
                if (hsmo.converge_count >= SMO_CONV_SAMPLES) {
                    hsmo.status = SMO_STATUS_CONVERGING;
                    hsmo.lock_count = 0U;
                }
            } else {
                hsmo.converge_count = 0U;
            }
            break;

        case SMO_STATUS_CONVERGING:
            if (err_mag < SMO_CONV_ERR_THRESH) {
                hsmo.lock_count++;
                if (hsmo.lock_count >= SMO_LOCK_SAMPLES) {
                    hsmo.status = SMO_STATUS_LOCKED;
                }
            } else {
                hsmo.lock_count = 0U;
            }
            break;

        case SMO_STATUS_LOCKED:
            if (err_mag > SMO_CONV_ERR_THRESH * 3.0f) {
                hsmo.status = SMO_STATUS_CONVERGING;
                hsmo.lock_count = 0U;
            }
            break;

        case SMO_STATUS_FAULT:
        default:
            break;
    }

    /* ---- Step 8: Encoder fault detection (cross-check) ---- */
    if (hsmo.encoder_valid && (hsmo.status == SMO_STATUS_LOCKED)) {
        /* external code compares hsmo.theta_hat with encoder theta */
    }
}

/* ---------------------------------------------------------------------------*/
/*  Encoder cross-check                                                       */
/* ---------------------------------------------------------------------------*/

/** @brief  Compare observer angle with encoder angle.
  *         If discrepancy exceeds threshold for enough consecutive samples,
  *         report encoder fault.
  * @param  theta_encoder  Current encoder electrical angle [rad].
  * @return 1 if encoder appears faulty, 0 otherwise.
  */
uint8_t SMO_CheckEncoder(float theta_encoder)
{
    static uint32_t fault_count = 0U;
    const uint32_t fault_thresh = 100U; /* 5 ms at 20 kHz */

    float diff = theta_encoder - hsmo.theta_hat;
    /* Normalise to [-pi, pi) */
    if (diff > 3.14159265f) diff -= 6.2831853f;
    if (diff < -3.14159265f) diff += 6.2831853f;

    if (fabsf(diff) > SMO_ENC_FAULT_THRESH) {
        fault_count++;
        if (fault_count >= fault_thresh) {
            return 1U; /* encoder fault */
        }
    } else {
        if (fault_count > 0U) fault_count--;
    }

    return 0U;
}

/* ---------------------------------------------------------------------------*/
/*  Public API — getters                                                      */
/* ---------------------------------------------------------------------------*/

/** @brief  Get the estimated rotor electrical angle [rad]. */
float SMO_GetTheta(void)
{
    return hsmo.theta_hat;
}

/** @brief  Get the estimated electrical speed [rad/s] (filtered). */
float SMO_GetOmega(void)
{
    return hsmo.omega_filtered;
}

/** @brief  Get the estimated back-EMF alpha component [V]. */
float SMO_GetEAlpha(void)
{
    return hsmo.e_alpha_hat;
}

/** @brief  Get the estimated back-EMF beta component [V]. */
float SMO_GetEBeta(void)
{
    return hsmo.e_beta_hat;
}

/** @brief  Get observer status (0=init, 1=converging, 2=locked, 3=fault). */
uint8_t SMO_GetStatus(void)
{
    return hsmo.status;
}

/** @brief  Set encoder validity flag (call from encoder fault handler). */
void SMO_SetEncoderValid(uint8_t valid)
{
    hsmo.encoder_valid = valid;
}

/** @brief  Get observer-locked flag. */
uint8_t SMO_IsLocked(void)
{
    return (hsmo.status == SMO_STATUS_LOCKED) ? 1U : 0U;
}

/** @brief  Get current estimation error magnitude (for diagnostics). */
float SMO_GetErrorMagnitude(void)
{
    return fabsf(hsmo.err_alpha) + fabsf(hsmo.err_beta);
}

/** @brief  Reset observer to initial state. */
void SMO_Reset(void)
{
    hsmo.i_alpha_hat    = 0.0f;
    hsmo.i_beta_hat     = 0.0f;
    hsmo.e_alpha_hat    = 0.0f;
    hsmo.e_beta_hat     = 0.0f;
    hsmo.theta_hat      = 0.0f;
    hsmo.omega_hat      = 0.0f;
    hsmo.omega_filtered = 0.0f;
    hsmo.pll_integral   = 0.0f;
    hsmo.pll_theta_err  = 0.0f;
    hsmo.status         = SMO_STATUS_INIT;
    hsmo.converge_count = 0U;
    hsmo.lock_count     = 0U;
}

/* ---------------------------------------------------------------------------*/
/*  Parameter update (runtime)                                                */
/* ---------------------------------------------------------------------------*/

/** @brief  Update motor parameters used by the observer (e.g., after thermal
  *         estimation changes Rs).
  */
void SMO_SetMotorParams(float Rs, float Ls)
{
    if (Rs > 0.0f) hsmo.Rs = Rs;
    if (Ls > 0.0f) hsmo.Ls = Ls;
}
