/**
  ******************************************************************************
  * @file    dock_handshake.c
  * @author  H747 Elite Team
  * @brief   Dock detection and handshake state machine.
  *
  *          Sensors:
  *            - IR beacon (38 kHz modulated, TSOP4838 sensor, active-low)
  *            - 19 V rail ADC (divider on ADC3_IN1)
  *
  *          State machine:
  *            SEARCHING -> ALIGNING -> DOCKED -> CHARGING -> STANDBY
  *            Any state -> SEARCHING (19V rail lost > 1 s)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include <string.h>
#include <stdint.h>

/* ---------------------------------------------------------------------------
 * Dock states
 * --------------------------------------------------------------------------- */
typedef enum {
    DOCK_SEARCHING  = 0,
    DOCK_ALIGNING   = 1,
    DOCK_DOCKED     = 2,
    DOCK_CHARGING   = 3,
    DOCK_STANDBY    = 4
} DockState_t;

/* ---------------------------------------------------------------------------
 * Constants
 * --------------------------------------------------------------------------- */
#define ADC_19V_GPIO_PORT           GPIOA
#define ADC_19V_GPIO_PIN            GPIO_PIN_0
#define ADC_19V_ADC                 ADC3
#define ADC_19V_RESOLUTION          ADC_RESOLUTION_12B
#define ADC_19V_VREF_mV             3300U
#define ADC_19V_DIVIDER_RATIO       6.33f
#define ADC_19V_THRESHOLD_mV        15000U
#define ADC_19V_LOST_mV             5000U
#define ADC_19V_SAMPLES             4U

/* IR sensor (TSOP4838, 38 kHz demodulator) */
#define IR_GPIO_PORT                GPIOE
#define IR_GPIO_PIN                 GPIO_PIN_0
#define IR_DEBOUNCE_MS              50U

/* Handshake timeouts */
#define DOCK_LOST_TIMEOUT_MS        1000U
#define DOCK_CHARGE_COMPLETE_RSOC   98U

/* Charger enable GPIO */
#define CHRG_EN_GPIO_PORT           GPIOD
#define CHRG_EN_GPIO_PIN            GPIO_PIN_13

/* Dock LED */
#define DOCK_LED_GPIO_PORT          GPIOD
#define DOCK_LED_GPIO_PIN           GPIO_PIN_14

/* ---------------------------------------------------------------------------
 * Module state
 * --------------------------------------------------------------------------- */
static DockState_t s_dock_state       = DOCK_SEARCHING;
static uint16_t    s_19v_mV           = 0;
static uint8_t     s_ir_detected      = 0;
static uint32_t    s_ir_change_tick   = 0;
static uint32_t    s_state_entry_tick = 0;
static uint32_t    s_adc_sum          = 0;
static uint8_t     s_adc_count        = 0;
static ADC_HandleTypeDef hadc3;

/*============================================================================
 *  ADC3 init
 *============================================================================*/
static void ADC3_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    ADC_ChannelConfTypeDef adc_ch = {0};

    __HAL_RCC_ADC3_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    gpio.Pin   = ADC_19V_GPIO_PIN;
    gpio.Mode  = GPIO_MODE_ANALOG;
    gpio.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(ADC_19V_GPIO_PORT, &gpio);

    hadc3.Instance                   = ADC_19V_ADC;
    hadc3.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc3.Init.Resolution            = ADC_19V_RESOLUTION;
    hadc3.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc3.Init.ContinuousConvMode    = DISABLE;
    hadc3.Init.DiscontinuousConvMode = DISABLE;
    hadc3.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc3.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc3.Init.NbrOfConversion       = 1;
    hadc3.Init.DMAContinuousRequests = DISABLE;
    hadc3.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    hadc3.Init.Overrun               = ADC_OVR_DATA_PRESERVED;
    hadc3.Init.BoostMode             = ENABLE;
    HAL_ADC_Init(&hadc3);

    adc_ch.Channel      = ADC_CHANNEL_1;
    adc_ch.Rank         = ADC_REGULAR_RANK_1;
    adc_ch.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
    adc_ch.SingleDiff   = ADC_SINGLE_ENDED;
    adc_ch.OffsetNumber = ADC_OFFSET_NONE;
    adc_ch.Offset       = 0;
    HAL_ADC_ConfigChannel(&hadc3, &adc_ch);
}

/*============================================================================
 *  ADC3 read (single conversion, blocking)
 *============================================================================*/
static uint16_t ADC3_Read_mV(void)
{
    uint32_t adc_val = 0;
    float    voltage_mV_float;

    if (HAL_ADC_Start(&hadc3) != HAL_OK) return 0;
    if (HAL_ADC_PollForConversion(&hadc3, 10) == HAL_OK) {
        adc_val = HAL_ADC_GetValue(&hadc3);
    }
    HAL_ADC_Stop(&hadc3);

    voltage_mV_float = (float)adc_val * (float)ADC_19V_VREF_mV / 4096.0f;
    voltage_mV_float *= ADC_19V_DIVIDER_RATIO;
    return (uint16_t)(voltage_mV_float + 0.5f);
}

/*============================================================================
 *  IR debounce
 *============================================================================*/
static uint8_t IR_Debounce(void)
{
    uint8_t level = (uint8_t)HAL_GPIO_ReadPin(IR_GPIO_PORT, IR_GPIO_PIN);
    level = (level == GPIO_PIN_RESET) ? 1 : 0;  /* TSOP4838: low = detected */

    if (level != s_ir_detected) {
        if (s_ir_change_tick == 0) {
            s_ir_change_tick = HAL_GetTick();
        } else if ((HAL_GetTick() - s_ir_change_tick) >= IR_DEBOUNCE_MS) {
            s_ir_detected = level;
            s_ir_change_tick = 0;
        }
    } else {
        s_ir_change_tick = 0;
    }
    return s_ir_detected;
}

/*============================================================================
 *  GPIO helpers
 *============================================================================*/
static void CHRGER_Enable(uint8_t enable)
{
    HAL_GPIO_WritePin(CHRG_EN_GPIO_PORT, CHRG_EN_GPIO_PIN,
                      enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
static void DOCK_LED_Set(uint8_t on)
{
    HAL_GPIO_WritePin(DOCK_LED_GPIO_PORT, DOCK_LED_GPIO_PIN,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/*============================================================================
 *  Dock_Init
 *============================================================================*/
void Dock_Init(I2C_HandleTypeDef *i2c)
{
    GPIO_InitTypeDef gpio = {0};
    (void)i2c;

    __HAL_RCC_GPIOD_CLK_ENABLE();

    gpio.Pin   = CHRG_EN_GPIO_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(CHRG_EN_GPIO_PORT, &gpio);
    HAL_GPIO_WritePin(CHRG_EN_GPIO_PORT, CHRG_EN_GPIO_PIN, GPIO_PIN_RESET);

    gpio.Pin   = DOCK_LED_GPIO_PIN;
    HAL_GPIO_Init(DOCK_LED_GPIO_PORT, &gpio);
    HAL_GPIO_WritePin(DOCK_LED_GPIO_PORT, DOCK_LED_GPIO_PIN, GPIO_PIN_RESET);

    ADC3_Init();

    s_dock_state       = DOCK_SEARCHING;
    s_19v_mV           = 0;
    s_ir_detected      = 0;
    s_ir_change_tick   = 0;
    s_state_entry_tick = HAL_GetTick();
}

/*============================================================================
 *  Dock_Run -- run dock state machine (call at 10 Hz)
 *============================================================================*/
uint8_t Dock_Run(I2C_HandleTypeDef *i2c)
{
    uint16_t    adc_mV;
    uint8_t     ir_now;
    uint32_t    now, elapsed;
    DockState_t next_state;
    (void)i2c;

    now = HAL_GetTick();
    elapsed = now - s_state_entry_tick;

    /* ADC averaging */
    s_adc_sum += ADC3_Read_mV();
    s_adc_count++;
    if (s_adc_count >= ADC_19V_SAMPLES) {
        s_19v_mV = (uint16_t)(s_adc_sum / ADC_19V_SAMPLES);
        s_adc_sum = 0;
        s_adc_count = 0;
    }

    ir_now = IR_Debounce();
    next_state = s_dock_state;

    switch (s_dock_state) {

    case DOCK_SEARCHING:
        CHRGER_Enable(0);
        DOCK_LED_Set(0);
        if (ir_now) {
            next_state = DOCK_ALIGNING;
            s_state_entry_tick = now;
        }
        break;

    case DOCK_ALIGNING:
        DOCK_LED_Set(1);
        if (s_19v_mV >= ADC_19V_THRESHOLD_mV && ir_now) {
            next_state = DOCK_DOCKED;
            s_state_entry_tick = now;
        }
        if (!ir_now && elapsed > 500) {
            next_state = DOCK_SEARCHING;
            s_state_entry_tick = now;
        }
        break;

    case DOCK_DOCKED:
        DOCK_LED_Set(1);
        CHRGER_Enable(1);
        if (elapsed > 200) {
            next_state = DOCK_CHARGING;
            s_state_entry_tick = now;
        }
        if (s_19v_mV < ADC_19V_LOST_mV && elapsed > DOCK_LOST_TIMEOUT_MS) {
            next_state = DOCK_SEARCHING;
            CHRGER_Enable(0);
            s_state_entry_tick = now;
        }
        break;

    case DOCK_CHARGING:
        DOCK_LED_Set((now / 500) & 1);
        {
            extern uint8_t g_shared_rsoc;
            uint8_t rsoc = ((volatile uint8_t *)0x38000000UL)[5];
            if (rsoc >= DOCK_CHARGE_COMPLETE_RSOC) {
                next_state = DOCK_STANDBY;
                s_state_entry_tick = now;
            }
        }
        if (elapsed > 21600000UL) {
            next_state = DOCK_STANDBY;
            s_state_entry_tick = now;
        }
        if (s_19v_mV < ADC_19V_LOST_mV && elapsed > DOCK_LOST_TIMEOUT_MS) {
            next_state = DOCK_SEARCHING;
            CHRGER_Enable(0);
            s_state_entry_tick = now;
        }
        break;

    case DOCK_STANDBY:
        DOCK_LED_Set(1);
        if (elapsed > 60000) {
            extern void Enter_Dock_Standby(void);
            Enter_Dock_Standby();
            next_state = DOCK_SEARCHING;
            s_state_entry_tick = now;
        }
        if (s_19v_mV < ADC_19V_LOST_mV) {
            next_state = DOCK_SEARCHING;
            CHRGER_Enable(0);
            s_state_entry_tick = now;
        }
        break;

    default:
        next_state = DOCK_SEARCHING;
        s_state_entry_tick = now;
        break;
    }

    s_dock_state = next_state;
    return (uint8_t)s_dock_state;
}

/*============================================================================
 *  Dock_Get19V_mV
 *============================================================================*/
uint16_t Dock_Get19V_mV(void) { return s_19v_mV; }
uint8_t  Dock_GetState(void)  { return (uint8_t)s_dock_state; }
