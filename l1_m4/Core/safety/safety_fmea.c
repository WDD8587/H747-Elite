/**
  ******************************************************************************
  * @file    safety_fmea.c
  * @author  M4 Safety Team
  * @brief   FMEA (Failure Mode and Effects Analysis) table for all 47 identified
  *          failure modes on the H747 Elite robot platform.
  *
  *          Each entry contains:
  *            - Component / subsystem
  *            - Failure mode description
  *            - Effect on the system
  *            - Severity rating (1-10, 10 = catastrophic)
  *            - Detection method
  *            - Mitigation / response
  *
  *          The table is a static const structure for zero runtime overhead.
  *          Runtime lookup by failure ID returns the corresponding entry.
  *
  *          Categories covered:
  *            M7 FOC (1-10),  M4 Safety (11-20), Sensors (21-30),
  *            Actuators (31-40), Power/Comms (41-47)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define FMEA_MAX_ENTRIES            47U
#define FMEA_DESCRIPTION_MAX_LEN    48U
#define FMEA_EFFECT_MAX_LEN         48U
#define FMEA_DETECTION_MAX_LEN      32U
#define FMEA_MITIGATION_MAX_LEN     48U

/* ---------------------------------------------------------------------------*/
/*  FMEA entry structure                                                      */
/* ---------------------------------------------------------------------------*/
typedef struct {
    uint16_t  id;                              /* unique failure ID (1-47) */
    uint8_t   severity;                        /* 1 (minor) .. 10 (catastrophic) */
    const char component[24];                  /* component name */
    const char failure_mode[FMEA_DESCRIPTION_MAX_LEN];
    const char effect[FMEA_EFFECT_MAX_LEN];
    const char detection[FMEA_DETECTION_MAX_LEN];
    const char mitigation[FMEA_MITIGATION_MAX_LEN];
} FMEA_EntryTypeDef;

/* ---------------------------------------------------------------------------*/
/*  FMEA table — 47 entries                                                   */
/* ---------------------------------------------------------------------------*/
static const FMEA_EntryTypeDef fmea_table[FMEA_MAX_ENTRIES] = {

    /* ==================== M7 FOC Failures (1-10) ==================== */
    { 1,  9, "M7_FOC",      "Phase A current sensor open",       "Loss of current feedback -> uncontrolled torque",         "ADC fault flag",          "Fallback to sensorless observer" },
    { 2,  9, "M7_FOC",      "Phase B current sensor short",      "Incorrect Ib reading -> torque ripple",                   "ADC saturation check",    "Disable affected phase, reduce power" },
    { 3,  10,"M7_FOC",      "DC bus overcurrent",                "MOSFET failure / fire hazard",                            "ADC comparator IRQ",      "Immediate PWM disable, brake engage" },
    { 4,  8, "M7_FOC",      "CORDIC calculation timeout",        "FOC ISR stall -> motor coast",                            "CORDIC status poll",      "Software CORDIC fallback, reset CORDIC" },
    { 5,  7, "M7_FOC",      "TIM1 PWM output stuck high",        "Phase short circuit -> overcurrent",                      "Overcurrent protection",  "Emergency PWM disable via BRK" },
    { 6,  7, "M7_FOC",      "TIM1 update interrupt missed",      "FOC period jitter -> current ripple",                     "ISR latency monitor",     "Increase ISR priority" },
    { 7,  6, "M7_FOC",      "PI integrator windup",              "Slow transient response -> overshoot",                     "Integral state monitor",  "Conditional anti-windup active" },
    { 8,  5, "M7_FOC",      "ADC sampling clock jitter",         "Noisy current reading -> torque ripple",                   "ADC clock monitor",       "Use hardware oversampling" },
    { 9,  4, "M7_FOC",      "Field weakening saturation",        "Motor cannot reach target speed",                         "FW active flag",          "Reduce load or speed target" },
    { 10, 3, "M7_FOC",      "Encoder count overflow",            "Position wrapping error",                                 "CNT direction check",     "Enable overflow ISR" },

    /* ==================== M4 Safety Failures (11-20) ==================== */
    { 11, 10,"M4_SAFETY",   "Safety watchdog timeout",           "M4 task hung -> no fault response",                       "IWDG reset pending",      "Reset M4, enter safe state on M7" },
    { 12, 9, "M4_SAFETY",   "Redundancy cross-check fail (2/3)", "Degraded sensor confidence",                              "Decision matrix",         "Enter degraded mode, reduce speed" },
    { 13, 10,"M4_SAFETY",   "Redundancy cross-check fail (3/3)", "No reliable sensor data",                                 "Decision matrix",         "Immediate safe stop" },
    { 14, 8, "M4_SAFETY",   "FMEA runtime lookup CRC error",     "Corrupted FMEA table",                                    "CRC check on table",      "Use default safe response" },
    { 15, 6, "M4_SAFETY",   "Tilt detection false positive",     "Unnecessary emergency stop",                              "IMU vs cliff cross-check","Increase debounce" },
    { 16, 7, "M4_SAFETY",   "Stall detection false negative",    "Undetected blockage -> motor damage",                      "Current + speed monitor", "Lower stall detection threshold" },
    { 17, 6, "M4_SAFETY",   "Drop sensor (ToF) soiling",         "False cliff detection -> false stop",                     "IR reflectivity check",   "Self-cleaning cycle, increase debounce" },
    { 18, 5, "M4_SAFETY",   "Drop sensor (IR) LED failure",      "No cliff detection on one side",                          "LED current monitor",     "Use ToF as backup" },
    { 19, 8, "M4_SAFETY",   "Collision detection missed",        "Bumper impact without response",                          "IMU jerk + current spike","Increase collision sensitivity" },
    { 20, 4, "M4_SAFETY",   "Fault injection test not cleared",  "Stuck in test mode after HIL",                            "Test mode flag",          "Power-cycle to clear" },

    /* ==================== Sensor Failures (21-30) ==================== */
    { 21, 9, "IMU",         "IMU I2C communication loss",        "No tilt/accel data -> blind to orientation",               "I2C timeout + NACK",      "Enter cliff-only tilt detection" },
    { 22, 7, "IMU",         "IMU accelerometer saturation",      "Impact > 16g not measured accurately",                     "Raw data clipping check", "Use gyro integration" },
    { 23, 6, "IMU",         "IMU gyro drift",                    "Yaw angle drift -> navigation error",                      "Gyro vs compass check",   "Periodic zero-gyro calibration" },
    { 24, 5, "IMU",         "IMU temperature bias",              "Offset drift with temperature",                            "Internal temp sensor",    "Apply temp compensation from datasheet" },
    { 25, 8, "TOF_CLIFF",   "ToF cliff sensor failure (all 4)",  "No floor distance measurement -> fall risk",               "Range status flags",      "Use IR cliff as backup" },
    { 26, 7, "TOF_CLIFF",   "ToF cliff sensor crosstalk",        "Spurious readings from neighbouring emitter",              "Sequential firing",       "Increase inter-sensor delay" },
    { 27, 6, "TOF_CLIFF",   "ToF ambient light saturation",      "Reduced ranging distance -> false cliff",                   "Ambient light level",     "Use IR filter, reduce integration" },
    { 28, 4, "TOF_WALL",    "ToF wall sensor degraded",          "Reduced wall-follow accuracy",                             "Signal quality indicator","Increase wall sensor fusion weight" },
    { 29, 5, "BATTERY",     "Battery voltage ADC noise",         "Inaccurate voltage reading -> premature shutdown",         "Oversampling + filter",   "Increase averaging depth" },
    { 30, 3, "BATTERY",     "BMS cell voltage mismatch",         "Reduced runtime, battery stress",                          "BMS cell balance report", "Initiate cell balancing" },

    /* ==================== Actuator Failures (31-40) ==================== */
    { 31, 10,"WHEEL_LEFT",  "Wheel L MOSFET short circuit",      "Uncontrolled drive -> spin",                                "Overcurrent + speed mismatch","Disable PWM, engage brake" },
    { 32, 9, "WHEEL_LEFT",  "Wheel L Hall sensor failure",       "Commutation failure -> cogging",                           "Hall pattern check",      "Switch to sensorless FOC" },
    { 33, 10,"WHEEL_RIGHT", "Wheel R encoder disconnect",        "No speed feedback -> runaway",                             "Encoder timeout",         "Switch to SMO observer, reduce speed" },
    { 34, 8, "WHEEL_LEFT",  "Wheel L mechanical jam",            "Motor stall -> high current",                              "Stall detection logic",   "Reverse pulse, retry, then report" },
    { 35, 7, "ROLLER",      "Roller brush tangle",               "Hair/string wrapped -> reduced cleaning",                   "Current ripple increase", "Reverse roller, self-clean cycle" },
    { 36, 6, "ROLLER",      "Roller brush belt slip",            "Brush not turning intermittently",                         "Speed vs current anomaly", "Inspect belt tension" },
    { 37, 5, "SIDE_BRUSH",  "Side brush wear",                   "Reduced edge cleaning effectiveness",                      "Speed control error",     "Replace brush" },
    { 38, 4, "SIDE_BRUSH",  "Side brush debris wrap",            "Increased current -> reduced speed",                       "Current threshold",       "Reverse direction to shed debris" },
    { 39, 6, "FAN",         "Vacuum fan obstruction",            "Reduced suction -> poor pickup",                           "Power vs speed anomaly",  "Reverse fan burst, check filter" },
    { 40, 5, "FAN",         "Vacuum fan bearing wear",           "Noise increase, eventual seizure",                         "Acoustic/vibration",      "Replace fan assembly" },

    /* ==================== Power / Comms Failures (41-47) ==================== */
    { 41, 9, "POWER",       "Battery under-voltage (< 18 V)",    "Motor torque insufficient, system brownout",               "ADC voltage monitor",     "Initiate return-to-dock" },
    { 42, 10,"POWER",       "Battery over-voltage (> 28 V)",     "MOSFET overstress -> failure",                             "ADC voltage monitor",     "Disable PWM, engage bleed resistor" },
    { 43, 8, "POWER",       "Charging MOSFET short",             "Battery continuously charging",                            "Charge current monitor",  "Disable charger FET, alert user" },
    { 44, 7, "COMMS",       "M7-M4 SPI communication loss",      "Safety commands not delivered",                            "SPI timeout + CRC fail",  "Enter safe state on both cores" },
    { 45, 6, "COMMS",       "M7-M4 shared memory corruption",    "Incorrect data passed between cores",                      "CRC + sequence counter",  "Reset shared memory region" },
    { 46, 4, "COMMS",       "CAN bus-off condition",             "No external communication",                                "CAN error counters",      "Bus recovery sequence" },
    { 47, 3, "COMMS",       "BLE/OTA link disconnect",           "No user command/telemetry",                                "Connection timeout",      "Auto-reconnect, store commands" },
};

/* ---------------------------------------------------------------------------*/
/*  Runtime lookup                                                            */
/* ---------------------------------------------------------------------------*/

/** @brief  Look up FMEA entry by failure ID.
  * @param  id  Failure ID (1-47).
  * @return Pointer to FMEA entry, or NULL if ID not found.
  */
const FMEA_EntryTypeDef* FMEA_Lookup(uint16_t id)
{
    if ((id < 1U) || (id > FMEA_MAX_ENTRIES)) return NULL;

    /* IDs are 1-indexed; table is 0-indexed */
    return &fmea_table[id - 1U];
}

/** @brief  Get the severity for a given failure ID.
  * @return Severity (1-10), or 0 if ID invalid.
  */
uint8_t FMEA_GetSeverity(uint16_t id)
{
    const FMEA_EntryTypeDef *entry = FMEA_Lookup(id);
    if (entry == NULL) return 0U;
    return entry->severity;
}

/** @brief  Get the component name for a given failure ID. */
const char* FMEA_GetComponent(uint16_t id)
{
    const FMEA_EntryTypeDef *entry = FMEA_Lookup(id);
    if (entry == NULL) return "UNKNOWN";
    return entry->component;
}

/** @brief  Get the failure mode description. */
const char* FMEA_GetFailureMode(uint16_t id)
{
    const FMEA_EntryTypeDef *entry = FMEA_Lookup(id);
    if (entry == NULL) return "UNKNOWN";
    return entry->failure_mode;
}

/** @brief  Get the effect description. */
const char* FMEA_GetEffect(uint16_t id)
{
    const FMEA_EntryTypeDef *entry = FMEA_Lookup(id);
    if (entry == NULL) return "UNKNOWN";
    return entry->effect;
}

/** @brief  Get the detection method. */
const char* FMEA_GetDetection(uint16_t id)
{
    const FMEA_EntryTypeDef *entry = FMEA_Lookup(id);
    if (entry == NULL) return "UNKNOWN";
    return entry->detection;
}

/** @brief  Get the mitigation strategy. */
const char* FMEA_GetMitigation(uint16_t id)
{
    const FMEA_EntryTypeDef *entry = FMEA_Lookup(id);
    if (entry == NULL) return "UNKNOWN";
    return entry->mitigation;
}

/* ---------------------------------------------------------------------------*/
/*  Bulk operations                                                           */
/* ---------------------------------------------------------------------------*/

/** @brief  Search FMEA table by component name substring.
  * @param  component  Component name to search for (case-sensitive).
  * @param  results    Output array of matching entry IDs.
  * @param  max_results Size of results array.
  * @return Number of matches found.
  */
uint32_t FMEA_SearchByComponent(const char *component,
                                 uint16_t *results, uint32_t max_results)
{
    uint32_t count = 0U;
    for (uint32_t i = 0U; i < FMEA_MAX_ENTRIES; i++) {
        if (strstr(fmea_table[i].component, component) != NULL) {
            if (count < max_results) {
                results[count] = fmea_table[i].id;
            }
            count++;
        }
    }
    return count;
}

/** @brief  Get total number of FMEA entries. */
uint32_t FMEA_GetEntryCount(void)
{
    return FMEA_MAX_ENTRIES;
}

/** @brief  Get entries with severity above a threshold.
  * @param  min_severity Minimum severity (e.g., 8 for critical+).
  * @param  results      Output array of matching entry IDs.
  * @param  max_results  Size of results array.
  * @return Number of matches.
  */
uint32_t FMEA_GetCriticalEntries(uint8_t min_severity,
                                  uint16_t *results, uint32_t max_results)
{
    uint32_t count = 0U;
    for (uint32_t i = 0U; i < FMEA_MAX_ENTRIES; i++) {
        if (fmea_table[i].severity >= min_severity) {
            if (count < max_results) {
                results[count] = fmea_table[i].id;
            }
            count++;
        }
    }
    return count;
}
