/**
 * @file    health_report.c
 * @brief   Periodic health report generation.
 *
 * @details Every 60 seconds, collects system health metrics and produces
 *          a JSON-formatted report. Metrics include:
 *            - Motor temperature (max of L/R)
 *            - BMS temperature
 *            - IMU temperature
 *            - CPU load (M7 + M4)
 *            - Uptime (seconds)
 *            - Error count since boot
 *            - WDT reset count
 *            - Flash write cycles
 *            - CAN bus error count
 *
 *          The report is written to shared SRAM for the RK3566 to pick up
 *          and forward to the cloud.
 */

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include "stm32h7xx_hal.h"
#include "health_report.h"

/* ---------------------------------------------------------------------------
 * Shared SRAM for health report (accessible from M4, read by M7)
 * -------------------------------------------------------------------------*/
#define HEALTH_REPORT_SRAM_ADDR  0x30020000UL   /* SRAM1 in D1 domain */
#define HEALTH_REPORT_MAX_LEN    2048

static volatile char *report_buffer = (volatile char *)HEALTH_REPORT_SRAM_ADDR;

/* ---------------------------------------------------------------------------
 * State tracking
 * -------------------------------------------------------------------------*/
static uint32_t last_report_time = 0;
static uint32_t boot_tick        = 0;
static uint32_t error_count      = 0;
static uint32_t wdt_reset_count  = 0;
static uint32_t flash_write_cycles = 0;
static uint32_t can_bus_error_count = 0;

/* ---------------------------------------------------------------------------
 * External references
 * -------------------------------------------------------------------------*/
extern RTC_HandleTypeDef hrtc;         /* RTC handle        */
extern FDCAN_HandleTypeDef hfdcan1;   /* CAN handle        */
extern volatile uint32_t uwTick;       /* HAL tick counter  */

/* Temperature sensor handles (defined in application code) */
extern float motor_temp_left;    /* Motor L temperature (deg C)  */
extern float motor_temp_right;   /* Motor R temperature (deg C)  */
extern float bms_temp;           /* BMS temperature (deg C)      */
extern float imu_temp;           /* ICM-20948 temperature (deg C)*/

/* CPU load (set by FreeRTOS idle hook or equivalent) */
extern float m7_cpu_load;        /* M7 core load (0.0 - 100.0)  */
extern float m4_cpu_load;        /* M4 core load (0.0 - 100.0)  */

/* ---------------------------------------------------------------------------
 * Internal helpers
 * -------------------------------------------------------------------------*/

/**
 * @brief  Get uptime in seconds.
 */
static uint32_t get_uptime_seconds(void)
{
    return (uwTick - boot_tick) / 1000U;
}

/**
 * @brief  Format a JSON health report string.
 *
 * @param  buf   Output buffer.
 * @param  len   Buffer size.
 */
static void format_health_json(char *buf, size_t len)
{
    uint32_t uptime_s = get_uptime_seconds();

    /* Get max motor temperature */
    float motor_temp = (motor_temp_left > motor_temp_right)
                           ? motor_temp_left
                           : motor_temp_right;

    /* Get current timestamp from RTC */
    RTC_DateTypeDef rtc_date;
    RTC_TimeTypeDef rtc_time;
    char timestamp[32] = {0};

    HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN);

    snprintf(timestamp, sizeof(timestamp),
             "%04u-%02u-%02uT%02u:%02u:%02uZ",
             2000U + (uint32_t)rtc_date.Year,
             (uint32_t)rtc_date.Month,
             (uint32_t)rtc_date.Date,
             (uint32_t)rtc_time.Hours,
             (uint32_t)rtc_time.Minutes,
             (uint32_t)rtc_time.Seconds);

    snprintf(buf, len,
             "{"
             "\"ts\":\"%s\","
             "\"uptime_s\":%u,"
             "\"motor_temp_c\":%.1f,"
             "\"bms_temp_c\":%.1f,"
             "\"imu_temp_c\":%.1f,"
             "\"cpu_load_m7_pct\":%.1f,"
             "\"cpu_load_m4_pct\":%.1f,"
             "\"error_count\":%u,"
             "\"wdt_resets\":%u,"
             "\"flash_writes\":%u,"
             "\"can_errors\":%u"
             "}\n",
             timestamp,
             (unsigned int)uptime_s,
             (double)motor_temp,
             (double)bms_temp,
             (double)imu_temp,
             (double)m7_cpu_load,
             (double)m4_cpu_load,
             (unsigned int)error_count,
             (unsigned int)wdt_reset_count,
             (unsigned int)flash_write_cycles,
             (unsigned int)can_bus_error_count);
}

/* ---------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------*/

/**
 * @brief  Initialise the health report generator.
 *
 * Records the boot tick and reads initial fault counters.
 */
void health_report_init(void)
{
    boot_tick = uwTick;

    /* Read WDT reset count from backup register (set by bootloader) */
    wdt_reset_count = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0);

    /* Estimate flash write cycles from a RTC backup register */
    flash_write_cycles = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1);

    last_report_time = uwTick;
}

/**
 * @brief  Periodic health report tick.
 *
 * Call from a 1-second timer or main loop. Generates a new report
 * every 60 seconds.
 */
void health_report_tick(void)
{
    /* Check if 60 seconds elapsed */
    uint32_t elapsed = uwTick - last_report_time;
    if (elapsed < 60000U) {
        return;  /* Not yet */
    }

    last_report_time = uwTick;

    char report[HEALTH_REPORT_MAX_LEN];
    format_health_json(report, sizeof(report));

    /* Write report to shared SRAM */
    size_t report_len = strlen(report);
    if (report_len >= HEALTH_REPORT_MAX_LEN) {
        report_len = HEALTH_REPORT_MAX_LEN - 1;
    }

    memcpy((void *)report_buffer, report, report_len);
    report_buffer[report_len] = '\0';
}

/**
 * @brief  Increment the error count.
 *
 * @param  count  Number of errors to add.
 */
void health_report_increment_errors(uint32_t count)
{
    error_count += count;
}

/**
 * @brief  Increment the CAN bus error count.
 *
 * @param  count  Number of errors to add.
 */
void health_report_increment_can_errors(uint32_t count)
{
    can_bus_error_count += count;
}

/**
 * @brief  Increment flash write cycle counter.
 */
void health_report_increment_flash_writes(void)
{
    flash_write_cycles++;

    /* Persist to RTC backup register (limited endurance; write every 100) */
    if ((flash_write_cycles % 100) == 0) {
        HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, flash_write_cycles);
    }
}

/**
 * @brief  Force the health report to regenerate immediately.
 *
 * @return Pointer to the report string in shared SRAM.
 */
const char *health_report_force(void)
{
    last_report_time = 0;  /* Force next tick to generate */
    health_report_tick();
    return (const char *)report_buffer;
}

/**
 * @brief  Get a pointer to the current health report in shared SRAM.
 *
 * @return Pointer to null-terminated JSON string.
 */
const char *health_report_get(void)
{
    return (const char *)report_buffer;
}

/**
 * @brief  Get the current error count.
 *
 * @return Error count since boot.
 */
uint32_t health_report_get_error_count(void)
{
    return error_count;
}
