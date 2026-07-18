/**
 * @file    health_report.h
 * @brief   Health report generation API.
 */
#ifndef HEALTH_REPORT_H
#define HEALTH_REPORT_H

#include <stdint.h>

void        health_report_init(void);
void        health_report_tick(void);
void        health_report_increment_errors(uint32_t count);
void        health_report_increment_can_errors(uint32_t count);
void        health_report_increment_flash_writes(void);
const char *health_report_force(void);
const char *health_report_get(void);
uint32_t    health_report_get_error_count(void);

#endif /* HEALTH_REPORT_H */
