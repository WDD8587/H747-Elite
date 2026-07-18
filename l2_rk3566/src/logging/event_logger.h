/**
 * @file    event_logger.h
 * @brief   Structured event logger API.
 */
#ifndef EVENT_LOGGER_H
#define EVENT_LOGGER_H

#include <stdint.h>

void     event_logger_init(void);
void     event_logger_state_change(const char *old_state,
                                    const char *new_state,
                                    const char *trigger);
void     event_logger_fault(uint32_t code, const char *severity,
                             const char *context);
void     event_logger_sensor(const char *sensor, const char *values);
void     event_logger_motor_cmd(float v, float w, uint64_t timestamp);
void     event_logger_battery(const char *type, float value);
void     event_logger_raw(const char *json);
void     event_logger_close(void);
uint32_t event_logger_total_events(void);

#endif /* EVENT_LOGGER_H */
