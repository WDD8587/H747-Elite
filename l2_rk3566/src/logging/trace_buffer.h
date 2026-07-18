/**
 * @file    trace_buffer.h
 * @brief   Trace buffer (ring buffer) API.
 */
#ifndef TRACE_BUFFER_H
#define TRACE_BUFFER_H

#include <stdint.h>

/* ---------------------------------------------------------------------------
 * Event levels
 * -------------------------------------------------------------------------*/
#define TRACE_LEVEL_ERROR   0
#define TRACE_LEVEL_WARN    1
#define TRACE_LEVEL_INFO    2
#define TRACE_LEVEL_DEBUG   3
#define TRACE_LEVEL_TRACE   4
#define TRACE_LEVEL_FATAL   5

/* ---------------------------------------------------------------------------
 * Module IDs
 * -------------------------------------------------------------------------*/
#define TRACE_MOD_SYSTEM    0
#define TRACE_MOD_MOTOR     1
#define TRACE_MOD_IMU       2
#define TRACE_MOD_BMS       3
#define TRACE_MOD_SENSOR    4
#define TRACE_MOD_CAN       5
#define TRACE_MOD_SAFETY    6
#define TRACE_MOD_NET       7
#define TRACE_MOD_DIAG      8
#define TRACE_MOD_BOOT      9

/* ---------------------------------------------------------------------------
 * Public event structure (unpacked)
 * -------------------------------------------------------------------------*/
typedef struct {
    uint32_t timestamp_us;
    uint8_t  level;
    uint8_t  module;
    uint32_t message;  /* 20-bit index */
} trace_entry_t;

/* ---------------------------------------------------------------------------
 * API
 * -------------------------------------------------------------------------*/
void     trace_buffer_init(void);
void     trace_buffer_push(uint8_t level, uint8_t module, uint32_t message);
int      trace_buffer_dump(const char *path);
void     trace_buffer_print(const char *path, uint8_t filter);
int      trace_buffer_query(trace_entry_t *out, int max,
                             uint8_t module, uint8_t level);
uint32_t trace_buffer_count(void);
void     trace_buffer_clear(void);

#endif /* TRACE_BUFFER_H */
