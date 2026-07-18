/**
 * @file    perf_profiler.h
 * @brief   Performance profiler API.
 */
#ifndef PERF_PROFILER_H
#define PERF_PROFILER_H

#include <stdint.h>

/* FreeRTOS trace hook compatibility */
#ifndef configMAX_TASK_NAME_LEN
#define configMAX_TASK_NAME_LEN  16
#endif

void perf_profiler_init(void);
void perf_profiler_task_switch_in(const char *task_name);
void perf_profiler_task_switch_out(const char *task_name);
void perf_profiler_stack_hwm(const char *task_name, uint16_t hwm);
void perf_profiler_flush(int force);
void perf_profiler_rotate(void);

#endif /* PERF_PROFILER_H */
