/* CMSIS-RTOS v2 minimal stub for CI cross-compilation */
#ifndef CMSIS_OS_STUB_H
#define CMSIS_OS_STUB_H
#include <stdint.h>
#include "FreeRTOS.h"

typedef void * osThreadId_t;
typedef void (* osThreadFunc_t)(void *);

static inline int osKernelInitialize(void) { return 0; }
static inline int osKernelStart(void) { return 0; }
static inline osThreadId_t osThreadNew(osThreadFunc_t fn, void *arg, void *attr) {
    (void)fn;(void)arg;(void)attr;
    return (osThreadId_t)0x1;
}
static inline uint32_t osKernelGetTickCount(void) { return 0; }
static inline int osDelay(uint32_t ticks) { (void)ticks; return 0; }
#endif
