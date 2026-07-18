/* Minimal FreeRTOS stub for CI cross-compilation */
#ifndef FREERTOS_STUB_H
#define FREERTOS_STUB_H

#include <stdint.h>
#include <stdbool.h>

typedef void * TaskHandle_t;
typedef void (* TaskFunction_t)(void *);

#define configMAX_PRIORITIES    8
#define tskIDLE_PRIORITY        0

static inline TaskHandle_t xTaskCreateStatic(TaskFunction_t fn, const char *name,
    uint32_t stack_depth, void *params, uint32_t prio, void *stack, void *tcb) {
    (void)fn;(void)name;(void)stack_depth;(void)params;(void)prio;(void)stack;(void)tcb;
    return (TaskHandle_t)0x1;
}

static inline void vTaskDelayUntil(uint32_t *prev, uint32_t inc) { (void)prev;(void)inc; }
static inline void vTaskDelay(uint32_t ticks) { (void)ticks; }
static inline void vTaskStartScheduler(void) {}
static inline uint32_t xTaskGetTickCount(void) { return 0; }
static inline void vTaskSuspend(TaskHandle_t h) { (void)h; }
static inline void vTaskResume(TaskHandle_t h) { (void)h; }

static inline int xSemaphoreCreateBinary(void) { return 0; }
static inline int xSemaphoreTake(int s, uint32_t t) { (void)s;(void)t; return 1; }
static inline int xSemaphoreGive(int s) { (void)s; return 1; }

static inline int xQueueCreate(uint32_t len, uint32_t item_size) {
    (void)len;(void)item_size; return 0;
}
static inline int xQueueSend(int q, const void *p, uint32_t t) {
    (void)q;(void)p;(void)t; return 1;
}
static inline int xQueueReceive(int q, void *p, uint32_t t) {
    (void)q;(void)p;(void)t; return 1;
}

#define pdMS_TO_TICKS(ms) ((ms) / 10)
#define pdFALSE 0
#define pdTRUE  1
#define pdPASS  1
#define pdFAIL  0

#endif
