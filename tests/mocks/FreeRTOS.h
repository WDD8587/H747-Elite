#ifndef FREERTOS_MOCK_H
#define FREERTOS_MOCK_H
#include <stdint.h>

typedef void *TaskHandle_t;
typedef uint32_t TickType_t;
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#define pdFALSE 0
#define pdTRUE  1
#define portMAX_DELAY 0xFFFFFFFF

static inline TickType_t xTaskGetTickCount(void) { return 0; }
static inline void vTaskDelayUntil(TickType_t *t, TickType_t d) { (void)t;(void)d; }
static inline void vTaskDelay(TickType_t d) { (void)d; }
#define vTaskStartScheduler() do{}while(0)
#endif
