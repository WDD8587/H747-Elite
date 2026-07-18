#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "m7_config.h"
#include "foc_params.h"
#include "ipc_proto.h"

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = 5;
    osc.PLL.PLLN = 120;
    osc.PLL.PLLP = 2;
    osc.PLL.PLLQ = 4;
    osc.PLL.PLLR = 2;
    HAL_RCC_OscConfig(&osc);
    RCC_ClkInitTypeDef clk = {0};
    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_4);
}

extern void FocSetSpeed(float vl, float vr);

static void vImuTask(void *pv)
{
    (void)pv;
    TickType_t t = xTaskGetTickCount();
    for (;;) {
        FocSetSpeed(0.0f, 0.0f);
        vTaskDelayUntil(&t, pdMS_TO_TICKS(1));
    }
}

static void vIpcBridgeTask(void *pv)
{
    (void)pv;
    TickType_t t = xTaskGetTickCount();
    static uint8_t heartbeat = 0;
    for (;;) {
        if (++heartbeat >= 10) {
            heartbeat = 0;
            HAL_HSEM_Release(HSEM_M7_HEARTBEAT, 0);
        }
        vTaskDelayUntil(&t, pdMS_TO_TICKS(10));
    }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    __HAL_RCC_HSEM_CLK_ENABLE();

    xTaskCreate(vImuTask, "IMU", 1024, NULL, 30, NULL);
    xTaskCreate(vIpcBridgeTask, "IPC", 2048, NULL, 25, NULL);
    vTaskStartScheduler();
    for (;;) {}
}
