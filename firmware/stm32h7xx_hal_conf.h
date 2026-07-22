/* stm32h7xx_hal_conf.h — minimal config for CI cross-compilation */
#ifndef STM32H7XX_HAL_CONF_H
#define STM32H7XX_HAL_CONF_H

#define HSE_VALUE    25000000UL
#define HSI_VALUE    64000000UL
#define LSE_VALUE    32768UL

#define TICK_INT_PRIORITY            0x0FUL
#define USE_RTOS                     1
#define PREFETCH_ENABLE              1
#define INSTRUCTION_CACHE_ENABLE     1
#define DATA_CACHE_ENABLE            1

/* Enable only the HAL modules used by this project */
#define HAL_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_HSEM_MODULE_ENABLED
#define HAL_I2C_MODULE_ENABLED
#define HAL_PCD_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_SPI_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED

#define assert_param(expr) ((void)0U)

#endif
