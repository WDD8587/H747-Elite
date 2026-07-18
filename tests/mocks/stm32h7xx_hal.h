#ifndef STM32H7XX_HAL_MOCK_H
#define STM32H7XX_HAL_MOCK_H
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* Minimal HAL mock for host GCC unit testing */
typedef enum { HAL_OK=0, HAL_ERROR=1, HAL_BUSY=2, HAL_TIMEOUT=3 } HAL_StatusTypeDef;
typedef struct { uint32_t ARR, CCR1, CCR2, CCR3, SR, BDTR, CR1; } TIM_TypeDef;
typedef struct {} ADC_TypeDef;
typedef struct {} GPIO_TypeDef;
typedef struct {} I2C_HandleTypeDef;
typedef struct {} UART_HandleTypeDef;
typedef struct {} SPI_HandleTypeDef;

#define TIM_BDTR_MOE    0x8000
#define TIM_SR_UIF      0x0001
#define TIM_CR1_CEN     0x0001
#define GPIO_PIN_SET    1
#define GPIO_PIN_RESET  0
#define GPIO_MODE_OUTPUT_PP 1
#define GPIO_NOPULL      0
#define GPIO_SPEED_FREQ_VERY_HIGH 3

extern TIM_TypeDef TIM1_MOCK, TIM8_MOCK;
extern ADC_TypeDef ADC1_MOCK, ADC2_MOCK;
#define TIM1 (&TIM1_MOCK)
#define TIM8 (&TIM8_MOCK)
#define ADC1 (&ADC1_MOCK)
#define ADC2 (&ADC2_MOCK)

/* NVIC/IRQ mocks */
typedef enum { TIM1_UP_TIM10_IRQn=0, TIM8_UP_TIM13_IRQn=1 } IRQn_Type;
static inline void NVIC_DisableIRQ(IRQn_Type n) { (void)n; }
static inline void HAL_NVIC_SetPriority(IRQn_Type n, int p, int s) { (void)n;(void)p;(void)s; }

/* HSEM mocks */
static inline void __HAL_RCC_HSEM_CLK_ENABLE(void) {}
static inline int  HAL_HSEM_IsSemTaken(int id) { (void)id; return 0; }
static inline void HAL_HSEM_Release(int id, int p) { (void)id;(void)p; }
static inline int  HAL_HSEM_Take(int id, int p) { (void)id;(void)p; return 0; }

/* GPIO mocks */
static inline void HAL_GPIO_WritePin(void *p, int pin, int v) { (void)p;(void)pin;(void)v; }
static inline int  HAL_GPIO_ReadPin(void *p, int pin) { (void)p;(void)pin; return 0; }
static inline void HAL_GPIO_Init(void *p, void *init) { (void)p;(void)init; }

/* I2C mocks */
static inline HAL_StatusTypeDef HAL_I2C_Mem_Read(void *h, uint16_t a, uint8_t c, uint16_t m, uint8_t *b, uint16_t s, uint32_t t) {
    (void)h;(void)a;(void)c;(void)m;(void)b;(void)s;(void)t; memset(b,0,s); return HAL_OK;
}
static inline HAL_StatusTypeDef HAL_I2C_Master_Receive(void *h, uint16_t a, uint8_t *b, uint16_t s, uint32_t t) {
    (void)h;(void)a;(void)b;(void)s;(void)t; return HAL_OK;
}
static inline HAL_StatusTypeDef HAL_I2C_Master_Seq_Transmit(void *h, uint16_t a, uint8_t *b, uint16_t s, uint32_t f, uint32_t t) {
    (void)h;(void)a;(void)b;(void)s;(void)f;(void)t; return HAL_OK;
}
static inline HAL_StatusTypeDef HAL_I2C_Master_Seq_Receive(void *h, uint16_t a, uint8_t *b, uint16_t s, uint32_t f, uint32_t t) {
    (void)h;(void)a;(void)b;(void)s;(void)f;(void)t; return HAL_OK;
}
#define I2C_FIRST_FRAME 0
#define I2C_LAST_FRAME  1
#define I2C_MEMADD_SIZE_8BIT 1

/* GPIO struct */
typedef struct {
    uint32_t Pin, Mode, Pull, Speed, Alternate;
} GPIO_InitTypeDef;

/* GPIO ports (dummy addresses for test) */
#define GPIOA ((GPIO_TypeDef*)0x1000)
#define GPIOB ((GPIO_TypeDef*)0x1004)
#define GPIOC ((GPIO_TypeDef*)0x1008)
#define GPIOD ((GPIO_TypeDef*)0x100C)

/* GPIO pin defines */
#define GPIO_PIN_0  0x0001
#define GPIO_PIN_1  0x0002
#define GPIO_PIN_2  0x0004
#define GPIO_PIN_3  0x0008
#define GPIO_PIN_4  0x0010
#define GPIO_PIN_5  0x0020
#define GPIO_PIN_6  0x0040
#define GPIO_PIN_7  0x0080
#define GPIO_PIN_12 0x1000

/* GPIO modes */
#define GPIO_MODE_AF_PP      2
#define GPIO_PULLDOWN        2
#define GPIO_AF1_TIM1        1
#define GPIO_AF3_TIM8        3
#define GPIO_AF5_SPI1        5
#define GPIO_AF7_UART7       7
#define GPIO_AF8_UART4       8
#define GPIO_AF9_FDCAN1      9
#define GPIO_AF9_QUADSPI     9
#define GPIO_AF10_QUADSPI    10

/* Clock enable macros */
#define __HAL_RCC_GPIOD_CLK_ENABLE()  do{}while(0)
#define __HAL_RCC_TIM1_CLK_ENABLE()   do{}while(0)
#define __HAL_RCC_GPIOA_CLK_ENABLE()  do{}while(0)
#define __HAL_RCC_GPIOB_CLK_ENABLE()  do{}while(0)
#define __HAL_RCC_GPIOC_CLK_ENABLE()  do{}while(0)
#define __HAL_RCC_SPI1_CLK_ENABLE()   do{}while(0)
#define __HAL_RCC_I2C1_CLK_ENABLE()   do{}while(0)
#define __HAL_RCC_I2C3_CLK_ENABLE()   do{}while(0)
#define __HAL_RCC_WWDG_CLK_ENABLE()   do{}while(0)
#define __HAL_RCC_LPTIM1_CLK_ENABLE() do{}while(0)
#define __HAL_RCC_DMA1_CLK_ENABLE()   do{}while(0)
#define __HAL_RCC_DMA2_CLK_ENABLE()   do{}while(0)

/* TIM BDTR bits */
#define TIM_BDTR_BKF_Msk  0x00F0
#define TIM_BDTR_AOE      0x4000
#define TIM_BDTR_BKE      0x1000
#define TIM_BDTR_BKP      0x2000
#define TIM_BDTR_OSSR     0x0008

/* WWDG registers */
#define WWDG_CFR_WDGTB_Pos 7
#define WWDG_CFR_W_Pos     0
#define WWDG_CFR_EWI       0x0200
#define WWDG_CR_T_Pos      0
#define WWDG_CR_WDGA       0x0080
typedef struct { volatile uint32_t CR, CFR, SR; } WWDG_TypeDef;
#define WWDG ((WWDG_TypeDef*)0x40002C00)

/* HAL common */
static inline void HAL_Init(void) {}
static inline void HAL_Delay(uint32_t ms) { (void)ms; }
static inline uint32_t HAL_GetTick(void) { return 0; }
static inline void HAL_NVIC_EnableIRQ(IRQn_Type n) { (void)n; }
static inline void SystemCoreClockUpdate(void) {}
static inline void HAL_PWREx_EnableWakeUpPin(int p) { (void)p; }
static inline void HAL_PWREx_EnterSTOP2Mode(int r) { (void)r; }
#define PWR_MAINREGULATOR_ON 0
#define PWR_WAKEUP_PIN1 0

/* HAL UART */
static inline HAL_StatusTypeDef HAL_UART_Transmit(void *h, uint8_t *b, uint16_t s, uint32_t t) {
    (void)h;(void)b;(void)s;(void)t; return HAL_OK;
}
static inline HAL_StatusTypeDef HAL_UART_Transmit_DMA(void *h, uint8_t *b, uint16_t s) {
    (void)h;(void)b;(void)s; return HAL_OK;
}

/* IWDG */
static inline void HAL_IWDG_Refresh(void *h) { (void)h; }
#define IWDG_Init(x) do{}while(0)

/* CORDIC mock */
typedef struct { volatile uint32_t WDATA, CSR, RDATA; } CORDIC_TypeDef;
extern CORDIC_TypeDef CORDIC_MOCK;
#define CORDIC (&CORDIC_MOCK)

#endif
