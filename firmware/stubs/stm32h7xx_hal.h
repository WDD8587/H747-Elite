/* HAL stub: real CMSIS provides register types and bit definitions.
 * This file provides ONLY the no-op HAL function implementations.
 */
#ifndef STM32H7XX_HAL_STUB_H
#define STM32H7XX_HAL_STUB_H

/* CMSIS: device header before core (IRQn_Type needed by core_cm*.h) */
#define __FPU_PRESENT 1U
#include "stm32h747xx.h"
#if defined(CORE_CM4)
  #include "core_cm4.h"
#else
  #include "core_cm7.h"
#endif
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- HAL Status ---- */
typedef enum { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2, HAL_TIMEOUT = 3 } HAL_StatusTypeDef;

/* ---- DMA ---- */
typedef struct {
    void *Instance;
    struct { uint32_t Channel, Direction, PeriphInc, MemInc, PeriphDataAlignment,
             MemDataAlignment, Mode, Priority, FIFOMode, FIFOThreshold, MemBurst, PeriphBurst; } Init;
} DMA_HandleTypeDef;

#define DMA_CHANNEL_0  0
#define DMA_CHANNEL_3  3
#define DMA_CHANNEL_6  6
#define DMA_CHANNEL_7  7

#define DMA_MEMORY_TO_PERIPH  1
#define DMA_PERIPH_TO_MEMORY  0
#define DMA_PINC_ENABLE   1
#define DMA_PINC_DISABLE  0
#define DMA_MINC_ENABLE   1
#define DMA_MINC_DISABLE  0
#define DMA_PDATAALIGN_BYTE     0
#define DMA_PDATAALIGN_HALFWORD 1
#define DMA_NORMAL    0
#define DMA_CIRCULAR  1
#define DMA_PRIORITY_LOW    0
#define DMA_PRIORITY_HIGH   2
#define DMA_FIFOMODE_ENABLE  1
#define DMA_FIFOMODE_DISABLE 0
#define DMA_FIFO_THRESHOLD_1QUARTERFULL 0
#define DMA_FIFO_THRESHOLD_FULL         3
#define DMA_MBURST_SINGLE 0
#define DMA_MBURST_INCR4  1
#define DMA_PBURST_SINGLE 0
#define HAL_DMA_XFER_CPLT_CB_ID      0x00000001U
#define HAL_DMA_XFER_ERROR_CB_ID     0x00000004U

#define DMA1_Stream0  ((void*)0x1000)
#define DMA1_Stream1  ((void*)0x1001)
#define DMA1_Stream4  ((void*)0x1004)
#define DMA2_Stream2  ((void*)0x2002)
#define DMA2_Stream3  ((void*)0x2003)
#define DMA2_Stream5  ((void*)0x2005)
#define DMA2_Stream6  ((void*)0x2006)

static inline HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *h) { (void)h; return HAL_OK; }
static inline HAL_StatusTypeDef HAL_DMA_DeInit(DMA_HandleTypeDef *h) { (void)h; return HAL_OK; }
static inline void HAL_DMA_RegisterCallback(DMA_HandleTypeDef *h, uint32_t id, void (*cb)(DMA_HandleTypeDef*)) {
    (void)h;(void)id;(void)cb;
}
#define __HAL_LINKDMA(h, member, dma) do { (h)->member = (dma); } while(0)

/* ---- UART ---- */
typedef struct { uint32_t BaudRate, WordLength, StopBits, Parity, Mode, HwFlowCtl, OverSampling; } UART_InitTypeDef;
typedef struct { void *Instance; UART_InitTypeDef Init; } UART_HandleTypeDef;
extern UART_HandleTypeDef huart7;

#define USART7 ((void*)0x40007800)
#define UART_WORDLENGTH_8B  0
#define UART_STOPBITS_1     0
#define UART_PARITY_NONE    0
#define UART_MODE_TX_RX     3
#define UART_HWCONTROL_NONE 0
#define UART_OVERSAMPLING_16 0

static inline HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *h) { (void)h; return HAL_OK; }
static inline HAL_StatusTypeDef HAL_UART_DeInit(UART_HandleTypeDef *h) { (void)h; return HAL_OK; }
static inline HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, uint8_t *b, uint16_t s, uint32_t t) {
    (void)h;(void)b;(void)s;(void)t; return HAL_OK;
}
static inline HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *h, uint8_t *b, uint16_t s) {
    (void)h;(void)b;(void)s; return HAL_OK;
}

/* ---- SPI ---- */
typedef struct {
    uint32_t Mode, Direction, DataSize, CLKPolarity, CLKPhase, NSS, FirstBit,
             TIMode, CRCCalculation, CRCPolynomial, NSSPMode;
} SPI_InitTypeDef;
typedef struct { void *Instance; SPI_InitTypeDef Init; DMA_HandleTypeDef *hdmarx, *hdmatx; } SPI_HandleTypeDef;

#define SPI6 ((void*)0x40015400)
#define SPI_MODE_SLAVE        1
#define SPI_DIRECTION_2LINES  0
#define SPI_DATASIZE_8BIT     0
#define SPI_POLARITY_LOW      0
#define SPI_PHASE_1EDGE       0
#define SPI_NSS_HARD_INPUT    1
#define SPI_FIRSTBIT_MSB      0
#define SPI_TIMODE_DISABLE    0
#define SPI_CRCCALCULATION_DISABLE 0
#define SPI_NSS_PULSE_DISABLE 1

static inline HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *h) { (void)h; return HAL_OK; }
static inline HAL_StatusTypeDef HAL_SPI_DeInit(SPI_HandleTypeDef *h) { (void)h; return HAL_OK; }
static inline HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *h, uint8_t *tx, uint8_t *rx, uint16_t s) {
    (void)h;(void)tx;(void)rx;(void)s; return HAL_OK;
}
static inline HAL_StatusTypeDef HAL_SPI_DMAStop(SPI_HandleTypeDef *h) { (void)h; return HAL_OK; }

/* ---- USB PCD ---- */
typedef struct {
    void *Instance;
    struct { uint32_t dev_endpoints, speed, ep0_mps, phy_itface, Sof_enable,
             low_power_enable, lpm_enable, battery_charging_enable; } Init;
} PCD_HandleTypeDef;

#define USB_OTG_FS  ((void*)0x50000000)
#define PCD_SPEED_FULL    2
#define PCD_PHY_EMBEDDED  2
#define HAL_PCD_SETUP_STAGE_CB_ID 0

static inline HAL_StatusTypeDef HAL_PCD_Init(PCD_HandleTypeDef *h) { (void)h; return HAL_OK; }
static inline HAL_StatusTypeDef HAL_PCD_DeInit(PCD_HandleTypeDef *h) { (void)h; return HAL_OK; }
static inline HAL_StatusTypeDef HAL_PCD_Start(PCD_HandleTypeDef *h) { (void)h; return HAL_OK; }
static inline HAL_StatusTypeDef HAL_PCD_Stop(PCD_HandleTypeDef *h) { (void)h; return HAL_OK; }
static inline HAL_StatusTypeDef HAL_PCD_EP_Open(PCD_HandleTypeDef *h, uint8_t ep, uint16_t mps, uint8_t type) {
    (void)h;(void)ep;(void)mps;(void)type; return HAL_OK;
}
static inline HAL_StatusTypeDef HAL_PCD_EP_Close(PCD_HandleTypeDef *h, uint8_t ep) { (void)h;(void)ep; return HAL_OK; }
static inline HAL_StatusTypeDef HAL_PCD_EP_Transmit(PCD_HandleTypeDef *h, uint8_t ep, uint8_t *b, uint16_t s) {
    (void)h;(void)ep;(void)b;(void)s; return HAL_OK;
}
static inline HAL_StatusTypeDef HAL_PCD_EP_Receive(PCD_HandleTypeDef *h, uint8_t ep, uint8_t *b, uint16_t s) {
    (void)h;(void)ep;(void)b;(void)s; return HAL_OK;
}
static inline HAL_StatusTypeDef HAL_PCD_EP_SetStall(PCD_HandleTypeDef *h, uint8_t ep) { (void)h;(void)ep; return HAL_OK; }
static inline void HAL_PCDEx_SetRxFiFo(PCD_HandleTypeDef *h, uint16_t s) { (void)h;(void)s; }
static inline void HAL_PCDEx_SetTxFiFo(PCD_HandleTypeDef *h, uint8_t ep, uint16_t s) { (void)h;(void)ep;(void)s; }
static inline void HAL_PCD_RegisterCallback(PCD_HandleTypeDef *h, uint32_t id, void (*cb)(PCD_HandleTypeDef*)) {
    (void)h;(void)id;(void)cb;
}
static inline void HAL_PCD_RegisterDataOutStageCallback(PCD_HandleTypeDef *h, void (*cb)(PCD_HandleTypeDef*, uint8_t)) {
    (void)h;(void)cb;
}
static inline void HAL_PCD_RegisterDataInStageCallback(PCD_HandleTypeDef *h, void (*cb)(PCD_HandleTypeDef*, uint8_t)) {
    (void)h;(void)cb;
}
static inline void HAL_PCD_RegisterResetCallback(PCD_HandleTypeDef *h, void (*cb)(PCD_HandleTypeDef*)) { (void)h;(void)cb; }
static inline void HAL_PCD_RegisterConnectCallback(PCD_HandleTypeDef *h, void (*cb)(PCD_HandleTypeDef*)) { (void)h;(void)cb; }
static inline void HAL_PCD_RegisterDisconnectCallback(PCD_HandleTypeDef *h, void (*cb)(PCD_HandleTypeDef*)) { (void)h;(void)cb; }
static inline void HAL_PCD_RegisterSuspendCallback(PCD_HandleTypeDef *h, void (*cb)(PCD_HandleTypeDef*)) { (void)h;(void)cb; }
static inline void HAL_PCD_RegisterResumeCallback(PCD_HandleTypeDef *h, void (*cb)(PCD_HandleTypeDef*)) { (void)h;(void)cb; }
static inline void HAL_PCD_RegisterSOFCallback(PCD_HandleTypeDef *h, void (*cb)(PCD_HandleTypeDef*)) { (void)h;(void)cb; }
static inline void HAL_PCD_RegisterIsoOutIncpltCallback(PCD_HandleTypeDef *h, void (*cb)(PCD_HandleTypeDef*, uint8_t)) {
    (void)h;(void)cb;
}
static inline void HAL_PCD_RegisterIsoInIncpltCallback(PCD_HandleTypeDef *h, void (*cb)(PCD_HandleTypeDef*, uint8_t)) {
    (void)h;(void)cb;
}

/* ---- I2C ---- */
typedef struct {} I2C_HandleTypeDef;
#define I2C_FIRST_FRAME  0x00000001U
#define I2C_LAST_FRAME   0x00000002U
#define I2C_MEMADD_SIZE_8BIT 1

static inline HAL_StatusTypeDef HAL_I2C_Mem_Read(void *h, uint16_t a, uint8_t c, uint16_t m, uint8_t *b, uint16_t s, uint32_t t) {
    (void)h;(void)a;(void)c;(void)m;(void)b;(void)s;(void)t; memset(b, 0, s); return HAL_OK;
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

/* ---- HSEM ---- */
static inline int HAL_HSEM_IsSemTaken(int id) { (void)id; return 0; }
static inline void HAL_HSEM_Release(int id, int p) { (void)id;(void)p; }
static inline int HAL_HSEM_Take(int id, int p) { (void)id;(void)p; return 0; }
static inline int HAL_HSEM_FastTake(int id) { (void)id; return 1; }

/* ---- RCC clock macros ---- */
#define RCC_OSCILLATORTYPE_HSE  0x00000001U
#define RCC_HSE_ON              0x00000001U
#define RCC_PLL_ON              0x00000001U
#define RCC_PLLSOURCE_HSE       0x00400000U
#define RCC_CLOCKTYPE_SYSCLK    0x00000004U
#define RCC_CLOCKTYPE_HCLK      0x00000001U
#define RCC_CLOCKTYPE_PCLK1     0x00000010U
#define RCC_CLOCKTYPE_PCLK2     0x00000020U
#define RCC_SYSCLKSOURCE_PLLCLK 0x80000000U
#define RCC_SYSCLK_DIV1         0x00000000U
#define RCC_HCLK_DIV2           0x00000400U
#define __HAL_RCC_HSEM_CLK_ENABLE() do{}while(0)

/* ---- RCC types ---- */
typedef struct {
    uint32_t OscillatorType, HSEState;
    struct { uint32_t PLLState, PLLSource, PLLM, PLLN, PLLP, PLLQ, PLLR; } PLL;
} RCC_OscInitTypeDef;
typedef struct {
    uint32_t ClockType, SYSCLKSource, AHBCLKDivider, APB1CLKDivider, APB2CLKDivider, APB3CLKDivider, APB4CLKDivider;
} RCC_ClkInitTypeDef;

static inline HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *o) { (void)o; return HAL_OK; }
static inline HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *c, uint32_t f) { (void)c;(void)f; return HAL_OK; }

/* ---- PWR ---- */
#define PWR_MAINREGULATOR_ON  0
#define PWR_WAKEUP_PIN1       0x00000001U
static inline void HAL_PWREx_EnableWakeUpPin(uint32_t p) { (void)p; }
static inline void HAL_PWREx_EnterSTOP2Mode(uint32_t r) { (void)r; }

/* ---- FLASH ---- */
#define FLASH_LATENCY_4  4U

/* ---- Common HAL ---- */
static inline void HAL_Init(void) {}
static inline void HAL_Delay(uint32_t ms) { (void)ms; }
static inline uint32_t HAL_GetTick(void) { return 0; }
/* SystemCoreClockUpdate defined in syscalls.c — stub only if not linked */
extern uint32_t SystemCoreClock;

/* ---- NVIC ---- */
#define NVIC_PRIORITYGROUP_4  0x00000003U
static inline void HAL_NVIC_SetPriorityGrouping(uint32_t g) { (void)g; }
static inline void HAL_NVIC_SetPriority(IRQn_Type n, uint32_t pre, uint32_t sub) { (void)n;(void)pre;(void)sub; }
static inline void HAL_NVIC_EnableIRQ(IRQn_Type n) { (void)n; }
static inline void HAL_NVIC_DisableIRQ(IRQn_Type n) { (void)n; }
#define NVIC_EnableIRQ(irq)   HAL_NVIC_EnableIRQ(irq)
#define NVIC_SetPriority(irq, prio) HAL_NVIC_SetPriority(irq, prio, 0)

/* ---- CMSIS Core helpers ---- */
#define __NOP()    do{}while(0)
#define __DSB()    do{}while(0)
#define __ISB()    do{}while(0)
#define __DMB()    do{}while(0)

/* ---- CORDIC (missing from ST CMSIS) ---- */
#define CORDIC_CSR_RRDY_Msk       (1UL << 31)
#define CORDIC_COS_SIN            0x00000001U
#define CORDIC_CSR_RESIZE         (1UL << 16)
#define CORDIC_CSR_NITER_Pos      4U
#define CORDIC_CSR_PRECISION_Pos  8U
#define CORDIC_CSR_FUNC_COSINE    0x00020000U
#define CORDIC_CSR_PRECISION_5CYCLES 0x00000010U
#define CORDIC_CSR_SCALE_Pos      0U

/* ---- GPIO macros (from stm32h7xx_hal_gpio.h) ---- */
#define GPIO_PIN_0   ((uint16_t)0x0001)
#define GPIO_PIN_1   ((uint16_t)0x0002)
#define GPIO_PIN_2   ((uint16_t)0x0004)
#define GPIO_PIN_3   ((uint16_t)0x0008)
#define GPIO_PIN_4   ((uint16_t)0x0010)
#define GPIO_PIN_5   ((uint16_t)0x0020)
#define GPIO_PIN_6   ((uint16_t)0x0040)
#define GPIO_PIN_7   ((uint16_t)0x0080)
#define GPIO_PIN_8   ((uint16_t)0x0100)
#define GPIO_PIN_9   ((uint16_t)0x0200)
#define GPIO_PIN_10  ((uint16_t)0x0400)
#define GPIO_PIN_11  ((uint16_t)0x0800)
#define GPIO_PIN_12  ((uint16_t)0x1000)
#define GPIO_PIN_13  ((uint16_t)0x2000)
#define GPIO_PIN_14  ((uint16_t)0x4000)
#define GPIO_PIN_15  ((uint16_t)0x8000)
#define GPIO_PIN_ALL ((uint16_t)0xFFFF)

#define GPIO_PIN_SET   1
#define GPIO_PIN_RESET 0

#define GPIO_MODE_INPUT        0x00000000U
#define GPIO_MODE_OUTPUT_PP    0x00000001U
#define GPIO_MODE_OUTPUT_OD    0x00000011U
#define GPIO_MODE_AF_PP        0x00000002U

#define GPIO_NOPULL    0x00000000U
#define GPIO_PULLUP    0x00000001U
#define GPIO_PULLDOWN  0x00000002U

#define GPIO_SPEED_FREQ_LOW         0x00000000U
#define GPIO_SPEED_FREQ_MEDIUM      0x00000001U
#define GPIO_SPEED_FREQ_HIGH        0x00000002U
#define GPIO_SPEED_FREQ_VERY_HIGH   0x00000003U

typedef struct {
    uint32_t Pin, Mode, Pull, Speed, Alternate;
} GPIO_InitTypeDef;

static inline void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint16_t pin, int s) { (void)p;(void)pin;(void)s; }
static inline int  HAL_GPIO_ReadPin(GPIO_TypeDef *p, uint16_t pin) { (void)p;(void)pin; return 0; }
static inline void HAL_GPIO_Init(GPIO_TypeDef *p, GPIO_InitTypeDef *i) { (void)p;(void)i; }

/* ---- DMA macros ---- */
#define DMA_MDATAALIGN_BYTE      DMA_PDATAALIGN_BYTE
#define DMA_MDATAALIGN_HALFWORD  DMA_PDATAALIGN_HALFWORD
#define DMA_MEM_TO_PERIPH        DMA_MEMORY_TO_PERIPH
#define HAL_DMA_XFER_HALFCPLT_CB_ID 0x00000002U

/* ---- M4 domain compatibility ---- */
#if defined(CORE_CM4)
#define WWDG  WWDG1   /* M4 accesses D2 domain WWDG as WWDG1 */
#endif

/* ---- Missing TIM macros ---- */
#define TIM_BDTR_BK  (0x1000U)  /* Break input enable (same as BKE for some channels) */
#define GPIO_AF1_TIM1  1

/* ---- CORDIC (not in ST CMSIS, STM32H7 specific) ---- */
typedef struct {
    volatile uint32_t CSR;
    volatile uint32_t WDATA;
    volatile uint32_t RDATA;
} CORDIC_TypeDef;

#define CORDIC_BASE 0x40020C00UL
#define CORDIC      ((CORDIC_TypeDef *)CORDIC_BASE)

#define CORDIC_CSR_FUNC_COSINE    0x00020000U
#define CORDIC_CSR_PRECISION_5CYCLES 0x00000010U
#define CORDIC_CSR_RRDY_Msk       (1UL << 31)
#define CORDIC_COS_SIN            0x00000001U
#define CORDIC_CSR_RESIZE         (1UL << 16)
#define CORDIC_CSR_NITER_Pos      4U
#define CORDIC_CSR_PRECISION_Pos  8U
#define CORDIC_CSR_SCALE_Pos      0U

/* ---- ADC ---- */
#define ADC_CFGR_JEXTEN_Pos   (14U)
#define ADC_CFG_JEXTEN_0      (0x1UL << ADC_CFGR_JEXTEN_Pos)
#define ADC12_CCR_JSWSTART    ((uint32_t)0x02000000)

/* ---- gpio_pin_cfg_t compatible definition ---- */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    uint32_t mode, pull, speed, alternate;
    uint8_t init_state;
    const char *purpose;
} gpio_pin_cfg_t;

/* ---- RCC clock enable macros ---- */
#define __HAL_RCC_GPIOA_CLK_ENABLE()  do{}while(0)
#define __HAL_RCC_GPIOB_CLK_ENABLE()  do{}while(0)
#define __HAL_RCC_GPIOC_CLK_ENABLE()  do{}while(0)
#define __HAL_RCC_GPIOD_CLK_ENABLE()  do{}while(0)
#define __HAL_RCC_GPIOE_CLK_ENABLE()  do{}while(0)
#define __HAL_RCC_GPIOF_CLK_ENABLE()  do{}while(0)
#define __HAL_RCC_GPIOG_CLK_ENABLE()  do{}while(0)
#define __HAL_RCC_GPIOH_CLK_ENABLE()  do{}while(0)
#define __HAL_RCC_GPIOI_CLK_ENABLE()  do{}while(0)
#define __HAL_RCC_TIM1_CLK_ENABLE()   do{}while(0)
#define __HAL_RCC_TIM8_CLK_ENABLE()   do{}while(0)
#define __HAL_RCC_SPI1_CLK_ENABLE()   do{}while(0)
#define __HAL_RCC_SPI6_CLK_ENABLE()   do{}while(0)
#define __HAL_RCC_I2C1_CLK_ENABLE()   do{}while(0)
#define __HAL_RCC_I2C3_CLK_ENABLE()   do{}while(0)
#define __HAL_RCC_UART7_CLK_ENABLE()  do{}while(0)
#define __HAL_RCC_DMA1_CLK_ENABLE()   do{}while(0)
#define __HAL_RCC_DMA2_CLK_ENABLE()   do{}while(0)
#define __HAL_RCC_HSEM_CLK_ENABLE()   do{}while(0)
#define __HAL_RCC_ADC3_CLK_ENABLE()   do{}while(0)
#define __HAL_RCC_WWDG_CLK_ENABLE()   do{}while(0)
#define __HAL_RCC_USB_OTG_FS_CLK_ENABLE() do{}while(0)
#define __HAL_RCC_DMA1_CLK_DISABLE()  do{}while(0)
#define __HAL_RCC_DMA2_CLK_DISABLE()  do{}while(0)
#define __HAL_RCC_SPI6_CLK_DISABLE()  do{}while(0)
#define __HAL_RCC_USB_OTG_FS_CLK_DISABLE() do{}while(0)

extern uint32_t SystemCoreClock;

#ifdef __cplusplus
}
#endif
#endif /* STM32H7XX_HAL_STUB_H */
