/* Minimal HAL stub for CI cross-compilation.
 * Provides type definitions and no-op implementations for all
 * STM32H7 peripherals used by this project.
 * Produces compilable but non-functional ELF files.
 */
#ifndef STM32H7XX_HAL_STUB_H
#define STM32H7XX_HAL_STUB_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- HAL Status ---- */
typedef enum { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2, HAL_TIMEOUT = 3 } HAL_StatusTypeDef;

/* ---- CMSIS Core ---- */
#define __NOP()        do{}while(0)
#define __DSB()        do{}while(0)
#define __ISB()        do{}while(0)
#define __DMB()        do{}while(0)

/* ---- NVIC ---- */
typedef enum {
  NonMaskableInt_IRQn = -14, MemoryManagement_IRQn = -12, BusFault_IRQn = -11,
  UsageFault_IRQn = -10, SVCall_IRQn = -5, DebugMonitor_IRQn = -4,
  PendSV_IRQn = -2, SysTick_IRQn = -1,
  WWDG_IRQn = 0, PVD_PVM_IRQn = 1, TAMP_STAMP_IRQn = 2, RTC_WKUP_IRQn = 3,
  FLASH_IRQn = 4, RCC_IRQn = 5,
  EXTI0_IRQn = 6, EXTI1_IRQn = 7, EXTI2_IRQn = 8, EXTI3_IRQn = 9, EXTI4_IRQn = 10,
  DMA1_Stream0_IRQn = 11, DMA1_Stream1_IRQn = 12, DMA1_Stream2_IRQn = 13,
  DMA1_Stream3_IRQn = 14, DMA1_Stream4_IRQn = 15, DMA1_Stream5_IRQn = 16,
  DMA1_Stream6_IRQn = 17, DMA1_Stream7_IRQn = 47,
  ADC_IRQn = 18,
  TIM1_BRK_IRQn = 24, TIM1_UP_IRQn = 25, TIM1_TRG_COM_IRQn = 26, TIM1_CC_IRQn = 27,
  TIM2_IRQn = 28, TIM3_IRQn = 29, TIM8_BRK_TIM12_IRQn = 43, TIM8_UP_TIM13_IRQn = 44,
  TIM8_TRG_COM_TIM14_IRQn = 45, TIM8_CC_IRQn = 46,
  I2C1_EV_IRQn = 31, I2C1_ER_IRQn = 32, I2C3_EV_IRQn = 72, I2C3_ER_IRQn = 73,
  SPI1_IRQn = 35, SPI2_IRQn = 36, SPI6_IRQn = 86,
  UART7_IRQn = 82, UART8_IRQn = 83,
  DMA2_Stream0_IRQn = 56, DMA2_Stream1_IRQn = 57, DMA2_Stream2_IRQn = 58,
  DMA2_Stream3_IRQn = 59, DMA2_Stream4_IRQn = 60, DMA2_Stream5_IRQn = 61,
  DMA2_Stream6_IRQn = 62, DMA2_Stream7_IRQn = 63,
  OTG_FS_IRQn = 67, OTG_HS_IRQn = 77,
  FDCAN1_IRQn = 19, FDCAN2_IRQn = 20,
  SVC_Handler_IRQn = SVCall_IRQn, TIM1_UP_TIM10_IRQn = TIM1_UP_IRQn,
  TIM8_UP_TIM13_IRQn = TIM8_UP_TIM13_IRQn,
} IRQn_Type;

#define NVIC_PRIORITYGROUP_4  0x00000003U

static inline void HAL_NVIC_SetPriorityGrouping(uint32_t g) { (void)g; }
static inline void HAL_NVIC_SetPriority(IRQn_Type n, uint32_t pre, uint32_t sub) { (void)n;(void)pre;(void)sub; }
static inline void HAL_NVIC_EnableIRQ(IRQn_Type n) { (void)n; }
static inline void HAL_NVIC_DisableIRQ(IRQn_Type n) { (void)n; }
static inline uint32_t HAL_NVIC_GetPriority(IRQn_Type n) { (void)n; return 0; }

/* ---- GPIO ---- */
typedef struct { volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFRL, AFRH; } GPIO_TypeDef;

#define GPIOA  ((GPIO_TypeDef*)0x1000)
#define GPIOB  ((GPIO_TypeDef*)0x1004)
#define GPIOC  ((GPIO_TypeDef*)0x1008)
#define GPIOD  ((GPIO_TypeDef*)0x100C)
#define GPIOE  ((GPIO_TypeDef*)0x1010)
#define GPIOF  ((GPIO_TypeDef*)0x1014)
#define GPIOG  ((GPIO_TypeDef*)0x1018)
#define GPIOH  ((GPIO_TypeDef*)0x101C)
#define GPIOI  ((GPIO_TypeDef*)0x1020)

#define GPIO_PIN_0   0x0001U
#define GPIO_PIN_1   0x0002U
#define GPIO_PIN_2   0x0004U
#define GPIO_PIN_3   0x0008U
#define GPIO_PIN_4   0x0010U
#define GPIO_PIN_5   0x0020U
#define GPIO_PIN_6   0x0040U
#define GPIO_PIN_7   0x0080U
#define GPIO_PIN_8   0x0100U
#define GPIO_PIN_9   0x0200U
#define GPIO_PIN_10  0x0400U
#define GPIO_PIN_11  0x0800U
#define GPIO_PIN_12  0x1000U
#define GPIO_PIN_13  0x2000U
#define GPIO_PIN_14  0x4000U
#define GPIO_PIN_15  0x8000U
#define GPIO_PIN_ALL 0xFFFFU

#define GPIO_PIN_SET   1
#define GPIO_PIN_RESET 0

/* GPIO modes */
#define GPIO_MODE_INPUT     0x00000000U
#define GPIO_MODE_OUTPUT_PP 0x00000001U
#define GPIO_MODE_OUTPUT_OD 0x00000011U
#define GPIO_MODE_AF_PP     0x00000002U
#define GPIO_MODE_AF_OD     0x00000012U

/* GPIO pull */
#define GPIO_NOPULL    0x00000000U
#define GPIO_PULLUP    0x00000001U
#define GPIO_PULLDOWN  0x00000002U

/* GPIO speed */
#define GPIO_SPEED_FREQ_LOW        0x00000000U
#define GPIO_SPEED_FREQ_MEDIUM     0x00000001U
#define GPIO_SPEED_FREQ_HIGH       0x00000002U
#define GPIO_SPEED_FREQ_VERY_HIGH  0x00000003U

/* GPIO AF numbers */
#define GPIO_AF0_SYSTEM   0
#define GPIO_AF1_TIM1     1
#define GPIO_AF2_TIM2     2
#define GPIO_AF3_TIM3     3
#define GPIO_AF3_TIM8     3
#define GPIO_AF4_I2C1     4
#define GPIO_AF4_I2C3     4
#define GPIO_AF5_SPI1     5
#define GPIO_AF5_SPI2     5
#define GPIO_AF5_SPI6     5
#define GPIO_AF7_UART4    7
#define GPIO_AF7_UART7    7
#define GPIO_AF8_UART8    8
#define GPIO_AF9_FDCAN1   9
#define GPIO_AF10_OTG_FS  10
#define GPIO_AF10_OTG_HS  10

typedef struct {
    uint32_t Pin, Mode, Pull, Speed, Alternate;
} GPIO_InitTypeDef;

static inline void HAL_GPIO_Init(GPIO_TypeDef *p, GPIO_InitTypeDef *i) { (void)p;(void)i; }
static inline void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint16_t pin, int s) { (void)p;(void)pin;(void)s; }
static inline int  HAL_GPIO_ReadPin(GPIO_TypeDef *p, uint16_t pin) { (void)p;(void)pin; return 0; }

/* ---- RCC Clock enable ---- */
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
#define __HAL_RCC_TIM2_CLK_ENABLE()   do{}while(0)
#define __HAL_RCC_TIM3_CLK_ENABLE()   do{}while(0)
#define __HAL_RCC_TIM8_CLK_ENABLE()   do{}while(0)
#define __HAL_RCC_SPI1_CLK_ENABLE()   do{}while(0)
#define __HAL_RCC_SPI2_CLK_ENABLE()   do{}while(0)
#define __HAL_RCC_SPI6_CLK_ENABLE()   do{}while(0)
#define __HAL_RCC_I2C1_CLK_ENABLE()   do{}while(0)
#define __HAL_RCC_I2C2_CLK_ENABLE()   do{}while(0)
#define __HAL_RCC_I2C3_CLK_ENABLE()   do{}while(0)
#define __HAL_RCC_USART1_CLK_ENABLE() do{}while(0)
#define __HAL_RCC_UART7_CLK_ENABLE()  do{}while(0)
#define __HAL_RCC_UART8_CLK_ENABLE()  do{}while(0)
#define __HAL_RCC_DMA1_CLK_ENABLE()   do{}while(0)
#define __HAL_RCC_DMA2_CLK_ENABLE()   do{}while(0)
#define __HAL_RCC_HSEM_CLK_ENABLE()   do{}while(0)
#define __HAL_RCC_ADC3_CLK_ENABLE()   do{}while(0)
#define __HAL_RCC_WWDG_CLK_ENABLE()   do{}while(0)
#define __HAL_RCC_LPTIM1_CLK_ENABLE() do{}while(0)
#define __HAL_RCC_USB_OTG_FS_CLK_ENABLE() do{}while(0)
#define __HAL_RCC_USB_OTG_HS_CLK_ENABLE() do{}while(0)
#define __HAL_RCC_FDCAN_CLK_ENABLE()  do{}while(0)
#define __HAL_RCC_SYSCFG_CLK_ENABLE() do{}while(0)
#define __HAL_RCC_DMA1_CLK_DISABLE()  do{}while(0)
#define __HAL_RCC_DMA2_CLK_DISABLE()  do{}while(0)
#define __HAL_RCC_SPI6_CLK_DISABLE()  do{}while(0)
#define __HAL_RCC_USB_OTG_FS_CLK_DISABLE() do{}while(0)

/* ---- DMA ---- */
typedef struct {} DMA_Stream_TypeDef;
typedef struct {
    DMA_Stream_TypeDef *Instance;
    struct { uint32_t Channel, Direction, PeriphInc, MemInc, PeriphDataAlignment,
             MemDataAlignment, Mode, Priority, FIFOMode, FIFOThreshold, MemBurst, PeriphBurst; } Init;
} DMA_HandleTypeDef;

#define DMA_CHANNEL_0  0
#define DMA_CHANNEL_3  3
#define DMA_CHANNEL_6  6
#define DMA_CHANNEL_7  7

#define DMA_MEMORY_TO_PERIPH  1
#define DMA_PERIPH_TO_MEMORY  0
#define DMA_MEMORY_TO_MEMORY  2

#define DMA_PINC_ENABLE   1
#define DMA_PINC_DISABLE  0
#define DMA_MINC_ENABLE   1
#define DMA_MINC_DISABLE  0

#define DMA_PDATAALIGN_BYTE     0
#define DMA_PDATAALIGN_HALFWORD 1
#define DMA_PDATAALIGN_WORD     2
#define DMA_MDATAALIGN_BYTE     0
#define DMA_MDATAALIGN_HALFWORD 1
#define DMA_MDATAALIGN_WORD     2

#define DMA_NORMAL    0
#define DMA_CIRCULAR  1

#define DMA_PRIORITY_LOW    0
#define DMA_PRIORITY_MEDIUM 1
#define DMA_PRIORITY_HIGH   2
#define DMA_PRIORITY_VERY_HIGH 3

#define DMA_FIFOMODE_ENABLE  1
#define DMA_FIFOMODE_DISABLE 0
#define DMA_FIFO_THRESHOLD_1QUARTERFULL 0
#define DMA_FIFO_THRESHOLD_HALFFULL     1
#define DMA_FIFO_THRESHOLD_3QUARTERFULL 2
#define DMA_FIFO_THRESHOLD_FULL         3

#define DMA_MBURST_SINGLE 0
#define DMA_MBURST_INCR4  1
#define DMA_MBURST_INCR8  2
#define DMA_MBURST_INCR16 3
#define DMA_PBURST_SINGLE 0
#define DMA_PBURST_INCR4  1

#define HAL_DMA_XFER_CPLT_CB_ID      0x00000001U
#define HAL_DMA_XFER_HALFCPLT_CB_ID  0x00000002U
#define HAL_DMA_XFER_ERROR_CB_ID     0x00000004U

#define DMA1_Stream0  ((DMA_Stream_TypeDef*)0x1000)
#define DMA1_Stream1  ((DMA_Stream_TypeDef*)0x1001)
#define DMA1_Stream2  ((DMA_Stream_TypeDef*)0x1002)
#define DMA1_Stream3  ((DMA_Stream_TypeDef*)0x1003)
#define DMA1_Stream4  ((DMA_Stream_TypeDef*)0x1004)
#define DMA1_Stream5  ((DMA_Stream_TypeDef*)0x1005)
#define DMA1_Stream6  ((DMA_Stream_TypeDef*)0x1006)
#define DMA1_Stream7  ((DMA_Stream_TypeDef*)0x1007)
#define DMA2_Stream0  ((DMA_Stream_TypeDef*)0x2000)
#define DMA2_Stream1  ((DMA_Stream_TypeDef*)0x2001)
#define DMA2_Stream2  ((DMA_Stream_TypeDef*)0x2002)
#define DMA2_Stream3  ((DMA_Stream_TypeDef*)0x2003)
#define DMA2_Stream4  ((DMA_Stream_TypeDef*)0x2004)
#define DMA2_Stream5  ((DMA_Stream_TypeDef*)0x2005)
#define DMA2_Stream6  ((DMA_Stream_TypeDef*)0x2006)
#define DMA2_Stream7  ((DMA_Stream_TypeDef*)0x2007)

static inline HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *h) { (void)h; return HAL_OK; }
static inline HAL_StatusTypeDef HAL_DMA_DeInit(DMA_HandleTypeDef *h) { (void)h; return HAL_OK; }
static inline void HAL_DMA_RegisterCallback(DMA_HandleTypeDef *h, uint32_t id, void (*cb)(DMA_HandleTypeDef*)) {
    (void)h;(void)id;(void)cb;
}

#define __HAL_LINKDMA(h, member, dma) do { (h)->member = (dma); } while(0)

/* ---- UART ---- */
typedef struct { uint32_t BaudRate, WordLength, StopBits, Parity, Mode, HwFlowCtl, OverSampling; } UART_InitTypeDef;
typedef struct {
    void *Instance;
    UART_InitTypeDef Init;
} UART_HandleTypeDef;

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

typedef struct {
    void *Instance;
    SPI_InitTypeDef Init;
    DMA_HandleTypeDef *hdmarx;
    DMA_HandleTypeDef *hdmatx;
} SPI_HandleTypeDef;

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

typedef struct {
    union { uint8_t bitmap; struct { uint8_t recipient:5, type:2, dir:1; } b; } bmRequestType;
    uint8_t  bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} USB_Setup_TypeDef;

#define USB_OTG_FS  ((void*)0x50000000)
#define USB_OTG_HS  ((void*)0x50040000)

#define PCD_SPEED_FULL    2
#define PCD_SPEED_HIGH    0
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

/* ---- TIM ---- */
typedef struct { volatile uint32_t CR1, CR2, SMCR, DIER, SR, EGR, CCMR1, CCMR2, CCER, CNT, PSC, ARR, RCR, CCR1, CCR2, CCR3, CCR4, BDTR, DCR, DMAR; } TIM_TypeDef;
#define TIM1  ((TIM_TypeDef*)0x40010000)
#define TIM2  ((TIM_TypeDef*)0x40000400)
#define TIM3  ((TIM_TypeDef*)0x40000800)
#define TIM8  ((TIM_TypeDef*)0x40010400)

#define TIM_BDTR_MOE  0x8000U
#define TIM_SR_UIF    0x0001U
#define TIM_CR1_CEN   0x0001U
#define TIM_BDTR_AOE  0x4000U
#define TIM_BDTR_BKE  0x1000U
#define TIM_BDTR_BKP  0x2000U
#define TIM_BDTR_OSSR 0x0008U
#define TIM_BDTR_BKF_Msk 0x00F0U
#define TIM_CCER_CC1E 0x0001U
#define TIM_CCER_CC1NE 0x0004U

/* ---- ADC ---- */
typedef struct { volatile uint32_t SR, CR1, CR2, SMPR1, SMPR2, JOFR1, JOFR2, JOFR3, JOFR4, HTR, LTR, SQR1, SQR2, SQR3, JSQR, JDR1, JDR2, JDR3, JDR4, DR; } ADC_TypeDef;
#define ADC1 ((ADC_TypeDef*)0x40012000)
#define ADC2 ((ADC_TypeDef*)0x40012100)
#define ADC3 ((ADC_TypeDef*)0x40012200)

/* ---- HSEM ---- */
static inline int HAL_HSEM_IsSemTaken(int id) { (void)id; return 0; }
static inline void HAL_HSEM_Release(int id, int p) { (void)id;(void)p; }
static inline int HAL_HSEM_Take(int id, int p) { (void)id;(void)p; return 0; }

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

/* ---- FLASH ---- */
typedef struct { volatile uint32_t ACR, KEYR, OPTKEYR, SR, CR, OPTCR, OPTCR1; } FLASH_TypeDef;
#define FLASH     ((FLASH_TypeDef*)0x52002000)
#define FLASH_BASE 0x52002000U

/* ---- WWDG ---- */
typedef struct { volatile uint32_t CR, CFR, SR; } WWDG_TypeDef;
#define WWDG ((WWDG_TypeDef*)0x40002C00)
#define WWDG_CFR_WDGTB_Pos 7
#define WWDG_CFR_W_Pos     0
#define WWDG_CFR_EWI       0x0200U
#define WWDG_CR_T_Pos      0
#define WWDG_CR_WDGA       0x0080U

/* ---- CORDIC ---- */
typedef struct { volatile uint32_t WDATA, CSR, RDATA; } CORDIC_TypeDef;
extern CORDIC_TypeDef CORDIC_MOCK;
#define CORDIC (&CORDIC_MOCK)
#define CORDIC_CSR_FUNC_COSINE  0x00020000U
#define CORDIC_CSR_PRECISION_5CYCLES 0x00000010U

/* ---- RCC ---- */
typedef struct { volatile uint32_t GCR; } RCC_TypeDef;
#define RCC ((RCC_TypeDef*)0x58024400)
#define RCC_GCR_BOOT_C2_Pos 16

/* ---- SYSCFG ---- */
typedef struct { volatile uint32_t UR0; } SYSCFG_TypeDef;
#define SYSCFG ((SYSCFG_TypeDef*)0x58000400)

/* ---- PWR ---- */
#define PWR_MAINREGULATOR_ON  0x00000000U
#define PWR_WAKEUP_PIN1       0x00000001U
static inline void HAL_PWREx_EnableWakeUpPin(uint32_t p) { (void)p; }
static inline void HAL_PWREx_EnterSTOP2Mode(uint32_t r) { (void)r; }

/* ---- Common HAL ---- */
static inline void HAL_Init(void) {}
static inline void HAL_Delay(uint32_t ms) { (void)ms; }
static inline uint32_t HAL_GetTick(void) { return 0; }
static inline void SystemCoreClockUpdate(void) {}
extern uint32_t SystemCoreClock;

/* ---- STM32H7 specific ---- */
#define STM32H747xx

/* ---- Vector table offset register ---- */
#define VTOR  ((volatile uint32_t*)0xE000ED08U)

/* ---- SRAM3 ---- */
#define SRAM3_BASE 0x20030000U

#ifdef __cplusplus
}
#endif
#endif /* STM32H7XX_HAL_STUB_H */
