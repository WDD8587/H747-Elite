/**
 * @file    dma_config.c
 * @brief   All DMA channel configurations for STM32H747 system.
 *
 * Configured streams:
 *   TIM1_UP  -> DMA1_Stream4  (ADC1, trigger for regular group)
 *   SPI1_TX  -> DMA2_Stream3  (ICM-20948 IMU TX)
 *   SPI1_RX  -> DMA2_Stream2  (ICM-20948 IMU RX)
 *   UART7_TX -> DMA1_Stream1  (debug/serial TX)
 *   UART7_RX -> DMA1_Stream0  (debug/serial RX)
 *
 * Each stream is configured with:
 *   - Circular mode (for continuous sampling/streaming)
 *   - Half-transfer and transfer-complete interrupts
 *   - Configurable priority (LOW, MEDIUM, HIGH, VERY_HIGH)
 */

#include "stm32h7xx_hal.h"
#include "dma_config.h"

/* ---------------------------------------------------------------------------
 * DMA descriptor table
 * -------------------------------------------------------------------------*/

typedef struct {
    DMA_Stream_TypeDef *stream;      /* Stream base address          */
    uint32_t            channel;     /* Channel (DMA_CHANNEL_x)      */
    uint32_t            direction;   /* PERIPH_TO_MEM, MEM_TO_PERIPH, MEM_TO_MEM */
    uint32_t            periph_inc;  /* DMA_PINC_ENABLE / DISABLE    */
    uint32_t            mem_inc;     /* DMA_MINC_ENABLE / DISABLE    */
    uint32_t            periph_dwidth; /* DMA_PDATAALIGN_BYTE/HALFWORD/WORD */
    uint32_t            mem_dwidth;  /* DMA_MDATAALIGN_BYTE/HALFWORD/WORD    */
    uint32_t            mode;        /* DMA_NORMAL / DMA_CIRCULAR    */
    uint32_t            priority;    /* DMA_PRIORITY_LOW/MEDIUM/HIGH/VERY_HIGH */
    uint32_t            fifo_mode;   /* DMA_FIFOMODE_ENABLE / DISABLE */
    uint32_t            fifo_thresh; /* DMA_FIFO_THRESHOLD_1QUARTERFULL etc. */
    uint32_t            mem_burst;   /* DMA_MBURST_SINGLE / INCR4 / INCR8 / INCR16 */
    uint32_t            periph_burst;/* DMA_PBURST_SINGLE / INCR4 / INCR8 / INCR16 */
    IRQn_Type           irq;         /* NVIC interrupt number       */
    uint32_t            irq_prio;    /* NVIC preempt priority        */
    const char         *purpose;     /* Description                  */
} dma_stream_cfg_t;

static const dma_stream_cfg_t dma_config[] = {
    /* TIM1_UP -> DMA1_Stream4, Channel6 : ADC1 trigger */
    {DMA1_Stream4, DMA_CHANNEL_6,
     DMA_PERIPH_TO_MEMORY,
     DMA_PINC_DISABLE, DMA_MINC_ENABLE,
     DMA_PDATAALIGN_HALFWORD, DMA_MDATAALIGN_HALFWORD,
     DMA_CIRCULAR,
     DMA_PRIORITY_HIGH,
     DMA_FIFOMODE_ENABLE, DMA_FIFO_THRESHOLD_FULL,
     DMA_MBURST_SINGLE, DMA_PBURST_SINGLE,
     DMA1_Stream4_IRQn, 6,
     "TIM1_UP -> ADC1 : trigger ADC regular group via timer"},

    /* SPI1_TX -> DMA2_Stream3, Channel3 : ICM-20948 TX */
    {DMA2_Stream3, DMA_CHANNEL_3,
     DMA_MEM_TO_PERIPH,
     DMA_PINC_DISABLE, DMA_MINC_ENABLE,
     DMA_PDATAALIGN_BYTE, DMA_MDATAALIGN_BYTE,
     DMA_NORMAL,
     DMA_PRIORITY_HIGH,
     DMA_FIFOMODE_ENABLE, DMA_FIFO_THRESHOLD_1QUARTERFULL,
     DMA_MBURST_SINGLE, DMA_PBURST_SINGLE,
     DMA2_Stream3_IRQn, 6,
     "SPI1 MOSI -> ICM-20948 : IMU register write DMA"},

    /* SPI1_RX -> DMA2_Stream2, Channel3 : ICM-20948 RX */
    {DMA2_Stream2, DMA_CHANNEL_3,
     DMA_PERIPH_TO_MEMORY,
     DMA_PINC_DISABLE, DMA_MINC_ENABLE,
     DMA_PDATAALIGN_BYTE, DMA_MDATAALIGN_BYTE,
     DMA_NORMAL,
     DMA_PRIORITY_HIGH,
     DMA_FIFOMODE_ENABLE, DMA_FIFO_THRESHOLD_1QUARTERFULL,
     DMA_MBURST_SINGLE, DMA_PBURST_SINGLE,
     DMA2_Stream2_IRQn, 6,
     "SPI1 MISO <- ICM-20948 : IMU register read DMA"},

    /* UART7_TX -> DMA1_Stream1, Channel0 : debug serial TX */
    {DMA1_Stream1, DMA_CHANNEL_0,
     DMA_MEM_TO_PERIPH,
     DMA_PINC_DISABLE, DMA_MINC_ENABLE,
     DMA_PDATAALIGN_BYTE, DMA_MDATAALIGN_BYTE,
     DMA_NORMAL,
     DMA_PRIORITY_LOW,
     DMA_FIFOMODE_ENABLE, DMA_FIFO_THRESHOLD_1QUARTERFULL,
     DMA_MBURST_SINGLE, DMA_PBURST_SINGLE,
     DMA1_Stream1_IRQn, 4,
     "UART7 TX : debug printf output DMA"},

    /* UART7_RX -> DMA1_Stream0, Channel0 : debug serial RX */
    {DMA1_Stream0, DMA_CHANNEL_0,
     DMA_PERIPH_TO_MEMORY,
     DMA_PINC_DISABLE, DMA_MINC_ENABLE,
     DMA_PDATAALIGN_BYTE, DMA_MDATAALIGN_BYTE,
     DMA_CIRCULAR,
     DMA_PRIORITY_LOW,
     DMA_FIFOMODE_ENABLE, DMA_FIFO_THRESHOLD_1QUARTERFULL,
     DMA_MBURST_SINGLE, DMA_PBURST_SINGLE,
     DMA1_Stream0_IRQn, 4,
     "UART7 RX : debug serial receive DMA (circular buffer)"},

    /* SPI6_RX -> DMA2_Stream5, Channel6 : IPC SPI slave RX */
    {DMA2_Stream5, DMA_CHANNEL_6,
     DMA_PERIPH_TO_MEMORY,
     DMA_PINC_DISABLE, DMA_MINC_ENABLE,
     DMA_PDATAALIGN_BYTE, DMA_MDATAALIGN_BYTE,
     DMA_NORMAL,
     DMA_PRIORITY_HIGH,
     DMA_FIFOMODE_ENABLE, DMA_FIFO_THRESHOLD_FULL,
     DMA_MBURST_INCR4, DMA_PBURST_SINGLE,
     DMA2_Stream5_IRQn, 6,
     "SPI6 RX : IPC SPI slave receive from RK3566"},

    /* SPI6_TX -> DMA2_Stream6, Channel7 : IPC SPI slave TX */
    {DMA2_Stream6, DMA_CHANNEL_7,
     DMA_MEM_TO_PERIPH,
     DMA_PINC_DISABLE, DMA_MINC_ENABLE,
     DMA_PDATAALIGN_BYTE, DMA_MDATAALIGN_BYTE,
     DMA_NORMAL,
     DMA_PRIORITY_HIGH,
     DMA_FIFOMODE_ENABLE, DMA_FIFO_THRESHOLD_FULL,
     DMA_MBURST_INCR4, DMA_PBURST_SINGLE,
     DMA2_Stream6_IRQn, 6,
     "SPI6 TX : IPC SPI slave transmit to RK3566"},
};

static const uint32_t dma_config_count =
    sizeof(dma_config) / sizeof(dma_config[0]);

/* ---------------------------------------------------------------------------
 * DMA clock enable helpers
 * -------------------------------------------------------------------------*/

/**
 * @brief  Enable the clock for a DMA controller.
 *
 * @param  stream  DMA stream pointer.
 */
static void dma_enable_clock(DMA_Stream_TypeDef *stream)
{
    /* Determine which DMA controller this stream belongs to */
    /* DMA1: 0x40026000, DMA2: 0x40026400 */
    if ((uint32_t)stream >= 0x40026000UL &&
        (uint32_t)stream <  0x40026400UL) {
        __HAL_RCC_DMA1_CLK_ENABLE();
    } else if ((uint32_t)stream >= 0x40026400UL &&
               (uint32_t)stream <  0x40026800UL) {
        __HAL_RCC_DMA2_CLK_ENABLE();
    }
}

/* ---------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------*/

/**
 * @brief  Initialise all DMA streams defined in the configuration table.
 *
 * Must be called after clock tree init and before any peripheral that
 * uses DMA. Sets up NVIC priorities as well.
 */
void dma_config_init_all(void)
{
    DMA_HandleTypeDef hdma;
    DMA_Stream_TypeDef *last_stream = NULL;

    for (uint32_t i = 0; i < dma_config_count; i++)
    {
        const dma_stream_cfg_t *cfg = &dma_config[i];

        /* Skip duplicate clock enable */
        if (cfg->stream != last_stream) {
            dma_enable_clock(cfg->stream);
            last_stream = cfg->stream;
        }

        /* Zero the handle and configure */
        memset(&hdma, 0, sizeof(hdma));
        hdma.Instance                 = cfg->stream;
        hdma.Init.Channel             = cfg->channel;
        hdma.Init.Direction           = cfg->direction;
        hdma.Init.PeriphInc           = cfg->periph_inc;
        hdma.Init.MemInc              = cfg->mem_inc;
        hdma.Init.PeriphDataAlignment = cfg->periph_dwidth;
        hdma.Init.MemDataAlignment    = cfg->mem_dwidth;
        hdma.Init.Mode                = cfg->mode;
        hdma.Init.Priority            = cfg->priority;
        hdma.Init.FIFOMode            = cfg->fifo_mode;
        hdma.Init.FIFOThreshold       = cfg->fifo_thresh;
        hdma.Init.MemBurst            = cfg->mem_burst;
        hdma.Init.PeriphBurst         = cfg->periph_burst;

        if (HAL_DMA_Init(&hdma) != HAL_OK)
        {
            /* Initialisation failure — trap in debug builds */
            while (1) { __NOP(); }
        }

        /* Configure NVIC for this stream */
        HAL_NVIC_SetPriority(cfg->irq, cfg->irq_prio, 0);
        HAL_NVIC_EnableIRQ(cfg->irq);
    }
}

/**
 * @brief  De-initialise a specific DMA stream (for reconfiguration).
 *
 * @param  stream  Pointer to the stream (e.g., DMA1_Stream0).
 */
void dma_config_deinit(DMA_Stream_TypeDef *stream)
{
    DMA_HandleTypeDef hdma;
    memset(&hdma, 0, sizeof(hdma));
    hdma.Instance = stream;
    HAL_DMA_DeInit(&hdma);
}

/**
 * @brief  Register HAL DMA callbacks for a stream.
 *
 * Typically called from HAL_UART_Init or HAL_SPI_Init via the
 * HAL_DMA_RegisterCallback() function. This wrapper handles
 * the registration of:
 *   - XferCpltCallback
 *   - XferHalfCpltCallback
 *   - XferErrorCallback
 *
 * @param  hdma     DMA handle.
 * @param  cplt_cb  Transfer-complete callback.
 * @param  half_cb  Half-transfer callback.
 * @param  err_cb   Error callback.
 */
void dma_config_register_callbacks(DMA_HandleTypeDef *hdma,
                                    void (*cplt_cb)(DMA_HandleTypeDef *),
                                    void (*half_cb)(DMA_HandleTypeDef *),
                                    void (*err_cb)(DMA_HandleTypeDef *))
{
    if (cplt_cb != NULL) {
        HAL_DMA_RegisterCallback(hdma, HAL_DMA_XFER_CPLT_CB_ID, cplt_cb);
    }
    if (half_cb != NULL) {
        HAL_DMA_RegisterCallback(hdma, HAL_DMA_XFER_HALFCPLT_CB_ID, half_cb);
    }
    if (err_cb != NULL) {
        HAL_DMA_RegisterCallback(hdma, HAL_DMA_XFER_ERROR_CB_ID, err_cb);
    }
}
