/**
 * @file    ipc_spi.c
 * @brief   SPI Slave DMA transport for STM32H747 <-> RK3566 IPC.
 *
 * SPI6 configured as slave (AF5 on GPIOG):
 *   SCK  = PG13, MISO = PG12, MOSI = PG14, NSS = PG9
 * READY GPIO: PE1 — pulled high when l1_report_t is loaded and ready for master.
 *
 * Transaction: 64-byte full-duplex DMA.
 *   STM32 TX: l1_report_t (45 B) + padding
 *   STM32 RX: l2_cmd_t   (12 B) + padding
 * CRC16 verified on RX; invalid commands are discarded.
 */

#include "stm32h7xx_hal.h"
#include "ipc_proto.h"
#include "ipc_transport.h"
#include "m7_config.h"
#include <string.h>

/* ---- SPI handle & DMA ---- */
static SPI_HandleTypeDef   gSpi6;
static DMA_HandleTypeDef   gDmaRx, gDmaTx;
static volatile bool       gSpiReady = false;
static volatile bool       gTxDone   = false;
static volatile bool       gRxDone   = false;

/* ---- Double-buffer TX (ping/pong) ---- */
static uint8_t  gTxBuf[2][IPC_SPI_FRAME] __attribute__((aligned(32)));
static uint8_t  gRxBuf[IPC_SPI_FRAME]     __attribute__((aligned(32)));
static uint8_t  gTxActive = 0;  /* 0 or 1 */

/* ---- RX ring for received l2_cmd_t ---- */
static l2_cmd_t gCmdRing[4];
static volatile uint8_t gCmdWrite = 0;
static volatile uint8_t gCmdRead  = 0;

/* ---- CRC16 table (shared with ipc_uart.c) ---- */
static const uint16_t crc16_tbl[256] = {
    0x0000,0xC0C1,0xC181,0x0140,0xC301,0x03C0,0x0280,0xC241,
    0xC601,0x06C0,0x0780,0xC741,0x0500,0xC5C1,0xC481,0x0440,
    0xCC01,0x0CC0,0x0D80,0xCD41,0x0F00,0xCFC1,0xCE81,0x0E40,
    0x0A00,0xCAC1,0xCB81,0x0B40,0xC901,0x09C0,0x0880,0xC841,
    0xD801,0x18C0,0x1980,0xD941,0x1B00,0xDBC1,0xDA81,0x1A40,
    0x1E00,0xDEC1,0xDF81,0x1F40,0xDD01,0x1DC0,0x1C80,0xDC41,
    0x1400,0xD4C1,0xD581,0x1540,0xD701,0x17C0,0x1680,0xD641,
    0xD201,0x12C0,0x1380,0xD341,0x1100,0xD1C1,0xD081,0x1040,
    0xF001,0x30C0,0x3180,0xF141,0x3300,0xF3C1,0xF281,0x3240,
    0x3600,0xF6C1,0xF781,0x3740,0xF501,0x35C0,0x3480,0xF441,
    0x3C00,0xFCC1,0xFD81,0x3D40,0xFF01,0x3FC0,0x3E80,0xFE41,
    0xFA01,0x3AC0,0x3B80,0xFB41,0x3900,0xF9C1,0xF881,0x3840,
    0x2800,0xE8C1,0xE981,0x2940,0xEB01,0x2BC0,0x2A80,0xEA41,
    0xEE01,0x2EC0,0x2F80,0xEF41,0x2D00,0xEDC1,0xEC81,0x2C40,
    0xE401,0x24C0,0x2580,0xE541,0x2700,0xE7C1,0xE681,0x2640,
    0x2200,0xE2C1,0xE381,0x2340,0xE101,0x21C0,0x2080,0xE041,
    0xA001,0x60C0,0x6180,0xA141,0x6300,0xA3C1,0xA281,0x6240,
    0x6600,0xA6C1,0xA781,0x6740,0xA501,0x65C0,0x6480,0xA441,
    0x6C00,0xACC1,0xAD81,0x6D40,0xAF01,0x6FC0,0x6E80,0xAE41,
    0xAA01,0x6AC0,0x6B80,0xAB41,0x6900,0xA9C1,0xA881,0x6840,
    0x7800,0xB8C1,0xB981,0x7940,0xBB01,0x7BC0,0x7A80,0xBA41,
    0xBE01,0x7EC0,0x7F80,0xBF41,0x7D00,0xBDC1,0xBC81,0x7C40,
    0xB401,0x74C0,0x7580,0xB541,0x7700,0xB7C1,0xB681,0x7640,
    0x7200,0xB2C1,0xB381,0x7340,0xB101,0x71C0,0x7080,0xB041,
    0x5000,0x90C1,0x9181,0x5140,0x9301,0x53C0,0x5280,0x9241,
    0x9601,0x56C0,0x5780,0x9741,0x5500,0x95C1,0x9481,0x5440,
    0x9C01,0x5CC0,0x5D80,0x9D41,0x5F00,0x9FC1,0x9E81,0x5E40,
    0x5A00,0x9AC1,0x9B81,0x5B40,0x9901,0x59C0,0x5880,0x9841,
    0x8801,0x48C0,0x4980,0x8941,0x4B00,0x8BC1,0x8A81,0x4A40,
    0x4E00,0x8EC1,0x8F81,0x4F40,0x8D01,0x4DC0,0x4C80,0x8C41,
    0x4400,0x84C1,0x8581,0x4540,0x8701,0x47C0,0x4680,0x8641,
    0x8201,0x42C0,0x4380,0x8341,0x4100,0x81C1,0x8081,0x4040
};

static uint16_t crc16_ibm(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    while (len--) crc = (crc >> 8) ^ crc16_tbl[(crc ^ *data++) & 0xFF];
    return crc;
}

/* ---- DMA callbacks ---- */
static void spi_dma_tx_cplt(DMA_HandleTypeDef *hdma)
{
    (void)hdma;
    gTxDone = true;
    if (gRxDone) {
        gSpiReady = true;  /* both done, ready for next */
    }
}

static void spi_dma_rx_cplt(DMA_HandleTypeDef *hdma)
{
    (void)hdma;
    gRxDone = true;
    if (gTxDone) {
        gSpiReady = true;
    }
}

static void spi_dma_error(DMA_HandleTypeDef *hdma)
{
    (void)hdma;
    HAL_SPI_DMAStop(&gSpi6);
    gTxDone = true;
    gRxDone = true;
    gSpiReady = true;
}

/* ---- Hardware init ---- */
static int spi_init(void)
{
    /* ---- GPIO: SPI6 pins on GPIOG AF5 ---- */
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};

    /* SCK  PG13, MISO PG12, MOSI PG14 */
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = IPC_SPI_GPIO_AF;

    gpio.Pin = GPIO_PIN_13; HAL_GPIO_Init(GPIOG, &gpio);  /* SCK  */
    gpio.Pin = GPIO_PIN_12; HAL_GPIO_Init(GPIOG, &gpio);  /* MISO */
    gpio.Pin = GPIO_PIN_14; HAL_GPIO_Init(GPIOG, &gpio);  /* MOSI */

    /* NSS PG9 */
    gpio.Pin  = GPIO_PIN_9;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOG, &gpio);

    /* READY pin PE1: output, push-pull, low by default */
    gpio.Mode      = GPIO_MODE_OUTPUT_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = 0;
    gpio.Pin       = IPC_SPI_RDY_PIN;
    HAL_GPIO_Init(IPC_SPI_RDY_PORT, &gpio);
    HAL_GPIO_WritePin(IPC_SPI_RDY_PORT, IPC_SPI_RDY_PIN, GPIO_PIN_RESET);

    /* ---- SPI6 peripheral clock ---- */
    __HAL_RCC_SPI6_CLK_ENABLE();

    /* ---- SPI6 Slave config ---- */
    memset(&gSpi6, 0, sizeof(gSpi6));
    gSpi6.Instance               = IPC_SPI;
    gSpi6.Init.Mode              = SPI_MODE_SLAVE;
    gSpi6.Init.Direction         = SPI_DIRECTION_2LINES;
    gSpi6.Init.DataSize          = SPI_DATASIZE_8BIT;
    gSpi6.Init.CLKPolarity       = SPI_POLARITY_LOW;
    gSpi6.Init.CLKPhase          = SPI_PHASE_1EDGE;
    gSpi6.Init.NSS               = SPI_NSS_HARD_INPUT;
    gSpi6.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    gSpi6.Init.TIMode            = SPI_TIMODE_DISABLE;
    gSpi6.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    gSpi6.Init.CRCPolynomial     = 7;
    gSpi6.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
    if (HAL_SPI_Init(&gSpi6) != HAL_OK) return -1;

    /* ---- DMA: SPI6_RX -> DMA2_Stream5, Channel 6 ---- */
    __HAL_RCC_DMA2_CLK_ENABLE();
    memset(&gDmaRx, 0, sizeof(gDmaRx));
    gDmaRx.Instance                 = DMA2_Stream5;
    gDmaRx.Init.Channel             = DMA_CHANNEL_6;
    gDmaRx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    gDmaRx.Init.PeriphInc           = DMA_PINC_DISABLE;
    gDmaRx.Init.MemInc              = DMA_MINC_ENABLE;
    gDmaRx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    gDmaRx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    gDmaRx.Init.Mode                = DMA_NORMAL;
    gDmaRx.Init.Priority            = DMA_PRIORITY_HIGH;
    gDmaRx.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
    gDmaRx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    gDmaRx.Init.MemBurst            = DMA_MBURST_INCR4;
    gDmaRx.Init.PeriphBurst         = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&gDmaRx) != HAL_OK) return -2;

    __HAL_LINKDMA(&gSpi6, hdmarx, gDmaRx);
    HAL_DMA_RegisterCallback(&gDmaRx, HAL_DMA_XFER_CPLT_CB_ID, spi_dma_rx_cplt);
    HAL_DMA_RegisterCallback(&gDmaRx, HAL_DMA_XFER_ERROR_CB_ID, spi_dma_error);

    /* ---- DMA: SPI6_TX -> DMA2_Stream6, Channel 7 ---- */
    memset(&gDmaTx, 0, sizeof(gDmaTx));
    gDmaTx.Instance                 = DMA2_Stream6;
    gDmaTx.Init.Channel             = DMA_CHANNEL_7;
    gDmaTx.Init.Direction           = DMA_MEM_TO_PERIPH;
    gDmaTx.Init.PeriphInc           = DMA_PINC_DISABLE;
    gDmaTx.Init.MemInc              = DMA_MINC_ENABLE;
    gDmaTx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    gDmaTx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    gDmaTx.Init.Mode                = DMA_NORMAL;
    gDmaTx.Init.Priority            = DMA_PRIORITY_HIGH;
    gDmaTx.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
    gDmaTx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    gDmaTx.Init.MemBurst            = DMA_MBURST_INCR4;
    gDmaTx.Init.PeriphBurst         = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&gDmaTx) != HAL_OK) return -3;

    __HAL_LINKDMA(&gSpi6, hdmatx, gDmaTx);
    HAL_DMA_RegisterCallback(&gDmaTx, HAL_DMA_XFER_CPLT_CB_ID, spi_dma_tx_cplt);
    HAL_DMA_RegisterCallback(&gDmaTx, HAL_DMA_XFER_ERROR_CB_ID, spi_dma_error);

    /* ---- NVIC ---- */
    HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 6, 5);
    HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
    HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 6, 6);
    HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

    /* Clear buffers */
    memset(gTxBuf, 0, sizeof(gTxBuf));
    memset(gRxBuf, 0, sizeof(gRxBuf));
    gTxActive = 0;
    gCmdWrite = 0;
    gCmdRead  = 0;
    gSpiReady = true;

    return 0;
}

/* ---- Send l1_report_t (load TX buffer, arm DMA, signal master) ---- */
static int spi_send(const uint8_t *buf, uint16_t len)
{
    if (len > IPC_SPI_FRAME) return -1;
    if (!gSpiReady) return -2;  /* previous transaction still in flight */

    gTxDone = false;
    gRxDone = false;
    gSpiReady = false;

    /* Load inactive TX buffer */
    uint8_t active = gTxActive;
    memset(gTxBuf[active], 0, IPC_SPI_FRAME);
    memcpy(gTxBuf[active], buf, len);

    /* Arm full-duplex DMA transfer */
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive_DMA(
        &gSpi6, gTxBuf[active], gRxBuf, IPC_SPI_FRAME);
    if (st != HAL_OK) {
        gSpiReady = true;
        return -3;
    }

    /* Signal master: data ready */
    HAL_GPIO_WritePin(IPC_SPI_RDY_PORT, IPC_SPI_RDY_PIN, GPIO_PIN_SET);

    return (int)len;
}

/* ---- Poll for completed RX and parse l2_cmd_t ---- */
static int spi_recv(uint8_t *buf, uint16_t capacity, uint32_t timeout_ms)
{
    (void)timeout_ms;

    /* Check ring buffer for parsed commands */
    uint8_t rd = gCmdRead;
    if (rd != gCmdWrite) {
        l2_cmd_t *cmd = &gCmdRing[rd];
        if (capacity >= sizeof(l2_cmd_t)) {
            memcpy(buf, cmd, sizeof(l2_cmd_t));
        }
        gCmdRead = (rd + 1) & 3;
        return sizeof(l2_cmd_t);
    }
    return 0;  /* no command available */
}

/* ---- Called after DMA completes: validate & enqueue received command ---- */
static bool spi_parse_rx(void)
{
    gTxActive ^= 1;  /* swap ping/pong for next send */

    /* Clear READY signal */
    HAL_GPIO_WritePin(IPC_SPI_RDY_PORT, IPC_SPI_RDY_PIN, GPIO_PIN_RESET);

    if (gRxBuf[0] != IPC_HEAD_RK) return false;

    uint16_t crc_calc = crc16_ibm(gRxBuf, sizeof(l2_cmd_t) - 2);
    uint16_t crc_rcv  = *(uint16_t *)(gRxBuf + sizeof(l2_cmd_t) - 2);
    if (crc_calc != crc_rcv) return false;

    l2_cmd_t *dst = &gCmdRing[gCmdWrite];
    memcpy(dst, gRxBuf, sizeof(l2_cmd_t));
    uint8_t next = (gCmdWrite + 1) & 3;
    if (next != gCmdRead) gCmdWrite = next;  /* drop if ring full */

    return true;
}

static bool spi_ready(void)
{
    return gSpiReady;
}

/* ---- Non-blocking poll: parse RX if DMA completed ---- */
static void spi_process(void)
{
    if (gTxDone && gRxDone) {
        spi_parse_rx();
    }
}

static void spi_deinit(void)
{
    HAL_SPI_DMAStop(&gSpi6);
    HAL_DMA_DeInit(&gDmaRx);
    HAL_DMA_DeInit(&gDmaTx);
    HAL_SPI_DeInit(&gSpi6);
    __HAL_RCC_DMA2_CLK_DISABLE();
    __HAL_RCC_SPI6_CLK_DISABLE();
    gSpiReady = false;
}

const ipc_transport_t ipc_transport_spi = {
    .init    = spi_init,
    .send    = spi_send,
    .recv    = spi_recv,
    .ready   = spi_ready,
    .process = spi_process,
    .type    = IPC_TRANSPORT_SPI,
    .name    = "SPI6-Slave-20MHz"
};
