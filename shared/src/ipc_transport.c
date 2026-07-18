/**
 * @file    ipc_transport.c
 * @brief   Transport registry — maps ipc_transport_type_t to active transport.
 *
 * Three transports implemented:
 *   UART — 3Mbps UART7 DMA (legacy, simplest, lowest throughput)
 *   SPI  — 20MHz SPI6 Slave DMA (full-duplex, medium throughput)
 *   USB  — 12Mbps USB CDC ACM (asymmetric, highest throughput, hot-plug)
 */

#include "ipc_transport.h"
#include "stm32h7xx_hal.h"

extern const ipc_transport_t ipc_transport_uart;
extern const ipc_transport_t ipc_transport_spi;
extern const ipc_transport_t ipc_transport_usb;

static const ipc_transport_t *gTransport = NULL;

static const ipc_transport_t *gTable[IPC_TRANSPORT_COUNT] = {
    &ipc_transport_uart,
    &ipc_transport_spi,
    &ipc_transport_usb,
};

const ipc_transport_t *ipc_transport_get(ipc_transport_type_t type)
{
    if (type >= IPC_TRANSPORT_COUNT) return NULL;
    return gTable[type];
}

const ipc_transport_t *ipc_transport_get_active(void)
{
    return gTransport;
}

void ipc_transport_select(ipc_transport_type_t type)
{
    if (type >= IPC_TRANSPORT_COUNT) return;
    if (gTransport) gTransport->deinit();
    gTransport = gTable[type];
    if (gTransport) gTransport->init();
}
