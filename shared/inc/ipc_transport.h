#ifndef IPC_TRANSPORT_H
#define IPC_TRANSPORT_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    IPC_TRANSPORT_UART = 0,
    IPC_TRANSPORT_SPI  = 1,
    IPC_TRANSPORT_USB  = 2,
    IPC_TRANSPORT_COUNT
} ipc_transport_type_t;

typedef struct ipc_transport {
    int  (*init)(void);
    int  (*send)(const uint8_t *buf, uint16_t len);
    int  (*recv)(uint8_t *buf, uint16_t len, uint32_t timeout_ms);
    bool (*ready)(void);
    void (*process)(void);
    void (*deinit)(void);
    ipc_transport_type_t type;
    const char *name;
} ipc_transport_t;

const ipc_transport_t *ipc_transport_get(ipc_transport_type_t type);
const ipc_transport_t *ipc_transport_get_active(void);
void ipc_transport_select(ipc_transport_type_t type);

#endif
