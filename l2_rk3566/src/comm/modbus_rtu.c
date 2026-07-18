/**
 * modbus_rtu.c — Modbus RTU master for motor drives
 *
 * Function codes: 0x03 (Read Holding Registers), 0x06 (Write Single Register),
 * 0x10 (Write Multiple Registers).
 * Register map:
 *   0x1000 speed_setpoint, 0x1002 current_limit,
 *   0x1100 actual_speed, 0x1102 actual_current, 0x1200 fault_code
 * CRC16 per Modbus specification.
 * UART interface with RS-485 transceiver control.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <pthread.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/stat.h>
#include <time.h>

#include "modbus_rtu.h"

/* ---------------------------------------------------------------------------
 * Constants
 * --------------------------------------------------------------------------- */
#define MODBUS_DEFAULT_BAUD      115200
#define MODBUS_DEFAULT_DEVICE    "/dev/ttyS1"
#define MODBUS_DEFAULT_TIMEOUT_MS 100
#define MODBUS_RETRY_COUNT       3
#define MODBUS_MAX_DRIVES        8

/* Register addresses */
#define REG_SPEED_SETPOINT       0x1000
#define REG_CURRENT_LIMIT        0x1002
#define REG_ACTUAL_SPEED         0x1100
#define REG_ACTUAL_CURRENT       0x1102
#define REG_FAULT_CODE           0x1200

/* RS-485 direction control GPIO (typical RK3566) */
#define RS485_GPIO_DIR           "/sys/class/gpio/gpio104/value"

/* ---------------------------------------------------------------------------
 * Modbus ADU
 * --------------------------------------------------------------------------- */
#pragma pack(push, 1)
typedef struct {
    uint8_t  slave_addr;
    uint8_t  func_code;
    union {
        /* Read Holding Registers request (0x03) */
        struct {
            uint16_t start_addr;
            uint16_t quantity;
        } read_req;
        /* Read Holding Registers response */
        struct {
            uint8_t  byte_count;
            uint16_t values[32]; /* max 32 registers per request */
        } read_resp;
        /* Write Single Register request (0x06) */
        struct {
            uint16_t reg_addr;
            uint16_t reg_value;
        } write_single;
        /* Write Multiple Registers request (0x10) */
        struct {
            uint16_t start_addr;
            uint16_t quantity;
            uint8_t  byte_count;
            uint16_t values[32];
        } write_multi_req;
        /* Write Multiple Registers response */
        struct {
            uint16_t start_addr;
            uint16_t quantity;
        } write_multi_resp;
        /* Error response */
        struct {
            uint8_t exception_code;
        } error;
    };
    uint16_t crc;
} modbus_adu_t;
#pragma pack(pop)

#define MODBUS_ADU_MIN_SIZE      4
#define MODBUS_READ_MAX_REGS     32
#define MODBUS_WRITE_MAX_REGS    32

/* ---------------------------------------------------------------------------
 * Context
 * --------------------------------------------------------------------------- */
typedef struct {
    int              uart_fd;
    int              baud;
    char             device[64];
    int              timeout_ms;

    pthread_mutex_t  lock;
    int              initialized;

    /* RS-485 direction control */
    int              rs485_gpio_fd;
} modbus_ctx_t;

static modbus_ctx_t g_ctx;

/* ---------------------------------------------------------------------------
 * CRC16 (Modbus standard)
 * --------------------------------------------------------------------------- */
static uint16_t modbus_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

/* ---------------------------------------------------------------------------
 * UART helpers
 * --------------------------------------------------------------------------- */
static int uart_open(const char *device, int baud) {
    int fd = open(device, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        fprintf(stderr, "[Modbus] Cannot open %s: %s\n", device, strerror(errno));
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, (speed_t)baud);
    cfsetispeed(&tty, (speed_t)baud);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS; /* no HW flow control */

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR | BRKINT | IGNBRK);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 2; /* 200ms timeout */

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        close(fd);
        return -1;
    }

    tcflush(fd, TCIOFLUSH);
    return fd;
}

/* ---------------------------------------------------------------------------
 * RS-485 direction control
 * --------------------------------------------------------------------------- */
static int rs485_init(void) {
    /* Export GPIO if not already */
    int fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd >= 0) {
        write(fd, "104", 3);
        close(fd);
    }

    /* Set direction to output */
    fd = open("/sys/class/gpio/gpio104/direction", O_WRONLY);
    if (fd >= 0) {
        write(fd, "out", 3);
        close(fd);
    }

    /* Open value file */
    g_ctx.rs485_gpio_fd = open(RS485_GPIO_DIR, O_WRONLY);
    if (g_ctx.rs485_gpio_fd < 0) {
        fprintf(stderr, "[Modbus] RS-485 GPIO not available\n");
        return -1;
    }
    return 0;
}

static void rs485_set_transmit(int tx) {
    if (g_ctx.rs485_gpio_fd >= 0) {
        write(g_ctx.rs485_gpio_fd, tx ? "1" : "0", 1);
    }
}

/* ---------------------------------------------------------------------------
 * Send modbus frame
 * --------------------------------------------------------------------------- */
static int modbus_send(const uint8_t *data, size_t len) {
    /* Set RS-485 to transmit */
    rs485_set_transmit(1);
    usleep(50); /* allow transceiver to switch */

    size_t written = 0;
    while (written < len) {
        ssize_t n = write(g_ctx.uart_fd, data + written, len - written);
        if (n < 0) { rs485_set_transmit(0); return -1; }
        written += (size_t)n;
    }

    /* Flush and switch to receive */
    tcdrain(g_ctx.uart_fd);
    usleep(100);
    rs485_set_transmit(0);

    return 0;
}

/* ---------------------------------------------------------------------------
 * Receive modbus frame with timeout
 * --------------------------------------------------------------------------- */
static int modbus_receive(uint8_t *buf, size_t cap, size_t *out_len) {
    uint8_t byte;
    size_t pos = 0;
    int64_t start_ms = (int64_t)(time(NULL) * 1000);
    int64_t timeout_ms = g_ctx.timeout_ms;

    /* Read until 3.5 character idle (silence) detected or timeout */
    while (1) {
        int64_t now_ms = (int64_t)(time(NULL) * 1000);
        if (now_ms - start_ms > timeout_ms)
            break;

        uint8_t tmp;
        ssize_t n = read(g_ctx.uart_fd, &tmp, 1);
        if (n > 0) {
            if (pos < cap) buf[pos++] = tmp;
            start_ms = (int64_t)(time(NULL) * 1000); /* reset idle timer */
        } else {
            /* No data; check if we have any data and idle time exceeded */
            if (pos > 0 && (now_ms - start_ms) > 2) /* ~2ms = 3.5 chars @ 115200 */
                break;
            usleep(500);
        }
    }

    *out_len = pos;
    return (pos > 0) ? 0 : -1;
}

/* ---------------------------------------------------------------------------
 * Execute modbus transaction
 * --------------------------------------------------------------------------- */
static int modbus_transaction(modbus_adu_t *req, size_t req_size,
                               modbus_adu_t *resp, size_t *resp_size) {
    uint8_t tx_buf[256];
    uint8_t rx_buf[256];

    /* Build raw frame */
    size_t tx_len = req_size;
    memcpy(tx_buf, req, tx_len);

    /* Compute and append CRC */
    uint16_t crc = modbus_crc16(tx_buf, tx_len);
    tx_buf[tx_len++] = crc & 0xFF;
    tx_buf[tx_len++] = (crc >> 8) & 0xFF;

    pthread_mutex_lock(&g_ctx.lock);

    for (int retry = 0; retry < MODBUS_RETRY_COUNT; retry++) {
        /* Send */
        if (modbus_send(tx_buf, tx_len) < 0) {
            fprintf(stderr, "[Modbus] Send failed\n");
            continue;
        }

        /* Receive */
        size_t rx_len = 0;
        if (modbus_receive(rx_buf, sizeof(rx_buf), &rx_len) < 0 || rx_len < MODBUS_ADU_MIN_SIZE) {
            fprintf(stderr, "[Modbus] No response (attempt %d)\n", retry + 1);
            continue;
        }

        /* Verify CRC */
        if (rx_len < 2) continue;
        uint16_t rx_crc = (uint16_t)rx_buf[rx_len - 2] | ((uint16_t)rx_buf[rx_len - 1] << 8);
        uint16_t calc_crc = modbus_crc16(rx_buf, rx_len - 2);
        if (rx_crc != calc_crc) {
            fprintf(stderr, "[Modbus] CRC mismatch (attempt %d)\n", retry + 1);
            continue;
        }

        /* Copy response */
        size_t copy_len = rx_len - 2; /* strip CRC */
        if (copy_len > sizeof(modbus_adu_t)) copy_len = sizeof(modbus_adu_t);
        memcpy(resp, rx_buf, copy_len);
        *resp_size = copy_len;

        /* Check for exception */
        if (resp->func_code & 0x80) {
            fprintf(stderr, "[Modbus] Exception %02x from slave %02x\n",
                    resp->error.exception_code, resp->slave_addr);
            pthread_mutex_unlock(&g_ctx.lock);
            return -1;
        }

        /* Verify slave address matches */
        if (resp->slave_addr != req->slave_addr) {
            fprintf(stderr, "[Modbus] Slave mismatch: req=%02x resp=%02x\n",
                    req->slave_addr, resp->slave_addr);
            continue;
        }

        pthread_mutex_unlock(&g_ctx.lock);
        return 0; /* success */
    }

    pthread_mutex_unlock(&g_ctx.lock);
    return -1;
}

/* ---------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------- */
int modbus_rtu_init(const char *device, int baud) {
    memset(&g_ctx, 0, sizeof(g_ctx));

    strncpy(g_ctx.device, device ? device : MODBUS_DEFAULT_DEVICE, sizeof(g_ctx.device) - 1);
    g_ctx.baud = baud > 0 ? baud : MODBUS_DEFAULT_BAUD;
    g_ctx.timeout_ms = MODBUS_DEFAULT_TIMEOUT_MS;
    g_ctx.uart_fd = -1;

    pthread_mutex_init(&g_ctx.lock, NULL);

    g_ctx.uart_fd = uart_open(g_ctx.device, g_ctx.baud);
    if (g_ctx.uart_fd < 0) {
        fprintf(stderr, "[Modbus] Failed to open UART\n");
        return -1;
    }

    rs485_init();
    g_ctx.initialized = 1;

    fprintf(stdout, "[Modbus] Initialized on %s @ %d baud\n", g_ctx.device, g_ctx.baud);
    return 0;
}

void modbus_rtu_set_timeout(int timeout_ms) {
    g_ctx.timeout_ms = timeout_ms;
}

/* ---------------------------------------------------------------------------
 * Read Holding Registers (0x03)
 * --------------------------------------------------------------------------- */
int modbus_read_registers(uint8_t slave_addr, uint16_t start_addr,
                           uint16_t quantity, uint16_t *values) {
    if (quantity > MODBUS_READ_MAX_REGS) quantity = MODBUS_READ_MAX_REGS;

    modbus_adu_t req;
    memset(&req, 0, sizeof(req));
    req.slave_addr = slave_addr;
    req.func_code = 0x03;
    req.read_req.start_addr = start_addr;
    req.read_req.quantity = quantity;

    modbus_adu_t resp;
    size_t resp_size = 0;

    if (modbus_transaction(&req, 4, &resp, &resp_size) < 0) {
        fprintf(stderr, "[Modbus] Read registers failed: slave=%02x addr=0x%04x qty=%d\n",
                slave_addr, start_addr, quantity);
        return -1;
    }

    /* Verify response */
    if (resp.func_code != 0x03) {
        fprintf(stderr, "[Modbus] Unexpected function code: 0x%02x\n", resp.func_code);
        return -1;
    }

    uint8_t byte_count = resp.read_resp.byte_count;
    uint16_t reg_count = byte_count / 2;
    if (reg_count > quantity) reg_count = quantity;

    for (uint16_t i = 0; i < reg_count; i++)
        values[i] = __builtin_bswap16(resp.read_resp.values[i]); /* big-endian to host */

    return reg_count;
}

/* ---------------------------------------------------------------------------
 * Write Single Register (0x06)
 * --------------------------------------------------------------------------- */
int modbus_write_register(uint8_t slave_addr, uint16_t reg_addr, uint16_t value) {
    modbus_adu_t req;
    memset(&req, 0, sizeof(req));
    req.slave_addr = slave_addr;
    req.func_code = 0x06;
    req.write_single.reg_addr = __builtin_bswap16(reg_addr);
    req.write_single.reg_value = __builtin_bswap16(value);

    modbus_adu_t resp;
    size_t resp_size = 0;

    if (modbus_transaction(&req, 4, &resp, &resp_size) < 0)
        return -1;

    if (resp.func_code != 0x06)
        return -1;

    return 0;
}

/* ---------------------------------------------------------------------------
 * Write Multiple Registers (0x10)
 * --------------------------------------------------------------------------- */
int modbus_write_registers(uint8_t slave_addr, uint16_t start_addr,
                            uint16_t quantity, const uint16_t *values) {
    if (quantity > MODBUS_WRITE_MAX_REGS) quantity = MODBUS_WRITE_MAX_REGS;

    modbus_adu_t req;
    memset(&req, 0, sizeof(req));
    req.slave_addr = slave_addr;
    req.func_code = 0x10;
    req.write_multi_req.start_addr = __builtin_bswap16(start_addr);
    req.write_multi_req.quantity = __builtin_bswap16(quantity);
    req.write_multi_req.byte_count = (uint8_t)(quantity * 2);
    for (uint16_t i = 0; i < quantity; i++)
        req.write_multi_req.values[i] = __builtin_bswap16(values[i]);

    modbus_adu_t resp;
    size_t resp_size = 0;
    size_t req_size = 6 + (size_t)quantity * 2; /* addr + func + start + qty + byte_cnt + values */

    if (modbus_transaction(&req, req_size, &resp, &resp_size) < 0)
        return -1;

    if (resp.func_code != 0x10)
        return -1;

    return 0;
}

/* ---------------------------------------------------------------------------
 * Motor drive convenience functions
 * --------------------------------------------------------------------------- */
int modbus_set_speed(uint8_t slave_addr, int16_t rpm) {
    return modbus_write_register(slave_addr, REG_SPEED_SETPOINT, (uint16_t)rpm);
}

int modbus_set_current_limit(uint8_t slave_addr, uint16_t ma) {
    return modbus_write_register(slave_addr, REG_CURRENT_LIMIT, ma);
}

int modbus_get_speed(uint8_t slave_addr, int16_t *rpm) {
    uint16_t val;
    int ret = modbus_read_registers(slave_addr, REG_ACTUAL_SPEED, 1, &val);
    if (ret > 0) {
        *rpm = (int16_t)val;
        return 0;
    }
    return -1;
}

int modbus_get_current(uint8_t slave_addr, uint16_t *ma) {
    return modbus_read_registers(slave_addr, REG_ACTUAL_CURRENT, 1, ma);
}

int modbus_get_fault(uint8_t slave_addr, uint16_t *fault_code) {
    return modbus_read_registers(slave_addr, REG_FAULT_CODE, 1, fault_code);
}
