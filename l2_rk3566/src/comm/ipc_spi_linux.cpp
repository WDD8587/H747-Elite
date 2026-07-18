/**
 * @file    ipc_spi_linux.cpp
 * @brief   RK3566 SPI Master for STM32H747 IPC.
 *
 * Communicates with STM32H7 SPI6 Slave @ 20 MHz, full-duplex 64-byte frames.
 *
 * Protocol:
 *   1. Wait for READY GPIO (STM32 PE1) to go HIGH.
 *   2. Initiate 64-byte full-duplex SPI transfer:
 *        - MOSI: l2_cmd_t (12 bytes + padding)
 *        - MISO: l1_report_t (45 bytes + padding)
 *   3. Parse received l1_report_t (validate header 0xAA + CRC16).
 *   4. Loop.
 *
 * Usage:
 *   IpcSpi_Open("/dev/spidev32766.0", 85);  // spidev + READY GPIO number
 *   while (running) {
 *       l1_report_t rpt;
 *       if (IpcSpi_Exchange(&cmd, &rpt)) { ... }
 *   }
 *   IpcSpi_Close();
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include "ipc_proto.h"

static int    gSpiFd   = -1;
static int    gReadyFd = -1;
static char   gReadyPath[64];
static bool   gReadyActive = false;

/* ---- GPIO sysfs helpers ---- */

static int gpio_export(int gpio)
{
    int fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd < 0) return -1;
    char s[8]; int n = snprintf(s, sizeof(s), "%d", gpio);
    write(fd, s, n);
    close(fd);
    return 0;
}

static int gpio_set_direction(int gpio, const char *dir)
{
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", gpio);
    int fd = open(path, O_WRONLY);
    if (fd < 0) return -1;
    write(fd, dir, strlen(dir));
    close(fd);
    return 0;
}

static int gpio_set_edge(int gpio, const char *edge)
{
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/edge", gpio);
    int fd = open(path, O_WRONLY);
    if (fd < 0) return -1;
    write(fd, edge, strlen(edge));
    close(fd);
    return 0;
}

static int gpio_open_value(int gpio, int flags)
{
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio);
    return open(path, flags);
}

/* ---- CRC16 ---- */

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

static uint16_t crc16_ibm(const uint8_t *d, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    while (len--) crc = (crc >> 8) ^ crc16_tbl[(crc ^ *d++) & 0xFF];
    return crc;
}

/* ---- Public API ---- */

#define SPI_FRAME  64
#define SPI_SPEED  20000000  /* 20 MHz */

int IpcSpi_Open(const char *spidev, int ready_gpio)
{
    /* ---- SPI device ---- */
    gSpiFd = open(spidev, O_RDWR);
    if (gSpiFd < 0) { perror("IpcSpi open"); return -1; }

    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    uint32_t speed = SPI_SPEED;

    ioctl(gSpiFd, SPI_IOC_WR_MODE, &mode);
    ioctl(gSpiFd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(gSpiFd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

    printf("[IPC SPI] %s @ %u MHz mode %d\n",
           spidev, speed / 1000000, mode);

    /* ---- READY GPIO ---- */
    gpio_export(ready_gpio);
    usleep(100000);  /* wait for sysfs to create */
    gpio_set_direction(ready_gpio, "in");
    gpio_set_edge(ready_gpio, "rising");

    snprintf(gReadyPath, sizeof(gReadyPath),
             "/sys/class/gpio/gpio%d/value", ready_gpio);
    gReadyFd = gpio_open_value(ready_gpio, O_RDONLY);
    if (gReadyFd < 0) { perror("IpcSpi ready gpio"); IpcSpi_Close(); return -2; }

    gReadyActive = false;
    return 0;
}

void IpcSpi_Close(void)
{
    if (gSpiFd >= 0)   { close(gSpiFd);   gSpiFd   = -1; }
    if (gReadyFd >= 0) { close(gReadyFd); gReadyFd = -1; }
}

static bool IpcSpi_PollReady(void)
{
    char c;
    lseek(gReadyFd, 0, SEEK_SET);
    if (read(gReadyFd, &c, 1) != 1) return false;
    return c == '1';
}

/**
 * @brief  Full-duplex exchange: send l2_cmd_t, receive l1_report_t.
 *
 * @param  cmd   [in]  Velocity command to send to STM32.
 * @param  rpt   [out] Received odometry report (valid if return == 1).
 * @return 1 if valid l1_report_t received, 0 if no data, -1 on error.
 */
int IpcSpi_Exchange(const l2_cmd_t *cmd, l1_report_t *rpt)
{
    if (gSpiFd < 0) return -1;

    /* Wait for STM32 to signal data ready */
    if (!IpcSpi_PollReady()) return 0;

    /* Build TX buffer: l2_cmd_t + padding */
    uint8_t tx[SPI_FRAME] = {0};
    uint8_t rx[SPI_FRAME] = {0};
    if (cmd) memcpy(tx, cmd, sizeof(l2_cmd_t));

    struct spi_ioc_transfer tr = {};
    tr.tx_buf        = (unsigned long)tx;
    tr.rx_buf        = (unsigned long)rx;
    tr.len           = SPI_FRAME;
    tr.speed_hz      = SPI_SPEED;
    tr.bits_per_word = 8;

    if (ioctl(gSpiFd, SPI_IOC_MESSAGE(1), &tr) < 0) {
        perror("IpcSpi SPI_IOC_MESSAGE");
        return -1;
    }

    /* Parse received l1_report_t */
    if (rx[0] != IPC_HEAD_M7) return 0;

    uint16_t crc_calc = crc16_ibm(rx, sizeof(l1_report_t) - 2);
    uint16_t crc_rcv;
    memcpy(&crc_rcv, rx + sizeof(l1_report_t) - 2, 2);
    if (crc_calc != crc_rcv) return 0;

    memcpy(rpt, rx, sizeof(l1_report_t));
    return 1;
}

/**
 * @brief  Blocking read: wait with timeout for a valid l1_report_t.
 *
 * @param  rpt        [out] Odometry report.
 * @param  timeout_ms Timeout in milliseconds.
 * @return 1 if received, 0 on timeout, -1 on error.
 */
int IpcSpi_ReadFrame(l1_report_t *rpt, int timeout_ms)
{
    if (gSpiFd < 0 || gReadyFd < 0) return -1;

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(gReadyFd, &fds);

    struct timeval tv;
    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int ret = select(gReadyFd + 1, NULL, NULL, &fds, &tv);
    if (ret <= 0) return 0;  /* timeout or error */

    /* READY pin went high, do exchange with empty command */
    l2_cmd_t cmd = {};
    cmd.head = IPC_HEAD_RK;
    cmd.crc  = crc16_ibm((uint8_t *)&cmd, sizeof(cmd) - 2);
    return IpcSpi_Exchange(&cmd, rpt);
}

/**
 * @brief  Send a velocity command. Blocks until READY, then full-duplex exchange.
 *
 * @param  cmd  [in]  Velocity command (head, v, w, flags, crc must be filled).
 * @return bytes written on success, -1 on error.
 */
int IpcSpi_SendCmd(const l2_cmd_t *cmd)
{
    l1_report_t rpt;
    return IpcSpi_Exchange(cmd, &rpt);  /* discard report if not needed */
}
