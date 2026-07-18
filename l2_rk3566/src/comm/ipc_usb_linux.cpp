/**
 * @file    ipc_usb_linux.cpp
 * @brief   RK3566 USB CDC ACM Host client for STM32H747 IPC.
 *
 * STM32 appears as /dev/ttyACM0 (CDC ACM). Protocol is identical to UART IPC:
 *   - Read  l1_report_t (45 bytes) from /dev/ttyACM0
 *   - Write l2_cmd_t   (12 bytes) to   /dev/ttyACM0
 *
 * Unlike UART, USB CDC is packet-based — baud rate is virtual/ignored.
 * Non-blocking I/O with select() for frame reads.
 *
 * Build: g++ -std=c++17 ipc_usb_linux.cpp -o ipc_usb_test
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>
#include "ipc_proto.h"

static int gFd = -1;

int IpcUsb_Open(const char *dev)
{
    gFd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (gFd < 0) { perror("IPC USB open"); return -1; }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    /* CDC ACM ignores baud rate, but set it for compatibility */
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    tty.c_cflag = CS8 | CREAD | CLOCAL;
    tty.c_iflag = IGNPAR;
    tty.c_oflag = 0;
    tty.c_lflag = 0;
    tty.c_cc[VTIME] = 0;
    tty.c_cc[VMIN]  = 0;
    tcflush(gFd, TCIFLUSH);
    tcsetattr(gFd, TCSANOW, &tty);

    printf("[IPC USB] %s opened\n", dev);
    return 0;
}

void IpcUsb_Close(void)
{
    if (gFd >= 0) { close(gFd); gFd = -1; }
}

/**
 * @brief  Read a complete l1_report_t frame from USB CDC.
 *
 * Uses select() with timeout for non-blocking poll.
 * Validates header byte (0xAA) and CRC16.
 *
 * @param  rpt        [out] Parsed odometry report.
 * @param  timeout_ms Timeout in milliseconds (0 = non-blocking poll).
 * @return 1 if valid frame received, 0 if no data, -1 on error.
 */
int IpcUsb_ReadFrame(l1_report_t *rpt, int timeout_ms)
{
    if (gFd < 0) return -1;

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(gFd, &fds);

    struct timeval tv;
    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int ret = select(gFd + 1, &fds, NULL, NULL, &tv);
    if (ret <= 0) return 0;

    uint8_t buf[sizeof(l1_report_t)];
    int n = (int)read(gFd, buf, sizeof(buf));
    if (n < (int)sizeof(l1_report_t)) return 0;
    if (buf[0] != IPC_HEAD_M7) return 0;

    /* CRC16 validation (UART path skipped this; USB does it properly) */
    extern uint16_t crc16_ibm(const uint8_t *data, uint16_t len);
    uint16_t crc_calc = crc16_ibm(buf, sizeof(l1_report_t) - 2);
    uint16_t crc_rcv;
    memcpy(&crc_rcv, buf + sizeof(l1_report_t) - 2, 2);
    if (crc_calc != crc_rcv) return 0;

    memcpy(rpt, buf, sizeof(l1_report_t));
    return 1;
}

/**
 * @brief  Send a velocity command to STM32 over USB CDC.
 *
 * The caller must fill cmd->head = IPC_HEAD_RK and cmd->crc.
 *
 * @param  cmd  [in]  Velocity command to send.
 */
void IpcUsb_SendCmd(const l2_cmd_t *cmd)
{
    if (gFd < 0) return;
    write(gFd, cmd, sizeof(l2_cmd_t));
}
