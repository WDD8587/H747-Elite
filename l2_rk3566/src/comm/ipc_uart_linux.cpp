#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>
#include "ipc_proto.h"

static int gFd = -1;

int IpcUart_Open(const char *dev, int baud)
{
    gFd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (gFd < 0) { perror("IPC UART open"); return -1; }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    cfsetispeed(&tty, B3000000);
    cfsetospeed(&tty, B3000000);
    tty.c_cflag = CS8 | CREAD | CLOCAL;
    tty.c_iflag = IGNPAR;
    tty.c_oflag = 0;
    tty.c_lflag = 0;
    tty.c_cc[VTIME] = 0;
    tty.c_cc[VMIN]  = 1;
    tcflush(gFd, TCIFLUSH);
    tcsetattr(gFd, TCSANOW, &tty);

    printf("[IPC UART] %s @ %d baud\n", dev, baud);
    return 0;
}

int IpcUart_ReadFrame(l1_report_t *rpt, int timeout_ms)
{
    if (gFd < 0) return -1;
    fd_set fds; FD_ZERO(&fds); FD_SET(gFd, &fds);
    struct timeval tv = {timeout_ms / 1000, (timeout_ms % 1000) * 1000};
    if (select(gFd + 1, &fds, NULL, NULL, &tv) <= 0) return 0;

    uint8_t buf[sizeof(l1_report_t)];
    int n = (int)read(gFd, buf, sizeof(buf));
    if (n < (int)sizeof(l1_report_t)) return 0;
    if (buf[0] != IPC_HEAD_M7) return 0;
    memcpy(rpt, buf, sizeof(l1_report_t));
    return 1;
}

void IpcUart_SendCmd(const l2_cmd_t *cmd)
{
    if (gFd < 0) return;
    write(gFd, cmd, sizeof(l2_cmd_t));
}
