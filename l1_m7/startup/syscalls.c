/**
 * @file    syscalls.c
 * @brief   Retarget printf/fprintf to UART7 for semihosting-free debug output.
 *
 * Implements the low-level system call stubs needed by newlib:
 *   _write()  - send buffer to UART7
 *   _read()   - read from UART7
 *   _isatty() - return 1 (all I/O is a TTY)
 *   _fstat()  - provide file stat
 *   _lseek()  - return 0 (no seeking on serial)
 *   _close()  - stub
 *   _link()   - stub
 *   _unlink() - stub
 *   _execve() - stub
 *   _fork()   - stub
 *   _getpid() - stub
 *   _kill()   - stub
 *   _times()  - stub
 *
 * All output is buffered line-by-line (newline-triggered flush).
 */

#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <stm32h7xx_hal.h>

/* ---------------------------------------------------------------------------
 * UART7 handle — must be initialised by application before first printf.
 * -------------------------------------------------------------------------*/
extern UART_HandleTypeDef huart7;

/* Timeout for UART TX (ms) */
#define UART_TX_TIMEOUT_MS   1000U
#define UART_RX_TIMEOUT_MS   5000U

/* Internal TX buffer for line-buffering */
#define TX_BUF_SIZE          128U
static char  tx_buf[TX_BUF_SIZE];
static uint32_t tx_buf_len = 0;

/* ---------------------------------------------------------------------------
 * flush_tx_buffer() — transmit buffered data via UART7
 * -------------------------------------------------------------------------*/
static void flush_tx_buffer(void)
{
    if (tx_buf_len == 0) {
        return;
    }

    /*
     * Use HAL_UART_Transmit with blocking mode.
     * In production, a DMA or interrupt-based variant is preferred.
     */
    HAL_StatusTypeDef ret;
    ret = HAL_UART_Transmit(&huart7, (uint8_t *)tx_buf, tx_buf_len,
                             UART_TX_TIMEOUT_MS);
    if (ret != HAL_OK) {
        /* UART error — silently discard to avoid infinite loop */
    }

    tx_buf_len = 0;
}

/* ---------------------------------------------------------------------------
 * _write() — write buffer to UART7 (line-buffered)
 *
 * File descriptor 0 = stdin, 1 = stdout, 2 = stderr all go to UART7.
 * Flushes on newline or when buffer is full.
 * -------------------------------------------------------------------------*/
int _write(int fd, const char *buf, int count)
{
    (void)fd;  /* All output FDs go to UART7 */

    if (buf == NULL || count <= 0) {
        return 0;
    }

    for (int i = 0; i < count; i++)
    {
        char c = buf[i];

        tx_buf[tx_buf_len++] = c;

        /* Flush on newline or full buffer */
        if (c == '\n' || tx_buf_len >= TX_BUF_SIZE) {
            flush_tx_buffer();
        }
    }

    return count;
}

/* ---------------------------------------------------------------------------
 * _read() — read from UART7 into buffer
 *
 * Reads up to 'count' bytes. Blocks until at least one byte is available.
 * Returns number of bytes actually read.
 * -------------------------------------------------------------------------*/
int _read(int fd, char *buf, int count)
{
    (void)fd;

    if (buf == NULL || count <= 0) {
        return 0;
    }

    /*
     * Use HAL_UART_Receive to get one byte at a time.
     * For efficiency in production, this should use DMA or interrupt-driven
     * reception with a ring buffer.
     */
    int bytes_read = 0;

    /* Read at least 1 byte */
    HAL_StatusTypeDef ret;
    ret = HAL_UART_Receive(&huart7, (uint8_t *)buf, (uint16_t)count,
                            UART_RX_TIMEOUT_MS);
    if (ret == HAL_OK) {
        bytes_read = count;
    } else if (ret == HAL_TIMEOUT) {
        /* Timeout is not an error for _read */
        bytes_read = 0;
    } else {
        /* Error */
        errno = EIO;
        return -1;
    }

    return bytes_read;
}

/* ---------------------------------------------------------------------------
 * _isatty() — return 1 (serial is a character device = TTY)
 * -------------------------------------------------------------------------*/
int _isatty(int fd)
{
    (void)fd;
    return 1;
}

/* ---------------------------------------------------------------------------
 * _fstat() — provide file status for stdout
 *
 * Needed by newlib to determine buffering mode. We claim the file is a
 * character device.
 * -------------------------------------------------------------------------*/
int _fstat(int fd, struct stat *st)
{
    (void)fd;

    if (st == NULL) {
        errno = EFAULT;
        return -1;
    }

    memset(st, 0, sizeof(*st));
    st->st_mode = S_IFCHR;  /* Character device */
    return 0;
}

/* ---------------------------------------------------------------------------
 * _lseek() — no seeking on a serial port
 * -------------------------------------------------------------------------*/
off_t _lseek(int fd, off_t offset, int whence)
{
    (void)fd;
    (void)offset;
    (void)whence;
    return 0;
}

/* ---------------------------------------------------------------------------
 * _close() — stub
 * -------------------------------------------------------------------------*/
int _close(int fd)
{
    (void)fd;
    return -1;
}

/* ---------------------------------------------------------------------------
 * Other newlib stubs
 * -------------------------------------------------------------------------*/
int _link(const char *old, const char *new)
{
    (void)old;
    (void)new;
    errno = EMLINK;
    return -1;
}

int _unlink(const char *name)
{
    (void)name;
    errno = ENOENT;
    return -1;
}

int _execve(const char *name, char *const argv[], char *const env[])
{
    (void)name;
    (void)argv;
    (void)env;
    errno = ENOMEM;
    return -1;
}

int _fork(void)
{
    errno = EAGAIN;
    return -1;
}

int _getpid(void)
{
    return 1;
}

int _kill(int pid, int sig)
{
    (void)pid;
    (void)sig;
    errno = EINVAL;
    return -1;
}

clock_t _times(struct tms *buf)
{
    (void)buf;
    return -1;
}

/* Heap-related: _sbrk for malloc */
extern uint32_t _sram3_heap_start;
extern uint32_t _sram3_heap_end;

static char *heap_ptr = NULL;

void *_sbrk(int incr)
{
    if (heap_ptr == NULL) {
        heap_ptr = (char *)&_sram3_heap_start;
    }

    char *prev = heap_ptr;
    char *next = heap_ptr + incr;

    if (next > (char *)&_sram3_heap_end) {
        errno = ENOMEM;
        return (void *)-1;
    }

    heap_ptr = next;
    return (void *)prev;
}

/* _exit — should not return */
void _exit(int status)
{
    (void)status;
    while (1) { __NOP(); }
}
