/**
 * lte_backup.c — LTE Cat-M1 backup connectivity for outdoor robots
 *
 * Quectel BG96 module via UART. AT command interface.
 * Fallback when WiFi RSSI < -80 dBm and no roam target.
 * Low-bandwidth mode: only safety + status telemetry (no map/OTA uploads).
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
#include <time.h>
#include <sys/stat.h>

#include "lte_backup.h"
#include "wifi_roam.h"   /* for checking WiFi status */

/* ---------------------------------------------------------------------------
 * Constants
 * --------------------------------------------------------------------------- */
#define LTE_UART_DEVICE           "/dev/ttyS2"
#define LTE_UART_BAUD             115200
#define LTE_CMD_TIMEOUT_MS        10000
#define LTE_RESP_BUF_SIZE         1024
#define LTE_MAX_RETRY             3

/* BG96 AT commands */
#define AT_CMD_AT                 "AT\r\n"
#define AT_CMD_CFUN               "AT+CFUN=1\r\n"
#define AT_CMD_CREG               "AT+CREG?\r\n"
#define AT_CMD_CGATT              "AT+CGATT?\r\n"
#define AT_CMD_CGDCONT            "AT+CGDCONT=1,\"IP\",\"%s\"\r\n"
#define AT_CMD_QIACT              "AT+QIACT=1\r\n"
#define AT_CMD_QISTATE            "AT+QISTATE=0,1\r\n"
#define AT_CMD_QIOPEN_TCP         "AT+QIOPEN=1,0,\"TCP\",\"%s\",%d,0,0\r\n"
#define AT_CMD_QISEND             "AT+QISEND=0,%d\r\n"
#define AT_CMD_QIRD               "AT+QIRD=0,%d\r\n"
#define AT_CMD_QIDEACT            "AT+QIDEACT=1\r\n"
#define AT_CMD_QPOWD              "AT+QPOWD=1\r\n"

/* APN configuration */
#define LTE_DEFAULT_APN           "iot.nb"

/* Fallback thresholds */
#define LTE_RSSI_THRESHOLD        -80    /* dBm — below this, try LTE */
#define LTE_CHECK_INTERVAL        10     /* seconds between WiFi checks */

/* ---------------------------------------------------------------------------
 * Context
 * --------------------------------------------------------------------------- */
typedef enum {
    LTE_STATE_OFF,
    LTE_STATE_INIT,
    LTE_STATE_SEARCHING,
    LTE_STATE_ATTACHING,
    LTE_STATE_ACTIVATING,
    LTE_STATE_CONNECTED,
    LTE_STATE_ERROR
} lte_state_t;

typedef struct {
    int              uart_fd;
    lte_state_t      state;
    int              keep_running;
    int              active;         /* LTE is the active connection */
    int              fallback_mode;  /* 1 = in fallback (low bandwidth) */
    char             apn[64];
    char             imei[32];
    char             iccid[32];
    int              signal_rssi;    /* dBm */
    int              reg_status;     /* 0-5 CREG status */

    pthread_t        monitor_thread;
    pthread_mutex_t  lock;

    uint64_t         connected_since;
    uint64_t         bytes_sent;
    uint64_t         bytes_received;

    /* TCP connection state */
    int              tcp_connected;
    int              tcp_connect_id;
    char             server_host[128];
    int              server_port;
} lte_ctx_t;

static lte_ctx_t g_ctx;

/* ---------------------------------------------------------------------------
 * Forward declarations
 * --------------------------------------------------------------------------- */
static int  uart_open(const char *device, int baud);
static void uart_close(void);
static int  uart_send(const char *cmd);
static int  uart_read_response(char *buf, size_t cap, size_t *out_len, int timeout_ms);
static int  send_at(const char *cmd, const char *expected, int timeout_ms);
static int  parse_creg(const char *resp, int *stat);
static int  parse_csq(const char *resp, int *rssi);
static int  lte_init_module(void);
static int  lte_attach_network(void);
static int  lte_activate_pdp(void);
static int  lte_connect_tcp(const char *host, int port);
static int  lte_disconnect_tcp(void);
static void lte_monitor_loop(void);

/* ---------------------------------------------------------------------------
 * UART helpers
 * --------------------------------------------------------------------------- */
static int uart_open(const char *device, int baud) {
    int fd = open(device, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        fprintf(stderr, "[LTE] Cannot open %s: %s\n", device, strerror(errno));
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) { close(fd); return -1; }

    cfsetospeed(&tty, (speed_t)baud);
    cfsetispeed(&tty, (speed_t)baud);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE; tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB; tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 2;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) { close(fd); return -1; }
    tcflush(fd, TCIOFLUSH);
    return fd;
}

static void uart_close(void) {
    if (g_ctx.uart_fd >= 0) {
        close(g_ctx.uart_fd);
        g_ctx.uart_fd = -1;
    }
}

static int uart_send(const char *cmd) {
    if (g_ctx.uart_fd < 0) return -1;
    size_t len = strlen(cmd);
    size_t written = 0;
    while (written < len) {
        ssize_t n = write(g_ctx.uart_fd, cmd + written, len - written);
        if (n < 0) return -1;
        written += (size_t)n;
    }
    return 0;
}

static int uart_read_response(char *buf, size_t cap, size_t *out_len, int timeout_ms) {
    size_t pos = 0;
    int64_t start_ms = (int64_t)(time(NULL) * 1000);

    while (1) {
        int64_t now_ms = (int64_t)(time(NULL) * 1000);
        if (now_ms - start_ms > timeout_ms) break;

        uint8_t byte;
        ssize_t n = read(g_ctx.uart_fd, &byte, 1);
        if (n > 0) {
            if (pos < cap - 1) {
                buf[pos++] = (char)byte;
                buf[pos] = '\0';
            }
            /* Check for OK, ERROR, or CONNECT */
            if (strstr(buf, "OK\r\n") || strstr(buf, "ERROR") ||
                strstr(buf, "CONNECT") || strstr(buf, "+QIURC:")) {
                break;
            }
        } else {
            usleep(10000); /* 10ms */
        }
    }

    *out_len = pos;
    return (pos > 0) ? 0 : -1;
}

static int send_at(const char *cmd, const char *expected, int timeout_ms) {
    char resp[LTE_RESP_BUF_SIZE];
    size_t rlen = 0;

    if (uart_send(cmd) < 0) return -1;
    if (uart_read_response(resp, sizeof(resp), &rlen, timeout_ms) < 0) return -1;

    if (expected && !strstr(resp, expected))
        return -1;

    return 0;
}

/* ---------------------------------------------------------------------------
 * CREG parsing
 * --------------------------------------------------------------------------- */
static int parse_creg(const char *resp, int *stat) {
    const char *p = strstr(resp, "+CREG:");
    if (!p) return -1;
    int n, s;
    if (sscanf(p, "+CREG: %d,%d", &n, &s) >= 2) {
        *stat = s;
        return 0;
    }
    return -1;
}

/* ---------------------------------------------------------------------------
 * CSQ to RSSI
 * --------------------------------------------------------------------------- */
static int parse_csq(const char *resp, int *rssi) {
    const char *p = strstr(resp, "+CSQ:");
    if (!p) return -1;
    int csq;
    if (sscanf(p, "+CSQ: %d,", &csq) >= 1) {
        /* CSQ 0 = -113 dBm, CSQ 31 = -53 dBm, step 2 dB */
        if (csq == 99) *rssi = -120;
        else *rssi = -113 + csq * 2;
        return 0;
    }
    return -1;
}

/* ---------------------------------------------------------------------------
 * Module initialization
 * --------------------------------------------------------------------------- */
static int lte_init_module(void) {
    /* AT sync */
    for (int i = 0; i < 3; i++) {
        if (send_at(AT_CMD_AT, "OK", LTE_CMD_TIMEOUT_MS) == 0) break;
        if (i == 2) return -1;
        sleep(1);
    }

    /* Get IMEI */
    if (send_at("AT+GSN\r\n", "OK", LTE_CMD_TIMEOUT_MS) == 0) {
        /* Extract from response — simplified */
    }

    /* Get ICCID */
    if (send_at("AT+ICCID\r\n", "OK", LTE_CMD_TIMEOUT_MS) == 0) {
        char resp[LTE_RESP_BUF_SIZE];
        size_t rlen;
        uart_read_response(resp, sizeof(resp), &rlen, 2000);
        const char *p = strstr(resp, "+ICCID:");
        if (p) sscanf(p, "+ICCID: %31s", g_ctx.iccid);
    }

    /* Set full functionality */
    if (send_at(AT_CMD_CFUN, "OK", 5000) < 0) {
        fprintf(stderr, "[LTE] CFUN failed\n");
        return -1;
    }

    fprintf(stdout, "[LTE] Module initialized\n");
    return 0;
}

/* ---------------------------------------------------------------------------
 * Network attach
 * --------------------------------------------------------------------------- */
static int lte_attach_network(void) {
    /* Wait for network registration */
    for (int i = 0; i < 30; i++) {
        char resp[LTE_RESP_BUF_SIZE];
        size_t rlen;

        if (uart_send(AT_CMD_CREG) == 0 &&
            uart_read_response(resp, sizeof(resp), &rlen, 5000) == 0) {
            int stat;
            if (parse_creg(resp, &stat) == 0) {
                g_ctx.reg_status = stat;
                /* 1=home, 5=roaming */
                if (stat == 1 || stat == 5) {
                    fprintf(stdout, "[LTE] Registered (status=%d)\n", stat);
                    break;
                }
            }
        }
        sleep(1);
        if (i == 29) {
            fprintf(stderr, "[LTE] Network registration timeout\n");
            return -1;
        }
    }

    /* Check GPRS attach */
    for (int i = 0; i < 10; i++) {
        char resp[LTE_RESP_BUF_SIZE];
        size_t rlen;

        if (uart_send(AT_CMD_CGATT) == 0 &&
            uart_read_response(resp, sizeof(resp), &rlen, 5000) == 0) {
            if (strstr(resp, "+CGATT: 1")) break;
        }
        sleep(2);
        if (i == 9) {
            fprintf(stderr, "[LTE] GPRS attach failed\n");
            return -1;
        }
    }

    return 0;
}

/* ---------------------------------------------------------------------------
 * Activate PDP context
 * --------------------------------------------------------------------------- */
static int lte_activate_pdp(void) {
    char cmd[128];
    snprintf(cmd, sizeof(cmd), AT_CMD_CGDCONT, g_ctx.apn);

    if (send_at(cmd, "OK", LTE_CMD_TIMEOUT_MS) < 0) {
        fprintf(stderr, "[LTE] CGDCONT failed\n");
        return -1;
    }

    for (int i = 0; i < 15; i++) {
        if (send_at(AT_CMD_QIACT, "OK", 15000) == 0) {
            g_ctx.state = LTE_STATE_CONNECTED;
            g_ctx.connected_since = (uint64_t)time(NULL);
            fprintf(stdout, "[LTE] PDP activated\n");

            /* Query IP */
            char resp[LTE_RESP_BUF_SIZE];
            size_t rlen;
            uart_read_response(resp, sizeof(resp), &rlen, 3000);
            return 0;
        }
        sleep(2);
        if (i % 5 == 4) {
            /* Deactivate and retry */
            send_at(AT_CMD_QIDEACT, "OK", 5000);
            sleep(2);
        }
    }

    return -1;
}

/* ---------------------------------------------------------------------------
 * TCP connection management
 * --------------------------------------------------------------------------- */
static int lte_connect_tcp(const char *host, int port) {
    char cmd[256];
    snprintf(cmd, sizeof(cmd), AT_CMD_QIOPEN_TCP, host, port);

    if (send_at(cmd, "OK", 15000) < 0) {
        fprintf(stderr, "[LTE] TCP connect failed\n");
        return -1;
    }

    /* Wait for +QIOPEN: 0,0 (success) */
    char resp[LTE_RESP_BUF_SIZE];
    size_t rlen;
    int64_t start = (int64_t)(time(NULL) * 1000);

    while (1) {
        if ((int64_t)(time(NULL) * 1000) - start > 30000) break;
        if (uart_read_response(resp, sizeof(resp), &rlen, 1000) == 0) {
            if (strstr(resp, "+QIOPEN: 0,0")) {
                g_ctx.tcp_connected = 1;
                g_ctx.tcp_connect_id = 0;
                strncpy(g_ctx.server_host, host, sizeof(g_ctx.server_host) - 1);
                g_ctx.server_port = port;
                fprintf(stdout, "[LTE] TCP connected to %s:%d\n", host, port);
                return 0;
            }
            if (strstr(resp, "+QIOPEN: 0,")) {
                int err;
                if (sscanf(resp, "+QIOPEN: 0,%d", &err) == 1 && err != 0) {
                    fprintf(stderr, "[LTE] TCP connect error: %d\n", err);
                    return -1;
                }
            }
        }
    }

    return -1;
}

static int lte_disconnect_tcp(void) {
    if (!g_ctx.tcp_connected) return 0;
    if (send_at("AT+QICLOSE=0\r\n", "OK", 5000) == 0) {
        g_ctx.tcp_connected = 0;
        return 0;
    }
    return -1;
}

/* ---------------------------------------------------------------------------
 * Data send
 * --------------------------------------------------------------------------- */
int lte_send_data(const uint8_t *data, size_t len) {
    if (!g_ctx.active || !g_ctx.tcp_connected) return -1;

    if (g_ctx.fallback_mode && len > 1024) {
        fprintf(stdout, "[LTE] Fallback mode: truncating large payload\n");
        len = 1024; /* limit in fallback */
    }

    char cmd[32];
    snprintf(cmd, sizeof(cmd), AT_CMD_QISEND, (int)len);

    if (send_at(cmd, ">", 5000) < 0) return -1;

    /* Send raw data */
    size_t written = 0;
    while (written < len) {
        ssize_t n = write(g_ctx.uart_fd, data + written, len - written);
        if (n < 0) return -1;
        written += (size_t)n;
    }

    /* Wait for SEND OK */
    char resp[LTE_RESP_BUF_SIZE];
    size_t rlen;

    /* Also check for URC "DATA SEND: 0,<len>" */
    if (send_at("", "SEND OK", 10000) < 0) {
        uart_read_response(resp, sizeof(resp), &rlen, 2000);
        if (!strstr(resp, "SEND OK")) return -1;
    }

    g_ctx.bytes_sent += len;
    return (int)len;
}

/* ---------------------------------------------------------------------------
 * Data receive (non-blocking)
 * --------------------------------------------------------------------------- */
int lte_receive_data(uint8_t *buf, size_t cap, size_t *out_len) {
    if (!g_ctx.active || !g_ctx.tcp_connected) return -1;

    /* Check for +QIURC: "recv",0 */
    char resp[LTE_RESP_BUF_SIZE];
    size_t rlen;

    if (uart_read_response(resp, sizeof(resp), &rlen, 100) < 0)
        return 0; /* no data */

    if (strstr(resp, "+QIURC: \"recv\"")) {
        /* Determine length from URC */
        int avail = 1024; /* default */
        sscanf(resp, "+QIURC: \"recv\",%d", &avail);

        /* Read data */
        char rd_cmd[32];
        snprintf(rd_cmd, sizeof(rd_cmd), AT_CMD_QIRD, avail);

        if (send_at(rd_cmd, "OK", 5000) == 0) {
            char data_resp[LTE_RESP_BUF_SIZE];
            size_t drlen;
            uart_read_response(data_resp, sizeof(data_resp), &drlen, 2000);

            /* Extract hex data if present — BG96 returns data in hex format +QIRD: <len>,<data> */
            const char *data_start = strstr(data_resp, ",,");
            if (data_start) {
                data_start += 2; /* skip ",," */
                size_t data_len = strlen(data_start);
                if (data_len > cap) data_len = cap;
                memcpy(buf, data_start, data_len);
                *out_len = data_len;
                g_ctx.bytes_received += data_len;
                return (int)data_len;
            }
        }
    }

    return 0;
}

/* ---------------------------------------------------------------------------
 * Monitor loop — checks WiFi and manages LTE fallback
 * --------------------------------------------------------------------------- */
static void lte_monitor_loop(void) {
    while (g_ctx.keep_running) {
        sleep(LTE_CHECK_INTERVAL);

        int wifi_rssi = wifi_roam_get_rssi();
        int need_lte = (wifi_rssi < LTE_RSSI_THRESHOLD);

        if (need_lte && !g_ctx.active) {
            /* Activate LTE backup */
            fprintf(stdout, "[LTE] WiFi weak (RSSI=%d), activating LTE backup\n", wifi_rssi);

            pthread_mutex_lock(&g_ctx.lock);
            g_ctx.state = LTE_STATE_INIT;
            g_ctx.fallback_mode = 1;
            pthread_mutex_unlock(&g_ctx.lock);

            if (lte_init_module() == 0 && lte_attach_network() == 0 &&
                lte_activate_pdp() == 0) {
                pthread_mutex_lock(&g_ctx.lock);
                g_ctx.active = 1;
                g_ctx.state = LTE_STATE_CONNECTED;
                pthread_mutex_unlock(&g_ctx.lock);
                fprintf(stdout, "[LTE] Backup active (low-bandwidth mode)\n");
            } else {
                pthread_mutex_lock(&g_ctx.lock);
                g_ctx.state = LTE_STATE_ERROR;
                pthread_mutex_unlock(&g_ctx.lock);
                fprintf(stderr, "[LTE] Activation failed\n");
            }
        } else if (!need_lte && g_ctx.active) {
            /* WiFi recovered — deactivate LTE */
            fprintf(stdout, "[LTE] WiFi recovered (RSSI=%d), deactivating LTE\n", wifi_rssi);
            lte_disconnect_tcp();
            send_at(AT_CMD_QIDEACT, "OK", 5000);

            pthread_mutex_lock(&g_ctx.lock);
            g_ctx.active = 0;
            g_ctx.tcp_connected = 0;
            g_ctx.fallback_mode = 0;
            g_ctx.state = LTE_STATE_OFF;
            pthread_mutex_unlock(&g_ctx.lock);

            fprintf(stdout, "[LTE] Backup deactivated\n");
        }

        /* Check LTE signal quality if active */
        if (g_ctx.active) {
            char resp[LTE_RESP_BUF_SIZE];
            size_t rlen;
            if (uart_send("AT+CSQ\r\n") == 0 &&
                uart_read_response(resp, sizeof(resp), &rlen, 3000) == 0) {
                int rssi;
                if (parse_csq(resp, &rssi) == 0)
                    g_ctx.signal_rssi = rssi;
            }
        }
    }
}

/* ---------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------- */
int lte_backup_init(const char *apn) {
    memset(&g_ctx, 0, sizeof(g_ctx));
    g_ctx.uart_fd = -1;
    g_ctx.keep_running = 1;
    g_ctx.state = LTE_STATE_OFF;

    strncpy(g_ctx.apn, apn ? apn : LTE_DEFAULT_APN, sizeof(g_ctx.apn) - 1);
    pthread_mutex_init(&g_ctx.lock, NULL);

    /* Open UART (keep open for monitoring) */
    g_ctx.uart_fd = uart_open(LTE_UART_DEVICE, LTE_UART_BAUD);
    if (g_ctx.uart_fd < 0) {
        fprintf(stderr, "[LTE] UART open failed, LTE unavailable\n");
        /* Non-fatal: LTE can start later */
    }

    fprintf(stdout, "[LTE] Backup initialized (APN: %s)\n", g_ctx.apn);
    return 0;
}

int lte_backup_start(void) {
    g_ctx.keep_running = 1;
    pthread_create(&g_ctx.monitor_thread, NULL,
                   (void *(*)(void *))lte_monitor_loop, NULL);
    return 0;
}

int lte_backup_stop(void) {
    g_ctx.keep_running = 0;
    lte_disconnect_tcp();
    send_at(AT_CMD_QIDEACT, "OK", 5000);
    send_at(AT_CMD_QPOWD, "OK", 3000);
    uart_close();
    pthread_join(g_ctx.monitor_thread, NULL);
    return 0;
}

int lte_backup_is_active(void) {
    return g_ctx.active ? 1 : 0;
}

int lte_backup_in_fallback(void) {
    return g_ctx.fallback_mode ? 1 : 0;
}

int lte_backup_get_signal(void) {
    return g_ctx.signal_rssi;
}

const char *lte_backup_get_imei(void) {
    return g_ctx.imei;
}

const char *lte_backup_get_iccid(void) {
    return g_ctx.iccid;
}

uint64_t lte_backup_get_bytes_sent(void) {
    return g_ctx.bytes_sent;
}

uint64_t lte_backup_get_bytes_received(void) {
    return g_ctx.bytes_received;
}
