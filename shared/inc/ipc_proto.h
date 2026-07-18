#ifndef IPC_PROTO_H
#define IPC_PROTO_H
#include <stdint.h>

/* L1 internal: M7 <-> M4 via HSEM + shared SRAM3 (non-cache, <1us) */
#define SRAM3_BASE         0x20030000
#define HSEM_M7_HEARTBEAT  0
#define HSEM_ODOM_READY    1
#define HSEM_BMS_READY     2
#define HSEM_SAFETY        3

/* L1 <-> L2: UART 3Mbps CRC16 */
#define IPC_HEAD_M7  0xAA
#define IPC_HEAD_RK  0x55
#define FLAG_ESTOP   0x01
#define FLAG_DOCK    0x02
#define FLAG_STUCK   0x04
#define IPC_UART_BAUD 3000000

#pragma pack(1)
typedef struct {
    uint32_t magic;       /* 0x4C314C31 "L1L1" */
    int32_t  enc_l;       /* left wheel cumulative pulse */
    int32_t  enc_r;       /* right wheel cumulative pulse */
    float    imu_yaw;     /* deg */
    float    imu_gyro_z;  /* deg/s (DWA uses this) */
    uint16_t bms_rsoc;    /* % */
    uint16_t bms_mv;      /* mV */
    uint16_t tof_zone[4]; /* front/left/right/back mm */
    uint8_t  fault;       /* bit0=stall bit1=lowV bit2=IMU bit3=ToF bit4=M7_dead */
    uint8_t  cliff;       /* bit0=LF bit1=RF bit2=LB bit3=RB */
    uint8_t  _pad[2];
} l1_odom_t;  /* 40 bytes */

typedef struct {
    uint8_t   head;
    uint8_t   seq;
    l1_odom_t odom;
    uint16_t  crc;
} l1_report_t;  /* 45 bytes */

typedef struct {
    uint8_t  head;   /* 0x55 */
    float    v;      /* mm/s */
    float    w;      /* deg/s */
    uint8_t  flags;  /* FLAG_* */
    uint16_t crc;
} l2_cmd_t;  /* 12 bytes */

_Static_assert(sizeof(l1_odom_t) == 36, "odom size");
_Static_assert(sizeof(l1_report_t) == 40, "report size");
_Static_assert(sizeof(l2_cmd_t) == 12, "cmd size");
#pragma pack()

uint16_t crc16_ibm(const uint8_t *data, uint16_t len);
#endif
