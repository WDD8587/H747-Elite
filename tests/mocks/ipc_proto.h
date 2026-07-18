#ifndef IPC_PROTO_MOCK_H
#define IPC_PROTO_MOCK_H
#include <stdint.h>

#define SRAM3_BASE        0x20030000
#define HSEM_M7_HEARTBEAT 0
#define HSEM_ODOM_READY   1
#define HSEM_BMS_READY    2
#define IPC_HEAD_M7       0xAA
#define IPC_HEAD_RK       0x55

#pragma pack(1)
typedef struct {
    uint32_t magic; int32_t enc_l, enc_r;
    float imu_yaw, imu_gyro_z;
    uint16_t bms_rsoc, bms_mv;
    uint16_t tof_zone[4];
    uint8_t fault, cliff, _pad[2];
} l1_odom_t;

typedef struct { uint8_t head; uint8_t seq; l1_odom_t odom; uint16_t crc; } l1_report_t;
typedef struct { uint8_t head; float v,w; uint8_t flags; uint16_t crc; } l2_cmd_t;
#pragma pack()

uint16_t crc16_ibm(const uint8_t *d, uint16_t n);
#endif
