# STM32H747 + RK3566 Triple-Core Sweeping Robot Architecture

## Core Assignment (元宝方案)

```
Chip: STM32H747XI (single die, dual core)
+ RK3566 (Quad A55 @ 1.8GHz, Linux)

H747 M7 @ 480MHz           H747 M4 @ 240MHz           RK3566 @ 1.8GHz
=======================    =======================    ======================
FOC 20kHz ISR (TIM1 UP)    Safety Watchdog (M7監)     Cartographer SLAM
CORDIC Clarke/Park/Inv      BMS SMBus 4S (平移ITE)     DWA Local Planner
IMU 1kHz Madgwick AHRS      BMS Charge (0.5C/1C/2C)    A* Global Planner
IPC Bridge (HSEM+UART)      ToF VL53L5CX 4x4           YOLOv8 RKNN
M4 唤醒 + 心跳               Dock Handshake (IR+19V)    MQTT Cloud
                             Stop2 uA Standby            OTA Manager
```

## Why M7 for FOC (not M4)?

ST文档确认: CORDIC硬件仅M7有，M4无。FOC的sin/cos/atan2必须放M7。
M7@480 CORDIC: clarke+park+invPark+svpwm = 0.45us。
M4@240 软件: 同样操作 = 3-5us，20kHz ISR(50us)中占10%，不可接受。

## M4 Safety: The Killer Interview Story

M4每10ms(100Hz)扫HSEM(0): M7心跳释放。
若M7心跳超50ms(6次miss)→ M4判M7挂→
  1. PWM_SHUTDOWN GPIO拉高 (<200ns)
  2. TIM1 BKIN硬件关PWM (<1时钟)
  3. NVIC关闭TIM1_UP IRQ
  4. 进入fail-safe: FS_SPIN_HOME→FS_DOCK_SEARCH→FS_STOP
  5. 通过红外beacon+19V ADC找基站自救

**单芯片实现"量产L1双MCU故障隔离"**

## ITE EC Experience Translation

| ITE EC (你的经验) | H747 M4 (扫地机) |
|-------------------|-----------------|
| SLP_Sx/RSMRST# | Stop2/RTC/LPTIM wake |
| KSCAN keyboard matrix | IR+TOUCH EXTI wake |
| SMBus Smart Battery SB1.1 | BMS SMBus 4S (BQ40Z50) |
| EC_ON power sequencing | HSEM(0) M7 wake |
| D-Notify thermal hysteresis | Charger stall derate 0.7x |
| Prochot/SD fault tolerance | M4 fail-safe state machine |

## Three-Core IPC Latency Chain

```
FOC ISR (M7, 20kHz): 1.0us
→ HSEM Release(1) 通知M4: <1us (总线锁)
→ M4 10ms task扫到(均5ms) → 读SRAM3 odom → 补BMS快照
→ M7 IPC_bridge (100Hz): 拼l1_report_t(45B) → UART7 DMA
→ UART线速: 45B × 10bits/baud / 3Mbps = 150us
→ RK3566收帧 + DWA算(均50ms) → 发l2_cmd_t(12B)回M7
→ M7收帧 → decimate到FOC ISR → 更新Iq_target

端到端(FOC→轮子动): ~60-70ms
```

## Hardware BOM

| Component | Model | Qty | Price |
|-----------|-------|:---:|-------|
| MCU | STM32H747ZIT6 | 1 | $8.50 |
| SOC | RK3566 | 1 | $12.00 |
| Motor Driver | DRV8313 | 2 | $1.60 |
| Current Sense | INA240A1 | 6 | $3.00 |
| IMU | ICM-20948 | 1 | $1.80 |
| ToF | VL53L5CX | 1 | $4.50 |
| Camera | IMX415 | 1 | $6.00 |
| BMS IC | BQ40Z50 | 1 | $2.50 |
| WiFi/BLE | ESP32-C3 | 1 | $0.60 |
| Flash | W25Q128 | 1 | $0.25 |
| DDR4 | 4GB LPDDR4 | 1 | $3.00 |
| eMMC | 32GB | 1 | $4.00 |
| PMIC | RK809 | 1 | $1.50 |
| **Total** | | | **$49.25** |

## 3 Critical Hardware Pitfalls

1. **H747选ZIT6 (LQFP144)**: VI档无HRTIM，多电机扩展废一半
2. **UART 3Mbps电平**: H747 Bank可配1.8V，RK3566 UART是1.8V。需TXB0104电平转换
3. **M7 D-Cache + HSEM**: odom放SRAM3(0x20030000,非Cache区)，避D-Cache一致性bug

## Resume Bullet (面试话术)

5年笔记本EC(ITE5508)测试与维护，深悉SMBus Smart Battery、SLP_Sx/EC_ON睡眠唤醒、KSCAN状态机；
深读Lenovo JLVI6项目274提交，复现D-Notify滞回、Prochot/SD容错、Battery Drop多层降档。
业余搭建扫地机三核验证平台：
L1-算力核(H747 M7@480)FOC 20kHz ISR + CORDIC加速clarke/park(1.2us→0.15us) + IMU 1kHz预处理；
L1-安全核(H747 M4@240)ToF VL53L5CX + BMS SMBus 4S锂电(平移ITE SB1.1) + 基站uA待机Stop2 + dock 19V握手 + M4 WWDG监M7心跳，超时fail-safe停PWM；
L2(RK3566 A55×4@1.8G, Linux pthread SCHED_FIFO)Cartographer + DWA/A* + YOLOv8 RKNN + MQTT；
三核IPC: M7-M4 HSEM共享SRAM(SRAM3避D-Cache) + M7-RK3566 UART 3Mbps CRC16，
fail-safe含L2失联60ms停轮/M7挂M4保命。
目标转扫地机嵌入式软件(底盘/电源/BMS)。
