# H747 + RK3566 Sweeping Robot — Hardware System Architecture

## 1. Top-Level Block Diagram

```
                          ┌──────────────────────────────────────────────────────┐
                          │                   19V DC DOCK INPUT                  │
                          └──────────────────────┬───────────────────────────────┘
                                                 │
                    ┌────────────────────────────┼────────────────────────────┐
                    │                            │                            │
                    ▼                            ▼                            ▼
            ┌───────────────┐          ┌──────────────────┐          ┌────────────────┐
            │  MP2315       │          │  TPS62130        │          │  STUSB4500     │
            │  19V → 12V    │          │  5V → 3.3V       │          │  USB-PD CC     │
            │  (Motor Rail) │          │  (Logic Rail)    │          │  Negotiation   │
            └───────┬───────┘          └────────┬─────────┘          └────────────────┘
                    │                           │
         ┌──────────┼──────────┐       ┌────────┼────────┬──────────┐
         ▼          ▼          ▼       ▼        ▼        ▼          ▼
    ┌─────────┐┌─────────┐┌─────────┐ ┌────┐┌──────┐┌─────┐┌──────────┐
    │DRV8313 L││DRV8313 R││ FAN     │ │3.3V││1.8V  ││1.2V ││SENSOR BUS│
    │3-Phase  ││3-Phase  ││BLDC     │ │Rail││Rail  ││Rail ││Power     │
    └────┬────┘└────┬────┘└────┬────┘ └──┬─┘└──┬───┘└──┬──┘└────┬─────┘
         │          │          │          │      │       │        │
    ┌────▼────┐┌───▼─────┐┌───▼──────┐   │      │       │        │
    │2204     ││2204     ││5010      │   │      │       │        │
    │BLDC L   ││BLDC R   ││Brushless │   │      │       │        │
    │190KV    ││190KV    ││Fan       │   │      │       │        │
    │+Encoder ││+Encoder ││          │   │      │       │        │
    └─────────┘└─────────┘└──────────┘   │      │       │        │
                                         ▼      ▼       ▼        ▼
                              ┌────────────────────────────────────────────┐
                              │            STM32H747ZIT6 (LQFP-144)        │
                              │                                            │
                              │  ┌───────────┐    ┌──────────────────────┐ │
                              │  │ Cortex-M7 │    │     Cortex-M4        │ │
                              │  │ 480MHz    │◄──►│     240MHz           │ │
                              │  │ DP-FPU    │    │     SP-FPU           │ │
                              │  │ 16KB I/D$ │    │     No Cache         │ │
                              │  │ MPU       │    │     MPU              │ │
                              │  │ CORDIC    │    │                      │ │
                              │  │ FMAC      │    │                      │ │
                              │  └─────┬─────┘    └──────────┬───────────┘ │
                              │        │ HSEM + SRAM3        │             │
                              │        └────────┬────────────┘             │
                              │                 │                          │
                              │    ┌────────────┴────────────┐             │
                              │    │    PERIPHERAL BUS       │             │
                              │    │  AHB/APB Matrix @240MHz │             │
                              │    └────────────┬────────────┘             │
                              └─────────────────┼──────────────────────────┘
                                                │
        ┌───────────────────┬───────────────────┼───────────────────┬──────────────────┐
        │                   │                   │                   │                  │
   ┌────▼────┐        ┌─────▼─────┐       ┌─────▼─────┐       ┌────▼────┐      ┌─────▼─────┐
   │ICM-20948│        │ VL53L5CX  │       │ DRV8313   │       │INA240   │      │ SN65HVD230│
   │9-Axis   │        │ ToF 8×8   │       │ SPI Config│       │Current  │      │ CAN-FD    │
   │SPI1     │        │ I2C1 0x52 │       │ SPI1 CS2  │       │ADC1/2   │      │ FDCAN1    │
   └─────────┘        └───────────┘       └───────────┘       └─────────┘      └───────────┘
        │                   │                   │                   │                  │
   ┌────▼────┐        ┌─────▼─────┐                                  │             ┌────▼────┐
   │ ESP32-C3│        │ BQ40Z50   │                                  │             │ CAN Bus │
   │ UART4   │        │ SMBus 0x16│                                  │             │ 500kbps │
   │ AT Cmd  │        │ I2C1 0x16 │                                  │             │ Motors  │
   └────┬────┘        └───────────┘                                  │             │ + BMS   │
        │                                                            │             └─────────┘
   ┌────▼────────┐                                          ┌────────▼────────┐
   │ WiFi 802.11ac│                                         │ W25Q128 16MB    │
   │ BLE 5.0     │                                          │ QSPI Flash      │
   │ MQTT Cloud  │                                          │ OTA + Map + Log │
   └─────────────┘                                          └─────────────────┘
```

## 2. Bus Topology

```
H747 Bus Matrix:

SPI1 (40MHz)  ──── ICM-20948 (IMU 9-axis)
           ├──── DRV8313_L (SPI config registers)
           └──── DRV8313_R (SPI config registers)

QSPI (80MHz)  ──── W25Q128 (16MB NOR Flash)
                  ├─ Partition: Bootloader(64KB) + App_A(512KB) + App_B(512KB)
                  ├─             + Config(64KB) + Log(1MB) + Map(13MB)

I2C1 (400kHz) ──── VL53L5CX (ToF, 0x52)
           ├──── BQ40Z50 (BMS Gas Gauge, 0x16)
           └──── SSD1306 (OLED, 0x3C)

FDCAN1 (500kbps) ──── SN65HVD230 ──── CAN Bus
                     ├─ DRV8313_L (motor telemetry)
                     ├─ DRV8313_R (motor telemetry)
                     └─ BQ40Z50 (battery telemetry, secondary path)

UART4 (921600) ──── ESP32-C3 (WiFi/BLE Co-Processor)

UART7 (3Mbps)  ──── RK3566 (Linux Algorithm Core) via on-board pin header

ADC1 (5MSPS)   ──── INA240_L ×3 (3-phase current, TIM1_TRGO trigger)
ADC2 (5MSPS)   ──── INA240_R ×3 (3-phase current, TIM8_TRGO trigger)
ADC3 (5MSPS)   ──── VBUS_SENSE + BATT_SENSE + DOCK_19V

TIM1 (20kHz)   ──── DRV8313_L (3-phase PWM + complementary, CH1/CH2/CH3)
TIM8 (20kHz)   ──── DRV8313_R (3-phase PWM + complementary, CH1/CH2/CH3)
TIM2 (10kHz)   ──── Wheel Encoder L (quadrature ABZ)
TIM3 (10kHz)   ──── Wheel Encoder R (quadrature ABZ)
TIM4 (20kHz)   ──── Fan BLDC (3-phase PWM)
TIM5 (10kHz)   ──── Fan Encoder (quadrature)

HSEM (32ch)    ──── M7↔M4 IPC (Hardware Semaphore)
SRAM3 (64KB)   ──── M7↔M4 Shared Memory (0x20030000, non-cache)
```

## 3. Power Tree

```
 DOCK 19V DC (5.5×2.1mm Barrel Jack)
  │
  ├─[MP2315 Buck]──→ +12V @ 10A ──→ DRV8313 L/R (Motor PWM)
  │                              ──→ Fan Motor
  │                              ──→ 12V Rail Test Point
  │
  ├─[TPS62130 Buck]─→ +5V @ 3A  ──→ ESP32-C3
  │                              ──→ VL53L5CX (AVDD)
  │                              ──→ SSD1306 OLED
  │                              ──→ USB Host VBUS
  │                              ──→ 5V Rail Test Point
  │
  ß─[AMS1117-3.3 LDO]─→ +3.3V @ 1A ──→ STM32H747 (VDD, VDDA, VBAT)
                                      ──→ ICM-20948
                                      ──→ W25Q128
                                      ──→ SN65HVD230
                                      ──→ INA240 ×6
                                      ──→ VL53L5CX (IOVDD)
                                      ──→ BQ40Z50 SMBus Pull-ups
                                      ──→ 3.3V Rail Test Point

 BATTERY 4S LiPo (14.8V nom, 12.0-16.8V range)
  │
  ├─[BQ40Z50 BMS]──→ Protection FETs ──→ PACK+
  │                 ├─ Charge FET (AO4407A, P-ch)
  │                 └─ Discharge FET (AO4407A, P-ch)
  │
  └─[MP2315 Battery Path]──→ Same +12V rail when undocked
```

## 4. Clock Tree

```
 HSE 25MHz (External Crystal, 3225 package, 18pF load)
  │
  ├─ PLL1 ──→ 480MHz ──→ M7 Core (SYSCLK)
  │                      ├─ 240MHz AHB Bus
  │                      ├─ 120MHz APB1 (TIM2/3/4/5)
  │                      └─ 120MHz APB2 (TIM1/8, SPI1)
  │
  ├─ PLL2 ──→ 240MHz ──→ M4 Core (CPUCLK)
  │                      ├─ 240MHz AHB Bus
  │                      └─ 120MHz APB
  │
  ├─ PLL3 ──→ 240MHz ──→ QSPI (80MHz max)
  │           ├─ 120MHz ──→ FDCAN (for 500kbps/2Mbps baud)
  │           └─ 48MHz  ──→ USB OTG FS
  │
  └─ LSE 32.768kHz (External Crystal, 3215 package, 12.5pF)
      └─ RTC + LPTIM (Stop2 wake-up timer)
```

## 5. Memory Map

```
 ┌─────────────────────────────────────────────┐
 │              STM32H743 MEMORY MAP           │
 ├─────────────────────────────────────────────┤
 │ 0x0800_0000   ITCM Flash (64KB)             │ ← Bootloader
 │ 0x0801_0000   AXIM Flash (1984KB)           │ ← M7 App + M4 App
 │                                             │
 │ 0x0000_0000   ITCM RAM (16KB)               │ ← M7 Critical Code
 │ 0x2000_0000   DTCM RAM (128KB)              │ ← M7 Critical Data
 │ 0x2400_0000   AXI SRAM (512KB)              │ ← M7 Heap
 │ 0x3000_0000   SRAM1 (128KB)                 │ ← M7 App Data
 │ 0x3002_0000   SRAM2 (128KB)                 │ ← M4 App Data + Stop2 Retained
 │ 0x3004_0000   SRAM3 (64KB)                  │ ← M7↔M4 Shared (HSEM protected)
 │ 0x3800_0000   SRAM4 (64KB)                  │ ← M7 Backup
 │                                             │
 │ 0x9000_0000   QSPI Flash (16MB)             │ ← W25Q128
 │ 0xC000_0000   SDRAM (256MB)                 │ ← DISCO Board
 │                                             │
 │ 0x1FF0_0000   System Boot ROM               │
 │ 0x1FFF_0000   Option Bytes                  │ ← BOR, WRP, RDP
 │ 0x5C00_0000   HSEM Registers                │ ← 32 hardware semaphores
 └─────────────────────────────────────────────┘

 QSPI Flash Partition (W25Q128 16MB):
 ┌──────────┬──────────┬──────────┬──────────┬──────────┬───────────┐
 │Bootloader│ App_A    │ App_B    │ Config   │ Event    │  SLAM Map │
 │ 64KB     │ 512KB    │ 512KB    │ 64KB     │ Log 1MB  │  13.9MB   │
 └──────────┴──────────┴──────────┴──────────┴──────────┴───────────┘

 SRAM3 Shared Memory Layout (M7↔M4, 64KB):
 ┌────────────────────────────────────────────────┐
 │ l1_odom_t       (40B)   Odometry + IMU + BMS  │
 │ VelCmd_t        (16B)   Velocity setpoint     │
 │ BmsData_t       (32B)   BMS snapshot          │
 │ ToFData_t       (256B)  8×8×2 zones + status  │
 │ SafetyState_t   (16B)   Fault codes + FSM     │
 │ Heartbeat       (4B)    M7 counter            │
 │ Reserved        (64KB - 364B)                  │
 └────────────────────────────────────────────────┘
```

## 6. Interrupt Priority Assignment

```
 Priority (preempt, sub)  ──  IRQ              ──  Handler
 ──────────────────────────────────────────────────────────────
 0, 0                     ──  TIM1_UP          ──  FOC Current Loop (20kHz, M7)
 0, 1                     ──  TIM8_UP          ──  FOC Current Loop R (20kHz, M7)
 1, 0                     ──  TIM2_IRQ         ──  Encoder L Capture (10kHz, M7)
 1, 1                     ──  TIM3_IRQ         ──  Encoder R Capture (10kHz, M7)
 2, 0                     ──  SPI1_IRQ         ──  ICM-20948 DMA Complete (1kHz, M7)
 2, 1                     ──  UART7_IRQ        ──  IPC RK3566 RX (3Mbps, M7)
 3, 0                     ──  I2C1_EV_IRQ      ──  BMS SMBus (10Hz, M4)
 3, 1                     ──  I2C1_ER_IRQ      ──  BMS Error Recovery (M4)
 4, 0                     ──  FDCAN1_IT0_IRQ   ──  CAN RX FIFO0 (M7)
 4, 1                     ──  FDCAN1_IT1_IRQ   ──  CAN RX FIFO1 (M7)
 5, 0                     ──  EXTI0_IRQ        ──  Bumper Left (M4)
 5, 1                     ──  EXTI1_IRQ        ──  Bumper Right (M4)
 5, 2                     ──  EXTI2_IRQ        ──  Dock IR Beacon (M4)
 5, 3                     ──  EXTI3_IRQ        ──  ToF Data Ready (M4)
 6, 0                     ──  DMA1_Stream0     ──  UART7 TX Complete (M7)
 6, 1                     ──  DMA1_Stream1     ──  UART7 RX Complete (M7)
 14,0                     ──  SysTick          ──  FreeRTOS Tick (1000Hz)
 15,0                     ──  PendSV           ──  FreeRTOS Context Switch
```

## 7. Signal Flow: Sensor → Decision → Actuator

```
 ┌─────────┐    ┌─────────┐    ┌─────────┐    ┌──────────┐    ┌─────────┐
 │ ICM20948│    │VL53L5CX │    │ BQ40Z50 │    │  Rotary  │    │ Bumper  │
 │ Accel   │    │ 8×8 ToF │    │ Voltage │    │ Encoder  │    │ Switch  │
 │ Gyro    │    │Distance │    │ Current │    │  L + R   │    │  L + R  │
 │ Mag     │    │         │    │ Temp    │    │          │    │         │
 └────┬────┘    └────┬────┘    └────┬────┘    └────┬─────┘    └────┬────┘
      │ SPI1         │ I2C1        │ I2C1         │ TIM2/3       │ EXTI0/1
      │ 1kHz         │ 100Hz       │ 10Hz         │ 10kHz        │ Async
      ▼              ▼             ▼              ▼              ▼
 ┌─────────────────────────────────────────────────────────────────────┐
 │                         M7 CORE @ 480MHz                           │
 │                                                                     │
 │  Madgwick ──→ Roll/Pitch/Yaw ──┐                                   │
 │                                ├──→ EKF Pose Estimator             │
 │  Encoders ──→ v, ω (odometry) ─┘       │                           │
 │                                         ▼                           │
 │                              ┌──────────────────┐                  │
 │                              │   Estimated Pose  │                  │
 │                              │  (x, y, θ, v, ω) │                  │
 │                              └────────┬─────────┘                  │
 │                                       │ UART7 3Mbps                │
 └───────────────────────────────────────┼─────────────────────────────┘
                                         │
                                         ▼
 ┌─────────────────────────────────────────────────────────────────────┐
 │                       RK3566 (Linux)                                │
 │                                                                     │
 │  Pose + ToF + LDS ──→ Cartographer ──→ Grid Map                    │
 │  Map + Goal ──→ A* Global Planner ──→ Waypoints                    │
 │  Waypoints + Pose + Costmap ──→ DWA Local Planner ──→ (v, ω)      │
 │                                                                     │
 │  (v, ω) ──→ UART IPC ──→ H747 M7                                  │
 └───────────────────────────────────────┬─────────────────────────────┘
                                         │ UART7 3Mbps
                                         ▼
 ┌─────────────────────────────────────────────────────────────────────┐
 │                         M7 CORE                                    │
 │                                                                     │
 │  Target v, ω ──→ Differential Decomposition ──→ vL, vR             │
 │  vL, vR ──→ Speed PI ──→ Iq_target ──→ Current PI ──→ Vd, Vq      │
 │  Vd, Vq ──→ CORDIC invPark ──→ SVPWM ──→ TIM1/8 CCR              │
 │                                                                     │
 │  ┌──────────────┐   ┌──────────────┐                               │
 │  │  M4 @240MHz  │   │   BMS Safety │                               │
 │  │  ToF Cliff   │   │   Cell UV/OV │                               │
 │  │  Tilt Detect │   │   Overcurrent│                               │
 │  │  Stall Detect│   │   Overtemp   │                               │
 │  │  Collision   │   │   Precharge  │                               │
 │  │  Drop Detect │   │   Balancing  │                               │
 │  └──────┬───────┘   └──────┬───────┘                               │
 │         │ HSEM              │ HSEM                                 │
 │         └────────┬──────────┘                                      │
 │                  ▼                                                  │
 │         ┌────────────────┐                                         │
 │         │  FAIL-SAFE FSM │                                         │
 │         │  NORMAL        │                                         │
 │         │  SPIN_HOME     │                                         │
 │         │  DOCK_SEARCH   │                                         │
 │         │  STOP          │                                         │
 │         └────────┬───────┘                                         │
 │                  │ PWM_SHUTDOWN GPIO                               │
 └──────────────────┼─────────────────────────────────────────────────┘
                    │
                    ▼
            ┌──────────────┐
            │ DRV8313 L/R  │
            │ BKIN → OFF   │
            │ PWM  → 0%    │
            │ Motors Stop  │
            └──────────────┘
```

## 8. H747 Pin Assignment (LQFP-144)

```
 Function         Pin   Alt Func    Connected To          Notes
 ─────────────────────────────────────────────────────────────────────
 POWER
 VDD              1,19,32,48,64,80,100,116,132      3.3V Rail
 VSS              18,31,47,63,79,99,115,131         GND
 VDDA             13                               3.3V Analog
 VSSA             12                               GND Analog
 VCAP             30                               4.7uF → GND
 VBAT             2                                3.3V (or coin cell)
 NRST             7                                10K↑ + 100nF↓

 CLOCK
 OSC_IN           8                                25MHz XTAL
 OSC_OUT          9                                25MHz XTAL
 OSC32_IN         3                                32.768K XTAL
 OSC32_OUT        4                                32.768K XTAL

 DEBUG
 SWCLK            37  (PA14)                       ST-LINK/V3
 SWDIO            34  (PA13)                       ST-LINK/V3
 SWO              39  (PB3)                        ST-LINK/V3 (optional)

 MOTOR L (TIM1)
 PA8              41  AF1 TIM1_CH1                 DRV8313_L INHA
 PA9              42  AF1 TIM1_CH2                 DRV8313_L INHB
 PA10             43  AF1 TIM1_CH3                 DRV8313_L INHC
 PB13             74  AF1 TIM1_CH1N                DRV8313_L INLA
 PB14             75  AF1 TIM1_CH2N                DRV8313_L INLB
 PB15             76  AF1 TIM1_CH3N                DRV8313_L INLC
 PB12             73  AF1 TIM1_BKIN                Safety Kill Input

 MOTOR R (TIM8)
 PC6              96  AF3 TIM8_CH1                 DRV8313_R INHA
 PC7              97  AF3 TIM8_CH2                 DRV8313_R INHB
 PC8              98  AF3 TIM8_CH3                 DRV8313_R INHC
 PA7              24  AF3 TIM8_CH1N                DRV8313_R INLA
 PB0              26  AF3 TIM8_CH2N                DRV8313_R INLB
 PB1              27  AF3 TIM8_CH3N                DRV8313_R INLC
 PA6              23  AF3 TIM8_BKIN                Safety Kill Input

 CURRENT ADC (ADC1 — Motor L, ADC2 — Motor R)
 PA0              14  ADC1_IN1                     INA240_L U-Phase
 PA1              15  ADC1_IN2                     INA240_L V-Phase
 PA2              16  ADC1_IN3                     INA240_L W-Phase
 PA4              20  ADC2_IN1                     INA240_R U-Phase
 PA5              21  ADC2_IN2                     INA240_R V-Phase
 PC2              92  ADC2_IN3                     INA240_R W-Phase
 PA3              17  ADC3_IN1                     VBUS Sense (divider)

 IMU (SPI1)
 PA5              21  AF5 SPI1_SCK                 ICM-20948 SCL
 PA6              23  AF5 SPI1_MISO                ICM-20948 SDO
 PA7              24  AF5 SPI1_MOSI                ICM-20948 SDI
 PD14             112 GPIO_OUTPUT                  ICM-20948 CS

 ToF + BMS (I2C1)
 PB6              58  AF4 I2C1_SCL                 VL53L5CX + BQ40Z50 + OLED
 PB7              59  AF4 I2C1_SDA                 VL53L5CX + BQ40Z50 + OLED

 CAN (FDCAN1)
 PD0              106 AF9 FDCAN1_TX                SN65HVD230 TXD
 PD1              107 AF9 FDCAN1_RX                SN65HVD230 RXD

 ESP32-C3 (UART4)
 PD1              107 AF8 UART4_TX                 ESP32-C3 RX (复用)
 PD0              106 AF8 UART4_RX                 ESP32-C3 TX (复用)

 RK3566 IPC (UART7)
 PF7              21  AF7 UART7_TX                 RK3566 UART RX
 PF6              18  AF7 UART7_RX                 RK3566 UART TX

 FLASH (QSPI)
 PB2              29  AF9 QUADSPI_CLK              W25Q128 CLK
 PB6              58  AF10 QUADSPI_BK1_NCS         W25Q128 CS
 PD11             110 AF9 QUADSPI_BK1_IO0          W25Q128 DI
 PD12             111 AF9 QUADSPI_BK1_IO1          W25Q128 DO
 PE2              2   AF9 QUADSPI_BK1_IO2          W25Q128 WP
 PD13             112 AF9 QUADSPI_BK1_IO3          W25Q128 HOLD

 ENCODER (TIM2 — Left, TIM3 — Right)
 PA15             50  AF1 TIM2_CH1                 Encoder L A
 PB3              39  AF1 TIM2_CH2                 Encoder L B
 PA0              14  AF2 TIM2_ETR                 Encoder L Z
 PC6              96  AF2 TIM3_CH1                 Encoder R A
 PC7              97  AF2 TIM3_CH2                 Encoder R B
 PB4              40  AF2 TIM3_ETR                 Encoder R Z

 SAFETY GPIO
 PD12             111 GPIO_OUTPUT                  PWM_SHUTDOWN (all motors)
 PB2              29  GPIO_INPUT                   nFAULT_DRV8313_L
 PE4              4   GPIO_INPUT                   nFAULT_DRV8313_R
 PA0              14  GPIO_INPUT                   BUMPER_L (EXTI0)
 PA1              15  GPIO_INPUT                   BUMPER_R (EXTI1)
 PA3              17  GPIO_INPUT                   DOCK_IR (EXTI2)
 PE6              6   GPIO_INPUT                   TOF_INT (EXTI3)

 STATUS LED
 PB0              26  GPIO_OUTPUT                  LED_R
 PB1              27  GPIO_OUTPUT                  LED_G
 PE1              1   GPIO_OUTPUT                  LED_B
```

## 9. DRV8313 Interface Detail

```
 STM32H747                          DRV8313 (QFN-28)
 ─────────                          ────────────────
 TIM1_CH1    ──────────────────→    INHA  (pin 11)
 TIM1_CH1N   ──────────────────→    INLA  (pin 12)
 TIM1_CH2    ──────────────────→    INHB  (pin 13)
 TIM1_CH2N   ──────────────────→    INLB  (pin 14)
 TIM1_CH3    ──────────────────→    INHC  (pin 15)
 TIM1_CH3N   ──────────────────→    INLC  (pin 16)

 SPI1_SCK    ──────────────────→    SCLK  (pin 20)
 SPI1_MOSI   ──────────────────→    SDI   (pin 21)
 SPI1_MISO   ──────────────────→    SDO   (pin 22)
 GPIO_PD14   ──────────────────→    SCS   (pin 23)

 GPIO_PB2    ←──────────────────    nFAULT(pin 18) 10K↑
 GPIO_PE4    ──────────────────→    nSLEEP(pin 19) 10K↓

 OUTA        ──────────────────→    Motor U-Phase
 OUTB        ──────────────────→    Motor V-Phase
 OUTC        ──────────────────→    Motor W-Phase

 INA240_U    ←──────────────────    Phase U shunt (50mΩ)
 INA240_V    ←──────────────────    Phase V shunt (50mΩ)
 INA240_W    ←──────────────────    Phase W shunt (50mΩ)

 VM          ←──────────────────    +12V Motor Rail
 VCP          ──── 1uF 50V ────→   Charge Pump
 CP1/CP2      ──── 0.1uF ────→    Charge Pump Capacitor

 PGND (1-4,8-10) ────────────→    Power GND (star ground)
 AGND (24-28)    ────────────→    Analog GND (star ground)
```

## 10. RK3566 Connection (via 40-pin header)

```
 H747 UART7           RK3566 40-pin GPIO Header
 ──────────           ─────────────────────────
 PF7 (TX)  ────────→  Pin 10 (UART3_RX)
 PF6 (RX)  ────────→  Pin 8  (UART3_TX)
 GND       ────────→  Pin 6  (GND)

 H747 GPIO             RK3566
 ─────────             ──────
 PG8       ────────→  Pin 12 (GPIO)   H747_READY (active low)
 PG7       ←────────  Pin 16 (GPIO)   RK3566_READY (active low)
```

## 11. Current Sense Detail (INA240)

```
 Motor Phase U ──── 50mΩ Shunt (1206, 1W) ──── GND
                     │          │
                     ├─ INA240 IN+ (pin 4)
                     └─ INA240 IN- (pin 3)

 INA240_A1 (Gain = 20V/V):
   Full scale: 0.05Ω × 3A × 20 = 3.0V → ADC reads 4095 @ 3.3V
   Resolution: 3A / 4096 = 0.73mA/LSB
   Bandwidth: 400kHz (plenty for 20kHz FOC)

 Power supply: 3.3V (pin 4), GND (pin 2)
 Output: pin 1 → ADC input (via 100Ω + 1nF low-pass)
 Reference: pin 5 → 1.65V (half-rail, resistor divider from 3.3V)
```

## 12. Fail-Safe Hardware Chain

```
 M4 Safety Monitor @ 100Hz
     │
     ├─ HSEM(0) heartbeat check ── M7 dead >50ms?
     │       │
     │       YES
     │       │
     │       ├─[1] GPIO_PD12 SET → PWM_SHUTDOWN (all motors, <200ns)
     │       │         │
     │       │         ├─ DRV8313_L nSLEEP → LOW → outputs Hi-Z
     │       │         └─ DRV8313_R nSLEEP → LOW → outputs Hi-Z
     │       │
     │       ├─[2] TIM1_BDTR_BKE = 0 → Break Function Disable (stop PWM)
     │       ├─[3] TIM8_BDTR_BKE = 0 → Break Function Disable
     │       ├─[4] NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn)
     │       └─[5] NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn)
     │
     └─ HSEM(0) alive → reset counter, system normal

 Hardware Redundancy:
   GPIO PD12 ──→ DRV8313 nSLEEP  (S/W kill)
   TIM1 BKIN  ──→ DRV8313 BKIN   (H/W kill, <1 clock)
   IWDG        ──→ M4 Reset       (if M4 itself hangs)
```

## 13. Component Datasheets Reference

```
 Component      Package        Key Parameters              Reference Design
 ─────────────────────────────────────────────────────────────────────────
 STM32H747ZIT6  LQFP-144       480+240MHz, 2MB Flash      AN4938 (Getting Started)
 DRV8313        QFN-28 (5×5)   8-60V, 2.5A, SPI          SLVSBA9 (Datasheet)
 INA240A1       SOT-23-5       20V/V, 400kHz BW           SBOS662 (Datasheet)
 ICM-20948      LGA-24 (3×3)   9-axis, 1.8V logic         DS-000189 (Datasheet)
 VL53L5CX       LGA-16         8×8 ToF, 4m, I2C            DS13891 (Datasheet)
 BQ40Z50        TSSOP-30       4S, SMBus v1.1 + PEC        SLUSCD5 (Datasheet)
 SN65HVD230     SOP-8          3.3V CAN, 1Mbps              SLLS546 (Datasheet)
 ESP32-C3       QFN-32 (5×5)   160MHz RISC-V, WiFi4/BLE5   ESP32-C3 Datasheet
 W25Q128JV     SOP-8           128Mbit, QSPI 133MHz         W25Q128JV Datasheet
 MP2315         SOP-8           24V, 3A, Sync Buck          MP2315 Datasheet
 TPS62130       QFN-16 (3×3)   17V, 3A, Buck               SLVSAE9 (Datasheet)
 AMS1117-3.3    SOT-223         1A, LDO, 1.3V dropout       AMS1117 Datasheet
 2204 BLDC      -              190KV, 12N14P, 3A cont       Motor Spec
 AMS22S         -              12-bit Mag Encoder, SSI      AMS22S Datasheet
```
