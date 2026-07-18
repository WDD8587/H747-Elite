# Power Tree Specification

## 1. Power Architecture

The H747 Elite robot is powered by a 4S LiPo battery (14.8V nominal, 12.6V-16.8V range) or a 19V dock input. The power tree distributes voltage through three main rails.

```
                     +------------------+
  19V DC In ---------|  MP2315 12V BUCK |-----> 12V MOTOR RAIL
                     |  (500 kHz)       |           |
                     +------------------+     +-----+------+
                                              |            |
                                        DRV8313 (2x)    Fans
                                          Motor A & B
                     +------------------+
  4S LiPo -----------|  BQ76952 BMS     |
  14.8V nom          |  (battery mgmt)  |
                     +--------+---------+
                              |
                     +--------+---------+
                     |  MP2315 12V BUCK |
                     +--------+---------+
                              |
                     +--------+---------+
                     |  3.3V LDO        |-----> 3.3V RAIL
                     |  (TPS7A91)       |         |
                     +------------------+    +----+------+
                                             |           |
                                       STM32H747     RK3566
                                         Sensors    (via SPI)
                                              |
                                          IMU, ToF,
                                        Motor encoders
```

## 2. Rail Specifications

### 2.1 12V Motor Rail

| Parameter | Value |
|-----------|-------|
| Regulator | MP2315 (buck converter) |
| Input | 19V dock or 14.8V battery via BMS |
| Output | 12.0V +/- 5% |
| Max Current | 6A (2x motor stall: 3A each) |
| Switching Freq | 500 kHz |
| Efficiency | 92% at 3A |
| Ripple | < 50 mVpp |

### 2.2 3.3V Logic Rail

| Parameter | Value |
|-----------|-------|
| Regulator | TPS7A91 (LDO) |
| Input | 12V rail |
| Output | 3.3V +/- 2% |
| Max Current | 1.5A |
| Dropout | 250 mV at 1A |
| PSRR | 60 dB at 100 kHz |

### 2.3 Battery Input (4S LiPo)

| Parameter | Value |
|-----------|-------|
| Configuration | 4S1P |
| Nominal Voltage | 14.8V |
| Voltage Range | 12.6V - 16.8V |
| Capacity | 5200 mAh |
| Energy | 77 Wh |
| Max Discharge | 20A (4C) |
| Chemistry | LiPo |

## 3. Current Budget Per Rail

### 3.1 12V Motor Rail Budget

| Load | Current (A) | Notes |
|------|-------------|-------|
| Motor A (DRV8313) | 0.5 (nom), 3.0 (stall) | 2W FET loss at stall |
| Motor B (DRV8313) | 0.5 (nom), 3.0 (stall) | 2W FET loss at stall |
| Cooling fans (2x) | 0.2 | 40 mA each |
| **Total nom** | **1.2 A** | Normal operation |
| **Total peak** | **6.2 A** | Both motors stalled |

### 3.2 3.3V Logic Rail Budget

| Load | Current (mA) | Notes |
|------|-------------|-------|
| STM32H747 M4 core | 150 | Active, no FPU |
| STM32H747 M7 core | 250 | Active, FPU, cache on |
| RK3566 | 800 | Via on-board LDO |
| IMU (BMI088) | 10 | Accel + Gyro |
| ToF (VL53L1X) | 20 | Active ranging |
| Motor encoders (2x) | 30 | 15 mA each |
| CAN transceiver | 25 | SN65HVD230 |
| WiFi (ESP32) | 80 | Active TX mode |
| Miscellaneous | 35 | Pull-ups, LEDs, level shifters |
| **Total** | **1,400 mA** | Headroom included |

## 4. Power Consumption Estimates

### 4.1 Normal Operation (Sweeping)

| Component | Power (W) | Notes |
|-----------|-----------|-------|
| Motor A (nom) | 6.0 | 12V x 0.5A |
| Motor B (nom) | 6.0 | 12V x 0.5A |
| DRV8313 losses | 1.5 | I^2*R (Rds(on)=100mohm) |
| STM32H747 | 0.8 | M4 + M7 active |
| RK3566 | 2.5 | Running SLAM + UI |
| BMS overhead | 0.3 | BQ76952 + balance |
| Sensors | 0.3 | IMU, ToF, encoders |
| WiFi | 0.4 | ESP32 connected |
| Fans | 2.4 | 12V x 0.2A |
| Conversion loss | 1.2 | 8% of 15W |
| **Total** | **~21.4 W** | |

### 4.2 Peak Power (Motor Stall)

| Component | Power (W) | Notes |
|-----------|-----------|-------|
| Motor A (stall) | 36.0 | 12V x 3.0A |
| Motor B (stall) | 36.0 | 12V x 3.0A |
| DRV8313 losses | 4.0 | 2W each at stall |
| Electronics | 6.0 | MCU + sensors + WiFi |
| Conversion loss | 2.0 | 8% overhead |
| **Total** | **~84 W** | Brief duration only |

### 4.3 Standby / Charging

| Component | Power (W) | Notes |
|-----------|-----------|-------|
| STM32H747 (sleep) | 0.05 | Stop mode |
| BMS | 0.02 | Monitoring only |
| Charger loss | 1.0 | 90% efficiency at 10W |
| **Total** | **~1.1 W** | |

## 5. Battery Runtime

| Scenario | Power | Runtime | Notes |
|----------|-------|---------|-------|
| Normal sweeping | 30 W avg | 2.5 h | Most realistic: intermittent motion |
| Continuous sweeping | 21.4 W | 3.6 h | Theoretical min power |
| Heavy load (peak) | 84 W | 55 min | Brief stalls; battery limited by C-rate |
| Standby | 1.1 W | 70 h | Docked, idle |

Calculation: 77 Wh / 30 W avg = 2.57 h. Derated by 10% for aging = 2.3 h minimum.

## 6. Power Sequencing

On power-up, rails must be enabled in order:

1. **12V rail**: Must stabilize before 3.3V LDO is enabled.
2. **3.3V rail**: Must stabilize before MCU reset is released.
3. **MCU reset**: Released 10 ms after 3.3V stable.
4. **Motor enable**: Gate drive enabled 50 ms after MCU boots.

```
19V In      |#################
12V Out     |    |############
3.3V Out    |    |    |#######
MCU RST     |    |    |    |###
Motor En    |    |    |    |    |###
            +----+----+----+----+----> Time
            0   10ms 20ms 30ms 80ms
```

## 7. Protection

- **Input reverse polarity**: P-channel MOSFET on 19V input.
- **Overcurrent on 12V**: MP2315 cycle-by-cycle current limit at 7A.
- **Overcurrent on motor**: DRV8313 overcurrent protection per phase.
- **Battery undervoltage**: BQ76952 disconnects load at 12.0V (3.0V/cell).
- **Battery overvoltage**: BQ76952 disconnects charger at 16.8V (4.2V/cell).
- **Thermal shutdown**: TPS7A91 folds back at 160C junction.
