# Power System Datasheet - H747 Elite

## Overview

The H747 Elite power system comprises a 4S LiPo battery pack with integrated BMS, charging circuitry, and a multi-rail power tree distributing 19V input to 12V, 5V, and 3.3V rails.

## Battery Pack

### Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| Chemistry | LiCoO2 / LiNiMnCo (NMC) | - |
| Configuration | 4S1P | 4 cells in series |
| Nominal Voltage | 14.8V | 4 * 3.7V |
| Full Charge Voltage | 16.8V | 4 * 4.2V |
| Discharge Cutoff | 11.2V | 4 * 2.8V |
| Capacity | 5200 mAh | At 0.2C discharge |
| Energy | 77 Wh | Nominal |
| Internal Resistance | < 50 mOhm | At 1 kHz, 25C |
| Max Continuous Discharge | 15A (3C) | Limited by BMS |
| Max Burst Discharge (2s) | 30A (6C) | Limited by BMS |
| Max Charge Rate | 2A (0.38C) | Limited by BMS |
| Cycle Life | > 500 cycles | To 80% capacity retention |
| Self-Discharge | < 5% per month | At 25C, 50% SOC |
| Operating Temperature (Discharge) | -10C to +60C | - |
| Operating Temperature (Charge) | 0C to +45C | - |
| Storage Temperature | -20C to +45C | Recommended: 25C |
| Storage SOC | 50% | For extended storage |
| Weight | 285 g | Pack only |
| Dimensions | 70 x 60 x 25 mm | L x W x H |
| Connector | Molex 4-pin Micro-Fit 3.0 | With sense wires |

### Cell Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| Cell Type | 18650 | Cylindrical |
| Cell Capacity | 2600 mAh | Per cell (2P = 5200 mAh) |
| Cell Voltage Range | 2.8V - 4.2V | - |
| Cell Max Discharge | 8A continuous | Per cell |
| Cell Impedance | < 25 mOhm | AC, 1 kHz |
| Cell Weight | 48 g | Per cell |

## Input Power

### Charging Dock Input

| Parameter | Value | Notes |
|-----------|-------|-------|
| Input Voltage | 19V DC | From AC adapter |
| Input Current | 3A max | - |
| Connector | Barrel jack 5.5 x 2.1 mm | Center positive |
| AC Adapter Input | 100-240V AC, 50/60 Hz | - |
| AC Adapter Output | 19V DC, 3A (57W) | - |
| AC Adapter Efficiency | > 88% | DoE Level VI |
| AC Adapter Standby Power | < 0.1W | - |

### Dock Contact Charging

| Parameter | Value | Notes |
|-----------|-------|-------|
| Voltage | 19V DC | From dock PCB |
| Current | 2A max | Limited by BMS |
| Contact Type | Spring-loaded pogo pin | 2 pins (+/-) |
| Contact Resistance | < 50 mOhm | After wipe cycle |

## Power Consumption

### Operating Modes

| Mode | Power | Current (from battery) | Description |
|------|-------|----------------------|-------------|
| Ship Mode | < 0.2 mW | < 15 uA | All systems off, BQ40Z50 SLEEP |
| Standby | 0.5 W | 35 mA | M7 sleep, BLE advertising |
| Idle | 5 W | 340 mA | All MCUs on, motors idle, WiFi connected |
| Cleaning (Quiet) | 15 W | 1.0 A | Low suction, low motors |
| Cleaning (Standard) | 30 W | 2.0 A | Nominal suction, nom. motors |
| Cleaning (Turbo) | 55 W | 3.7 A | Max suction, max motors |
| Cleaning (Max) | 72 W | 4.9 A | Max all (both motors + suction stall) |
| Charging (CC phase) | 33 W | 2A @ 16.8V | From input, battery charging |
| Charging (CV phase) | < 10 W | Tapering | From input, battery topping |
| Dock Communication | 1 W | - | Dock idle, waiting for robot |

### Estimated Runtime by Mode

| Mode | Runtime (5200 mAh) | Notes |
|------|-------------------|-------|
| Standby | > 140 hours | 5.8 days |
| Cleaning (Quiet) | 4.5 hours | Hard floor |
| Cleaning (Standard) | 2.5 hours | 50% carpet |
| Cleaning (Turbo) | 1.3 hours | Max suction |
| Cleaning (Max) | 0.9 hours | Sustained max |

## Power Tree

```
                     +------------------+
  19V DC Input ------|  Input Protection |
  (Dock / Adapter)   |  TVS + Polarity  |
                     +--------+---------+
                              |
              +---------------+---------------+
              |                               |
     +--------v---------+           +---------v--------+
     | Battery Pack     |           | MP2315 Buck      |
     | 4S LiPo 14.8V    |           | 19V -> 12V @ 10A |
     | BQ40Z50 BMS      |           | Efficiency: 92%   |
     +--------+---------+           +--------+---------+
              |                               |
              |                        +------v------+
              |                        | 12V Rail    |
              |                        | - DRV8313   |
              |                        | - Motor     |
              |                        | - Fan       |
              |                        +------+------+
              |                               |
              |                        +------v------+
              |                        | TPS62130    |
              |                        | 12V->5V @1A |
              |                        | Efficiency  |
              |                        | 90%         |
              |                        +------+------+
              |                               |
              |                        +------v------+
              |                        | 5V Rail     |
              |                        | - Lidar     |
              |                        | - WiFi/BT   |
              |                        | - USB       |
              |                        | - Cam       |
              |                        +------+------+
              |                               |
              |                        +------v------+
              |                        | AMS1117     |
              |                        | 5V -> 3.3V  |
              |                        | @ 500mA     |
              |                        | Efficiency  |
              |                        | 66%         |
              |                        +------+------+
              |                               |
              |                        +------v------+
              |                        | 3.3V Rail   |
              |                        | - MCU       |
              |                        | - Sensors   |
              |                        | - CAN XCVR  |
              |                        +-------------+
              |
     +--------v---------+
     | Power MUX        |
     | (Battery or 19V) |
     +------------------+
```

### Rail Specifications

#### 12V Rail (Motor Power)

| Parameter | Min | Typ | Max | Unit |
|-----------|-----|-----|-----|------|
| Input Voltage | 8 | 19 | 19 | V |
| Output Voltage | 11.4 | 12.0 | 12.6 | V |
| Output Current (continuous) | - | 5 | 8 | A |
| Output Current (peak) | - | - | 10 | A |
| Output Ripple | - | 20 | 50 | mVpp |
| Load Regulation | - | 0.5 | 1.0 | % |
| Line Regulation | - | 0.2 | 0.5 | % |
| Converter | MP2315 | - | - | - |
| Switching Frequency | - | 500 | - | kHz |

#### 5V Rail (Peripherals)

| Parameter | Min | Typ | Max | Unit |
|-----------|-----|-----|-----|------|
| Input Voltage | 12 | 12 | 12 | V |
| Output Voltage | 4.9 | 5.0 | 5.1 | V |
| Output Current | - | 0.5 | 1.0 | A |
| Output Ripple | - | 10 | 30 | mVpp |
| Load Regulation | - | 0.5 | 1.0 | % |
| Converter | TPS62130 | - | - | - |
| Switching Frequency | - | 2500 | - | kHz |

#### 3.3V Rail (MCU + Sensors)

| Parameter | Min | Typ | Max | Unit |
|-----------|-----|-----|-----|------|
| Input Voltage | 5 | 5 | 5 | V |
| Output Voltage | 3.23 | 3.30 | 3.37 | V |
| Output Current | - | 200 | 500 | mA |
| Output Ripple | - | 5 | 15 | mVpp |
| Load Regulation | - | 0.2 | 0.5 | % |
| Regulator | AMS1117-3.3 | - | - | - |
| Dropout Voltage | - | 1.1 | 1.3 | V |

## Protection

### Input Protection

| Protection | Method | Threshold | Response |
|------------|--------|-----------|----------|
| Reverse Polarity | Series Schottky diode | -19V | Blocks reverse current |
| Overvoltage | TVS diode SMCJ20A | 20V (clamp at 32.4V) | Clamps to safe level |
| Overcurrent | Input fuse (PTC resettable) | 3.5A trip | Opens, auto-reset on cooldown |
| ESD | TVS array USBLC6-2 | Contact 8kV | Clamps to VCC + 0.5V |

### Battery Protection (BQ40Z50 Internal)

| Protection | Threshold | Delay | Recovery |
|------------|-----------|-------|----------|
| Cell Overvoltage (COV) | 4.25V | 2s | Auto when cell < 4.15V, no load |
| Cell Undervoltage (CUV) | 2.8V | 10s | Auto when charge applied |
| Overcurrent Charge (OCC) | 5A | 5s | Auto after 30s no load |
| Overcurrent Discharge (OCD1) | 15A | 2s | Auto after 10s no load |
| Overcurrent Discharge (OCD2) | 30A | 1s | Auto after 10s no load |
| Short Circuit Discharge (SCD) | 100A | 150 us | Latch, requires charger |
| Short Circuit Charge (SCC) | 20A | 2s | Latch, requires reset |
| Internal Overtemp (OT1) | 60C | 5s | Auto when temp < 50C |
| Internal Overtemp (OT2) | 65C | 5s | Auto when temp < 50C |
| Internal Undertemp (UT) | 0C | 30s | Auto when temp > 5C |

## Connectors

| Connector | Type | Pins | Voltage | Current | Purpose |
|-----------|------|------|---------|---------|---------|
| Power Input | Barrel 5.5x2.1mm | 2 | 19V | 3A | AC adapter input |
| Battery | Molex Micro-Fit 3.0 | 4 | 14.8V | 15A | Main power + sense |
| Charge Contact | Spring pogo pin | 2 | 19V | 2A | Dock charging |
| Motor Left | JST-VH | 3 | 12V | 3A | 3-phase motor |
| Motor Right | JST-VH | 3 | 12V | 3A | 3-phase motor |

## Thermal Management

| Component | Cooling Method | Max Temp | Derating |
|-----------|---------------|----------|----------|
| MP2315 (12V buck) | Copper pour, 2oz PCB | 85C | -5% per decade > 60C |
| TPS62130 (5V buck) | Copper pour, 2oz PCB | 85C | -5% per decade > 60C |
| AMS1117 (3.3V LDO) | Copper pour | 100C (junction) | -5% per decade > 70C |
| DRV8313 | Copper pour + fan | 100C (junction) | Current limit at 80C |
| BQ40Z50 | Copper pour | 85C | - |
| Battery (NTC monitored) | Passive | 60C | Charge rate limit at 45C |

*Specifications subject to change without notice. Last updated: 2025-01-15.*
