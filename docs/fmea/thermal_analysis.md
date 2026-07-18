# Thermal Analysis

## 1. Overview

This document analyzes the thermal characteristics of the H747 Elite robot, identifies hot spots, and specifies the cooling strategy to ensure junction temperatures remain below maximum ratings across the operating ambient range (-10C to 45C).

## 2. Hot Spot Identification

### 2.1 DRV8313 Motor Drivers (2x)

| Parameter | Value | Notes |
|-----------|-------|-------|
| Device | DRV8313 | Triple half-bridge |
| Max Rds(on) | 100 mOhm | Per FET at 25C |
| Power per device (nom) | 0.5 W | I_motor = 0.5A, 3 phases |
| Power per device (stall) | 2.0 W | I_motor = 3.0A, 3 phases |
| Max junction temp | 150C | Absolute maximum |
| Recommended max | 125C | For reliability |
| Theta_JA (PCB) | 35 C/W | With 2oz copper, 4-layer board |
| Theta_JT (top) | 15 C/W | With thermal vias to top |

**Total driver dissipation (both motors at stall): 4.0 W**

### 2.2 STM32H747 MCU

| Parameter | Value | Notes |
|-----------|-------|-------|
| Device | STM32H747AI | Dual-core Cortex-M7/M4 |
| Max power | 0.5 W | Both cores active, peripherals on |
| Max junction temp | 125C | Industrial grade |
| Theta_JA | 28 C/W | 4-layer PCB with thermal pad |
| Package | TFBGA225 | Exposed pad on bottom |

### 2.3 RK3566 Application Processor

| Parameter | Value | Notes |
|-----------|-------|-------|
| Device | RK3566 | Quad-core Cortex-A55 |
| Max power | 3.0 W | Full load with GPU |
| Max junction temp | 115C | Industrial spec |
| Theta_JA | 20 C/W | With heatsink |
| Package | FCBGA364 | External heatsink required |

### 2.4 Other Sources

| Component | Power (W) | Notes |
|-----------|-----------|-------|
| TPS7A91 LDO | 0.8 | (12V - 3.3V) * 0.1A quiescent |
| MP2315 buck | 0.5 | Switching loss at 3A |
| BQ76952 BMS | 0.3 | Monitoring + balancing |
| ESP32 WiFi | 0.4 | TX mode |
| Sensors total | 0.1 | IMU, ToF, encoders |

## 3. Thermal Model

### 3.1 System Thermal Resistance Network

```
Tambient (45C max)
    |
    +--- R_cabinet (5 C/W) --- T_case (50C)
    |        |
    |        +--- R_thermal_pad (3 C/W) --- DRV8313_1 (Tj)
    |        |
    |        +--- R_thermal_pad (3 C/W) --- DRV8313_2 (Tj)
    |        |
    |        +--- R_pcb_spread (8 C/W) --- STM32H747 (Tj)
    |        |
    |        +--- R_heatsink (10 C/W) --- RK3566 (Tj)
    |
    +--- R_natural_convection (depends on airflow)
```

### 3.2 Junction Temperature Calculations

**Ambient = 45C, worst-case steady state:**

| Component | Power | Rth | Temp Rise | T_junction | Margin |
|-----------|-------|-----|-----------|------------|--------|
| DRV8313 (stall) | 2.0 W | 35 C/W | 70C | 115C | 10C < 125C |
| DRV8313 (nom) | 0.5 W | 35 C/W | 17.5C | 62.5C | 62.5C < 125C |
| STM32H747 | 0.5 W | 28 C/W | 14C | 59C | 66C < 125C |
| RK3566 | 3.0 W | 20 C/W | 60C | 105C | 10C < 115C |

All components operate within limits at 45C ambient with the specified cooling strategy.

### 3.3 Temperature Gradient Across PCB

The PCB copper pour creates a thermal gradient. At maximum dissipation:
- Motor driver pad: ~65C (center of pour)
- MCU area: ~52C
- Board edge: ~48C
- Ambient air: 45C

## 4. Cooling Strategy

### 4.1 PCB Copper Pour

- **Top layer (signal ground)**: 2 oz copper continuous pour covering the entire board area (100 mm x 100 mm).
- **Inner layer 1 (ground)**: 1 oz copper pour, solid under all high-power components.
- **Inner layer 2 (power)**: 1 oz copper pour for 12V and 3.3V rails.
- **Bottom layer (ground)**: 2 oz copper pour, stitching vias every 5 mm.

Total copper thickness: 6 oz equivalent for thermal spreading.

### 4.2 Thermal Vias Under DRV8313

- **Array size**: 4x4 grid = 16 vias per device.
- **Via diameter**: 0.3 mm finished hole, 0.6 mm pad.
- **Fill**: Conductive epoxy (not solder mask tented).
- **Pitch**: 1.0 mm between via centers.
- **Placement**: Directly under the exposed thermal pad of the DRV8313.

### 4.3 RK3566 Heatsink

- **Type**: Extruded aluminum, finned.
- **Dimensions**: 25 mm x 25 mm x 8 mm (height).
- **Attachment**: Thermally conductive adhesive tape (3M 8810, 0.5 mm thick).
- **Thermal resistance**: 10 C/W (including interface material).
- **Airflow**: Passive convection. Fans provide forced airflow during sweeping.

### 4.4 System Fans

- **Quantity**: 2x 40 mm axial fans.
- **Flow rate**: 5 CFM each at 12V.
- **Placement**: One intake over RK3566, one exhaust over DRV8313 area.
- **Control**: PWM controlled by M7 MCU based on PCB temperature sensor.
- **Fan curve**: 0-40C = off, 40-50C = 30%, 50-60C = 60%, >60C = 100%.

### 4.5 Thermal Interface Materials

| Location | Material | Thickness | Conductivity |
|----------|----------|-----------|--------------|
| DRV8313 -> PCB pad | Solder (SnAgCu) | 0.1 mm | 50 W/mK |
| STM32H747 -> PCB pad | Solder (SnAgCu) | 0.1 mm | 50 W/mK |
| RK3566 -> heatsink | 3M 8810 tape | 0.5 mm | 0.6 W/mK |
| PCB -> chassis | Silicone pad | 1.0 mm | 3.0 W/mK |

## 5. Worst-Case Scenario: Motor Stall at 45C Ambient

At motor stall (3A per phase, both motors), total system dissipation is approximately 10 W (drivers + electronics). The system can sustain this for 30 seconds before the DRV8313 reaches 115C junction temperature. After 30 seconds, the firmware must reduce current (foldback) or shut down to prevent thermal damage.

```
Time to thermal limit (DRV8313, 45C ambient, stall):
    C_thermal = 0.5 J/K (estimated package + pad thermal capacity)
    dT/dt = P / C = 2.0W / 0.5 J/K = 4 K/s
    Max allowed rise = 125C - 65C (steady at 0.5W) = 60C
    Time = 60C / 4 K/s = 15 seconds at sustained stall
```

Safety margin applied: firmware limits stall to 10 seconds, then reduces current by 50%.

## 6. PCB Copper Pour Specifications

| Layer | Cu Weight | Coverage | Notes |
|-------|-----------|----------|-------|
| Top | 2 oz (70 um) | >90% | Ground pour with thermal reliefs |
| Inner 1 | 1 oz (35 um) | >80% | Ground pour, solid under drivers |
| Inner 2 | 1 oz (35 um) | >60% | Power distribution |
| Bottom | 2 oz (70 um) | >90% | Ground pour with fanout vias |

## 7. Thermal Testing Validation

During production, each unit is tested with the following thermal test:

1. Run both motors at 2A each for 60 seconds.
2. Measure PCB temperature at 4 points (under each DRV8313, near MCU, near RK3566).
3. Verify all temperatures below 85C at 25C ambient.
4. Record thermal rise rate for quality tracking.
