# Motor Drive System Specifications

## 1. Overview

This document specifies the motor drive system for the H747 Elite differential-drive robot.

---

## 2. System Parameters

| Parameter                | Value            | Unit         | Notes                                   |
|--------------------------|------------------|--------------|-----------------------------------------|
| PWM frequency            | 20               | kHz          | Above audible range                     |
| Dead-time insertion      | 100              | ns           | Prevents shoot-through                  |
| Current loop bandwidth   | 2                | kHz          | FOC current regulation                  |
| Speed loop bandwidth     | 200              | Hz           | Speed PI controller                     |
| Position loop bandwidth  | 20               | Hz           | Position PID controller                 |
| Max linear speed         | 300              | mm/s         | 30 cm/s                                 |
| Max acceleration         | 500              | mm/s^2       | 0.05 g                                  |
| Wheel diameter           | 75               | mm           | Polyurethane-coated                     |
| Track width (wheelbase)  | 250              | mm           | Differential drive                      |
| Motor type               | BLDC (PMSM)      | --           | 3-phase, 12 poles                       |
| Nominal voltage          | 12               | V            | From battery BMS output                  |

---

## 3. PWM and Timing

### 3.1 PWM Frequency Selection

20 kHz is chosen to be above the audible range (~16 kHz upper limit for young adults)
while keeping switching losses manageable. At 12 V nominal, IGBTs are not required;
N-channel MOSFETs with fast body diodes suffice.

### 3.2 Dead-Time

The 100 ns dead-time accommodates:
- MOSFET turn-off fall time: ~25 ns (typical)
- MOSFET turn-on rise time: ~15 ns (typical)
- Gate driver propagation delay mismatch: ~20 ns
- Safety margin: 40 ns

Dead-time is enforced in hardware by the STM32H747 advanced timer (TIM1) break
and dead-time insertion feature.

---

## 4. Control Loop Architecture

```
Position Setpoint + -->[Position PID 20 Hz]--> Speed Setpoint
                                                |
                                                v
                              +--[Speed PI 200 Hz]--> Current Setpoint (Iq)
                              |                              |
                              |                              v
                              |          +--[Current PI 2 kHz]--> SVPWM --> Inverter --> Motor
                              |          |                         |
                              |          +--[Current PI 2 kHz]     |
                              |                                    |
                              +---- Encoder (360 cpr) <------------+
```

### 4.1 Current Loop (2 kHz)

- Implements Field-Oriented Control (FOC)
- PI controller with anti-windup
- Iq (torque) and Id (flux) regulation
- Id reference = 0 for surface PMSM (MTPA if IPMSM)
- Sampling synchronized to PWM carrier (center-aligned)
- Current sense via inline shunt resistors (3-shunt topology)
- Overcurrent protection in hardware (comparator, <2 us response)

### 4.2 Speed Loop (200 Hz)

- PI controller with feed-forward
- Speed derived from encoder position differentiation
- Low-pass filter on speed estimate: 50 Hz cutoff
- Torque limiting configurable (max current)

### 4.3 Position Loop (20 Hz)

- PID controller (PD + limited integral)
- Position from encoder (absolute within revolution + accumulated turns)
- Trajectory generator with trapezoidal / S-curve profiles
- Max velocity and acceleration limits enforced

---

## 5. Encoder Specifications

| Parameter               | Value       | Unit         | Notes                                   |
|-------------------------|-------------|--------------|-----------------------------------------|
| Physical resolution     | 12          | PPR          | Pulses per revolution (magnetic)        |
| Gear ratio              | 30:1        | --           | Planetary gearbox                       |
| Effective resolution    | 360         | counts/rev   | After gearbox (12 PPR x 30:1)           |
| Interface               | SPI + Hall  | --           | Redundant outputs                       |
| Index pulse             | 1 per rev   | --           | For homing                              |
| Max electrical speed    | 3000        | RPM          | Motor shaft                             |
| Max mechanical speed    | 100         | RPM          | Output shaft (after gearbox)            |

### 5.1 Position Resolution Calculation

- Encoder: 12 PPR (physical)
- Gearbox: 30:1 reduction
- Effective: 12 x 30 = 360 counts per output revolution
- Angle per count: 360 / 360 = 1.0 degree per count
- With 75 mm wheel diameter:
  - Circumference: pi x 75 = 235.6 mm
  - Distance per count: 235.6 / 360 = 0.65 mm
  - Sub-degree interpolation: 4x quadrature gives 0.16 mm theoretical

---

## 6. Motion Performance

| Parameter                | Value        | Unit         | Notes                                   |
|--------------------------|--------------|--------------|-----------------------------------------|
| Max linear speed         | 300          | mm/s         | 18 m/min                                |
| Max rotational speed     | 120          | deg/s        | 2.09 rad/s                              |
| Max linear acceleration  | 500          | mm/s^2       | 0.05 g                                  |
| Max rotational accel     | 200          | deg/s^2      | 3.49 rad/s^2                            |
| Min turn radius          | 0            | mm           | Differential drive, zero-radius turn    |
| Speed resolution         | ~1           | mm/s         | At 20 Hz position loop update           |

---

## 7. Current Sensing

| Parameter               | Value        | Unit         | Notes                                   |
|-------------------------|--------------|--------------|-----------------------------------------|
| Topology                | 3-shunt      | --           | One shunt per low-side FET              |
| Shunt resistance        | 5            | mOhm         | Low value for minimal power loss        |
| Amplifier gain          | 50           | V/V          | Differential amplifier                  |
| Max measurable current  | 20           | A            | Limited by ADC reference (3.3 V)        |
| Current sense bandwidth | 1            | MHz          | Amplifier bandwidth                     |
| ADC sampling            | 12-bit       | --           | Synchronized to PWM center              |

---

## 8. Thermal Design

| Component         | Max Temp   | Cooling    | Protection                             |
|-------------------|------------|------------|----------------------------------------|
| MOSFETs           | 125 C      | PCB copper | Derating at T > 100 C, shutdown at 125 |
| Gate drivers      | 125 C      | Conduction | UVLO, thermal shutdown                 |
| Shunt resistors   | 100 C      | PCB copper | N/A (passive)                          |
| Motor windings    | 150 C      | Convection | I2t protection in firmware             |
| Gearbox           | 80 C       | Conduction | Temperature sensor on housing          |

---

## 9. Protection Features

| Protection              | Threshold         | Response Time  | Action                                |
|-------------------------|-------------------|----------------|---------------------------------------|
| Phase overcurrent       | 20 A             | <2 us          | Hardware PWM shutdown                 |
| Motor overcurrent       | 10 A             | 5 s            | Firmware current limit reduction      |
| Supply undervoltage     | 9 V              | 1 ms           | Motor stop, safe state                |
| Supply overvoltage      | 16 V             | 1 ms           | Motor stop, charge FET off            |
| MOSFET overtemperature  | 125 C            | 100 ms         | Derate current, stop if exceeding     |
| Motor overtemperature   | 150 C            | 1 s            | Emergency stop                        |
| Encoder fault           | Invalid position | 1 ms           | Switch to Hall-sensor backup          |
| Stall detection         | Speed < 5% set   | 500 ms         | Current limit, retry, fault if repeat |

---

## 10. Calibration Parameters (Production)

Parameters measured during factory calibration (see `factory/motor_cal.c`):

| Parameter                | Typical Value  | Unit           |
|--------------------------|----------------|----------------|
| Phase resistance (R)     | 50             | mOhm           |
| Phase inductance (L)     | 50             | uH             |
| Back-EMF constant (Ke)   | 50             | Vpk/krpm       |
| Encoder offset           | 0-360          | electrical deg |
