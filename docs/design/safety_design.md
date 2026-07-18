# Safety System Design

## Overview

The H747 Elite safety system is designed per ISO 13849-1 to achieve Performance Level PL=c. The architecture is Category 3 (dual channel with monitoring), providing two independent fault detection channels and a cross-monitoring mechanism.

### Applicable Standards

| Standard | Description | Application |
|----------|-------------|-------------|
| ISO 13849-1 | Safety of machinery - Safety-related parts of control systems | Architecture, PL determination |
| ISO 13849-2 | Validation | Test procedures |
| IEC 61508 | Functional safety of electrical/electronic systems | Reference for SIL concepts |
| IEC 60335-1 | Household appliances - Safety | Base standard for robot safety |
| ISO 13482 | Service robot safety | Application-specific requirements |

## Architecture (Category 3)

```
                   +---------------------------------------------+
                   |              Safety System                   |
                   |                                             |
   +---------------+      Channel 1 (M4 Software)                |
   |  Sensor Inputs ---> M4 Safety Monitor Task                  |
   |  (All sensors)  |   - Watchdog timer: 100ms                 |
   |                 |   - Sensor fusion: cross-check readings    |
   |                 |   - Fault detection: 6 categories          |
   |                 |   - Action: CAN-FD fault frame             |
   |                 |   - Brake: TIM1/8 BKIN via SW              |
   +----------------+                                             |
                          xxxxxxxxxxxxxxxxxxxxxxxxxx              |
                   +------+    Cross-Monitoring    +------+       |
                   |      x(heartbeat, diagnostics)x      |       |
                   |      xxxxxxxxxxxxxxxxxxxxxxxxxx      |       |
                   |                                       |       |
   +---------------+      Channel 2 (Hardware TIM BKIN)     |       |
   |  Independent   ---> TIM1 BKIN (PA6)                   |       |
   |  Watchdog:     |   - Independent of M4 firmware       |       |
   |  STM32H747 IWDG |  - Triggered by:                   |       |
   |                 |      * M4 fail (IWDG timeout)       |       |
   |                 |      * HW overcurrent comparator    |       |
   |                 |      * Emergency stop button        |       |
   |                 |      * Stall detection (HW)         |       |
   |                 |   - Action: Immediate PWM disable    |       |
   +----------------+                                       |       |
                                                           |       |
                   +---------------------------------------+       |
                   |  Actuators:                                  |
                   |  - DRV8313 EN pin -> PWM disable              |
                   |  - Brake resistor FET -> dynamic braking      |
                   +---------------------------------------------+
```

## Diagnostic Coverage

### Coverage Analysis per ISO 13849-1

| Component | Failure Mode | Diagnostic Measure | DC |
|-----------|-------------|-------------------|-----|
| M4 MCU | Stuck-at, program counter runaway | IWDG independent watchdog | 99% |
| M4 MCU | Memory corruption | SRAM ECC (STM32H747 feature) | 99% |
| M4 MCU | Clock failure | CSS (Clock Security System) | 99% |
| M4 MCU | Stack overflow | MPU protection + stack canary | 95% |
| Motor driver DRV8313 | Short circuit | Overcurrent comparator + nFAULT | 99% |
| Motor driver DRV8313 | Overtemperature | Internal thermal shutdown | 99% |
| Motor driver DRV8313 | Gate drive failure | Shoot-through detection | 90% |
| Current sensor | Offset drift | Periodic zero-current calibration | 90% |
| Current sensor | Saturation | Redundant sensing (dual ADC) | 90% |
| Encoder | Stuck/ missing pulses | Quadrature state machine check | 90% |
| Encoder | Noise | Debounce filter + plausibility | 95% |
| Battery voltage | Over/under voltage | Comparator + M4 ADC cross-check | 99% |
| Temperature sensor | Open/short circuit | Bias voltage check | 90% |
| Communication (CAN) | Data corruption | CRC + frame counter | 99% |
| Communication (I2C BMS) | Stuck bus | I2C timeout + bus recovery | 90% |
| **System Total DC** | | | **90%** (meets Category 3) |

## MTTFd Calculation

### Component Reliability Data

| Component | Quantity | Failure Rate (FIT) | Source |
|-----------|----------|-------------------|--------|
| STM32H747 MCU | 1 | 50 | STM reliability report |
| DRV8313 (per driver) | 2 | 20 | TI reliability data |
| AMS22S encoder | 2 | 10 | Bourns reliability |
| BQ40Z50 | 1 | 15 | TI reliability data |
| MP2315 (buck) | 1 | 8 | MPS reliability |
| TPS62130 (buck) | 1 | 8 | TI reliability data |
| AMS1117 (LDO) | 1 | 5 | AMS reliability |
| CAN transceiver | 1 | 10 | NXP reliability |
| USBLC6-2 (ESD) | 2 | 1 | STM reliability |
| Passives (total) | ~200 | 40 | Generic (0.2 FIT each) |
| Connectors | 12 | 24 | Generic (2 FIT each) |
| PCB (10 layers) | 1 | 30 | IPC standard |

**Total FIT**: 221  
**MTTFd**: 1e9 / 221 = 4,524,887 hours **> 30 years** (meets Category 3 requirement of > 30 years)

### Assumptions
- FIT values based on industrial temperature range (-40C to +85C)
- Mission profile: 8 hours/day, 365 days/year = 2,920 hours/year
- Useful life: 10 years = 29,200 hours
- All FIT values at 40C ambient with standard quality level

## Achieved PL: c

### PL Determination per ISO 13849-1

| Parameter | Value | Requirement for PL=c |
|-----------|-------|---------------------|
| Category | 3 | 3 |
| DCavg | 90% | High (90-99%) |
| MTTFd | 30 years | Medium (10-30 years) |
| PL | c | Achieved |

## Safety Functions

### SF1: Safe Motor Torque Off (STO)

```
Description: Remove torque from both drive motors
Trigger Conditions:
  - M4 detects fault
  - IWDG timeout
  - Emergency stop button pressed
  - Overcurrent comparator trips
  
Actuation Path:
  Path 1 (M4): GPIO -> DRV8313 EN pin LOW -> PWM disabled
  Path 2 (HW): TIM1/8 BKIN -> Immediate PWM output disable
  
Response Time: < 5 ms (from fault to torque removal)
Stop Category: 0 (uncontrolled stop per IEC 60204-1)
Proof Test Interval: 1 year
```

### SF2: Safe Speed Limit (SLS)

```
Description: Prevent motor speed from exceeding 120% of rated maximum
Trigger Conditions:
  - M4 speed estimation exceeds 120% rated for > 100ms
  - Encoder frequency exceeds 1.2 * f_max for > 10ms
  
Actuation Path:
  M4 -> CAN-FD -> M7 speed loop limit -> throttle to 100% rated
  
Response Time: < 50 ms (from overspeed to limited operation)
Stop Category: 1 (controlled stop, ramp down)
```

### SF3: Safe Battery Disconnect

```
Description: Disconnect battery in fault conditions
Trigger Conditions:
  - BQ40Z50 safety alert
  - M4 detects overcurrent on 12V rail
  - Thermal runaway detected (delta T > 10C/min)
  
Actuation Path:
  M4 -> I2C -> BQ40Z50 -> FET OFF
  
Response Time: < 100 ms
Note: Only disconnects in genuine fault; avoids premature disconnect
```

### SF4: Collision Detection

```
Description: Detect and respond to collisions
Trigger Conditions:
  - Current spike > 2x nominal for < 10ms
  - Sudden velocity drop (> 50% in 10ms)
  - Bumper switch active
  
Actuation Path:
  HW: Bumper switch -> TIM1 BKIN (immediate)
  SW: M4 current anomaly -> M7 -> Stop + reverse 10cm
  
Response Time: < 2 ms (HW), < 20 ms (SW)
```

### SF5: Cliff/Stall Prevention

```
Description: Prevent robot from falling down stairs or driving off ledges
Trigger Conditions:
  - ToF cliff sensor detects drop > 50mm from floor
  - Drop sensor detects no ground within 30mm
  - Wheel stall detected (motor on, no encoder movement for 100ms)
  
Actuation Path:
  M4 -> CAN-FD -> M7 -> Emergency stop -> reverse 20cm
  
Response Time: < 50 ms
```

## Safety-Critical Software Architecture

### M4 Safety Monitor Task

```
Priority: Highest (configMAX_PRIORITIES - 1)
Period: 10 ms (100 Hz)
Stack: 1024 words (4 KB)

Task Flow:
1. Read all sensor inputs (ADC, I2C, GPIO)
2. Validate sensor readings (range check, rate-of-change check)
3. Cross-check redundant sensors
4. Update fault state machine
5. Generate health status
6. Send heartbeat to M7 via shared memory
7. Check M7 heartbeat (expected every 50 ms)
8. Feed IWDG
```

### M4 Safety Monitor State Machine

```
           +-----------+
           |   INIT    | --> Startup checks (1s)
           +-----+-----+
                 |
                 v
           +-----------+
           |   NORMAL  | <-----------------------+
           +-----+-----+                         |
                 |                               |
        +--------+--------+                     |
        |        |        |                     |
        v        v        v                     |
  +---------+ +-------+ +-----------+           |
  | WARNING | | FAULT | | EMERGENCY |-----------+
  +---------+ +-------+ +-----------+           |
        |        |        |                     |
        +--------+--------+---------------------+
```

#### States

| State | Duration | Action | Exit |
|-------|----------|--------|------|
| INIT | 1s | Self-test: check all sensors respond, verify checksums | All self-tests pass -> NORMAL |
| NORMAL | Indefinite | Normal monitoring at 100 Hz | Warning condition -> WARNING; Fault -> FAULT; Emergency -> EMERGENCY |
| WARNING | Variable | Log warning, notify M7, reduce operational limits | 3s without trigger -> NORMAL; Worsening -> FAULT |
| FAULT | Variable | Stop motors, send fault frame, notify user | Fault cleared 10s -> NORMAL; Emergency condition -> EMERGENCY |
| EMERGENCY | Until reset | Immediate stop, latch state, require power cycle | Power cycle only |

### Safety-Critical Memory (MPU Configuration)

```
Region 0: Flash (code)      0x08000000 - 0x080FFFFF   Read-only + Executable
Region 1: SRAM1 (data)      0x30000000 - 0x3003FFFF   Read-write, No-execute
Region 2: SRAM2 (safety)    0x30040000 - 0x30043FFF   Read-write, No-execute
Region 3: Peripherals       0x40000000 - 0x5FFFFFFF   Privileged access only
Region 4: Backup SRAM       0x38800000 - 0x38800FFF   Read-write (fault logs)
Region 5: IWDG + WWDG       0x58002800 - 0x58002FFF   Privileged access only
```

## Fault Logging

Fault events are logged to backup SRAM (retained during reset if VBAT present):

| Field | Size | Description |
|-------|------|-------------|
| fault_code | 1 byte | Unique fault identifier |
| timestamp_ms | 4 bytes | System uptime at fault |
| m4_state | 1 byte | M4 safety state at fault |
| m7_heartbeat_age | 2 bytes | ms since last M7 heartbeat |
| sensor_values[8] | 16 bytes | Snapshot of 8 key sensors |
| crc32 | 4 bytes | CRC32 of entire entry |

Log capacity: 100 entries (28 bytes each = 2800 bytes)

## Validation and Testing

### Type Tests (per ISO 13849-2)

| Test | Description | Pass Criteria |
|------|-------------|---------------|
| Overcurrent injection | Inject 10A into motor driver | STO triggered within 5 ms |
| IWDG failure | Disable M4 watchdog feed | BKIN triggered within IWDG period (4s) |
| CAN bus failure | Disconnect CAN transceiver | M4 detects 3 consecutive missed messages |
| Encoder failure | Disconnect encoder | Sensorless fallback activated, fault logged |
| Emergency stop | Press E-Stop button | Both motors stop within 5 ms |
| Short circuit | Short motor phase to GND | DRV8313 nFAULT asserted, STO triggered |

### Production Tests (100% of units)

| Test | Duration | Acceptance |
|------|----------|------------|
| Self-test on first power-up | 1s | All checks pass, green LED |
| Bumper switch actuation | 3 presses per bumper | All actuations detected |
| Emergency stop button | 3 presses | STO activates each time |
| Encoder plausibility | 10s rotation | No quadrature errors |
| Overcurrent test | 1 pulse | Comparator trips at 5A |

## Maintenance

### Periodic Verification (Recommended)

| Interval | Activity |
|----------|----------|
| Daily (user) | Check nothing stuck under robot, clean sensors |
| Monthly (user) | Test emergency stop function |
| Annually (service) | Full safety function test per ISO 13849-2 |
| 5 years (factory) | Replace battery, inspect wiring, update safety firmware |

### Proof Test

The safety system requires a proof test at 1-year intervals. The proof test validates all safety functions per ISO 13849-2 Table A.1 through A.15. Test procedure is documented in the service manual (Doc #H747-SVC-001).
