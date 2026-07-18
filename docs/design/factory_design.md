# Factory Test System Design

## Overview

The H747 Elite factory test system consists of 7 test stations arranged in a production line. Each station performs specific tests with automated pass/fail determination. The system integrates with a Manufacturing Execution System (MES) via REST API for traceability and yield analysis.

## Production Line Layout

```
  Station 1         Station 2         Station 3          Station 4
+---------------+  +---------------+  +---------------+  +---------------+
| ICT / Flying  |->| Functional   |->| Calibration   |->| ToF Test Box  |
| Probe         |  | Test         |  | Chamber (IMU) |  |               |
| (30s)         |  | (45s)        |  | (60s)         |  | (20s)         |
+---------------+  +---------------+  +---------------+  +---------------+
                                                              |
                                                              v
  Station 7         Station 6         Station 5
+---------------+  +---------------+  +---------------+
| Final         |<-| Aging / Burn  |<-| RF Test       |
| Inspection    |  | In (2h)       |  | Chamber       |
| (30s)         |  |               |  | (30s)         |
+---------------+  +---------------+  +---------------+

Target Takt Time: 90 seconds (excluding aging)
Production Rate:  300 units/day (single shift, 7.5 hours)
```

## Station Specifications

### Station 1: ICT / Flying Probe

**Purpose**: Verify PCB manufacturing quality before population

```
Equipment:     Flying probe tester (Takaya APT-1400F or equivalent)
Fixture:       Vacuum bed of nails, 128 test points
Time:          30 seconds per board

Tests:
  1. Continuity: All nets verified for shorts/opens
  2. Component presence: Key passives (R, C, L) measured
  3. Diode orientation: TVS and protection diodes verified
  4. Voltage rail shorts: 3.3V, 5V, 12V rails checked for shorts
  5. IC power/GND check: All BGA/QFN power pins show correct resistance
  6. Boundary scan: JTAG scan chain integrity (if available)

Pass Criteria:
  - All nets within 5 ohm of expected
  - No shorts (< 10 ohm between different nets)
  - All components present within +/- 20% tolerance
  - No opens on JTAG chain

Output: PCB serial number, test result JSON
```

### Station 2: Functional Test

**Purpose**: Verify assembled PCB functions correctly with minimal firmware

```
Equipment:     Custom test fixture with pogo-pin bed
Fixture:       Pneumatic clamp, 64 pogo pins + USB + SWD + CAN
Time:          45 seconds

Setup:
  1. Load minimal test firmware via SWD (ST-Link/V3)
  2. Power via bench supply at 19V, 5A limit
  3. Connect CAN to test bus
  4. Connect UART for debug output

Automated Tests:
  1. Power-on sequence: Measure rail sequencing (3.3V, 5V, 12V)
     - All rails within 5% of nominal within 50ms of power
  2. MCU clock: Verify HSE (25 MHz), HSI (64 MHz), LSE (32.768 kHz)
     - Measure via MCO pin, frequency within 1%
  3. Flash readback: Write pattern, read back, verify
  4. SRAM test: March C algorithm on all SRAM (1 MB)
  5. I2C scan: Verify BQ40Z50 at 0x0B, ICM-20948 at 0x68
  6. SPI loopback: MISO->MOSI short, send pattern, verify echo
  7. UART loopback: TX->RX short, send "UART_TEST", expect echo
  8. CAN loopback: Send test frame, verify echo
  9. GPIO toggle: All GPIOs toggle, verify via loopback connector
  10. USB device: Enumerate as HID device, verify descriptor

Pass Criteria: All 10 tests pass
Output: Test protocol JSON, firmware version, MAC (from UID)
```

### Station 3: Calibration Chamber (IMU)

**Purpose**: Calibrate ICM-20948 IMU (accelerometer, gyroscope, magnetometer)

```
Equipment:     Automated 6-face rotation stage (custom)
Fixture:       Robot clamped in known orientation
Time:          60 seconds

Calibration Sequence:
  1. 6-Face Static Calibration (30s)
     Orient robot in 6 orthogonal faces:
     - Face 1: +X up (robot forward up)
     - Face 2: -X up (robot backward up)
     - Face 3: +Y up (robot left up)
     - Face 4: -Y up (robot right up)
     - Face 5: +Z up (wheels down, normal)
     - Face 6: -Z up (wheels up, inverted)
     
     For each face (5 seconds):
       - Collect 500 accelerometer samples at 100 Hz
       - Compute mean and variance
       - Expected: +/- 1G on axis, +/- 0G on others

  2. Gyroscope Bias (15s)
     - Stationary for 15 seconds
     - Compute gyro bias (mean of 1500 samples at 100 Hz)
     - Bias limit: < 0.5 dps per axis

  3. Magnetometer Hard Iron (15s)
     - Continuous rotation for 2 full revolutions (360 degrees)
     - Record magnetometer at 50 Hz
     - Fit ellipsoid to compute hard-iron offsets
     - Hard iron limit: < 100 uT per axis

Calibration Data Stored:
  - Accelerometer: 3x3 misalignment matrix, 3 offsets
  - Gyroscope: 3 biases (dps)
  - Magnetometer: 3 hard-iron offsets, 3x3 soft-iron matrix
  - Temperature: Reference temperature for bias compensation

Output: Calibration parameters in JSON, stored to persistent flash
```

### Station 4: ToF Test Box

**Purpose**: Verify VL53L5CX ToF sensor ranging accuracy

```
Equipment:     Calibrated target at known distances (black matte)
Fixture:       Dark box (anechoic, no external IR), targets on linear slide
Time:          20 seconds

Test Sequence:
  1. Background check: Measure ambient IR (no target)
     Expected: < 500 counts (dark box)
     
  2. Range accuracy at 4 distances:
     - 100 mm +/- 5 mm
     - 300 mm +/- 10 mm
     - 500 mm +/- 15 mm
     - 1000 mm +/- 50 mm
     
     For each distance (5 seconds):
       - Take 10 measurements
       - Verify mean within tolerance
       - Verify variance < 10 mm
       - Verify all 8x8 zones report consistent distances (+/- 20%)

  3. Crosstalk check: Measure with target at 45 degrees
     - Expected: reduced signal but valid range

Pass Criteria: All distance measurements within tolerance
Output: Calibration offset for each zone (if systematic error > 2 mm)
```

### Station 5: RF Test Chamber

**Purpose**: Verify WiFi and BLE RF performance

```
Equipment:     Shielded box + spectrum analyzer + signal generator
Fixture:       Robot in box, RF port connected via u.FL cable
Time:          30 seconds

WiFi Tests (802.11ac, 5 GHz band):
  1. Transmitter power: CH36 (5.18 GHz), CH64 (5.32 GHz), CH149 (5.745 GHz)
     - TX power: 15 dBm +/- 2 dB
     - EVM: < -30 dB (64-QAM MCS7)
     - Spectral mask: Within regulatory limits
     
  2. Receiver sensitivity: CH36 at varying levels
     - PER < 10% at -82 dBm (MCS0)
     - PER < 10% at -68 dBm (MCS7)

  3. BLE TX power: CH0 (2.402 GHz), CH19 (2.44 GHz), CH39 (2.48 GHz)
     - TX power: 0 dBm +/- 3 dB
     - Modulation index: 0.45 - 0.55

  4. BLE Receiver sensitivity
     - PER < 10% at -90 dBm

Pass Criteria: All RF parameters within tolerance
Output: RF test report, MAC addresses written to flash
```

### Station 6: Aging / Burn-In

**Purpose**: Verify reliability with extended operation

```
Equipment:     Burn-in rack (holds 12 robots), ambient chamber
Fixture:       Robots on rack with dock power + UART monitoring
Time:          2 hours

Burn-In Profile:
  Phase 1 (30 min): Ambient temperature, 25C +- 5C
    - Continuous motor exercise (sweep, random walk)
    - Sensor polling at operational rates
    
  Phase 2 (60 min): Elevated temperature, 45C +- 2C
    - Same motor exercise
    - Monitor for thermal faults
    - Verify no excessive drift in IMU calibration

  Phase 3 (30 min): Cool down, ambient temperature
    - Final functional check (short POST)
    - Verify all systems nominal

Monitoring (during entire burn-in):
  - Temperature logging (MCU, DRV8313, BQ40Z50): every 60s
  - Motor current: every 10s
  - Error counters: reported on any increment
  - Watchdog: verify no unexpected resets
  - Heartbeat: verify all nodes responsive

Pass Criteria:
  - No unexpected resets
  - No persistent faults
  - All temperatures below limits:
    - MCU < 85C
    - DRV8313 < 100C (junction)
    - BQ40Z50 < 60C
  - Motor current within 20% of expected range

Output: Burn-in log (temperature profile, events, final status)
```

### Station 7: Final Inspection

**Purpose**: Visual inspection, packaging, and shipping

```
Equipment:     Visual inspection station, calibrated scale, packaging
Fixture:       Manual (operator-assisted)
Time:          30 seconds

Inspection Items:
  1. Cosmetic inspection:
     - No scratches, dents, or discoloration on housing
     - Button alignment and actuation feel
     - LED window clarity
     - Wheel alignment (both wheels parallel)
     - Bumper free movement (no binding)
     
  2. Label verification:
     - Serial number label present and legible
     - Regulatory labels (FCC, CE, SRRC, RoHS)
     - Model number: H747-ELITE
     - MAC address label (matches stored MAC)
     
  3. Accessories:
     - Charging dock (1)
     - Power adapter 19V 3A (1)
     - Power cord (1)
     - User manual (1)
     - Extra dust filter (2)
     - Side brushes (2)
     
  4. Weight: 3.2 kg +/- 0.1 kg

  5. Documentation:
     - Packing slip scanned into MES
     - Test summary report printed
     - Warranty card included

Output: Inspection checklist signed off in MES
```

## MES Integration

### REST API

```
Base URL: https://mes.h747-elite.com/api/v1
Auth:     Bearer token (per-station, rotated monthly)
Timeout:   5 seconds
Retries:   3 (exponential backoff)

Endpoints:

POST /api/v1/units
  Description: Register new unit (start of production)
  Payload: {
    "serial": "H747-2025-00042",
    "product": "H747-ELITE",
    "station": "ict",
    "timestamp": "2025-01-15T09:00:00Z"
  }
  Response: 201 { "unit_id": "uuid" }

POST /api/v1/units/{serial}/tests
  Description: Report test result for a station
  Payload: {
    "station": "functional",
    "result": "pass" | "fail",
    "timestamp": "2025-01-15T09:00:45Z",
    "tests": [
      {"name": "power_on", "result": "pass", "value": "3.31V"},
      {"name": "flash_test", "result": "pass"}
    ],
    "operator": "operator-42",
    "test_software_version": "1.2.3"
  }
  Response: 200 { "status": "recorded" }

GET /api/v1/units/{serial}
  Description: Get unit production history
  Response: {
    "serial": "H747-2025-00042",
    "status": "in_production" | "passed" | "failed" | "shipped",
    "stations": [...],
    "current_station": "inspection"
  }

POST /api/v1/units/{serial}/ship
  Description: Mark unit as shipped
  Payload: {
    "ship_date": "2025-01-15",
    "destination": "warehouse-A",
    "order_number": "ORD-2025-00123"
  }
  Response: 200 { "status": "shipped" }

POST /api/v1/units/barcode
  Description: Trigger station operation by barcode scan
  Payload: {
    "barcode": "H747-2025-00042",
    "station": "calibration"
  }
  Response: {
    "unit_id": "uuid",
    "serial": "H747-2025-00042",
    "previous_station": "functional",
    "previous_result": "pass",
    "next_action": "start_calibration"
  }
```

### Barcode Format

```
Format:      Code 128
Content:     H747-{YYYY}-{NNNNN}
Example:     H747-2025-00042
Check Digit: Luhn algorithm (mod 10)
Length:      16 characters (fixed)
Location:    Label on bottom of robot, also on box
```

### Data Flow

```
Operator scans barcode
  -> MES lookup: is unit in correct station?
  -> If yes: station UI shows test instructions
  -> Operator starts test (button press or auto-trigger)
  -> Station runs automated tests
  -> Test results sent to MES
  -> MES records result
  -> If pass: unit advances to next station
  -> If fail: unit routed to rework station
  -> Operator places unit on next station conveyor
```

## Error Handling

### Station Failures

```
Station Timeout:
  - If station returns no result within 2x expected time -> alert operator
  - If no response for 5 minutes -> auto-cancel station, route unit to review

Test Hardware Failure:
  - Fixture sensor not responding -> pause line, alert maintenance
  - If maintenance not resolved in 15 min -> bypass station for emergency orders

MES Connection Loss:
  - Local station buffer: store up to 100 test results
  - Queue upload when MES recovers
  - If MES down > 1 hour -> continue production with local log
  - When MES recovers -> upload all buffered results with flag "offline"
```

### Rework Process

```
Failed Unit -> Rework Station (Station 1.5)
  1. Operator reviews test failure log
  2. Performs diagnosis per Rework Manual (Doc H747-RWK-001)
  3. Repairs defect (solder, replace component, clean)
  4. Re-runs failed station test
  5. If pass: unit returns to production flow at next station
  6. If fail again: route to engineering review
```

## Production KPIs

| KPI | Target | Measurement Method |
|-----|--------|--------------------|
| Station cycle time | < 90s (excl. aging) | MES timestamps per station |
| First-pass yield | > 95% | Units pass / total tested per station |
| Final yield | > 90% | Units shipped / units started |
| Station uptime | > 99% | MES heartbeat + operator report |
| Mean time to repair | < 30 min | Maintenance log |
| Rework rate | < 5% | Units sent to rework / total |
| Aging failure rate | < 1% | Failures during burn-in / total tested |
