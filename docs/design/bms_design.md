# BMS System Design

## Overview

The H747 Elite uses a 4S (14.8V nominal) Lithium Polymer battery pack managed by a BQ40Z50 gas gauge IC. The BQ40Z50 provides primary protection (overcurrent, overvoltage, undervoltage, short circuit, temperature) via internal FETs. The M4 core provides a secondary software monitor for redundancy.

## Battery Pack Configuration

```
Cell Configuration: 4S1P (4 cells in series, 1 parallel)
Nominal Voltage:    14.8V (4 * 3.7V)
Full Charge:        16.8V (4 * 4.2V)
Capacity:           5200 mAh (77 Wh)
Discharge Rate:     35C continuous (182A)
Chemistry:          LiCoO2 / LiNiMnCo (NMC)
```

## BQ40Z50 Connections

```
+------------------+          +-------------------+
|   BQ40Z50        |          |   STM32H747 M4    |
|                  |          |                   |
| BAT+ <-----------+----------+ Battery positive  |
| BAT- <-----------+----------+ Battery negative  |
|                  |          |                   |
| SDA (pin 17) ----+----------+ PB7 (I2C1_SDA)    |
| SCL (pin 18) ----+----------+ PB6 (I2C1_SCL)    |
|                  |          |                   |
| ALERT (pin 20) --+----------+ PC13 (EXTI)       |
|                  |          |                   |
| CHG (pin 21) ----+----------+ (LED indicator)   |
| DSG (pin 22) ----+----------+ (LED indicator)   |
|                  |          |                   |
| TS1 (pin 10) ----+----------+ NTC thermistor    |
| TS2 (pin 11) ----+----------+ Board temp NTC    |
+------------------+          +-------------------+
```

### I2C Addresses
- BQ40Z50: 0x0B (write) / 0x0C (read)
- BQ40Z50 (alt): 0x16 (write) / 0x17 (read) when CHEM_ID programmed

### I2C Speed
- Standard mode: 100 kHz (for reliable comm during low-voltage conditions)
- Clock stretch: Enabled, max 200 us

## Protection Architecture

### Primary Protection (BQ40Z50 Internal)

The BQ40Z50 provides integrated protection with programmable thresholds. All thresholds configured via Data Flash during calibration.

| Protection | Threshold | Delay | Action |
|------------|-----------|-------|--------|
| Cell Overvoltage (COV) | 4.25V per cell | 2s | CHG FET OFF |
| Cell Undervoltage (CUV) | 2.8V per cell | 10s | DSG FET OFF, SLEEP |
| Overcurrent in Charge (OCC) | 5A | 5s | CHG FET OFF |
| Overcurrent in Discharge (OCD1) | 15A | 2s | DSG FET OFF |
| Overcurrent in Discharge (OCD2) | 30A | 1s | DSG FET OFF |
| Short Circuit in Discharge (SCD) | 100A | 150 us | DSG FET OFF (latch) |
| Short Circuit in Charge (SCC) | 20A | 2s | CHG FET OFF (latch) |
| Internal Temp Hot (T1) | 60C | 5s | CHG FET OFF |
| Internal Temp Hot (T2) | 65C | 5s | CHG + DSG FET OFF |
| Internal Temp Cold (T5) | 0C | 30s | CHG FET OFF |
| External Temp Hot | 55C | 10s | CHG FET OFF |

### Secondary Protection (M4 Software Monitor)

The M4 core continuously monitors the BQ40Z50 readouts as a secondary watchdog.

```
Monitoring Rate:  100 ms (10 Hz)
Communication:    I2C1, read all BQ40Z50 registers
```

#### Monitored Parameters

| Parameter | Register | Warning Threshold | Critical Threshold |
|-----------|----------|-------------------|--------------------|
| Pack voltage | 0x08 | < 12.0V, > 17.0V | < 11.0V, > 17.5V |
| Cell 1 voltage | 0x3F | < 3.0V, > 4.23V | < 2.9V, > 4.28V |
| Cell 2 voltage | 0x40 | < 3.0V, > 4.23V | < 2.9V, > 4.28V |
| Cell 3 voltage | 0x41 | < 3.0V, > 4.23V | < 2.9V, > 4.28V |
| Cell 4 voltage | 0x42 | < 3.0V, > 4.23V | < 2.9V, > 4.28V |
| Pack current | 0x0A | > 12A discharge | > 14A discharge |
| Remaining capacity | 0x0D | < 10% | < 5% |
| State of charge | 0x06 | < 15% | < 10% |
| Temperature (internal) | 0x08 | > 55C | > 60C |
| Temperature (external) | 0x0A | > 50C | > 55C |
| Safety status | 0x50 | Any bit set | Alert triggered |

#### M4 Response Actions

| Condition | Warning Action | Critical Action |
|-----------|---------------|-----------------|
| Low cell voltage | Throttle cleaning power to 50% | Request immediate dock return |
| High cell voltage | Stop charging, send alert | Open charge FET via BQ40Z50 |
| High current | Throttle cleaning power to 70% | Emergency brake + stop |
| High temperature | Throttle all power to 50% | Initiate cool-down: stop all motors |
| Low SOC | Request dock return | Enter low-power navigation |
| Cell imbalance > 100mV | Enable balancing | Return to dock, balance |

## State Machine

```
         +------------+
         |   NORMAL   | <-------------------+
         +-----+------+                     |
               |                            |
       +-------+--------+                   |
       |                |                   |
       v                v                   |
  +----------+    +-------------+           |
  |PRECHARGE |--->| DISCHARGING |-----------+
  +----------+    +------+------+           |
                         |                  |
                         v                  |
                    +----------+            |
                    | CHARGING |------------+
                    +-----+----+           |
                          |                |
                          v                |
                     +-----------+         |
                     | BALANCING |---------+
                     +-----+-----+         |
                           |               |
                     +-----v------+        |
                     |    FAULT   |--------+
                     +-----+------+        |
                           |               |
                           v               |
                     +------------+        |
                     | SHIP_MODE  |--------+
                     +------------+
```

### State Descriptions

#### NORMAL
- **Entry**: Power-on, BQ40Z50 initialized, no faults
- **Action**: Monitor BQ40Z50 readings at 10 Hz; all systems powered
- **Activities**: Allow charging, discharging, balancing as needed
- **Exit Conditions**:
  - Charger detected (CHG_IN sense high) -> PRECHARGE
  - Discharge current > 100mA -> DISCHARGING
  - Fault condition -> FAULT
  - Ship mode command received -> SHIP_MODE
  - Cell imbalance > 100mV for 5 min -> BALANCING

#### PRECHARGE
- **Purpose**: Limit inrush current when charger is first connected
- **Entry Condition**: Charger detected (19V on CHG_IN sense line)
- **Action**: Enable PRECHARGE FET via BQ40Z50 (if available) or external precharge circuit
  - Precharge current limited to 500 mA
  - Duration: Minimum 100 ms or until pack voltage > 16.0V
- **Exit Conditions**:
  - Precharge complete -> CHARGING
  - Precharge timeout (30 sec) -> FAULT (precharge failure)

#### DISCHARGING
- **Purpose**: Normal operation while robot is moving
- **Entry Condition**: Discharge current detected for > 5 seconds
- **Action**: 
  - Monitor SOC, current, voltage, temperature
  - Report to M7 via IPC
  - Periodically update BQ40Z50 load select (current profile)
- **Exit Conditions**:
  - No discharge current for 60 seconds -> NORMAL
  - Charger detected -> PRECHARGE
  - Fault condition -> FAULT
  - SOC < 10% -> Request dock return, transition to NORMAL on dock

#### CHARGING
- **Purpose**: Charge the battery at CC/CV profile
- **Entry Condition**: Precharge complete
- **Action**:
  - Set BQ40Z50 charge voltage: 16.8V (4.2V/cell)
  - Set BQ40Z50 charge current: 2.0A (0.38C)
  - CC phase: Constant 2.0A until cell voltage reaches 4.2V/cell
  - CV phase: Maintain 16.8V, current tapers naturally
  - Termination: Current < 200 mA (0.1C)
  - Monitoring: Cell temperatures every 10s, stop if any cell > 45C
- **Exit Conditions**:
  - Charge complete (all cells > 4.15V AND current < 200mA) -> NORMAL
  - Charger disconnected -> NORMAL
  - Fault condition -> FAULT
  - Cell temperature out of range (0C to 45C) -> FAULT

#### BALANCING
- **Purpose**: Equalize cell voltages to maximize usable capacity
- **Entry Condition**: Cell voltage delta > 100 mV for 5 consecutive minutes
- **Action**:
  - Enable BQ40Z50 cell balancing (passive, bleed resistor)
  - Balancing current: ~50 mA per cell (through 47 ohm resistor)
  - Continue until delta < 20 mV
  - Balancing only enabled when:
    - All cells > 3.5V
    - Pack is idle (no charge/discharge for 30 min)
    - Temperature between 10C and 40C
- **Exit Conditions**:
  - All cells within 20 mV -> NORMAL
  - Charger connected -> CHARGING
  - Robot movement detected -> DISCHARGING
  - Balancing timeout (8 hours) -> NORMAL
  - Fault condition -> FAULT

#### FAULT
- **Purpose**: Safe state for any detected fault condition
- **Entry Condition**: Any protection triggered or warning threshold exceeded
- **Action**:
  - Set BQ40Z50 FETs OFF (both CHG and DSG)
  - Signal fault to M7 and safety monitor
  - Log fault code and timestamp
  - Blink fault LED pattern (2 Hz, red)
  - Wake up every 30 seconds to re-check fault condition
- **Exit Conditions**:
  - Fault condition cleared for 10 consecutive checks -> NORMAL
  - Manual reset via external command -> NORMAL
  - Persistent fault (1 hour) -> SHIP_MODE

#### SHIP_MODE
- **Purpose**: Ultra-low-power state for shipping and storage
- **Entry Condition**: Command via serial console or long press of power button (5s)
- **Action**:
  - BQ40Z50 enters SLEEP mode (draw < 10 uA)
  - All system regulators disabled
  - Only ALERT pin interrupt can wake (from charger or button)
  - BQ40Z50 FETs OFF
- **Power Consumption**:
  - BQ40Z50 in SLEEP: 8 uA typical
  - Total system leakage: < 15 uA
  - Storage duration: > 12 months at 50% SOC
- **Wake Conditions**:
  - Charger connected (CHG_IN 19V)
  - Power button pressed (> 2s)
  - BQ40Z50 ALERT (safety event)
- **Exit Condition**: Wake event detected -> NORMAL

## Gauging (BQ40Z50 Configuration)

### Chemistry Selection
- CHEM_ID: 0x0300 (NMC, 4.2V)
- Verify with: `bqStudio -> Chemistry -> NMC_4200`

### Key Data Flash Parameters

| Parameter | Address | Value | Notes |
|-----------|---------|-------|-------|
| Design Capacity | 0x40A3 | 5200 mAh | 4S1P capacity |
| Design Voltage | 0x40A1 | 14800 mV | 14.8V nominal |
| QMax | 0x4022 | 5200 mAh | Learned after full cycle |
| Taper Current | 0x4096 | 200 mA | Charge termination |
| Taper Voltage | 0x4081 | 4240 mV | Per cell taper threshold |
| Cell Term Voltage | 0x40D4 | 4200 mV | CV per cell |
| Charge Current | 0x400F | 2000 mA | CC charge current |
| Sleep Current | 0x40D3 | 200 uA | Sleep entry threshold |
| Load Mode | 0x4063 | 1 | Current-based (vs power) |
| Load Select | 0x4062 | 0 | Constant current |

### IT (Impedance Track) Configuration
- IT Enable: 1 (enabled)
- IT Cfg: 0x0C (Ra table update + QMax update)
- Learning cycle: Full charge -> discharge to CUV -> full charge required for QMax convergence
- Ra table: Updated after each partial cycle

## Communication Protocol

### Register Map (M4 reads via I2C)

| Register | Address | Size | Read Rate | Description |
|----------|---------|------|-----------|-------------|
| ControlStatus | 0x00 | 2 bytes | 10 Hz | Primary status word |
| SOC | 0x06 | 2 bytes | 10 Hz | State of charge in % |
| PackVoltage | 0x08 | 2 bytes | 10 Hz | Total pack voltage (mV) |
| PackCurrent | 0x0A | 2 bytes | 10 Hz | Pack current (mA, positive=charge) |
| RemainingCap | 0x0D | 2 bytes | 10 Hz | Remaining capacity (mAh) |
| FullChargeCap | 0x0E | 2 bytes | 10 Hz | Full charge capacity (mAh) |
| Temperature | 0x08 | 2 bytes | 10 Hz | Internal temp (0.1K) |
| CellVoltage1 | 0x3F | 2 bytes | 10 Hz | Cell 1 voltage (mV) |
| CellVoltage2 | 0x40 | 2 bytes | 10 Hz | Cell 2 voltage (mV) |
| CellVoltage3 | 0x41 | 2 bytes | 10 Hz | Cell 3 voltage (mV) |
| CellVoltage4 | 0x42 | 2 bytes | 10 Hz | Cell 4 voltage (mV) |
| SafetyAlert | 0x50 | 2 bytes | 1 Hz | Safety alert status |
| SafetyStatus | 0x51 | 2 bytes | 1 Hz | Safety fault status |
| PFAlert | 0x60 | 2 bytes | 1 Hz | Permanent failure alert |
| PFStatus | 0x61 | 2 bytes | 1 Hz | Permanent failure status |

### M4 -> M7 IPC Messages (via shared memory)

| Message ID | Payload | Rate | Description |
|------------|---------|------|-------------|
| 0x10 | SOC (uint8), Volts (uint16) | 10 Hz | Battery status update |
| 0x11 | Current (int16), Temp (int16) | 10 Hz | Current and temperature |
| 0x12 | Cell delta (uint16), fault code(uint8) | 1 Hz | Cell balance and fault info |
| 0x1F | Status (uint8) | On change | Alert / fault notification |

## PCB Layout Considerations

- BQ40Z50 placed close to battery connector (Kelvin sense connection)
- Sense traces: 0.25mm width, matched length for cells 1-4
- I2C pull-ups: 4.7k ohm to 3.3V
- NTC thermistors: 10k ohm, Beta=3435, placed on battery pack and near charge FETs
- Bulk capacitance on pack input: 100 uF electrolytic + 10 uF ceramic + 0.1 uF ceramic
- Charge FETs: Dual N-channel (back-to-back), Rds(on) < 5 mOhm
- Discharge FETs: Single N-channel, Rds(on) < 3 mOhm
- TVS on pack input: 20V bidirectional, 600W peak pulse power
