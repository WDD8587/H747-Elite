# Motor Control System Design

## Overview

The H747 Elite uses two BLDC motors driven by field-oriented control (FOC) on the STM32H747 M7 core. Each motor is driven by a dedicated timer channel (TIM1 for left, TIM8 for right) feeding into a DRV8313 three-phase gate driver. Feedback is provided by magnetic encoders on the wheel shafts and current sense resistors on the low-side FETs.

## Architecture

```
                       +------------------+
                       |   STM32H747 M7   |
                       |                  |
  TIM1_CH1/2/3 --------+-> Left PWM       |
  TIM8_CH1/2/3 --------+-> Right PWM      |
                       |                  |
  ADC1_IN1  <----------+ Left phase U cur |
  ADC1_IN2  <----------+ Left phase V cur |
  ADC2_IN1  <----------+ Right phase U cur|
  ADC2_IN2  <----------+ Right phase V cur|
                       |                  |
  TIM2_CH1  <----------+ Left encoder A   |
  TIM2_CH2  <----------+ Left encoder B   |
  TIM3_CH1  <----------+ Right encoder A  |
  TIM3_CH2  <----------+ Right encoder B  |
                       +--------+---------+
                                |
              +-----------------+------------------+
              |                                    |
     +--------v---------+              +-----------v--------+
     |   DRV8313 Left   |              |   DRV8313 Right    |
     |  (3-phase gate)  |              |  (3-phase gate)    |
     +--+--+--+--+--+---+              +--+--+--+--+--+-----+
        |  |  |  |  |                     |  |  |  |  |
        U  V  W  |  FAULT                U  V  W  |  FAULT
                 |                                   |
           +-----v-----+                      +-----v-----+
           | Left BLDC |                      | Right BLDC |
           +-----------+                      +-----------+
                 |                                   |
           +-----v-----+                      +-----v-----+
           | Encoder   |                      | Encoder   |
           | AMS22S    |                      | AMS22S    |
           +-----------+                      +-----------+
```

### Communication Buses

- **TIM1/TIM8**: Complementary PWM outputs with dead-time insertion for DRV8313
- **ADC1/ADC2**: Synchronous sampling of phase currents triggered by PWM center-aligned
- **TIM2/TIM3**: Encoder interface in quadrature encoder mode (TI1+TI2)
- **SPI1**: Optional SPI interface to DRV8313 for configuration (gain, slew rate)

## Control Loops

Three nested control loops run on the M7 core at fixed rates scheduled by a dedicated timer (TIM5):

### Current Loop (PI, 2 kHz Bandwidth)

```
Sample Rate: 20 kHz (every 50 us)
Update Rate:  20 kHz (every PWM cycle)

Input:  iq_ref, id_ref (from speed loop)
Output: Vq, Vd (to inverse Park transform)
Sensor: Phase current via ADC (two-phase reconstruction)

PI gains (tuned for 2 kHz BW):
  Kp = 0.85 V/A
  Ki = 320 V/(A*s)

Anti-windup:  clamping with back-calculation, Kb = 1.0/Kp
```

### Speed Loop (PI, 200 Hz Bandwidth)

```
Sample Rate: 1 kHz (every 1 ms)
Update Rate:  1 kHz

Input:  omega_ref (from position loop or remote command)
Output: iq_ref (to current loop)
Sensor: Electrical speed from encoder (M/T method)

PI gains (tuned for 200 Hz BW):
  Kp = 0.12 A/(rad/s) (per pole pair)
  Ki = 4.0 A/rad

Anti-windup:  clamping with back-calculation, Kb = 2.0/Kp

Speed estimation:
  - Low speed (< 100 RPM):  M method (period measurement, 1 encoder pulse)
  - High speed (>= 100 RPM): T method (frequency measurement, fixed time base)
  - Transition:  Hysteresis with 20 RPM overlap
```

### Position Loop (P, 20 Hz Bandwidth)

```
Sample Rate: 100 Hz (every 10 ms)
Update Rate: 100 Hz

Input:  theta_ref (from navigation or trajectory planner)
Output: omega_ref (to speed loop)
Sensor: Cumulative encoder count (integrated from quadrature)

P gain (tuned for 20 Hz BW):
  Kp = 2.5 (rad/s)/rad

Feed-forward:
  - Velocity feed-forward from trajectory: omega_ff = d(theta_ref)/dt
  - Default: enabled during cleaning, disabled during spot cleaning
```

## FOC State Machine

The FOC driver implements a 5-state state machine for robust startup and fault handling:

```
         +---------+
         |  IDLE   | <----------+
         +----+----+            |
              |                 |
              v                 |
         +---------+            |
         |  ALIGN  |            |
         +----+----+            |
              |                 |
              v                 |
         +------------+         |
         | OPEN_LOOP  |         |
         +-----+------+         |
               |                |
               v                |
         +-------------+        |
         | CLOSED_LOOP |        |
         +------+------+        |
                |               |
                v               |
         +----------+           |
         |  RUNNING |-----------+
         +----------+
```

### State Descriptions

#### IDLE
- **Entry**: System init or after fault cleared
- **Action**: PWM outputs disabled, all bridges high-impedance
- **Exit Condition**: Receive START command from system manager
- **Timeout**: N/A (blocking)

#### ALIGN
- **Purpose**: Align rotor to known electrical angle by applying DC current
- **Action**: Apply Id = 0.5A for 200 ms on the estimated 0-degree vector
- **Duration**: 250 ms fixed
- **Exit Condition**: Timer expiry
- **Note**: Rotor may move slightly (< 5 degrees mechanical)

#### OPEN_LOOP
- **Purpose**: Ramp up speed to minimum closed-loop threshold (50 RPM)
- **Action**: Generate rotating field with V/f profile:
  - Start: 10 Hz electrical (5 RPM mechanical for 12-pole motor)
  - Ramp: 20 (Hz/s) electrical
  - End: 100 Hz electrical (50 RPM mechanical)
  - Voltage: 0.1 V/Hz constant
- **Duration**: ~4.5 seconds typical
- **Exit Condition**: Achieve 50 RPM mechanical AND encoder readings consistent with open-loop command

#### CLOSED_LOOP
- **Purpose**: Transition to sensor-based FOC with all loops active
- **Action**: Enable current loop, speed loop, and position loop
  - Initial iq_ref from speed loop output (limit to 0.3A for smooth transition)
  - Enable encoder-based angle (replace open-loop angle estimate)
- **Duration**: 100 ms transient
- **Exit Condition**: Speed error < 5% for 3 consecutive samples

#### RUNNING
- **Purpose**: Normal operation
- **Action**: All control loops active, accepting velocity commands
- **Fault Monitoring** (checked every control cycle):
  - Overcurrent (> 3.5A for > 100 ms)
  - Overvoltage (> 16V on 12V rail)
  - Undervoltage (< 8V on 12V rail)
  - Overspeed (> 120% max rated speed)
  - Encoder fault (no edges for 50 ms)
  - DRV8313 FAULT pin asserted
- **Exit Conditions**:
  - STOP command: Transition to IDLE via coast
  - Fault detected: Transition to IDLE via brake + fault flag
  - Emergency stop: Immediate PWM disable (BKIN)

## Sensorless Fallback Strategy

If the encoder fails during operation, the system degrades gracefully:

### Detection (within 50 ms)
- No encoder edges detected for 50 consecutive ms at > 50 RPM
- Encoder A/B transition violates quadrature sequence (state machine error)
- Encoder count jumps by > 10 counts in one 1 ms period

### Fallback Sequence
1. **Immediate**: Set sensorless_engaged flag, log fault
2. **Speed Estimation**: Switch to BEMF zero-crossing detection via ADC
   - Sample phase voltage at zero-current crossing in PWM off-time
   - Accuracy: +/- 30 RPM above 200 RPM, unreliable below
3. **Angle Estimation**: Use open-loop angle integrator corrected by BEMF phase
   - theta_est = integral(omega_est * dt) + PLL correction from BEMF
4. **Performance Degradation** (graceful):
   - Reduce max torque to 40%
   - Reduce max speed to 60%
   - Disable position hold (position loop forced to omega_ref = 0)
5. **Recovery Monitoring**: Continuously check encoder health
   - If encoder recovers for 100 ms: restore full FOC, clear sensorless flag
6. **Persistent Failure**: After 10 seconds in sensorless mode:
   - Signal fault to M4 safety monitor
   - Initiate controlled stop (decelerate at 200 RPM/s)
   - Coast to idle

## PWM and Timing Configuration

### Timer Configuration (TIM1, TIM8)
```
Mode:              Center-aligned PWM 1
Counter Period:    1680 (50 kHz PWM at 168 MHz)
Dead Time:         84 ns (generator: 14 counts at 168 MHz)
Complementary:     CH1/CH1N, CH2/CH2N, CH3/CH3N
Break Input:       BKIN on PA6 (hardware brake from safety monitor)
Automatic Output:  Disabled (manual enable via SW after fault clear)
```

### ADC Triggering
```
Trigger:           TIM1_TRGO (update event)
Sampling Time:     18.75 cycles
Injected Channels: 4 (phase U/V for left and right)
Conversion Mode:   Dual regular simultaneous (ADC1 + ADC2)
Result Alignment:  Left-aligned (12-bit)
```

### Current Reconstruction
```
Two-phase reconstruction: i_w = -(i_u + i_v)
Sampling window: Center of PWM on-time (minimum 2 us from edge)
Gain: 0.5 V/A (DRV8313 internal sense, 200 mV/A typical)
ADC reference: 3.3V, 12-bit = 0.8 mV/LSB
Resolution: 1.6 mA/LSB
```

## Motor Parameters (From Datasheet)

| Parameter | Value | Unit |
|-----------|-------|------|
| Type | BLDC outrunner | - |
| Pole pairs | 7 (14 poles) | - |
| Stator slots | 12 | - |
| Kv | 190 | RPM/V |
| R (phase-phase) | 0.45 | ohm |
| L (phase-phase) | 0.12 | mH |
| Max continuous current | 3.0 | A |
| Max peak current (2s) | 5.0 | A |
| Rotor inertia | 1.2e-5 | kg*m^2 |
| Gear ratio | 30:1 | - |
| Wheel diameter | 70 | mm |
| Encoder CPR | 360 (12 PPR * 30) | counts/rev |

## Tuning Procedure

1. **Current Loop Tuning**: Inject iq step (0 to 1A), measure rise time via ADC. Target: 175 us rise time (2 kHz BW). Adjust Kp/Ki until critical damping achieved.
2. **Speed Loop Tuning**: Apply speed step (0 to 500 RPM), measure settling time. Target: 1.6 ms settling time (200 Hz BW).
3. **Position Loop Tuning**: Apply position step (0 to 10 rad), measure overshoot. Target: < 5% overshoot, < 100 ms settling time.
4. **Startup Transition Tuning**: Observe ALIGN -> OPEN_LOOP -> CLOSED_LOOP transition. Target: smooth velocity without audible cogging or reverse rotation.

## Fault Handling

| Fault Code | Name | Action | Recovery |
|------------|------|--------|----------|
| 0x01 | Overcurrent | Immediate brake, set fault flag | Manual reset via system manager |
| 0x02 | Overvoltage | Coast, discharge via braking resistor | Auto-recover when V < 13V |
| 0x03 | Undervoltage | Coast, disable PWM | Auto-recover when V > 10V |
| 0x04 | Overspeed | Coast, log event | Auto-recover when omega < max |
| 0x05 | Encoder fault | Fallback to sensorless | Auto-recover after 100ms stable |
| 0x06 | DRV8313 fault | Immediate brake, read SPI fault reg | Manual reset |
| 0x07 | Temperature warning | Reduce max current to 50% | Auto-recover when temp drops |
| 0x08 | Temperature critical | Immediate brake | Manual reset after cooldown |
