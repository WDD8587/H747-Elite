# FMEA Table -- H747 Elite Robot

| #  | Component          | Failure Mode                       | Effect                                                       | Sev | Mitigation                                                         |
|----|--------------------|-----------------------------------|--------------------------------------------------------------|-----|--------------------------------------------------------------------|
| 1  | Motor Driver       | MOSFET short (drain-source)       | Uncontrolled motor motion, overcurrent, thermal runaway      | 10  | Desat protection per FET, fast OC comparator (<2us), thermal sensor|
| 2  | Motor Driver       | MOSFET gate driver open           | Phase open-circuit, torque ripple, motor stall               | 8   | Phase current imbalance detection, stall watchdog timer            |
| 3  | Motor Driver       | Bootstrap capacitor failure       | High-side gate drive loss, partial phase operation           | 7   | Bootstrap undervoltage lockout, phase current monitoring           |
| 4  | Motor Driver       | PWM signal stuck high             | Phase short to battery, overcurrent                          | 9   | Hardware cross-conduction prevention, dead-time enforcement        |
| 5  | Motor Driver       | Thermal shutdown                  | Motor stops, robot immobilised                               | 6   | Redundant cooling, current derating before shutdown                |
| 6  | Encoder            | Magnetic field loss               | Position estimation drift, commutation failure                | 9   | Hall sensor backup mode, plausibility check against back-EMF      |
| 7  | Encoder            | SPI communication bit error       | Intermittent invalid position data                           | 7   | CRC on SPI transaction, median filter on readings                 |
| 8  | Encoder            | Power supply dropout              | Complete position loss                                        | 9   | Supply monitor, position extrapolation for short dropout          |
| 9  | Encoder            | Bearing wear                      | Increasing position noise, eventual mechanical failure        | 5   | Condition monitoring via acceleration spectrum, maintenance alert |
| 10 | IMU                | Accelerometer sensor stuck        | False tilt/orientation readings                              | 8   | Sensor self-test on boot, gyro cross-check, 6-face calibration    |
| 11 | IMU                | Gyroscope drift over temperature  | Heading integration error, navigation drift                   | 7   | Temperature-compensated bias tracking, periodic zero-velocity update|
| 12 | IMU                | SPI bus lockup                    | No IMU data available                                        | 7   | SPI timeout with reset pulse, redundant IMU (secondary)           |
| 13 | IMU                | Vibration-induced aliasing        | Noisy attitude estimate during motion                        | 5   | Low-pass filter with configurable cutoff, notch filter at drive freq|
| 14 | ToF Sensor         | Cover glass contamination         | Reduced ranging distance, false obstacle detection           | 6   | Dirty sensor detection via reference measurement, cleaning alert  |
| 15 | ToF Sensor         | Ambient light saturation          | Invalid range readings in sunlight                           | 6   | Ambient light rejection algorithm, histogram-based detection      |
| 16 | ToF Sensor         | I2C communication failure         | No range data, obstacle detection lost                       | 7   | I2C timeout and retry, watchdog reset of ToF sensor               |
| 17 | ToF Sensor         | Multi-path interference           | Range overestimation near reflective surfaces                | 5   | Multi-zone consistency check, crosstalk calibration               |
| 18 | BMS - Fuel Gauge  | Coulomb counter drift             | Incorrect state-of-charge estimation                         | 7   | Periodic OCV correction at rest, temperature-compensated capacity  |
| 19 | BMS - Cell Monitor | Cell voltage measurement offset   | False overvoltage / undervoltage trip                        | 8   | Cross-check with backup ADC, periodic self-calibration             |
| 20 | BMS - Balancing   | Bleed MOSFET shorted              | Continuous cell discharge, thermal event                     | 9   | MOSFET fault detection, current-limited design, thermal monitoring |
| 21 | BMS - Balancing   | Bleed MOSFET open                 | Unable to balance cells, accelerated cell degradation         | 6   | Cell voltage delta threshold elevated alarm to cloud              |
| 22 | BMS - Precharge   | Precharge FET shorted             | Inrush current on main FET closure, connector arcing          | 8   | Precharge current monitoring, main FET open before precharge fail |
| 23 | BMS - Precharge   | Precharge timeout                 | Main contactor cannot close, robot immobilised                | 7   | Retry 3x, log fault, enter safe state with charge FET only        |
| 24 | Power Supply      | 5V LDO output dropout             | Microcontroller brown-out, system reset                      | 10 | Dual redundant LDOs, supply voltage supervisor (STM32 PVD)        |
| 25 | Power Supply      | 3.3V DC-DC switcher failure       | Digital logic undervoltage, erratic behaviour                 | 9  | Independent monitoring by BMS MCU, graceful shutdown              |
| 26 | Power Supply      | Reverse polarity protection fail  | Whole-system damage from incorrect battery connection         | 10 | Ideal diode OR-ing, TVS clamping, fuse                            |
| 27 | CAN Bus           | Bus short (CANH-CANL)             | Complete communication loss between modules                   | 8  | CAN bus protection diodes, redundant CAN channel                  |
| 28 | CAN Bus           | Termination resistor failure      | Signal reflections, increased bit error rate                  | 5  | Built-in termination with enable/disable, error passive detection |
| 29 | CAN Bus           | CAN controller stuck in bus-off   | No communication for that node                                | 7  | Bus-off recovery protocol, error counter monitoring               |
| 30 | Flash Memory      | Write endurance exceeded          | Calibration data corruption, configuration loss               | 7  | Wear-leveling across sectors, CRC-32 on each page                 |
| 31 | Flash Memory      | Read disturb                      | Single-bit flip in calibration data                           | 6  | ECC (1-bit correction), periodic scrubbing, triple-redundant storage|
| 32 | Flash Memory      | Erase failure (block stuck)       | Unable to update firmware or logs                             | 7  | Bad block management, redundant sectors, factory reflash tool     |
| 33 | Button            | Switch contact bounce             | Multiple triggers on single press                             | 3  | Debounce filter (50 ms), edge-triggered detection                 |
| 34 | Button            | Switch stuck closed               | Continuous press detection, auto-repeat behavior              | 4  | Long-press duration limit, diagnostic on start                   |
| 35 | Speaker           | Amplifier output short            | No audio feedback, potential amplifier damage                 | 4  | Short-circuit protection in audio amplifier, current limiting     |
| 36 | Speaker           | Piezo element cracked             | Reduced volume or no sound                                    | 2  | Acoustic self-test on boot (play tone, detect via microphone)     |
| 37 | LED Indicator     | LED open circuit                  | No visual status indication                                   | 2  | Current monitoring per LED channel (open-load detection)          |
| 38 | LED Indicator     | LED dim due to aging              | Reduced visibility of status                                  | 2  | PWM current drive compensates aging, calibration on manufacturing  |
| 39 | WiFi (ESP32-C3)   | RF power amplifier failure        | Reduced range, inability to connect to cloud                  | 6  | TX power monitoring, antenna diversity                            |
| 40 | WiFi (ESP32-C3)   | Watchdog reset loop               | Periodic connectivity drops, delayed telemetry                | 5  | Co-host watchdog, ESP32 health check via UART heartbeat           |
| 41 | WiFi (ESP32-C3)   | Firmware corruption               | Module non-functional, no cloud connection                    | 7  | Dual firmware slots (golden image + active), CRC verification     |
| 42 | BMS - 1-Wire      | DS2431 EEPROM wear                | Pack identification data lost                                 | 6  | Redundant storage of pack ID in BMS flash, EEPROM wear levelling  |
| 43 | BMS - SMBus       | Clock stretch timeout             | BMS gauge communication failure                               | 6  | SMBus timeout recovery, retry with back-off                       |
| 44 | BMS - Thermistor  | NTC dry joint / open              | Temperature reads -40 C, false undertemp protection           | 7  | Open-circuit detection (R > 500k), redundant NTC per cell group   |
| 45 | BMS - Thermistor  | NTC short circuit                 | Temperature reads 125 C, false overtemp protection            | 7  | Short-circuit detection (R < 100 ohm), redundant NTC per cell group|
| 46 | Charger           | CC/CV loop instability            | Voltage overshoot, cell overvoltage                           | 9  | Hardware OV clamp, redundant comparator BMS protection            |
| 47 | Charger           | Charger disconnection             | Charging stops prematurely, SoC may decrease                  | 4  | Disconnection detection via current sense, resume on reconnect    |
| 48 | Chassis Grounding | Ground loop through CAN shield    | Communication noise, intermittent errors                      | 5  | Star ground topology, isolated CAN transceiver                   |
| 49 | Chassis Grounding | ESD discharge to chassis          | Controller reset or latch-up                                  | 7  | TVS clamping on all external connectors, chassis-to-ground strap  |
| 50 | Main Controller   | STM32H747 clock PLL lock loss     | System halt, watchdog reset                                   | 8  | Clock security system (CSS) interrupt, fallback to HSI            |

**Severity Scale:**
- 10 = Catastrophic (safety-critical, fire, permanent damage)
- 9  = Critical (system damage, major function lost)
- 8  = Severe (primary function lost, no backup)
- 7  = Major (primary function degraded, backup available)
- 6  = Moderate (secondary function lost)
- 5  = Minor (partial degradation)
- 4  = Low (noticeable but acceptable)
- 3  = Minor nuisance
- 2  = Cosmetic only
- 1  = Negligible
