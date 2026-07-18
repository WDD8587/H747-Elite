# EMI Test Plan

## 1. Scope

This document defines the electromagnetic interference (EMI) test plan for the H747 Elite robot. Testing covers conducted emissions (150 kHz - 30 MHz) and radiated emissions (30 MHz - 1 GHz) per FCC Part 15 and CISPR 32.

## 2. Test Standards

| Standard | Scope | Limit |
|----------|-------|-------|
| FCC Part 15B | Unintentional radiator | Class B (residential) |
| FCC Part 15C | Intentional radiator (ESP32) | Class B + band-specific |
| CISPR 32 | ITE equipment | Class B |
| CISPR 25 | Automotive (if applicable) | Class 5 |
| IEC 61000-4-2 | ESD immunity | 8 kV contact, 15 kV air |
| IEC 61000-4-4 | EFT / burst | 1 kV power port |

## 3. Conducted Emissions (150 kHz - 30 MHz)

### 3.1 Test Setup

- **Test standard**: FCC Part 15B / CISPR 32 conducted emissions.
- **Equipment**: Rohde & Schwarz ESRP EMI test receiver.
- **LISN**: 50 uH / 50 ohm line impedance stabilization network.
- **Configuration**: Robot powered via dock (19V DC input). Battery-powered operation also tested.
- **Setup**: Robot placed 40 cm from vertical ground plane, 80 cm above ground plane.

### 3.2 Limits (Class B, Quasi-Peak)

| Frequency Range | Quasi-Peak Limit | Average Limit |
|-----------------|-------------------|---------------|
| 150 kHz - 500 kHz | 66 - 56 dBuV* | 56 - 46 dBuV* |
| 500 kHz - 5 MHz | 56 dBuV | 46 dBuV |
| 5 MHz - 30 MHz | 60 dBuV | 50 dBuV |

*Linear interpolation; limit decreases with log frequency.

### 3.3 Critical Noise Sources

| Source | Frequency | Mechanism | Mitigation |
|--------|-----------|-----------|------------|
| PWM motor drive | 20 kHz fundamental | H-bridge switching | Ferrite bead + 10 nF cap to GND per phase |
| PWM harmonics | 40 kHz - 2 MHz | Sharp switching edges | Gate resistor (10 ohm) on DRV8313 |
| MP2315 buck | 500 kHz fundamental | DC-DC switching | Input/output caps, shielded inductor |
| MP2315 harmonics | 1 - 10 MHz | Ringing on switch node | Snubber (1 nF + 2 ohm) across inductor |
| ESP32 WiFi | 2.4 GHz band | Radiated from antenna | Shield can, ferrite on antenna feed |
| STM32H747 | 400 MHz core clock | Digital switching | On-die decoupling, PCB stackup |
| SPI (M7-RK3566) | 40 MHz | Clock harmonics | Series termination, return path vias |

### 3.4 Mitigation: PWM Filter Design

```
DRV8313 Out ---+--- Ferrite Bead (BLM18KG601, 600 ohm @ 100 MHz) --- Motor
               |
               +--- 10 nF (100V, X7R) --- GND (at driver pin)
```

Ferrite bead selection:
- Impedance: 600 ohm at 100 MHz.
- DC resistance: < 0.05 ohm.
- Current rating: > 3A.
- Part: Murata BLM18KG601TN1.

### 3.5 Mitigation: DC-DC Filter

```
19V In ---+--- C1 (10 uF) --- L1 (4.7 uH) --- C2 (22 uF) ---+--- MP2315
          |                                                   |
          +--- Ferrite bead (600 ohm) ---+--- C3 (0.1 uF) ---+
                                         |
                                        GND
```

## 4. Radiated Emissions (30 MHz - 1 GHz)

### 4.1 Test Setup

- **Test standard**: FCC Part 15B radiated emissions.
- **Equipment**: BiLog antenna (30 MHz - 1 GHz), EMI receiver.
- **Distance**: 3 m measurement distance.
- **Configuration**: Robot operating in typical sweeping mode.
- **Turntable**: 360-degree rotation, antenna height scan 1-4 m.

### 4.2 Limits (Class B, 3 m)

| Frequency Range | Quasi-Peak Limit |
|-----------------|-------------------|
| 30 MHz - 230 MHz | 40 dBuV/m |
| 230 MHz - 1 GHz | 47 dBuV/m |

### 4.3 Critical Nodes

| Node | Frequency | Risk | Mitigation |
|------|-----------|------|------------|
| Motor cables (12V, 3-phase) | 20 kHz - 2 MHz | Conducted emission couples to radiated | Shielded cable, ferrite choke at connector |
| SPI bus (M7-RK3566) | 40 MHz | Clock harmonics to 400 MHz | Series resistor (22 ohm), short traces, ground plane |
| ESP32 antenna | 2.4 GHz | In-band emission | Pre-certified module, PCB trace antenna |
| USB-C port | 480 MHz (USB 2.0) | Common-mode radiation | CM choke, shielded connector |
| Charging contacts | DC + PWM noise | Contact arcing | Snubber (RC) across contacts |

### 4.4 Shielding Strategy

| Area | Shield Type | Material | Notes |
|------|-------------|----------|-------|
| DC-DC converter | EMI shield can | 0.2 mm tin-plated steel | Solder-mount, 6-pin |
| RK3566 | EMI shield can | 0.2 mm tin-plated steel | Covers entire SoC + DDR |
| ESP32 module | Integrated shield | On-module | Pre-certified module |
| Motor cables | Braided shield | Tinned copper | 85% coverage, 360 termination |

## 5. ESD Testing (IEC 61000-4-2)

### 5.1 Test Levels

| Port Type | Contact Discharge | Air Discharge |
|-----------|-------------------|---------------|
| USB-C connector | 8 kV | 15 kV |
| Charging contacts | 8 kV | 15 kV |
| Exposed metal (chassis) | 8 kV | 15 kV |
| Plastic enclosure | N/A | 8 kV |
| Button / UI panel | 4 kV | 8 kV |

### 5.2 ESD Mitigation

| Interface | Protection | Part |
|-----------|-----------|------|
| USB-C D+/D- | TVS diode array | TPD4S012 |
| Charging contacts | TVS diode | SMCJ18A |
| CAN bus | Common-mode choke + TVS | ACT45B-510 + PESD1CAN |
| Button inputs | Series resistor + cap | 1k ohm + 100 pF |
| Motor phases | RC snubber to GND | 1 nF + 2 ohm |

### 5.3 PCB Design Rules for ESD

1. All I/O connectors have TVS protection within 5 mm of the connector.
2. Ground plane flood on all layers with 0.5 mm spacing to edge.
3. No traces run directly under connectors.
4. Chassis ground connected to PCB GND via 1 M ohm + 1 nF (isolate DC, couple ESD).
5. Spark gap (0.3 mm) between chassis GND and signal GND at connector.

## 6. Test Pass Criteria

| Test | Pass Criteria |
|------|---------------|
| Conducted emissions | All frequencies >= 6 dB below limit |
| Radiated emissions | All frequencies >= 4 dB below limit (6 dB margin preferred) |
| ESD, 8 kV contact | No functional disruption, no latch-up |
| ESD, 15 kV air | No permanent damage, auto-recovery within 2 s |
| EFT, 1 kV | No reset, no data corruption |

## 7. Required Test Equipment

| Item | Specification |
|------|---------------|
| EMI Test Receiver | 20 Hz - 26.5 GHz, R&S ESRP or equiv |
| LISN | 50 uH / 50 ohm, 2x 16A |
| BiLog Antenna | 30 MHz - 1 GHz, Schwarzbeck VULB 9162 |
| Horn Antenna | 1 - 6 GHz (for WiFi harmonics) |
| ESD Simulator | IEC 61000-4-2 compliant, 30 kV max |
| EFT/Burst Generator | IEC 61000-4-4 compliant |
| Turntable + Antenna Mast | 3 m semi-anechoic chamber |
