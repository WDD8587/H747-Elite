# Certification Requirements

## 1. Applicable Certifications

The H747 Elite robot requires certification for sale in the following markets: United States (FCC), European Union (CE), China (SRRC), and global RoHS compliance.

| Certification | Market | Scope | Priority |
|---------------|--------|-------|----------|
| FCC Part 15B | USA | Unintentional radiator (digital device) | Required |
| FCC Part 15C | USA | Intentional radiator (ESP32 WiFi) | Required |
| CE RED | EU | Radio equipment + EMC + LVD | Required |
| RoHS | Global | Hazardous substance limits | Required |
| UL 2595 | USA | Battery-operated appliances | Recommended |
| IEC 60335-1 | EU | Household appliance safety | Recommended |
| SRRC | China | Radio transmission equipment | Required for China |
| WEEE | EU | Waste electronics recycling | Required |

---

## 2. FCC Part 15 (USA)

### 2.1 Part 15B - Unintentional Radiator

**Applicable standard**: FCC Part 15 Subpart B / ANSI C63.4-2014.

**Scope**: The robot contains a digital device (STM32H747 microcontroller and RK3566 application processor) that generates RF energy internally.

**Test requirements**:
- Conducted emissions: 150 kHz - 30 MHz (Section 15.107)
- Radiated emissions: 30 MHz - 1 GHz (Section 15.109)
- Limits: Class B (residential environment)

**Key mitigations**:
- Fully shielded enclosure for RK3566 and DDR memory.
- Ferrite beads on all DC-DC converter outputs.
- PCB design with continuous ground plane, 4-layer stackup.
- Shielded motor cables with 360-degree ground termination.

**Lab estimate**: $3,500 - $5,000. Duration: 1 week.

### 2.2 Part 15C - Intentional Radiator (ESP32)

**Applicable standard**: FCC Part 15 Subpart C / ANSI C63.10-2013.

**Scope**: The ESP32 WiFi module (2.4 GHz, 802.11 b/g/n) is a pre-certified modular transmitter.

**Test requirements** (per 15.247):
- Conducted output power: < 1 W (30 dBm)
- Band-edge emissions: > 20 dB below peak
- Spurious emissions: > 30 dB below peak (30 MHz - 25 GHz)
- Power spectral density: < 8 dBm per 3 kHz
- Duty cycle correction: Per ANSI C63.10

**Modular certification**: The ESP32 module carries its own FCC ID (FCC ID: 2AC7Z-ESP32). The H747 Elite may use this module under the original grant if:
- The module is shielded.
- The antenna is the same type as certified (PCB trace antenna).
- No RF tuning modifications are made.
- The host product labeling includes the module FCC ID.

If modular re-certification is needed: $8,000 - $12,000.

---

## 3. CE RED (European Union)

### 3.1 Radio Equipment Directive (RED) 2014/53/EU

**Applicable standards**:
- **EMC**: EN 301 489-1 / EN 301 489-17 (WiFi)
- **Radio**: EN 300 328 (2.4 GHz wideband data systems)
- **Safety**: EN 62368-1 (audio/video and ICT equipment)
- **SAR**: EN 62479 (for devices < 20 mW EIRP; ESP32 qualifies)

**Test requirements**:
- Transmitter spurious emissions: 30 MHz - 12.75 GHz
- Receiver spurious emissions: 30 MHz - 6 GHz
- EMC immunity: EN 55024 / EN 55035
- Harmonic distortion: > 20 dB below fundamental
- Frequency stability: +/- 10 ppm

**Notified body**: Required for RED compliance. Use TUV Rheinland or DEKRA.

**Lab estimate**: $10,000 - $15,000 (including EMC, radio, and safety). Duration: 2-3 weeks.

### 3.2 Electromagnetic Compatibility (EMC) Directive 2014/30/EU

**Applicable standards**:
- **Emissions**: EN 55032 (CISPR 32) - ITE equipment
- **Immunity**: EN 55035 (CISPR 35) - ITE immunity

**Key tests**:
- ESD immunity (IEC 61000-4-2): 8 kV contact, 15 kV air
- Radiated RF immunity (IEC 61000-4-3): 80 MHz - 6 GHz, 3 V/m
- EFT / burst (IEC 61000-4-4): 1 kV AC power, 0.5 kV DC
- Surge (IEC 61000-4-5): 1 kV line-to-line, 2 kV line-to-ground
- Conducted RF immunity (IEC 61000-4-6): 150 kHz - 80 MHz, 3 V

---

## 4. RoHS (Global)

**Applicable standard**: EU Directive 2011/65/EU (+ amendments 2015/863).

**Scope**: The H747 Elite must comply with RoHS 3 (six original substances + four phthalates).

**Substance limits**:

| Substance | Maximum Concentration |
|-----------|----------------------|
| Lead (Pb) | < 0.1% (1000 ppm) |
| Mercury (Hg) | < 0.1% |
| Cadmium (Cd) | < 0.01% (100 ppm) |
| Hexavalent Chromium (Cr VI) | < 0.1% |
| PBBs | < 0.1% |
| PBDEs | < 0.1% |
| DEHP | < 0.1% |
| BBP | < 0.1% |
| DBP | < 0.1% |
| DIBP | < 0.1% |

**Implementation**:
- All purchased components must have RoHS declaration from supplier.
- PCB assembly must use RoHS-compliant solder (SnAgCu).
- No exempted uses of lead (all solders must be lead-free).
- Certification self-declared; technical documentation retained.

**Cost**: Minimal (design-in cost). Documentation: $500 - $1,000 for compliance file.

---

## 5. UL 2595 (Battery-Operated Appliances)

**Applicable standard**: UL 2595-2015.

**Scope**: The H747 Elite is a battery-operated appliance with a 4S LiPo battery.

**Key requirements**:
- Overcharge protection: BMS must cut off charge at 4.25V/cell max.
- Overdischarge protection: BMS must cut off load at 3.0V/cell min.
- Short-circuit protection: Fusing on battery output.
- Thermal runaway containment: Battery enclosure must contain failure.
- Cell balancing: Required for series packs > 3 cells.
- Labeling: Battery warning label on enclosure.

**Lab estimate**: $15,000 - $25,000. Duration: 4-6 weeks. Includes battery pack testing, product-level testing, and factory inspection.

**Alternative**: If the battery pack carries its own UL recognition (UL 1642 for cells, UL 2054 for pack), the product-level testing may be reduced.

---

## 6. IEC 60335-1 (Household Appliance Safety)

**Applicable standard**: IEC 60335-1:2010 + A1:2013 + A2:2016.

**Scope**: Household and similar electrical appliances. Applies to the robot's charging dock and mains adapter.

**Key requirements**:
- Creepage and clearance distances.
- Dielectric strength (hi-pot test): 1500 VAC for 1 minute.
- Touch current: < 0.5 mA for Class II.
- Temperature rise: < 65K on accessible surfaces.
- Mechanical hazard: Moving parts guards.
- Overload protection: Motor stalled must not cause fire risk.

**Lab estimate**: $8,000 - $12,000 for the charger/adapter alone (if not already certified).

---

## 7. SRRC (China)

**Applicable standard**: SRRC (State Radio Regulation of China) CEPREI certification.

**Scope**: Any product with radio transmission capabilities sold in China requires SRRC approval.

**Key requirements**:
- RF output power must meet Chinese limits (lower than FCC).
- Frequency range must be within Chinese ISM band (2.4 - 2.4835 GHz).
- Spurious emissions per Chinese standard.
- Product labeling in Chinese.

**Process**:
1. Submit application through Chinese agent.
2. Test at CEPREI or other CNAS-accredited lab.
3. Wait 6-8 weeks for certificate.
4. Label product with SRRC ID.

**Lab estimate**: $3,000 - $5,000. Duration: 6-8 weeks.

---

## 8. Other Regional Certifications

| Certification | Market | Estimated Cost | Notes |
|---------------|--------|---------------|-------|
| ISED (IC) | Canada | $2,000 - $4,000 | Similar to FCC, separate filing |
| MIC / TELEC | Japan | $5,000 - $8,000 | WiFi certification required |
| KC | South Korea | $3,000 - $5,000 | EMC + radio |
| RCM | Australia | $1,500 - $3,000 | EMC + safety, self-declaring |
| ANATEL | Brazil | $4,000 - $6,000 | Radio + safety, local testing |
| BSMI | Taiwan | $2,000 - $4,000 | EMC + safety |

---

## 9. Certification Schedule

```
Phase 1: Pre-compliance (self-test)
  - EMC pre-scan at 3m chamber: Week 1-2
  - ESD / immunity bench test: Week 2-3
  - Fix issues identified: Week 3-4

Phase 2: Formal testing
  - FCC Part 15 (USA): Week 5
  - CE RED (EU): Week 6-7
  - SRRC (China): Week 7-8 (can overlap)

Phase 3: Documentation
  - DoC (Declaration of Conformity): Week 8
  - Technical construction file: Week 8-9
  - User manual / safety instructions: Week 9

Phase 4: Certification
  - Submit to FCC (TCB): Week 9
  - Submit to CE (notified body): Week 9
  - Submit SRRC (agent): Week 9
  - Certificates received: Week 14-16

Total estimated timeline: 16 weeks from start of pre-compliance.
```

---

## 10. Budget Summary

| Certification | Estimated Cost |
|---------------|---------------|
| FCC Part 15B (conducted + radiated) | $5,000 |
| FCC Part 15C (ESP32, if re-cert needed) | $12,000 |
| CE RED (EMC + radio + safety) | $15,000 |
| RoHS (self-declaration) | $1,000 |
| UL 2595 (battery appliance) | $25,000 |
| IEC 60335-1 (charger/adapter) | $12,000 |
| SRRC (China radio) | $5,000 |
| ISED Canada | $3,000 |
| Agent fees (China, Brazil) | $3,000 |
| Pre-compliance testing | $5,000 |
| **Total** | **$86,000** |
