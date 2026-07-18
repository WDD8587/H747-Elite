# Motor Datasheet - H747 Elite Drive Motor

## General Description

Custom BLDC outrunner motor designed for robotic vacuum cleaner drive applications. The motor features a 12-slot stator with 14-pole rotor, integrated planetary gearbox, and magnetic encoder for closed-loop control.

## Motor Specifications

### Electrical Characteristics (Motor Only)

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
|-----------|--------|-----|-----|-----|------|------------|
| Voltage | V_nom | 8 | 12 | 14 | V | Nominal operating range |
| Pole Count | P | - | 14 | - | - | 7 pole pairs |
| Stator Slots | N_s | - | 12 | - | - | Concentric winding |
| Winding Type | - | - | LRK | - | - | Distributed winding |
| Phase Resistance | R_ph-ph | 0.38 | 0.45 | 0.52 | ohm | Line-to-line, 25C |
| Phase Inductance | L_ph-ph | 0.10 | 0.12 | 0.14 | mH | Line-to-line, 1 kHz |
| Motor Constant | Kv | 185 | 190 | 195 | RPM/V | Back-EMF constant |
| Torque Constant | Kt | 0.050 | 0.052 | 0.054 | Nm/A | Kt = 60 / (2*pi*Kv) |
| Max Continuous Current | I_cont | - | 3.0 | - | A | RMS, at 25C ambient |
| Max Peak Current | I_peak | - | 5.0 | 6.0 | A | RMS, < 2 seconds |
| Max Speed (electrical) | omega_e_max | - | 2100 | - | rad/s | 20,000 RPM electrical |
| Max Speed (mechanical) | omega_m_max | - | 300 | - | rad/s | 2,857 RPM mechanical |
| Rotor Inertia | J_rotor | 1.0 | 1.2 | 1.4 | 10^-5 kg*m^2 | Rotor only |
| Damping | B | - | 5.0e-6 | - | Nm/(rad/s) | Mechanical damping |

### Thermal Characteristics

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Thermal Resistance (R_th) | 8.5 | K/W | Winding to ambient, still air |
| Thermal Time Constant | 180 | s | Winding temperature |
| Max Winding Temperature | 130 | C | Class B insulation |
| Max Housing Temperature | 85 | C | Measured on external housing |
| Operating Ambient Temp | -10 to +60 | C | - |

### Mechanical Characteristics (Motor + Gearbox)

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Motor Diameter | 28 | mm | Outrunner housing |
| Motor Length | 20 | mm | Excluding shaft |
| Motor Weight | 52 | g | Motor assembly only |
| Shaft Diameter | 3.175 | mm | 1/8 inch |
| Shaft Length | 12 | mm | Exposed |
| Gearbox Type | Planetary | - | 2-stage |
| Gear Ratio | 30:1 | - | Precision: +/- 1% |
| Gearbox Efficiency | 80 | % | At rated torque |
| Gearbox Rated Torque | 0.8 | Nm | At output shaft |
| Gearbox Max Torque | 1.2 | Nm | Peak, intermittent |
| Gearbox Backlash | < 1.0 | deg | At output |
| Lubrication | Grease | - | Synthetic, NLGI 2 |
| Bearing Type | Ball | - | Shielded, ABEC 3 |

### Combined Motor + Gearbox + Wheel

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Wheel Diameter | 70 | mm | Silicone rubber tire |
| Wheel Width | 25 | mm | Tread contact width |
| Tire Hardness | 55 | Shore A | - |
| Effective Gear Ratio (motor shaft to wheel) | 30:1 | - | - |
| Max Wheel Speed | 95 | RPM | At max motor speed |
| Max Linear Speed | 0.35 | m/s | At max wheel speed |
| Max Tractive Force | 25 | N | At max torque, good grip |
| Typical Stall Torque at Wheel | 1.0 | Nm | At 3A motor current |
| Typical Stall Force | 28 | N | At wheel-ground contact |
| Encoder Resolution at Wheel | 360 | counts/rev | After gearbox, x4 decoding |
| Distance Per Count | 0.61 | mm/count | 70mm wheel / 360 counts |

## Encoder Specifications (AMS22S)

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Type | Magnetic | - | AMS22S (Bourns) |
| Pulses Per Revolution | 12 | PPR | At motor shaft |
| Quadrature Counts | 48 | CPR | x4 decoding, motor shaft |
| Output Type | Incremental | - | A, B channels |
| Output Level | 3.3V | - | Push-pull |
| Max Response Frequency | 25 | kHz | At 125,000 RPM |
| Hysteresis | 10 | deg | Electrical |
| Accuracy | +/- 1.5 | deg | Electrical angle |
| Repeatability | +/- 0.5 | deg | Electrical angle |
| Operating Temperature | -40 to +150 | C | - |

## Performance Curves

### Torque-Speed Characteristic (at 12V)

```
Torque (mNm)    Speed (RPM motor)    Current (A)    Power Out (W)
   0               2,280                0.25             0
   5               2,240                0.35             1.2
   10              2,200                0.45             2.3
   15              2,160                0.55             3.4
   20              2,120                0.65             4.4
   25              2,080                0.75             5.4
   30              2,040                0.85             6.4
   35              2,000                0.95             7.3
   40              1,960                1.05             8.2
   45              1,920                1.15             9.0
   50              1,880                1.25             9.8
   55              1,840                1.35             10.6
   60              1,800                1.45             11.3
   65              1,760                1.55             12.0
   70              1,720                1.65             12.6
   75              1,680                1.75             13.2
   80              1,640                1.85             13.7
   85              1,600                1.95             14.2
   90              1,560                2.05             14.7
   95              1,520                2.15             15.1
   100             1,480                2.25             15.5
```

### Efficiency Map

```
Efficiency (%) vs Torque (mNm) and Speed (RPM motor):

Torque\Speed   500   1000   1500   2000   2280
   10           52     65     72     70     62
   20           58     72     78     76     68
   30           62     75     81     79     71
   40           64     77     82     80     72
   50           65     78     83     80     72
   60           65     78     82     79     71
   70           64     77     81     78     70
   80           63     75     79     76     68
   90           61     73     77     74     66
   100          59     71     75     72     64

Peak Efficiency: 83% at 50 mNm, 1500 RPM
```

## Wiring

| Wire | Color | Function | Connector |
|------|-------|----------|-----------|
| Phase U | Red | Motor phase U | JST-VH 3-pin |
| Phase V | White | Motor phase V | JST-VH 3-pin |
| Phase W | Black | Motor phase W | JST-VH 3-pin |
| Encoder A | Green | Encoder channel A | JST-SH 4-pin |
| Encoder B | Blue | Encoder channel B | JST-SH 4-pin |
| VCC | Red | Encoder power (3.3V) | JST-SH 4-pin |
| GND | Black | Encoder ground | JST-SH 4-pin |

## Ordering Information

| Part Number | Variant | Gearbox | Encoder |
|-------------|---------|---------|---------|
| MOT-H747-L | Left wheel | 30:1 | AMS22S |
| MOT-H747-R | Right wheel | 30:1 | AMS22S |

*Specifications subject to change without notice. For custom gear ratios or encoder options, contact factory.*
