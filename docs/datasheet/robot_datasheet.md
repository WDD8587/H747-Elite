# H747 Elite Robot Vacuum Datasheet

## Product Overview

The H747 Elite is a robotic vacuum cleaner designed for residential and light commercial use. It features LiDAR navigation, SLAM-based mapping, smart home connectivity, and advanced cleaning performance.

## Physical Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| Diameter | 350 mm | Main body |
| Height | 95 mm | With lidar dome |
| Weight | 3.2 kg | With battery |
| Cleaning Path Width | 280 mm | With side brush extended |
| Wheelbase | 320 mm | Center-to-center |
| Ground Clearance | 15 mm | At wheel |
| Dustbin Capacity | 400 ml | Transparent polycarbonate |
| Water Tank Capacity | 200 ml | Mop attachment |
| Material | ABS + PC (UL94 V-0) | Main body |
| Color | Obsidian Black / Pearl White | Two options |

## Performance Specifications

### Cleaning Performance

| Parameter | Value | Test Standard |
|-----------|-------|---------------|
| Suction Power | 2500 Pa | Max setting |
| Suction Motor | Digital BLDC | 100,000 RPM max |
| Airflow | 1.2 m^3/min | At nominal suction |
| Dust Pickup (Carpet) | > 95% | IEC 62885-7 |
| Dust Pickup (Hard Floor) | > 98% | IEC 62885-7 |
| Edge Cleaning | > 90% | IEC 62885-7 |
| Noise Level | 65 dBA (quiet), 72 dBA (turbo) | IEC 60704-2-1 |

### Battery and Runtime

| Parameter | Value | Notes |
|-----------|-------|-------|
| Battery Type | 4S1P LiPo NMC | - |
| Capacity | 5200 mAh / 77 Wh | - |
| Runtime (Quiet Mode) | 150 min | Hard floor, nom. suction |
| Runtime (Standard) | 120 min | 50% carpet, nom. suction |
| Runtime (Turbo) | 60 min | Max suction + max motor |
| Charge Time | 4 hours | 0% to 100% |
| Charge Input | 19V DC, 3A | Barrel jack 5.5x2.1mm |
| Auto-Recharge | Yes | Returns to dock at < 15% |
| Resume Cleaning | Yes | After recharge |

### Navigation and Sensors

| Sensor | Type | Qty | Purpose |
|--------|------|-----|---------|
| LiDAR | LDS triangulation | 1 | Mapping and navigation |
| ToF Ranging | VL53L5CX | 1 | Cliff detection (forward) |
| Camera | IMX415 8MP | 1 | Visual localization |
| IMU | ICM-20948 6-axis | 1 | Orientation and motion |
| Wheel Encoder | AMS22S magnetic | 2 | Odometry |
| Bumper Switch | Mechanical | 2 | Obstacle detection |
| Cliff IR Sensor | IR LED + photodiode | 4 | Drop detection |
| Drop Sensor | Mechanical | 4 | Stair/edge detection |
| Dust Sensor | Optical | 1 | Dust concentration |

### Connectivity

| Interface | Standard | Speed | Range |
|-----------|----------|-------|-------|
| WiFi | 802.11ac (2.4 + 5 GHz) | Up to 867 Mbps | 30m indoor |
| Bluetooth | BLE 5.0 | 2 Mbps | 10m |
| CAN-FD | ISO 11898-1 | 500 kbps / 2 Mbps | Internal bus |

### Smart Features

| Feature | Support |
|---------|---------|
| Mapping | Yes (SLAM, multi-floor, up to 5 maps) |
| Room Selection | Yes (per-room cleaning) |
| No-Go Zones | Yes (via mobile app) |
| Scheduled Cleaning | Yes (weekly schedule) |
| Voice Assistants | Alexa, Google Assistant, Siri Shortcuts |
| Mobile App | iOS 15+, Android 12+ |
| OTA Updates | Yes (A/B partition, auto-rollback) |
| Remote Control | Yes (via app + cloud) |

## Environmental Specifications

| Parameter | Operating | Storage |
|-----------|-----------|---------|
| Temperature | 0C to +40C | -20C to +60C |
| Humidity | 10% to 90% non-condensing | 5% to 95% |
| IP Rating | IPX4 (splash resistant) | - |
| Altitude | < 2000m | < 12000m |

## Regulatory and Certifications

| Certification | Standard | Status |
|---------------|----------|--------|
| FCC (USA) | FCC Part 15 | Certified |
| CE (EU) | EN 60335, EN 55014, EN 300 328 | Certified |
| SRRC (China) | - | Certified |
| RoHS | EU 2011/65/EU | Compliant |
| REACH | EU 1907/2006 | Compliant |
| WEEE | EU 2012/19/EU | Registered |
| UKCA | UK S.I. 2016 No. 1101 | Certified |
| RCM (AU/NZ) | - | Certified |

## Interfaces

### Physical Ports

| Port | Type | Location |
|------|------|----------|
| Charge Contact | 2-pin spring contact | Rear bottom |
| USB-C | USB 2.0 (service only) | Under cover, rear |
| Debug UART | 4-pin JST-SH (3.3V) | Under cover, rear |

### Indicator LEDs

| LED | Color | Pattern | Meaning |
|-----|-------|---------|---------|
| Power | White | Solid | On, normal operation |
| Power | White | Breathing | Charging |
| Power | Amber | Solid | Low battery |
| Power | Amber | Flashing | Battery fault |
| WiFi | Blue | Solid | Connected |
| WiFi | Blue | Flashing | Connection pending |
| Error | Red | 1 flash | Motor fault |
| Error | Red | 2 flashes | Sensor fault |
| Error | Red | 3 flashes | Battery fault |
| Error | Red | 4 flashes | System fault |

## What's in the Box

| Item | Quantity |
|------|----------|
| H747 Elite Robot | 1 |
| Charging Dock | 1 |
| Power Adapter (19V 3A) | 1 |
| Power Cord | 1 |
| User Manual | 1 |
| Quick Start Guide | 1 |
| Extra Dust Filter (HEPA) | 2 |
| Side Brushes | 2 |
| Mop Cloth | 2 |

## Ordering Information

| SKU | Region | Color | Power Cord |
|-----|--------|-------|------------|
| H747-US-BLK | North America | Obsidian Black | US plug |
| H747-US-WHT | North America | Pearl White | US plug |
| H747-EU-BLK | Europe | Obsidian Black | EU plug |
| H747-EU-WHT | Europe | Pearl White | EU plug |
| H747-UK-BLK | UK | Obsidian Black | UK plug |
| H747-AU-BLK | Australia/NZ | Obsidian Black | AU plug |

## Warranty

| Region | Warranty Period | Details |
|--------|----------------|---------|
| USA | 1 year | Parts and labor |
| EU | 2 years | Statutory warranty |
| UK | 2 years | Consumer rights |
| China | 1 year | Parts and labor |

*Specifications subject to change without notice. Last updated: 2025-01-15.*
