# Communication Architecture Design

## Overview

The H747 Elite uses three communication buses: CAN-FD for real-time motor and BMS control, UART IPC for M7-to-RK3566 communication, and WiFi for cloud connectivity. Each bus is optimized for its specific use case with appropriate prioritization and QoS mechanisms.

## System Diagram

```
+------------------+           +------------------+           +------------------+
|   STM32H747 M7   |           |   STM32H747 M4   |           |   RK3566 App     |
|                  |           |                  |           |                  |
| - Motor control  |           | - Safety monitor |           | - SLAM           |
| - FOC            |           | - BMS monitor    |           | - Navigation     |
| - Sensor fusion  |           | - Watchdog       |           | - Cloud comm     |
| - Velocity loop  |           | - Fault handling |           | - User interface |
+--------+---------+           +--------+---------+           +--------+---------+
         |                              |                              |
         |   +--------------------------+                              |
         |   | CAN-FD 500kbps                                         |
         |   | Motor commands, BMS status, safety frames              |
         |   | Priority: safety > motor > BMS > telemetry            |
         |   +-------------------------------------------------------+
         |
         |   +-------------------------------------------------------+
         +---| UART 3Mbps (M7 <-> RK3566)                            |
             | Velocity commands, sensor data, configuration         |
             | Priority: velocity > sensor > config > debug          |
             +-------------------------------------------------------+
         
                           +-------------------------------------------------------+
                           | WiFi 802.11ac (RK3566 <-> Cloud)                     |
                           | Telemetry, OTA, remote control, logs                  |
                           | Priority: remote stop > OTA > telemetry > logs        |
                           +-------------------------------------------------------+
```

## Bus 1: CAN-FD (Motor + BMS)

### Physical Layer

```
Standard:   CAN-FD (ISO 11898-1:2015 + ISO 11898-2:2016)
Bit Rate:   500 kbps (arbitration phase)
            2 Mbps (data phase, CAN-FD)
Controller: STM32H747 FDCAN1 + FDCAN2
Transceiver: TJA1051GT/3 (3.3V compatible)
Termination: 120 ohm, enable via jumper for end-of-bus
Bus Length:  < 1m (internal robot bus)
```

### Message Format

```
Standard CAN 2.0B (29-bit extended ID for compatibility):
  Arbitration ID: 29-bit
  DLC: 0-8 bytes

CAN-FD:
  Arbitration ID: 29-bit
  DLC: 0-64 bytes
  BRS: 1 (bit rate switching enabled for data phase)
  ESI: 1 (error state indicator, set on node error passive)

ID Mapping (29-bit extended):
  Bit 28-24: Priority (0=highest, 31=lowest)
  Bit 23-16: Message Type
  Bit 15-8:  Source Node
  Bit 7-0:   Destination / Subtype
```

### Priority Assignment

| Priority | Type | Message Examples | Max Rate | DLC |
|----------|------|-----------------|----------|-----|
| 0 | Safety | Emergency stop, fault frame | On event | 8 |
| 1 | Motor cmd | Velocity setpoint, torque limit | 100 Hz | 8 |
| 2 | Motor status | Actual speed, current, position | 100 Hz | 8 |
| 3 | BMS status | SOC, voltage, current, temp | 10 Hz | 8 |
| 4 | System status | Health, mode, heartbeat | 10 Hz | 4 |
| 5 | Telemetry | Debug data, diagnostics | 1 Hz | 64 (FD) |
| 6 | Configuration | Parameter read/write | On request | 8 |

### CAN-FD Frame Definitions

#### Safety Frames (Priority 0)

```
ID: 0x00000001 - Emergency Stop (broadcast)
Data[0]: Reason code (0 = user stop, 1 = bumper, 2 = cliff, 3 = stall, 4 = overcurrent)
Data[1]: Reserved
Data[2-3]: Timestamp (ms)

ID: 0x00000002 - Fault Frame
Data[0]: Fault source (1=M7, 2=M4, 3=RK3566)
Data[1]: Fault code
Data[2-3]: Timestamp (ms)
Data[4-7]: Fault data (source-specific)
```

#### Motor Command Frames (Priority 1)

```
ID: 0x01000010 - Left Motor Velocity (M7 -> M4/DRV)
Data[0-3]: Setpoint velocity (float32, rad/s)
Data[4]: Control mode (0=velocity, 1=position, 2=torque)
Data[5]: Acceleration limit (0-255, 0=max)
Data[6-7]: Reserved

ID: 0x01000011 - Right Motor Velocity (M7 -> M4/DRV)
Data[0-3]: Setpoint velocity (float32, rad/s)
Data[4]: Control mode (0=velocity, 1=position, 2=torque)
Data[5]: Acceleration limit (0-255, 0=max)
Data[6-7]: Reserved
```

#### BMS Status Frames (Priority 3)

```
ID: 0x03000030 - BMS Status (M4 -> M7)
Data[0]: State of charge (%, 0-100)
Data[1-2]: Pack voltage (uint16, mV)
Data[3-4]: Pack current (int16, mA)
Data[5]: Fault flags
Data[6]: Cell balance status
Data[7]: Checksum (XOR of bytes 0-6)

ID: 0x03000031 - BMS Cell Voltages (M4 -> M7)
Data[0-1]: Cell 1 voltage (uint16, mV)
Data[2-3]: Cell 2 voltage (uint16, mV)
Data[4-5]: Cell 3 voltage (uint16, mV)
Data[6-7]: Cell 4 voltage (uint16, mV)
```

#### Heartbeat Frames

```
ID: 0x04000040 - M7 Heartbeat (100 Hz)
Data[0]: Sequence number (increments each cycle)
Data[1]: Mode (0=idle, 1=cleaning, 2=docking, 3=charging, 4=fault)
Data[2]: Error code (0=no error)
Data[3]: Checksum

ID: 0x04000041 - M4 Heartbeat (100 Hz)
Data[0]: Sequence number
Data[1]: Safety state (0=normal, 1=warning, 2=fault, 3=emergency)
Data[2]: Error code
Data[3]: Checksum
```

### CAN Bus QoS

```
Delivery Guarantee:
  - Safety frames: Ack-based, 3 retries, 10ms timeout
  - Motor command: Ack-based, 2 retries (last-is-best paradigm)
  - Status frames: No ack (loss tolerable at high rate)
  - Configuration: Ack-based, 3 retries, 100ms timeout

Bus Load Calculation:
  Avg frame rate: ~250 frames/sec (at 100 Hz motor + 100 Hz status + 10 Hz BMS + overhead)
  Avg frame length: ~120 bits (including overhead)
  Bus load at 500 kbps: 250 * 120 / 500,000 = 6%
  Peak load with all bursts: < 20%
  Headroom: 80% for fault bursts and CAN-FD data phase
```

## Bus 2: UART IPC (M7 to RK3566)

### Physical Layer

```
Standard:    TTL UART (3.3V)
Baud Rate:   3,000,000 bps (3 Mbps)
Data Bits:   8
Parity:      None
Stop Bits:   1
Flow Ctrl:   None (high rate makes flow control overhead unacceptable)
Buffer:      4 KB TX / 4 KB RX (DMA circular)
Pins:        M7 PA9(TX) -> RK3566 RX, M7 PA10(RX) <- RK3566 TX
```

### Frame Format

```
+------+--------+--------+--------+--------+--------+----------+--------+
| SYNC | LENGTH | TYPE   | SRC    | DST    | SEQ    | PAYLOAD  | CRC16  |
| 2B   | 2B     | 1B     | 1B     | 1B     | 2B     | 0-1024B  | 2B     |
+------+--------+--------+--------+--------+--------+----------+--------+

SYNC:     0xAA 0x55 (byte stuffing: if payload contains 0xAA, write 0xAA 0x00)
LENGTH:   Total frame length including header and CRC (little-endian)
TYPE:     Message type (see below)
SRC:      Source ID (1=M7, 2=M4, 3=RK3566)
DST:      Destination ID
SEQ:      Sequence number (increments per source, wraparound)
PAYLOAD:  Type-dependent data (big-endian for multi-byte values)
CRC16:    CCITT CRC-16 over header + payload, polynomial 0x1021
```

### Message Types

| Type | Name | Payload | Direction | Rate | Priority |
|------|------|---------|-----------|------|----------|
| 0x01 | VelocityCmd | vx(int16), vyaw(int16) | RK->M7 | 50 Hz | High |
| 0x02 | SensorData | imu[12B],odom[8B],lidar[2B] | M7->RK | 100 Hz | High |
| 0x03 | BatteryStatus | soc(uint8),volt(uint16),cur(int16) | M7->RK | 10 Hz | Medium |
| 0x04 | SystemState | mode(uint8),error(uint8) | M7->RK | 10 Hz | Medium |
| 0x05 | MapChunk | offset(uint32),data[varies] | RK->M7 | 1 Hz | Low |
| 0x06 | OTAChunk | offset(uint32),crc32(uint32),data[256] | RK->M7 | On update | Low |
| 0x07 | ConfigRead | param_id(uint16) | RK->M7 | On request | Low |
| 0x08 | ConfigWrite | param_id(uint16),value(int32) | RK->M7 | On request | Low |
| 0x09 | ConfigResp | param_id(uint16),value(int32) | M7->RK | Response | Low |
| 0x0A | DebugLog | text[N] (null-terminated) | M7->RK | 10 Hz | Lowest |
| 0x0B | ResetCmd | reason(uint8) | RK->M7 | On event | High |
| 0x0C | Ack | seq(uint16),status(uint8) | Both | Response | Medium |

### Priority Queue

```
The UART transmitter maintains 3 priority queues:

Priority 1 (High - interrupts lower queues):
  - VelocityCmd (0x01)
  - ResetCmd (0x0B)
  - SensorData (0x02) — only if buffer > 50% full

Priority 2 (Medium):
  - BatteryStatus (0x03)
  - SystemState (0x04)
  - Ack (0x0C)

Priority 3 (Low):
  - MapChunk (0x05)
  - OTAChunk (0x06)
  - ConfigRead/Write/Resp (0x07-0x09)
  - DebugLog (0x0A)

Transmission:
  - Always drain Priority 1 first
  - If Priority 1 empty, drain Priority 2
  - If Priority 1+2 empty, drain Priority 3
  - Max one frame per 330 us (3 Mbps / ~1000 bits per frame)
```

### CRC16 Protection

```c
/* CRC-16-CCITT, polynomial 0x1021, initial 0xFFFF */
uint16_t uart_crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}
```

### Error Handling

```
No ack on individual frames (rate too high for ack overhead):
  - At 50 Hz velocity commands, missing one frame is tolerable
  - Next frame overwrites previous command ("last is best")
  
Detection of broken link:
  - If no VelocityCmd received for 200 ms: M7 enters limp mode (decelerate to 0)
  - If no SensorData received for 500 ms: RK3566 pauses navigation
  - Both nodes send heartbeats every 100 ms

Recovery:
  - On CRC error: drop frame, increment error counter
  - On 10 consecutive CRC errors: reset UART DMA, re-sync
  - On 50 consecutive CRC errors: report fault, request re-init
```

## Bus 3: WiFi (RK3566 to Cloud)

### Physical Layer

```
Standard:  IEEE 802.11ac (WiFi 5)
Band:      2.4 GHz + 5 GHz
Module:    Realtek RTL8822CE (SDIO + Bluetooth 5.0)
Antenna:   PCB trace, 2 dBi gain
Data Rate: Up to 867 Mbps (MCS 9, 80 MHz, 2 streams)
Range:     > 30m indoor
Security:  WPA3-Enterprise (production) / WPA3-Personal (home)
```

### Connectivity Architecture

```
Robot (RK3566)                    Cloud
     |                              |
     |--- MQTT (TLS 1.3)  -------->|  AWS IoT Core / Mosquitto
     |    Port 8883                 |
     |    Keepalive: 60s            |
     |                              |
     |--- HTTPS (TLS 1.3) -------->|  REST API for config/status
     |    Port 443                  |
     |    Auth: Bearer token        |
     |                              |
     |<-- MQTT Subscribe ----------|  Commands, OTA, config
     |    Topics:                   |
     |      robot/<serial>/cmd     |
     |      robot/<serial>/config  |
     |      robot/<serial>/ota     |
     |                              |
     |--- WebSocket (TLS) -------->|  Real-time log streaming
     |    Port 443                  |
     |    Path: /ws/logs            |
     |                              |
```

### MQTT Topics

```
Publish (Robot -> Cloud):
  robot/<serial>/status        JSON, 10 Hz - System status
  robot/<serial>/telemetry     JSON, 1 Hz  - Extended telemetry
  robot/<serial>/sensor        JSON, 10 Hz - Sensor snapshot
  robot/<serial>/map           Binary, on change - Occupancy grid
  robot/<serial>/log           JSON, 0.1 Hz - Log messages
  robot/<serial>/error         JSON, on event - Error reports

Subscribe (Robot <- Cloud):
  robot/<serial>/cmd           JSON - Remote control commands
  robot/<serial>/config        JSON - Configuration changes
  robot/<serial>/ota           JSON - OTA update trigger
  robot/<serial>/dock          JSON - Dock commands
  robot/<serial>/reset         JSON - Reset commands

QoS Levels:
  /status, /telemetry:     QoS 0 (fire and forget, high rate)
  /cmd, /config, /ota:     QoS 1 (at least once, reliable)
  /error, /dock:           QoS 1 (at least once)
```

### Data Prioritization

```
Network Priority (from RK3566 traffic shaper):

Priority 1 (Real-time, < 50ms latency):
  - Remote emergency stop commands
  - Safety-related MQTT messages
  Throttle: Always permitted

Priority 2 (Interactive, < 200ms latency):
  - Velocity commands from app joystick
  - Configuration changes
  - Dock commands
  Throttle: Always permitted

Priority 3 (Bulk data):
  - Telemetry upload
  - Map data upload
  - Log streaming
  Throttle: Limit to 1 Mbps when on 2.4 GHz
            Limit to 5 Mbps when on 5 GHz

Priority 4 (Background):
  - OTA download
  - Debug log streaming
  Throttle: Limit to 500 kbps when cleaning
            Full bandwidth when docked
```

## Cross-Bus Priority Summary

```
System-wide Message Priority (highest to lowest):

  1. CAN-FD Safety frames (Emergency stop, fault)
  2. CAN-FD Motor commands (velocity setpoints)
  3. CAN-FD Motor status (actual speed, current)
  4. UART Velocity commands (from RK3566 to M7)
  5. UART Sensor data (from M7 to RK3566)
  6. CAN-FD BMS status (SOC, voltage)
  7. CAN-FD System heartbeat
  8. UART Battery / System status
  9. WiFi Remote commands (MQTT)
  10. WiFi Telemetry
  11. CAN-FD Telemetry / Debug
  12. UART Config / Map / OTA
  13. UART Debug log
  14. WiFi OTA download
  15. WiFi Log streaming
```

## Diagnostic and Monitoring

Each bus maintains diagnostic counters accessible via the system manager:

```c
typedef struct {
    uint32_t tx_frames;
    uint32_t rx_frames;
    uint32_t tx_errors;
    uint32_t rx_errors;
    uint32_t crc_errors;
    uint32_t bus_off_count;
    uint32_t last_bus_off_time;
    uint8_t  bus_load_percent;
    uint32_t overflow_count;
    uint32_t timeout_count;
} bus_diagnostics_t;
```

Diagnostics are reported at 1 Hz via the system status interface and can be viewed through the serial console or mobile app for troubleshooting.
