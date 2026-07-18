# IPC Protocol Specification

## 1. Overview

The Inter-Processor Communication (IPC) protocol governs message exchange between the M4 (real-time control) and M7 (application/vision) cores on the STM32H747, as well as between the M7 and the RK3566 (Linux application processor). The protocol is designed for deterministic, low-latency communication with built-in error detection.

### 1.1 Communication Channels

| Channel | Link | Speed | Priority |
|---------|------|-------|----------|
| M4 -> M7 | HSEM + Mailbox | 100 Mbps (via AXI-SRAM) | High |
| M7 -> M4 | HSEM + Mailbox | 100 Mbps | High |
| M7 -> RK3566 | SPI (quad) | 40 Mbps | Medium |
| RK3566 -> M7 | SPI (quad) | 40 Mbps | Medium |

---

## 2. Frame Format

All IPC frames share a common 8-byte header. Payload size varies by message type.

### 2.1 Header Structure (8 bytes)

```
Byte 0:    [ Start Flag (0xAA) ]
Byte 1:    [ Version:4 | Priority:4 ]
Byte 2:    [ Message Type ]
Byte 3:    [ Flags:3 | Sequence Number[4:0]:5 ]
Byte 4-5:  [ Payload Length (big-endian uint16) ]
Byte 6:    [ Source Address ]
Byte 7:    [ Destination Address ]
```

#### Field Descriptions

- **Start Flag** (0xAA): Frame synchronization byte.
- **Version** (bits 7-4 of byte 1): Protocol version. Current version = 0x1.
- **Priority** (bits 3-0 of byte 1): 0 = Low, 1 = Normal, 2 = High, 3 = Critical.
- **Message Type**: See Section 2.3.
- **Flags** (bits 7-5 of byte 3): Bit 7 = ACK requested, Bit 6 = ACK, Bit 5 = Retransmit.
- **Sequence Number** (bits 4-0 of byte 3): 5-bit rolling sequence number (0-31). Wraps around.
- **Payload Length**: Length of payload in bytes. 0 = no payload. Max 1024.
- **Source Address**: 0 = M4, 1 = M7, 2 = RK3566, 3 = CAN bus, 4 = BMS.
- **Destination Address**: Same encoding as source.

### 2.2 CRC Trailer (2 bytes)

```
Byte N:     [ CRC-16 MSB ]
Byte N+1:   [ CRC-16 LSB ]
```

CRC-16-CCITT polynomial: 0x1021 (x^16 + x^12 + x^5 + 1). Initial value: 0xFFFF. Covers header + payload. No remainder on valid frame.

### 2.3 Message Types

| Type | Value | Direction | Description |
|------|-------|-----------|-------------|
| NMT_CMD | 0x01 | Any -> Any | Network management command |
| NMT_ACK | 0x02 | Any -> Any | NMT acknowledgment |
| PDO_DATA | 0x10 | M4 -> M7 | Process data object (motor states) |
| PDO_CMD | 0x11 | M7 -> M4 | Process data command (setpoints) |
| SDO_REQ | 0x20 | Any -> Any | Service data object request |
| SDO_RES | 0x21 | Any -> Any | Service data object response |
| HEARTBEAT | 0x30 | Any -> Any | Heartbeat / liveness |
| EVENT | 0x40 | M7 -> M4 | Event notification |
| FAULT | 0x50 | M4 -> M7 | Fault/alarm notification |
| LOG | 0x60 | M4 -> Any | Log message |
| OTA_REQ | 0x70 | RK3566 -> M7 | OTA update request |
| OTA_DATA | 0x71 | RK3566 -> M7 | OTA firmware chunk |
| OTA_ACK | 0x72 | M7 -> RK3566 | OTA chunk acknowledgment |
| CONFIG | 0x80 | Any -> Any | Configuration parameter |

---

## 3. Timing Requirements

### 3.1 Deadline by Priority

| Priority | Max Latency | Guarantee |
|----------|-------------|-----------|
| Critical | 100 us | Preempts all other traffic |
| High | 500 us | Preempts Normal/Low |
| Normal | 2 ms | Best effort within budget |
| Low | 10 ms | May be queued |

### 3.2 Timeout Values

| Parameter | Value | Notes |
|-----------|-------|-------|
| ACK timeout | 5 ms | Wait for acknowledgment before retry |
| Retransmit interval | 10 ms | Time between retransmissions |
| Max retransmit count | 3 | Frames exhausted; error reported |
| Heartbeat interval | 100 ms | Normal operation |
| Heartbeat timeout | 500 ms | After 5 consecutive missed heartbeats |

---

## 4. Error Handling

### 4.1 Frame Validation

Receivers MUST validate every frame in the following order:

1. Start flag check (byte 0 must be 0xAA)
2. Version check (must match negotiated version)
3. CRC-16 check (entire header + payload)
4. Sequence number check (for duplicate/out-of-order detection)
5. Length bounds check (0 <= length <= 1024)

If any check fails, the frame is silently discarded. The receiver MUST NOT send NAK for invalid frames.

### 4.2 Retransmission

The sender retransmits if ACK is not received within `ACK timeout`. The retransmitted frame carries the same sequence number with the Retransmit flag set.

### 4.3 Sequence Number Rules

- Sender increments sequence number for each new unacknowledged frame.
- Sequence numbers wrap from 31 to 0.
- Receiver maintains a 5-entry sliding window for reordering (max window = 3 frames).
- Frames with sequence number <= last delivered are treated as duplicates and discarded.
- Frames with sequence number > expected by more than 3 are treated as gap.

### 4.4 Error Reporting

Errors are reported via the FAULT message type (0x50):

| Fault Code | Description |
|------------|-------------|
| 0x01 | CRC mismatch |
| 0x02 | Sequence number gap > 3 |
| 0x03 | Retransmit exhausted |
| 0x04 | ACK timeout |
| 0x05 | Frame too long |
| 0x06 | Unknown message type |

---

## 5. Priority Levels

| Level | Value | Used For |
|-------|-------|----------|
| Critical | 3 | Emergency stop, fault propagation, overcurrent |
| High | 2 | Motor setpoints, encoder data, PID outputs |
| Normal | 1 | Sensor data (IMU, ToF), heartbeat, PDO |
| Low | 0 | Configuration, log messages, OTA bulk data |

Priority is enforced by the mailbox scheduler. A frame with higher priority always preempts a lower-priority frame in the transmit queue.

---

## 6. Shared Memory Protocol (AXI-SRAM)

For M4-M7 communication via HSEM, the frame is written to a lock-free ring buffer in AXI-SRAM.

### 6.1 Ring Buffer Structure

```
Offset 0:    [ Write Index (uint32) ]
Offset 4:    [ Read Index (uint32) ]
Offset 8:    [ Buffer Size (uint32) ]
Offset 16:   [ Frame Data ... ]
```

### 6.2 Access Protocol

1. Writer claims HSEM semaphore (spin up to 100 us)
2. Writer checks if space available ((write_idx + 1) % size != read_idx)
3. Writer writes frame at write_idx
4. Writer increments write_idx: write_idx = (write_idx + 1) % size
5. Writer releases HSEM
6. Reader claims HSEM, reads, releases

---

## 7. IPC Frame Byte Layout Diagram

```
+--------+--------+--------+--------+--------+--------+--------+--------+
| 0xAA   | V:4|P:4|  Type  | F:3|Seq:5|  Length MSB  |  Length LSB  |
+--------+--------+--------+--------+--------+--------+--------+--------+
|  Src   |  Dst   |  Payload Byte 0  |  Payload Byte 1  |  ...        |
+--------+--------+--------+--------+--------+--------+--------+--------+
|  ... Payload ...                                        | CRC MSB|CRC LSB|
+--------+--------+--------+--------+--------+--------+--------+--------+
```

Total overhead: 10 bytes (8 header + 2 CRC). Maximum total frame: 1034 bytes.
