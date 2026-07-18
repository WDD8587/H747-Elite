# OTA System Design

## Overview

The H747 Elite supports over-the-air firmware updates with A/B partitioning for safe rollback. The system uses bsdiff for delta generation, MQTT for transport, and a boot counter mechanism for automatic rollback detection.

## Partition Layout

### Flash Memory Map (STM32H747, 2MB Flash)

```
+-----------------------+----------+--------+----------------------------------+
| Region                | Address  | Size   | Description                      |
+-----------------------+----------+--------+----------------------------------+
| Bootloader            | 0x08000000 | 64 KB | Primary bootloader               |
| Slot A (M7)           | 0x08010000 | 512 KB| Active M7 firmware slot          |
| Slot B (M7)           | 0x08090000 | 512 KB| Standby M7 firmware slot         |
| Shared Config         | 0x08110000 | 16 KB | Boot config, counters, flags     |
| M4 Firmware           | 0x08114000 | 384 KB| M4 firmware (single slot)        |
| System Storage        | 0x08174000 | 64 KB | Logs, calibration, persistent    |
| Unused                | 0x08184000 | 184 KB| Reserved                         |
+-----------------------+----------+--------+----------------------------------+
```

### Partition Sizing Rationale

```
Total Flash:       2,048 KB
Bootloader:         64 KB  (3.1%)
M7 Slot A:         512 KB  (25.0%)
M7 Slot B:         512 KB  (25.0%)
Shared Config:      16 KB  (0.8%)
M4 Firmware:       384 KB  (18.8%)
System Storage:     64 KB  (3.1%)
Unused:            184 KB  (9.0%)
Reserved (future): 200 KB  (14.8%)
```

## Boot Flow

```
Power On / Reset
      |
      v
+-----------+
| Bootloader|  <-- Executes from 0x08000000
+-----+-----+
      |
      | Read shared config (0x08110000)
      | - active_slot (A or B)
      | - boot_count
      | - max_boot_attempts (default: 3)
      | - firmware_status (VALID / ATTEMPTING / FAILED)
      |
      v
+-------------------+
| Check Boot Count  |
+-------+-----------+
        |
   +----+----+
   |         |
   v         v
  <= 3      > 3
   |         |
   |         v
   |  +----------------+
   |  | Mark slot FAILED|
   |  | Switch to other |
   |  | slot, reset cnt |
   |  +-------+--------+
   |          |
   +----+-----+
        |
        v
+----------------------+
| Load M7 from         |
| active_slot          |
+----------------------+
        |
        v
+----------------------+
| M7 Boot: increment   |
| boot_count           |
+----------------------+
        |
        v
+----------------------+
| M7 Application       |
| - Initialize system  |
| - Start FreeRTOS     |
| - Decrement boot     |
|   counter on success |
| - If counter > 0     |
|   after 5 min:       |
|   mark VALID, reset  |
|   counter            |
+----------------------+
```

### Boot Counter Management

```
Boot Counter Implementation (16-bit, in shared config):
  - Initial value: 0
  - On boot: incremented by 1
  - On successful app launch (5 min uptime): reset to 0, mark slot VALID
  - On boot to ATTEMPTING state: max_boot_attempts = 3

Rollback Trigger:
  - If boot_count > max_boot_attempts (3):
    - Mark current slot as FAILED
    - Switch active_slot to other slot
    - Reset boot_count to 0
    - Reset watchdog and reboot
```

## Update Process

### Overview

```
Build Server                  Robot (RK3566)              Robot (STM32H747)
     |                             |                            |
     |-- Generate delta ---------->|                            |
     |   (bsdiff old->new)         |                            |
     |                             |                            |
     |-- Transmit via MQTT ------->|                            |
     |   (4KB chunks, 5s interval) |                            |
     |                             |                            |
     |                   +---------v--------+                   |
     |                   | Receive chunks   |                   |
     |                   | Verify SHA256    |                   |
     |                   | Store to eMMC   |                   |
     |                   +---------+--------+                   |
     |                             |                            |
     |                   +---------v--------+                   |
     |                   | All chunks OK?   |                   |
     |                   +---------+--------+                   |
     |                             |                            |
     |                   +---------v--------+                   |
     |                   | Apply bspatch    |                   |
     |                   | to reconstruct   |                   |
     |                   | full image       |                   |
     |                   +---------+--------+                   |
     |                             |                            |
     |                   +---------v--------+                   |
     |                   | Send image via   |                   |
     |                   | UART IPC to M7   |                   |
     |                   | (3Mbps)          |                   |
     |                   +---------+--------+                   |
     |                             |                            |
     |                             |  +----------------------+  |
     |                             |  | Receive from RK3566  |  |
     |                             |  | Write to inactive    |  |
     |                             |  | slot via IAP         |  |
     |                             |  +----------+-----------+  |
     |                             |             |              |
     |                             |  +----------v-----------+  |
     |                             |  | Verify full image    |  |
     |                             |  | SHA256 + CRC32       |  |
     |                             |  +----------+-----------+  |
     |                             |             |              |
     |                   +---------v------------v+              |
     |                   | Update shared config   |             |
     |                   | Set active_slot = B    |             |
     |                   | Set firmware_status =  |             |
     |                   |   ATTEMPTING           |             |
     |                   +---------+--------------+             |
     |                             |                            |
     |                   +---------v--------+                   |
     |                   | Reboot system    |                   |
     |                   +------------------+                   |
```

### Delta Generation (Build Server)

```
Tool: bsdiff
Input: old_firmware.bin + new_firmware.bin
Output: delta.bin (typically 10-30% of full image size)

Typical Delta Sizes:
  - Patch release (minor bug fix):   10-50 KB  (2-10% of 512KB)
  - Minor release (new features):    50-150 KB (10-30%)
  - Major release (full rewrite):    200-400 KB (40-80%)
  
Delta Generation Command:
  bsdiff old_firmware.bin new_firmware.bin delta.bin

Delta Application (on robot):
  bspatch old_firmware.bin new_firmware.bin delta.bin
```

### MQTT Transport

```
Topic:     robot/<serial>/ota/update
QoS:       1 (at least once delivery)
Retain:    false
Frequency: Chunk every 5 seconds

Chunk Format (JSON):
{
  "serial": "H747-2025-00042",
  "update_id": "upd_20250115_001",
  "chunk_index": 47,
  "chunk_total": 128,
  "data": "<base64 encoded, 4KB max>",
  "sha256_chunk": "a1b2c3d4...",
  "sha256_full": "deadbeef..."
}

Transport Security:
  - MQTT over TLS 1.3
  - Client certificate authentication
  - Payload encrypted with per-robot key (AES-256-GCM)
```

### Verification

#### Per-Chunk Verification

```c
// On RK3566, after receiving each MQTT chunk:
bool verify_chunk(const uint8_t* data, size_t len, const char* expected_sha) {
    uint8_t hash[32];
    mbedtls_sha256_ret(data, len, hash, 0);  // 0 = SHA-256
    char hex[65];
    for (int i = 0; i < 32; i++) {
        sprintf(hex + i*2, "%02x", hash[i]);
    }
    hex[64] = '\0';
    return strcmp(hex, expected_sha) == 0;
}
```

#### Full Image Verification

```c
// After applying bspatch and before writing to flash:
bool verify_full_image(const uint8_t* image, size_t len, const char* expected_sha) {
    // Verify SHA256
    uint8_t hash[32];
    mbedtls_sha256_ret(image, len, hash, 0);
    
    // Verify CRC32 (embedded in image header at offset 8)
    uint32_t expected_crc;
    memcpy(&expected_crc, image + 8, 4);
    uint32_t actual_crc = crc32(image + 12, len - 12);
    
    return memcmp(hash, expected_sha, 32) == 0 && actual_crc == expected_crc;
}
```

## M7 IAP (In-Application Programming)

### Flash Driver

```c
/* IAP driver configuration */
#define IAP_FLASH_BASE       0x08000000
#define IAP_SECTOR_SIZE       (128 * 1024)  // 128 KB per sector
#define IAP_SLOT_A_START     0x08010000
#define IAP_SLOT_A_SECTORS   4             // 4 sectors = 512 KB
#define IAP_SLOT_B_START     0x08090000
#define IAP_SLOT_B_SECTORS   4             // 4 sectors = 512 KB
#define IAP_FLASH_TIMEOUT    1000          // ms

/* Flash programming sequence */
int iap_write_slot(uint32_t slot_start, const uint8_t* data, size_t len) {
    // 1. Unlock FLASH controller
    HAL_FLASH_Unlock();
    
    // 2. Erase existing sectors (4 sectors = 512KB)
    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Sector = get_sector_from_address(slot_start);
    erase.NbSectors = 4;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    
    uint32_t sector_error;
    HAL_FLASHEx_Erase(&erase, &sector_error);
    
    // 3. Program flash word-by-word (256 bits = 32 bytes per write)
    for (size_t offset = 0; offset < len; offset += 32) {
        uint32_t address = slot_start + offset;
        uint32_t words[8];  // 8 x 32-bit = 256-bit
        memcpy(words, data + offset, 32);
        
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, address,
                              (uint32_t)words) != HAL_OK) {
            HAL_FLASH_Lock();
            return -1;
        }
    }
    
    // 4. Lock FLASH controller
    HAL_FLASH_Lock();
    return 0;
}
```

### Slot Selection Logic

```c
/* Determine which slot to write to and which to boot from */
typedef enum {
    SLOT_A = 0,
    SLOT_B = 1
} slot_t;

slot_t get_inactive_slot(void) {
    slot_t active = get_active_slot();
    return (active == SLOT_A) ? SLOT_B : SLOT_A;
}

slot_t get_active_slot(void) {
    /* Read from shared config at 0x08110000 */
    shared_config_t* cfg = (shared_config_t*)SHARED_CONFIG_ADDR;
    return cfg->active_slot;
}

void set_active_slot(slot_t slot) {
    /* Write to shared config (backup SRAM or last flash sector) */
    shared_config_t cfg;
    memcpy(&cfg, (void*)SHARED_CONFIG_ADDR, sizeof(cfg));
    cfg->active_slot = slot;
    cfg->boot_count = 0;
    cfg->firmware_status = STATUS_ATTEMPTING;
    
    /* Write back to shared config area */
    write_shared_config(&cfg);
}
```

## Security

### Image Signing

```
Signing Algorithm: ECDSA secp256r1 (P-256)
Hash Algorithm:    SHA-256
Key Size:          256 bits
Signature Size:    64 bytes (R + S)

Image Header Structure:
  Offset  Size  Field
  0       4     Magic (0x484F5441 = "HOTA")
  4       4     Image size (bytes)
  8       4     CRC32 of payload
  12      64    ECDSA signature (R || S)
  76      32    SHA-256 of payload
  108     4     Version major
  112     4     Version minor
  116     4     Version patch
  120     N     Payload (firmware binary)
```

### Key Management

```
Signing Key (Build Server):
  - Stored as GitHub secret: SIGNING_KEY
  - Format: PEM-encoded EC private key
  - Used only in release workflow

Verification Key (Bootloader):
  - Embedded in bootloader flash (read-only)
  - Public key only (cannot derive private key)
  - Hash of public key printed during boot for verification

Per-Robot Key (MQTT Encryption):
  - Unique AES-256-GCM key per robot
  - Stored in secure element (if available) or encrypted flash
  - Derived from device serial + HUK (Hardware Unique Key)
```

## Rollback Strategy

### Automatic Rollback (On Boot Failure)

```
Sequence:
  1. Bootloader loads Slot A (active), sets boot_count = 1
  2. Application starts, runs for 3 minutes, crashes
  3. Watchdog resets the system
  4. Bootloader loads Slot A (still active), increments boot_count = 2
  5. Application starts, runs for 30 seconds, crashes
  6. Watchdog resets the system
  7. Bootloader loads Slot A, increments boot_count = 3
  8. Application starts, immediately crashes
  9. Watchdog resets the system
  10. Bootloader loads Slot A, increments boot_count = 4
  11. boot_count(4) > max_boot_attempts(3)
  12. Mark Slot A as FAILED
  13. Switch active_slot to B
  14. Reset boot_count = 0
  15. Load Slot B (known good)
  16. Robot recovers on previous firmware
```

### Manual Rollback (User Initiated)

```
Methods:
  1. Mobile app: "Rollback firmware" button
     - Sends MQTT command: robot/<serial>/ota/rollback
     - Robot sets active_slot to other slot, reboots
     
  2. Serial console:
     > ota rollback
     Switching to slot B. Rebooting in 3 seconds...
     
  3. Button sequence (no connectivity):
     - Hold power button + bumper for 5 seconds during boot
     - Bootloader detects button combo, switches slot
```

## Testing and Validation

| Test | Description | Pass Criteria |
|------|-------------|---------------|
| Full OTA update | Push v1.0 -> v1.1 over MQTT | Update completes, robot runs v1.1 |
| Delta OTA update | Push patch delta | Delta applied, SHA256 matches |
| Rollback on crash | Flash corrupted firmware | Robot auto-rollbacks to previous slot |
| Rollback on command | Send rollback MQTT message | Robot switches slot within 10 seconds |
| Power loss during update | Cut power mid-write | Slot remains consistent, robot boots from other slot |
| Concurrent update | Update while robot cleaning | Robot finishes cleaning, then applies update |
| Multiple updates | v1.0 -> v1.1 -> v1.2 -> v1.1 | All transitions succeed |
| Chunk corruption | Inject corrupted MQTT chunk | Robot rejects chunk, requests retransmit |
| Signature failure | Flash image with invalid signature | Bootloader rejects image, stays on current slot |
| Storage pressure | Fill eMMC to 95% before update | Update succeeds, temp files cleaned up |
