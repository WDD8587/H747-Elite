# OTA Subsystem Specification

## 1. End-to-End Flow

```
+-------------+     +----------+     +-----------+     +------------+
| Build Server|---->| OTA Cloud|---->| RK3566    |---->| H747 (M7) |
| (signed FW) |     | (storage)|     | (manager) |     |  + HSEM   |
+-------------+     +----------+     +-----------+     +----+-------+
                                          |                   |
                                     [eMMC staging]    [UART IPC]
                                          |                   |
                                     [backup part]      [M4 flash]
                                          |                   |
                                     [set boot flag]    [verify CRC]
                                          |                   |
                                     [reboot]           [reboot]
```

### 1.1 Normal Update Sequence

1. **Poll**: RK3566 queries OTA cloud for available firmware version (HTTPS GET /api/ota/check).
2. **Manifest**: Cloud returns JSON manifest: version, size, SHA256, download URL, delta URL, target.
3. **Download**: RK3566 downloads full or delta firmware to `/data/ota/staging.bin`.
   - If delta upgrade: download delta patch, apply to current firmware, verify SHA256.
4. **Verify**: Compute SHA256 of staged image, compare against manifest. Verify ECDSA P-256 signature against embedded public key in write-protected flash.
5. **Flash**:
   - If target is RK3566: write to inactive A/B partition (`mmcblk0p5` or `mmcblk0p6`).
   - If target is H747: send over UART IPC to M7 core, which relays via HSEM to M4 core for flash programming.
6. **Post-verify**: Read back flashed partition, compute CRC32, compare against staged CRC.
7. **Set boot flag**: Mark new slot as active in A/B metadata.
8. **Reboot**: System reboots into new firmware.

### 1.2 Rollback on Failure

- If new firmware fails to boot (WDT timeout, assert, or explicit boot fail), the bootloader increments the fail counter in A/B metadata.
- After 3 consecutive failures, the slot is marked BAD and the system switches to the other slot.
- Cloud notification is sent: `{"event": "rollback", "from_slot": "B", "to_slot": "A", "reason": "3_boot_failures"}`.

---

## 2. Security Model

### 2.1 Key Management

| Component | Detail |
|-----------|--------|
| Signing key | ECDSA P-256 private key stored on air-gapped build server. Never transmitted over network. Hardware security module (HSM) recommended. |
| Public key | Embedded as compile-time constant in bootloader, stored in **write-protected flash sector** (`.rodata.secure` section). |
| Key rotation | Public key can only be updated via physical JTAG/SWD programming with authentication. |

### 2.2 Signing Process

1. Build server compiles firmware binary.
2. Compute SHA256 of payload.
3. Sign SHA256 hash with ECDSA P-256 private key → 64-byte signature (r || s).
4. Prepend header: magic(4B) + version(4B) + size(4B) + signature(64B) = 76B header.
5. Append payload after header. Result is complete signed firmware image.

### 2.3 Verification (Bootloader)

1. Read magic bytes → must equal `"FW01"`.
2. Read payload size → must be sane (0 < size < 16MB).
3. Compute SHA256 of payload.
4. Verify ECDSA P-256 signature over hash using embedded public key.
5. **Constant-time comparison** of signature to prevent timing attacks.
6. **Double verification**: Verify twice to detect fault injection.

### 2.4 Anti-Brick Protection

- **Golden image** in write-protected bootloader flash sector.
- Bootloader checks at startup:
  1. Valid magic (`0xDEADBEEF`) at application entry point.
  2. Stack pointer within valid RAM range (`0x20000000` - `0x20040000`).
  3. CRC32 of vector table matches stored CRC at offset 0x40.
- If any check fails → load golden image → notify cloud.
- Golden image updated only after successful verified OTA.

---

## 3. Rollback Protection

- Firmware header contains a **monotonic version counter** (uint32_t).
- Version counter is stored in **OTP (one-time programmable) memory**.
- Bootloader checks: `new_version > stored_version`. If not, reject update.
- Version counter in OTP can only increase (hardware enforced).
- Prevents downgrade attacks to known-vulnerable firmware versions.

---

## 4. Bandwidth & Performance

| Metric | Value |
|--------|-------|
| Full firmware size | ~4 MB |
| Delta patch size (typical) | ~500 KB (12.5% of full) |
| Download time (full, WiFi) | ~10 seconds (at 4 Mbps) |
| Download time (delta, WiFi) | ~1.5 seconds |
| Download time (full, cellular) | ~60 seconds (at 500 Kbps) |
| Verification time (SHA256) | < 500 ms (hardware accelerated) |
| Signature verification | < 100 ms |
| Flash write (eMMC) | < 2 seconds |
| Total OTA time (delta) | ~5-10 seconds |
| Total OTA time (full) | ~15-30 seconds |

---

## 5. Resume Transfer

| Parameter | Value |
|-----------|-------|
| Block size | 4 KB |
| Bitmap granularity | 1 bit per block |
| Max resume attempts | 5 |
| Block verification | SHA256 per block |
| Corruption recovery | Request re-download of individual block |

After 5 resume attempts, a full re-download is forced.

---

## 6. A/B Partition Scheme

| Slot | Device | Size | Purpose |
|------|--------|------|---------|
| Boot | `mmcblk0p1` | 4 MB | U-Boot / bootloader |
| Slot A | `mmcblk0p5` | 32 MB | Active firmware |
| Slot B | `mmcblk0p6` | 32 MB | Standby firmware |
| Metadata | `mmcblk0p7` | 1 MB | A/B metadata, golden image ref |
| Data | `mmcblk0p8` | (remaining) | User data, logs, config |

### Metadata Contents

- Active slot (0=A, 1=B)
- Boot count for active slot
- Fail count (consecutive)
- Last known good slot + timestamp
- Total boot attempts per slot
- CRC32 of metadata block

---

## 7. Recovery

### 7.1 Golden Image Fallback

Location: First 128 KB of bootloader flash (write-protected).

Triggers:
- Application magic invalid
- Stack pointer out of range
- Vector table CRC mismatch
- Signature verification failure (after retries)

### 7.2 Factory Reset

Holding a physical button during boot forces:
- Switch to last known good slot
- Clear A/B fail counters
- If LKG also fails → load golden image

---

## 8. Cloud API

### Check for update

```
GET /api/ota/check
Headers: X-Device-ID: <serial>
         X-FW-Version: <current>
Response 200:
{
  "update_available": true,
  "version": "2.1.0",
  "size": 4194304,
  "sha256": "ab47...",
  "signature": "3045...",
  "delta_url": "https://ota.example.com/fw/2.1.0.delta",
  "full_url": "https://ota.example.com/fw/2.1.0.bin",
  "target": "rk3566",
  "mandatory": false,
  "changelog": "Bug fixes, performance improvements"
}
```

### Report status

```
POST /api/ota/status
{
  "device_id": "...",
  "version": "2.1.0",
  "state": "flashing|complete|error|rollback",
  "progress_pct": 65,
  "error_code": "SHA256_MISMATCH"
}
```

---

## 9. Components

| File | Purpose |
|------|---------|
| `ota_manager.c` | OTA orchestrator, state machine, pipeline |
| `diff_upgrade.c` | bsdiff compute/apply for delta updates |
| `verify_sign.c` | ECDSA P-256 signature verification |
| `ab_partition.c` | A/B boot slot management |
| `resume_transfer.c` | Resumable download with bitmap |
| `anti_brick.c` | Safety checks and golden image fallback |
