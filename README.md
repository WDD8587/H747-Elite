# H747 Elite

STM32H747 + RK3566 triple-core vacuum robot firmware.

## Architecture

```
STM32H747 CM7 (480MHz)    STM32H747 CM4 (240MHz)    RK3566 (Quad A55)
├─ FOC motor control      ├─ BMS battery            ├─ SLAM
├─ IPC bridge             ├─ Safety watchdog        ├─ DWA planner
├─ IMU prep               ├─ ToF VL53L5CX           ├─ YOLO vision
└─ Bootloader             ├─ Dock handshake         ├─ Cloud MQTT
                          └─ Factory calibration    └─ OTA A/B
       │                          │                       │
       └── HSEM+SRAM3 (<1μs) ────┘                       │
                                                          │
       ──────────── SPI 20MHz / USB CDC / UART ──────────┘
```

## Directory Structure

| Directory | Description |
|-----------|-------------|
| `l1_m7/`  | M7 firmware: FOC, IPC bridge, HAL config, startup |
| `l1_m4/`  | M4 firmware: BMS, safety, ToF, dock, factory cal |
| `l2_rk3566/` | RK3566 Linux: SLAM, cloud, OTA, comm protocols |
| `shared/` | Shared IPC protocol, CRC16, transport layer (UART/SPI/USB) |
| `tests/`  | 58 unit tests (Unity framework, PC-hosted) |
| `firmware/` | CMake build system + linker scripts + HAL stubs |
| `config/` | Robot parameters (YAML) |
| `.github/` | CI/CD: PR checks, nightly builds, e2e tests |

## Build

### Unit Tests (PC)
```bash
cd tests
.\run_tests.ps1          # Windows PowerShell
make test                 # Linux
```

### Firmware Cross-Compile (ARM)
```bash
cd firmware
cmake -B build/m7 -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain_arm_none_eabi.cmake -DTARGET_CORE=M7
cmake --build build/m7 -j$(nproc)
```

## Tests
58 tests: 58 passed, 0 failed  
Modules: FOC Math, BMS SMBus, BMS Charge, Safety Watchdog, OTA Diff, SPI IPC, USB CDC
