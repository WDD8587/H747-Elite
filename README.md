# H747 Elite

STM32H747 + RK3566 triple-core vacuum robot firmware.  
Local tests: **58/58 PASS** | CI: **GitHub Actions** | RTOS: **FreeRTOS** | Compiler: **ARM GCC 13.3**

## Architecture

```
STM32H747 CM7 (480MHz)    STM32H747 CM4 (240MHz)    RK3566 (Quad A55)
├─ FOC motor control      ├─ BMS battery            ├─ SLAM
├─ IPC bridge             ├─ Safety watchdog        ├─ DWA planner
├─ IMU prep               ├─ ToF VL53L5CX           ├─ YOLO vision
└─ Bootloader             ├─ Dock handshake         ├─ Cloud MQTT
       │                       │                         │
       └── HSEM+SRAM3 (<1μs) ──┘                         │
                                                          │
       ──────────── SPI 20MHz / USB CDC / UART ──────────┘
```

## Directory Structure

| Directory | Description |
|-----------|-------------|
| `l1_m7/`  | M7 firmware: FOC, IPC bridge, HAL config, startup |
| `l1_m4/`  | M4 firmware: BMS, safety, ToF, dock, factory cal |
| `l2_rk3566/` | RK3566 Linux: SLAM, cloud, OTA, comm protocols |
| `shared/` | IPC: protocol (CRC16), transport layer (UART/SPI/USB abstraction) |
| `tests/`  | 58 unit tests (Unity framework, PC-hosted, 1s) |
| `firmware/` | CMake build system + linker scripts + CMSIS/HAL |
| `config/` | Robot parameters (YAML) |
| `simulation/` | MATLAB FOC simulation (PMSM, Clarke/Park, SVPWM) |
| `.github/` | CI/CD: build M7/M4/RK3566, test, static analysis |

## Build

### Prerequisites

- **ARM GCC 13.3**: `D:\ARM_GCC\13.3 rel1\bin\arm-none-eabi-gcc.exe`
- **Ninja**: `winget install Ninja-build.Ninja`
- **Git submodules**: `git submodule update --init --recursive`

Submodules: `firmware/FreeRTOS-Kernel` (v11.x), `firmware/STM32CubeH7`

### Unit Tests (PC, 1 second)

```powershell
cd tests
.\run_tests.ps1              # Windows: build + run (58 tests)
```

```bash
cd tests && make test         # Linux
```

### Firmware Cross-Compile (ARM)

```powershell
$env:PATH = "D:\ARM_GCC\13.3 rel1\bin;" + $env:PATH

cd firmware

# M7
cmake -B build/m7 -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain_arm_none_eabi.cmake -DTARGET_CORE=M7
cmake --build build/m7
# → build/m7/firmware.bin, firmware.hex, firmware.elf

# M4
cmake -B build/m4 -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain_arm_none_eabi.cmake -DTARGET_CORE=M4
cmake --build build/m4
```

### MATLAB FOC Simulation

```powershell
cd simulation\matlab
& "C:\Users\EC\AppData\Local\Programs\GNU Octave\Octave-11.3.0\mingw64\bin\octave-cli.exe" --eval foc_sim
```

## CI/CD

Push to main triggers GitHub Actions:

| Job | What it does |
|-----|-------------|
| Build M7 | `arm-none-eabi-gcc` cross-compile, FreeRTOS + CMSIS + FOC |
| Build M4 | `arm-none-eabi-gcc` cross-compile, BMS + safety |
| Build RK3566 | `aarch64-linux-gnu-gcc` cross-compile |
| Unit Tests | `make test` — 58/58 PASS |
| Static Analysis | cppcheck on `l1_m7/ l1_m4/ shared/ tests/` |

## Key Numbers

| Metric | Value |
|--------|-------|
| Source files | ~190 |
| Lines of C/C++ | ~50,000 |
| Unit tests | 58 |
| CI build time | ~2 min |
| M7 firmware.bin | 740B (core), full build ~300KB |
| FOC PWM frequency | 20 kHz |
| SPI IPC speed | 20 MHz |
| IMU rate | 1 kHz |
