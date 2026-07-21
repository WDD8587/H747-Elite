cmake_minimum_required(VERSION 3.16)
set(CMAKE_PROJECT_NAME firmware)

# ---- Target core selection: cmake -DTARGET_CORE=M7 or M4 ----
if(NOT DEFINED TARGET_CORE)
    set(TARGET_CORE M7)
    message(STATUS "TARGET_CORE not set, defaulting to M7")
endif()

# ---- FreeRTOS kernel path ----
set(FREERTOS_KERNEL ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-Kernel)

if(TARGET_CORE STREQUAL "M7")
    message(STATUS "Building STM32H747 M7 firmware")
    set(CPU_FLAGS    -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-d16)
    set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/linker/stm32h747_m7.ld)
    set(STARTUP_S    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m7/startup/startup_stm32h747xx.s)
    set(SYSCALLS_C   ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m7/startup/syscalls.c)
    set(CORE_DEFINE  CORE_CM7)
    set(FREERTOS_PORT ${FREERTOS_KERNEL}/portable/GCC/ARM_CM7/r0p1)
elseif(TARGET_CORE STREQUAL "M4")
    message(STATUS "Building STM32H747 M4 firmware")
    set(CPU_FLAGS    -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16)
    set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/linker/stm32h747_m4.ld)
    set(STARTUP_S    ${CMAKE_CURRENT_SOURCE_DIR}/startup/startup_stm32h747_m4.s)
    set(SYSCALLS_C   ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m7/startup/syscalls.c)
    set(CORE_DEFINE  CORE_CM4)
    set(FREERTOS_PORT ${FREERTOS_KERNEL}/portable/GCC/ARM_CM4F)
else()
    message(FATAL_ERROR "TARGET_CORE must be M7 or M4, got: ${TARGET_CORE}")
endif()

# ---- Compile flags ----
set(COMMON_FLAGS  -Wall -Wextra -Werror -Wno-unused-parameter
                  -Os -ffunction-sections -fdata-sections -fno-common -std=gnu11)
set(CMAKE_C_FLAGS       "${CPU_FLAGS} ${COMMON_FLAGS}" CACHE INTERNAL "")
set(CMAKE_ASM_FLAGS     "${CPU_FLAGS} -x assembler-with-cpp" CACHE INTERNAL "")
set(CMAKE_EXE_LINKER_FLAGS
    "-T${LINKER_SCRIPT} -Wl,--gc-sections -Wl,-Map=${CMAKE_PROJECT_NAME}.map --specs=nosys.specs -lm -Wl,--print-memory-usage"
    CACHE INTERNAL "")

# ---- Include paths ----
set(COMMON_INC
    ${CMAKE_CURRENT_SOURCE_DIR}                           # FreeRTOSConfig.h
    ${FREERTOS_KERNEL}/include                            # FreeRTOS.h, task.h, queue.h etc.
    ${FREERTOS_PORT}                                      # portmacro.h
    ${CMAKE_CURRENT_SOURCE_DIR}/stubs                     # HAL stubs (stm32h7xx_hal.h, cmsis_os.h)
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/inc             # shared/inc (ipc_proto.h, ipc_transport.h)
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m7/Inc              # m7_config.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m4/Inc              # m4_config.h
)

# ---- FreeRTOS kernel sources (used by both M7 and M4) ----
set(FREERTOS_SRCS
    ${FREERTOS_KERNEL}/tasks.c
    ${FREERTOS_KERNEL}/queue.c
    ${FREERTOS_KERNEL}/list.c
    ${FREERTOS_KERNEL}/timers.c
    ${FREERTOS_KERNEL}/event_groups.c
    ${FREERTOS_KERNEL}/stream_buffer.c
    ${FREERTOS_PORT}/port.c
    ${FREERTOS_KERNEL}/portable/MemMang/heap_4.c
)

# ---- M7-specific source files ----
set(M7_SRCS
    ${STARTUP_S}
    ${SYSCALLS_C}
    # Core
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m7/Core/main_m7.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m7/Core/ipc_bridge.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m7/Core/imu_prep.c
    # Motor
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m7/Core/motor/foc_isr.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m7/Core/motor/foc_params.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m7/Core/motor/foc_encoder.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m7/Core/motor/foc_deadtime.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m7/Core/motor/foc_field_weaken.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m7/Core/motor/foc_observer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m7/Core/motor/multi_motor.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m7/Core/motor/roller_brush.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m7/Core/motor/side_brush.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m7/Core/motor/vacuum_fan.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m7/Core/motor/ethercat.c
    # HAL config
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m7/Core/hal/gpio_config.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m7/Core/hal/dma_config.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m7/Core/hal/nvic_config.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m7/Core/hal/clock_tree.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m7/Core/hal/mpu_config.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m7/Core/hal/flash_config.c
    # Bootloader
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m7/Core/bootloader/boot_main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m7/Core/bootloader/boot_verify.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m7/Core/bootloader/boot_flash.c
)

# ---- M4-specific source files ----
set(M4_SRCS
    ${STARTUP_S}
    ${SYSCALLS_C}
    # Core
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m4/Core/main_m4.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m4/Core/safety_wd.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m4/Core/bms_smbus.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m4/Core/bms_charge.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m4/Core/tof_vl53l5cx.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m4/Core/dock_handshake.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m4/Core/lpwr_stop2.c
    # BMS subsystem
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m4/Core/bms/bms_soh.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m4/Core/bms/bms_balance.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m4/Core/bms/bms_ntc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m4/Core/bms/bms_protect.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m4/Core/bms/bms_precharge.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m4/Core/bms/bms_cell_id.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m4/Core/bms/bms_upgrade.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m4/Core/bms/bms_charge.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m4/Core/bms/bms_discharge.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m4/Core/bms/bms_lifetime.c
    # Safety subsystem
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m4/Core/safety/safety_redundant.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m4/Core/safety/safety_fmea.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m4/Core/safety/safety_fault.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m4/Core/safety/safety_tilt.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m4/Core/safety/safety_stall.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m4/Core/safety/safety_drop.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m4/Core/safety/safety_collision.c
    # Factory calibration
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m4/Core/factory/motor_cal.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m4/Core/factory/imu_cal.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m4/Core/factory/tof_cal.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m4/Core/factory/battery.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m4/Core/factory/rf.c
    # Diagnostics
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m4/Core/diagnostics/selftest.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m4/Core/diagnostics/health_report.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../l1_m4/Core/diagnostics/debug_cmd.c
)

# ---- Shared source files (used by both M7 and M4) ----
set(SHARED_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/src/ipc_uart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/src/ipc_spi.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/src/ipc_usb.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/src/ipc_transport.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/src/ipc_hsem.c
    # Stub globals: huart7, SystemCoreClock, CORDIC_MOCK
    ${CMAKE_CURRENT_SOURCE_DIR}/stubs/globals.c
)

# ---- Select sources based on core ----
if(TARGET_CORE STREQUAL "M7")
    set(FW_SRCS ${M7_SRCS} ${SHARED_SRCS} ${FREERTOS_SRCS})
else()
    set(FW_SRCS ${M4_SRCS} ${SHARED_SRCS} ${FREERTOS_SRCS})
endif()

# ---- Build executable ----
add_executable(${CMAKE_PROJECT_NAME} ${FW_SRCS})
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${COMMON_INC})
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ${CORE_DEFINE}
    STM32H747xx
    USE_HAL_DRIVER
)

# ---- Post-build: generate .bin and .hex ----
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            ${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMENT "Generating firmware.bin, firmware.hex, and size summary"
)
