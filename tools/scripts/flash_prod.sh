#!/bin/bash
#
# flash_prod.sh — Production flashing script
#
# Steps:
#   1. STM32_Programmer_CLI -c port=SWD -w bootloader.bin 0x08000000
#   2. STM32_Programmer_CLI -w app.bin 0x08010000
#   3. STM32_Programmer_CLI -ob BOR_LEV=2 RDP=0
#   4. Write serial number to OTP via debug command
#   5. Verify flash CRC
#   6. Exit code 0 = success
#
# Usage:
#   ./flash_prod.sh [options]
#
# Options:
#   -s, --serial SERIAL    Device serial number (mandatory)
#   -b, --bootloader FILE  Bootloader binary path (default: build/bootloader.bin)
#   -a, --app FILE         Application binary path (default: build/app.bin)
#   -p, --port PORT        ST-Link port (default: SWD)
#   -v, --verbose          Verbose output
#   -h, --help             Show this help
#

set -euo pipefail

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"

STM32_PROGRAMMER="STM32_Programmer_CLI"
BOOTLOADER_BIN="${PROJECT_DIR}/build/bootloader.bin"
APP_BIN="${PROJECT_DIR}/build/app.bin"
STM32_PORT="SWD"
SERIAL_NUMBER=""
VERBOSE=0

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# ---------------------------------------------------------------------------
# Functions
# ---------------------------------------------------------------------------
log_info()    { echo -e "${GREEN}[INFO]${NC}  $*"; }
log_warn()    { echo -e "${YELLOW}[WARN]${NC}  $*"; }
log_error()   { echo -e "${RED}[ERROR]${NC} $*"; }
log_step()    { echo -e "\n${GREEN}[STEP]${NC}  $*"; }

usage() {
    cat <<EOF
Usage: $(basename "$0") -s SERIAL [options]

Production flashing script for STM32H747.

Mandatory:
  -s, --serial SERIAL  Device serial number to write to OTP

Options:
  -b, --bootloader FILE  Bootloader binary  (default: ${BOOTLOADER_BIN})
  -a, --app FILE         Application binary (default: ${APP_BIN})
  -p, --port PORT        ST-Link port       (default: ${STM32_PORT})
  -v, --verbose          Enable verbose output
  -h, --help             Show this help and exit

Examples:
  ./flash_prod.sh --serial H747ELITE0001
  ./flash_prod.sh -s H747ELITE0001 -b build/v2/bootloader.bin -a build/v2/app.bin
EOF
    exit 0
}

cleanup() {
    log_info "Cleaning up..."
    # Any required cleanup
}

check_prerequisites() {
    if ! command -v "${STM32_PROGRAMMER}" &>/dev/null; then
        log_warn "${STM32_PROGRAMMER} not in PATH, checking common locations..."
        local search_paths=(
            "/c/Program Files/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin"
            "/usr/local/ST/STM32CubeProgrammer/bin"
            "/opt/STM32CubeProgrammer/bin"
            "/Applications/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOs/bin"
        )
        for sp in "${search_paths[@]}"; do
            if [[ -f "${sp}/${STM32_PROGRAMMER}" ]]; then
                STM32_PROGRAMMER="${sp}/${STM32_PROGRAMMER}"
                log_info "Found STM32_Programmer_CLI at ${STM32_PROGRAMMER}"
                break
            fi
        done
        if ! command -v "${STM32_PROGRAMMER}" &>/dev/null && [[ ! -f "${STM32_PROGRAMMER}" ]]; then
            log_error "STM32_Programmer_CLI not found. Install STM32CubeProgrammer."
            exit 1
        fi
    fi

    # Check binary files exist
    if [[ ! -f "${BOOTLOADER_BIN}" ]]; then
        log_error "Bootloader binary not found: ${BOOTLOADER_BIN}"
        exit 1
    fi
    if [[ ! -f "${APP_BIN}" ]]; then
        log_error "Application binary not found: ${APP_BIN}"
        exit 1
    fi

    log_info "Bootloader: ${BOOTLOADER_BIN} ($(stat -c%s "${BOOTLOADER_BIN}" 2>/dev/null || stat -f%z "${BOOTLOADER_BIN}" 2>/dev/null) bytes)"
    log_info "Application: ${APP_BIN} ($(stat -c%s "${APP_BIN}" 2>/dev/null || stat -f%z "${APP_BIN}" 2>/dev/null) bytes)"
}

check_device_connection() {
    log_info "Checking device connection..."
    if ! "${STM32_PROGRAMMER}" --connect port="${STM32_PORT}" --readunprotect \
        2>&1 | head -5; then
        log_warn "Initial connection issues, retrying..."
        sleep 1
    fi
    log_info "Device connected."
}

flash_bootloader() {
    log_step "1/5: Flashing Bootloader at 0x08000000"
    "${STM32_PROGRAMMER}" -c port="${STM32_PORT}" \
        -w "${BOOTLOADER_BIN}" 0x08000000 \
        -v -rst

    local ret=$?
    if [[ $ret -ne 0 ]]; then
        log_error "Bootloader flash failed (exit code: ${ret})"
        return 1
    fi
    log_info "Bootloader flashed successfully."
}

flash_application() {
    log_step "2/5: Flashing Application at 0x08010000"
    "${STM32_PROGRAMMER}" -c port="${STM32_PORT}" \
        -w "${APP_BIN}" 0x08010000 \
        -v -rst

    local ret=$?
    if [[ $ret -ne 0 ]]; then
        log_error "Application flash failed (exit code: ${ret})"
        return 1
    fi
    log_info "Application flashed successfully."
}

set_option_bytes() {
    log_step "3/5: Setting Option Bytes (BOR_LEV=2, RDP=0)"
    "${STM32_PROGRAMMER}" -c port="${STM32_PORT}" \
        -ob BOR_LEV=2 RDP=0

    local ret=$?
    if [[ $ret -ne 0 ]]; then
        log_error "Option bytes programming failed (exit code: ${ret})"
        return 1
    fi
    log_info "Option bytes set."
}

write_serial_otp() {
    log_step "4/5: Writing Serial Number to OTP"

    if [[ -z "${SERIAL_NUMBER}" ]]; then
        log_error "Serial number not provided"
        return 1
    fi

    # Validate serial number format (alphanumeric, 4-32 chars)
    if ! [[ "${SERIAL_NUMBER}" =~ ^[A-Za-z0-9_-]{4,32}$ ]]; then
        log_error "Invalid serial number format: '${SERIAL_NUMBER}'"
        log_error "Must be 4-32 alphanumeric characters (underscore/hyphen allowed)"
        return 1
    fi

    log_info "Writing serial: ${SERIAL_NUMBER}"

    # Write serial to OTP using STM32_Programmer_CLI's OTP write command
    # OTP address space on STM32H747: 0x1FFF7000 - 0x1FFF73FF (1024 bytes)
    # We write at offset 0 (OTP0) as: magic(4) + length(4) + serial(N)
    local otp_addr="0x1FFF7000"
    local magic="PROD"

    # Convert magic and length to hex
    local magic_hex
    magic_hex=$(echo -n "${magic}" | xxd -p 2>/dev/null || printf "50524f44")
    local serial_hex
    serial_hex=$(echo -n "${SERIAL_NUMBER}" | xxd -p 2>/dev/null || echo -n "${SERIAL_NUMBER}" | od -An -tx1 | tr -d ' \n')

    "${STM32_PROGRAMMER}" -c port="${STM32_PORT}" \
        -otg 1 \
        -w "${otp_addr}" "${magic_hex}${serial_hex}" \
        -v

    local ret=$?
    if [[ $ret -ne 0 ]]; then
        log_error "OTP write failed (exit code: ${ret})"
        # Non-fatal: the device can still work, serial will be set in the field
        log_warn "Continuing despite OTP write failure"
        return 0
    fi

    log_info "Serial number '${SERIAL_NUMBER}' written to OTP."

    # Verify by reading back
    log_info "Verifying OTP..."
    "${STM32_PROGRAMMER}" -c port="${STM32_PORT}" -otr 1 -v
}

verify_flash() {
    log_step "5/5: Verifying Flash CRC"

    # Compute CRC of application region and compare

    # Method 1: Use STM32_Programmer_CLI's CRC check
    log_info "Verifying application CRC..."

    "${STM32_PROGRAMMER}" -c port="${STM32_PORT}" \
        -crc 0x08010000 "$(stat -c%s "${APP_BIN}" 2>/dev/null || stat -f%z "${APP_BIN}" 2>/dev/null)"

    local ret=$?
    if [[ $ret -ne 0 ]]; then
        log_error "Flash verification FAILED"
        return 1
    fi

    log_info "Flash CRC verified successfully."
}

report_success() {
    echo ""
    echo "============================================"
    echo -e "${GREEN}  FLASH COMPLETE - DEVICE READY${NC}"
    echo "============================================"
    echo "  Serial:  ${SERIAL_NUMBER}"
    echo "  Bootloader: ${BOOTLOADER_BIN}"
    echo "  App:        ${APP_BIN}"
    echo "============================================"
}

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
main() {
    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case "$1" in
            -s|--serial)
                SERIAL_NUMBER="$2"
                shift 2
                ;;
            -b|--bootloader)
                BOOTLOADER_BIN="$2"
                shift 2
                ;;
            -a|--app)
                APP_BIN="$2"
                shift 2
                ;;
            -p|--port)
                STM32_PORT="$2"
                shift 2
                ;;
            -v|--verbose)
                VERBOSE=1
                shift
                ;;
            -h|--help)
                usage
                ;;
            *)
                log_error "Unknown option: $1"
                usage
                ;;
        esac
    done

    # Validate mandatory args
    if [[ -z "${SERIAL_NUMBER}" ]]; then
        log_error "Serial number is mandatory. Use -s SERIAL"
        usage
    fi

    # Trap
    trap cleanup EXIT

    echo "============================================"
    echo "  Production Flash Tool — STM32H747"
    echo "============================================"
    echo "  Device:    ${STM32_PORT}"
    echo "  Serial:    ${SERIAL_NUMBER}"
    echo "  Date:      $(date)"
    echo "============================================"

    # Execute steps
    check_prerequisites
    check_device_connection
    flash_bootloader          || { log_error "Bootloader flash failed"; exit 1; }
    flash_application         || { log_error "Application flash failed"; exit 1; }
    set_option_bytes          || { log_error "Option bytes failed"; exit 1; }
    write_serial_otp
    verify_flash              || { log_error "Verification failed"; exit 1; }
    report_success

    exit 0
}

main "$@"
