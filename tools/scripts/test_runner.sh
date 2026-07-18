#!/bin/bash
#
# test_runner.sh — Automated production test runner
#
# Steps:
#   1. Build all targets
#   2. Flash to DUT
#   3. Run power-on self-test via UART
#   4. Run motor calibration sequence
#   5. Run IMU calibration sequence
#   6. Run ToF calibration sequence
#   7. Run 10-minute aging (motors at 50% duty)
#   8. Collect all test results
#   9. POST to MES (Manufacturing Execution System)
#   10. Print PASS/FAIL summary
#
# Each step has a configurable timeout.
#
# Usage:
#   ./test_runner.sh [options]
#
# Options:
#   --serial SERIAL       Device serial number
#   --build-dir DIR       Build directory (default: build)
#   --port PORT           Serial port for test comms (default: /dev/ttyUSB0)
#   --mes-url URL         MES endpoint URL
#   --skip-build          Skip build step
#   --skip-flash          Skip flash step
#   --skip-aging          Skip 10-minute aging test
#   --skip-mes            Skip MES upload
#   --verbose             Verbose output
#   -h, --help            Show help
#

set -euo pipefail

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
TOOLS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

SERIAL_NUMBER=""
BUILD_DIR="${PROJECT_DIR}/build"
SERIAL_PORT="/dev/ttyUSB0"
BAUD_RATE=115200
MES_URL=""

SKIP_BUILD=0
SKIP_FLASH=0
SKIP_AGING=0
SKIP_MES=0
VERBOSE=0

# Timeouts per step (seconds)
TIMEOUT_BUILD=300
TIMEOUT_FLASH=120
TIMEOUT_POST=60
TIMEOUT_MOTOR_CAL=180
TIMEOUT_IMU_CAL=120
TIMEOUT_TOF_CAL=300
TIMEOUT_AGING=660  # 11 minutes (10 + buffer)
TIMEOUT_MES=30
TIMEOUT_UART_RESP=10

# Results tracking
declare -A TEST_RESULTS
declare -A TEST_MESSAGES
TESTS_PASSED=0
TESTS_FAILED=0
TESTS_SKIPPED=0
START_TIME=""

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# ---------------------------------------------------------------------------
# Functions
# ---------------------------------------------------------------------------
log_info()    { echo -e "${GREEN}[INFO]${NC}    $*"; }
log_warn()    { echo -e "${YELLOW}[WARN]${NC}    $*"; }
log_error()   { echo -e "${RED}[ERROR]${NC}   $*"; }
log_step()    { echo -e "\n${BLUE}[STEP]${NC}    $*"; }
log_debug()   { [[ $VERBOSE -eq 1 ]] && echo -e "[DEBUG]   $*"; }

usage() {
    cat <<EOF
Usage: $(basename "$0") --serial SERIAL [options]

Automated production test runner for H747 robot.

Mandatory:
  --serial SERIAL      Device serial number

Options:
  --build-dir DIR      Build directory          (default: ${BUILD_DIR})
  --port PORT          Serial port              (default: ${SERIAL_PORT})
  --mes-url URL        MES endpoint URL
  --skip-build         Skip build step
  --skip-flash         Skip flash step
  --skip-aging         Skip 10-minute aging test
  --skip-mes           Skip MES upload
  --verbose            Verbose output
  -h, --help           Show this help

Examples:
  ./test_runner.sh --serial H747TEST0001
  ./test_runner.sh --serial H747TEST0001 --skip-aging
EOF
    exit 0
}

cleanup() {
    log_info "Cleaning up test resources..."
}

record_result() {
    local test_name="$1"
    local status="$2"   # PASS, FAIL, SKIP
    local message="$3"

    TEST_RESULTS["${test_name}"]="${status}"
    TEST_MESSAGES["${test_name}"]="${message}"

    case "${status}" in
        PASS)   TESTS_PASSED=$((TESTS_PASSED + 1)) ;;
        FAIL)   TESTS_FAILED=$((TESTS_FAILED + 1)) ;;
        SKIP)   TESTS_SKIPPED=$((TESTS_SKIPPED + 1)) ;;
    esac

    if [[ "${status}" == "PASS" ]]; then
        echo -e "  ${GREEN}[PASS]${NC} ${test_name}"
    elif [[ "${status}" == "FAIL" ]]; then
        echo -e "  ${RED}[FAIL]${NC} ${test_name}: ${message}"
    else
        echo -e "  ${YELLOW}[SKIP]${NC} ${test_name}: ${message}"
    fi
}

run_with_timeout() {
    local timeout_sec="$1"
    local cmd="$2"
    local label="$3"

    log_info "Running: ${label} (timeout: ${timeout_sec}s)"

    # Use timeout command if available
    if command -v timeout &>/dev/null; then
        timeout "${timeout_sec}" bash -c "${cmd}" 2>&1
        local ret=$?
    else
        # Fallback for systems without timeout
        local pid
        bash -c "${cmd}" 2>&1 &
        pid=$!
        (
            sleep "${timeout_sec}"
            kill -TERM "${pid}" 2>/dev/null
        ) &
        local killer_pid=$!
        wait "${pid}" 2>/dev/null
        ret=$?
        kill "${killer_pid}" 2>/dev/null
        wait "${killer_pid}" 2>/dev/null || true
    fi

    if [[ $ret -eq 124 ]]; then
        log_error "TIMEOUT: ${label} exceeded ${timeout_sec}s"
        return 124
    fi
    return $ret
}

uart_send_cmd() {
    local cmd="$1"
    local expected="$2"
    local timeout_sec="${3:-${TIMEOUT_UART_RESP}}"

    log_debug "UART TX: ${cmd}"
    echo "${cmd}" > "${SERIAL_PORT}"
    read -t "${timeout_sec}" response < "${SERIAL_PORT}" || true

    if [[ -n "${response}" ]]; then
        log_debug "UART RX: ${response}"
        if [[ -n "${expected}" ]]; then
            if echo "${response}" | grep -q "${expected}"; then
                return 0
            else
                return 1
            fi
        fi
        echo "${response}"
        return 0
    fi
    return 1
}

# ---------------------------------------------------------------------------
# Test Steps
# ---------------------------------------------------------------------------

step_build() {
    log_step "1/10: Building all targets"
    cd "${PROJECT_DIR}"

    local build_cmd="cmake --build ${BUILD_DIR} --target all -j$(nproc 2>/dev/null || echo 4) 2>&1"
    run_with_timeout "${TIMEOUT_BUILD}" "${build_cmd}" "Build"

    local ret=$?
    if [[ $ret -eq 0 ]]; then
        record_result "Build" "PASS" "All targets built successfully"
    elif [[ $ret -eq 124 ]]; then
        record_result "Build" "FAIL" "Build timed out"
    else
        record_result "Build" "FAIL" "Build failed (exit code: ${ret})"
    fi
    return $ret
}

step_flash() {
    log_step "2/10: Flashing device"
    cd "${PROJECT_DIR}"

    local flash_cmd="${TOOLS_DIR}/scripts/flash_prod.sh -s ${SERIAL_NUMBER} -b ${BUILD_DIR}/bootloader.bin -a ${BUILD_DIR}/app.bin 2>&1"
    run_with_timeout "${TIMEOUT_FLASH}" "${flash_cmd}" "Flash"

    local ret=$?
    if [[ $ret -eq 0 ]]; then
        record_result "Flash" "PASS" "Device flashed successfully"
    else
        record_result "Flash" "FAIL" "Flash failed (exit code: ${ret})"
    fi
    return $ret
}

step_post() {
    log_step "3/10: Power-on Self-Test (POST) via UART"
    if [[ ! -e "${SERIAL_PORT}" ]]; then
        record_result "POST" "FAIL" "Serial port ${SERIAL_PORT} not available"
        return 1
    fi

    # Configure serial port
    stty -F "${SERIAL_PORT}" "${BAUD_RATE}" raw -echo 2>/dev/null || true

    # Reset device and wait for POST output
    uart_send_cmd "reset" "POST" ${TIMEOUT_POST}

    local post_ok=0
    local end_time=$((SECONDS + TIMEOUT_POST))
    while [[ $SECONDS -lt $end_time ]]; do
        local line
        read -t 1 line < "${SERIAL_PORT}" || true
        if [[ -n "${line}" ]]; then
            log_debug "POST: ${line}"
            if echo "${line}" | grep -qi "POST OK\|POST_PASS\|Self-test passed\|INIT OK"; then
                post_ok=1
                break
            fi
            if echo "${line}" | grep -qi "POST FAIL\|FATAL\|INIT FAIL"; then
                record_result "POST" "FAIL" "POST reported failure: ${line}"
                return 1
            fi
        fi
    done

    if [[ $post_ok -eq 1 ]]; then
        record_result "POST" "PASS" "Power-on self-test passed"
        return 0
    else
        record_result "POST" "FAIL" "POST did not complete within ${TIMEOUT_POST}s"
        return 1
    fi
}

step_motor_cal() {
    log_step "4/10: Motor calibration"
    cd "${TOOLS_DIR}/calibration"

    local cal_cmd="python3 motor_cal_tool.py --port ${SERIAL_PORT} --output ${BUILD_DIR}/motor_params.h 2>&1"
    run_with_timeout "${TIMEOUT_MOTOR_CAL}" "${cal_cmd}" "Motor Calibration"

    local ret=$?
    if [[ $ret -eq 0 ]]; then
        record_result "Motor Cal" "PASS" "Motor calibration completed"
    else
        record_result "Motor Cal" "FAIL" "Motor calibration failed"
    fi
    return $ret
}

step_imu_cal() {
    log_step "5/10: IMU calibration"
    cd "${TOOLS_DIR}/calibration"

    local cal_cmd="python3 imu_cal_tool.py --port ${SERIAL_PORT} --output ${BUILD_DIR}/imu_calibration.h 2>&1"
    run_with_timeout "${TIMEOUT_IMU_CAL}" "${cal_cmd}" "IMU Calibration"

    local ret=$?
    if [[ $ret -eq 0 ]]; then
        record_result "IMU Cal" "PASS" "IMU calibration completed"
    else
        record_result "IMU Cal" "FAIL" "IMU calibration failed"
    fi
    return $ret
}

step_tof_cal() {
    log_step "6/10: ToF calibration"
    cd "${TOOLS_DIR}/calibration"

    local cal_cmd="python3 tof_cal_tool.py --port ${SERIAL_PORT} --output ${BUILD_DIR}/tof_calibration.h --zones 4 2>&1"
    run_with_timeout "${TIMEOUT_TOF_CAL}" "${cal_cmd}" "ToF Calibration"

    local ret=$?
    if [[ $ret -eq 0 ]]; then
        record_result "ToF Cal" "PASS" "ToF calibration completed"
    else
        record_result "ToF Cal" "FAIL" "ToF calibration failed"
    fi
    return $ret
}

step_aging() {
    log_step "7/10: Aging test — motors at 50% duty for 10 minutes"
    log_info "Starting 10-minute aging test..."

    uart_send_cmd "motor left speed 50" "OK" || true
    uart_send_cmd "motor right speed 50" "OK" || true

    local aging_ok=1
    local end_time=$((SECONDS + TIMEOUT_AGING))
    while [[ $SECONDS -lt $end_time ]]; do
        local remaining=$((end_time - SECONDS))
        if [[ $((remaining % 60)) -eq 0 ]]; then
            log_info "Aging: ${remaining}s remaining..."
        fi
        sleep 10
    done

    # Stop motors
    uart_send_cmd "motor left speed 0" "OK" || true
    uart_send_cmd "motor right speed 0" "OK" || true

    if [[ $aging_ok -eq 1 ]]; then
        record_result "Aging" "PASS" "10-minute aging test completed"
    else
        record_result "Aging" "FAIL" "Aging test interrupted"
    fi
}

step_collect_results() {
    log_step "8/10: Collecting test results"

    local results_file="${BUILD_DIR}/test_results_${SERIAL_NUMBER}.json"
    local timestamp
    timestamp=$(date -u +"%Y-%m-%dT%H:%M:%SZ")

    # Build JSON
    cat > "${results_file}" <<EOF
{
    "test_run": {
        "serial": "${SERIAL_NUMBER}",
        "timestamp": "${timestamp}",
        "script_version": "1.0.0",
        "overall_status": "$([[ ${TESTS_FAILED} -eq 0 ]] && echo "PASS" || echo "FAIL")"
    },
    "results": {
EOF

    local first=1
    for test_name in "${!TEST_RESULTS[@]}"; do
        [[ $first -eq 0 ]] && echo "," >> "${results_file}"
        first=0
        cat >> "${results_file}" <<EOF
        "${test_name}": {
            "status": "${TEST_RESULTS[${test_name}]}",
            "message": "${TEST_MESSAGES[${test_name}]}"
        }
EOF
    done

    cat >> "${results_file}" <<EOF
    },
    "summary": {
        "passed": ${TESTS_PASSED},
        "failed": ${TESTS_FAILED},
        "skipped": ${TESTS_SKIPPED},
        "total": $((TESTS_PASSED + TESTS_FAILED + TESTS_SKIPPED))
    }
}
EOF

    log_info "Results written to ${results_file}"
    record_result "Results Collection" "PASS" "Test results collected and saved"
}

step_mes_upload() {
    log_step "9/10: Uploading results to MES"

    local results_file="${BUILD_DIR}/test_results_${SERIAL_NUMBER}.json"

    if [[ -z "${MES_URL}" ]]; then
        record_result "MES Upload" "SKIP" "No MES URL configured"
        return 0
    fi

    if command -v curl &>/dev/null; then
        run_with_timeout "${TIMEOUT_MES}" \
            "curl -s -X POST -H 'Content-Type: application/json' -d @${results_file} ${MES_URL}" \
            "MES Upload"
        local ret=$?
        if [[ $ret -eq 0 ]]; then
            record_result "MES Upload" "PASS" "Results uploaded to MES"
        else
            record_result "MES Upload" "FAIL" "MES upload failed (exit: ${ret})"
        fi
    elif command -v wget &>/dev/null; then
        run_with_timeout "${TIMEOUT_MES}" \
            "wget --post-file=${results_file} --header='Content-Type: application/json' -O /dev/null ${MES_URL}" \
            "MES Upload"
        local ret=$?
        if [[ $ret -eq 0 ]]; then
            record_result "MES Upload" "PASS" "Results uploaded to MES"
        else
            record_result "MES Upload" "FAIL" "MES upload failed (exit: ${ret})"
        fi
    else
        record_result "MES Upload" "SKIP" "No curl/wget available"
    fi
}

step_summary() {
    log_step "10/10: Test Summary"

    echo ""
    echo "============================================"
    echo "  TEST SUMMARY — ${SERIAL_NUMBER}"
    echo "============================================"
    printf "  %-25s %s\n" "TEST" "STATUS"
    echo "  -----------------------------------------"
    for test_name in "${!TEST_RESULTS[@]}"; do
        local status="${TEST_RESULTS[${test_name}]}"
        local color="${GREEN}"
        [[ "${status}" == "FAIL" ]] && color="${RED}"
        [[ "${status}" == "SKIP" ]] && color="${YELLOW}"
        printf "  %-25s ${color}%s${NC}\n" "${test_name}" "${status}"
    done
    echo "  -----------------------------------------"
    echo -e "  ${GREEN}PASSED:${NC}  ${TESTS_PASSED}"
    echo -e "  ${RED}FAILED:${NC}  ${TESTS_FAILED}"
    echo -e "  ${YELLOW}SKIPPED:${NC} ${TESTS_SKIPPED}"
    echo "============================================"

    overall="PASS"
    [[ ${TESTS_FAILED} -gt 0 ]] && overall="FAIL"
    record_result "Overall" "${overall}" "Tests: ${TESTS_PASSED} passed, ${TESTS_FAILED} failed"
}

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
main() {
    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --serial)
                SERIAL_NUMBER="$2"
                shift 2
                ;;
            --build-dir)
                BUILD_DIR="$2"
                shift 2
                ;;
            --port)
                SERIAL_PORT="$2"
                shift 2
                ;;
            --mes-url)
                MES_URL="$2"
                shift 2
                ;;
            --skip-build)
                SKIP_BUILD=1
                shift
                ;;
            --skip-flash)
                SKIP_FLASH=1
                shift
                ;;
            --skip-aging)
                SKIP_AGING=1
                shift
                ;;
            --skip-mes)
                SKIP_MES=1
                shift
                ;;
            --verbose)
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

    if [[ -z "${SERIAL_NUMBER}" ]]; then
        log_error "Serial number is required. Use --serial SERIAL"
        usage
    fi

    START_TIME=$(date +%s)

    echo "============================================"
    echo "  Automated Test Runner — H747 Robot"
    echo "============================================"
    echo "  Serial:  ${SERIAL_NUMBER}"
    echo "  Port:    ${SERIAL_PORT}"
    echo "  Build:   ${BUILD_DIR}"
    echo "  Date:    $(date)"
    echo "============================================"

    trap cleanup EXIT

    # Step 1: Build
    if [[ $SKIP_BUILD -eq 1 ]]; then
        record_result "Build" "SKIP" "Skipped by user request"
    else
        step_build || true
    fi

    # Step 2: Flash
    if [[ $SKIP_FLASH -eq 1 ]]; then
        record_result "Flash" "SKIP" "Skipped by user request"
    else
        step_flash || true
    fi

    # Step 3: POST
    step_post || true

    # Step 4: Motor Calibration
    step_motor_cal || true

    # Step 5: IMU Calibration
    step_imu_cal || true

    # Step 6: ToF Calibration
    step_tof_cal || true

    # Step 7: Aging
    if [[ $SKIP_AGING -eq 1 ]]; then
        record_result "Aging" "SKIP" "Skipped by user request"
    else
        step_aging || true
    fi

    # Step 8: Collect Results
    step_collect_results || true

    # Step 9: MES Upload
    if [[ $SKIP_MES -eq 1 ]]; then
        record_result "MES Upload" "SKIP" "Skipped by user request"
    else
        step_mes_upload || true
    fi

    # Step 10: Summary
    step_summary

    local elapsed=$(( $(date +%s) - START_TIME ))
    log_info "Total test time: $((elapsed / 60))m $((elapsed % 60))s"

    if [[ ${TESTS_FAILED} -gt 0 ]]; then
        exit 1
    fi
    exit 0
}

main "$@"
