#!/usr/bin/env python3
"""
Motor Calibration Tool

Commands:
- PhaseResistance: Send DC current, measure steady-state V, compute R = V/I
- PhaseInductance: Send 1 kHz square wave, measure di/dt, compute L = V*dt/di
- EncoderOffset: Lock rotor to phase A, read encoder position

Output: motor_params.h with calibration constants.

Usage:
    python motor_cal_tool.py --port /dev/ttyUSB0 --output motor_params.h
"""

import argparse
import sys
import json
import time
import math
import statistics
import struct
from typing import List, Tuple


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
NUM_MOTORS = 2  # left + right wheel
PHASE_RESISTANCE_CURRENT_A = 1.0  # DC current for R measurement
PHASE_RESISTANCE_SAMPLES = 50
PHASE_RESISTANCE_SETTLE_MS = 500

PHASE_INDUCTANCE_FREQ_HZ = 1000.0
PHASE_INDUCTANCE_SAMPLES = 100

ENCODER_OFFSET_SAMPLES = 20

# ---------------------------------------------------------------------------
# Serial communication helper
# ---------------------------------------------------------------------------
class MotorSerialInterface:
    """Serial interface to robot motor controller."""

    def __init__(self, port: str, baud: int = 115200):
        import serial
        self.ser = serial.Serial(port, baud, timeout=2.0)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def send_command(self, cmd: str) -> str:
        """Send a command and read response."""
        self.ser.write((cmd + "\n").encode("ascii"))
        time.sleep(0.1)
        response = self.ser.readline().decode("ascii", errors="ignore").strip()
        return response

    def send_command_with_retries(self, cmd: str, expected_prefix: str = "",
                                   retries: int = 3) -> str:
        """Send command with retry logic."""
        for attempt in range(retries):
            resp = self.send_command(cmd)
            if not expected_prefix or resp.startswith(expected_prefix):
                return resp
            time.sleep(0.2)
        return ""

    def read_data(self, num_lines: int = 1) -> List[str]:
        """Read multiple data lines."""
        lines = []
        for _ in range(num_lines):
            line = self.ser.readline().decode("ascii", errors="ignore").strip()
            if line:
                lines.append(line)
        return lines

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()


# ---------------------------------------------------------------------------
# Phase Resistance measurement
# ---------------------------------------------------------------------------
def measure_phase_resistance(iface: MotorSerialInterface, motor_id: int,
                              current_a: float = PHASE_RESISTANCE_CURRENT_A) -> dict:
    """
    Measure phase resistance by applying DC current and measuring voltage.

    Returns dict with resistance and stats.
    """
    print(f"\nMotor {motor_id}: Measuring phase resistance...")

    # Enable motor and set DC current
    resp = iface.send_command_with_retries(
        f"motor {motor_id} cal resistance_start {current_a:.3f}",
        expected_prefix="OK"
    )
    if not resp:
        print(f"  ERROR: Failed to start resistance measurement")
        return {"resistance_ohm": 0.0, "error": "Communication failed"}

    # Wait for settling
    time.sleep(PHASE_RESISTANCE_SETTLE_MS / 1000.0)

    # Collect V/I samples
    voltages = []
    currents = []
    for i in range(PHASE_RESISTANCE_SAMPLES):
        resp = iface.send_command(f"motor {motor_id} cal resistance_sample")
        if resp:
            try:
                # Expected format: "V=1.234 I=0.987"
                parts = resp.replace("V=", "").replace("I=", "").split()
                if len(parts) >= 2:
                    v = float(parts[0])
                    cur = float(parts[1])
                    voltages.append(v)
                    currents.append(cur)
            except (ValueError, IndexError):
                pass

    # Disable current
    iface.send_command(f"motor {motor_id} cal resistance_stop")

    if not voltages:
        return {"resistance_ohm": 0.0, "error": "No samples collected"}

    # Compute resistance: R = V/I per sample, then average
    resistances = []
    for v, c in zip(voltages, currents):
        if abs(c) > 0.001:  # avoid division by zero
            resistances.append(v / c)

    r_mean = statistics.mean(resistances)
    r_stdev = statistics.stdev(resistances) if len(resistances) > 1 else 0.0
    v_mean = statistics.mean(voltages)
    i_mean = statistics.mean(currents)

    # Robust: also compute via linear regression V = R * I + offset
    # (not implemented here for simplicity)

    # Temperature estimate from resistance (copper: 0.00393/K)
    # R(T) = R_ref * (1 + alpha * (T - T_ref))
    # If R_ref at 25C is known, estimate temperature
    temp_est = None
    r_ref_25c = None  # would come from prior calibration or datasheet

    result = {
        "resistance_ohm": round(r_mean, 6),
        "stdev_ohm": round(r_stdev, 6),
        "voltage_mean": round(v_mean, 4),
        "current_mean": round(i_mean, 4),
        "num_samples": len(resistances),
        "error": None
    }

    print(f"  R = {r_mean:.6f} Ohm (stdev={r_stdev:.6f})")
    print(f"  V = {v_mean:.4f} V, I = {i_mean:.4f} A")
    return result


# ---------------------------------------------------------------------------
# Phase Inductance measurement
# ---------------------------------------------------------------------------
def measure_phase_inductance(iface: MotorSerialInterface, motor_id: int) -> dict:
    """
    Measure phase inductance by applying PWM square wave and measuring di/dt.

    L = V * dt / di  (from V = L * di/dt)

    Returns dict with inductance and stats.
    """
    print(f"\nMotor {motor_id}: Measuring phase inductance...")

    resp = iface.send_command_with_retries(
        f"motor {motor_id} cal inductance_start {PHASE_INDUCTANCE_FREQ_HZ}",
        expected_prefix="OK"
    )
    if not resp:
        return {"inductance_h": 0.0, "error": "Communication failed"}

    time.sleep(0.5)

    inductances = []
    for i in range(PHASE_INDUCTANCE_SAMPLES):
        resp = iface.send_command(f"motor {motor_id} cal inductance_sample")
        if resp:
            try:
                # Expected: "V=12.0 dt=0.00005 di=0.5 L=0.0012"
                parts = resp.replace("V=", "").replace("dt=", "")\
                            .replace("di=", "").replace("L=", "").split()
                if len(parts) >= 4:
                    v = float(parts[0])
                    dt = float(parts[1])
                    di = float(parts[2])
                    l_val = float(parts[3])
                    inductances.append(l_val)
            except (ValueError, IndexError):
                pass

    iface.send_command(f"motor {motor_id} cal inductance_stop")

    if not inductances:
        return {"inductance_h": 0.0, "error": "No samples collected"}

    l_mean = statistics.mean(inductances)
    l_stdev = statistics.stdev(inductances) if len(inductances) > 1 else 0.0

    result = {
        "inductance_h": round(l_mean, 9),
        "stdev_h": round(l_stdev, 9),
        "num_samples": len(inductances),
        "error": None
    }

    print(f"  L = {l_mean * 1e6:.3f} uH (stdev={l_stdev * 1e6:.3f} uH)")
    return result


# ---------------------------------------------------------------------------
# Encoder offset measurement
# ---------------------------------------------------------------------------
def measure_encoder_offset(iface: MotorSerialInterface, motor_id: int) -> dict:
    """
    Measure encoder offset by locking rotor to phase A and reading encoder.

    The electrical angle offset between encoder zero and phase A flux
    is critical for field-oriented control.

    Returns dict with encoder offset in degrees and counts.
    """
    print(f"\nMotor {motor_id}: Measuring encoder offset...")

    resp = iface.send_command_with_retries(
        f"motor {motor_id} cal encoder_offset_start",
        expected_prefix="OK"
    )
    if not resp:
        return {"offset_deg": 0.0, "error": "Communication failed"}

    # Rotor needs time to align to phase A
    print("  Aligning rotor to phase A...")
    time.sleep(2.0)

    offsets_deg = []
    offsets_counts = []

    for i in range(ENCODER_OFFSET_SAMPLES):
        resp = iface.send_command(f"motor {motor_id} cal encoder_offset_read")
        if resp:
            try:
                if resp.startswith("OFFSET:"):
                    parts = resp[7:].split()
                    if len(parts) >= 2:
                        deg = float(parts[0])
                        counts = int(parts[1])
                        offsets_deg.append(deg)
                        offsets_counts.append(counts)
            except (ValueError, IndexError):
                pass
        time.sleep(0.1)

    iface.send_command(f"motor {motor_id} cal encoder_offset_stop")

    if not offsets_deg:
        return {"offset_deg": 0.0, "offset_counts": 0, "error": "No samples"}

    offset_mean = statistics.mean(offsets_deg)
    offset_stdev = statistics.stdev(offsets_deg) if len(offsets_deg) > 1 else 0.0
    count_mean = int(statistics.mean(offsets_counts)) if offsets_counts else 0

    result = {
        "offset_deg": round(offset_mean, 2),
        "offset_electric_deg": round(offset_mean, 2),  # for 1 pole-pair
        "offset_counts": count_mean,
        "stdev_deg": round(offset_stdev, 3),
        "num_samples": len(offsets_deg),
        "error": None
    }

    print(f"  Encoder offset = {offset_mean:.2f} deg (stdev={offset_stdev:.3f})")
    print(f"  Offset counts = {count_mean}")
    return result


# ---------------------------------------------------------------------------
# C header generation
# ---------------------------------------------------------------------------
def generate_c_header(results: dict, output_path: str):
    """Generate motor_params.h from calibration results."""

    header = f"""/**
 * motor_params.h — Auto-generated motor calibration parameters
 *
 * Generated by motor_cal_tool.py on {time.strftime("%Y-%m-%d %H:%M:%S")}
 *
 * Contains per-motor phase resistance, inductance, and encoder offset.
 */

#ifndef MOTOR_PARAMS_H
#define MOTOR_PARAMS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {{
#endif

/* ---------------------------------------------------------------------------
 * Constants
 * --------------------------------------------------------------------------- */
#define MOTOR_NUM_MOTORS    {NUM_MOTORS}

/* ---------------------------------------------------------------------------
 * Per-motor parameters
 * --------------------------------------------------------------------------- */
typedef struct {{
    float phase_resistance_ohm;      /**< Phase-to-neutral resistance */
    float phase_inductance_h;        /**< Phase-to-neutral inductance */
    float encoder_offset_deg;        /**< Electrical encoder offset (degrees) */
    int32_t encoder_offset_counts;   /**< Encoder offset in counts */
    float max_current_a;             /**< Maximum rated current */
    float torque_constant_nm_per_a;  /**< Kt: torque per amp */
    float pole_pairs;                /**< Number of pole pairs */
}} motor_params_t;

/* ---------------------------------------------------------------------------
 * Motor parameters instance
 * --------------------------------------------------------------------------- */
static const motor_params_t g_motor_params[MOTOR_NUM_MOTORS] = {{
"""
    for motor_id in range(NUM_MOTORS):
        key = str(motor_id)
        r = results.get(key, results.get(motor_id, {}))
        r_ohm = r.get("resistance_ohm", 0.1)
        l_h = r.get("inductance_h", 0.0001)
        e_deg = r.get("offset_deg", 0.0)
        e_counts = r.get("offset_counts", 0)

        comma = "," if motor_id < NUM_MOTORS - 1 else ""
        header += (
            f"    {{/* Motor {motor_id} */\n"
            f"        .phase_resistance_ohm = {r_ohm}f,\n"
            f"        .phase_inductance_h   = {l_h}f,\n"
            f"        .encoder_offset_deg   = {e_deg}f,\n"
            f"        .encoder_offset_counts = {e_counts},\n"
            f"        .max_current_a        = 3.0f,\n"
            f"        .torque_constant_nm_per_a = 0.05f,\n"
            f"        .pole_pairs           = 7.0f\n"
            f"    }}{comma}\n"
        )

    header += """};

/* ---------------------------------------------------------------------------
 * Inline accessors
 * --------------------------------------------------------------------------- */

/** Get motor parameters by index. */
static inline const motor_params_t *motor_get_params(uint8_t motor_id)
{
    if (motor_id >= MOTOR_NUM_MOTORS)
        return NULL;
    return &g_motor_params[motor_id];
}

/** Get phase resistance for a motor. */
static inline float motor_get_resistance(uint8_t motor_id)
{
    const motor_params_t *p = motor_get_params(motor_id);
    return p ? p->phase_resistance_ohm : 0.0f;
}

/** Get phase inductance for a motor. */
static inline float motor_get_inductance(uint8_t motor_id)
{
    const motor_params_t *p = motor_get_params(motor_id);
    return p ? p->phase_inductance_h : 0.0f;
}

/** Get encoder offset for a motor. */
static inline float motor_get_encoder_offset(uint8_t motor_id)
{
    const motor_params_t *p = motor_get_params(motor_id);
    return p ? p->encoder_offset_deg : 0.0f;
}

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_PARAMS_H */
"""
    with open(output_path, "w") as f:
        f.write(header)

    print(f"Motor parameters written to {output_path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Motor Calibration Tool — measure R, L, and encoder offset"
    )
    parser.add_argument("--port", "-p", default="/dev/ttyUSB0",
                        help="Serial port to robot")
    parser.add_argument("--baud", "-b", type=int, default=115200,
                        help="Baud rate")
    parser.add_argument("--output", "-o", default="motor_params.h",
                        help="Output C header file")
    parser.add_argument("--motors", "-n", type=int, default=NUM_MOTORS,
                        help="Number of motors to calibrate")
    parser.add_argument("--demo", action="store_true",
                        help="Run with synthetic data for testing")
    parser.add_argument("--skip-resistance", action="store_true",
                        help="Skip phase resistance measurement")
    parser.add_argument("--skip-inductance", action="store_true",
                        help="Skip phase inductance measurement")
    parser.add_argument("--skip-encoder", action="store_true",
                        help="Skip encoder offset measurement")

    args = parser.parse_args()

    results = {}

    if args.demo:
        print("Running with synthetic demo data...")
        import numpy as np
        np.random.seed(42)
        for mid in range(args.motors):
            key = str(mid)
            results[key] = {
                "resistance_ohm": round(0.12 + 0.005 * mid + np.random.normal(0, 0.002), 6),
                "inductance_h": round(0.000120 + 0.000005 * mid + np.random.normal(0, 2e-6), 9),
                "offset_deg": round(45.0 + 2.0 * mid + np.random.normal(0, 0.5), 2),
                "offset_counts": int(2048 * (45.0 + 2.0 * mid) / 360.0),
            }
            # Duplicate into both key types
            results[mid] = results[key]
            print(f"  Motor {mid}: R={results[key]['resistance_ohm']:.6f} "
                  f"L={results[key]['inductance_h']*1e6:.3f}uH "
                  f"offset={results[key]['offset_deg']:.1f}deg")
    else:
        iface = MotorSerialInterface(args.port, args.baud)

        # Enable motor controllers
        iface.send_command("motor all enable")

        for mid in range(args.motors):
            print(f"\n{'='*50}")
            print(f"Calibrating Motor {mid}")
            print(f"{'='*50}")

            motor_results = {}

            # Phase resistance
            if not args.skip_resistance:
                res = measure_phase_resistance(iface, mid)
                if res.get("error"):
                    print(f"  WARNING: {res['error']}")
                motor_results.update(res)
            else:
                motor_results["resistance_ohm"] = 0.12

            # Phase inductance
            if not args.skip_inductance:
                ind = measure_phase_inductance(iface, mid)
                if ind.get("error"):
                    print(f"  WARNING: {ind['error']}")
                motor_results.update(ind)
            else:
                motor_results["inductance_h"] = 0.000150

            # Encoder offset
            if not args.skip_encoder:
                enc = measure_encoder_offset(iface, mid)
                if enc.get("error"):
                    print(f"  WARNING: {enc['error']}")
                motor_results.update(enc)
            else:
                motor_results["offset_deg"] = 45.0
                motor_results["offset_counts"] = 256

            results[mid] = motor_results
            results[str(mid)] = motor_results

        iface.send_command("motor all disable")
        iface.close()

    if not results:
        print("ERROR: No calibration results")
        sys.exit(1)

    # Generate output
    generate_c_header(results, args.output)

    # Save JSON
    json_path = args.output.replace(".h", ".json")
    with open(json_path, "w") as f:
        json.dump(results, f, indent=2)
    print(f"Motor calibration JSON written to {json_path}")

    print("\nMotor calibration complete.")


if __name__ == "__main__":
    main()
