#!/usr/bin/env python3
"""
IMU Calibration Tool

Reads 1000+ samples from robot UART.
Computes 6-parameter accelerometer calibration (scale + offset per axis).
Computes 3-parameter gyroscope calibration (offset only).
Uses Levenberg-Marquardt optimization for accel calibration.
Outputs calibration struct as C header file.

Usage:
    python imu_cal_tool.py --port /dev/ttyUSB0 --baud 115200 --output imu_calibration.h
"""

import argparse
import sys
import struct
import json
import time
import math
import numpy as np
from scipy.optimize import least_squares


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
G_REF = 9.80665  # m/s^2
NUM_SAMPLES = 1000
ACCEL_SAMPLES_PER_POSE = 200
NUM_POSES = 6  # 6-axis static poses


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------
class IMUSample:
    """Represents a single IMU sensor reading."""

    def __init__(self, ax=0.0, ay=0.0, az=0.0, gx=0.0, gy=0.0, gz=0.0,
                 temperature=0.0, timestamp=0):
        self.ax = ax
        self.ay = ay
        self.az = az
        self.gx = gx
        self.gy = gy
        self.gz = gz
        self.temperature = temperature
        self.timestamp = timestamp

    @classmethod
    def from_bytes(cls, data: bytes) -> "IMUSample":
        """Parse binary IMU sample (24-byte frame)."""
        if len(data) < 24:
            return None
        fmt = "<6fHh"  # 6 floats (accel xyz, gyro xyz) + uint16 temp + int16 ts
        values = struct.unpack_from(fmt, data, 0)
        return cls(
            ax=values[0], ay=values[1], az=values[2],
            gx=values[3], gy=values[4], gz=values[5],
            temperature=float(values[6]) / 100.0,
            timestamp=values[7]
        )

    @classmethod
    def from_csv_line(cls, line: str) -> "IMUSample":
        """Parse CSV line."""
        parts = line.strip().split(",")
        if len(parts) < 6:
            return None
        return cls(
            ax=float(parts[0]), ay=float(parts[1]), az=float(parts[2]),
            gx=float(parts[3]), gy=float(parts[4]), gz=float(parts[5]),
            temperature=float(parts[6]) if len(parts) > 6 else 0.0
        )

    def to_dict(self):
        return {
            "ax": self.ax, "ay": self.ay, "az": self.az,
            "gx": self.gx, "gy": self.gy, "gz": self.gz,
            "temp": self.temperature, "ts": self.timestamp
        }


# ---------------------------------------------------------------------------
# Serial reader
# ---------------------------------------------------------------------------
def read_samples_serial(port: str, baud: int, num_samples: int,
                        timeout: float = 10.0) -> list:
    """Read IMU samples from robot serial port."""
    import serial

    ser = serial.Serial(port, baud, timeout=0.1)
    samples = []
    start_time = time.time()

    print(f"Collecting {num_samples} IMU samples from {port}...")

    while len(samples) < num_samples:
        if time.time() - start_time > timeout:
            print(f"Timeout after {len(samples)} samples")
            break

        line = ser.readline()
        if not line:
            continue

        try:
            line_str = line.decode("ascii", errors="ignore").strip()
        except UnicodeDecodeError:
            continue

        if line_str.startswith("IMU:") or line_str.startswith("imu:"):
            data_str = line_str[4:]
            sample = IMUSample.from_csv_line(data_str)
            if sample:
                samples.append(sample)
                if len(samples) % 100 == 0:
                    print(f"  Collected {len(samples)}/{num_samples}")
        elif len(line) >= 24:
            sample = IMUSample.from_bytes(line)
            if sample:
                samples.append(sample)
                if len(samples) % 100 == 0:
                    print(f"  Collected {len(samples)}/{num_samples}")

    ser.close()
    print(f"Total samples: {len(samples)}")
    return samples


# ---------------------------------------------------------------------------
# Accelerometer calibration (6-parameter)
# Each axis has scale factor S_i and offset O_i:
#   corrected = S_i * (raw - O_i)
# For 6-parameter, we have [Sx, Sy, Sz, Ox, Oy, Oz]
# ---------------------------------------------------------------------------
def accel_calibration_residuals(params, samples, g_ref):
    """
    Residuals for accel calibration.
    param: [scale_x, scale_y, scale_z, offset_x, offset_y, offset_z]
    For each sample: residual = ||corrected||^2 - g_ref^2
    """
    sx, sy, sz, ox, oy, oz = params

    residuals = []
    for s in samples:
        cx = sx * (s.ax - ox)
        cy = sy * (s.ay - oy)
        cz = sz * (s.az - oz)
        squared_norm = cx * cx + cy * cy + cz * cz
        residuals.append(squared_norm - g_ref * g_ref)

    return np.array(residuals)


def calibrate_accelerometer(samples, g_ref=G_REF):
    """
    Compute 6-parameter accelerometer calibration via Levenberg-Marquardt.
    Returns dict with scale, offset, and stats.
    """
    # Initial guess: unity scale, zero offset
    x0 = np.array([1.0, 1.0, 1.0, 0.0, 0.0, 0.0])

    print("Running Levenberg-Marquardt optimization for accel calibration...")
    result = least_squares(
        accel_calibration_residuals, x0,
        args=(samples, g_ref),
        method="lm",
        max_nfev=200,
        ftol=1e-12,
        xtol=1e-12,
        gtol=1e-12
    )

    sx, sy, sz, ox, oy, oz = result.x

    # Compute RMS residual
    rms = np.sqrt(np.mean(result.fun ** 2))
    rms_mg = math.sqrt(rms) / g_ref * 1000.0  # in mg

    print(f"  Accel calibration converged: {result.nfev} iterations")
    print(f"  Scale:   X={sx:.6f}  Y={sy:.6f}  Z={sz:.6f}")
    print(f"  Offset:  X={ox:.4f}  Y={oy:.4f}  Z={oz:.4f}  (m/s^2)")
    print(f"  RMS residual: {rms:.6f} (m/s^2)^2 = {rms_mg:.2f} mg")

    # Apply calibration and compute statistics
    corrected_norms = []
    for s in samples:
        cx = sx * (s.ax - ox)
        cy = sy * (s.ay - oy)
        cz = sz * (s.az - oz)
        corrected_norms.append(math.sqrt(cx * cx + cy * cy + cz * cz))

    cal = {
        "scale": {"x": float(sx), "y": float(sy), "z": float(sz)},
        "offset": {"x": float(ox), "y": float(oy), "z": float(oz)},
        "rms_error_mg": float(rms_mg),
        "mean_norm_after": float(np.mean(corrected_norms)),
        "std_norm_after": float(np.std(corrected_norms)),
        "g_ref": g_ref
    }

    return cal


def calibrate_gyroscope(samples):
    """
    Compute 3-parameter gyroscope calibration (offset only).
    For static samples, gyro should read zero.
    """
    gx_vals = np.array([s.gx for s in samples])
    gy_vals = np.array([s.gy for s in samples])
    gz_vals = np.array([s.gz for s in samples])

    offset_x = float(np.mean(gx_vals))
    offset_y = float(np.mean(gy_vals))
    offset_z = float(np.mean(gz_vals))

    std_x = float(np.std(gx_vals))
    std_y = float(np.std(gy_vals))
    std_z = float(np.std(gz_vals))

    print(f"  Gyro offset: X={offset_x:.4f}  Y={offset_y:.4f}  Z={offset_z:.4f} (rad/s)")
    print(f"  Gyro std:    X={std_x:.6f}  Y={std_y:.6f}  Z={std_z:.6f} (rad/s)")

    return {
        "offset": {"x": offset_x, "y": offset_y, "z": offset_z},
        "std": {"x": float(std_x), "y": float(std_y), "z": float(std_z)}
    }


# ---------------------------------------------------------------------------
# C header generation
# ---------------------------------------------------------------------------
def generate_c_header(accel_cal, gyro_cal, output_path: str):
    """Generate C header file with calibration structs."""

    header = f"""/**
 * imu_calibration.h — Auto-generated IMU calibration parameters
 *
 * Generated by imu_cal_tool.py on {time.strftime("%Y-%m-%d %H:%M:%S")}
 *
 * Accelerometer: 6-parameter calibration (scale + offset per axis)
 *   corrected = scale[i] * (raw[i] - offset[i])
 *
 * Gyroscope: 3-parameter offset calibration
 *   corrected = raw[i] - offset[i]
 *
 * Calibration performed with reference gravity = {G_REF} m/s^2
 */

#ifndef IMU_CALIBRATION_H
#define IMU_CALIBRATION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {{
#endif

/* ---------------------------------------------------------------------------
 * IMU Calibration Structure
 * --------------------------------------------------------------------------- */
typedef struct {{
    /* Accelerometer scale factors (unitless) */
    float accel_scale_x;
    float accel_scale_y;
    float accel_scale_z;

    /* Accelerometer offsets (m/s^2) */
    float accel_offset_x;
    float accel_offset_y;
    float accel_offset_z;

    /* Gyroscope offsets (rad/s) */
    float gyro_offset_x;
    float gyro_offset_y;
    float gyro_offset_z;

    /* Calibration quality metrics */
    float rms_error_mg;       /**< RMS residual in milli-gravity */
    float calib_gravity_ref;  /**< Reference gravity used */
    uint32_t crc32;           /**< Integrity check */
}} imu_calibration_t;

/* ---------------------------------------------------------------------------
 * Calibration instance
 * --------------------------------------------------------------------------- */
static const imu_calibration_t g_imu_calibration = {{
    /* Accelerometer scale */
    .accel_scale_x = {accel_cal["scale"]["x"]:.10f}f,
    .accel_scale_y = {accel_cal["scale"]["y"]:.10f}f,
    .accel_scale_z = {accel_cal["scale"]["z"]:.10f}f,

    /* Accelerometer offsets */
    .accel_offset_x = {accel_cal["offset"]["x"]:.10f}f,
    .accel_offset_y = {accel_cal["offset"]["y"]:.10f}f,
    .accel_offset_z = {accel_cal["offset"]["z"]:.10f}f,

    /* Gyroscope offsets */
    .gyro_offset_x = {gyro_cal["offset"]["x"]:.10f}f,
    .gyro_offset_y = {gyro_cal["offset"]["y"]:.10f}f,
    .gyro_offset_z = {gyro_cal["offset"]["z"]:.10f}f,

    /* Quality metrics */
    .rms_error_mg = {accel_cal["rms_error_mg"]:.4f}f,
    .calib_gravity_ref = {G_REF}f,
    .crc32 = 0
}};

/* ---------------------------------------------------------------------------
 * Inline calibration helpers
 * --------------------------------------------------------------------------- */

/** Apply accelerometer calibration to raw sample. */
static inline void imu_calibrate_accel(float raw[3], float corrected[3])
{{
    corrected[0] = g_imu_calibration.accel_scale_x * (raw[0] - g_imu_calibration.accel_offset_x);
    corrected[1] = g_imu_calibration.accel_scale_y * (raw[1] - g_imu_calibration.accel_offset_y);
    corrected[2] = g_imu_calibration.accel_scale_z * (raw[2] - g_imu_calibration.accel_offset_z);
}}

/** Apply gyroscope calibration to raw sample. */
static inline void imu_calibrate_gyro(float raw[3], float corrected[3])
{{
    corrected[0] = raw[0] - g_imu_calibration.gyro_offset_x;
    corrected[1] = raw[1] - g_imu_calibration.gyro_offset_y;
    corrected[2] = raw[2] - g_imu_calibration.gyro_offset_z;
}}

/** Get pointer to calibration struct. */
static inline const imu_calibration_t *imu_get_calibration(void)
{{
    return &g_imu_calibration;
}}

#ifdef __cplusplus
}}
#endif

#endif /* IMU_CALIBRATION_H */
"""
    with open(output_path, "w") as f:
        f.write(header)

    print(f"Calibration header written to {output_path}")


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------
def validate_calibration(samples, accel_cal, gyro_cal):
    """Apply calibration and print validation statistics."""
    print("\nValidation:")
    norms_before = []
    norms_after = []

    for s in samples[:500]:
        # Before
        nb = math.sqrt(s.ax ** 2 + s.ay ** 2 + s.az ** 2)
        norms_before.append(nb)

        # After
        cx = accel_cal["scale"]["x"] * (s.ax - accel_cal["offset"]["x"])
        cy = accel_cal["scale"]["y"] * (s.ay - accel_cal["offset"]["y"])
        cz = accel_cal["scale"]["z"] * (s.az - accel_cal["offset"]["z"])
        na = math.sqrt(cx * cx + cy * cy + cz * cz)
        norms_after.append(na)

    print(f"  Accel norm before: mean={np.mean(norms_before):.4f}, "
          f"std={np.std(norms_before):.4f}")
    print(f"  Accel norm after:  mean={np.mean(norms_after):.4f}, "
          f"std={np.std(norms_after):.4f}")
    print(f"  Deviation from {G_REF}: {abs(np.mean(norms_after) - G_REF):.4f} m/s^2")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="IMU Calibration Tool — IMU calibration parameter computation"
    )
    parser.add_argument("--port", "-p", default="/dev/ttyUSB0",
                        help="Serial port to robot UART")
    parser.add_argument("--baud", "-b", type=int, default=115200,
                        help="Serial baud rate")
    parser.add_argument("--output", "-o", default="imu_calibration.h",
                        help="Output C header file path")
    parser.add_argument("--samples", "-n", type=int, default=NUM_SAMPLES,
                        help=f"Number of samples to collect (default: {NUM_SAMPLES})")
    parser.add_argument("--input", "-i",
                        help="Input CSV/log file instead of serial port")
    parser.add_argument("--no-serial", action="store_true",
                        help="Skip serial collection, use placeholder data")

    args = parser.parse_args()

    # Collect samples
    samples = []
    if args.no_serial:
        print("Generating synthetic IMU data for testing...")
        np.random.seed(42)
        for _ in range(args.samples):
            # Simulate near-perfect IMU with slight noise
            ax = np.random.normal(0, 0.05)
            ay = np.random.normal(0, 0.05)
            az = np.random.normal(G_REF, 0.05)
            gx = np.random.normal(0, 0.002)
            gy = np.random.normal(0, 0.002)
            gz = np.random.normal(0, 0.002)
            samples.append(IMUSample(ax, ay, az, gx, gy, gz))
    elif args.input:
        print(f"Reading samples from {args.input}...")
        with open(args.input, "r") as f:
            for line in f:
                s = IMUSample.from_csv_line(line)
                if s:
                    samples.append(s)
                    if len(samples) >= args.samples:
                        break
    else:
        samples = read_samples_serial(args.port, args.baud, args.samples)

    if len(samples) < 10:
        print("ERROR: Too few samples collected")
        sys.exit(1)

    print(f"\nProcessing {len(samples)} samples...")

    # Run calibrations
    accel_cal = calibrate_accelerometer(samples)
    gyro_cal = calibrate_gyroscope(samples)

    # Validate
    validate_calibration(samples, accel_cal, gyro_cal)

    # Generate output
    generate_c_header(accel_cal, gyro_cal, args.output)

    # Also save JSON for reference
    json_path = args.output.replace(".h", ".json")
    with open(json_path, "w") as f:
        json.dump({
            "accel": accel_cal,
            "gyro": gyro_cal,
            "sample_count": len(samples),
            "generated": time.strftime("%Y-%m-%d %H:%M:%S")
        }, f, indent=2)
    print(f"Calibration JSON written to {json_path}")

    print("\nIMU calibration complete.")


if __name__ == "__main__":
    main()
