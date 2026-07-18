#!/usr/bin/env python3
"""
ToF Calibration Tool

Reads ranging data at 5 known distances from multiple zones.
Performs linear regression per zone: actual = a * measured + b.
Computes R^2. Flags zones with R^2 < 0.95 for re-calibration.
Outputs zone calibration table as C header.

Usage:
    python tof_cal_tool.py --port /dev/ttyUSB0 --output tof_calibration.h
"""

import argparse
import sys
import json
import time
import math
import struct
import numpy as np
from typing import List, Tuple


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
KNOWN_DISTANCES_MM = [100, 500, 1000, 2000, 3500]  # 5 known distances
SAMPLES_PER_DISTANCE = 50
R_SQUARED_THRESHOLD = 0.95
MAX_ZONES = 8
NUM_DISTANCES = 5

# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------
class ToFReading:
    """Single ToF ranging reading."""

    def __init__(self, zone: int, distance_mm: float, confidence: float = 1.0,
                 timestamp: float = 0.0):
        self.zone = zone
        self.distance_mm = distance_mm
        self.confidence = confidence
        self.timestamp = timestamp

    @classmethod
    def from_json(cls, data: dict) -> "ToFReading":
        return cls(
            zone=data.get("zone", 0),
            distance_mm=data.get("distance_mm", 0.0),
            confidence=data.get("confidence", 1.0),
            timestamp=data.get("ts", 0.0)
        )


# ---------------------------------------------------------------------------
# Serial collector
# ---------------------------------------------------------------------------
def collect_zone_readings(port: str, baud: int, zone: int,
                          actual_mm: int, num_samples: int,
                          timeout: float = 15.0) -> List[float]:
    """Collect ToF readings for a zone at a known distance."""
    import serial

    ser = serial.Serial(port, baud, timeout=0.5)
    readings = []
    start = time.time()

    print(f"  Zone {zone}: reading at {actual_mm} mm ({num_samples} samples)...")

    while len(readings) < num_samples:
        if time.time() - start > timeout:
            break

        line = ser.readline()
        if not line:
            continue

        try:
            text = line.decode("ascii", errors="ignore").strip()
        except UnicodeDecodeError:
            continue

        # Parse JSON or CSV ToF output
        if text.startswith("{"):
            try:
                data = json.loads(text)
                if data.get("zone") == zone:
                    readings.append(data.get("distance_mm", 0.0))
            except json.JSONDecodeError:
                pass
        elif text.startswith(f"TOF{zone}:") or text.startswith(f"tof{zone}:"):
            parts = text.split(":")[1].strip().split(",")
            if parts:
                try:
                    readings.append(float(parts[0]))
                except ValueError:
                    pass

        if len(readings) % 10 == 0:
            print(f"    {len(readings)}/{num_samples}")

    ser.close()
    return readings


def interactive_collection(port: str, baud: int, num_zones: int) -> dict:
    """Interactive collection of calibration data."""
    print("\nToF Calibration — Interactive Collection")
    print("==========================================")
    print(f"Place the robot at known distances from a flat surface.")
    print(f"Distances to measure: {KNOWN_DISTANCES_MM} mm\n")

    all_data = {}  # {zone: {distance_mm: [readings]}}

    for zone in range(num_zones):
        all_data[zone] = {}
        for dist_mm in KNOWN_DISTANCES_MM:
            input(f"Place Zone {zone} at {dist_mm} mm, then press Enter...")
            readings = collect_zone_readings(port, baud, zone, dist_mm,
                                              SAMPLES_PER_DISTANCE)
            if readings:
                all_data[zone][dist_mm] = readings
                mean_val = np.mean(readings)
                print(f"    Mean reading: {mean_val:.1f} mm")
            else:
                print(f"    WARNING: No readings collected!")

    return all_data


# ---------------------------------------------------------------------------
# Linear regression per zone
# ---------------------------------------------------------------------------
def calibrate_zone(readings_by_distance: dict) -> dict:
    """
    Perform linear regression for a single zone.
    Returns: {slope, intercept, r_squared, points_used, errors}
    """
    measured = []
    actual = []

    for dist_mm in KNOWN_DISTANCES_MM:
        if dist_mm not in readings_by_distance:
            continue
        vals = readings_by_distance[dist_mm]
        if not vals:
            continue
        # Use mean of readings at this distance
        measured.append(np.mean(vals))
        actual.append(float(dist_mm))

    if len(measured) < 3:
        return {
            "slope": 1.0, "intercept": 0.0, "r_squared": 0.0,
            "points_used": len(measured), "error": "Insufficient data"
        }

    # Linear regression: actual = a * measured + b
    x = np.array(measured)
    y = np.array(actual)

    A = np.vstack([x, np.ones_like(x)]).T
    slope, intercept = np.linalg.lstsq(A, y, rcond=None)[0]

    # R-squared
    y_pred = slope * x + intercept
    ss_res = np.sum((y - y_pred) ** 2)
    ss_tot = np.sum((y - np.mean(y)) ** 2)
    r_squared = 1.0 - (ss_res / ss_tot) if ss_tot > 0 else 0.0

    # Residual standard error
    n = len(y)
    rse = math.sqrt(ss_res / (n - 2)) if n > 2 else 0.0

    result = {
        "slope": float(slope),
        "intercept": float(intercept),
        "r_squared": float(r_squared),
        "rse_mm": float(rse),
        "points_used": n,
        "error": None
    }

    return result


def calibrate_all_zones(data: dict, num_zones: int) -> dict:
    """Run calibration for all zones."""
    results = {}

    for zone in range(num_zones):
        if zone not in data:
            print(f"  Zone {zone}: No data, skipping")
            continue

        cal = calibrate_zone(data[zone])
        results[zone] = cal

        status = "OK" if cal["r_squared"] >= R_SQUARED_THRESHOLD else "WARN"
        print(f"  Zone {zone}: a={cal['slope']:.6f}  b={cal['intercept']:.2f}  "
              f"R^2={cal['r_squared']:.4f}  RSE={cal.get('rse_mm', 0):.2f}mm  [{status}]")

        if cal["r_squared"] < R_SQUARED_THRESHOLD:
            print(f"    *** WARNING: Zone {zone} R^2 < {R_SQUARED_THRESHOLD}, "
                  f"re-calibration recommended!")

    return results


# ---------------------------------------------------------------------------
# C header generation
# ---------------------------------------------------------------------------
def generate_c_header(cal_results: dict, num_zones: int,
                      output_path: str):
    """Generate C calibration header file."""

    header = f"""/**
 * tof_calibration.h — Auto-generated ToF zone calibration table
 *
 * Generated by tof_cal_tool.py on {time.strftime("%Y-%m-%d %H:%M:%S")}
 *
 * Calibration formula: actual_mm = slope * measured_mm + intercept
 * Calibration performed at {NUM_DISTANCES} known distances:
 *   {KNOWN_DISTANCES_MM}
 *
 * Zones with R^2 < {R_SQUARED_THRESHOLD} are flagged and require re-calibration.
 */

#ifndef TOF_CALIBRATION_H
#define TOF_CALIBRATION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {{
#endif

/* ---------------------------------------------------------------------------
 * Constants
 * --------------------------------------------------------------------------- */
#define TOF_CALIBRATION_NUM_ZONES       {num_zones}
#define TOF_CALIBRATION_R2_THRESHOLD    {R_SQUARED_THRESHOLD}f

/* ---------------------------------------------------------------------------
 * Per-zone calibration
 * --------------------------------------------------------------------------- */
typedef struct {{
    float    slope;              /**< Calibration slope (a) */
    float    intercept;          /**< Calibration intercept (b), in mm */
    float    r_squared;          /**< Goodness of fit */
    float    rse_mm;             /**< Residual standard error (mm) */
    uint8_t  valid;              /**< 1 if R^2 >= threshold */
}} tof_zone_calibration_t;

/* ---------------------------------------------------------------------------
 * Global calibration table
 * --------------------------------------------------------------------------- */
typedef struct {{
    tof_zone_calibration_t zones[TOF_CALIBRATION_NUM_ZONES];
    uint16_t               calibration_date;  /**< YYYYMMDD */
}} tof_calibration_t;

/* ---------------------------------------------------------------------------
 * Calibration table instance
 * --------------------------------------------------------------------------- */
static const tof_calibration_t g_tof_calibration = {{
    .zones = {{
"""
    for zone in range(num_zones):
        if zone in cal_results:
            cal = cal_results[zone]
            # Provide defaults for missing zones
            slope = cal.get("slope", 1.0)
            intercept = cal.get("intercept", 0.0)
            r2 = cal.get("r_squared", 1.0)
            rse = cal.get("rse_mm", 0.0)
            valid = 1 if r2 >= R_SQUARED_THRESHOLD else 0
        else:
            slope = 1.0
            intercept = 0.0
            r2 = 1.0
            rse = 0.0
            valid = 0

        comma = "," if zone < num_zones - 1 else ""
        header += (
            f"        {{.slope = {slope:.10f}f, .intercept = {intercept:.4f}f,\n"
            f"         .r_squared = {r2:.6f}f, .rse_mm = {rse:.4f}f, .valid = {valid}}}{comma}\n"
        )

    import time as _time
    cal_date = int(_time.strftime("%Y%m%d"))

    header += f"""    }},
    .calibration_date = {cal_date},
}};

/* ---------------------------------------------------------------------------
 * Inline calibration apply
 * --------------------------------------------------------------------------- */

/** Apply zone calibration to raw distance reading. */
static inline float tof_apply_calibration(uint8_t zone, float raw_mm)
{{
    if (zone >= TOF_CALIBRATION_NUM_ZONES)
        return raw_mm;
    return g_tof_calibration.zones[zone].slope * raw_mm
         + g_tof_calibration.zones[zone].intercept;
}}

/** Check if zone calibration is valid. */
static inline int tof_is_calibration_valid(uint8_t zone)
{{
    if (zone >= TOF_CALIBRATION_NUM_ZONES)
        return 0;
    return g_tof_calibration.zones[zone].valid ? 1 : 0;
}}

/** Get pointer to calibration table. */
static inline const tof_calibration_t *tof_get_calibration(void)
{{
    return &g_tof_calibration;
}}

#ifdef __cplusplus
}}
#endif

#endif /* TOF_CALIBRATION_H */
"""
    with open(output_path, "w") as f:
        f.write(header)

    print(f"Calibration header written to {output_path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="ToF Calibration Tool — Time-of-Flight sensor calibration"
    )
    parser.add_argument("--port", "-p", default="/dev/ttyUSB0",
                        help="Serial port to robot UART")
    parser.add_argument("--baud", "-b", type=int, default=115200,
                        help="Serial baud rate")
    parser.add_argument("--output", "-o", default="tof_calibration.h",
                        help="Output C header file")
    parser.add_argument("--zones", "-z", type=int, default=4,
                        help="Number of ToF zones (default: 4)")
    parser.add_argument("--input", "-i",
                        help="Input JSON file with calibration data")
    parser.add_argument("--demo", action="store_true",
                        help="Run with synthetic demo data")

    args = parser.parse_args()

    data = {}

    if args.demo:
        print("Running with synthetic demo data...")
        np.random.seed(42)
        for zone in range(args.zones):
            data[zone] = {}
            for dist in KNOWN_DISTANCES_MM:
                # Simulate readings with slight nonlinearity
                raw = dist * (1.0 + 0.02 * (zone + 1)) + (zone * 5 - 10)
                noise = np.random.normal(0, 3, SAMPLES_PER_DISTANCE)
                data[zone][dist] = list(raw + noise.tolist())

    elif args.input:
        print(f"Loading calibration data from {args.input}...")
        with open(args.input, "r") as f:
            loaded = json.load(f)
            for zone_str, distances in loaded.items():
                zone = int(zone_str)
                data[zone] = {int(k): v for k, v in distances.items()}
    else:
        # Interactive serial collection
        print("Starting interactive collection...")
        print("Ensure the robot is connected and sending ToF data.")
        data = interactive_collection(args.port, args.baud, args.zones)

    if not data:
        print("ERROR: No calibration data collected")
        sys.exit(1)

    print(f"\nCalibrating {len(data)} zones...")
    cal_results = calibrate_all_zones(data, args.zones)

    # Print summary
    zones_below = [z for z, c in cal_results.items()
                   if c.get("r_squared", 1.0) < R_SQUARED_THRESHOLD]
    if zones_below:
        print(f"\n*** WARNING: Zones {zones_below} need re-calibration "
              f"(R^2 < {R_SQUARED_THRESHOLD})")
    else:
        print(f"\nAll zones passed calibration check (R^2 >= {R_SQUARED_THRESHOLD})")

    # Generate C header
    generate_c_header(cal_results, args.zones, args.output)

    # Save JSON
    json_path = args.output.replace(".h", ".json")
    with open(json_path, "w") as f:
        json.dump(cal_results, f, indent=2)
    print(f"Calibration JSON written to {json_path}")

    print("\nToF calibration complete.")


if __name__ == "__main__":
    main()
