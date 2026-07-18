#!/usr/bin/env python3
"""
Battery Calibration Tool

Full charge->discharge cycle at 0.2C.
Log: voltage, current, temperature at 1-second intervals.
Compute: actual capacity (integrate I*dt), internal resistance (dV/dI at 1s pulses),
OCV-SOC curve (rest 30 min at each 10% SOC).
Output calibration data appended to robot.yaml.

Usage:
    python battery_cal_tool.py --port /dev/ttyUSB0 --capacity 5200 --output robot.yaml
"""

import argparse
import sys
import json
import time
import math
import struct
import csv
import os
from typing import List, Tuple, Optional
from collections import OrderedDict

try:
    import numpy as np
except ImportError:
    np = None
    print("WARNING: numpy not available, using pure Python math")


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
C_RATE = 0.2  # Discharge rate
LOG_INTERVAL_S = 1.0
REST_MINUTES = 30  # Rest at each SOC step
NUM_SOC_POINTS = 11  # 0%, 10%, ..., 100%
PULSE_CURRENT_A = 0.5  # Current pulse for internal resistance
PULSE_DURATION_S = 1.0


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------
class BatterySample:
    """Single battery measurement sample."""

    def __init__(self, voltage_v: float, current_a: float,
                 temperature_c: float, timestamp: float,
                 soc_pct: float = 0.0):
        self.voltage_v = voltage_v
        self.current_a = current_a
        self.temperature_c = temperature_c
        self.timestamp = timestamp
        self.soc_pct = soc_pct

    def to_dict(self):
        return {
            "v": round(self.voltage_v, 4),
            "i": round(self.current_a, 4),
            "t": round(self.temperature_c, 2),
            "ts": round(self.timestamp, 1),
            "soc": round(self.soc_pct, 1)
        }


# ---------------------------------------------------------------------------
# Serial collector
# ---------------------------------------------------------------------------
class BatterySerialInterface:
    """Serial interface to robot battery monitor."""

    def __init__(self, port: str, baud: int = 115200):
        import serial
        self.ser = serial.Serial(port, baud, timeout=1.0)
        self.ser.reset_input_buffer()

    def read_sample(self) -> Optional[BatterySample]:
        """Read one battery sample from serial."""
        line = self.ser.readline()
        if not line:
            return None
        try:
            text = line.decode("ascii", errors="ignore").strip()
        except UnicodeDecodeError:
            return None

        # Format: "BAT:12.345,0.000,25.0" or JSON
        if text.startswith("BAT:") or text.startswith("bat:"):
            parts = text[4:].split(",")
            if len(parts) >= 3:
                return BatterySample(
                    voltage_v=float(parts[0]),
                    current_a=float(parts[1]),
                    temperature_c=float(parts[2]),
                    timestamp=time.time()
                )
        elif text.startswith("{"):
            try:
                data = json.loads(text)
                return BatterySample(
                    voltage_v=data.get("voltage", 0.0),
                    current_a=data.get("current", 0.0),
                    temperature_c=data.get("temperature", 25.0),
                    timestamp=time.time(),
                    soc_pct=data.get("soc", 0.0)
                )
            except json.JSONDecodeError:
                pass
        return None

    def send_command(self, cmd: str):
        """Send a command to the battery monitor."""
        self.ser.write((cmd + "\n").encode("ascii"))

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()


# ---------------------------------------------------------------------------
# Capacity measurement (Coulomb counting)
# ---------------------------------------------------------------------------
def measure_capacity(samples: List[BatterySample]) -> dict:
    """
    Compute actual battery capacity by integrating current over time.

    Capacity (Ah) = integral(I * dt) / 3600
    """
    if len(samples) < 10:
        return {"capacity_ah": 0.0, "error": "Insufficient samples"}

    # Filter to discharge portion (where current < 0 typically)
    # Integrate using trapezoidal rule
    ah = 0.0
    wh = 0.0

    for i in range(1, len(samples)):
        dt = samples[i].timestamp - samples[i - 1].timestamp
        if dt <= 0 or dt > 10:
            continue  # skip gaps

        i_avg = (samples[i].current_a + samples[i - 1].current_a) / 2.0
        v_avg = (samples[i].voltage_v + samples[i - 1].voltage_v) / 2.0

        # Only integrate when discharging (positive current into load)
        # Negative current = charging in our convention
        if i_avg > 0:
            ah += i_avg * dt / 3600.0
            wh += i_avg * v_avg * dt / 3600.0

    # Also compute via cumulative sum for energy
    result = {
        "capacity_ah": round(ah, 3),
        "energy_wh": round(wh, 3),
        "num_samples": len(samples),
        "duration_h": round((samples[-1].timestamp - samples[0].timestamp) / 3600.0, 2),
        "error": None
    }

    if ah > 0:
        result["avg_voltage_v"] = round(wh / ah, 4)
    else:
        result["avg_voltage_v"] = 0.0

    return result


# ---------------------------------------------------------------------------
# Internal resistance (dV/dI at pulse)
# ---------------------------------------------------------------------------
def measure_internal_resistance(samples: List[BatterySample]) -> dict:
    """
    Compute internal resistance from voltage drop during current pulse.
    R = (V_rest - V_load) / (I_load - I_rest)

    Returns dict with R in milliohms.
    """
    if len(samples) < 5:
        return {"r_milliohm": 0.0, "error": "Insufficient samples"}

    # Find pulses: sudden current changes with voltage response
    resistances = []

    for i in range(2, len(samples) - 2):
        di = abs(samples[i + 1].current_a - samples[i - 1].current_a)
        if di > PULSE_CURRENT_A * 0.5:
            # Found a pulse
            v_before = samples[i - 1].voltage_v
            v_during = samples[i + 1].voltage_v
            i_before = samples[i - 1].current_a
            i_during = samples[i + 1].current_a

            dv = v_before - v_during
            di_val = i_during - i_before

            if abs(di_val) > 0.01 and dv > 0:
                r = dv / di_val
                resistances.append(r)

    if not resistances:
        return {"r_milliohm": 0.0, "error": "No pulses detected"}

    r_avg = sum(resistances) / len(resistances) * 1000.0  # convert to mOhm

    if np:
        r_std = float(np.std(resistances)) * 1000.0
    else:
        r_std = 0.0

    return {
        "r_milliohm": round(r_avg, 2),
        "r_stdev_milliohm": round(r_std, 2),
        "num_pulses": len(resistances),
        "error": None
    }


# ---------------------------------------------------------------------------
# OCV-SOC curve (resting voltage at each SOC)
# ---------------------------------------------------------------------------
def measure_ocv_soc(samples: List[BatterySample], nominal_capacity_ah: float) -> list:
    """
    Extract OCV-SOC points by finding resting voltages at known SOC.

    SOC is estimated by Coulomb counting from full charge.
    """
    if len(samples) < 100:
        return []

    # Integrate from start (assumed 100% SOC)
    soc = 100.0  # percent
    ocv_points = []

    # Add 100% SOC point (start)
    ocv_points.append({"soc_pct": 100.0, "ocv_v": round(samples[0].voltage_v, 4)})

    cumulative_ah = 0.0

    for i in range(1, len(samples)):
        dt = samples[i].timestamp - samples[i - 1].timestamp
        if dt <= 0 or dt > 10:
            continue

        i_avg = (samples[i].current_a + samples[i - 1].current_a) / 2.0
        d_ah = abs(i_avg) * dt / 3600.0
        cumulative_ah += d_ah

        soc = 100.0 - (cumulative_ah / nominal_capacity_ah * 100.0)
        soc = max(0.0, min(100.0, soc))

        # Record at each 10% boundary
        for target in range(90, -1, -10):
            already = any(abs(p["soc_pct"] - target) < 1.0 for p in ocv_points)
            if not already and abs(soc - target) < 1.0:
                # Found resting point (check for low current)
                window_start = max(0, i - 5)
                window_end = min(len(samples), i + 5)
                avg_current = sum(abs(samples[j].current_a)
                                  for j in range(window_start, window_end)) / (window_end - window_start)
                if avg_current < 0.05:  # near zero current = rest
                    ocv_points.append({
                        "soc_pct": target,
                        "ocv_v": round(samples[i].voltage_v, 4)
                    })
                    print(f"  OCV at {target}% SOC: {samples[i].voltage_v:.4f} V")
                break

    # Sort by SOC descending
    ocv_points.sort(key=lambda p: -p["soc_pct"])

    return ocv_points


# ---------------------------------------------------------------------------
# YAML output generation
# ---------------------------------------------------------------------------
def append_to_robot_yaml(results: dict, output_path: str):
    """Append battery calibration data to robot.yaml."""
    # Build YAML block
    yaml_block = f"""
# ============================================================
# Battery Calibration Data
# Auto-generated by battery_cal_tool.py on {time.strftime("%Y-%m-%d %H:%M:%S")}
# ============================================================
battery:
  calibration:
    date: {time.strftime("%Y-%m-%d")}
    capacity_ah: {results.get('capacity_ah', 0.0)}
    energy_wh: {results.get('energy_wh', 0.0)}
    internal_resistance_mohm: {results.get('r_milliohm', 0.0)}
    cycles_completed: 0

    # OCV-SOC curve (open-circuit voltage vs state of charge)
    ocv_soc_curve:
"""
    ocv_points = results.get("ocv_curve", [])
    for pt in ocv_points:
        yaml_block += f"      - {{soc: {pt['soc_pct']}, ocv: {pt['ocv_v']}}}\n"

    yaml_block += f"""
    # Voltage limits
    voltage_min_cutoff: 3.0
    voltage_max: {results.get('voltage_max', 4.2)}
    voltage_nominal: {results.get('avg_voltage_v', 3.7)}

    # Temperature limits
    temp_min_charge: 0
    temp_max_charge: 45
    temp_min_discharge: -10
    temp_max_discharge: 60

    # Current limits
    max_charge_current_a: {results.get('capacity_ah', 5.2) * C_RATE:.2f}
    max_discharge_current_a: {results.get('capacity_ah', 5.2) * 2.0:.2f}
"""

    # Append to file
    mode = "a" if os.path.exists(output_path) else "w"
    with open(output_path, mode) as f:
        f.write(yaml_block)

    print(f"Battery calibration appended to {output_path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Battery Calibration Tool — full charge/discharge characterization"
    )
    parser.add_argument("--port", "-p", default="/dev/ttyUSB0",
                        help="Serial port to robot battery monitor")
    parser.add_argument("--baud", "-b", type=int, default=115200,
                        help="Baud rate")
    parser.add_argument("--capacity", "-c", type=float, default=5.2,
                        help="Nominal battery capacity in Ah (default: 5.2)")
    parser.add_argument("--output", "-o", default="robot.yaml",
                        help="Output YAML file")
    parser.add_argument("--demo", action="store_true",
                        help="Run with synthetic data for testing")
    parser.add_argument("--discharge-only", action="store_true",
                        help="Only run discharge cycle (assumes battery is fully charged)")

    args = parser.parse_args()

    if args.demo:
        print("Running with synthetic demo data...")
        samples = []

        if np:
            np.random.seed(42)
            # Simulate discharge from 4.2V to 3.0V over 5 hours
            t_start = time.time()
            capacity_ah = args.capacity
            discharge_a = capacity_ah * C_RATE

            for i in range(3600 * 5):  # 5 hours at 1s intervals
                t = t_start + i
                soc = 100.0 - (i / (3600 * 5)) * 100.0
                # Simulated OCV with hysteresis and noise
                v_ocv = 4.2 - (100.0 - soc) * 0.012
                v_load = v_ocv - discharge_a * 0.05  # R ~50mOhm
                noise = np.random.normal(0, 0.002)
                temp = 25.0 + np.random.normal(0, 0.5)

                sample = BatterySample(
                    voltage_v=round(v_load + noise, 4),
                    current_a=round(discharge_a + np.random.normal(0, 0.01), 4),
                    temperature_c=round(temp, 2),
                    timestamp=t,
                    soc_pct=max(0, soc)
                )
                samples.append(sample)
                if i % 600 == 0:
                    print(f"  t={i//3600}h {i%3600//60}m: V={sample.voltage_v:.4f} "
                          f"I={sample.current_a:.3f} SOC={soc:.1f}%")
        else:
            print("ERROR: numpy required for demo mode")
            sys.exit(1)

    else:
        print("Starting battery calibration...")
        iface = BatterySerialInterface(args.port, args.baud)

        # Start data logging
        iface.send_command("bat log start")
        print("Logging started. Ensure battery is discharging at 0.2C.")

        samples = []
        start_time = time.time()

        try:
            while True:
                sample = iface.read_sample()
                if sample:
                    samples.append(sample)
                    if len(samples) % 60 == 0:
                        elapsed_h = (sample.timestamp - start_time) / 3600.0
                        print(f"  t={elapsed_h:.2f}h: V={sample.voltage_v:.4f} "
                              f"I={sample.current_a:.3f} T={sample.temperature_c:.1f}C")

                # Check for termination (voltage below cutoff)
                if sample and sample.voltage_v < 3.0:
                    print("  Voltage below 3.0V cutoff, stopping.")
                    break

                time.sleep(LOG_INTERVAL_S)

        except KeyboardInterrupt:
            print("\nLogging stopped by user.")

        iface.send_command("bat log stop")
        iface.close()

    if len(samples) < 10:
        print("ERROR: Too few samples collected")
        sys.exit(1)

    print(f"\nProcessing {len(samples)} samples...")

    # Compute capacity
    cap_result = measure_capacity(samples)
    print(f"  Measured capacity: {cap_result.get('capacity_ah', 0):.3f} Ah")
    print(f"  Energy: {cap_result.get('energy_wh', 0):.1f} Wh")

    # Measure internal resistance
    r_result = measure_internal_resistance(samples)
    print(f"  Internal resistance: {r_result.get('r_milliohm', 0):.2f} mOhm")

    # OCV-SOC curve
    print("  Extracting OCV-SOC curve...")
    ocv_curve = measure_ocv_soc(samples, args.capacity)
    print(f"  OCV points collected: {len(ocv_curve)}")

    # Collect results
    results = {
        "capacity_ah": cap_result.get("capacity_ah", 0.0),
        "energy_wh": cap_result.get("energy_wh", 0.0),
        "avg_voltage_v": cap_result.get("avg_voltage_v", 0.0),
        "r_milliohm": r_result.get("r_milliohm", 0.0),
        "voltage_max": max(s.voltage_v for s in samples[:100]),
        "voltage_min": min(s.voltage_v for s in samples[-100:]),
        "ocv_curve": ocv_curve,
    }

    # Save raw data as CSV
    csv_path = args.output.replace(".yaml", "_battery_log.csv")
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["timestamp", "voltage", "current",
                                                "temperature", "soc"])
        writer.writeheader()
        for s in samples:
            writer.writerow({
                "timestamp": s.timestamp,
                "voltage": s.voltage_v,
                "current": s.current_a,
                "temperature": s.temperature_c,
                "soc": s.soc_pct
            })
    print(f"Raw data written to {csv_path}")

    # Generate JSON
    json_path = args.output.replace(".yaml", "_battery_cal.json")
    with open(json_path, "w") as f:
        json.dump(results, f, indent=2)
    print(f"Calibration JSON written to {json_path}")

    # Append to YAML
    append_to_robot_yaml(results, args.output)

    print("\nBattery calibration complete.")


if __name__ == "__main__":
    main()
