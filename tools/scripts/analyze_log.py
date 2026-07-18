#!/usr/bin/env python3
"""
Log Analyzer — Parse crash dump logs, decode fault info, and generate HTML report

Features:
- Parse crash dump logs: extract PC, LR, fault type, and stack trace
- Look up symbols from .elf via arm-none-eabi-addr2line
- Print decoded call stack
- Correlate with event logs to find root cause
- Generate HTML report

Usage:
    python analyze_log.py --elf build/app.elf --log crash.log --output report.html
    python analyze_log.py --elf build/app.elf --log crash.log --interactive
"""

import argparse
import re
import os
import sys
import subprocess
import json
import time
from typing import List, Dict, Optional, Tuple
from collections import OrderedDict
from datetime import datetime
from pathlib import Path


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
ARM_REGISTERS = [
    "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7",
    "r8", "r9", "r10", "r11", "r12", "sp", "lr", "pc",
    "xpsr", "msp", "psp", "primask", "faultmask", "basepri", "control"
]

FAULT_TYPES = {
    "0": "HardFault",
    "1": "MemManage (MPU violation)",
    "2": "BusFault (precise)",
    "3": "BusFault (imprecise)",
    "4": "UsageFault (undefined instruction)",
    "5": "UsageFault (Unaligned access)",
    "6": "UsageFault (No FPU)",
    "7": "DebugMonitor",
    "8": "HardFault (vector catch)",
}

HARDWARE_DESCRIPTIONS = {
    "HardFault": "Unexpected fault with no configurable handler. Usually caused by: "
                 "dereferencing NULL pointer, double free, stack overflow, or "
                 "accessing invalid memory region.",
    "MemManage": "MPU (Memory Protection Unit) violation. Task attempted to access "
                 "memory outside its assigned regions.",
    "BusFault": "Bus error: accessing invalid memory address (precise: address known, "
                "imprecise: address unknown). Common causes: wrong peripheral address, "
                "NULL pointer dereference.",
    "UsageFault": "Usage fault: undefined instruction, unaligned access, or attempt "
                  "to use FPU without enabling it.",
}


# ---------------------------------------------------------------------------
# Crash log parser
# ---------------------------------------------------------------------------
class CrashLog:
    """Parsed crash dump data."""

    def __init__(self):
        self.fault_type: str = "Unknown"
        self.pc: int = 0
        self.lr: int = 0
        self.sp: int = 0
        self.psr: int = 0
        self.stack_frames: List[int] = []
        self.registers: Dict[str, int] = {}
        self.fault_status_regs: Dict[str, int] = {}
        self.raw_lines: List[str] = []
        self.event_log_entries: List[str] = []
        self.task_name: str = ""
        self.timestamp: str = ""

    def __repr__(self):
        return (f"<CrashLog fault={self.fault_type} "
                f"PC=0x{self.pc:08X} LR=0x{self.lr:08X}>")


class CrashLogParser:
    """Parse raw text crash dump into CrashLog object."""

    @staticmethod
    def parse_hex_value(text: str) -> int:
        """Parse hex value from string, stripping prefixes."""
        text = text.strip().replace("0x", "").replace("0X", "")
        try:
            return int(text, 16)
        except ValueError:
            return 0

    @staticmethod
    def parse_registers(line: str) -> Tuple[Optional[str], Optional[int]]:
        """Extract register name and value from log line."""
        patterns = [
            r"(r[0-9]|sp|lr|pc|xpsr|psr|msp|psp|primask|basepri|control)\s*[:=]\s*(0x[0-9a-fA-F]+)",
            r"(r[0-9]|sp|lr|pc|xpsr|psr)\s*=\s*(0x[0-9a-fA-F]+)",
            r"(r[0-9]|sp|lr|pc)[:\s]+(0x[0-9a-fA-F]+)",
        ]
        for pattern in patterns:
            m = re.search(pattern, line, re.IGNORECASE)
            if m:
                return m.group(1).lower(), CrashLogParser.parse_hex_value(m.group(2))
        return None, None

    @staticmethod
    def parse_fault_type(line: str) -> Optional[str]:
        """Extract fault type from log line."""
        for fault_name in FAULT_TYPES.values():
            if fault_name.lower() in line.lower():
                return fault_name

        # Also match numeric fault codes
        m = re.search(r"fault[_\s]type[:\s]+(\d)", line, re.IGNORECASE)
        if m:
            return FAULT_TYPES.get(m.group(1), f"Unknown ({m.group(1)})")

        m = re.search(r"(hardfault|hard_fault|memmanage|busfault|usagefault)",
                       line, re.IGNORECASE)
        if m:
            name = m.group(1).lower()
            mapping = {
                "hardfault": "HardFault",
                "hard_fault": "HardFault",
                "memmanage": "MemManage (MPU violation)",
                "busfault": "BusFault",
                "usagefault": "UsageFault",
            }
            return mapping.get(name, "HardFault")

        return None

    @staticmethod
    def parse_stack_frame(line: str) -> Optional[int]:
        """Extract stack frame address."""
        # Patterns: "  [00] 0x08001234", "  #0  0x08001234 in func", "  0x08001234"
        patterns = [
            r"\[\d+\]\s+(0x[0-9a-fA-F]+)",
            r"#\d+\s+(0x[0-9a-fA-F]+)",
            r"^\s+(0x[0-9a-fA-F]+)\s",
        ]
        for pattern in patterns:
            m = re.search(pattern, line)
            if m:
                return CrashLogParser.parse_hex_value(m.group(1))
        return None

    def parse(self, text: str) -> CrashLog:
        """Parse full crash dump text into CrashLog."""
        log = CrashLog()
        log.raw_lines = text.split("\n")

        in_fault_section = False
        in_stack_section = False
        in_event_log = False

        for line in log.raw_lines:
            stripped = line.strip()
            if not stripped:
                continue

            # Detect sections
            if re.search(r"(crash|fault|exception)", stripped, re.IGNORECASE):
                in_fault_section = True
            if re.search(r"(stack trace|call stack|backtrace|stack frames)",
                          stripped, re.IGNORECASE):
                in_stack_section = True
                continue
            if re.search(r"(event log|eventlog|system log)", stripped, re.IGNORECASE):
                in_event_log = True
                continue

            # Timestamp
            m = re.match(r"(\d{4}-\d{2}-\d{2}[T ]\d{2}:\d{2}:\d{2})", stripped)
            if m and not log.timestamp:
                log.timestamp = m.group(1)

            # Task name
            m = re.search(r"(task|thread)[:\s]*(\S+)", stripped, re.IGNORECASE)
            if m:
                log.task_name = m.group(2)

            # Fault type
            ft = self.parse_fault_type(stripped)
            if ft:
                log.fault_type = ft

            # Registers
            reg_name, reg_val = self.parse_registers(stripped)
            if reg_name:
                log.registers[reg_name] = reg_val
                if reg_name == "pc":
                    log.pc = reg_val
                elif reg_name == "lr":
                    log.lr = reg_val
                elif reg_name == "sp":
                    log.sp = reg_val
                elif reg_name == "xpsr" or reg_name == "psr":
                    log.psr = reg_val

            # Fault status registers
            for fsr in ["cfsr", "hfsr", "bfar", "mmar", "sfsr", "mfsr", "shcsr"]:
                m = re.search(rf"{fsr}[:\s]*(0x[0-9a-fA-F]+|\d+)", stripped, re.IGNORECASE)
                if m:
                    log.fault_status_regs[fsr.lower()] = self.parse_hex_value(m.group(1))

            # Stack frames
            if in_stack_section:
                addr = self.parse_stack_frame(stripped)
                if addr:
                    log.stack_frames.append(addr)

            # Event log entries
            if in_event_log:
                log.event_log_entries.append(stripped)

        # If stack frames not in dedicated section, search whole log
        if not log.stack_frames:
            for line in log.raw_lines:
                addr = self.parse_stack_frame(line)
                if addr:
                    log.stack_frames.append(addr)

        return log


# ---------------------------------------------------------------------------
# Symbol lookup via addr2line
# ---------------------------------------------------------------------------
class SymbolResolver:
    """Resolve addresses to function names and source locations."""

    def __init__(self, elf_path: str):
        self.elf_path = elf_path
        self.cache: Dict[int, str] = {}
        self.addr2line_available = self._check_tool()

    def _check_tool(self) -> bool:
        """Check if arm-none-eabi-addr2line is available."""
        if not self.elf_path or not os.path.exists(self.elf_path):
            return False

        for tool in ["arm-none-eabi-addr2line", "addr2line"]:
            try:
                subprocess.run(
                    [tool, "--version"],
                    capture_output=True, timeout=5
                )
                self._tool = tool
                return True
            except (subprocess.TimeoutExpired, FileNotFoundError):
                continue
        return False

    def resolve(self, address: int) -> str:
        """Resolve address to function:file:line."""
        if not self.addr2line_available:
            return f"0x{address:08X}"

        if address in self.cache:
            return self.cache[address]

        try:
            result = subprocess.run(
                [self._tool, "-e", self.elf_path, "-f", "-C",
                 f"0x{address:08X}"],
                capture_output=True, text=True, timeout=10
            )
            if result.returncode == 0:
                output = result.stdout.strip()
                # addr2line returns: function\nfile:line
                lines = output.split("\n")
                if len(lines) >= 2 and lines[1] != "??:0":
                    symbol = f"{lines[0]} at {lines[1]}"
                    self.cache[address] = symbol
                    return symbol
        except (subprocess.TimeoutExpired, FileNotFoundError):
            self.addr2line_available = False

        result = f"0x{address:08X}"
        self.cache[address] = result
        return result

    def is_available(self) -> bool:
        return self.addr2line_available


# ---------------------------------------------------------------------------
# Event log correlation
# ---------------------------------------------------------------------------
def correlate_events(crash: CrashLog) -> List[str]:
    """
    Correlate event log entries with the crash to find probable root cause.

    Returns list of relevant findings.
    """
    findings = []
    events = crash.event_log_entries

    if not events:
        return ["No event log entries available for correlation."]

    # Look for warning/error events preceding the crash
    error_events = []
    warning_events = []
    rtos_events = []

    for entry in events:
        lower = entry.lower()
        if re.search(r"(error|fail|fault|assert)", lower):
            error_events.append(entry)
        elif re.search(r"(warn|overflow|timeout)", lower):
            warning_events.append(entry)
        elif re.search(r"(task|queue|semaphore|mutex|stack)", lower):
            rtos_events.append(entry)

    if error_events:
        findings.append(f"Preceding errors ({len(error_events)}):")
        for e in error_events[-5:]:
            findings.append(f"  {e}")

    if warning_events:
        findings.append(f"Preceding warnings ({len(warning_events)}):")
        for w in warning_events[-3:]:
            findings.append(f"  {w}")

    if rtos_events:
        findings.append(f"RTOS events ({len(rtos_events)}):")
        for r in rtos_events[-3:]:
            findings.append(f"  {r}")

    # Check for stack overflow pattern
    for entry in events[-20:]:
        if re.search(r"(stack overflow|sp = 0x[0-9a-f]+)", entry, re.IGNORECASE):
            findings.append("STACK OVERFLOW DETECTED in event log.")
            break

    # Check for memory issues
    for entry in events[-20:]:
        if re.search(r"(malloc|free|alloc|heap|out of memory)", entry, re.IGNORECASE):
            findings.append("Memory management events detected near crash time.")
            break

    return findings


def analyze_fault_status(crash: CrashLog) -> List[str]:
    """
    Decode fault status registers to identify exact cause.
    """
    findings = []
    cfsr = crash.fault_status_regs.get("cfsr", 0)
    hfsr = crash.fault_status_regs.get("hfsr", 0)
    bfar = crash.fault_status_regs.get("bfar", 0)
    mmar = crash.fault_status_regs.get("mmar", 0)

    if cfsr:
        # Configurable Fault Status Register
        findings.append(f"CFSR = 0x{cfsr:08X}:")
        if cfsr & 0x0001:
            address = f"BFAR=0x{bfar:08X}" if bfar else "no address"
            findings.append(f"  [IACCVIOL] Instruction access violation ({address})")
        if cfsr & 0x0002:
            findings.append("  [DACCVIOL] Data access violation")
        if cfsr & 0x0080:
            findings.append("  [MMARVALID] MMAR holds valid fault address")
        if cfsr & 0x0100:
            findings.append(f"  [IBUSERR] Instruction bus error (BFAR=0x{bfar:08X})")
        if cfsr & 0x0200:
            findings.append("  [PRECISERR] Precise data bus error")
        if cfsr & 0x0400:
            findings.append("  [IMPRECISERR] Imprecise data bus error")
        if cfsr & 0x10000:
            findings.append("  [UNDEFINSTR] Undefined instruction")
        if cfsr & 0x20000:
            findings.append("  [INVSTATE] Invalid EPSR/control state")
        if cfsr & 0x40000:
            findings.append("  [INVPC] Invalid PC load (EXC_RETURN)")
        if cfsr & 0x80000:
            findings.append("  [NOCP] No coprocessor (FPU not enabled)")
        if cfsr & 0x1000000:
            findings.append("  [UNALIGNED] Unaligned memory access")
        if cfsr & 0x2000000:
            findings.append("  [DIVBYZERO] Divide by zero")

    if hfsr:
        findings.append(f"HFSR = 0x{hfsr:08X}:")
        if hfsr & 0x40000000:
            findings.append("  [DEBUGEVT] Debug event (halt or breakpoint)")
        if hfsr & 0x00000002:
            findings.append("  [FORCED] Forced hard fault (see CFSR)")

    if crash.fault_type == "HardFault" or "HardFault" in crash.fault_type:
        if cfsr == 0 and hfsr == 0:
            findings.append("  Both CFSR and HFSR are zero — possible double fault "
                            "or stack corruption prevented fault register capture.")

    return findings


# ---------------------------------------------------------------------------
# HTML report generation
# ---------------------------------------------------------------------------
def generate_html_report(crash: CrashLog, resolved_stack: List[Tuple[int, str]],
                          correlation: List[str], fault_analysis: List[str],
                          output_path: str):
    """Generate HTML report of crash analysis."""

    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    # Build stack table rows
    stack_rows = ""
    for i, (addr, symbol) in enumerate(resolved_stack):
        stack_rows += f"""
        <tr>
            <td>{i}</td>
            <td><code>0x{addr:08X}</code></td>
            <td><code>{symbol}</code></td>
        </tr>"""

    # Build correlation list
    corr_html = ""
    for finding in correlation:
        corr_html += f"<li>{finding}</li>\n"

    # Build fault analysis list
    fault_html = ""
    for finding in fault_analysis:
        fault_html += f"<li>{finding}</li>\n"

    # Register table
    reg_rows = ""
    for reg_name in ARM_REGISTERS:
        if reg_name in crash.registers:
            reg_rows += f"""
            <tr>
                <td><code>{reg_name}</code></td>
                <td><code>0x{crash.registers[reg_name]:08X}</code></td>
                <td>{crash.registers[reg_name]}</td>
            </tr>"""

    # Hardware description
    hw_desc = HARDWARE_DESCRIPTIONS.get(
        crash.fault_type.split(" ")[0],  # base fault name
        "No description available."
    )

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Crash Analysis Report — {crash.fault_type}</title>
<style>
* {{ margin: 0; padding: 0; box-sizing: border-box; }}
body {{ font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
       background: #f5f5f5; color: #333; line-height: 1.6; }}
.container {{ max-width: 1200px; margin: 0 auto; padding: 20px; }}
.header {{ background: linear-gradient(135deg, #e74c3c, #c0392b); color: #fff;
         padding: 30px; border-radius: 8px; margin-bottom: 24px; }}
.header h1 {{ font-size: 28px; margin-bottom: 8px; }}
.header .meta {{ opacity: 0.9; font-size: 14px; }}
.card {{ background: #fff; border-radius: 8px; padding: 24px; margin-bottom: 20px;
        box-shadow: 0 2px 8px rgba(0,0,0,0.1); }}
.card h2 {{ font-size: 18px; color: #2c3e50; margin-bottom: 16px;
           padding-bottom: 8px; border-bottom: 2px solid #eee; }}
table {{ width: 100%; border-collapse: collapse; }}
th, td {{ text-align: left; padding: 10px 12px; border-bottom: 1px solid #eee; }}
th {{ background: #f8f8f8; font-weight: 600; font-size: 13px; text-transform: uppercase; }}
tr:hover {{ background: #f0f7ff; }}
code {{ background: #f0f0f0; padding: 2px 6px; border-radius: 3px; font-size: 13px; }}
.fault-badge {{ display: inline-block; padding: 4px 12px; border-radius: 4px;
               font-weight: 600; font-size: 14px; }}
.fault-hardfault {{ background: #fee; color: #c00; }}
.fault-busfault {{ background: #fff3cd; color: #856404; }}
.fault-usagefault {{ background: #d1ecf1; color: #0c5460; }}
.fault-memmanage {{ background: #f8d7da; color: #721c24; }}
ul {{ margin-left: 20px; }}
li {{ margin-bottom: 6px; }}
.error-event {{ color: #c0392b; }}
.warning-event {{ color: #e67e22; }}
.info {{ background: #e8f4f8; border-left: 4px solid #2980b9; padding: 12px; margin: 8px 0;
        border-radius: 4px; }}
.warning {{ background: #fef3cd; border-left: 4px solid #f39c12; padding: 12px; margin: 8px 0;
           border-radius: 4px; }}
</style>
</head>
<body>
<div class="container">
    <div class="header">
        <h1>Crash Analysis Report</h1>
        <div class="meta">
            <p>Fault: <strong>{crash.fault_type}</strong> &nbsp;|&nbsp;
            Generated: {timestamp} &nbsp;|&nbsp;
            PC: 0x{crash.pc:08X} &nbsp;|&nbsp;
            LR: 0x{crash.lr:08X}</p>
            <p>Task: {crash.task_name if crash.task_name else "unknown"}
            &nbsp;|&nbsp; Timestamp: {crash.timestamp if crash.timestamp else "N/A"}</p>
        </div>
    </div>

    <div class="card">
        <h2>Fault Description</h2>
        <div class="info">{hw_desc}</div>
    </div>

    <div class="card">
        <h2>Fault Status Register Analysis</h2>
        <ul>
            {fault_html if fault_html else "<li>No fault status register data available.</li>"}
        </ul>
    </div>

    <div class="card">
        <h2>Call Stack</h2>
        <table>
            <tr><th>#</th><th>Address</th><th>Symbol / Source</th></tr>
            {stack_rows if stack_rows else "<tr><td colspan='3'>No stack frames available</td></tr>"}
        </table>
    </div>

    <div class="card">
        <h2>CPU Registers</h2>
        <table>
            <tr><th>Register</th><th>Value (hex)</th><th>Value (dec)</th></tr>
            {reg_rows if reg_rows else "<tr><td colspan='3'>No register data available</td></tr>"}
        </table>
    </div>

    <div class="card">
        <h2>Event Log Correlation</h2>
        <ul>
            {corr_html if corr_html else "<li>No event log data available for correlation.</li>"}
        </ul>
    </div>

    <div class="card">
        <h2>Raw Crash Dump</h2>
        <pre style="background:#f8f8f8; padding:16px; border-radius:4px; font-size:12px; overflow-x:auto; max-height:400px;">{chr(10).join(crash.raw_lines)}</pre>
    </div>
</div>
</body>
</html>"""

    with open(output_path, "w") as f:
        f.write(html)

    print(f"HTML report written to {output_path}")


# ---------------------------------------------------------------------------
# Main analysis pipeline
# ---------------------------------------------------------------------------
def analyze(elf_path: str, log_path: str, output_path: str,
            interactive: bool = False):
    """Run the full analysis pipeline."""

    # Read log
    if not os.path.exists(log_path):
        print(f"ERROR: Log file not found: {log_path}")
        sys.exit(1)

    with open(log_path, "r") as f:
        log_text = f.read()

    # Parse
    parser = CrashLogParser()
    crash = parser.parse(log_text)

    print(f"\nCrash Analysis")
    print(f"{'='*50}")
    print(f"Fault type: {crash.fault_type}")
    print(f"PC: 0x{crash.pc:08X}")
    print(f"LR: 0x{crash.lr:08X}")
    if crash.task_name:
        print(f"Task: {crash.task_name}")
    print(f"Stack frames: {len(crash.stack_frames)}")
    print(f"Registers captured: {len(crash.registers)}")
    print(f"Event log entries: {len(crash.event_log_entries)}")

    # Resolve symbols
    resolver = SymbolResolver(elf_path)
    resolved_stack = []

    if crash.stack_frames:
        print(f"\nCall Stack:")
        for i, addr in enumerate(crash.stack_frames):
            symbol = resolver.resolve(addr)
            resolved_stack.append((addr, symbol))
            print(f"  [{i:02d}] 0x{addr:08X} — {symbol}")

        # Also resolve PC and LR
        pc_symbol = resolver.resolve(crash.pc)
        lr_symbol = resolver.resolve(crash.lr)
        print(f"\n  PC: 0x{crash.pc:08X} — {pc_symbol}")
        print(f"  LR: 0x{crash.lr:08X} — {lr_symbol}")
    else:
        # Still resolve PC/LR
        pc_symbol = resolver.resolve(crash.pc)
        lr_symbol = resolver.resolve(crash.lr)
        print(f"\n  PC: 0x{crash.pc:08X} — {pc_symbol}")
        print(f"  LR: 0x{crash.lr:08X} — {lr_symbol}")
        resolved_stack = [(crash.pc, pc_symbol), (crash.lr, lr_symbol)]

    # Fault analysis
    print(f"\nFault Status Register Analysis:")
    fault_analysis = analyze_fault_status(crash)
    for fa in fault_analysis:
        print(f"  {fa}")

    # Event correlation
    print(f"\nEvent Log Correlation:")
    correlation = correlate_events(crash)
    for c in correlation:
        print(f"  {c}")

    # Interactive mode
    if interactive and resolver.is_available():
        print(f"\nInteractive mode — enter address to resolve (or 'q' to quit):")
        while True:
            try:
                inp = input("> ").strip()
                if inp.lower() in ("q", "quit", "exit"):
                    break
                try:
                    addr = int(inp, 16) if inp.startswith("0x") else int(inp)
                    print(f"  0x{addr:08X} → {resolver.resolve(addr)}")
                except ValueError:
                    print(f"  Invalid address")
            except (EOFError, KeyboardInterrupt):
                break

    # Generate HTML report if requested
    if output_path:
        # Prepend PC/LR if not in stack
        if crash.pc and not any(a == crash.pc for a, _ in resolved_stack):
            resolved_stack.insert(0, (crash.pc, pc_symbol))
        if crash.lr and not any(a == crash.lr for a, _ in resolved_stack):
            resolved_stack.append((crash.lr, lr_symbol))

        generate_html_report(crash, resolved_stack, correlation,
                              fault_analysis, output_path)

    # Return summary
    return {
        "fault_type": crash.fault_type,
        "pc": crash.pc,
        "lr": crash.lr,
        "task": crash.task_name,
        "stack_frames": len(crash.stack_frames),
        "symbol_resolver_available": resolver.is_available()
    }


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Crash Log Analyzer — parse, decode, and report crash dumps"
    )
    parser.add_argument("--elf", "-e", required=True,
                        help="Path to .elf file with debug symbols")
    parser.add_argument("--log", "-l", required=True,
                        help="Path to crash dump log file")
    parser.add_argument("--output", "-o", default="",
                        help="Output HTML report path")
    parser.add_argument("--interactive", "-i", action="store_true",
                        help="Interactive symbol resolution mode")

    args = parser.parse_args()

    if not os.path.exists(args.elf):
        print(f"WARNING: ELF file not found: {args.elf}")
        print("Symbol resolution will not be available.")

    if not args.output:
        args.output = os.path.splitext(args.log)[0] + "_report.html"

    summary = analyze(args.elf, args.log, args.output, args.interactive)

    print(f"\n{'='*50}")
    if summary["symbol_resolver_available"]:
        print("Symbol resolution: available")
    else:
        print("Symbol resolution: NOT available (install arm-none-eabi-addr2line)")
    print(f"Report: {args.output}")
    print("Analysis complete.")


if __name__ == "__main__":
    main()
