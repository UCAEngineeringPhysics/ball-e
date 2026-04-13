"""
Test 03: Serial protocol validation for host<->Pico link.

Checks:
- Host command format uses 5 comma-separated fields
- Pico feedback lines parse as 2 floats (lin, ang)

Debug mode:
- Prints raw serial lines
- Classifies Pico debug lines that start with '#DBG'
"""

from __future__ import annotations

import argparse
import time

import serial


DBG_PREFIX = "#DBG"


def build_cmd(lin: float, ang: float, shoulder: int, claw: int, arm_state: int) -> bytes:
    msg = f"{lin:.2f}, {ang:.2f}, {int(shoulder)}, {int(claw)}, {int(arm_state)}\n"
    parts = [p.strip() for p in msg.strip().split(",")]
    if len(parts) != 5:
        raise ValueError(f"Command did not produce 5 fields: {msg!r}")
    return msg.encode("utf-8")


def parse_feedback(line: str):
    parts = [x.strip() for x in line.split(",")]
    if len(parts) < 2:
        return None
    try:
        return float(parts[0]), float(parts[1])
    except Exception:
        return None


def drain_lines(ser, rx_buffer: str):
    available = ser.in_waiting
    if available <= 0:
        return [], rx_buffer

    chunk = ser.read(available).decode("utf-8", "ignore")
    if not chunk:
        return [], rx_buffer

    chunk = chunk.replace("\r\n", "\n").replace("\r", "\n").replace("\\n", "\n")
    rx_buffer += chunk
    lines = []
    while "\n" in rx_buffer:
        line, rx_buffer = rx_buffer.split("\n", 1)
        lines.append(line.strip())
    return lines, rx_buffer


def classify_line(line: str):
    if line.startswith(DBG_PREFIX):
        return "dbg"
    parsed = parse_feedback(line)
    if parsed is not None:
        return "feedback"
    return "other"


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate 5-field command and 2-field feedback over serial")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--min-feedback", type=int, default=10)
    parser.add_argument("--boot-wait", type=float, default=1.5, help="Wait after opening serial so Pico can reboot")
    parser.add_argument("--debug", action="store_true", help="Print raw serial lines for diagnosis")
    args = parser.parse_args()

    print(f"[INFO] Opening serial: {args.port} @ {args.baud}")
    ser = serial.Serial(port=args.port, baudrate=args.baud, timeout=0)

    sent = 0
    raw_lines = 0
    valid_feedback = 0
    parse_errors = 0
    dbg_lines = 0
    sample_bad = []
    sample_dbg = []
    rx_buffer = ""

    print(f"[INFO] Waiting {args.boot_wait:.1f}s for Pico boot...")
    t_boot = time.monotonic()
    while time.monotonic() - t_boot < args.boot_wait:
        lines, rx_buffer = drain_lines(ser, rx_buffer)
        for line in lines:
            if not line:
                continue
            raw_lines += 1
            kind = classify_line(line)
            if kind == "dbg":
                dbg_lines += 1
                if len(sample_dbg) < 5:
                    sample_dbg.append(line)
            elif kind == "feedback":
                valid_feedback += 1
            else:
                parse_errors += 1
                if len(sample_bad) < 5:
                    sample_bad.append(line)
            if args.debug:
                print(f"[RX:{kind}] {line}")
        time.sleep(0.01)

    start = time.monotonic()
    next_tx = start
    try:
        while time.monotonic() - start < args.duration:
            now = time.monotonic()
            if now >= next_tx:
                # Keep arm neutral while validating motion protocol.
                ser.write(build_cmd(0.0, 0.0, 0, 0, 10))
                sent += 1
                next_tx = now + 0.05

            lines, rx_buffer = drain_lines(ser, rx_buffer)
            for line in lines:
                if not line:
                    continue
                raw_lines += 1
                kind = classify_line(line)
                if kind == "dbg":
                    dbg_lines += 1
                    if len(sample_dbg) < 5:
                        sample_dbg.append(line)
                elif kind == "feedback":
                    valid_feedback += 1
                else:
                    parse_errors += 1
                    if len(sample_bad) < 5:
                        sample_bad.append(line)
                if args.debug:
                    print(f"[RX:{kind}] {line}")

            time.sleep(0.005)

        print(f"[INFO] Commands sent: {sent}")
        print(f"[INFO] Raw feedback/debug lines: {raw_lines}")
        print(f"[INFO] Valid feedback lines: {valid_feedback}")
        print(f"[INFO] Debug lines (#DBG): {dbg_lines}")
        print(f"[INFO] Non-parse lines (excluding #DBG): {parse_errors}")
        if sample_dbg:
            print("[INFO] Sample #DBG lines:")
            for s in sample_dbg:
                print(f"  {s}")
        if sample_bad:
            print("[INFO] Sample non-parseable lines:")
            for s in sample_bad:
                print(f"  {s}")

    finally:
        try:
            ser.write(build_cmd(0.0, 0.0, 0, 0, 10))
        except Exception:
            pass
        ser.close()

    if raw_lines == 0:
        print("[FAIL] No serial output received from Pico (likely Pico main not running or not printing anything)")
        return 1

    if valid_feedback < args.min_feedback:
        print("[FAIL] Too few valid feedback lines from Pico")
        return 1

    print("[PASS] Serial protocol test succeeded (5-field TX / 2-field RX)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())