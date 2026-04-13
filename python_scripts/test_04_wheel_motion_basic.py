"""
Test 04: Basic wheel motion test over serial.

Safety:
- Put wheels off ground before running.

Pass criteria:
- Non-zero feedback during linear and angular command phases.
"""

from __future__ import annotations

import argparse
import time

import serial


def build_cmd(lin: float, ang: float, shoulder: int = 0, claw: int = 0, arm_state: int = 10) -> bytes:
    return f"{lin:.2f}, {ang:.2f}, {int(shoulder)}, {int(claw)}, {int(arm_state)}\n".encode("utf-8")


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


def run_phase(ser, name: str, lin: float, ang: float, duration: float, period: float, rx_buffer: str):
    t0 = time.monotonic()
    next_tx = t0
    samples = []
    while time.monotonic() - t0 < duration:
        now = time.monotonic()
        if now >= next_tx:
            ser.write(build_cmd(lin, ang))
            next_tx = now + period

        lines, rx_buffer = drain_lines(ser, rx_buffer)
        for line in lines:
            parsed = parse_feedback(line)
            if parsed is not None:
                samples.append(parsed)

        time.sleep(0.003)

    print(f"[INFO] Phase {name}: sent lin={lin:.2f}, ang={ang:.2f}, feedback samples={len(samples)}")
    return samples, rx_buffer


def mean_abs(values):
    if not values:
        return 0.0
    return sum(abs(v) for v in values) / len(values)


def main() -> int:
    parser = argparse.ArgumentParser(description="Wheel motion smoke test")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--period", type=float, default=0.05)
    args = parser.parse_args()

    print("[WARN] Ensure wheels are OFF THE GROUND before continuing.")
    ser = serial.Serial(port=args.port, baudrate=args.baud, timeout=0)

    try:
        phases = [
            ("stop_1", 0.00, 0.00, 0.8),
            ("forward", 0.22, 0.00, 1.4),
            ("stop_2", 0.00, 0.00, 0.6),
            ("reverse", -0.22, 0.00, 1.4),
            ("stop_3", 0.00, 0.00, 0.6),
            ("turn_left", 0.00, 0.30, 1.2),
            ("turn_right", 0.00, -0.30, 1.2),
            ("stop_4", 0.00, 0.00, 0.8),
        ]

        phase_data = {}
        rx_buffer = ""
        for name, lin, ang, dur in phases:
            data, rx_buffer = run_phase(ser, name, lin, ang, dur, args.period, rx_buffer)
            phase_data[name] = data

    finally:
        try:
            ser.write(build_cmd(0.0, 0.0))
            time.sleep(0.1)
        except Exception:
            pass
        ser.close()

    fwd_lin = mean_abs([p[0] for p in phase_data.get("forward", [])])
    rev_lin = mean_abs([p[0] for p in phase_data.get("reverse", [])])
    left_ang = mean_abs([p[1] for p in phase_data.get("turn_left", [])])
    right_ang = mean_abs([p[1] for p in phase_data.get("turn_right", [])])

    print(f"[INFO] mean |lin| forward={fwd_lin:.4f}, reverse={rev_lin:.4f}")
    print(f"[INFO] mean |ang| left={left_ang:.4f}, right={right_ang:.4f}")

    if fwd_lin < 0.005 and rev_lin < 0.005:
        print("[FAIL] No meaningful linear motion feedback observed")
        return 1
    if left_ang < 0.005 and right_ang < 0.005:
        print("[FAIL] No meaningful angular motion feedback observed")
        return 1

    print("[PASS] Wheel motion test succeeded")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())