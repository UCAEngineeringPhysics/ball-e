"""
Test 05: Basic arm motion sequence over serial.

Safety:
- Keep arm clear of floor/obstacles.
- Start with regulator on and robot stable.

This test sends a short manual sequence:
neutral -> lower -> claw close -> claw open -> raise -> neutral
"""

from __future__ import annotations

import argparse
import time

import serial


def build_cmd(lin: float, ang: float, shoulder: int, claw: int, arm_state: int) -> bytes:
    return f"{lin:.2f}, {ang:.2f}, {int(shoulder)}, {int(claw)}, {int(arm_state)}\n".encode("utf-8")


def stream_cmd(ser, lin: float, ang: float, shoulder: int, claw: int, arm_state: int, duration: float, period: float):
    t0 = time.monotonic()
    sent = 0
    while time.monotonic() - t0 < duration:
        ser.write(build_cmd(lin, ang, shoulder, claw, arm_state))
        sent += 1
        time.sleep(period)
    return sent


def main() -> int:
    parser = argparse.ArgumentParser(description="Arm motion smoke test")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--period", type=float, default=0.05)
    parser.add_argument("--shoulder-cmd", type=int, default=1200)
    parser.add_argument("--claw-cmd", type=int, default=2500)
    parser.add_argument("--confirm", action="store_true", help="Prompt for manual pass/fail confirmation")
    args = parser.parse_args()

    print("[WARN] Arm test is about to run. Ensure clear space around arm.")
    ser = serial.Serial(port=args.port, baudrate=args.baud, timeout=0.03)

    total_sent = 0
    try:
        print("[INFO] neutral")
        total_sent += stream_cmd(ser, 0.0, 0.0, 0, 0, 10, 0.8, args.period)

        print("[INFO] lower shoulder")
        total_sent += stream_cmd(ser, 0.0, 0.0, args.shoulder_cmd, 0, 0, 0.9, args.period)

        print("[INFO] hold")
        total_sent += stream_cmd(ser, 0.0, 0.0, 0, 0, 0, 0.3, args.period)

        print("[INFO] close claw")
        total_sent += stream_cmd(ser, 0.0, 0.0, 0, args.claw_cmd, 0, 0.8, args.period)

        print("[INFO] open claw")
        total_sent += stream_cmd(ser, 0.0, 0.0, 0, -args.claw_cmd, 0, 0.8, args.period)

        print("[INFO] raise shoulder")
        total_sent += stream_cmd(ser, 0.0, 0.0, -args.shoulder_cmd, 0, 0, 0.9, args.period)

        print("[INFO] neutral")
        total_sent += stream_cmd(ser, 0.0, 0.0, 0, 0, 10, 1.0, args.period)

    finally:
        try:
            ser.write(build_cmd(0.0, 0.0, 0, 0, 10))
            time.sleep(0.1)
        except Exception:
            pass
        ser.close()

    print(f"[INFO] Total command packets sent: {total_sent}")

    if args.confirm:
        ans = input("Did the arm move smoothly and safely through the sequence? [y/N]: ").strip().lower()
        if ans != "y":
            print("[FAIL] Manual confirmation failed")
            return 1

    print("[PASS] Arm command sequence sent successfully")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())