"""
Run a selected subset of comprehensive tests sequentially.

Default sequence keeps hardware risk low:
- camera
- model
- serial
- state-machine dry-run

Wheel/arm tests are opt-in using flags.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


TEST_MAP = {
    "camera": "test_01_camera_stream_exact.py",
    "model": "test_02_model_pipeline_smoke.py",
    "serial": "test_03_serial_protocol_5field.py",
    "wheel": "test_04_wheel_motion_basic.py",
    "arm": "test_05_arm_motion_basic.py",
    "fsm": "test_06_state_machine_dry_run.py",
}


def run_one(script_path: Path) -> int:
    print(f"\n[RUN] {script_path.name}")
    result = subprocess.run([sys.executable, str(script_path)], check=False)
    if result.returncode == 0:
        print(f"[PASS] {script_path.name}")
    else:
        print(f"[FAIL] {script_path.name} (exit={result.returncode})")
    return result.returncode


def main() -> int:
    parser = argparse.ArgumentParser(description="Run comprehensive subsystem tests")
    parser.add_argument("--include-wheel", action="store_true")
    parser.add_argument("--include-arm", action="store_true")
    args = parser.parse_args()

    here = Path(__file__).resolve().parent
    order = ["camera", "model", "serial", "fsm"]
    if args.include_wheel:
        order.append("wheel")
    if args.include_arm:
        order.append("arm")

    failing = []
    for key in order:
        code = run_one(here / TEST_MAP[key])
        if code != 0:
            failing.append(TEST_MAP[key])

    if failing:
        print("\n[SUMMARY] Failures:")
        for name in failing:
            print(f"- {name}")
        return 1

    print("\n[SUMMARY] All selected tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())