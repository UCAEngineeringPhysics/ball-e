"""
Test 01: RealSense camera bring-up using the exact stream settings used by the course runner.

Pass criteria:
- Can start color+depth streams at 640x480@30
- Can receive the requested number of frames without timeout
"""

from __future__ import annotations

import argparse
import time

import pyrealsense2 as rs


def main() -> int:
    parser = argparse.ArgumentParser(description="Test RealSense stream startup and frame acquisition")
    parser.add_argument("--frames", type=int, default=120, help="Number of frames to receive before pass")
    parser.add_argument("--timeout-ms", type=int, default=3000, help="Per-frame timeout in milliseconds")
    args = parser.parse_args()

    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    got_color = 0
    got_depth = 0
    start_t = time.monotonic()

    print("[INFO] Starting RealSense pipeline (color+depth 640x480 @30)...")
    profile = pipe.start(cfg)
    print(f"[INFO] Pipeline started: {profile}")

    try:
        for idx in range(args.frames):
            frames = pipe.wait_for_frames(timeout_ms=args.timeout_ms)
            color = frames.get_color_frame()
            depth = frames.get_depth_frame()
            if color:
                got_color += 1
            if depth:
                got_depth += 1

            if idx == 0:
                if depth:
                    print(f"[INFO] First depth profile: {depth.profile}")
                if color:
                    print(f"[INFO] First color profile: {color.profile}")

            if (idx + 1) % 30 == 0:
                print(f"[INFO] Frames received: {idx + 1}/{args.frames}")

    except RuntimeError as err:
        print(f"[FAIL] Frame timeout or RealSense runtime error: {err}")
        return 1
    finally:
        pipe.stop()

    elapsed = max(1e-6, time.monotonic() - start_t)
    fps = args.frames / elapsed
    print(f"[INFO] Received {args.frames} frames in {elapsed:.2f}s ({fps:.1f} fps)")
    print(f"[INFO] Color frames: {got_color}, Depth frames: {got_depth}")

    if got_color < args.frames or got_depth < args.frames:
        print("[FAIL] Missing color/depth frames in stream")
        return 1

    print("[PASS] Camera stream test succeeded")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())