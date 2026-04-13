"""
Test 06: State-machine dry run (no camera hardware, no serial hardware).

Purpose:
- Validate core autonomous flow transitions in software only.
- Confirm 5-field command formatting from state_machine.build_msg.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


# Make BARE_BONES modules importable when this script is run directly.
BASE_DIR = Path(__file__).resolve().parents[1]
if str(BASE_DIR) not in sys.path:
    sys.path.insert(0, str(BASE_DIR))

import autonomy_tuning as tune
import state_machine as sm
from odom_autonomous_bridge import OdomAutonomousBridge


class DummyBBox:
    def __init__(self, xmin, ymin, xmax, ymax):
        self._xmin = xmin
        self._ymin = ymin
        self._xmax = xmax
        self._ymax = ymax

    def xmin(self):
        return self._xmin

    def ymin(self):
        return self._ymin

    def xmax(self):
        return self._xmax

    def ymax(self):
        return self._ymax


class DummyIntrinsics:
    def __init__(self, fx=615.0, fy=615.0, ppx=320.0, ppy=240.0):
        self.fx = fx
        self.fy = fy
        self.ppx = ppx
        self.ppy = ppy


class DummyProfile:
    def __init__(self):
        self.intrinsics = DummyIntrinsics()

    def as_video_stream_profile(self):
        return self


class DummyDepthFrame:
    def __init__(self, depth_m):
        self.depth_m = depth_m
        self.profile = DummyProfile()

    def get_distance(self, _x, _y):
        return self.depth_m


class DummyUD:
    def __init__(self):
        self.latest_msg = b"0.0, 0.0, 0, 0, 0\n"
        self.mode = "detect"
        self.arm_state = "idle"

        self.fixed_travel_counter = 0
        self.picker_counter = 0
        self.lap_counter = 0

        self.distance = 0.0
        self.distance_from_depth = False

        self.carrying_ball_color = None
        self.target_bucket_color = None

        self.ball_seen_streak = 0
        self.ball_close_streak = 0
        self.ball_lost_streak = 0
        self.bucket_seen_streak = 0
        self.bucket_close_streak = 0
        self.bucket_lost_streak = 0

        self.odom_enabled = True
        self.odom = OdomAutonomousBridge(
            cam_offset_x_m=tune.ODOM_CAM_OFFSET_X_M,
            cam_offset_y_m=tune.ODOM_CAM_OFFSET_Y_M,
            cam_offset_z_m=tune.ODOM_CAM_OFFSET_Z_M,
            kp_v=tune.ODOM_KP_V,
            kp_w=tune.ODOM_KP_W,
            max_v=tune.ODOM_MAX_V,
            max_w=tune.ODOM_MAX_W,
            distance_tolerance_m=tune.ODOM_GOAL_TOLERANCE_M,
            forward_is_negative=tune.ODOM_FORWARD_IS_NEGATIVE,
        )


def assert_msg_5_fields(msg: bytes):
    parts = [p.strip() for p in msg.decode("utf-8", "ignore").strip().split(",")]
    if len(parts) != 5:
        raise AssertionError(f"Expected 5 command fields, got {len(parts)}: {msg!r}")


def run_detect_to_pick(ud: DummyUD, depth_w: int, depth_h: int):
    ball = [("red ball", 0.95, DummyBBox(0.42, 0.42, 0.58, 0.58))]

    # First far detections should produce motion commands.
    far_depth = DummyDepthFrame(1.20)
    for _ in range(max(4, tune.CONFIRM_FRAMES + 1)):
        sm.handle_detect_ball(ud, ball, far_depth, depth_w, depth_h, 640)
        assert_msg_5_fields(ud.latest_msg)

    # Then near detections should transition to pick mode.
    near_depth = DummyDepthFrame(0.62)
    for _ in range(max(6, tune.CLOSE_CONFIRM_FRAMES + 2)):
        sm.handle_detect_ball(ud, ball, near_depth, depth_w, depth_h, 640)
        assert_msg_5_fields(ud.latest_msg)
        if ud.mode == "pick":
            break

    if ud.mode != "pick":
        raise AssertionError("Did not transition to pick mode during near-ball phase")


def run_pick_cycle(ud: DummyUD):
    # Advance through lower -> close -> raise -> fixed_back
    max_steps = tune.PICK_LOWER_FRAMES + tune.PICK_CLOSE_FRAMES + tune.PICK_RAISE_FRAMES + 20
    for _ in range(max_steps):
        sm.handle_pick(ud)
        assert_msg_5_fields(ud.latest_msg)
        if ud.mode == "fixed_back" and ud.arm_state == "idle":
            return
    raise AssertionError("Pick cycle did not complete")


def run_bucket_to_drop(ud: DummyUD, depth_w: int, depth_h: int):
    ud.mode = "detect_bucket"
    ud.target_bucket_color = "red"

    bucket = [("red bucket", 0.95, DummyBBox(0.40, 0.36, 0.62, 0.86))]
    mid_depth = DummyDepthFrame(0.50)

    for _ in range(max(8, tune.CONFIRM_FRAMES + tune.CLOSE_CONFIRM_FRAMES + 4)):
        sm.handle_detect_bucket(ud, bucket, mid_depth, depth_w, depth_h, 640)
        assert_msg_5_fields(ud.latest_msg)
        if ud.mode == "drop":
            break

    if ud.mode != "drop":
        raise AssertionError("Did not transition to drop mode")


def run_drop_cycle(ud: DummyUD):
    max_steps = tune.DROP_LOWER_FRAMES + tune.DROP_OPEN_FRAMES + 20
    lap_before = ud.lap_counter
    for _ in range(max_steps):
        sm.handle_drop(ud)
        assert_msg_5_fields(ud.latest_msg)
        if ud.lap_counter > lap_before:
            return
    raise AssertionError("Drop cycle did not increment lap counter")


def main() -> int:
    parser = argparse.ArgumentParser(description="State machine dry run test")
    parser.add_argument("--depth-width", type=int, default=640)
    parser.add_argument("--depth-height", type=int, default=480)
    args = parser.parse_args()

    ud = DummyUD()
    assert_msg_5_fields(sm.build_msg(0.0, 0.0, 0, 0, 0))

    run_detect_to_pick(ud, args.depth_width, args.depth_height)
    run_pick_cycle(ud)
    run_bucket_to_drop(ud, args.depth_width, args.depth_height)
    run_drop_cycle(ud)

    print(f"[PASS] Dry-run state machine test succeeded. lap_counter={ud.lap_counter}, mode={ud.mode}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())