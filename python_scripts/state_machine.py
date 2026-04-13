"""
Bare-bones state machine focused on detection + odom + autonomous pick/drop flow.
"""

from __future__ import annotations

import re
from math import cos, sin

import autonomy_tuning as tune

BALL_HEIGHT_M = tune.BALL_HEIGHT_M
BUCKET_HEIGHT_M = tune.BUCKET_HEIGHT_M
FOCAL_PX = tune.FOCAL_PX
MIN_CONFIDENCE = tune.MIN_CONFIDENCE
CONFIRM_FRAMES = tune.CONFIRM_FRAMES
CLOSE_CONFIRM_FRAMES = tune.CLOSE_CONFIRM_FRAMES
HOST_LINEAR_CMD_SCALE = float(getattr(tune, "HOST_LINEAR_CMD_SCALE", 1.0))
ARM_SHOULDER_CMD = int(getattr(tune, "ARM_SHOULDER_CMD", 3000))
ARM_CLAW_CMD = int(getattr(tune, "ARM_CLAW_CMD", 3000))


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def build_msg(lin_vel, ang_vel, shoulder, claw, arm_state=0):
    lin_wire = float(lin_vel) * HOST_LINEAR_CMD_SCALE
    return f"{lin_wire:.2f}, {ang_vel:.2f}, {shoulder}, {claw}, {arm_state}\\n".encode("utf-8")


def _bbox_center(bbox):
    return (bbox.xmin() + bbox.xmax()) / 2.0, (bbox.ymin() + bbox.ymax()) / 2.0


def _label_tokens(label):
    return [t for t in re.split(r"[^a-z0-9]+", label.lower()) if t]


def _base_color_key(text):
    key = str(text).lower().strip()
    for suffix in ("_ball", "_bucket"):
        if key.endswith(suffix):
            key = key[: -len(suffix)]
            break
    key = key.replace("ball", " ").replace("bucket", " ")
    return "_".join(_label_tokens(key))


def _extract_color_from_label(label):
    known = {_base_color_key(x) for x in getattr(tune, "KNOWN_COLOR_TOKENS", [])}
    tokens = _label_tokens(label)
    for token in tokens:
        if token in known:
            return token
    return None


def _matches_required_color(label, required_color):
    return _extract_color_from_label(label) == _base_color_key(required_color)


def pick_best_detection(detections, class_substring, min_conf=MIN_CONFIDENCE, required_color=None):
    class_substring = class_substring.lower().strip()
    candidates = []
    for label, conf, bbox in detections:
        low = str(label).lower()
        if class_substring not in low or conf < min_conf:
            continue
        if required_color and not _matches_required_color(low, required_color):
            continue
        candidates.append((label, conf, bbox))
    if not candidates:
        return None
    return max(candidates, key=lambda x: x[1])


def _depth_median(depth_frame, cx_px, cy_px, width, height, radius):
    vals = []
    for y in range(cy_px - radius, cy_px + radius + 1):
        if y < 0 or y >= height:
            continue
        for x in range(cx_px - radius, cx_px + radius + 1):
            if x < 0 or x >= width:
                continue
            try:
                d = depth_frame.get_distance(x, y)
            except Exception:
                d = 0.0
            if d > tune.MIN_VALID_DEPTH_M:
                vals.append(d)
    if not vals:
        return 0.0
    vals.sort()
    return vals[len(vals) // 2]


def compute_distance(depth_frame, depth_width, depth_height, bbox, model_height, real_height_m):
    cx, cy = _bbox_center(bbox)
    cx_px = int(_clamp(cx * depth_width, 0, depth_width - 1))
    cy_px = int(_clamp(cy * depth_height, 0, depth_height - 1))
    hw = _depth_median(depth_frame, cx_px, cy_px, depth_width, depth_height, tune.DEPTH_KERNEL_RADIUS)

    h_px = (bbox.ymax() - bbox.ymin()) * model_height
    h_px = max(1.0, h_px)
    geom = (FOCAL_PX * real_height_m) / h_px

    if hw > tune.MIN_VALID_DEPTH_M:
        return hw, True
    return geom, False


def _turn_rate(center_x):
    err = center_x - 0.5
    if abs(err) < tune.CENTER_DEADBAND:
        return 0.0
    return _clamp(tune.STEER_KP * err, -tune.MAX_TURN_RATE, tune.MAX_TURN_RATE)


def _search_motion(lost_streak, base_turn, fwd_lin, preferred_dir):
    active = max(0, lost_streak - tune.SEARCH_START_FRAMES)
    phase = (active // max(1, tune.SEARCH_PHASE_FRAMES)) % 4
    s = -1.0 if preferred_dir < 0 else 1.0
    if phase == 0:
        return 0.0, s * base_turn
    if phase == 1:
        return 0.0, -s * base_turn * tune.SEARCH_WIDE_TURN_MULT
    if phase == 2:
        return fwd_lin, s * base_turn * tune.SEARCH_ARC_TURN_MULT
    return fwd_lin, -s * base_turn * tune.SEARCH_ARC_TURN_MULT


def _odom_goal_from_bbox(ud, depth_frame, depth_width, depth_height, bbox):
    if not getattr(ud, "odom_enabled", False):
        return False
    odom = getattr(ud, "odom", None)
    if odom is None or depth_frame is None:
        return False

    cx, cy = _bbox_center(bbox)
    cx_px = int(_clamp(cx * depth_width, 0, depth_width - 1))
    cy_px = int(_clamp(cy * depth_height, 0, depth_height - 1))
    depth_m = _depth_median(depth_frame, cx_px, cy_px, depth_width, depth_height, tune.ODOM_GOAL_DEPTH_KERNEL_RADIUS)
    if depth_m <= tune.MIN_VALID_DEPTH_M:
        return False

    intr = depth_frame.profile.as_video_stream_profile().intrinsics
    fx = float(getattr(intr, "fx", 0.0))
    fy = float(getattr(intr, "fy", 0.0))
    ppx = float(getattr(intr, "ppx", 0.0))
    ppy = float(getattr(intr, "ppy", 0.0))
    if fx <= 1e-6 or fy <= 1e-6:
        return False

    cam_x = (cx_px - ppx) / fx * depth_m
    cam_y = (cy_px - ppy) / fy * depth_m
    cam_z = depth_m
    goal_x, goal_y, _ = odom.transform_cam_to_odom((cam_x, cam_y, cam_z))

    if getattr(odom, "goal_active", False):
        dx = goal_x - float(getattr(odom, "goal_x", 0.0))
        dy = goal_y - float(getattr(odom, "goal_y", 0.0))
        delta = (dx * dx + dy * dy) ** 0.5
        if tune.ODOM_GOAL_HYSTERESIS_ENABLE and delta < tune.ODOM_GOAL_HYSTERESIS_M:
            return True
        if delta < tune.ODOM_GOAL_UPDATE_MIN_DELTA_M:
            return True

    odom.set_goal(goal_x, goal_y)
    return True


def _drive_to_initial_ball_zone(ud):
    if not bool(getattr(tune, "INITIAL_BALL_ZONE_GOAL_ENABLE", True)):
        return False
    if getattr(ud, "lap_counter", 0) > 0:
        return False
    if not getattr(ud, "odom_enabled", False):
        return False

    odom = getattr(ud, "odom", None)
    if odom is None:
        return False

    if not getattr(odom, "goal_active", False):
        dist_fwd = float(getattr(tune, "INITIAL_BALL_ZONE_FORWARD_M", 3.8))
        gx = odom.pose.x + dist_fwd * cos(odom.pose.theta)
        gy = odom.pose.y + dist_fwd * sin(odom.pose.theta)
        odom.set_goal(gx, gy)

    stop_dist = float(getattr(tune, "INITIAL_BALL_ZONE_STOP_DIST_M", 0.45))
    dx = float(getattr(odom, "goal_x", 0.0)) - float(getattr(odom.pose, "x", 0.0))
    dy = float(getattr(odom, "goal_y", 0.0)) - float(getattr(odom.pose, "y", 0.0))
    if (dx * dx + dy * dy) ** 0.5 <= stop_dist:
        odom.clear_goal()
        return False

    cmd_v, cmd_w = odom.compute_goal_velocity()
    vmax = abs(float(getattr(tune, "INITIAL_BALL_ZONE_V_MAX", 0.16)))
    wmax = abs(float(getattr(tune, "INITIAL_BALL_ZONE_W_MAX", 0.30)))
    cmd_v = _clamp(cmd_v, -vmax, vmax)
    cmd_w = _clamp(cmd_w, -wmax, wmax)

    if abs(cmd_v) <= tune.ODOM_CMD_EPSILON and abs(cmd_w) <= tune.ODOM_CMD_EPSILON:
        return False

    ud.latest_msg = build_msg(cmd_v, cmd_w, 0, 0, 0)
    return True


def handle_pause(ud):
    ud.latest_msg = build_msg(0.0, 0.0, 0, 0, 0)


def handle_pick(ud):
    if ud.arm_state == "idle":
        ud.arm_state = "lower"
        ud.picker_counter = 0

    if ud.arm_state == "lower":
        ud.latest_msg = build_msg(0.0, 0.0, ARM_SHOULDER_CMD, 0, 0)
        ud.picker_counter += 1
        if ud.picker_counter >= tune.PICK_LOWER_FRAMES:
            ud.arm_state = "close"
            ud.picker_counter = 0
    elif ud.arm_state == "close":
        ud.latest_msg = build_msg(0.0, 0.0, 0, ARM_CLAW_CMD, 0)
        ud.picker_counter += 1
        if ud.picker_counter >= tune.PICK_CLOSE_FRAMES:
            ud.arm_state = "raise"
            ud.picker_counter = 0
    elif ud.arm_state == "raise":
        ud.latest_msg = build_msg(0.0, 0.0, -ARM_SHOULDER_CMD, 0, 0)
        ud.picker_counter += 1
        if ud.picker_counter >= tune.PICK_RAISE_FRAMES:
            ud.latest_msg = build_msg(0.0, 0.0, 0, 0, 0)
            ud.mode = "fixed_back"
            ud.arm_state = "idle"
            ud.picker_counter = 0


def handle_drop(ud):
    if ud.arm_state == "lower":
        ud.latest_msg = build_msg(0.0, 0.0, ARM_SHOULDER_CMD, 0, 0)
        ud.picker_counter += 1
        if ud.picker_counter >= tune.DROP_LOWER_FRAMES:
            ud.arm_state = "open"
            ud.picker_counter = 0
    elif ud.arm_state == "open":
        ud.latest_msg = build_msg(0.0, 0.0, 0, -ARM_CLAW_CMD, 0)
        ud.picker_counter += 1
        if ud.picker_counter >= tune.DROP_OPEN_FRAMES:
            ud.lap_counter += 1
            ud.arm_state = "idle"
            ud.picker_counter = 0
            ud.carrying_ball_color = None
            ud.target_bucket_color = None
            if ud.lap_counter >= int(getattr(tune, "MAX_LAPS", 4)):
                ud.mode = "pause"
            else:
                ud.mode = "swivel_large_right" if tune.TURN_TO_CENTER_AFTER_DROP else "detect"
    else:
        ud.latest_msg = build_msg(0.0, 0.0, 0, 0, 0)


def handle_fixed_ball(ud):
    if tune.FIXED_BALL_TRAVEL_FRAMES <= 0:
        ud.mode = "detect"
        ud.latest_msg = build_msg(0.0, 0.0, 0, 0, 0)
        return
    ud.latest_msg = build_msg(-0.30, 0.0, 0, 0, 10)
    ud.fixed_travel_counter += 1
    if ud.fixed_travel_counter >= tune.FIXED_BALL_TRAVEL_FRAMES:
        ud.fixed_travel_counter = 0
        ud.mode = "detect"


def handle_fixed_bucket(ud):
    ud.latest_msg = build_msg(-0.30, 0.0, 0, 0, 0)
    ud.fixed_travel_counter += 1
    if ud.fixed_travel_counter >= tune.FIXED_BUCKET_TRAVEL_FRAMES:
        ud.fixed_travel_counter = 0
        ud.mode = "detect_bucket"


def handle_fixed_back(ud):
    ud.latest_msg = build_msg(0.1, 0.0, 0, 0, 0)
    ud.fixed_travel_counter += 1
    if ud.fixed_travel_counter >= tune.FIXED_BACK_TRAVEL_FRAMES:
        ud.fixed_travel_counter = 0
        ud.mode = "swivel_small_left"


def handle_swivel_small_left(ud):
    ud.latest_msg = build_msg(0.0, -0.4, 0, 0, 0)
    ud.fixed_travel_counter += 1
    if ud.fixed_travel_counter >= tune.SWIVEL_SMALL_LEFT_FRAMES:
        ud.fixed_travel_counter = 0
        ud.mode = "fixed_bucket"


def handle_swivel_large_right(ud):
    ud.latest_msg = build_msg(0.0, 0.4, 0, 0, 0)
    ud.fixed_travel_counter += 1
    if ud.fixed_travel_counter >= tune.SWIVEL_LARGE_RIGHT_FRAMES:
        ud.fixed_travel_counter = 0
        ud.mode = "fixed_ball"


def handle_detect_ball(ud, detections, depth_frame, depth_width, depth_height, model_height=640):
    if getattr(ud, "arm_state", "idle") != "idle":
        ud.latest_msg = build_msg(0.0, 0.0, 0, 0, 0)
        return

    req_color = tune.TARGET_BALL_COLOR if (not tune.TARGET_ONLY_ON_FIRST_LAP or ud.lap_counter == 0) else None
    best = pick_best_detection(detections, "ball", MIN_CONFIDENCE, req_color)

    if best is None:
        ud.ball_seen_streak = 0
        ud.ball_close_streak = 0
        ud.ball_lost_streak = getattr(ud, "ball_lost_streak", 0) + 1

        if _drive_to_initial_ball_zone(ud):
            return

        if ud.ball_lost_streak >= tune.SEARCH_START_FRAMES:
            pref = getattr(ud, "ball_search_dir", 1.0)
            lin, ang = _search_motion(ud.ball_lost_streak, tune.BALL_SEARCH_TURN_RATE, tune.BALL_SEARCH_FORWARD_LIN, pref)
            ud.latest_msg = build_msg(lin, ang, 0, 0, 0)
        else:
            ud.latest_msg = build_msg(0.0, 0.0, 0, 0, 0)

        if tune.ODOM_CLEAR_GOAL_WHEN_BALL_LOST and getattr(ud, "odom", None) is not None and not (bool(getattr(tune, "INITIAL_BALL_ZONE_GOAL_ENABLE", True)) and getattr(ud, "lap_counter", 0) == 0):
            ud.odom.clear_goal()
        return

    ud.ball_lost_streak = 0
    ud.ball_seen_streak = getattr(ud, "ball_seen_streak", 0) + 1
    if ud.ball_seen_streak < CONFIRM_FRAMES:
        ud.latest_msg = build_msg(0.0, 0.0, 0, 0, 0)
        return

    label, _, bbox = best
    dist_m, used_depth = compute_distance(depth_frame, depth_width, depth_height, bbox, model_height, BALL_HEIGHT_M)
    cx, _ = _bbox_center(bbox)
    ud.ball_search_dir = -1.0 if cx < 0.5 else 1.0
    ang = _turn_rate(cx)

    ud.distance = dist_m
    ud.distance_from_depth = used_depth

    if dist_m > tune.BALL_PICK_MAX_M:
        if (not used_depth) and dist_m <= float(getattr(tune, "BALL_NO_DEPTH_NEAR_STOP_M", 0.80)):
            ud.latest_msg = build_msg(0.0, ang * 0.7, 0, 0, 0)
            return

        if dist_m >= tune.BALL_FAR_M:
            lin = tune.BALL_SPEED_FAR
        elif dist_m >= tune.BALL_MID_M:
            lin = tune.BALL_SPEED_MID
        else:
            lin = tune.BALL_SPEED_NEAR

        if abs(cx - 0.5) > tune.MISALIGN_ERR_FOR_SLOWDOWN:
            lin *= tune.MISALIGN_LINEAR_SCALE

        if tune.ODOM_ASSIST_BALL_ENABLE and _odom_goal_from_bbox(ud, depth_frame, depth_width, depth_height, bbox):
            cmd_v, cmd_w = ud.odom.compute_goal_velocity()
            eps = tune.ODOM_CMD_EPSILON
            if abs(cmd_v) > eps or abs(cmd_w) > eps:
                if not (tune.ODOM_REQUIRE_VISION_SIGN_MATCH and abs(lin) > eps and abs(cmd_v) > eps and (lin * cmd_v < 0.0)):
                    lin = cmd_v
                    ang = _clamp(cmd_w, -tune.MAX_TURN_RATE, tune.MAX_TURN_RATE)

        hard_stop_edge = tune.BALL_PICK_MAX_M + float(getattr(tune, "BALL_PICK_HARD_STOP_MARGIN_M", 0.04))
        if dist_m <= hard_stop_edge:
            lin = min(0.0, lin)

        ud.latest_msg = build_msg(lin, ang, 0, 0, 0)
        return

    if dist_m < tune.BALL_PICK_MIN_M:
        ud.latest_msg = build_msg(tune.BALL_TOO_CLOSE_BACKUP_LIN, ang * 0.5, 0, 0, 0)
        return

    if abs(cx - 0.5) <= tune.BALL_CENTER_TOL_FOR_PICK:
        ud.ball_close_streak = getattr(ud, "ball_close_streak", 0) + 1
    else:
        ud.ball_close_streak = 0

    if ud.ball_close_streak < CLOSE_CONFIRM_FRAMES:
        ud.latest_msg = build_msg(0.0, ang, 0, 0, 0)
        return

    ud.latest_msg = build_msg(0.0, 0.0, 0, 0, 0)
    ud.mode = "pick"
    ud.arm_state = "lower"
    ud.carrying_ball_color = _extract_color_from_label(label)
    ud.target_bucket_color = ud.carrying_ball_color
    if getattr(ud, "odom", None) is not None:
        ud.odom.clear_goal()


def handle_detect_bucket(ud, detections, depth_frame, depth_width, depth_height, model_height=640):
    if getattr(ud, "arm_state", "idle") != "idle":
        ud.latest_msg = build_msg(0.0, 0.0, 0, 0, 0)
        return

    req_color = getattr(ud, "target_bucket_color", None)
    best = pick_best_detection(detections, "bucket", MIN_CONFIDENCE, req_color)

    if best is None:
        ud.bucket_seen_streak = 0
        ud.bucket_close_streak = 0
        ud.bucket_lost_streak = getattr(ud, "bucket_lost_streak", 0) + 1
        if ud.bucket_lost_streak >= tune.SEARCH_START_FRAMES:
            pref = getattr(ud, "bucket_search_dir", 1.0)
            lin, ang = _search_motion(ud.bucket_lost_streak, tune.BUCKET_SEARCH_TURN_RATE, tune.BUCKET_SEARCH_FORWARD_LIN, pref)
            ud.latest_msg = build_msg(lin, ang, 0, 0, 0)
        else:
            ud.latest_msg = build_msg(0.0, 0.0, 0, 0, 0)
        if tune.ODOM_CLEAR_GOAL_WHEN_BUCKET_LOST and getattr(ud, "odom", None) is not None:
            ud.odom.clear_goal()
        return

    ud.bucket_lost_streak = 0
    ud.bucket_seen_streak = getattr(ud, "bucket_seen_streak", 0) + 1
    if ud.bucket_seen_streak < CONFIRM_FRAMES:
        ud.latest_msg = build_msg(0.0, 0.0, 0, 0, 0)
        return

    _, _, bbox = best
    dist_m, used_depth = compute_distance(depth_frame, depth_width, depth_height, bbox, model_height, BUCKET_HEIGHT_M)
    cx, _ = _bbox_center(bbox)
    ud.bucket_search_dir = -1.0 if cx < 0.5 else 1.0
    ang = _turn_rate(cx)

    ud.distance = dist_m
    ud.distance_from_depth = used_depth

    if dist_m > tune.BUCKET_DROP_MAX_M:
        if dist_m >= tune.BUCKET_FAR_M:
            lin = tune.BUCKET_SPEED_FAR
        elif dist_m >= tune.BUCKET_MID_M:
            lin = tune.BUCKET_SPEED_MID
        else:
            lin = tune.BUCKET_SPEED_NEAR
        if abs(cx - 0.5) > tune.MISALIGN_ERR_FOR_SLOWDOWN:
            lin *= tune.MISALIGN_LINEAR_SCALE

        if tune.ODOM_ASSIST_BUCKET_ENABLE and _odom_goal_from_bbox(ud, depth_frame, depth_width, depth_height, bbox):
            cmd_v, cmd_w = ud.odom.compute_goal_velocity()
            eps = tune.ODOM_CMD_EPSILON
            if abs(cmd_v) > eps or abs(cmd_w) > eps:
                if not (tune.ODOM_REQUIRE_VISION_SIGN_MATCH and abs(lin) > eps and abs(cmd_v) > eps and (lin * cmd_v < 0.0)):
                    lin = cmd_v
                    ang = _clamp(cmd_w, -tune.MAX_TURN_RATE, tune.MAX_TURN_RATE)

        ud.latest_msg = build_msg(lin, ang, 0, 0, 0)
        return

    if dist_m < tune.BUCKET_DROP_MIN_M:
        ud.latest_msg = build_msg(tune.BUCKET_TOO_CLOSE_BACKUP_LIN, ang * 0.5, 0, 0, 0)
        return

    if abs(cx - 0.5) <= tune.BUCKET_CENTER_TOL_FOR_DROP:
        ud.bucket_close_streak = getattr(ud, "bucket_close_streak", 0) + 1
    else:
        ud.bucket_close_streak = 0

    if ud.bucket_close_streak < CLOSE_CONFIRM_FRAMES:
        ud.latest_msg = build_msg(0.0, ang, 0, 0, 0)
        return

    ud.latest_msg = build_msg(0.0, 0.0, 0, 0, 0)
    ud.mode = "drop"
    ud.arm_state = "lower"
    if getattr(ud, "odom", None) is not None:
        ud.odom.clear_goal()