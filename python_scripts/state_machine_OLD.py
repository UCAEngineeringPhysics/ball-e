"""
Centralized state machine and navigation helpers for the autonomous course robot.
- Pick best detection by class (ball/bucket) and confidence, not just the last one.
- Compute distance using depth (D455) when valid, else geometry as backup.
- Handlers for each mode update user_data.latest_msg and optionally mode/arm_state.
Used by course_autonomous_depth.py (main: depth + YOLO) and course_camera.py (backup: geometry only).
"""

from __future__ import annotations

import re

import autonomy_tuning as tune

# -----------------------------------------------------------------------------
# Constants: distance bands (m), speeds, and geometry fallback
# -----------------------------------------------------------------------------
BALL_HEIGHT_M = tune.BALL_HEIGHT_M
BUCKET_HEIGHT_M = tune.BUCKET_HEIGHT_M
FOCAL_PX = tune.FOCAL_PX
MIN_CONFIDENCE = tune.MIN_CONFIDENCE
CONFIRM_FRAMES = tune.CONFIRM_FRAMES
CLOSE_CONFIRM_FRAMES = tune.CLOSE_CONFIRM_FRAMES

# Pico message format: "lin_vel, ang_vel, shoulder_cmd, claw_cmd, arm_state\n"
# arm_state: 0 = idle, 10 = neutral/return
def build_msg(lin_vel: float, ang_vel: float, shoulder: int, claw: int, arm_state: int = 0) -> bytes:
    """Build the 5-value message sent to the Pico."""
    return f"{lin_vel:.2f}, {ang_vel:.2f}, {shoulder}, {claw}, {arm_state}\n".encode("utf-8")


# -----------------------------------------------------------------------------
# Best detection: filter by class and pick one (by confidence, then center distance)
# -----------------------------------------------------------------------------
def _bbox_center(bbox) -> tuple[float, float]:
    """Return normalized (cx, cy) in [0,1]."""
    cx = (bbox.xmin() + bbox.xmax()) / 2.0
    cy = (bbox.ymin() + bbox.ymax()) / 2.0
    return cx, cy


def _label_tokens(label: str) -> list[str]:
    """Split a model class label into lowercase alphanumeric tokens."""
    return [token for token in re.split(r"[^a-z0-9]+", label.lower()) if token]


def _base_color_key(text: str) -> str:
    """Normalize class-like strings to a color-family key (e.g. red_ball -> red)."""
    key = text.lower().strip()
    for suffix in ("_ball", "_bucket"):
        if key.endswith(suffix):
            key = key[: -len(suffix)]
            break
    key = key.replace("ball", " ").replace("bucket", " ")
    key = "_".join(token for token in _label_tokens(key))
    return key


def _known_base_colors() -> set[str]:
    """Return normalized base color names derived from tuning tokens."""
    return {_base_color_key(token) for token in tune.KNOWN_COLOR_TOKENS if _base_color_key(token)}


def _extract_color_from_label(label: str) -> str | None:
    """Extract color family from a class label (e.g. red_ball/red_bucket -> red)."""
    lower_label = label.lower()
    known_bases = _known_base_colors()
    for class_token in tune.KNOWN_COLOR_TOKENS:
        token_l = class_token.lower()
        if token_l in lower_label:
            return _base_color_key(token_l)
    label_tokens = _label_tokens(lower_label)
    for token in label_tokens:
        if token in ("ball", "bucket"):
            continue
        if token in known_bases:
            return token
    return None


def _matches_required_color(label: str, required_color: str) -> bool:
    """
    Match required color robustly across label styles:
    - required can be "red", "red_ball", or "red_bucket"
    - label can be "red_ball", "ball_red", compact, etc.
    """
    req_key = _base_color_key(required_color)
    if not req_key:
        return False
    label_key = _extract_color_from_label(label)
    return label_key == req_key


def _get_requested_ball_color(ud) -> str | None:
    """
    Return the ball color requested by tuning.
    - If TARGET_BALL_COLOR is empty, no color preference is enforced.
    - If TARGET_ONLY_ON_FIRST_LAP is True, enforce only while lap_counter == 0.
    """
    requested = str(getattr(tune, "TARGET_BALL_COLOR", "")).strip().lower()
    if not requested:
        return None
    first_lap_only = bool(getattr(tune, "TARGET_ONLY_ON_FIRST_LAP", True))
    if first_lap_only and getattr(ud, "lap_counter", 0) > 0:
        return None
    return requested


def pick_best_detection(
    detections,
    class_substring: str,
    min_conf: float = MIN_CONFIDENCE,
    required_color: str | None = None,
):
    """
    From a list of (label, confidence, bbox), keep only those whose label contains
    class_substring (e.g. 'ball' or 'bucket') and with confidence >= min_conf.
    Return the single best: highest confidence; if tie, closest to image center (0.5, 0.5).
    Return None if no valid detection.
    """
    class_substring = class_substring.lower().strip()
    required_color = required_color.lower().strip() if required_color else None
    candidates = []
    for (label, conf, bbox) in detections:
        lower_label = label.lower()
        if class_substring not in lower_label or conf < min_conf:
            continue
        if required_color:
            if not _matches_required_color(label, required_color):
                continue
        candidates.append((label, conf, bbox))
    if not candidates:
        return None
    # Sort by confidence descending, then by distance from center ascending
    def score(item):
        label, conf, bbox = item
        cx, cy = _bbox_center(bbox)
        dist_from_center = (cx - 0.5) ** 2 + (cy - 0.5) ** 2
        return (-conf, dist_from_center)
    best = min(candidates, key=score)
    return best


# -----------------------------------------------------------------------------
# Distance: depth-first, then geometry fallback
# -----------------------------------------------------------------------------
def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _turn_rate_from_center(center_x: float) -> float:
    """Proportional steering command from normalized center-x error."""
    err = center_x - 0.5
    if abs(err) < tune.CENTER_DEADBAND:
        return 0.0
    return _clamp(tune.STEER_KP * err, -tune.MAX_TURN_RATE, tune.MAX_TURN_RATE)


def _search_motion(lost_streak: int, base_turn_rate: float, forward_lin: float, preferred_dir: float) -> tuple[float, float]:
    """
    Generate a phased search command when a target is missing:
    1) narrow sweep in last-seen direction
    2) wide sweep opposite direction
    3) forward arc in first direction
    4) forward arc opposite direction
    """
    phase_frames = max(1, int(getattr(tune, "SEARCH_PHASE_FRAMES", 24)))
    active_lost = max(0, lost_streak - tune.SEARCH_START_FRAMES)
    phase = (active_lost // phase_frames) % 4
    wide_mul = float(getattr(tune, "SEARCH_WIDE_TURN_MULT", 1.45))
    arc_turn_mul = float(getattr(tune, "SEARCH_ARC_TURN_MULT", 0.55))
    dir_sign = -1.0 if preferred_dir < 0 else 1.0
    if phase == 0:
        return (0.0, dir_sign * base_turn_rate)
    if phase == 1:
        return (0.0, -dir_sign * base_turn_rate * wide_mul)
    if phase == 2:
        return (forward_lin, dir_sign * base_turn_rate * arc_turn_mul)
    return (forward_lin, -dir_sign * base_turn_rate * arc_turn_mul)


def _depth_median_at_center(depth_frame, cx_px: int, cy_px: int, width: int, height: int) -> float:
    """
    Compute median depth from a small kernel around target center.
    This is more robust than a single pixel.
    """
    if depth_frame is None:
        return 0.0
    radius = max(0, int(tune.DEPTH_KERNEL_RADIUS))
    dvals = []
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
                dvals.append(d)
    if not dvals:
        return 0.0
    dvals.sort()
    return dvals[len(dvals) // 2]


def compute_distance(
    depth_frame,
    depth_width: int,
    depth_height: int,
    bbox,
    model_height: int,
    real_height_m: float,
) -> tuple[float, bool]:
    """
    Compute distance to object in meters.
    - depth_frame: RealSense depth frame (or None for geometry-only).
    - bbox: normalized 0-1 (from YOLO on model_height x model_height input).
    - real_height_m: real-world height of object (BALL_HEIGHT_M or BUCKET_HEIGHT_M).
    Returns (distance_m, used_depth). If used_depth is False, distance came from geometry.
    """
    cx_norm = (bbox.xmin() + bbox.xmax()) / 2.0
    cy_norm = (bbox.ymin() + bbox.ymax()) / 2.0
    # Map normalized coords to depth image pixels (depth is typically 640x480)
    cx_px = int(cx_norm * depth_width)
    cy_px = int(cy_norm * depth_height)
    cx_px = max(0, min(depth_width - 1, cx_px))
    cy_px = max(0, min(depth_height - 1, cy_px))

    hw_dist = _depth_median_at_center(depth_frame, cx_px, cy_px, depth_width, depth_height)

    # Geometry fallback: Z = (f * H) / h_pixels; h_pixels from bbox in model space
    h_pixels = (bbox.ymax() - bbox.ymin()) * model_height
    if h_pixels < 1.0:
        h_pixels = 1.0
    calc_dist = (FOCAL_PX * real_height_m) / h_pixels

    if hw_dist > tune.MIN_VALID_DEPTH_M:
        return (hw_dist, True)
    return (calc_dist, False)


# -----------------------------------------------------------------------------
# Steering: set linear and angular velocity from distance band and bbox center
# -----------------------------------------------------------------------------
def steering_from_bbox_and_distance(
    bbox,
    distance_m: float,
    is_ball: bool,
    forward_positive: bool,
) -> tuple[float, float]:
    """
    Returns (lin_vel, ang_vel) to approach the target.
    - is_ball: True for ball (drive forward = negative lin_vel in current convention), False for bucket.
    - forward_positive: if True, positive lin_vel = drive toward target; if False, negative = toward target.
    """
    center_x = (bbox.xmin() + bbox.xmax()) / 2.0
    # Steer: left of center (center_x < 0.5) -> turn left (negative ang for some conventions)
    # We use: center_x < 0.4 -> turn left, center_x > 0.6 -> turn right, else straight
    if center_x < 0.4:
        ang = 0.5
    elif center_x > 0.6:
        ang = -0.5
    else:
        ang = 0.0

    if is_ball:
        # Toward ball: user_data convention was negative lin_vel
        if distance_m >= tune.BALL_FAR_M:
            lin = -0.35 if forward_positive else -0.35
            ang = -ang  # match course_camera sign
        elif distance_m >= tune.BALL_MID_M:
            lin = -0.2
            ang = -ang
        else:
            lin = -0.2
            ang = -ang
        if not forward_positive:
            lin = -lin
        return (lin, ang)
    else:
        # Toward bucket
        if distance_m >= tune.BUCKET_FAR_M:
            lin = -0.2
            ang = -ang
        elif distance_m >= tune.BUCKET_MID_M:
            lin = -0.1
            ang = -ang
        else:
            lin = -0.1
            ang = -ang
        if not forward_positive:
            lin = -lin
        return (lin, ang)


# -----------------------------------------------------------------------------
# Mode handlers: each takes user_data and optional detections/depth; updates user_data
# -----------------------------------------------------------------------------
def handle_pause(ud):
    """Stop all motion."""
    ud.latest_msg = build_msg(0.0, 0.0, 0, 0, 0)


def handle_pick(ud) -> None:
    """Run arm sequence: lower -> close -> raise, then transition to fixed_back."""
    pick_lower_frames = max(1, int(getattr(tune, "PICK_LOWER_FRAMES", 170)))
    pick_close_frames = max(1, int(getattr(tune, "PICK_CLOSE_FRAMES", 150)))
    pick_raise_frames = max(1, int(getattr(tune, "PICK_RAISE_FRAMES", 180)))
    if ud.arm_state == "idle":
        ud.arm_state = "lower"
        ud.picker_counter = 0
    ud.latest_msg = build_msg(0.0, 0.0, 0, 0, 0)
    if ud.arm_state == "lower":
        ud.latest_msg = build_msg(0.0, 0.0, 3000, 0, 0)
        ud.picker_counter += 1
        if ud.picker_counter >= pick_lower_frames:
            ud.arm_state = "close"
            ud.picker_counter = 0
    elif ud.arm_state == "close":
        ud.latest_msg = build_msg(0.0, 0.0, 0, 3000, 0)
        ud.picker_counter += 1
        if ud.picker_counter >= pick_close_frames:
            ud.arm_state = "raise"
            ud.picker_counter = 0
    elif ud.arm_state == "raise":
        ud.latest_msg = build_msg(0.0, 0.0, -3000, 0, 0)
        ud.picker_counter += 1
        if ud.picker_counter >= pick_raise_frames:
            ud.latest_msg = build_msg(0.0, 0.0, 0, 0, 0)
            ud.mode = "fixed_back"
            ud.picker_counter = 0
            ud.arm_state = "idle"


def handle_drop(ud) -> None:
    """Lower arm, open claw, then go to detect (next ball) or swivel_large_right (done)."""
    drop_lower_frames = max(1, int(getattr(tune, "DROP_LOWER_FRAMES", 40)))
    drop_open_frames = max(1, int(getattr(tune, "DROP_OPEN_FRAMES", 40)))
    turn_to_center_after_drop = bool(getattr(tune, "TURN_TO_CENTER_AFTER_DROP", True))
    ud.latest_msg = build_msg(0.0, 0.0, 0, 0, 0)
    if ud.arm_state == "lower":
        ud.latest_msg = build_msg(0.0, 0.0, 3000, 0, 0)
        ud.picker_counter += 1
        if ud.picker_counter >= drop_lower_frames:
            ud.arm_state = "open"
            ud.picker_counter = 0
    elif ud.arm_state == "open":
        ud.latest_msg = build_msg(0.0, 0.0, 0, -3000, 0)
        ud.picker_counter += 1
        if ud.picker_counter >= drop_open_frames:
            ud.lap_counter += 1
            ud.carrying_ball_color = None
            ud.target_bucket_color = None
            if turn_to_center_after_drop:
                ud.mode = "swivel_large_right"
            else:
                ud.latest_msg = build_msg(0.0, 0.0, 0, 0, 10)
                ud.mode = "detect"
                ud.picker_counter = 0
                ud.fixed_travel_counter = 0
            ud.arm_state = "idle"


def handle_fixed_ball(ud) -> None:
    """Drive forward (encoder phase) toward center; then switch to detect."""
    fixed_ball_frames = max(1, int(getattr(tune, "FIXED_BALL_TRAVEL_FRAMES", 500)))
    ud.latest_msg = build_msg(-0.30, 0.0, 0, 0, 10)
    ud.fixed_travel_counter += 1
    if ud.fixed_travel_counter >= fixed_ball_frames:
        ud.mode = "detect"
        ud.fixed_travel_counter = 0
        ud.latest_msg = build_msg(0.0, 0.0, 0, 0, 0)


def handle_fixed_bucket(ud) -> None:
    """Drive forward toward bucket area; then switch to detect_bucket."""
    fixed_bucket_frames = max(1, int(getattr(tune, "FIXED_BUCKET_TRAVEL_FRAMES", 300)))
    ud.latest_msg = build_msg(-0.30, 0.0, 0, 0, 0)
    ud.fixed_travel_counter += 1
    if ud.fixed_travel_counter >= fixed_bucket_frames:
        ud.mode = "detect_bucket"
        ud.fixed_travel_counter = 0
        ud.latest_msg = build_msg(0.0, 0.0, 0, 0, 0)


def handle_fixed_back(ud) -> None:
    """Back up a bit after pick; then swivel_small_left."""
    fixed_back_frames = max(1, int(getattr(tune, "FIXED_BACK_TRAVEL_FRAMES", 80)))
    ud.latest_msg = build_msg(0.1, 0.0, 0, 0, 0)
    ud.fixed_travel_counter += 1
    if ud.fixed_travel_counter >= fixed_back_frames:
        ud.mode = "swivel_small_left"
        ud.fixed_travel_counter = 0
        ud.latest_msg = build_msg(0.0, 0.0, 0, 0, 0)


def handle_swivel_small_left(ud) -> None:
    """Turn left ~45 deg; then fixed_bucket."""
    swivel_small_left_frames = max(1, int(getattr(tune, "SWIVEL_SMALL_LEFT_FRAMES", 95)))
    ud.latest_msg = build_msg(0.0, -0.4, 0, 0, 0)
    ud.fixed_travel_counter += 1
    if ud.fixed_travel_counter >= swivel_small_left_frames:
        ud.mode = "fixed_bucket"
        ud.fixed_travel_counter = 0
        ud.latest_msg = build_msg(0.0, 0.0, 0, 0, 0)


def handle_swivel_large_right(ud) -> None:
    """Turn right ~180 deg back toward center; then fixed_ball."""
    swivel_large_right_frames = max(1, int(getattr(tune, "SWIVEL_LARGE_RIGHT_FRAMES", 540)))
    ud.latest_msg = build_msg(0.0, 0.4, 0, 0, 0)
    ud.fixed_travel_counter += 1
    if ud.fixed_travel_counter >= swivel_large_right_frames:
        ud.mode = "fixed_ball"
        ud.fixed_travel_counter = 0
        ud.latest_msg = build_msg(0.0, 0.0, 0, 0, 0)


def handle_detect_ball(
    ud,
    detections: list,
    depth_frame,
    depth_width: int,
    depth_height: int,
    model_height: int = 640,
) -> None:
    """
    In "detect" mode: find best ball; compute distance (depth or geometry);
    set velocity or transition to pick when close.
    """
    seen_streak = getattr(ud, "ball_seen_streak", 0)
    close_streak = getattr(ud, "ball_close_streak", 0)
    lost_streak = getattr(ud, "ball_lost_streak", 0)

    requested_ball_color = _get_requested_ball_color(ud)
    best = pick_best_detection(
        detections,
        "ball",
        MIN_CONFIDENCE,
        required_color=requested_ball_color,
    )
    if requested_ball_color and best is None:
        target_missing = getattr(ud, "ball_target_missing_streak", 0) + 1
        ud.ball_target_missing_streak = target_missing
        allow_fallback = bool(getattr(tune, "FALLBACK_TO_ANY_BALL_IF_TARGET_MISSING", True))
        fallback_frames = max(1, int(getattr(tune, "TARGET_FALLBACK_FRAMES", 45)))
        if allow_fallback and target_missing >= fallback_frames:
            best = pick_best_detection(detections, "ball", MIN_CONFIDENCE)
    else:
        ud.ball_target_missing_streak = 0

    if best is None:
        ud.ball_seen_streak = 0
        ud.ball_close_streak = 0
        lost_streak += 1
        ud.ball_lost_streak = lost_streak
        if lost_streak >= tune.SEARCH_START_FRAMES:
            search_dir = getattr(ud, "ball_search_dir", 1.0)
            lin_cmd, ang_cmd = _search_motion(
                lost_streak,
                tune.BALL_SEARCH_TURN_RATE,
                tune.BALL_SEARCH_FORWARD_LIN,
                search_dir,
            )
            ud.latest_msg = build_msg(lin_cmd, ang_cmd, 0, 0, 0)
        else:
            ud.latest_msg = build_msg(0.0, 0.0, 0, 0, 0)
        return

    ud.ball_lost_streak = 0
    seen_streak += 1
    ud.ball_seen_streak = seen_streak
    if seen_streak < CONFIRM_FRAMES:
        ud.latest_msg = build_msg(0.0, 0.0, 0, 0, 0)
        return

    label, conf, bbox = best
    detected_ball_color = _extract_color_from_label(label)
    if detected_ball_color is None and requested_ball_color:
        # If label taxonomy is color-implicit, preserve configured target color.
        detected_ball_color = requested_ball_color
    dist_m, used_depth = compute_distance(
        depth_frame, depth_width, depth_height, bbox, model_height, BALL_HEIGHT_M
    )
    center_x = (bbox.xmin() + bbox.xmax()) / 2.0
    ud.ball_search_dir = -1.0 if center_x < 0.5 else 1.0
    ang_cmd = _turn_rate_from_center(center_x)

    ud.distance = dist_m
    ud.distance_from_depth = used_depth

    if dist_m > tune.BALL_PICK_MAX_M:
        ud.ball_close_streak = 0
        if dist_m >= tune.BALL_FAR_M:
            lin_cmd = tune.BALL_SPEED_FAR
        elif dist_m >= tune.BALL_MID_M:
            lin_cmd = tune.BALL_SPEED_MID
        else:
            lin_cmd = tune.BALL_SPEED_NEAR
        if abs(center_x - 0.5) > tune.MISALIGN_ERR_FOR_SLOWDOWN:
            lin_cmd *= tune.MISALIGN_LINEAR_SCALE
        ud.latest_msg = build_msg(lin_cmd, ang_cmd, 0, 0, 0)
        return

    if dist_m < tune.BALL_PICK_MIN_M:
        ud.ball_close_streak = 0
        ud.latest_msg = build_msg(tune.BALL_TOO_CLOSE_BACKUP_LIN, ang_cmd * 0.5, 0, 0, 0)
        return

    if abs(center_x - 0.5) <= tune.BALL_CENTER_TOL_FOR_PICK:
        close_streak += 1
    else:
        close_streak = 0
    ud.ball_close_streak = close_streak

    if close_streak < CLOSE_CONFIRM_FRAMES:
        ud.latest_msg = build_msg(0.0, ang_cmd, 0, 0, 0)
        return

    ud.latest_msg = build_msg(0.0, 0.0, 0, 0, 0)
    ud.arm_state = "lower"
    ud.mode = "pick"
    ud.carrying_ball_color = detected_ball_color
    ud.target_bucket_color = detected_ball_color
    ud.ball_seen_streak = 0
    ud.ball_close_streak = 0
    ud.ball_lost_streak = 0


def handle_detect_bucket(
    ud,
    detections: list,
    depth_frame,
    depth_width: int,
    depth_height: int,
    model_height: int = 640,
) -> None:
    """Same as detect_ball but for bucket and transition to drop."""
    seen_streak = getattr(ud, "bucket_seen_streak", 0)
    close_streak = getattr(ud, "bucket_close_streak", 0)
    lost_streak = getattr(ud, "bucket_lost_streak", 0)

    target_bucket_color = getattr(ud, "target_bucket_color", None)
    best = pick_best_detection(
        detections,
        "bucket",
        MIN_CONFIDENCE,
        required_color=target_bucket_color,
    )
    if target_bucket_color and best is None:
        target_missing = getattr(ud, "bucket_target_missing_streak", 0) + 1
        ud.bucket_target_missing_streak = target_missing
        allow_fallback = bool(getattr(tune, "BUCKET_FALLBACK_TO_ANY_IF_TARGET_MISSING", False))
        fallback_frames = max(1, int(getattr(tune, "BUCKET_TARGET_FALLBACK_FRAMES", 120)))
        if allow_fallback and target_missing >= fallback_frames:
            best = pick_best_detection(detections, "bucket", MIN_CONFIDENCE)
    else:
        ud.bucket_target_missing_streak = 0

    if best is None:
        ud.bucket_seen_streak = 0
        ud.bucket_close_streak = 0
        lost_streak += 1
        ud.bucket_lost_streak = lost_streak
        if lost_streak >= tune.SEARCH_START_FRAMES:
            search_dir = getattr(ud, "bucket_search_dir", 1.0)
            lin_cmd, ang_cmd = _search_motion(
                lost_streak,
                tune.BUCKET_SEARCH_TURN_RATE,
                tune.BUCKET_SEARCH_FORWARD_LIN,
                search_dir,
            )
            ud.latest_msg = build_msg(lin_cmd, ang_cmd, 0, 0, 0)
        else:
            ud.latest_msg = build_msg(0.0, 0.0, 0, 0, 0)
        return

    ud.bucket_lost_streak = 0
    seen_streak += 1
    ud.bucket_seen_streak = seen_streak
    if seen_streak < CONFIRM_FRAMES:
        ud.latest_msg = build_msg(0.0, 0.0, 0, 0, 0)
        return

    label, conf, bbox = best
    dist_m, used_depth = compute_distance(
        depth_frame, depth_width, depth_height, bbox, model_height, BUCKET_HEIGHT_M
    )
    center_x = (bbox.xmin() + bbox.xmax()) / 2.0
    ud.bucket_search_dir = -1.0 if center_x < 0.5 else 1.0
    ang_cmd = _turn_rate_from_center(center_x)

    ud.distance = dist_m
    ud.distance_from_depth = used_depth

    if dist_m > tune.BUCKET_DROP_MAX_M:
        ud.bucket_close_streak = 0
        if dist_m >= tune.BUCKET_FAR_M:
            lin_cmd = tune.BUCKET_SPEED_FAR
        elif dist_m >= tune.BUCKET_MID_M:
            lin_cmd = tune.BUCKET_SPEED_MID
        else:
            lin_cmd = tune.BUCKET_SPEED_NEAR
        if abs(center_x - 0.5) > tune.MISALIGN_ERR_FOR_SLOWDOWN:
            lin_cmd *= tune.MISALIGN_LINEAR_SCALE
        ud.latest_msg = build_msg(lin_cmd, ang_cmd, 0, 0, 0)
        return

    if dist_m < tune.BUCKET_DROP_MIN_M:
        ud.bucket_close_streak = 0
        ud.latest_msg = build_msg(tune.BUCKET_TOO_CLOSE_BACKUP_LIN, ang_cmd * 0.5, 0, 0, 0)
        return

    if abs(center_x - 0.5) <= tune.BUCKET_CENTER_TOL_FOR_DROP:
        close_streak += 1
    else:
        close_streak = 0
    ud.bucket_close_streak = close_streak

    if close_streak < CLOSE_CONFIRM_FRAMES:
        ud.latest_msg = build_msg(0.0, ang_cmd, 0, 0, 0)
        return

    ud.latest_msg = build_msg(0.0, 0.0, 0, 0, 0)
    ud.arm_state = "lower"
    ud.mode = "drop"
    ud.bucket_seen_streak = 0
    ud.bucket_close_streak = 0
    ud.bucket_lost_streak = 0