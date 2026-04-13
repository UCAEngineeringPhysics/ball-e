"""
Central tuning values for autonomous behavior.
Edit only this file for quick field tuning.
Units:
- Distance: meters
- Velocity: same units used by Pico command protocol
- Angles: rad/s style command magnitudes
"""

# -----------------------------------------------------------------------------
# Detection Quality Gates
# -----------------------------------------------------------------------------
# Minimum detector confidence accepted as a "real" object.
# Raise this if you get false positives on the carpet/background.
# Lower this if true balls/buckets are being ignored too often.
MIN_CONFIDENCE = 0.50

# Number of consecutive frames a target must be seen before motion is allowed.
# Raise for more stability under flicker; lower for faster response.
CONFIRM_FRAMES = 3

# Number of consecutive close/aligned frames required before pick/drop transition.
# Raise to avoid accidental arm triggers; lower to speed up cycle time.
CLOSE_CONFIRM_FRAMES = 3


# -----------------------------------------------------------------------------
# Color Targeting (ball-first strategy)
# -----------------------------------------------------------------------------
# Ordered list of class/label tokens the parser can recognize in model labels.
# These can be full class names (e.g. red_ball, red_bucket) and the state
# machine will map them to the same color family automatically.
KNOWN_COLOR_TOKENS = (
    "blue bucket",
    "blue ball",
    "yellow bucket",
    "green ball",
    "green bucket",
    "red ball",
    "red bucket",
    "yellow ball"
)

# Requested first ball color/class. Use "" to disable color priority entirely.
# Accepted examples: "red", "red ball", "red_ball", or any value in KNOWN_COLOR_TOKENS.
TARGET_BALL_COLOR = "red ball"

# If True, enforce TARGET_BALL_COLOR only while lap_counter == 0 (first cycle).
# If False, every lap will keep prioritizing TARGET_BALL_COLOR.
TARGET_ONLY_ON_FIRST_LAP = True

# If True, robot can eventually fall back to "any ball" when target color is
# missing for too long. If False, robot will keep searching only that color.
FALLBACK_TO_ANY_BALL_IF_TARGET_MISSING = True

# Frames of missing target color before falling back to any ball.
# Raise for stricter color behavior; lower to keep mission moving.
TARGET_FALLBACK_FRAMES = 45

# For buckets: if False, robot will only approach the color-matched bucket.
# If True, after BUCKET_TARGET_FALLBACK_FRAMES it can use any bucket.
BUCKET_FALLBACK_TO_ANY_IF_TARGET_MISSING = False

# Frames to wait before bucket fallback is allowed (if enabled above).
BUCKET_TARGET_FALLBACK_FRAMES = 120


# -----------------------------------------------------------------------------
# Geometry / Depth Distance Fusion
# -----------------------------------------------------------------------------
# Focal length in pixels for geometry fallback distance equation:
# distance ~= (FOCAL_PX * real_object_height_m) / bbox_height_px
# Re-calibrate if geometry distance is consistently biased.
FOCAL_PX = 3386.0

# Real ball diameter/height used by geometry fallback.
# Update if competition ball size changes.
BALL_HEIGHT_M = 0.1524

# Real bucket height used by geometry fallback.
# Update if bucket model/size changes.
BUCKET_HEIGHT_M = 0.381

# Depth values below this are treated as invalid noise.
# Raise if near-zero spikes appear; lower if camera is very close to objects.
MIN_VALID_DEPTH_M = 0.10

# Ball approach and pickup
BALL_FAR_M = 6.0
BALL_MID_M = 1.4

# Lower bound of legal pickup distance window.
# If robot is closer than this, code commands a brief reverse.
BALL_PICK_MIN_M = 0.56
BALL_PICK_MAX_M = 0.67
BALL_TOO_CLOSE_BACKUP_LIN = 0.40

# Linear speed used when ball is farther than BALL_FAR_M.
# Negative means "forward" in your current platform convention.
BALL_SPEED_FAR = -0.35

# Linear speed used in mid band (BALL_MID_M to BALL_FAR_M).
BALL_SPEED_MID = -0.18

# Linear speed used near pickup zone (below BALL_MID_M).
BALL_SPEED_NEAR = -0.10

# Required center alignment tolerance before pick is allowed.
# Smaller value = stricter centering.
BALL_CENTER_TOL_FOR_PICK = 0.08

# Bucket approach and drop
BUCKET_FAR_M = 4.0
BUCKET_MID_M = 1.5

# Lower bound of legal drop distance window.
# If robot is too close, it backs up.
BUCKET_DROP_MIN_M = 0.41

# Upper bound of legal drop distance window.
# If robot is farther than this, it keeps approaching.
BUCKET_DROP_MAX_M = 0.55

# Reverse speed when too close to bucket.
BUCKET_TOO_CLOSE_BACKUP_LIN = 0.40

# Linear speed in far bucket band.
BUCKET_SPEED_FAR = -0.35

# Linear speed in mid bucket band.
BUCKET_SPEED_MID = -0.30

# Linear speed in near bucket band.
BUCKET_SPEED_NEAR = -0.15

# Required center alignment tolerance before drop is allowed.
BUCKET_CENTER_TOL_FOR_DROP = 0.10


# -----------------------------------------------------------------------------
# Steering and Alignment Response
# -----------------------------------------------------------------------------
# Proportional steering gain on horizontal image error.
# Raise for stronger turn response; lower if steering oscillates.
STEER_KP = 1.6

# Steering command hard clamp.
MAX_TURN_RATE = 0.55

# Ignore tiny center error inside this deadband.
# Raise to reduce jitter; lower for tighter tracking.
CENTER_DEADBAND = 0.03

# If horizontal error is larger than this, linear speed is reduced.
MISALIGN_ERR_FOR_SLOWDOWN = 0.16

# Linear speed multiplier when target is significantly off-center.
# Lower value = stronger slowdown while turning.
MISALIGN_LINEAR_SCALE = 0.65


# -----------------------------------------------------------------------------
# Lost-Target Search Behavior (phased sweep + forward arcs)
# -----------------------------------------------------------------------------
# Frames with no valid target before active searching starts.
SEARCH_START_FRAMES = 7

# Base yaw rate used for ball search phases.
BALL_SEARCH_TURN_RATE = 0.44

# Base yaw rate used for bucket search phases.
BUCKET_SEARCH_TURN_RATE = 0.44

# Forward component during search arcs for balls.
# Negative value drives forward in your current sign convention.
BALL_SEARCH_FORWARD_LIN = -0.20

# Forward component during search arcs for buckets.
# Typically a little more aggressive because bucket detection is harder at range.
BUCKET_SEARCH_FORWARD_LIN = -0.30

# Duration (frames) of each search phase before switching pattern.
# Lower = quicker scan changes, higher = smoother longer sweeps.
SEARCH_PHASE_FRAMES = 24

# Turn-rate multiplier used for the "wide opposite sweep" phase.
SEARCH_WIDE_TURN_MULT = 1.45

# Turn-rate multiplier used for forward-arc phases.
SEARCH_ARC_TURN_MULT = 0.55


# -----------------------------------------------------------------------------
# Depth Robustness
# -----------------------------------------------------------------------------
# Radius of square kernel sampled around detection center for depth median.
# 2 means a 5x5 sample window.
# Raise if depth is noisy; lower if objects are tiny and nearby clutter leaks in.
DEPTH_KERNEL_RADIUS = 2