# Autonomous Tuning Guide

Edit only `autonomy_tuning.py` for normal field tuning.

## Most important settings

- `BALL_PICK_MIN_M`, `BALL_PICK_MAX_M`:
  Ball pickup window. Current target is `0.55m` to `0.70m`.

- `BUCKET_DROP_MIN_M`, `BUCKET_DROP_MAX_M`:
  Bucket drop window.

- `MIN_CONFIDENCE`:
  Detection confidence threshold. Increase if false positives occur.

- `CONFIRM_FRAMES`, `CLOSE_CONFIRM_FRAMES`:
  Number of consecutive frames required before driving/transitioning.
  Increase for stability, decrease for faster reaction.

- `STEER_KP`, `MAX_TURN_RATE`, `CENTER_DEADBAND`:
  Steering smoothness and aggressiveness.

- `BALL_SPEED_FAR`, `BALL_SPEED_MID`, `BALL_SPEED_NEAR`:
  Approach speed toward ball (negative means forward in your current convention).

- `BUCKET_SPEED_FAR`, `BUCKET_SPEED_MID`, `BUCKET_SPEED_NEAR`:
  Approach speed toward bucket.

## Recovery behavior

- `SEARCH_START_FRAMES`:
  Frames to wait after losing target before entering rotate-search.

- `BALL_SEARCH_TURN_RATE`, `BUCKET_SEARCH_TURN_RATE`:
  Search rotation speed.

## Depth robustness

- `MIN_VALID_DEPTH_M`:
  Depth values below this are ignored.

- `DEPTH_KERNEL_RADIUS`:
  Median depth is computed from `(2r+1)x(2r+1)` pixels around bbox center.
  Larger radius is more stable but slightly slower.

## Recommended field workflow

1. Tune ball pickup window and ball speeds first.
2. Tune steering (`STEER_KP`, `MAX_TURN_RATE`) until approach is smooth.
3. Tune bucket drop window and bucket speeds.
4. Increase `MIN_CONFIDENCE` and confirm frames if false triggers remain.
