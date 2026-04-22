from blind_navigator import BlindNavigator

# Import to save modes and counters
from types import SimpleNamespace


import numpy as np
import cv2

import argparse
import pyrealsense2 as rs
import queue
import hailo

import gi

# ******************
import serial
import threading
from time import sleep

gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib


# ---------------------------------------------------------
# EDITABLE COURSE LAYOUT
# ---------------------------------------------------------
# Keep this sequence to control pick/drop order.
TARGET_SEQUENCE = ["red", "yellow", "green", "blue"]

# Ball waypoints (inside hoop), taken from odom_course_4-14.py.
# Update only the (x, y) tuple to retune.
BALL_POSITIONS = {
    "blue": (4.0, 4.0),
    "red": (2.5, 2.5),
    "yellow": (5.3, 5.3),
    "green": (2.5, 2.5),
}

# Bucket waypoints (corners), taken from odom_course_4-14.py.
# Update only the (x, y) tuple to retune.
BUCKET_POSITIONS = {
    "blue": (1.0, 1.0),
    "red": (0.0, 0.0),
    "yellow": (1.0, 8.3),
    "green": (0.0, 0.0),
}

# Keep detection disabled for labels that are currently unreliable.
DISABLE_BUCKET_DETECTION_FOR = {"yellow"}

# Optional midpoint detours keyed by target color.
# pre_ball_midpoints: before searching for that color's ball
# pre_bucket_midpoints: after picking that color's ball, before navigating to its bucket
PRE_BALL_MIDPOINTS_BY_COLOR = {
    "yellow": {
        "waypoint": (2.5, 8.0),
        "target_label": "eric",
        "claw_pw": 1700000,
        "shoa_pw": 1500000,
        "focus_type": "ball",
        "start_msg": "Setting mid waypoint...",
        "arrival_msg": "Arrived at midpoint location. Switching to next ball search.",
    },
    "red": {
        "waypoint": (7.0, 4.5),
        "target_label": "eric",
        "claw_pw": 1080000,
        "shoa_pw": 1500000,
        "focus_type": "ball",
        "start_msg": "Setting mid way point...",
        "arrival_msg": "Arrived at midpoint location. Switching to bucket search.",
    }
}
PRE_BUCKET_MIDPOINTS_BY_COLOR = {None
#    "red": {
#        "waypoint": (7.0, 2.5),
#        "target_label": "eric",
#        "claw_pw": 1080000,
#        "shoa_pw": 1500000,
#        "focus_type": "ball",
#        "start_msg": "Setting mid way point...",
#        "arrival_msg": "Arrived at midpoint location. Switching to bucket search.",
#    }
}

# Quick backup after each successful pickup to avoid clipping the center hoop.
BACKUP_AFTER_PICK = {
    "enabled": True,
    "duration_s": 2.0,
    "speed_mps": 0.30,
    "claw_pw": 1080000,
    "shoa_pw": 1500000,
}


def transform_cam_to_odom(coords_cam, robot_pose, dist_offset=-0.605):
    """
    Transforms a 3D point from the camera frame to the odom frame.

    Args:
        coords_cam (list or tuple): (x_c, y_c, z_c) representing the object in the camera frame.
        robot_pose (list or tuple): (X, Y, theta) representing the robot's current odometry.
    Returns:
        coords_odom (numpy.ndarray): [x_o, y_o, z_o] representing the object in the odom frame.
    """
    # 1. Parse inputs into numpy arrays
    coords_cam_arr = np.array(coords_cam)
    X, Y, theta = robot_pose

    # 2. Static Transformation: camera_link to base_link
    R_c_b = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    t_c_b = np.array(
        [dist_offset, 0.0, 0.2],  # use dist_offset to govern stop distance
    )  # Set camera displacement from robot's base center

    # Calculate point in base_link
    # p_b = R_c_b @ coords_cam_arr + t_c_b
    coords_base_arr = R_c_b @ coords_cam_arr + t_c_b

    # 3. Dynamic Transformation: base_link to odom
    # Building the rotation matrix manually for speed (Z-axis rotation only)
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)

    R_b_o = np.array([[cos_t, -sin_t, 0.0], [sin_t, cos_t, 0.0], [0.0, 0.0, 1.0]])
    t_b_o = np.array([X, Y, 0.0])  # Robot's position on the ground plane

    # Calculate final point in odom
    coords_odom = R_b_o @ coords_base_arr + t_b_o

    return coords_odom


# Function for odometry navigation (logic from greenball_follower)
def process_targeting(
    navigator, depth_frame, detections, target_label, claw_pw, shoa_pw
):
    """
    Looks for a specific label in detections and updates navigator goal.
    Returns: True if target found/updated, False otherwise.
    """
    for label, conf, bbox in detections:
        if conf > 0.5 and label == target_label:
            x1 = int(bbox.xmin() * 640)  # Map bbox (0.0-1.0) to 640x480
            y1 = int(bbox.ymin() * 480)
            x2 = int(bbox.xmax() * 640)
            y2 = int(bbox.ymax() * 480)
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            # Compute goal coords
            depth_in_meters = depth_frame.get_distance(cx, cy)
            print(f"depth: {depth_in_meters}m")  # debug
            print(label)
            if depth_in_meters > 0:
                intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
                coords_cam = rs.rs2_deproject_pixel_to_point(
                    intrinsics,
                    [cx, cy],
                    depth_in_meters,
                )  # Convert 2D pixel to 3D point (X, Y, Z in meters)
                # print(
                #     f"Object {label} at X:{coords_cam[0]:.2f}m, Y:{coords_cam[1]:.2f}m, Z:{coords_cam[2]:.2f}m"
                # )  # debug
                if "ball" in target_label:
                    dist_offset = -0.59
                else:
                    dist_offset = -0.45
                goal_coords = transform_cam_to_odom(
                    (coords_cam[0], coords_cam[1], coords_cam[2]),
                    (navigator.x, navigator.y, navigator.theta),
                    dist_offset=dist_offset,
                )
                print(
                    f"Goal coors in odom: {goal_coords}, distance offset: {dist_offset}"
                )
                if (
                    np.linalg.norm(
                        np.array(goal_coords[:2])
                        - np.array((navigator.goal_x, navigator.goal_y))
                    )
                    > 0.01
                ):  # update goal when necessary
                    navigator.set_goal(goal_coords[0], goal_coords[1], claw_pw, shoa_pw)
                    print(f"Set goal at: {goal_coords}")
                print(f"robot pose: {navigator.x, navigator.y, navigator.theta}")
                break


def draw_detections(img_display, detections, focus_type):
    for label, conf, bbox in detections:
        x1 = int(bbox.xmin() * 640)
        y1 = int(bbox.ymin() * 480)
        x2 = int(bbox.xmax() * 640)
        y2 = int(bbox.ymax() * 480)

        color = (0, 255, 0) if focus_type in label else (255, 0, 0)
        cv2.rectangle(img_display, (x1, y1), (x2, y2), color, 2)

        text = f"{label}: {conf:.2f}"
        cv2.putText(
            img_display,
            text,
            (x1, y1 - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            2,
        )


def set_waypoint_once(state, navigator, waypoint, claw_pw, shoa_pw, message):
    if not state.targeting_active:
        print(message)
        navigator.set_goal(waypoint[0], waypoint[1], claw_pw, shoa_pw)
        state.targeting_active = True


def run_pick_sequence(navigator, state):
    if state.arm_state == "idle":
        state.arm_state = "lower"
        navigator.manual_override_msg = "0.0,0.0,1700000,500000\n"
        sleep(0.2)

    if state.arm_state == "lower":
        navigator.manual_override_msg = "0.0,0.0,1700000,500000\n"
        sleep(0.1)
        if navigator.goal_status == 1:
            state.arm_state = "close"
            navigator.manual_override_msg = "0.0,0.0,1070000,500000\n"
            sleep(0.2)

    elif state.arm_state == "close":
        navigator.manual_override_msg = "0.0,0.0,1070000,500000\n"
        sleep(0.1)
        if navigator.goal_status == 1:
            state.arm_state = "raise"
            navigator.manual_override_msg = "0.0,0.0,1070000,1500000\n"
            sleep(0.2)

    elif state.arm_state == "raise":
        navigator.manual_override_msg = "0.0,0.0,1070000,1500000\n"
        sleep(0.1)
        if navigator.goal_status == 1:
            return True

    return False


def run_drop_sequence(navigator, state):
    if state.arm_state == "idle":
        state.arm_state = "lower"
        navigator.manual_override_msg = "0.0,0.0,1070000,1100000\n"
        sleep(0.2)

    if state.arm_state == "lower":
        navigator.manual_override_msg = "0.0,0.0,1070000,1100000\n"
        sleep(0.1)
        if navigator.goal_status == 1:
            state.arm_state = "open"
            navigator.manual_override_msg = "0.0,0.0,1700000,1100000\n"
            sleep(0.2)

    elif state.arm_state == "open":
        navigator.manual_override_msg = "0.0,0.0,1700000,1100000\n"
        sleep(0.1)
        if navigator.goal_status == 1:
            state.arm_state = "raise"
            navigator.manual_override_msg = "0.0,0.0,1700000,1500000\n"
            sleep(0.2)

    elif state.arm_state == "raise":
        navigator.manual_override_msg = "0.0,0.0,1700000,1500000\n"
        sleep(0.1)
        if navigator.goal_status == 1:
            return True

    return False


# ---------------------------------------------------------
# 1. HAILO INFERENCE CLASS (The "Engine")
# ---------------------------------------------------------
class HailoRemoteInference:
    def __init__(self, hef_path, labels_json):
        Gst.init(None)
        self.running = False
        self.detection_queue = queue.Queue(maxsize=1)
        # Determine Post-Process Shared Object
        post_process_so = "/usr/lib/aarch64-linux-gnu/hailo/tappas/post_processes/libyolo_hailortpp_post.so"

        # --- FIXED PIPELINE ---
        # Added 'force-writable=true' to hailonet to fix the buffer error.
        pipeline_str = f"""
            appsrc name=source is-live=true block=true format=GST_FORMAT_TIME ! \
            videoconvert ! video/x-raw,format=RGB,width=640,height=640 ! \
            hailonet hef-path={hef_path} force-writable=true ! \
            hailofilter so-path={post_process_so} config-path={labels_json} qos=false ! \
            queue leaky=no max-size-buffers=3 ! \
            appsink name=sink emit-signals=true max-buffers=1 drop=true
        """

        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
        except Exception as e:
            print(f"Error building pipeline: {e}")
            raise

        self.appsrc = self.pipeline.get_by_name("source")
        self.appsink = self.pipeline.get_by_name("sink")
        self.appsink.connect("new-sample", self._on_new_sample)

        caps = Gst.Caps.from_string(
            "video/x-raw,format=RGB,width=640,height=640,framerate=30/1"
        )
        self.appsrc.set_property("caps", caps)

    def start(self):
        self.pipeline.set_state(Gst.State.PLAYING)
        self.running = True
        print("Hailo Engine Started.")

    def stop(self):
        self.running = False
        self.pipeline.set_state(Gst.State.NULL)

    def _on_new_sample(self, sink):
        sample = sink.emit("pull-sample")
        buffer = sample.get_buffer()
        roi = hailo.get_roi_from_buffer(buffer)
        detections = roi.get_objects_typed(hailo.HAILO_DETECTION)

        results = []
        for det in detections:
            label = det.get_label()
            confidence = det.get_confidence()
            bbox = det.get_bbox()
            results.append((label, confidence, bbox))

        if self.detection_queue.full():
            try:
                self.detection_queue.get_nowait()
            except:
                pass
        self.detection_queue.put(results)
        return Gst.FlowReturn.OK

    def infer_frame(self, numpy_frame):
        # Resize to 640x640 for YOLO
        resized = cv2.resize(numpy_frame, (640, 640))
        data = resized.tobytes()
        buf = Gst.Buffer.new_allocate(None, len(data), None)
        buf.fill(0, data)
        self.appsrc.emit("push-buffer", buf)

    def get_latest_result(self):
        try:
            return self.detection_queue.get_nowait()
        except queue.Empty:
            return []


# ---------------------------------------------------------
# 2. MAIN ROBOT LOGIC
# ---------------------------------------------------------
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--hef-path", default="../models/4_2.hef")
    parser.add_argument("--labels-json", default="../models/ball_bucket.json")
    # Added dummy --input argument so your existing command string works
    parser.add_argument("--input", default=None, help="Ignored: RealSense is hardcoded")
    args = parser.parse_args()

    # A. Setup navigator
    navigator = BlindNavigator()

    # B. Setup Hailo
    engine = HailoRemoteInference(args.hef_path, args.labels_json)
    engine.start()

    # Build route data from editable layout constants.
    targets = TARGET_SEQUENCE
    ball_waypoints = [BALL_POSITIONS[target] for target in targets]
    bucket_waypoints = [BUCKET_POSITIONS[target] for target in targets]
    ball_labels = [f"{target} ball" for target in targets]
    bucket_labels = [f"{target} bucket" for target in targets]
    for color in DISABLE_BUCKET_DETECTION_FOR:
        if color in targets:
            bucket_labels[targets.index(color)] = ""

    # Optional midpoint detours by target index, derived from color keys.
    pre_ball_midpoints = {
        targets.index(color): cfg
        for color, cfg in PRE_BALL_MIDPOINTS_BY_COLOR.items()
        if color in targets
    }
#    pre_bucket_midpoints = {
#        targets.index(color): cfg
#        for color, cfg in PRE_BUCKET_MIDPOINTS_BY_COLOR.items()
#        if color in targets
#    }

    # Create a state object with generic modes.
    state = SimpleNamespace(
        mode="SEARCH_BALL",
        arm_state="idle",
        target_index=0,
        targeting_active=False,
        midpoint_cfg=None,
        next_mode=None,
    )

    # C. Setup RealSense
    print("Starting RealSense...")
    pipeline = rs.pipeline()
    config = rs.config()
    # Using 640x480 standard resolution
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    align = rs.align(rs.stream.color)
    pipeline.start(config)

    print("System Running. Press 'q' to quit.")

    # navigator.set_goal(5.0, 0.0)

    try:
        while True:
            # 1. Get frames
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue
            img_color = np.asanyarray(color_frame.get_data())
            img_display = cv2.cvtColor(img_color, cv2.COLOR_RGB2BGR)

            # ------------------------------------MODES-------------------------------------------
            detections = []
            if state.mode in ("SEARCH_BALL", "NAV_TO_BUCKET", "NAV_MIDPOINT"):
                engine.infer_frame(img_color)
                detections = engine.get_latest_result()

            if state.mode == "pause":
                navigator.manual_override_msg = "0.0,0.0,1700000,1500000\n"

            elif state.mode == "PICK_BALL":
                if run_pick_sequence(navigator, state):
                    state.arm_state = "idle"
                    state.targeting_active = False
                    if BACKUP_AFTER_PICK["enabled"]:
                        navigator.backup_for(
                            duration_s=BACKUP_AFTER_PICK["duration_s"],
                            speed_mps=BACKUP_AFTER_PICK["speed_mps"],
                            claw_pw=BACKUP_AFTER_PICK["claw_pw"],
                            shoa_pw=BACKUP_AFTER_PICK["shoa_pw"],
                        )
#                    if state.target_index in pre_bucket_midpoints:
#                        state.mode = "NAV_MIDPOINT"
#                        state.midpoint_cfg = pre_bucket_midpoints[state.target_index]
#                        state.next_mode = "NAV_TO_BUCKET"
#                    else:
                    state.mode = "NAV_TO_BUCKET"

            elif state.mode == "DROP_BALL":
                if run_drop_sequence(navigator, state):
                    state.arm_state = "idle"
                    state.targeting_active = False
                    state.target_index += 1

                    if state.target_index >= len(targets):
                        state.mode = "pause"
                    elif state.target_index in pre_ball_midpoints:
                        state.mode = "NAV_MIDPOINT"
                        state.midpoint_cfg = pre_ball_midpoints[state.target_index]
                        state.next_mode = "SEARCH_BALL"
                    else:
                        state.mode = "SEARCH_BALL"

            elif state.mode == "SEARCH_BALL":
                navigator.manual_override_msg = ""
                draw_detections(img_display, detections, "ball")

                target_name = targets[state.target_index]
                set_waypoint_once(
                    state,
                    navigator,
                    ball_waypoints[state.target_index],
                    1700000,
                    1500000,
                    f"Setting waypoint for {target_name} ball...",
                )
                process_targeting(
                    navigator,
                    depth_frame,
                    detections,
                    ball_labels[state.target_index],
                    1700000,
                    1500000,
                )

                if navigator.is_goal_reached:
                    print(f"Arrived at {target_name} ball. Switching to pick.")
                    state.mode = "PICK_BALL"
                    state.arm_state = "lower"
                    state.targeting_active = False

            elif state.mode == "NAV_TO_BUCKET":
                navigator.manual_override_msg = ""
                draw_detections(img_display, detections, "bucket")

                target_name = targets[state.target_index]
                set_waypoint_once(
                    state,
                    navigator,
                    bucket_waypoints[state.target_index],
                    1080000,
                    1500000,
                    f"Setting waypoint for {target_name} bucket...",
                )
                process_targeting(
                    navigator,
                    depth_frame,
                    detections,
                    bucket_labels[state.target_index],
                    1080000,
                    1500000,
                )

                if navigator.is_goal_reached:
                    print(f"Arrived at {target_name} bucket. Switching to drop.")
                    state.mode = "DROP_BALL"
                    state.arm_state = "idle"
                    state.targeting_active = False

            elif state.mode == "NAV_MIDPOINT":
                navigator.manual_override_msg = ""
                midpoint_cfg = state.midpoint_cfg

                draw_detections(img_display, detections, midpoint_cfg["focus_type"])
                set_waypoint_once(
                    state,
                    navigator,
                    midpoint_cfg["waypoint"],
                    midpoint_cfg["claw_pw"],
                    midpoint_cfg["shoa_pw"],
                    midpoint_cfg["start_msg"],
                )
                process_targeting(
                    navigator,
                    depth_frame,
                    detections,
                    midpoint_cfg["target_label"],
                    midpoint_cfg["claw_pw"],
                    midpoint_cfg["shoa_pw"],
                )

                if navigator.is_goal_reached:
                    print(midpoint_cfg["arrival_msg"])
                    state.mode = state.next_mode
                    state.targeting_active = False
                    state.midpoint_cfg = None
                    state.next_mode = None

            else:
                print(f"Unknown mode {state.mode}. Switching to pause.")
                state.mode = "pause"

            # ------------------------------------------------------------------------------
            # Show the frame
            cv2.putText(
                img_display, f"Mode: {state.mode}", (10, 30), 1, 1, (0, 255, 0), 2
            )
            cv2.imshow("Robot View", img_display)

            # REQUIRED: This allows the window to refresh and catches the 'q' key to quit
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    finally:
        engine.stop()
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

