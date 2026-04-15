from blind_navigator_el import BlindNavigator

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
                    dist_offset = -0.61
                else:
                    dist_offset = -0.47
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

    # Create a state object to hold modes and counters
    state = SimpleNamespace(
        mode="fixed_ball_1",
        arm_state="idle",
        picker_counter=0,
        lap_counter=0,
        targeting_active=False,  # Initialize as False
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

            if state.mode == "pause":
                navigator.manual_override_msg = "0.0,0.0,1700000,1500000\n"

            # *******************************         ARM          ****************************************************

            elif state.mode == "pick_1":
                # # Always reset arm state when entering pick
                if state.arm_state == "idle":
                    state.arm_state = "lower"
                    navigator.manual_override_msg = "0.0,0.0,1700000,500000\n"
                    sleep(0.2)
                if state.arm_state == "lower":
                    navigator.manual_override_msg = "0.0,0.0,1700000,500000\n"
                    sleep(0.1)
                    if navigator.goal_status == 1:
                        state.arm_state = "close"
                        navigator.manual_override_msg = "0.0,0.0,1080000,500000\n"
                        sleep(0.2)

                elif state.arm_state == "close":
                    navigator.manual_override_msg = "0.0,0.0,1080000,500000\n"
                    sleep(0.1)
                    if navigator.goal_status == 1:
                        state.arm_state = "raise"
                        navigator.manual_override_msg = "0.0,0.0,1080000,1500000\n"
                        sleep(0.2)

                elif state.arm_state == "raise":
                    navigator.manual_override_msg = "0.0,0.0,1080000,1500000\n"
                    sleep(0.1)
                    if navigator.goal_status == 1:
                        state.mode = "fixed_bucket_1"  # "pause" for testing
                        state.arm_state = "idle"
                        state.targeting_active = False  # Reset

            elif state.mode == "drop_1":
                # # Always reset arm state when entering pick
                if state.arm_state == "idle":
                    state.arm_state = "lower"
                    navigator.manual_override_msg = "0.0,0.0,1080000,1100000\n"
                    sleep(0.2)
                if state.arm_state == "lower":
                    navigator.manual_override_msg = "0.0,0.0,1080000,1100000\n"
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
                        state.mode = "mid_1"  # "pause" for testing
                        state.arm_state = "idle"
                        state.targeting_active = False  # Reset

            elif state.mode == "pick_2":
                # # Always reset arm state when entering pick
                if state.arm_state == "idle":
                    state.arm_state = "lower"
                    navigator.manual_override_msg = "0.0,0.0,1700000,500000\n"
                    sleep(0.2)
                if state.arm_state == "lower":
                    navigator.manual_override_msg = "0.0,0.0,1700000,500000\n"
                    sleep(0.1)
                    if navigator.goal_status == 1:
                        state.arm_state = "close"
                        navigator.manual_override_msg = "0.0,0.0,1080000,500000\n"
                        sleep(0.2)

                elif state.arm_state == "close":
                    navigator.manual_override_msg = "0.0,0.0,1080000,500000\n"
                    sleep(0.1)
                    if navigator.goal_status == 1:
                        state.arm_state = "raise"
                        navigator.manual_override_msg = "0.0,0.0,1080000,1500000\n"
                        sleep(0.2)

                elif state.arm_state == "raise":
                    navigator.manual_override_msg = "0.0,0.0,1080000,1500000\n"
                    sleep(0.1)
                    if navigator.goal_status == 1:
                        state.mode = "fixed_bucket_2"  # "pause" for testing
                        state.arm_state = "idle"
                        state.targeting_active = False  # Reset

            elif state.mode == "drop_2":
                # # Always reset arm state when entering pick
                if state.arm_state == "idle":
                    state.arm_state = "lower"
                    navigator.manual_override_msg = "0.0,0.0,1080000,1100000\n"
                    sleep(0.2)
                if state.arm_state == "lower":
                    navigator.manual_override_msg = "0.0,0.0,1080000,1100000\n"
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
                        state.mode = "fixed_ball_3"  # "fixed_ball_3" for testing
                        state.arm_state = "idle"
                        state.targeting_active = False  # Reset

            elif state.mode == "pick_3":
                # # Always reset arm state when entering pick
                if state.arm_state == "idle":
                    state.arm_state = "lower"
                    navigator.manual_override_msg = "0.0,0.0,1700000,500000\n"
                    sleep(0.2)
                if state.arm_state == "lower":
                    navigator.manual_override_msg = "0.0,0.0,1700000,500000\n"
                    sleep(0.1)
                    if navigator.goal_status == 1:
                        state.arm_state = "close"
                        navigator.manual_override_msg = "0.0,0.0,1080000,500000\n"
                        sleep(0.2)

                elif state.arm_state == "close":
                    navigator.manual_override_msg = "0.0,0.0,1080000,500000\n"
                    sleep(0.1)
                    if navigator.goal_status == 1:
                        state.arm_state = "raise"
                        navigator.manual_override_msg = "0.0,0.0,1080000,1500000\n"
                        sleep(0.2)

                elif state.arm_state == "raise":
                    navigator.manual_override_msg = "0.0,0.0,1080000,1500000\n"
                    sleep(0.1)
                    if navigator.goal_status == 1:
                        state.mode = "fixed_bucket_3"  # "pause" for testing
                        state.arm_state = "idle"
                        state.targeting_active = False  # Reset

            elif state.mode == "drop_3":
                # # Always reset arm state when entering pick
                if state.arm_state == "idle":
                    state.arm_state = "lower"
                    navigator.manual_override_msg = "0.0,0.0,1080000,1100000\n"
                    sleep(0.2)
                if state.arm_state == "lower":
                    navigator.manual_override_msg = "0.0,0.0,1080000,1100000\n"
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
                        state.mode = "fixed_ball_4"  # "pause" for testing
                        state.arm_state = "idle"
                        state.targeting_active = False  # Reset

            elif state.mode == "pick_4":
                # # Always reset arm state when entering pick
                if state.arm_state == "idle":
                    state.arm_state = "lower"
                    navigator.manual_override_msg = "0.0,0.0,1700000,500000\n"
                    sleep(0.2)
                if state.arm_state == "lower":
                    navigator.manual_override_msg = "0.0,0.0,1700000,500000\n"
                    sleep(0.1)
                    if navigator.goal_status == 1:
                        state.arm_state = "close"
                        navigator.manual_override_msg = "0.0,0.0,1080000,500000\n"
                        sleep(0.2)

                elif state.arm_state == "close":
                    navigator.manual_override_msg = "0.0,0.0,1080000,500000\n"
                    sleep(0.1)
                    if navigator.goal_status == 1:
                        state.arm_state = "raise"
                        navigator.manual_override_msg = "0.0,0.0,1080000,1500000\n"
                        sleep(0.2)

                elif state.arm_state == "raise":
                    navigator.manual_override_msg = "0.0,0.0,1080000,1500000\n"
                    sleep(0.1)
                    if navigator.goal_status == 1:
                        state.mode = "mid_2"  # "pause" for testing
                        state.arm_state = "idle"
                        state.targeting_active = False  # Reset

            elif state.mode == "drop_4":
                # # Always reset arm state when entering pick
                if state.arm_state == "idle":
                    state.arm_state = "lower"
                    navigator.manual_override_msg = "0.0,0.0,1080000,1100000\n"
                    sleep(0.2)
                if state.arm_state == "lower":
                    navigator.manual_override_msg = "0.0,0.0,1080000,1100000\n"
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
                        state.mode = "pause"  # "pause" for testing
                        state.arm_state = "idle"
                        state.targeting_active = False  # Reset

            # *******************************         MOTORS          ****************************************************

            elif state.mode == "fixed_ball_1":
                # Regular odometry driving (set string to empty)
                navigator.manual_override_msg = ""
                # 2. Infer
                engine.infer_frame(img_color)
                detections = engine.get_latest_result()

                # ************************************************************************
                # Display obj detection in real time
                for label, conf, bbox in detections:
                    # Convert normalized coordinates (0.0-1.0) to pixel coordinates
                    x1 = int(bbox.xmin() * 640)
                    y1 = int(bbox.ymin() * 480)
                    x2 = int(bbox.xmax() * 640)
                    y2 = int(bbox.ymax() * 480)

                    # Draw the box (Green for ball, Blue for bucket, etc.)
                    color = (0, 255, 0) if label == "ball" else (255, 0, 0)
                    cv2.rectangle(img_display, (x1, y1), (x2, y2), color, 2)

                    # Add Label and Confidence
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
                # ************************************************************************

                # 1. Set first way point - targeting_active is to help prevent resetting to OG way point during obj detection
                if not state.targeting_active:
                    print("Setting waypoint for 1st ball...")
                    navigator.set_goal(
                        4.0, 4.0, 1700000, 1500000
                    )  # Coordinates for first way point
                    state.targeting_active = True
                    # sleep(0.1)

                # 2. Use obj detection (for *ball* specifically) to improve/update way point
                process_targeting(
                    navigator, depth_frame, detections, "blue ball", 1700000, 1500000
                )

                # 3. Robot has arrived at ball, switch modes
                if navigator.is_goal_reached:
                    print("Arrived at 1st ball. Switching to 1st pick.")
                    state.mode = "pick_1"
                    state.arm_state = "lower"
                    state.targeting_active = False

            elif state.mode == "fixed_bucket_1":
                # Regular odometry driving (set string to empty)
                navigator.manual_override_msg = ""

                # 2. Infer
                engine.infer_frame(img_color)
                detections = engine.get_latest_result()

                # ************************************************************************
                # Display obj detection in real time
                for label, conf, bbox in detections:
                    # Convert normalized coordinates (0.0-1.0) to pixel coordinates
                    x1 = int(bbox.xmin() * 640)
                    y1 = int(bbox.ymin() * 480)
                    x2 = int(bbox.xmax() * 640)
                    y2 = int(bbox.ymax() * 480)

                    # Draw the box (Green for ball, Blue for bucket, etc.)
                    color = (0, 255, 0) if label == "bucket" else (255, 0, 0)
                    cv2.rectangle(img_display, (x1, y1), (x2, y2), color, 2)

                    # Add Label and Confidence
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
                # ************************************************************************

                # 1. Set first way point - targeting_active is to help prevent resetting to OG way point during obj detection
                if not state.targeting_active:
                    print("Setting waypoint for 1st bucket...")
                    navigator.set_goal(
                        1.0, 1.0, 1080000, 1500000
                    )  # Coordinates for first way point
                    state.targeting_active = True
                    # sleep(0.1)

                # 2. Use obj detection (for *ball* specifically) to improve/update way point
                process_targeting(
                    navigator, depth_frame, detections, "blue bucket", 1080000, 1500000
                )

                # 3. Robot has arrived at ball, switch modes
                if navigator.is_goal_reached:
                    print("Arrived at 1st bucket. Switching to 1st drop.")
                    state.mode = "drop_1"
                    state.arm_state = "idle"
                    state.targeting_active = False

            elif state.mode == "fixed_ball_2":
                # Regular odometry driving (set string to empty)
                navigator.manual_override_msg = ""
                # 2. Infer
                engine.infer_frame(img_color)
                detections = engine.get_latest_result()

                # ************************************************************************
                # Display obj detection in real time
                for label, conf, bbox in detections:
                    # Convert normalized coordinates (0.0-1.0) to pixel coordinates
                    x1 = int(bbox.xmin() * 640)
                    y1 = int(bbox.ymin() * 480)
                    x2 = int(bbox.xmax() * 640)
                    y2 = int(bbox.ymax() * 480)

                    # Draw the box (Green for ball, Blue for bucket, etc.)
                    color = (0, 255, 0) if label == "ball" else (255, 0, 0)
                    cv2.rectangle(img_display, (x1, y1), (x2, y2), color, 2)

                    # Add Label and Confidence
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
                # ************************************************************************

                # 1. Set first way point - targeting_active is to help prevent resetting to OG way point during obj detection
                if not state.targeting_active:
                    print("Setting waypoint for 2nd ball...")
                    navigator.set_goal(
                        5.3, 4.0, 1700000, 1500000
                    )  # Coordinates for first way point
                    state.targeting_active = True
                    # sleep(0.1)

                # 2. Use obj detection (for *ball* specifically) to improve/update way point
                process_targeting(
                    navigator, depth_frame, detections, "red ball", 1700000, 1500000
                )

                # 3. Robot has arrived at ball, switch modes
                if navigator.is_goal_reached:
                    print("Arrived at 2nd ball. Switching to 2nd pick.")
                    state.mode = "pick_2"
                    state.arm_state = "lower"
                    state.targeting_active = False

            elif state.mode == "fixed_bucket_2":
                # Regular odometry driving (set string to empty)
                navigator.manual_override_msg = ""

                # 2. Infer
                engine.infer_frame(img_color)
                detections = engine.get_latest_result()

                # ************************************************************************
                # Display obj detection in real time
                for label, conf, bbox in detections:
                    # Convert normalized coordinates (0.0-1.0) to pixel coordinates
                    x1 = int(bbox.xmin() * 640)
                    y1 = int(bbox.ymin() * 480)
                    x2 = int(bbox.xmax() * 640)
                    y2 = int(bbox.ymax() * 480)

                    # Draw the box (Green for ball, Blue for bucket, etc.)
                    color = (0, 255, 0) if label == "bucket" else (255, 0, 0)
                    cv2.rectangle(img_display, (x1, y1), (x2, y2), color, 2)

                    # Add Label and Confidence
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
                # ************************************************************************

                # 1. Set first way point - targeting_active is to help prevent resetting to OG way point during obj detection
                if not state.targeting_active:
                    print("Setting waypoint for 2nd bucket...")
                    navigator.set_goal(
                        8.3, 8.7, 1080000, 1500000
                    )  # Coordinates for first way point
                    state.targeting_active = True
                    # sleep(0.1)

                # 2. Use obj detection (for *ball* specifically) to improve/update way point
                process_targeting(
                    navigator, depth_frame, detections, "red bucket", 1080000, 1500000
                )

                # 3. Robot has arrived at ball, switch modes
                if navigator.is_goal_reached:
                    print("Arrived at 2nd bucket. Switching to 2nd drop.")
                    state.mode = "drop_2"
                    state.arm_state = "idle"
                    state.targeting_active = False

            elif state.mode == "fixed_ball_3":
                # Regular odometry driving (set string to empty)
                navigator.manual_override_msg = ""
                # 2. Infer
                engine.infer_frame(img_color)
                detections = engine.get_latest_result()

                # ************************************************************************
                # Display obj detection in real time
                for label, conf, bbox in detections:
                    # Convert normalized coordinates (0.0-1.0) to pixel coordinates
                    x1 = int(bbox.xmin() * 640)
                    y1 = int(bbox.ymin() * 480)
                    x2 = int(bbox.xmax() * 640)
                    y2 = int(bbox.ymax() * 480)

                    # Draw the box (Green for ball, Blue for bucket, etc.)
                    color = (0, 255, 0) if label == "ball" else (255, 0, 0)
                    cv2.rectangle(img_display, (x1, y1), (x2, y2), color, 2)

                    # Add Label and Confidence
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
                # ************************************************************************

                # 1. Set first way point - targeting_active is to help prevent resetting to OG way point during obj detection
                if not state.targeting_active:
                    print("Setting waypoint for 3rd ball...")
                    navigator.set_goal(
                        5.3, 5.3, 1700000, 1500000
                    )  # Coordinates for first way point
                    state.targeting_active = True
                    # sleep(0.1)

                # 2. Use obj detection (for *ball* specifically) to improve/update way point
                process_targeting(
                    navigator, depth_frame, detections, "yellow ball", 1700000, 1500000
                )

                # 3. Robot has arrived at ball, switch modes
                if navigator.is_goal_reached:
                    print("Arrived at 3rd ball. Switching to 3rd pick.")
                    state.mode = "pick_3"
                    state.arm_state = "lower"
                    state.targeting_active = False

            elif state.mode == "fixed_bucket_3":
                # Regular odometry driving (set string to empty)
                navigator.manual_override_msg = ""

                # 2. Infer
                engine.infer_frame(img_color)
                detections = engine.get_latest_result()

                # ************************************************************************
                # Display obj detection in real time
                for label, conf, bbox in detections:
                    # Convert normalized coordinates (0.0-1.0) to pixel coordinates
                    x1 = int(bbox.xmin() * 640)
                    y1 = int(bbox.ymin() * 480)
                    x2 = int(bbox.xmax() * 640)
                    y2 = int(bbox.ymax() * 480)

                    # Draw the box (Green for ball, Blue for bucket, etc.)
                    color = (0, 255, 0) if label == "bucket" else (255, 0, 0)
                    cv2.rectangle(img_display, (x1, y1), (x2, y2), color, 2)

                    # Add Label and Confidence
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
                # ************************************************************************

                # 1. Set first way point - targeting_active is to help prevent resetting to OG way point during obj detection
                if not state.targeting_active:
                    print("Setting waypoint for 3rd bucket...")
                    navigator.set_goal(
                        1.0, 8.3, 1080000, 1500000
                    )  # Coordinates for first way point
                    state.targeting_active = True
                    # sleep(0.1)

                # 2. Use obj detection (for *ball* specifically) to improve/update way point
                process_targeting(
                    navigator, depth_frame, detections, "", 1080000, 1500000
                )  # UNLABELED

                # 3. Robot has arrived at ball, switch modes
                if navigator.is_goal_reached:
                    print("Arrived at 3rd bucket. Switching to 3rd drop.")
                    state.mode = "drop_3"
                    state.arm_state = "idle"
                    state.targeting_active = False

            elif state.mode == "fixed_ball_4":
                # Regular odometry driving (set string to empty)
                navigator.manual_override_msg = ""
                # 2. Infer
                engine.infer_frame(img_color)
                detections = engine.get_latest_result()

                # ************************************************************************
                # Display obj detection in real time
                for label, conf, bbox in detections:
                    # Convert normalized coordinates (0.0-1.0) to pixel coordinates
                    x1 = int(bbox.xmin() * 640)
                    y1 = int(bbox.ymin() * 480)
                    x2 = int(bbox.xmax() * 640)
                    y2 = int(bbox.ymax() * 480)

                    # Draw the box (Green for ball, Blue for bucket, etc.)
                    color = (0, 255, 0) if label == "ball" else (255, 0, 0)
                    cv2.rectangle(img_display, (x1, y1), (x2, y2), color, 2)

                    # Add Label and Confidence
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
                # ************************************************************************

                # 1. Set first way point - targeting_active is to help prevent resetting to OG way point during obj detection
                if not state.targeting_active:
                    print("Setting waypoint for 4th ball...")
                    navigator.set_goal(
                        4.0, 5.3, 1700000, 1500000
                    )  # Coordinates for first way point
                    state.targeting_active = True
                    # sleep(0.1)

                # 2. Use obj detection (for *ball* specifically) to improve/update way point
                process_targeting(
                    navigator, depth_frame, detections, "green ball", 1700000, 1500000
                )

                # 3. Robot has arrived at ball, switch modes
                if navigator.is_goal_reached:
                    print("Arrived at 4th ball. Switching to 4th pick.")
                    state.mode = "pick_4"
                    state.arm_state = "lower"
                    state.targeting_active = False

            elif state.mode == "fixed_bucket_4":
                # Regular odometry driving (set string to empty)
                navigator.manual_override_msg = ""

                # 2. Infer
                engine.infer_frame(img_color)
                detections = engine.get_latest_result()

                # ************************************************************************
                # Display obj detection in real time
                for label, conf, bbox in detections:
                    # Convert normalized coordinates (0.0-1.0) to pixel coordinates
                    x1 = int(bbox.xmin() * 640)
                    y1 = int(bbox.ymin() * 480)
                    x2 = int(bbox.xmax() * 640)
                    y2 = int(bbox.ymax() * 480)

                    # Draw the box (Green for ball, Blue for bucket, etc.)
                    color = (0, 255, 0) if label == "bucket" else (255, 0, 0)
                    cv2.rectangle(img_display, (x1, y1), (x2, y2), color, 2)

                    # Add Label and Confidence
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
                # ************************************************************************

                # 1. Set first way point - targeting_active is to help prevent resetting to OG way point during obj detection
                if not state.targeting_active:
                    print("Setting waypoint for 4th bucket...")
                    navigator.set_goal(
                        8.3, 2.0, 1080000, 1500000
                    )  # Coordinates for first way point
                    state.targeting_active = True
                    # sleep(0.1)

                # 2. Use obj detection (for *ball* specifically) to improve/update way point
                process_targeting(
                    navigator, depth_frame, detections, "green bucket", 1080000, 1500000
                )

                # 3. Robot has arrived at ball, switch modes
                if navigator.is_goal_reached:
                    print("Arrived at 4th bucket. Switching to 4th drop.")
                    state.mode = "drop_4"
                    state.arm_state = "idle"
                    state.targeting_active = False

            # *******************************         MIDPOINTS          ****************************************************

            elif state.mode == "mid_1":
                # Regular odometry driving (set string to empty)
                navigator.manual_override_msg = ""
                # 2. Infer
                engine.infer_frame(img_color)
                detections = engine.get_latest_result()

                # ************************************************************************
                # Display obj detection in real time
                for label, conf, bbox in detections:
                    # Convert normalized coordinates (0.0-1.0) to pixel coordinates
                    x1 = int(bbox.xmin() * 640)
                    y1 = int(bbox.ymin() * 480)
                    x2 = int(bbox.xmax() * 640)
                    y2 = int(bbox.ymax() * 480)

                    # Draw the box (Green for ball, Blue for bucket, etc.)
                    color = (0, 255, 0) if label == "ball" else (255, 0, 0)
                    cv2.rectangle(img_display, (x1, y1), (x2, y2), color, 2)

                    # Add Label and Confidence
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
                # ************************************************************************

                # 1. Set first way point - targeting_active is to help prevent resetting to OG way point during obj detection
                if not state.targeting_active:
                    print("Setting mid waypoint...")
                    navigator.set_goal(
                        7.0, 2.5, 1700000, 1500000
                        
                    )  # Coordinates for first way point
                    state.targeting_active = True
                    # sleep(0.1)

                # # 2. Use obj detection (for *ball* specifically) to improve/update way point
                process_targeting(
                    navigator, depth_frame, detections, "eric", 1700000, 1500000
                )

                # 3. Robot has arrived at ball, switch modes
                if navigator.is_goal_reached:
                    print("Arrived at midpoint location. Switching to 2nd ball search.")
                    state.mode = "fixed_ball_2"
                    state.targeting_active = False

            elif state.mode == "mid_2":
                # Regular odometry driving (set string to empty)
                navigator.manual_override_msg = ""
                # 2. Infer
                engine.infer_frame(img_color)
                detections = engine.get_latest_result()
                # ************************************************************************
                # Display obj detection in real time
                for label, conf, bbox in detections:
                    # Convert normalized coordinates (0.0-1.0) to pixel coordinates
                    x1 = int(bbox.xmin() * 640)
                    y1 = int(bbox.ymin() * 480)
                    x2 = int(bbox.xmax() * 640)
                    y2 = int(bbox.ymax() * 480)

                    # Draw the box (Green for ball, Blue for bucket, etc.)
                    color = (0, 255, 0) if label == "ball" else (255, 0, 0)
                    cv2.rectangle(img_display, (x1, y1), (x2, y2), color, 2)

                    # Add Label and Confidence
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
                # ************************************************************************

                # 1. Set first way point - targeting_active is to help prevent resetting to OG way point during obj detection
                if not state.targeting_active:
                    print("Setting mid way point...")
                    navigator.set_goal(
                        5.0, 5.5, 1080000, 1500000
                    )  # Coordinates for first way point
                    state.targeting_active = True
                    # sleep(0.1)

                # # 2. Use obj detection (for *ball* specifically) to improve/update way point
                process_targeting(
                    navigator, depth_frame, detections, "red ball", 1080000, 1500000
                )

                # 3. Robot has arrived at ball, switch modes
                if navigator.is_goal_reached:
                    print("Arrived at midpoint location. Switching to 4th ball search.")
                    state.mode = "fixed_bucket_4"
                    state.targeting_active = False

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
