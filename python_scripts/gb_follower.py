from bnav import BlindNavigator

import numpy as np
import cv2

import argparse
import pyrealsense2 as rs
import queue
import hailo

import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib


def transform_cam_to_odom(coords_cam, robot_pose):
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
        [-0.64, 0.0, 0.2],  # 0.64 is the gap between camera and ball
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
    parser.add_argument("--hef-path", default="../models/12_8_25.hef")
    parser.add_argument("--labels-json", default="../models/ball_bucket.json")
    # Added dummy --input argument so your existing command string works
    parser.add_argument("--input", default=None, help="Ignored: RealSense is hardcoded")
    args = parser.parse_args()

    # A. Setup navigotro
    navigator = BlindNavigator()

    # B. Setup Hailo
    engine = HailoRemoteInference(args.hef_path, args.labels_json)
    engine.start()

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
    navigator.set_goal(2.0, 0.0)

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
            # 2. Infer
            engine.infer_frame(img_color)
            detections = engine.get_latest_result()
            # 3. Process
            for label, conf, bbox in detections:
                if conf < 0.5:
                    continue
                x1 = int(bbox.xmin() * 640)  # Map bbox (0.0-1.0) to 640x480
                y1 = int(bbox.ymin() * 480)
                x2 = int(bbox.xmax() * 640)
                y2 = int(bbox.ymax() * 480)
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                # Compute goal coords
                depth_in_meters = depth_frame.get_distance(cx, cy)
                # print(f"depth: {depth_in_meters}m")  # debug
                if depth_in_meters > 0:
                    intrinsics = (
                        depth_frame.profile.as_video_stream_profile().intrinsics
                    )
                    coords_cam = rs.rs2_deproject_pixel_to_point(
                        intrinsics,
                        [cx, cy],
                        depth_in_meters,
                    )  # Convert 2D pixel to 3D point (X, Y, Z in meters)
                    # print(
                    #     f"Object {label} at X:{coords_cam[0]:.2f}m, Y:{coords_cam[1]:.2f}m, Z:{coords_cam[2]:.2f}m"
                    # )  # debug
                    goal_coords = transform_cam_to_odom(
                        (coords_cam[0], coords_cam[1], coords_cam[2]),
                        (navigator.x, navigator.y, navigator.theta),
                    )
                    print(f"Goal coors in odom: {goal_coords}")
                    if (
                        np.linalg.norm(
                            np.array(goal_coords[:2])
                            - np.array((navigator.goal_x, navigator.goal_y))
                        )
                        > 0.02
                    ):  # update goal when necessary
                        navigator.set_goal(goal_coords[0], goal_coords[1])
                        print(f"Set goal at: {goal_coords}")
                    print(f"robot pose: {navigator.x, navigator.y, navigator.theta}")

                    # Draw
                    cv2.rectangle(img_display, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    # cv2.putText(
                    #     img_display,
                    #     f"{label}",
                    #     cv2.FONT_HERSHEY_SIMPLEX,
                    #     0.6,
                    #     (0, 255, 0),
                    #     2,
                    # )

    finally:
        engine.stop()
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
