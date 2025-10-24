from pathlib import Path
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import os
import numpy as np
import cv2
import hailo
from time import sleep

from hailo_apps.hailo_app_python.core.common.buffer_utils import get_caps_from_pad, get_numpy_from_buffer
from hailo_apps.hailo_app_python.core.gstreamer.gstreamer_app import app_callback_class
from hailo_apps.hailo_app_python.apps.detection.detection_pipeline import GStreamerDetectionApp

import serial

# -----------------------------------------------------------------------------------------------
# User-defined class to be used in the callback function
# -----------------------------------------------------------------------------------------------
# Inheritance from the app_callback_class
class user_app_callback_class(app_callback_class):
    def __init__(self):
        super().__init__()
        self.messenger = serial.Serial(port='/dev/ttyACM0', baudrate=115200)  # New variable example
        self.vel = 0.0  # persistent velocity state
        print(f"Messenger initiated at: {self.messenger.name}\n")

# -----------------------------------------------------------------------------------------------
# User-defined callback function
# -----------------------------------------------------------------------------------------------

# This is the callback function that will be called when data is available from the pipeline
def app_callback(pad, info, user_data):
    # Get the GstBuffer from the probe info
    buffer = info.get_buffer()
    # Check if the buffer is valid
    if buffer is None:
        return Gst.PadProbeReturn.OK

    # Using the user_data to count the number of frames
    user_data.increment()
    string_to_print = f"Frame count: {user_data.get_count()}\n"

    # Get the detections from the buffer
    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)

    ball_detected = False
    msg = "0.0, 0.0\n".encode('utf-8')

    # Grab aligned frame, depth and color: Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)

    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()

    if not depth_frame or not color_frame:
        return Gst.PadProbeReturn.OK

    depth_image = np.asanyarray(depth_frame.get_data())

    # Parse the detections
    detection_count = 0
    for detection in detections:
        label = detection.get_label()
        bbox = detection.get_bbox()
        confidence = detection.get_confidence()

        if "ball" in label:
            ball_detected = True

            # Get bounding box center (normalized coordinates)
            x_center = (bbox.xmin() + bbox.xmax()) / 2
            y_center = (bbox.ymin() + bbox.ymax()) / 2

            # Convert normalized coordinates to pixel coordinates
            h, w = depth_image.shape
            px = int(x_center * w)
            py = int(y_center * h)

            # Get depth in meters at that pixel
            depth = depth_frame.get_distance(px, py)
            string_to_print += f"Depth at ball: {depth:.2f} m\n"

            if x_center < 0.3:
                msg = "0.4, 1.0\n".encode('utf-8')
            elif x_center > 0.7:
                    msg = "0.4, -1.0\n".encode('utf-8')
            else:
                msg = "0.4, 0.0\n".encode('utf-8')
           
            detection_count += 1


    # If no ball detected, gradually reduce velocity
    if not ball_detected:
        user_data.vel = max(user_data.vel - 0.005, 0.0)
        msg = f"{user_data.vel}, 0.0\n".encode('utf-8')

    string_to_print += (f"Target velocity: {msg}")
    user_data.messenger.write(msg)
    print(string_to_print)
    return Gst.PadProbeReturn.OK

#-----------------------------------------------------------------------------------------------
# Initialize RealSense Camera
# -----------------------------------------------------------------------------------------------
import pyrealsense2 as rs

# Initialize RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

# Align depth to color frame
align_to = rs.stream.color
align = rs.align(align_to)
# -----------------------------------------------------------------------------------------------


if __name__ == "__main__":
    project_root = Path(__file__).resolve().parent.parent
    env_file     = project_root / ".env"
    env_path_str = str(env_file)
    os.environ["HAILO_ENV_FILE"] = env_path_str
    # Create an instance of the user app callback class
    user_data = user_app_callback_class()
    app = GStreamerDetectionApp(app_callback, user_data)
    app.run()
    # Stop RealSense
    try:
        app.run()
    finally:
        pipeline.stop()