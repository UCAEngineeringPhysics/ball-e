from pathlib import Path
import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib
import os
import numpy as np
import cv2
import hailo
from time import sleep
import threading
import time

from hailo_apps.hailo_app_python.core.common.buffer_utils import (
    get_caps_from_pad,
    get_numpy_from_buffer,
)
from hailo_apps.hailo_app_python.core.gstreamer.gstreamer_app import app_callback_class
from hailo_apps.hailo_app_python.apps.detection.detection_pipeline import (
    GStreamerDetectionApp,
)

import serial


# -----------------------------------------------------------------------------------------------
# User-defined class to be used in the callback function
# -----------------------------------------------------------------------------------------------
# Inheritance from the app_callback_class
class user_app_callback_class(app_callback_class):
    def __init__(self):
        super().__init__()
        self.messenger = serial.Serial(
            port="/dev/ttyACM0", baudrate=115200
        )  # New variable example
        print(f"Messenger initiated at: {self.messenger.name}\n")
        # Shared variable for latest message
        self.latest_msg = "0.0, 0.0, 0, 0\n".encode("utf-8")

        # # Initialize start time and state for arm motion
        self.arm_state = "idle"
        self.arm_state_start = 0
    
        # Start Pico update thread
        self.pico_thread = threading.Thread(target=self.send_msg, daemon=True)
        self.pico_thread.start()
        self.vel = 0


    def send_msg(self):
        """Continuously send the latest message to the Pico."""
        while True:
            self.handle_arm_state()
            if self.messenger.inWaiting() > 0:
                # print("pico msg received")
                in_msg = self.messenger.readline().strip().decode("utf-8", "ignore")
                # print(f"RPi recieved: {in_msg}")
            self.messenger.write(self.latest_msg)
            sleep(0.02)
    
    def handle_arm_state(self):
        now = time.time()

        # lower arm and open claw
        if self.arm_state == "lower":
            self.latest_msg = "0.0, 0.0, 1000, -1000\n".encode()
            if now - self.arm_start_time > 5.0:
                self.arm_state = "close"
                self.arm_start_time = now

        # close claw
        elif self.arm_state == "close":
            self.latest_msg = "0.0, 0.0, 0, 1000\n".encode()
            if now - self.arm_start_time > 5.0:
                self.arm_state = "raise"
                self.arm_start_time = now

        # raise arm
        elif self.arm_state == "raise":
            self.latest_msg = "0.0, 0.0, -1000, 0\n".encode()
            if now - self.arm_start_time > 5.0:
                self.arm_state = "idle"  # done

        # idle = normal driving
        elif self.arm_state == "idle":
            pass

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

    # Get resolution size
    (
        caps_string,
        frame_width,
        frame_height,
    ) = get_caps_from_pad(pad)
    user_data.frame_width = frame_width
    user_data.frame_height = frame_height

    # Get the detections from the buffer
    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)

    # Get the detections from the buffer
    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)

    # Parse the detections
    detection_count = 0
    for detection in detections:
        label = detection.get_label()
        bbox = detection.get_bbox()
        confidence = detection.get_confidence()

        if "ball" in label:
            # Get bounding box height in pixels
            h_pixels = (bbox.ymax() - bbox.ymin()) * user_data.frame_height
            # focal length in pixels
            f_pixels = 3386.0
            # Height of ball
            H_real = 0.1524  # meters
            # Distance from camera to ball
            Z = (f_pixels * H_real) / h_pixels

            # Get track ID
            user_data.vel = 0.4
            track_id = 0
            track = detection.get_objects_typed(hailo.HAILO_UNIQUE_ID)
            if len(track) == 1:
                track_id = track[0].get_id()
            string_to_print += f"Detection: ID: {track_id} Label: {label} Confidence: {confidence:.2f}\n"
            string_to_print += f"X Center: {(bbox.xmin() + bbox.xmax()) / 2}, Y Center: {(bbox.ymin() + bbox.ymax()) / 2}\n"

            # Drive robot until small distance from ball
            # -------------------------------------------------------------------
            # Continue at regular speed if ball is more than 2.4 m (8 ft)from camera
            if Z > 2.4:
                if (bbox.xmin() + bbox.xmax()) / 2 < 0.3:
                    user_data.latest_msg = "0.4, 1.0, 0, 0\n".encode("utf-8")
                elif (bbox.xmin() + bbox.xmax()) / 2 > 0.7:
                    user_data.latest_msg = "0.4, -1.0, 0, 0\n".encode("utf-8")
                else:
                    user_data.latest_msg = "0.4, 0.0, 0, 0\n".encode("utf-8")

            # Slow down if ball is within 2.4 m (8 ft) of camera
            elif Z < 2.4 and Z > 1:
                if (bbox.xmin() + bbox.xmax()) / 2 < 0.3:
                    user_data.latest_msg = "0.2, 1.0, 0, 0\n".encode("utf-8")
                elif (bbox.xmin() + bbox.xmax()) / 2 > 0.7:
                    user_data.latest_msg = "0.2, -1.0, 0, 0\n".encode("utf-8")
                else:
                    user_data.latest_msg = "0.2, 0.0, 0, 0\n".encode("utf-8")

            # Stop if ball is within 1 m (3.28 ft) away from camera and trigger arm motion
            else:
                # Stop wheels and start arm sequence only once
                if user_data.arm_state == "idle":
                    user_data.arm_state = "lower"
                    user_data.arm_start_time = time.time()
            
                # Freeze wheels while arm moves
                if user_data.arm_state != "idle":
                    user_data.latest_msg = "0.0, 0.0, 0, 0\n".encode()
                    return Gst.PadProbeReturn.OK

            detection_count += 1

            break

        # If no ball detected, gradually reduce velocity
    if detection_count == 0:
        user_data.vel = max(user_data.vel - 0.05, 0.0, 0, 0)
        user_data.latest_msg = f"{user_data.vel}, 0.0, 0, 0\n".encode("utf-8")

    string_to_print += f"Target velocity: {user_data.latest_msg}"
    print(string_to_print)

    return Gst.PadProbeReturn.OK

if __name__ == "__main__":
    project_root = Path(__file__).resolve().parent.parent
    env_file = project_root / ".env"
    env_path_str = str(env_file)
    os.environ["HAILO_ENV_FILE"] = env_path_str
    # Create an instance of the user app callback class
    user_data = user_app_callback_class()
    app = GStreamerDetectionApp(app_callback, user_data)
    app.run()

