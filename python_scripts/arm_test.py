
from pathlib import Path
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import os
import numpy as np
import cv2
import hailo
import time
from time import sleep

import threading

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
        print(f"Messenger initiated at: {self.messenger.name}\n")
        # Shared variable for latest message
        self.latest_msg = "0.0, 0.0, 0, 0\n".encode('utf-8')
        
        # Start Pico update thread
        self.pico_thread = threading.Thread(target=self.send_msg, daemon=True)
        self.pico_thread.start()
        self.vel = 0
        
        self.mode = "fixed_ball"
        self.arm_state = "idle"
        self.fixed_travel_counter = 0
        self.picker_counter = 0

        
    def send_msg(self):
        """Continuously send the latest message to the Pico."""
        while True:
            if self.messenger.inWaiting() > 0:
                # print("pico msg received")
                in_msg = self.messenger.readline().strip().decode("utf-8", "ignore")
                # print(f"RPi recieved: {in_msg}")
            self.messenger.write(self.latest_msg)
            sleep(0.02)


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

    if user_data.mode == "pause":
        user_data.latest_msg = "0.0, 0.0, 0, 0\n".encode('utf-8')
                
    elif user_data.mode == "pick":
        user_data.latest_msg = "0.0, 0.0, 0, 0\n".encode('utf-8')
        if user_data.arm_state == "lower":
            user_data.latest_msg = "0.0, 0.0, 3000, 0\n".encode('utf-8')
            user_data.picker_counter += 1
            if user_data.picker_counter >= 210:
                user_data.arm_state = "close"
                user_data.picker_counter = 0
                
        elif user_data.arm_state == "close":
            user_data.latest_msg = "0.0, 0.0, 0, 3000\n".encode('utf-8')
            user_data.picker_counter += 1
            if user_data.picker_counter >= 70:
                user_data.arm_state = "raise"
                user_data.picker_counter = 0
                
        elif user_data.arm_state == "raise":
            user_data.latest_msg = "0.0, 0.0, -3000, 0\n".encode('utf-8')
            user_data.picker_counter += 1
            if user_data.picker_counter >= 180:
                user_data.arm_state = "idle"

                user_data.picker_counter = 0
        # idle = normal driving
        elif user_data.arm_state == "idle":
            pass         
        
    # elif user_data.mode == "drop":
    #     user_data.latest_msg = "0.0, 0.0, 0, 0\n".encode('utf-8')
    #     if user_data.arm_state == "lower":
    #         user_data.latest_msg = "0.0, 0.0, 3000, 0\n".encode('utf-8')
    #         user_data.picker_counter += 1
    #         if user_data.picker_counter >= 80:
    #             user_data.arm_state = "open"
    #             user_data.picker_counter = 0          
                
    #     elif user_data.arm_state == "open":
    #         user_data.latest_msg = "0.0, 0.0, 0, -3000\n".encode('utf-8')
    #         user_data.picker_counter += 1
    #         if user_data.picker_counter >= 40:
    #             #Return both arm and claw to neutral
    #             user_data.arm_state = "swivel_large_right"
    #             user_data.picker_counter = 0
                          
            
    elif user_data.mode == "fixed_ball":
        user_data.latest_msg = "0.0, 0.0, 0, 0\n".encode('utf-8')
        user_data.fixed_travel_counter += 1
        if user_data.fixed_travel_counter >= 20:
            user_data.mode = "pick"
            user_data.arm_state = "lower"
            user_data.fixed_travel_counter = 0
            user_data.latest_msg = "0.0, 0.0, 0, 0\n".encode('utf-8')
            




            
                  
    elif user_data.mode == "detect":
        #Travel based on obj detection
        # Get the detections from the buffer
        roi = hailo.get_roi_from_buffer(buffer)
        detections = roi.get_objects_typed(hailo.HAILO_DETECTION)

        if len(detections):            
            for detection in detections:
                label = detection.get_label()
                bbox = detection.get_bbox()
                confidence = detection.get_confidence()

            if "ball" not in label:   
                print("ball not in label")         
                user_data.vel = max(user_data.vel - 0.05, 0.0)
                user_data.latest_msg = f"{user_data.vel}, 0.0\n".encode('utf-8')               
            else:
                # Get bounding box height in pixels
                h_pixels = (bbox.ymax() - bbox.ymin()) * user_data.frame_height
                # focal length in pixels
                f_pixels = 3386.0
                # Height of ball
                H_real = 0.1524  # meters
                # Distance from camera to ball
                # Z = (f_pixels * H_real) / h_pixels
                user_data.distance = (f_pixels * H_real) / h_pixels

                # Get track ID
                user_data.vel = 0.4
                track_id = 0
                track = detection.get_objects_typed(hailo.HAILO_UNIQUE_ID)
                if len(track) == 1:
                    track_id = track[0].get_id()
                string_to_print += (f"Detection: ID: {track_id} Label: {label} Confidence: {confidence:.2f}\n")
                string_to_print += (f"X Center: {(bbox.xmin() + bbox.xmax()) / 2}, Y Center: {(bbox.ymin() + bbox.ymax()) / 2}\n")
                # if Z > 2.4:
                if user_data.distance >= 2.4:
                    if (bbox.xmin() + bbox.xmax()) / 2 < 0.45:
                        user_data.latest_msg = "0.2, 0.5, 0, 0\n".encode('utf-8')
                    elif (bbox.xmin() + bbox.xmax()) / 2 > 0.7:
                        user_data.latest_msg = "0.2, -0.5, 0, 0\n".encode('utf-8')
                    else:
                        user_data.latest_msg = "0.2, 0.0, 0, 0\n".encode('utf-8')
                # elif Z <= 2.4 and Z > 1.0:
                elif 1.237 < user_data.distance <= 2.4:
                    if (bbox.xmin() + bbox.xmax()) / 2 < 0.5:
                        user_data.latest_msg = "0.1, 0.5, 0, 0\n".encode("utf-8")
                    elif (bbox.xmin() + bbox.xmax()) / 2 > 0.7:
                        user_data.latest_msg = "0.1, -0.5, 0, 0\n".encode("utf-8")
                    else:
                        user_data.latest_msg = "0.1, 0.0, 0, 0\n".encode("utf-8")
                else:
                    # Stop wheels and start arm sequence only once
                    user_data.latest_msg = "0.0, 0.0, 0, 0\n".encode()
                    user_data.arm_state = "lower"
                    user_data.mode = "pick"
        else:  # no detection
            user_data.latest_msg = "0.0, 0.0, 0, 0\n".encode()


    string_to_print += (f"Target velocity: {user_data.latest_msg}")
    print(user_data.mode)
    print(string_to_print)
    return Gst.PadProbeReturn.OK

if __name__ == "__main__":
    project_root = Path(__file__).resolve().parent.parent
    env_file     = project_root / ".env"
    env_path_str = str(env_file)
    os.environ["HAILO_ENV_FILE"] = env_path_str
    # Create an instance of the user app callback class
    user_data = user_app_callback_class()
    app = GStreamerDetectionApp(app_callback, user_data)
    app.run()