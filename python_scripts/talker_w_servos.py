
from pathlib import Path
#For servos
#--------------------------
from machine import Pin, PWM, reset
#-------------------------
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
        self.last_bbox_size = 0.0  # store last bounding box area/size

        #initialize servos to resting position
        #closed position: 
        # servo_0 = PWM(Pin(15))  #claw
        # servo_0.freq(50)
        # servo_0.duty_ns(2300000)

        # servo_1 = PWM(Pin(14))  #arm
        # servo_1.freq(50)
        # servo_1.duty_ns(700000)

        #rest position: 
        servo_0 = PWM(Pin(15))  #claw
        servo_0.freq(50)
        servo_0.duty_ns(1800000)

        servo_1 = PWM(Pin(14))  #arm
        servo_1.freq(50)
        servo_1.duty_ns(1650000)
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
    # ~ 90 fps resolution for bb size 1280x720
    FRAME_HEIGHT = 720 

    ball_detected = False
    msg = "0.0, 0.0\n".encode('utf-8')

    # Parse the detections
    detection_count = 0
    for detection in detections:
        label = detection.get_label()
        bbox = detection.get_bbox()
        confidence = detection.get_confidence()

        if "ball" in label:
            # Get track ID
            ball_detected = True
            user_data.vel = 0.4

            # Bounding box height in pixels
            h_pixels = (bbox.ymax() - bbox.ymin()) * FRAME_HEIGHT
            # focal length in pixels
            f_pixels = 3386.0
            # Height of ball
            H_real = 0.1524  # meters
            # Distance from camera to ball
            Z = (f_pixels * H_real) / h_pixels

            track_id = 0
            track = detection.get_objects_typed(hailo.HAILO_UNIQUE_ID)
            if len(track) == 1:
                track_id = track[0].get_id()
            # string_to_print += (f"Detection: ID: {track_id} Label: {label} Confidence: {confidence:.2f}\n")
            string_to_print += (f"X Center: {(bbox.xmin() + bbox.xmax()) / 2}, Y Center: {(bbox.ymin() + bbox.ymax()) / 2}\n")
            
            # Continue at regular speed if ball is more than 1 ft from camera
            if Z > 0.3: 
                if (bbox.xmin() + bbox.xmax()) / 2 < 0.3:
                    msg = "0.4, 1.0\n".encode('utf-8')
                elif (bbox.xmin() + bbox.xmax()) / 2 > 0.7:
                    msg = "0.4, -1.0\n".encode('utf-8')
                else:
                    msg = "0.4, 0.0\n".encode('utf-8')
            # Slow down if ball is within 1 ft of camera
            elif Z < 0.3 and Z > 0.15:
                if (bbox.xmin() + bbox.xmax()) / 2 < 0.3:
                    msg = "0.2, 1.0\n".encode('utf-8')
                elif (bbox.xmin() + bbox.xmax()) / 2 > 0.7:
                    msg = "0.2, -1.0\n".encode('utf-8')
                else:
                    msg = "0.2, 0.0\n".encode('utf-8')
            # Stop if ball is within 0.5 ft away from camera
            else: 
                    msg = "0.0, 0.0\n".encode('utf-8')

            # Trigger arm and claw motion if ball is 0.5 ft away from camera (robot stopped)
            if msg == "0.0, 0.0\n".encode('utf-8'): 
                # Lower arm
                user_data.servo_1.duty_ns(2300000)
                sleep(0.5)
                # Open claw
                user_data.servo_0.duty_ns(1550000)
                sleep(0.5)
                # Close claw
                user_data.servo_0.duty_ns(2300000)
                sleep (0.5)
                # Raise arm
                user_data.servo_1.duty_ns(1650000)

            detection_count += 1

            break

    # If no ball detected, gradually reduce velocity
    if not ball_detected:
        user_data.vel = max(user_data.vel - 0.05, 0.0)
        msg = f"{user_data.vel}, 0.0\n".encode('utf-8')

    string_to_print += (f"Target velocity: {msg}")
    user_data.messenger.write(msg)
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
