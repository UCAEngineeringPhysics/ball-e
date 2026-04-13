
from pathlib import Path
#For servos
#--------------------------
#-------------------------
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import os
import numpy as np
import cv2
import hailo
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
        self.last_bbox_size = 0.0  # store last bounding box area/size
        # Shared variable for latest message
        self.latest_msg = "0.0, 0.0\n".encode('utf-8')
        # # self.latest_msg = "grab\n".encode('utf-8')
        # self.work_mode = "base"  # other option: "arm"

        # Start Pico update thread
        self.pico_thread = threading.Thread(target=self.send_msg, daemon=True)
        self.pico_thread.start()
        self.vel =0

        
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
    
    # # Start arm and claw in rest position
    # user_data.messenger.write(b"RELEASE\n")

    # SET resolution size
    frame_width = 1280
    frame_height = 720

    # Get the detections from the buffer
    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)
    # ~ 90 fps resolution for bb size 1280x720

    # Parse the detections
    detection_count = 0
    for detection in detections:
        label = detection.get_label()
        bbox = detection.get_bbox()
        confidence = detection.get_confidence()

        # if user_data.mode is "arm":
        #     # send_arm_moving_message_to_pico()
        #     pass
        # else:
        #     # send base moving message to pico
        #     pass


        if "ball" in label:
            # Get track ID
            ball_detected = True
            user_data.vel = 0.4
            track_id = 0
            track = detection.get_objects_typed(hailo.HAILO_UNIQUE_ID)

            # Bounding box height in pixels
            h_pixels = (bbox.ymax() - bbox.ymin()) * frame_height
            # focal length in pixels
            f_pixels = 3386.0
            # Height of ball
            H_real = 0.1524  # meters
            # Distance from camera to ball
            Z = (f_pixels * H_real) / h_pixels

            if len(track) == 1:
                track_id = track[0].get_id()
            # string_to_print += (f"Detection: ID: {track_id} Label: {label} Confidence: {confidence:.2f}\n")
            #string_to_print += (f"X Center: {(bbox.xmin() + bbox.xmax()) / 2}, Y Center: {(bbox.ymin() + bbox.ymax()) / 2}\n")
            
            # Continue at regular speed if ball is ~2 ft from camera
            if Z > 0.66: 
                if (bbox.xmin() + bbox.xmax()) / 2 < 0.3:
                    user_data.latest_msg = "0.4, 1.0\n".encode('utf-8')
                elif (bbox.xmin() + bbox.xmax()) / 2 > 0.7:
                    user_data.latest_msg = "0.4, -1.0\n".encode('utf-8')
                else:
                    user_data.latest_msg = "0.4, 0.0\n".encode('utf-8')
            # Slow down if ball is within 1.5 ft of camera
            elif Z < 0.66 and Z > 0.36:
                if (bbox.xmin() + bbox.xmax()) / 2 < 0.3:
                    user_data.latest_msg = "0.2, 1.0\n".encode('utf-8')
                elif (bbox.xmin() + bbox.xmax()) / 2 > 0.7:
                    user_data.latest_msg = "0.2, -1.0\n".encode('utf-8')
                else:
                    user_data.latest_msg = "0.2, 0.0\n".encode('utf-8')
            # Stop if ball is within 0.5 ft away from camera
            else: 
                    user_data.latest_msg = "0.0, 0.0\n".encode('utf-8')

            # Trigger arm and claw motion if ball is ~0.5 ft away from camera (robot stopped)
            if user_data.latest_msg == "0.0, 0.0\n".encode('utf-8'): 
                user_data.messenger.write(b"GRAB\n")

            detection_count += 1

            break



        # If no ball detected, gradually reduce velocity
        else:
            user_data.vel = max(user_data.vel - 0.5, 0.0)
            user_data.latest_msg = f"{user_data.vel}, 0.0\n".encode('utf-8')

    string_to_print += (f"Target velocity: {user_data.latest_msg}")
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
