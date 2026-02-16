from pathlib import Path
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import os
import numpy as np
import cv2
from time import sleep
import threading
import serial
import queue
import pyrealsense2 as rs
import argparse

# Local application-specific imports
import hailo
from hailo_apps.hailo_app_python.core.common.buffer_utils import get_caps_from_pad, get_numpy_from_buffer
from hailo_apps.hailo_app_python.core.gstreamer.gstreamer_app import app_callback_class
from hailo_apps.hailo_app_python.apps.detection_simple.detection_pipeline_simple import GStreamerDetectionApp
# endregion imports

#*********************************IMPROVEMENTS************************************************
#Increase speed during encoder portions
#Make turning threshold smaller to try to improve centering (decrease 0.3 and 0.7)


# User-defined class to be used in the callback function: Inheritance from the app_callback_class
class user_app_callback_class(app_callback_class):
    def __init__(self):
        super().__init__()

        self.depth_frame = None
        
        self.messenger = serial.Serial(port='/dev/ttyACM0', baudrate=115200)  # New variable example
        print(f"Messenger initiated at: {self.messenger.name}\n")
        # Shared variable for latest message
        self.latest_msg = "0.0, 0.0, 0, 0, 0\n".encode('utf-8')
        
        # Start Pico update thread
        self.pico_thread = threading.Thread(target=self.send_msg, daemon=True)
        self.pico_thread.start()
        self.vel =0
        
        self.mode = "fixed_ball"
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


# User-defined callback function: This is the callback function that will be called when data is available from the pipeline
def app_callback(pad, info, user_data):

    # Get latest depth frame
    frames = rs_pipeline.poll_for_frames()
    if frames:
        aligned = rs_align.process(frames)
        user_data.depth_frame = aligned.get_depth_frame()

    user_data.increment()  # Using the user_data to count the number of frames
    string_to_print = f"Frame count: {user_data.get_count()}\n"

    buffer = info.get_buffer()  # Get the GstBuffer from the probe info
    if buffer is None:  # Check if the buffer is valid
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
            user_data.latest_msg = "0.0, 0.0, 0, 0, 0\n".encode('utf-8')

    elif user_data.mode == "detect":

        roi = hailo.get_roi_from_buffer(buffer)
        detections = roi.get_objects_typed(hailo.HAILO_DETECTION)

        if len(detections):            
            for detection in detections:
            #for detection in hailo.get_roi_from_buffer(buffer).get_objects_typed(hailo.HAILO_DETECTION):  # Get the detections from the buffer & Parse the detections

                label = detection.get_label()
                bbox = detection.get_bbox()
                confidence = detection.get_confidence()

            if "ball" in label:
                # Pixel coordinates
                x_center = int(((bbox.xmin() + bbox.xmax()) / 2) * user_data.frame_width)
                y_center = int(((bbox.ymin() + bbox.ymax()) / 2) * user_data.frame_height)

                # ---------------- HW DEPTH ----------------
                hw_dist = 0
                if user_data.depth_frame:
                    hw_dist = user_data.depth_frame.get_distance(x_center, y_center)

                # ---------------- FALLBACK MATH ----------------
                # Get bounding box height in pixels
                h_pixels = (bbox.ymax() - bbox.ymin()) * user_data.frame_height
                # focal length in pixels
                f_pixels = 3386.0
                # Height of bucket
                H_real = 0.381  # meters

                calc_dist = (f_pixels * H_real) / h_pixels if h_pixels > 0 else 0
                # ---------------- FINAL DIST ----------------
                # Distance from camera to bucket
                user_data.distance = hw_dist if hw_dist > 0.1 else calc_dist

                # Get track ID
                user_data.vel = 0.4
                track_id = 0
                track = detection.get_objects_typed(hailo.HAILO_UNIQUE_ID)
                if len(track) == 1:
                    track_id = track[0].get_id()
                string_to_print += (f"Detection: ID: {track_id} Label: {label} Confidence: {confidence:.2f}\n")
                string_to_print += (f"X Center: {(bbox.xmin() + bbox.xmax()) / 2}, Y Center: {(bbox.ymin() + bbox.ymax()) / 2}\n")

                # if Z > 5.0:
                if user_data.distance >= 5.0:            
                    if (bbox.xmin() + bbox.xmax()) / 2 < 0.3:
                        user_data.latest_msg = "0.2, 0.5,0, 0, 0\n".encode('utf-8')
                    elif (bbox.xmin() + bbox.xmax()) / 2 > 0.7:
                        user_data.latest_msg = "0.2, -0.5,0, 0, 0\n".encode('utf-8')
                    else:
                        user_data.latest_msg = "0.35, 0.0,0, 0, 0\n".encode('utf-8')

                # elif Z <= 3.5 and Z > 5.0:
                elif 3.5 < user_data.distance <= 5.0:
                    if (bbox.xmin() + bbox.xmax()) / 2 < 0.5:
                        user_data.latest_msg = "0.2, 0.5, 0, 0, 0\n".encode("utf-8")
                    elif (bbox.xmin() + bbox.xmax()) / 2 > 0.7:
                        user_data.latest_msg = "0.2, -0.5, 0, 0, 0\n".encode("utf-8")
                    else:
                        user_data.latest_msg = "0.2, 0.0, 0, 0, 0\n".encode("utf-8")

                else:
                    # Stop wheels and start arm sequence only once
                    user_data.latest_msg = "0.0, 0.0, 0, 0, 0\n".encode()
                    user_data.mode = "pause"

        # If no ball detected, gradually reduce velocity
        else:
            user_data.latest_msg = "0.0, 0.0, 0, 0, 0\n".encode()


    string_to_print += (f"Target velocity: {user_data.latest_msg}")
    print(string_to_print)
    return Gst.PadProbeReturn.OK


if __name__ == "__main__":
    user_data = user_app_callback_class()  # Create an instance of the user app callback class
    app = GStreamerDetectionApp(app_callback, user_data)
    # ---------------- REALSENSE SETUP ----------------
    rs_pipeline = rs.pipeline()
    rs_config = rs.config()
    rs_config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    rs_config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    rs_align = rs.align(rs.stream.color)
    rs_pipeline.start(rs_config)
    #try:
    app.run()
    #finally:
    rs_pipeline.stop()
