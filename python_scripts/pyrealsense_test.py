from pathlib import Path
import pyrealsense2 as rs
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
import os

# Path to HEF file relative to your python_scripts folder
hef_rel_path = "../models/yolov8s_11_4_25.hef"

# Convert to absolute paths
HEF_PATH = os.path.abspath(os.path.join(os.getcwd(), hef_rel_path))

print("HEF path:", HEF_PATH)

#******HAILO PIPLINE*******************
Gst.init(None)

HAILO_PIPELINE = (
    "appsrc name=src is-live=true block=true format=time "
    "caps=video/x-raw,format=BGR,width=640,height=480,framerate=30/1 "
    "! videoconvert "
    "! hailonet hef-path=\"{}\" "
    "! hailofilter "
    "! appsink name=sink emit-signals=true sync=false"
).format(HEF_PATH)


def on_new_sample(sink):
    sample = sink.emit("pull-sample")
    buffer = sample.get_buffer()
    if buffer:
        app_callback(None, type("Info", (), {"get_buffer": lambda self=buffer: buffer})(), user_data)
    return Gst.FlowReturn.OK

#appsink.connect("new-sample", on_new_sample)
#*******************************************************


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

#**************REALSENSE RGB THREAD*******************          
def realsense_frame_loop(appsrc):
    rs_pipeline = rs.pipeline()
    rs_config = rs.config()
    rs_config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    rs_pipeline.start(rs_config)

    try:
        while True:
            frames = rs_pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            image = np.asanyarray(color_frame.get_data())

            buf = Gst.Buffer.new_allocate(None, image.nbytes, None)
            buf.fill(0, image.tobytes())
            buf.duration = Gst.util_uint64_scale(1, Gst.SECOND, 30)

            ret = appsrc.emit("push-buffer", buf)
            if ret != Gst.FlowReturn.OK:
                print("push-buffer failed:", ret)

            sleep(0.03)

    finally:
        rs_pipeline.stop()
#*************************************************************          


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


    # Parse the detections
    detection_count = 0
    for detection in detections:
        label = detection.get_label()
        bbox = detection.get_bbox()
        confidence = detection.get_confidence()

        if "ball" in label:
            # Get track ID
            user_data.vel = 0.4
            track_id = 0
            track = detection.get_objects_typed(hailo.HAILO_UNIQUE_ID)
            if len(track) == 1:
                track_id = track[0].get_id()
            string_to_print += (f"Detection: ID: {track_id} Label: {label} Confidence: {confidence:.2f}\n")
            string_to_print += (f"X Center: {(bbox.xmin() + bbox.xmax()) / 2 }, Y Center: {(bbox.ymin() + bbox.ymax()) / 2 }\n")
            if (bbox.xmin() + bbox.xmax()) / 2  < 0.3:
                user_data.latest_msg = "0.4, 1.0, 0, 0\n".encode('utf-8')
            elif (bbox.xmin() + bbox.xmax()) / 2 > 0.7:
                user_data.latest_msg = "0.4, -1.0, 0, 0\n".encode('utf-8')
            else:
                user_data.latest_msg = "0.4, 0.0, 0, 0\n".encode('utf-8')
            detection_count += 1

            break

        # If no ball detected, gradually reduce velocity
        else:
            user_data.vel = max(user_data.vel - 0.05, 0.0, 0, 0)
            user_data.latest_msg = f"{user_data.vel}, 0.0, 0, 0\n".encode('utf-8')

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

    #******HAILO PIPLINE*******************
    pipeline = Gst.parse_launch(HAILO_PIPELINE)
    appsrc = pipeline.get_by_name("src")
    appsink = pipeline.get_by_name("sink")
    appsink.connect("new-sample", on_new_sample)

    pipeline.set_state(Gst.State.PLAYING)

    rs_thread = threading.Thread(
        target=realsense_frame_loop,
        args=(appsrc,),
        daemon=True
    )
    rs_thread.start()

    print("RealSense → Hailo → Robot pipeline running")

    while True:
        sleep(1)
#************************************************
