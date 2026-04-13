#!/usr/bin/env python3
# realsense_hailo.py
import os
import sys
import time
import threading
from pathlib import Path
import numpy as np
import pyrealsense2 as rs
import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib
import serial
from time import sleep

# ----- CONFIG -----
HEF_REL = "../models/3-12-caleb.hef"
HEF_PATH = os.path.abspath(os.path.join(os.getcwd(), HEF_REL))

WIDTH = 640
HEIGHT = 480
FPS = 30

# Optional: set to True if you want to fallback to CPU YOLO when hailonet fails.
USE_CPU_FALLBACK = False
# ------------------

Gst.init(None)

HAILO_PIPELINE = (
    "appsrc name=src is-live=true block=true format=time "
    f"caps=video/x-raw,format=BGR,width={WIDTH},height={HEIGHT},framerate={FPS}/1 "
    "! videoconvert "
    f"! hailonet hef-path=\"{HEF_PATH}\" batch-size=1 "
    "! hailofilter "
    "! appsink name=sink emit-signals=true sync=false"
)

def check_prereqs():
    # check HEF exists
    if not os.path.isfile(HEF_PATH):
        print(f"ERROR: HEF not found at: {HEF_PATH}")
        print("Use absolute path or place the HEF at that location.")
        sys.exit(1)

    # check hailo device node presence
    devs = [p for p in Path("/dev").glob("hailo*")]
    if not devs:
        print("WARNING: no /dev/hailo* nodes found. Hailo kernel driver may not be loaded.")
        print("If you expect an AI Hat, ensure drivers installed or use USB mode / fallback.")
    else:
        print("Found Hailo device nodes:", ", ".join(str(p) for p in devs))

    # Advice: must source Hailo env before running
    if "HAILO_ENV_FILE" not in os.environ and "HAILO_SDK_ROOT" not in os.environ:
        print("NOTE: make sure you sourced the Hailo environment (e.g. source setup_env.sh) in this shell.")
        print("If you didn't, do: cd ~/hailo-rpi5-examples && source setup_env.sh && cd ~/your/script/dir")

# ----- Serial / robot class -----
class RobotMessenger:
    def __init__(self, port="/dev/ttyACM0", baud=115200):
        self.m = serial.Serial(port=port, baudrate=baud, timeout=0.1)
        print(f"Messenger initiated at: {self.m.name}")
        self.latest_msg = "0.0,0.0,0,0\n".encode()
        self.vel = 0.0
        self._thr = threading.Thread(target=self._loop, daemon=True)
        self._thr.start()

    def _loop(self):
        while True:
            try:
                if self.m.inWaiting() > 0:
                    _ = self.m.readline().strip()
            except Exception:
                pass
            try:
                self.m.write(self.latest_msg)
            except Exception:
                pass
            sleep(0.02)

# ----- RealSense producer -----
def realsense_producer(appsrc):
    rs_pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
    profile = rs_pipeline.start(cfg)
    try:
        while True:
            frames = rs_pipeline.wait_for_frames()
            color = frames.get_color_frame()
            if not color:
                continue
            img = np.asanyarray(color.get_data())

            buf = Gst.Buffer.new_allocate(None, img.nbytes, None)
            buf.fill(0, img.tobytes())
            buf.duration = Gst.util_uint64_scale(1, Gst.SECOND, FPS)
            ret = appsrc.emit("push-buffer", buf)
            if ret != Gst.FlowReturn.OK:
                print("Warning: push-buffer returned", ret)
            sleep(1.0 / FPS)
    finally:
        rs_pipeline.stop()

# ----- appsink handler -----
def on_new_sample(sink, user_data):
    sample = sink.emit("pull-sample")
    if not sample:
        return Gst.FlowReturn.OK
    buffer = sample.get_buffer()
    # This buffer is produced by hailonet; to parse detections:
    try:
        import hailo
        roi = hailo.get_roi_from_buffer(buffer)
        dets = roi.get_objects_typed(hailo.HAILO_DETECTION)
        # simple print
        for d in dets:
            print(f"Label={d.get_label()} conf={d.get_confidence():.3f}")
            # optionally add robot control using user_data.latest_msg here
    except Exception as e:
        # If hailonet isn't working, buffer parsing may fail
        print("Processed buffer received (could not parse detections):", e)
    return Gst.FlowReturn.OK

# ----- main -----
def main():
    check_prereqs()

    # Try to create and run the hailo GStreamer pipeline
    print("Launching GStreamer pipeline (appsrc -> hailonet -> appsink)...")
    try:
        pipeline = Gst.parse_launch(HAILO_PIPELINE)
    except Exception as e:
        print("Gst.parse_launch failed:", e)
        print("If this mentions hailonet element not found, ensure you sourced Hailo SDK env (setup_env.sh).")
        if USE_CPU_FALLBACK:
            print("Falling back to CPU path (enable USE_CPU_FALLBACK=True)")
        return

    appsrc = pipeline.get_by_name("src")
    appsink = pipeline.get_by_name("sink")
    appsink.connect("new-sample", on_new_sample, None)

    # Start pipeline
    try:
        pipeline.set_state(Gst.State.PLAYING)
    except Exception as e:
        print("Failed to set pipeline PLAYING:", e)
        print("Common causes: hailo runtime/service holds device or incompatible kernel/drivers.")
        print("If hailort service is running, stop it: sudo systemctl stop hailort")
        if USE_CPU_FALLBACK:
            print("Falling back to CPU path (enable USE_CPU_FALLBACK=True)")
        return

    # Start messenger and realsense producer
    user = RobotMessenger()
    rs_thread = threading.Thread(target=realsense_producer, args=(appsrc,), daemon=True)
    rs_thread.start()

    print("Pipeline running. Press Ctrl+C to exit.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        pipeline.set_state(Gst.State.NULL)
        print("Stopped.")

if __name__ == "__main__":
    main()
