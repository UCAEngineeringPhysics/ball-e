import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import os
import argparse
import numpy as np
import cv2
import time
import pyrealsense2 as rs
import threading
import queue
import hailo
import serial
from time import sleep
from pathlib import Path

# ---------------------------------------------------------
# 1. HAILO INFERENCE & COMMUNICATION CLASS
# ---------------------------------------------------------
class HailoRobotEngine:
    def __init__(self, hef_path, labels_json):
        Gst.init(None)
        self.running = False
        self.detection_queue = queue.Queue(maxsize=1)
        
        # --- Serial Setup (from course_nav_talker_arm) ---
        try:
            self.messenger = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=0.1)
            print(f"Messenger initiated at: {self.messenger.name}")
        except Exception as e:
            print(f"Serial Error: {e}. Running in simulation mode (no Pico).")
            self.messenger = None

        # --- Shared Robot State ---
        self.latest_msg = "0.0, 0.0, 0, 0, 10\n".encode('utf-8')
        self.vel = 0.0
        self.mode = "fixed_ball"  # Starting mode
        self.arm_state = "idle"
        self.fixed_travel_counter = 0
        self.picker_counter = 0
        self.lap_counter = 0

        # Start Pico background thread
        if self.messenger:
            self.pico_thread = threading.Thread(target=self._send_msg_loop, daemon=True)
            self.pico_thread.start()

        # --- GStreamer Pipeline ---
        post_process_so = "/usr/lib/aarch64-linux-gnu/hailo/tappas/post_processes/libyolo_hailortpp_post.so"
        pipeline_str = f"""
            appsrc name=source is-live=true block=true format=GST_FORMAT_TIME ! \
            videoconvert ! video/x-raw,format=RGB,width=640,height=640 ! \
            hailonet hef-path={hef_path} force-writable=true ! \
            hailofilter so-path={post_process_so} config-path={labels_json} qos=false ! \
            queue leaky=no max-size-buffers=3 ! \
            appsink name=sink emit-signals=true max-buffers=1 drop=true
        """
        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsrc = self.pipeline.get_by_name("source")
        self.appsink = self.pipeline.get_by_name("sink")
        self.appsink.connect("new-sample", self._on_new_sample)
        
        caps = Gst.Caps.from_string("video/x-raw,format=RGB,width=640,height=640,framerate=30/1")
        self.appsrc.set_property("caps", caps)

    def _send_msg_loop(self):
        """Background thread to keep the Pico updated."""
        while True:
            if self.messenger and self.messenger.is_open:
                if self.messenger.in_waiting > 0:
                    self.messenger.readline() # Clear input buffer
                self.messenger.write(self.latest_msg)
            sleep(0.02)

    def _on_new_sample(self, sink):
        sample = sink.emit("pull-sample")
        buffer = sample.get_buffer()
        roi = hailo.get_roi_from_buffer(buffer)
        detections = roi.get_objects_typed(hailo.HAILO_DETECTION)
        
        results = []
        for det in detections:
            results.append((det.get_label(), det.get_confidence(), det.get_bbox()))
        
        if self.detection_queue.full():
            try: self.detection_queue.get_nowait()
            except: pass
        self.detection_queue.put(results)
        return Gst.FlowReturn.OK

    def infer_frame(self, numpy_frame):
        resized = cv2.resize(numpy_frame, (640, 640))
        data = resized.tobytes()
        buf = Gst.Buffer.new_allocate(None, len(data), None)
        buf.fill(0, data)
        self.appsrc.emit("push-buffer", buf)
    
    def get_latest_result(self):
        try: return self.detection_queue.get_nowait()
        except queue.Empty: return []

    def start(self):
        self.pipeline.set_state(Gst.State.PLAYING)
        self.running = True

    def stop(self):
        self.running = False
        self.pipeline.set_state(Gst.State.NULL)

# ---------------------------------------------------------
# 2. MAIN NAVIGATION & LOGIC
# ---------------------------------------------------------
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--hef-path", default="../models/yolov8s_h8l.hef")
    parser.add_argument("--labels-json", default="../models/ball_bucket.json")
    args = parser.parse_args()

    robot = HailoRobotEngine(args.hef_path, args.labels_json)
    robot.start()

    # RealSense Setup
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    align = rs.align(rs.stream.color)
    pipeline.start(config)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            if not color_frame or not depth_frame: continue

            img_color = np.asanyarray(color_frame.get_data())
            img_display = cv2.cvtColor(img_color, cv2.COLOR_RGB2BGR)

            # --- 1. STATE MACHINE (Logic from nav_talker) ---
            # Always run inference and get results if we want to see them
            robot.infer_frame(img_color)
            detections = robot.get_latest_result()

            # DRAWING SECTION (Outside the state machine so it always renders)
            for label, conf, bbox in detections:
                # Scale coordinates to your display image size (640x480)
                x1 = int(bbox.xmin() * 640)
                y1 = int(bbox.ymin() * 480)
                x2 = int(bbox.xmax() * 640)
                y2 = int(bbox.ymax() * 480)
                
                # Draw Box
                cv2.rectangle(img_display, (x1, y1), (x2, y2), (255, 0, 0), 2)
                
                # Draw Label, Confidence, and Distance (if calculated)
                display_text = f"{label}: {conf:.2f}"
                cv2.putText(img_display, display_text, (x1, y1 - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            # FIXED MOVEMENTS (Blind timers)
            if robot.mode == "fixed_ball":
                robot.latest_msg = "0.2, 0.0, 0, 0, 10\n".encode()
                robot.fixed_travel_counter += 1
                if robot.fixed_travel_counter >= 460:
                    robot.mode = "detect"
                    robot.fixed_travel_counter = 0

            elif robot.mode == "pick":
                # Handle Arm sequence (simplified for space, same logic as your source)
                robot.latest_msg = "0.0, 0.0, 0, 0, 0\n".encode()
                if robot.arm_state == "lower":
                    robot.latest_msg = "0.0, 0.0, 3000, 0, 0\n".encode()
                    robot.picker_counter += 1
                    if robot.picker_counter >= 210: robot.arm_state = "close"; robot.picker_counter = 0
                # ... (Add your close/raise logic here similarly)

            # DETECTION MOVEMENTS
            elif robot.mode == "detect":
                robot.infer_frame(img_color)
                detections = robot.get_latest_result()
                target_found = False
       
                if conf > 0.5 and "ball" in label:
                    target_found = True
                    # Get distance
                    cx = int((bbox.xmin() + bbox.xmax()) / 2 * 640)
                    cy = int((bbox.ymin() + bbox.ymax()) / 2 * 480)
                    dist = depth_frame.get_distance(cx, cy)
                    if dist == 0: # Backup math if depth fails
                        dist = (3386.0 * 0.1524) / ((bbox.ymax() - bbox.ymin()) * 480)

                    # if Z > 2.4:
                    if dist >= 80.0:
                        if (bbox.xmin() + bbox.xmax()) / 2 < 0.4:
                            robot.latest_msg = "0.2, 0.5, 0, 0, 0\n".encode('utf-8')
                        elif (bbox.xmin() + bbox.xmax()) / 2 > 0.7:
                            robot.latest_msg = "0.2, -0.5, 0, 0, 0\n".encode('utf-8')
                        else:
                            robot.latest_msg = "0.2, 0.0, 0, 0, 0\n".encode('utf-8')
                    # elif Z <= 2.4 and Z > 1.0:
                    elif 5.0 < dist <= 80.0:
                        if (bbox.xmin() + bbox.xmax()) / 2 < 0.4:
                            robot.latest_msg = "0.1, 0.5, 0, 0, 0\n".encode("utf-8")
                        elif (bbox.xmin() + bbox.xmax()) / 2 > 0.7:
                            robot.latest_msg = "0.1, -0.5, 0, 0, 0\n".encode("utf-8")
                        else:
                            robot.latest_msg = "0.1, 0.0, 0, 0, 0\n".encode("utf-8")
                    else:
                        # Stop wheels and start arm sequence only once
                        robot.latest_msg = "0.0, 0.0, 0, 0, 0\n".encode()
                        robot.arm_state = "lower"
                        robot.mode = "pick"

                    # Draw on screen
                    cv2.rectangle(img_display, (int(bbox.xmin()*640), int(bbox.ymin()*480)), 
                                    (int(bbox.xmax()*640), int(bbox.ymax()*480)), (255, 0, 0), 2)
                
                if not target_found:
                    robot.latest_msg = "0.0, 0.0, 0, 0, 0\n".encode()

            # --- 2. GUI & STOP ---
            cv2.putText(img_display, f"Mode: {robot.mode}", (10, 30), 1, 1, (0, 255, 0), 2)
            cv2.imshow("Robot View", img_display)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        robot.stop()
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()