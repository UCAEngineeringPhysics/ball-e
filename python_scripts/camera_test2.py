#uses multiple pipelines for detection and depth. 


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
        
        caps = Gst.Caps.from_string("video/x-raw,format=RGB,width=640,height=640,framerate=30/1")
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
            try: self.detection_queue.get_nowait()
            except: pass
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
    parser.add_argument("--hef-path", default="../models/yolov8s_h8l.hef")
    parser.add_argument("--labels-json", default="../models/ball_bucket.json")
    # Added dummy --input argument so your existing command string works
    parser.add_argument("--input", default=None, help="Ignored: RealSense is hardcoded")
    args = parser.parse_args()

    # A. Setup Hailo
    engine = HailoRemoteInference(args.hef_path, args.labels_json)
    engine.start()

    # B. Setup RealSense
    print("Starting RealSense...")
    pipeline = rs.pipeline()
    config = rs.config()
    # Using 640x480 standard resolution
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    align = rs.align(rs.stream.color)
    pipeline.start(config)

    print("System Running. Press 'q' to quit.")

    try:
        while True:
            # 1. Capture
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            if not color_frame or not depth_frame: continue

            img_color = np.asanyarray(color_frame.get_data())
            # For display only (OpenCV needs BGR)
            img_display = cv2.cvtColor(img_color, cv2.COLOR_RGB2BGR)

            # 2. Infer
            engine.infer_frame(img_color)
            detections = engine.get_latest_result()

            # 3. Process
            for label, conf, bbox in detections:
                if conf < 0.5: continue

                # Map bbox (0.0-1.0) to 640x480
                x1 = int(bbox.xmin() * 640)
                y1 = int(bbox.ymin() * 480)
                x2 = int(bbox.xmax() * 640)
                y2 = int(bbox.ymax() * 480)
                
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2

                # 4. Get Distance (Hardware + Calc)
                hw_dist = depth_frame.get_distance(cx, cy)
                
                # Backup Math
                h_pixels = y2 - y1
                calc_dist = (3386.0 * 0.381) / h_pixels if h_pixels > 0 else 0

                final_dist = hw_dist if hw_dist > 0.1 else calc_dist
                src = "HW" if hw_dist > 0.1 else "Calc"

                # Draw
                cv2.rectangle(img_display, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(img_display, f"{label}: {final_dist:.2f}m ({src})", 
                           (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # 5. Display
            cv2.imshow("Robot Vision", img_display)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        engine.stop()
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
