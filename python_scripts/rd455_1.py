import argparse
import json
import os
import sys
import threading
import time
import numpy as np
import cv2
import serial
import pyrealsense2 as rs

# --- STANDARD IMPORTS (No hacks) ---
from hailo_platform import (HEF, VDevice, HailoStreamInterface, ConfigureParams, 
                            InputVStreamParams, OutputVStreamParams, InferVStreams, FormatType)

# -----------------------------------------------------------------------------
# 1. ARGUMENT PARSING
# -----------------------------------------------------------------------------
def parse_arguments():
    parser = argparse.ArgumentParser(description="Ball-E Robot Logic with RealSense & Hailo")
    parser.add_argument("--hef-path", type=str, required=True, help="Path to .hef model file")
    parser.add_argument("--labels-json", type=str, required=True, help="Path to label map JSON")
    parser.add_argument("--input", type=str, default="rpi", help="Input source (ignored)")
    return parser.parse_args()

# -----------------------------------------------------------------------------
# 2. HELPER FUNCTIONS
# -----------------------------------------------------------------------------
def load_labels(json_path):
    with open(json_path, 'r') as f:
        data = json.load(f)
    
    label_map = {}
    labels_source = data
    if isinstance(data, dict) and "labels" in data:
        labels_source = data["labels"]

    if isinstance(labels_source, list):
        for idx, name in enumerate(labels_source):
            label_map[name] = idx
    elif isinstance(labels_source, dict):
        for k, v in labels_source.items():
            try:
                label_map[v] = int(k)
            except ValueError:
                pass 
    return label_map

# -----------------------------------------------------------------------------
# 3. ROBOT STATE MACHINE
# -----------------------------------------------------------------------------
class RobotController:
    def __init__(self, labels_map):
        try:
            self.messenger = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=0.1)
            print(f"Connected to Pico at {self.messenger.name}")
        except:
            print("WARNING: Pico not connected (Simulation Mode)")
            self.messenger = None

        self.latest_msg = "0.0, 0.0, 0, 0, 0\n".encode('utf-8')
        self.mode = "fixed_ball"
        self.fixed_travel_counter = 0
        self.picker_counter = 0
        self.vel = 0.0
        self.distance = 0.0
        
        self.ball_ids = []
        self.bucket_ids = []
        
        print("Mapping IDs:")
        for name, class_id in labels_map.items():
            if "ball" in name.lower():
                self.ball_ids.append(class_id)
                print(f"  - Found Ball: '{name}' (ID {class_id})")
            elif "bucket" in name.lower():
                self.bucket_ids.append(class_id)
                print(f"  - Found Bucket: '{name}' (ID {class_id})")

        if self.messenger:
            self.thread = threading.Thread(target=self.send_msg_loop, daemon=True)
            self.thread.start()

    def send_msg_loop(self):
        while True:
            if self.messenger:
                self.messenger.write(self.latest_msg)
            time.sleep(0.02)

    def update_logic(self, detections, depth_frame, frame_width):
        msg_str = "0.0, 0.0, 0, 0, 0"

        # --- STATE MACHINE LOGIC ---
        if self.mode == "pick":
            if 0 <= self.picker_counter <= 210:
                msg_str = "0.0, 0.0, 3000, 0, 0"
                self.picker_counter += 1
            elif 210 < self.picker_counter <= 295:
                msg_str = "0.0, 0.0, 0, 3000, 0"
                self.picker_counter += 1
            elif 295 < self.picker_counter <= 470:
                msg_str = "0.0, 0.0, -3000, 0, 0"
                self.picker_counter += 1
            else:
                self.mode = "fixed_back"
                self.picker_counter = 0

        elif self.mode == "drop":
            if 0 <= self.picker_counter <= 50:
                msg_str = "0.0, 0.0, 3000, 0, 0"
                self.picker_counter += 1
            elif 50 < self.picker_counter <= 90:  
                msg_str = "0.0, 0.0, 0, -3000, 0"
                self.picker_counter += 1
            else:    
                msg_str = "0.0, 0.0, 0, 0, 10"
                self.mode = "swivel_large_right"
                self.picker_counter = 0
                self.fixed_travel_counter = 0

        elif self.mode == "fixed_ball":
            if self.fixed_travel_counter < 500:
                msg_str = "0.2, 0.0, 0, 0, 10"
                self.fixed_travel_counter += 1
            else:
                self.mode = "detect"
                self.fixed_travel_counter = 0
                msg_str = "0.0, 0.0, 0, 0, 0"
        
        elif self.mode == "fixed_bucket":
            if self.fixed_travel_counter < 580:
                msg_str = "0.2, 0.0, 0, 0, 0"
                self.fixed_travel_counter += 1
            else:
                self.mode = "detect_bucket"
                self.fixed_travel_counter = 0
                msg_str = "0.0, 0.0, 0, 0, 0"
        
        elif self.mode == "detect":
            target_box = None
            for det in detections:
                if int(det[5]) in self.ball_ids: 
                    target_box = det
                    break
            
            if target_box is None:
                self.vel = max(self.vel - 0.05, 0.0)
                msg_str = f"{self.vel:.2f}, 0.0, 0, 0, 0"
            else:
                x1, y1, x2, y2 = target_box[:4]
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)
                
                dist_raw = depth_frame.get_distance(center_x, center_y)
                if dist_raw > 0: self.distance = dist_raw
                
                self.vel = 0.4
                norm_center = center_x / frame_width
                
                if self.distance >= 2.4:
                    if norm_center < 0.4: msg_str = "0.4, 0.5, 0, 0, 0"
                    elif norm_center > 0.7: msg_str = "0.4, -0.5, 0, 0, 0"
                    else: msg_str = "0.4, 0.0, 0, 0, 0"
                elif 1.258 < self.distance <= 2.4:
                    if norm_center < 0.4: msg_str = "0.2, 0.5, 0, 0, 0"
                    elif norm_center > 0.7: msg_str = "0.2, -0.5, 0, 0, 0"
                    else: msg_str = "0.2, 0.0, 0, 0, 0"
                else:
                    msg_str = "0.0, 0.0, 0, 0, 0"
                    self.mode = "pick"

        elif self.mode == "detect_bucket":
            target_box = None
            for det in detections:
                if int(det[5]) in self.bucket_ids: 
                    target_box = det
                    break
            
            if target_box is None:
                self.vel = max(self.vel - 0.05, 0.0)
                msg_str = f"{self.vel:.2f}, 0.0, 0, 0, 0"
            else:
                x1, y1, x2, y2 = target_box[:4]
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)
                dist_raw = depth_frame.get_distance(center_x, center_y)
                if dist_raw > 0: self.distance = dist_raw
                
                norm_center = center_x / frame_width
                if self.distance >= 3.5:
                    if norm_center < 0.4: msg_str = "0.4, 0.5, 0, 0, 0"
                    elif norm_center > 0.7: msg_str = "0.4, -0.5, 0, 0, 0"
                    else: msg_str = "0.4, 0.0, 0, 0, 0"
                elif 1.35 < self.distance <= 3.0:
                    if norm_center < 0.5: msg_str = "0.2, 0.5, 0, 0, 0"
                    elif norm_center > 0.7: msg_str = "0.2, -0.5, 0, 0, 0"
                    else: msg_str = "0.2, 0.0, 0, 0, 0"
                else:
                    msg_str = "0.0, 0.0, 0, 0, 0"
                    self.mode = "fixed_drop"

        elif self.mode == "fixed_back":
             if self.fixed_travel_counter < 80:
                msg_str = "-0.1, 0.0, 0, 0, 0"
                self.fixed_travel_counter += 1
             else:
                self.mode = "swivel_small_left"
                self.fixed_travel_counter = 0
                msg_str = "0.0, 0.0, 0, 0, 0"

        elif self.mode == "swivel_small_left":
             if self.fixed_travel_counter < 130:
                 msg_str = "0.0, 0.3, 0, 0, 0"
                 self.fixed_travel_counter += 1
             else:
                 self.mode = "fixed_bucket"
                 self.fixed_travel_counter = 0
                 msg_str = "0.0, 0.0, 0, 0, 0"
                 
        elif self.mode == "swivel_large_right":
            if self.fixed_travel_counter < 370:
                msg_str = "0.0, -0.3, 0, 0, 0"
                self.fixed_travel_counter += 1
            else:
                self.mode = "fixed_ball"
                self.fixed_travel_counter = 0
                msg_str = "0.0, 0.0, 0, 0, 0"

        elif self.mode == "fixed_drop":
            if self.fixed_travel_counter < 127:
                msg_str = "0.2, 0.0, 0, 0, 0"
                self.fixed_travel_counter += 1
            else:
                self.mode = "drop"
                self.fixed_travel_counter = 0
                msg_str = "0.0, 0.0, 0, 0, 0"

        self.latest_msg = (msg_str + "\n").encode('utf-8')
        print(f"Mode: {self.mode} | Dist: {self.distance:.2f}m | Cmd: {msg_str}")

# -----------------------------------------------------------------------------
# 4. MAIN EXECUTION
# -----------------------------------------------------------------------------
def get_hailo_inference(hef_path):
    hef = HEF(hef_path)
    target = VDevice()
    configure_params = ConfigureParams.create_from_hef(hef, interface=HailoStreamInterface.PCIe)
    network_groups = target.configure(hef, configure_params)
    network_group = network_groups[0]
    
    # [FIX] Force explicit Format Types to resolve mismatch crash (Status 8)
    # We ask the driver to convert Input -> UINT8 and Output -> FLOAT32
    input_vstreams_params = InputVStreamParams.make(
        network_group, 
        format_type=FormatType.UINT8
    )
    output_vstreams_params = OutputVStreamParams.make(
        network_group, 
        format_type=FormatType.FLOAT32
    )
    
    # Use High-Level Wrapper with Explicit Params
    pipeline = InferVStreams(network_group, input_vstreams_params, output_vstreams_params)
    
    return pipeline, hef

def run_robot():
    args = parse_arguments()
    
    if not os.path.exists(args.hef_path):
        print(f"Error: HEF file not found at {args.hef_path}")
        return
    if not os.path.exists(args.labels_json):
        print(f"Error: Labels file not found at {args.labels_json}")
        return

    labels_map = load_labels(args.labels_json)
    bot = RobotController(labels_map)

    # RealSense Setup
    print("Initializing RealSense (RSUSB)...")
    ctx = rs.context()
    if len(ctx.devices) == 0:
        print("CRITICAL ERROR: No RealSense devices detected! Unplug/Replug camera.")
        return

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    profile = pipeline.start(config)
    align = rs.align(rs.stream.color)

    # Hailo Setup
    print(f"Loading HEF: {args.hef_path}")
    infer_pipeline, hef = get_hailo_inference(args.hef_path)
    
    # Get Names
    input_name = hef.get_input_vstream_infos()[0].name
    output_name = hef.get_output_vstream_infos()[0].name

    with infer_pipeline:
        print("System Ready. Press 'q' to quit.")

        try:
            while True:
                frames = pipeline.wait_for_frames()
                aligned_frames = align.process(frames)
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()
                if not color_frame: continue

                img_rgb = np.asanyarray(color_frame.get_data())
                img_resized = cv2.resize(img_rgb, (640, 640))
                input_batch = np.expand_dims(img_resized, axis=0)

                # Send Data
                infer_pipeline.send({input_name: input_batch})
                
                # Receive Result
                infer_dict = infer_pipeline.recv()
                infer_result = infer_dict[output_name]

                detections = []
                if infer_result.shape[0] > 0:
                    for det in infer_result[0]:
                        # Format is typically [ymin, xmin, ymax, xmax, score, class_id]
                        if len(det) >= 6:
                            ymin, xmin, ymax, xmax, score, class_id = det[:6]
                            if score < 0.5: continue
                            detections.append([int(xmin*640), int(ymin*480), int(xmax*640), int(ymax*480), score, class_id])

                bot.update_logic(detections, depth_frame, 640)

                img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
                for x1, y1, x2, y2, s, c in detections:
                    cv2.rectangle(img_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(img_bgr, f"ID {int(c)}", (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                
                cv2.imshow('Robot Vision', img_bgr)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        finally:
            pipeline.stop()
            cv2.destroyAllWindows()
            
if __name__ == "__main__":
    run_robot()