"""
Bare-bones autonomous runner.
Keeps only essentials:
- RealSense frame grab
- Hailo detection
- state machine dispatch
- serial command send + feedback odom integration
"""

# current run command below:
# python /home/ball-e/ball-e/python_scripts/BARE_BONES/course_autonomous_depth.py --hef-path /home/ball-e/ball-e/models/3-12-caleb.hef --labels-json /home/ball-e/ball-e/models/ball_bucket.json --port /dev/ttyACM0 --no-display --save-run --allow-label-mismatch


import argparse
import json
import threading
from pathlib import Path
from time import monotonic, sleep

import cv2
import gi
import numpy as np
import pyrealsense2 as rs
import serial

import hailo
import autonomy_tuning as tune
from odom_autonomous_bridge import OdomAutonomousBridge
from state_machine import (
    handle_detect_ball,
    handle_detect_bucket,
    handle_drop,
    handle_fixed_back,
    handle_fixed_ball,
    handle_fixed_bucket,
    handle_pause,
    handle_pick,
    handle_swivel_large_right,
    handle_swivel_small_left,
)

gi.require_version("Gst", "1.0")
from gi.repository import Gst


class HailoInference:
    def __init__(self, hef_path, labels_json):
        Gst.init(None)
        self.queue = __import__("queue").Queue(maxsize=1)
        post_so = "/usr/lib/aarch64-linux-gnu/hailo/tappas/post_processes/libyolo_hailortpp_post.so"
        pipe = f"""
            appsrc name=source is-live=true block=true format=GST_FORMAT_TIME !
            videoconvert ! video/x-raw,format=RGB,width=640,height=640 !
            hailonet hef-path={hef_path} force-writable=true !
            hailofilter so-path={post_so} config-path={labels_json} qos=false !
            queue leaky=no max-size-buffers=3 !
            appsink name=sink emit-signals=true max-buffers=1 drop=true
        """
        self.pipeline = Gst.parse_launch(pipe)
        self.appsrc = self.pipeline.get_by_name("source")
        self.sink = self.pipeline.get_by_name("sink")
        self.sink.connect("new-sample", self._on_sample)
        caps = Gst.Caps.from_string("video/x-raw,format=RGB,width=640,height=640,framerate=30/1")
        self.appsrc.set_property("caps", caps)

    def _on_sample(self, sink):
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.ERROR
        buf = sample.get_buffer()
        roi = hailo.get_roi_from_buffer(buf)
        dets = roi.get_objects_typed(hailo.HAILO_DETECTION)
        out = [(d.get_label(), d.get_confidence(), d.get_bbox()) for d in dets]
        if self.queue.full():
            try:
                self.queue.get_nowait()
            except Exception:
                pass
        self.queue.put(out)
        return Gst.FlowReturn.OK

    def start(self):
        self.pipeline.set_state(Gst.State.PLAYING)

    def stop(self):
        self.pipeline.set_state(Gst.State.NULL)

    def push_frame(self, rgb_640):
        data = rgb_640.tobytes()
        buf = Gst.Buffer.new_allocate(None, len(data), None)
        buf.fill(0, data)
        self.appsrc.emit("push-buffer", buf)

    def get_latest(self):
        try:
            return self.queue.get_nowait()
        except __import__("queue").Empty:
            return []


class UserData:
    def __init__(self, serial_port):
        self.messenger = serial.Serial(port=serial_port, baudrate=115200)
        self.latest_msg = b"0.0, 0.0, 0, 0, 0\\n"

        start_mode = str(getattr(tune, "START_MODE", "detect")).strip().lower()
        self.mode = start_mode if start_mode in {"detect", "fixed_ball"} else "detect"
        self.arm_state = "idle"

        self.fixed_travel_counter = 0
        self.picker_counter = 0
        self.lap_counter = 0

        self.distance = 0.0
        self.distance_from_depth = False

        self.carrying_ball_color = None
        self.target_bucket_color = None

        self.ball_seen_streak = 0
        self.ball_close_streak = 0
        self.ball_lost_streak = 0
        self.bucket_seen_streak = 0
        self.bucket_close_streak = 0
        self.bucket_lost_streak = 0

        self.odom_enabled = bool(getattr(tune, "ODOM_ENABLE", True))
        self.odom = OdomAutonomousBridge(
            cam_offset_x_m=float(getattr(tune, "ODOM_CAM_OFFSET_X_M", -0.64)),
            cam_offset_y_m=float(getattr(tune, "ODOM_CAM_OFFSET_Y_M", 0.0)),
            cam_offset_z_m=float(getattr(tune, "ODOM_CAM_OFFSET_Z_M", 0.2)),
            kp_v=float(getattr(tune, "ODOM_KP_V", 0.5)),
            kp_w=float(getattr(tune, "ODOM_KP_W", 0.5)),
            max_v=float(getattr(tune, "ODOM_MAX_V", 0.30)),
            max_w=float(getattr(tune, "ODOM_MAX_W", 0.60)),
            distance_tolerance_m=float(getattr(tune, "ODOM_GOAL_TOLERANCE_M", 0.05)),
            forward_is_negative=bool(getattr(tune, "ODOM_FORWARD_IS_NEGATIVE", True)),
        )

        self._last_odom_t = monotonic()
        self._running = True

    @staticmethod
    def _parse_feedback(line):
        parts = [x.strip() for x in line.split(",")]
        if len(parts) < 2:
            return None
        try:
            return float(parts[0]), float(parts[1])
        except Exception:
            return None

    def send_loop(self):
        while self._running:
            if self.messenger.in_waiting > 0:
                try:
                    line = self.messenger.readline().decode("utf-8", "ignore").strip()
                except Exception:
                    line = ""
                if line:
                    parsed = self._parse_feedback(line)
                    if parsed is not None and self.odom_enabled:
                        lin, ang = parsed
                        now = monotonic()
                        dt = now - self._last_odom_t
                        self._last_odom_t = now
                        self.odom.update_pose_from_feedback(lin, ang, dt)
            try:
                self.messenger.write(self.latest_msg)
            except Exception:
                pass
            sleep(0.02)

    def stop(self):
        self._running = False


def _load_labels(labels_path):
    with open(labels_path, "r", encoding="utf-8") as fp:
        cfg = json.load(fp)
    labels = cfg.get("labels")
    if not isinstance(labels, list) or not labels:
        raise ValueError("labels JSON must contain a non-empty 'labels' list")
    return labels

def _resolve_existing_path(raw_value, here, candidate_model_name=False):
    """
    Resolve a possibly-relative path from common working locations.

    This keeps launch behavior compatible with commands where scripts and model
    assets live in sibling folders on the Pi.
    """
    raw = Path(raw_value)
    if raw.is_absolute():
        if raw.exists():
            return raw
        raise FileNotFoundError(f"Path not found: {raw}")

    candidates = [
        Path.cwd() / raw,
        here / raw,
        here.parent / raw,
        here.parent.parent / raw,
    ]
    if candidate_model_name:
        candidates.extend(
            [
                here / "models" / raw.name,
                here.parent / "models" / raw.name,
                here.parent.parent / "models" / raw.name,
            ]
        )

    seen = set()
    ordered = []
    for c in candidates:
        key = str(c.resolve(strict=False))
        if key in seen:
            continue
        seen.add(key)
        ordered.append(c)

    for c in ordered:
        if c.exists():
            return c

    tried = "\n".join(str(c) for c in ordered)
    raise FileNotFoundError(f"Could not resolve path '{raw_value}'. Tried:\n{tried}")


def main():
    here = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="Bare-bones autonomous + odom")
    parser.add_argument("--hef", "--hef-path", dest="hef", default="models/3-12-caleb.hef")
    parser.add_argument("--labels-json", dest="labels", default="models/ball_bucket.json")
    parser.add_argument("--labels", dest="labels_alias", default=None)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--no-display", action="store_true")
    # Accepted for command compatibility with the full runtime.
    parser.add_argument("--save-run", action="store_true")
    parser.add_argument("--allow-label-mismatch", action="store_true")
    args = parser.parse_args()

    labels_arg = args.labels_alias if args.labels_alias else args.labels
    hef_path = _resolve_existing_path(args.hef, here, candidate_model_name=True)
    labels_path = _resolve_existing_path(labels_arg, here, candidate_model_name=True)

    _load_labels(labels_path)

    engine = HailoInference(str(hef_path), str(labels_path))
    engine.start()

    rs_cfg = rs.config()
    rs_cfg.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    rs_cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    rs_pipe = rs.pipeline()
    rs_pipe.start(rs_cfg)
    align = rs.align(rs.stream.color)

    ud = UserData(args.port)
    sender = threading.Thread(target=ud.send_loop, daemon=True)
    sender.start()

    show = not args.no_display
    if show:
        cv2.namedWindow("BareBones", cv2.WINDOW_NORMAL)

    depth_w, depth_h = 640, 480
    model_h = 640

    try:
        while True:
            frames = rs_pipe.wait_for_frames()
            aligned = align.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            if not color_frame:
                continue

            color_np = np.asanyarray(color_frame.get_data())
            rgb_640 = cv2.resize(color_np, (640, 640))
            engine.push_frame(rgb_640)
            detections = engine.get_latest()

            if ud.mode == "pause":
                handle_pause(ud)
            elif ud.mode == "pick":
                handle_pick(ud)
            elif ud.mode == "drop":
                handle_drop(ud)
            elif ud.mode == "fixed_ball":
                handle_fixed_ball(ud)
            elif ud.mode == "fixed_bucket":
                handle_fixed_bucket(ud)
            elif ud.mode == "fixed_back":
                handle_fixed_back(ud)
            elif ud.mode == "swivel_small_left":
                handle_swivel_small_left(ud)
            elif ud.mode == "swivel_large_right":
                handle_swivel_large_right(ud)
            elif ud.mode == "detect":
                handle_detect_ball(ud, detections, depth_frame, depth_w, depth_h, model_h)
            elif ud.mode == "detect_bucket":
                handle_detect_bucket(ud, detections, depth_frame, depth_w, depth_h, model_h)
            else:
                handle_pause(ud)

            if show:
                frame_bgr = cv2.cvtColor(color_np, cv2.COLOR_RGB2BGR)
                for label, conf, bbox in detections:
                    if conf < 0.25:
                        continue
                    x1 = int(max(0, min(depth_w - 1, bbox.xmin() * depth_w)))
                    y1 = int(max(0, min(depth_h - 1, bbox.ymin() * depth_h)))
                    x2 = int(max(0, min(depth_w - 1, bbox.xmax() * depth_w)))
                    y2 = int(max(0, min(depth_h - 1, bbox.ymax() * depth_h)))
                    cv2.rectangle(frame_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame_bgr, f"{label}:{conf:.2f}", (x1, max(20, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                cv2.putText(frame_bgr, f"mode={ud.mode} arm={ud.arm_state}", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.putText(frame_bgr, f"dist={ud.distance:.2f} src={'depth' if ud.distance_from_depth else 'geom'}", (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
                cv2.putText(frame_bgr, f"odom={ud.odom.pose.x:.2f},{ud.odom.pose.y:.2f},{ud.odom.pose.theta:.2f}", (10, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
                cv2.putText(frame_bgr, ud.latest_msg.decode("utf-8", "ignore").strip(), (10, 96), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

                cv2.imshow("BareBones", frame_bgr)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break
    finally:
        ud.stop()
        ud.latest_msg = b"0.0, 0.0, 0, 0, 0\\n"
        try:
            ud.messenger.write(ud.latest_msg)
            sleep(0.1)
            ud.messenger.close()
        except Exception:
            pass
        engine.stop()
        rs_pipe.stop()
        if show:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()