"""
Test 02: Hailo model load + inference pipeline smoke test.

Pass criteria:
- HEF and labels JSON resolve correctly
- GStreamer/Hailo pipeline starts
- At least one inference callback packet is received
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import gi
import hailo
import numpy as np
import pyrealsense2 as rs


gi.require_version("Gst", "1.0")
from gi.repository import Gst


def resolve_existing_path(raw_value: str, here: Path, candidate_model_name: bool = False) -> Path:
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
    for c in candidates:
        key = str(c.resolve(strict=False))
        if key in seen:
            continue
        seen.add(key)
        if c.exists():
            return c

    tried = "\n".join(str(c) for c in candidates)
    raise FileNotFoundError(f"Could not resolve path '{raw_value}'. Tried:\n{tried}")


def load_labels(path: Path) -> list[str]:
    with open(path, "r", encoding="utf-8") as fp:
        cfg = json.load(fp)
    labels = cfg.get("labels")
    if not isinstance(labels, list) or not labels:
        raise ValueError("labels JSON must contain non-empty 'labels' list")
    return [str(x) for x in labels]


class HailoInferenceSmoke:
    def __init__(self, hef_path: str, labels_json: str):
        Gst.init(None)
        self.packet_count = 0
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
        out = [(d.get_label(), d.get_confidence()) for d in dets]

        self.packet_count += 1
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

    def push_frame(self, rgb_640: np.ndarray) -> None:
        data = rgb_640.tobytes()
        buf = Gst.Buffer.new_allocate(None, len(data), None)
        buf.fill(0, data)
        self.appsrc.emit("push-buffer", buf)


def main() -> int:
    here = Path(__file__).resolve().parent

    parser = argparse.ArgumentParser(description="Model + inference smoke test")
    parser.add_argument("--hef-path", default="models/3-12-caleb.hef")
    parser.add_argument("--labels-json", default="models/ball_bucket.json")
    parser.add_argument("--frames", type=int, default=150)
    parser.add_argument("--timeout-ms", type=int, default=3000)
    args = parser.parse_args()

    hef = resolve_existing_path(args.hef_path, here, candidate_model_name=True)
    labels = resolve_existing_path(args.labels_json, here, candidate_model_name=True)
    loaded_labels = load_labels(labels)

    print(f"[INFO] HEF: {hef}")
    print(f"[INFO] Labels: {labels} ({len(loaded_labels)} classes)")

    engine = HailoInferenceSmoke(str(hef), str(labels))
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    print("[INFO] Starting Hailo and camera pipeline...")
    engine.start()
    pipe.start(cfg)

    frame_count = 0
    try:
        start_t = time.monotonic()
        for i in range(args.frames):
            frames = pipe.wait_for_frames(timeout_ms=args.timeout_ms)
            color = frames.get_color_frame()
            if not color:
                continue
            color_np = np.asanyarray(color.get_data())
            rgb_640 = __import__("cv2").resize(color_np, (640, 640))
            engine.push_frame(rgb_640)
            frame_count += 1
            if (i + 1) % 30 == 0:
                print(f"[INFO] Pushed frames: {i + 1}/{args.frames}, inference packets: {engine.packet_count}")

        elapsed = max(1e-6, time.monotonic() - start_t)
        print(f"[INFO] Pushed {frame_count} frames in {elapsed:.2f}s")

    except RuntimeError as err:
        print(f"[FAIL] Runtime error while waiting frames/inference: {err}")
        return 1
    finally:
        pipe.stop()
        engine.stop()

    if frame_count == 0:
        print("[FAIL] No color frames were pushed into model")
        return 1
    if engine.packet_count == 0:
        print("[FAIL] No inference callbacks were received")
        return 1

    print(f"[PASS] Model smoke test succeeded with {engine.packet_count} inference packets")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())