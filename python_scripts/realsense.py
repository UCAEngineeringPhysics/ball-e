import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
import pyrealsense2 as rs
import numpy as np
import time

# ----------------------------
# Init GStreamer
# ----------------------------
Gst.init(None)

PIPELINE = (
    "appsrc name=src is-live=true block=true format=time "
    "caps=video/x-raw,format=BGR,width=640,height=480,framerate=30/1 "
    "! videoconvert "
    "! videoconvert "
    "! appsink name=sink emit-signals=true sync=false"
)

pipeline = Gst.parse_launch(PIPELINE)
appsrc = pipeline.get_by_name("src")
appsink = pipeline.get_by_name("sink")

pipeline.set_state(Gst.State.PLAYING)

# ----------------------------
# Init RealSense
# ----------------------------
rs_pipeline = rs.pipeline()
rs_config = rs.config()
rs_config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

rs_pipeline.start(rs_config)

print("RealSense + GStreamer appsrc test started")

# ----------------------------
# Pull from appsink
# ----------------------------
def on_new_sample(sink):
    sample = sink.emit("pull-sample")
    buffer = sample.get_buffer()
    print(f"Frame received via GStreamer: {buffer.get_size()} bytes")
    return Gst.FlowReturn.OK

appsink.connect("new-sample", on_new_sample)

# ----------------------------
# Main loop
# ----------------------------
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

        time.sleep(0.03)

except KeyboardInterrupt:
    pass

finally:
    pipeline.set_state(Gst.State.NULL)
    rs_pipeline.stop()
    print("Stopped")
