import os
from pathlib import Path
import gi


gi.require_version("Gst", "1.0")
from gi.repository import Gst
import hailo
from hailo_apps.hailo_app_python.core.gstreamer.gstreamer_app import app_callback_class
from hailo_apps.hailo_app_python.apps.detection_simple.detection_pipeline_simple import (
    GStreamerDetectionApp,
)

from serial import Serial
import threading
from time import time, sleep
from math import atan2, cos, sin, hypot

# User-defined class to be used in the callback function: Inheritance from the app_callback_class
class user_app_callback_class(app_callback_class):
    def __init__(self):
        super().__init__()
        self.pico_msngr = Serial(port="/dev/ttyACM0", baudrate=115200, timeout=0.01)
        print(f"Messenger initiated at: {self.pico_msngr.name}\n")
        # Variables
        self.is_goal_reached = True
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.targ_lin_vel = 0.0
        self.targ_ang_vel = 0.0
        self.motion_data = {key: 0.0 for key in ["meas_lin_vel", "fuse_ang_vel"]}
        self.last_ts = time()  # time stamp in s
        self.pico_thread = threading.Thread(target=self.process_pico_msgs, daemon=True)
        self.pico_thread.start()

    def process_pico_msgs(self):
        last_ts = time()
        while self.pico_msngr is not None:
            # Transmit velocity commands to Pico
            curr_ts = time()
            dt = curr_ts - last_ts
            # if (curr_ts - last_ts) >= 0.04:  # TX freq: 25 Hz
            if dt >= 0.04:  # TX freq: 25 Hz
                if not self.is_goal_reached:
                    self.compute_target_velocity()
                else:
                    pass  # TODO: hard coded vels
                msg_to_pico = f"{self.targ_lin_vel:.3f},{self.targ_ang_vel:.3f}\n"
                # Encode string to bytes and send
                self.pico_msngr.write(msg_to_pico.encode("utf-8"))
                last_ts = curr_ts
                # Update odometry
                self.x += self.motion_data["meas_lin_vel"] * cos(self.theta) * dt
                self.y += self.motion_data["meas_lin_vel"] * sin(self.theta) * dt
                self.theta += self.motion_data["fuse_ang_vel"] * dt
                self.theta = atan2(
                    sin(self.theta), cos(self.theta)
                )  # restrict theta between -pi and pi

            # Receive motion data from Pico
            if self.pico_msngr.inWaiting() > 0:
                msg_from_pico = (
                    self.pico_msngr.readline().decode("utf-8", "ignore").strip()
                )
                if msg_from_pico:
                    data_strings = msg_from_pico.split(",")
                    try:
                        self.motion_data.update(
                            zip(
                                self.motion_data.keys(),
                                map(
                                    float, data_strings
                                ),  # convert all str in list to float
                            )
                        )
                    except ValueError:
                        pass

    def compute_target_velocity(
        self,
        kp_v=0.5,
        kp_w=0.5,
        max_v=0.3,
        max_w=0.6,
        distance_tolerance=0.05,
    ):
        """
        Calculates the command velocities to reach the target coordinates.
        Returns: (cmd_v, cmd_w)
        """
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        distance_error = hypot(dx, dy)
        if distance_error < distance_tolerance:
            self.is_goal_reached = True
            self.targ_lin_vel = 0.0  # Stop the robot
            self.targ_ang_vel = 0.0  # Stop the robot
        else:
            self.is_goal_reached = False
            target_heading = atan2(dy, dx)
            heading_error = target_heading - self.theta
            heading_error = atan2(sin(heading_error), cos(heading_error))
            cmd_w = kp_w * heading_error
            direction_alignment = max(
                0.0, cos(heading_error)
            )  # slow down if heading too off
            cmd_v = kp_v * distance_error * direction_alignment

            self.targ_lin_vel = max(min(cmd_v, max_v), -max_v)
            self.targ_ang_vel = max(min(cmd_w, max_w), -max_w)

    def set_goal(self, goal_x, goal_y):
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.is_goal_reached = False


# User-defined callback function: This is the callback function that will be called when data is available from the pipeline
def app_callback(pad, info, user_data):
    user_data.set_goal(1.0, -0.5)  # Set a goal for the robot to navigate to (example: x=1.0, y=1.0)
    print(user_data.motion_data)
    user_data.increment()  # Using the user_data to count the number of frames
    string_to_print = f"Frame count: {user_data.get_count()}\n"
    buffer = info.get_buffer()  # Get the GstBuffer from the probe info
    if buffer is None:  # Check if the buffer is valid
        return Gst.PadProbeReturn.OK
    for detection in hailo.get_roi_from_buffer(buffer).get_objects_typed(
        hailo.HAILO_DETECTION
    ):  # Get the detections from the buffer & Parse the detections
        string_to_print += f"Detection: {detection.get_label()} Confidence: {detection.get_confidence():.2f}\n"
    # print(string_to_print)
    # print(dir(buffer))
    return Gst.PadProbeReturn.OK


if __name__ == "__main__":
    project_root = Path(__file__).resolve().parent.parent
    env_file = project_root / ".env"
    env_path_str = str(env_file)
    os.environ["HAILO_ENV_FILE"] = env_path_str
    user_data = (
        user_app_callback_class()
    )  # Create an instance of the user app callback class
    app = GStreamerDetectionApp(app_callback, user_data)
    app.run()
