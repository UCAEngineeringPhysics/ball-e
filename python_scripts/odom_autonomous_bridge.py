"""
Bare-bones odom bridge used by bare-bones autonomous state machine.
"""

from dataclasses import dataclass
from math import atan2, cos, hypot, sin


@dataclass
class OdomPose:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


class OdomAutonomousBridge:
    def __init__(
        self,
        cam_offset_x_m=-0.64,
        cam_offset_y_m=0.0,
        cam_offset_z_m=0.2,
        kp_v=0.5,
        kp_w=0.5,
        max_v=0.30,
        max_w=0.60,
        distance_tolerance_m=0.05,
        forward_is_negative=True,
    ):
        self.cam_offset_x_m = cam_offset_x_m
        self.cam_offset_y_m = cam_offset_y_m
        self.cam_offset_z_m = cam_offset_z_m
        self.kp_v = kp_v
        self.kp_w = kp_w
        self.max_v = max_v
        self.max_w = max_w
        self.distance_tolerance_m = distance_tolerance_m
        self.forward_is_negative = forward_is_negative

        self.pose = OdomPose()
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_active = False

    @staticmethod
    def _clamp(value, low, high):
        return max(low, min(high, value))

    def set_goal(self, goal_x, goal_y):
        self.goal_x = float(goal_x)
        self.goal_y = float(goal_y)
        self.goal_active = True

    def clear_goal(self):
        self.goal_active = False

    def update_pose_from_feedback(self, meas_lin_vel, fuse_ang_vel, dt_s):
        if dt_s <= 0.0:
            return
        self.pose.x += meas_lin_vel * cos(self.pose.theta) * dt_s
        self.pose.y += meas_lin_vel * sin(self.pose.theta) * dt_s
        self.pose.theta += fuse_ang_vel * dt_s
        self.pose.theta = atan2(sin(self.pose.theta), cos(self.pose.theta))

    def compute_goal_velocity(self):
        if not self.goal_active:
            return 0.0, 0.0

        dx = self.goal_x - self.pose.x
        dy = self.goal_y - self.pose.y
        dist = hypot(dx, dy)
        if dist < self.distance_tolerance_m:
            self.clear_goal()
            return 0.0, 0.0

        heading = atan2(dy, dx)
        heading_err = atan2(sin(heading - self.pose.theta), cos(heading - self.pose.theta))

        cmd_w = self._clamp(self.kp_w * heading_err, -self.max_w, self.max_w)
        cmd_v_mag = self._clamp(self.kp_v * dist * max(0.0, cos(heading_err)), 0.0, self.max_v)
        cmd_v = -cmd_v_mag if self.forward_is_negative else cmd_v_mag
        return cmd_v, cmd_w

    def transform_cam_to_odom(self, coords_cam):
        x_c, y_c, z_c = coords_cam

        x_b = z_c + self.cam_offset_x_m
        y_b = -x_c + self.cam_offset_y_m
        z_b = -y_c + self.cam_offset_z_m

        c = cos(self.pose.theta)
        s = sin(self.pose.theta)
        x_o = c * x_b - s * y_b + self.pose.x
        y_o = s * x_b + c * y_b + self.pose.y
        return x_o, y_o, z_b