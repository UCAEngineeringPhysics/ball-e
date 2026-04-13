import threading
from time import time, sleep
from serial import Serial
from math import sin, cos, atan2, hypot


class BlindNavigator:
    def __init__(self) -> None:
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
        kp_w=1.0,
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


if __name__ == "__main__":
    navigator = BlindNavigator()
    navigator.set_goal(3.0, 0.0)
    while not navigator.is_goal_reached:
        print(f"[{time()}]: x={navigator.x}, y ={navigator.y}, theta={navigator.theta}")
        sleep(0.1)
    print(f"Goal x={navigator.goal_x}, y={navigator.goal_y} reached.")
    navigator.set_goal(0.7, 3.8)
    while not navigator.is_goal_reached:
        print(f"[{time()}]: x={navigator.x}, y ={navigator.y}, theta={navigator.theta}")
        sleep(0.1)
    print(f"Goal x={navigator.goal_x}, y={navigator.goal_y} reached.")
    navigator.set_goal(8.5, 3.8)
    while not navigator.is_goal_reached:
        print(f"[{time()}]: x={navigator.x}, y ={navigator.y}, theta={navigator.theta}")
        sleep(0.1)
    print(f"Goal x={navigator.goal_x}, y={navigator.goal_y} reached.")
    navigator.set_goal(5.5, 0.0)
    while not navigator.is_goal_reached:
        print(f"[{time()}]: x={navigator.x}, y ={navigator.y}, theta={navigator.theta}")
        sleep(0.1)
    print(f"Goal x={navigator.goal_x}, y={navigator.goal_y} reached.")
    navigator.set_goal(8.5, -3.8)
    while not navigator.is_goal_reached:
        print(f"[{time()}]: x={navigator.x}, y ={navigator.y}, theta={navigator.theta}")
        sleep(0.1)
    print(f"Goal x={navigator.goal_x}, y={navigator.goal_y} reached.")
    navigator.set_goal(5.5, -2.0)
    while not navigator.is_goal_reached:
        print(f"[{time()}]: x={navigator.x}, y ={navigator.y}, theta={navigator.theta}")
        sleep(0.1)
    print(f"Goal x={navigator.goal_x}, y={navigator.goal_y} reached.")
    navigator.set_goal(0.7, -3.8)
    while not navigator.is_goal_reached:
        print(f"[{time()}]: x={navigator.x}, y ={navigator.y}, theta={navigator.theta}")
        sleep(0.1)
    print(f"Goal x={navigator.goal_x}, y={navigator.goal_y} reached.")
    navigator.set_goal(3.0, 0.0)
    while not navigator.is_goal_reached:
        print(f"[{time()}]: x={navigator.x}, y ={navigator.y}, theta={navigator.theta}")
        sleep(0.1)
    print(f"Goal x={navigator.goal_x}, y={navigator.goal_y} reached.")
    navigator.set_goal(0.0, 0.0)
    while not navigator.is_goal_reached:
        print(f"[{time()}]: x={navigator.x}, y ={navigator.y}, theta={navigator.theta}")
        sleep(0.1)
    print(f"Goal x={navigator.goal_x}, y={navigator.goal_y} reached.")
