from machine import Timer
from wheel_controller import WheelController
import sys


class DiffDriveController:
    def __init__(self, left_ids: tuple, right_ids: tuple) -> None:
        # Wheels
        self.left_wheel = WheelController(*left_ids)
        self.right_wheel = WheelController(*right_ids)
        self.velmon_timer = Timer(
            mode=Timer.PERIODIC, freq=50, callback=self.monitor_velocity
        )
        # Properties
        self.WHEEL_SEP = 0.21  # wheel separation distance
        # Variables
        self.lin_vel = 0.0
        self.ang_vel = 0.0

    def monitor_velocity(self, timer):
        """
        Compute and transmit robot velocity
        Note - if transmitting activated, Pico may stop responding.
        Nuke the Pico if further changes on code are needed.
        """
        self.lin_vel = 0.5 * (
            self.left_wheel.lin_vel + self.right_wheel.lin_vel
        )  # robot's linear velocity
        self.ang_vel = (
            self.right_wheel.lin_vel - self.left_wheel.lin_vel
        ) / self.WHEEL_SEP  # robot's angular velocity
        sys.stdout.write(
            f"{self.lin_vel},{self.ang_vel}\n"
        )  # uncomment to transmit robot velocity

    def set_vel(self, target_lin_vel, target_ang_vel):
        left_target = target_lin_vel - 0.5 * (target_ang_vel * self.WHEEL_SEP)
        right_target = target_lin_vel + 0.5 * (target_ang_vel * self.WHEEL_SEP)
        self.left_wheel.set_lin_vel(left_target)
        self.right_wheel.set_lin_vel(right_target)


# TEST
if __name__ == "__main__":
    from time import sleep

    bot = DiffDriveController(
        left_ids=((6, 7, 8), (11, 10)), right_ids=((2, 3, 4), (21, 20))
    )
    for v in range(1, 11):
        bot.set_vel(v / 10, 0.0)
        sleep(1.5)
        print(f"target velocity: {v/10}, actual velocity: {bot.lin_vel}")
    for v in reversed(range(10)):
        bot.set_vel(v / 10, 0.0)
        sleep(1.5)
        print(f"target velocity: {v/10}, actual velocity: {bot.lin_vel}")
    bot.set_vel(0.0, 0.0)
    sleep(1)
