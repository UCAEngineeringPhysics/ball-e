from sensored_motor_driver import SensoredMotorDriver
from machine import Timer
from math import pi

class SensoredDualMotorDriver:
    
    def __init__(
        self,
        left_drive_pin_ids,
        left_encoder_pin_ids,
        left_ab,
        right_drive_pin_ids,
        right_encoder_pin_ids,
        right_ab,
    ):
        # Configuration
        self.left_motor = SensoredMotorDriver(
            left_drive_pin_ids,
            left_encoder_pin_ids,
            left_ab
        )
        self.right_motor = SensoredMotorDriver(
            right_drive_pin_ids,
            right_encoder_pin_ids,
            right_ab
        )
        self.tim = Timer()
        self.tim.init(freq=10, mode=Timer.PERIODIC, callback=self.vel_mon)
        # Variables
        self.left_prev_counts = 0
        self.right_prev_counts = 0
        self.lin_vel = 0.
        self.ang_vel = 0.
        # Properties
        self.WHEEL_SEP = 0.525
        
    def vel_mon(self, timer):
        # Left wheel linear velocity
        left_pulses_inc = self.left_motor.counts - self.left_prev_counts
        self.left_prev_counts = self.left_motor.counts
        left_revs_inc = left_pulses_inc / self.left_motor.PPR
        left_rads_inc = left_revs_inc * 2 * pi
        left_ang_vel_motor = left_rads_inc / 0.1  # motor angular velocity
        left_ang_vel_wheel = left_ang_vel_motor / self.left_motor.GEAR_RATIO  # wheel angular velocity
        left_lin_vel = left_ang_vel_wheel * self.left_motor.WHEEL_RADIUS
        # Right wheel linear velocity
        right_pulses_inc = self.right_motor.counts - self.right_prev_counts
        self.right_prev_counts = self.right_motor.counts
        right_revs_inc = right_pulses_inc / self.right_motor.PPR
        right_rads_inc = right_revs_inc * 2 * pi
        right_ang_vel_motor = right_rads_inc / 0.1  # motor angular velocity
        right_ang_vel_wheel = right_ang_vel_motor / self.right_motor.GEAR_RATIO  # wheel angular velocity
        right_lin_vel = right_ang_vel_wheel * self.right_motor.WHEEL_RADIUS
        # Robot linear and angular velocity
        self.lin_vel = (left_lin_vel + right_lin_vel) * 0.5
        self.ang_vel = (right_lin_vel - left_lin_vel) / self.WHEEL_SEP
    
    def forward(self, duty):
        self.left_motor.forward(duty)
        self.right_motor.forward(duty)
        
    def backward(self, duty):
        self.left_motor.backward(duty)
        self.right_motor.backward(duty)

    def spin_left(self, duty):
        self.left_motor.backward(duty)
        self.right_motor.forward(duty)
        
    def spin_right(self, duty):
        self.left_motor.forward(duty)
        self.right_motor.backward(duty)

    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()        
        

if __name__ == '__main__':
    from time import sleep
    dmd = SensoredDualMotorDriver(
        (6, 7, 8),
        (11, 10),
        -1,
        (2, 3, 4),
        (21, 20),
        1,
    )
    dmd.spin_right(40000)
    for _ in range(4):
        sleep(1)
        print(f"robot linear velocity: {dmd.lin_vel}, robot angular velocity: {dmd.ang_vel}")  
    dmd.stop()
    