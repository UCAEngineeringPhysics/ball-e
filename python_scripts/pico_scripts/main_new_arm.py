"""
Rename this file to main.py
"""

import sys
from utime import ticks_us, ticks_diff
import select
from machine import freq
from manipulation.arm_controller import ArmController
from mobile_base.diff_drive_controller import DiffDriveController
from perception.inertial_sensor import MPU6050

# SETUP
ALPHA = 0.90  # weight for gyro measured angular velocity
# Overclock
freq(240_000_000)  # Pico2 original: 150_000_000
# Instantiate robot
imu = MPU6050(pow_id=3, scl_id=5, sda_id=4, i2c_addr=0x68)
arm = ArmController(
    claw_pin=15,
    shoulder_a_pin=13,
    shoulder_b_pin=14,
)
mobile_base = DiffDriveController(
    left_ids=((21, 19, 20), (6, 7)),
    right_ids=((16, 18, 17), (26, 27)),
)
pico_messenger = select.poll()  # create a poll object
pico_messenger.register(sys.stdin, select.POLLIN)  # peek at serial port input
# Constants
tx_period_us = 16_667  # 60Hz
# Variables
targ_lin_vel, targ_ang_vel = 0.0, 0.0
last_us = ticks_us()
# print("Pico is ready...")  # debug

# LOOP
while True:
    # Transmit data (TX)
    now_us = ticks_us()
    if ticks_diff(now_us, last_us) >= tx_period_us:
        meas_lin_vel, meas_ang_vel = mobile_base.get_vels()
        motion_data = imu.read_data()
        fuse_ang_vel = ALPHA * motion_data["omg_z"] + (1 - ALPHA) * meas_ang_vel
        shoulder_duty_a = arm.target_shoa
        shoulder_duty_b = arm.target_shob
        claw_duty = arm.target_claw
        goal_met = int(arm.is_target_reached)
        out_msg = f"{meas_lin_vel:.3f},{fuse_ang_vel:.3f}, {goal_met}"
        print(out_msg)  # main.py will send this to computer
        last_us = now_us  # update last time stamp
    # Receive data (RX)
    is_waiting = pico_messenger.poll(0)  # check data in USB
    if is_waiting:
        in_msg = sys.stdin.readline().strip()  # take out whitespaces
        targ_vels = in_msg.split(",")  # get a list
        if len(targ_vels) == 4:
            targ_lin_vel = float(targ_vels[0])
            targ_ang_vel = float(targ_vels[1])
            claw_pw = int(targ_vels[2])
            shoa_pw = int(targ_vels[3])
            mobile_base.set_vels(targ_lin_vel, targ_ang_vel)
            arm.set_joint_positions(claw_pw, shoa_pw)
             

