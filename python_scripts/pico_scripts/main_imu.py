"""
Rename this file to main.py
"""

import sys
from utime import ticks_us, ticks_diff
import select
from machine import freq
from mobile_base.diff_drive_controller import DiffDriveController
from arm_controller import ArmController
from perception.inertial_sensor import MPU6050

# SETUP
ALPHA = 0.95  # weight for gyro measured angular velocity
# Overclock
freq(240_000_000)  # Pico2 original: 150_000_000
# Instantiate robot
imu = MPU6050(pow_id=3, scl_id=5, sda_id=4, i2c_addr=0x68)

mobile_base = DiffDriveController(
    left_ids=((21, 19, 20), (6, 7)), right_ids=((16, 18, 17), (26, 27))
)

arm = ArmController(15, 13, 14)



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
        
        shoulder_duty_a = arm.shoulder_duty_a
        shoulder_duty_b = arm.shoulder_duty_b
        claw_duty = arm.claw_duty
        
        motion_data = imu.read_data()
        fuse_ang_vel = ALPHA * motion_data["omg_z"] + (1 - ALPHA) * meas_ang_vel
        out_msg = f"{meas_lin_vel:.3f},{fuse_ang_vel:.3f}"
        print(out_msg)  # main.py will send this to computer
        last_us = now_us  # update last time stamp
    # Receive data (RX)
    is_waiting = pico_messenger.poll(0)  # check data in USB
    if is_waiting:
        in_msg = sys.stdin.readline().strip()  # take out whitespaces
        targ_vels = in_msg.split(",")  # get a list
        if len(targ_vels) == 5:
            targ_lin_vel = float(targ_vels[0])
            targ_ang_vel = float(targ_vels[1])
            
            sho_vel = int(buffer[2])
            cla_vel = int(buffer[3])
            arm_state = int(buffer[4])
            
            mobile_base.set_vels(targ_lin_vel, targ_ang_vel)
            
            if arm_state == 10:      # idle ? go to neutral
                arm.set_neutral()
            else:
                arm.lower_claw(sho_vel)
                arm.close_claw(cla_vel)  


