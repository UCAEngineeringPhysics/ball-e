"""
Rename this script to main.py, then upload to the pico board.
"""

import sys
import select
from diff_drive_controller import DiffDriveController
from arm_controller import ArmController
from machine import freq, reset
from utime import ticks_us

# SETUP
# Overclock
freq(250_000_000)  # Pico 2 original: 150_000_000
# Instantiate robot
ddc = DiffDriveController(
    left_ids=((6, 7, 8), (11, 10)), right_ids=((2, 3, 4), (21, 20))
)
arm = ArmController(12, 13, 14)
# Create a poll to receive messages from host machine
cmd_vel_listener = select.poll()
cmd_vel_listener.register(sys.stdin, select.POLLIN)
event = cmd_vel_listener.poll()
target_lin_vel, target_ang_vel = 0.0, 0.0
tic = ticks_us()

# LOOP
while True:
    for msg, _ in event:
        buffer = msg.readline().strip().split(",")
        if len(buffer) == 5:
            target_lin_vel = float(buffer[0])
            target_ang_vel = float(buffer[1])
            sho_vel = int(buffer[2])  # shoulder velocity increment
            cla_vel = int(buffer[3])  # claw velocity increment
            arm_state = int(buffer[4])
            
            # Set wheel velocities
            ddc.set_vel(target_lin_vel, target_ang_vel)
            
            # Handle arm control based on arm_state
            if arm_state == 10:  # idle/neutral state
                if sho_vel == 0 and cla_vel == 0:
                    arm.set_neutral()
            else:  # active control state
                # Apply shoulder movement (lift/lower arm)
                if sho_vel != 0:
                    arm.lower_claw(sho_vel)  # This actually controls shoulder
                
                # Apply claw movement (open/close)
                if cla_vel != 0:
                    arm.close_claw(cla_vel)  # This controls claw

    toc = ticks_us()
    if toc - tic >= 10000:
        meas_lin_vel, meas_ang_vel = ddc.get_vel()
        shoulder_duty_a = arm.shoulder_duty_a
        shoulder_duty_b = arm.shoulder_duty_b
        claw_duty = arm.claw_duty
        
        out_msg = f"{meas_lin_vel}, {meas_ang_vel}\n"
        sys.stdout.write(out_msg)
        tic = ticks_us()