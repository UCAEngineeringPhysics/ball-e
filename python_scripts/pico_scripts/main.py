"""
Rename this script to main.py, then upload to the pico board.
"""

import sys
import select
from diff_drive_controller import DiffDriveController
from arm_cont import ArmController
from machine import freq
from utime import ticks_us

# SETUP
# Overclock
freq(250_000_000)  # Pico 2 original: 150_000_000

# Instantiate robot
# left_ids: ((motor_pins), (encoder_pins))
# From your files: left = pins (6,7,8) and encoders (11,10)
#                  right = pins (2,3,4) and encoders (21,20)
ddc = DiffDriveController(
    left_ids=((6, 7, 8), (11, 10)), 
    right_ids=((2, 3, 4), (21, 20))
)

# arm_controller params: (claw_pin, arm_pin_left, arm_pin_right)
arm = ArmController(12, 13, 14)

# Create a poll to receive messages from host machine
cmd_vel_listener = select.poll()
cmd_vel_listener.register(sys.stdin, select.POLLIN)

target_lin_vel = 0.0
target_ang_vel = 0.0
sho_vel = 0
cla_vel = 0
prev_arm_state = 10

tic = ticks_us()

# LOOP
while True:
    event = cmd_vel_listener.poll(0)  # Non-blocking poll
    
    for msg, _ in event:
        buffer = msg.readline().strip().split(",")
        if len(buffer) == 5:
            try:
                target_lin_vel = float(buffer[0])
                target_ang_vel = float(buffer[1])
                sho_vel = int(buffer[2])
                cla_vel = int(buffer[3])
                arm_state = int(buffer[4])
                
                # Always update wheel velocities
                ddc.set_vel(target_lin_vel, target_ang_vel)
                
                # Handle arm/claw control
                if arm_state == 20:  # Active control mode
                    # Apply shoulder movement if non-zero
                    if sho_vel != 0:
                        arm.lower_claw(sho_vel)
                    
                    # Apply claw movement if non-zero
                    if cla_vel != 0:
                        arm.close_claw(cla_vel)
                        
                elif arm_state == 10 and prev_arm_state == 20:
                    # Transition back to neutral only when moving from active to idle
                    # and both velocities are zero
                    if sho_vel == 0 and cla_vel == 0:
                        arm.set_neutral()
                
                prev_arm_state = arm_state
                
            except (ValueError, IndexError) as e:
                # Skip malformed messages
                pass

    # Periodic velocity feedback
    toc = ticks_us()
    if toc - tic >= 100000:  # Every 100ms
        meas_lin_vel, meas_ang_vel = ddc.get_vel()
        out_msg = f"{meas_lin_vel:.2f}, {meas_ang_vel:.2f}\n"
        sys.stdout.write(out_msg)
        tic = ticks_us()