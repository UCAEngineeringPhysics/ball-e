"""
Rename this script to main.py, then upload to the pico board.
"""

import sys
import select
from diff_drive_controller import DiffDriveController
from arm_controller import ArmController
from machine import freq
from utime import ticks_us, ticks_diff

# SETUP
# Overclock
freq(250_000_000)

# Instantiate robot
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
arm_state = 10

tic = ticks_us()
last_cmd_time = ticks_us()

print("Pico ready!")

# LOOP
while True:
    # Check for incoming commands (non-blocking)
    event = cmd_vel_listener.poll(0)
    
    if event:
        for msg, _ in event:
            try:
                line = msg.readline()
                if line:
                    buffer = line.strip().split(",")
                    if len(buffer) == 5:
                        target_lin_vel = float(buffer[0])
                        target_ang_vel = float(buffer[1])
                        sho_vel = int(buffer[2])
                        cla_vel = int(buffer[3])
                        arm_state = int(buffer[4])
                        
                        last_cmd_time = ticks_us()
                        
                        # Always update wheel velocities
                        ddc.set_vel(target_lin_vel, target_ang_vel)
                        
                        # Handle arm/claw control
                        if arm_state == 20:  # Active control mode
                            if sho_vel != 0:
                                arm.lower_claw(sho_vel)
                            if cla_vel != 0:
                                arm.close_claw(cla_vel)
                        elif arm_state == 10:
                            # Only reset to neutral if both velocities are zero
                            if sho_vel == 0 and cla_vel == 0:
                                arm.set_neutral()
                        
            except (ValueError, IndexError, AttributeError) as e:
                # Skip malformed messages
                pass
    
    # Safety timeout: stop if no commands received for 500ms
    if ticks_diff(ticks_us(), last_cmd_time) > 500000:
        ddc.set_vel(0.0, 0.0)
    
    # Send feedback less frequently (every 100ms)
    toc = ticks_us()
    if ticks_diff(toc, tic) >= 100000:
        try:
            meas_lin_vel, meas_ang_vel = ddc.get_vel()
            out_msg = f"{meas_lin_vel:.2f}, {meas_ang_vel:.2f}\n"
            sys.stdout.write(out_msg)
        except:
            pass
        tic = toc