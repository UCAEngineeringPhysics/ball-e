#(main)

"""
Rename this script to main.py, then upload to the pico board.
"""

import sys
import select
from diff_drive_controller import DiffDriveController
from arm_drive_controller import ArmDrive
from machine import freq, Pin, PWM
from utime import ticks_us

# --- SETUP ---
freq(200_000_000)  # Overclock Pico 2

balle = DiffDriveController(
    left_ids=((6, 7, 8), (11, 10)), right_ids=((2, 3, 4), (21, 20))
)

arm = ArmDrive(claw_pin = 12, arm_pin = 13)


# Create a poller
cmd_vel_listener = select.poll()
cmd_vel_listener.register(sys.stdin, select.POLLIN)
event = cmd_vel_listener.poll()
target_lin_vel, target_ang_vel = 0.0, 0.0
arm_msg = 0.0
# arm_msg: 
# -1.0 = lower arm and open claw
# 0 = close claw
# 1 = raise arm

tic = ticks_us()

# initialize variables for arm+claw motion

arm_state = "idle"
arm_timer = ticks_us()

# Timer function for non-blocking arm+claw motion

def update_arm():
    global arm_state, arm_timer

    now = ticks_us()

    # IDLE
    if arm_state == "idle":
        return

    # Lower arm + open claw
    if arm_state == "lower":
        arm.lower_arm()
        arm.open_claw()
        if now - arm_timer > 700000:   # 0.7 seconds
            arm_state = "close"
            arm_timer = now
        return

    # Close claw
    if arm_state == "close":
        arm.close_claw()
        if now - arm_timer > 700000:
            arm_state = "raise"
            arm_timer = now
        return

    # Raise arm
    if arm_state == "raise":
        arm.raise_arm()
        if now - arm_timer > 700000:
            arm_state = "idle"
        return


# --- MAIN LOOP ---

while True:
    # Check for new serial input
    new_events = cmd_vel_listener.poll(0)
    
    for msg, _ in new_events:
        buffer = msg.readline().strip().split(",")

    for msg, _ in new_events:
        buffer = msg.readline().strip().split(",")

        # Must have at least linear and angular velocity
        if len(buffer) >= 2:
            target_lin_vel = float(buffer[0])
            target_ang_vel = float(buffer[1])
            balle.set_vels(target_lin_vel, target_ang_vel)

            # Optional 3rd value for arm
            if len(buffer) >= 3:
                arm_msg = float(buffer[2])
                # handle arm_msg here

            if arm_msg == -1.0 and arm_state == "idle":
                arm_state = "lower"
                arm_timer = ticks_us()

            elif arm_msg == 0.0 and arm_state == "idle":
                arm_state = "close"
                arm_timer = ticks_us()

            elif arm_msg == 1.0 and arm_state == "idle":
                arm_state = "raise"
                arm_timer = ticks_us()

    # Update arm every iteration
    update_arm()

    # Send feedback @ 100Hz
    toc = ticks_us()
    if toc - tic >= 10000:
        meas_lin_vel, meas_ang_vel = balle.get_vels()
        sys.stdout.write(f"{meas_lin_vel}, {meas_ang_vel}\n")
        tic = ticks_us()
