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

    for msg, _ in event:
        buffer = msg.readline().strip().split(",")
        # print(f"{balle.lin_vel},{balle.ang_vel}")
        if len(buffer) == 3:
            #Handle diff messages
            target_lin_vel = float(buffer[0])
            target_ang_vel = float(buffer[1])
            balle.set_vels(target_lin_vel, target_ang_vel)
            
            #Handle arm messages
            arm_msg = float(buffer[2])
            
            # Handle arm messages using non-blocking timer for loops
            if arm_msg == -1.0 and arm_state == "idle":
                arm_state = "lower"
                arm_timer = ticks_us()

            elif arm_msg == 0.0 and arm_state == "idle":
                arm_state = "close"
                arm_timer = ticks_us()

            elif arm_msg == 1.0 and arm_state == "idle":
                arm_state = "raise"
                arm_timer = ticks_us()
                
    # Check arm+claw status every loop        
    update_arm()
                    
    toc = ticks_us()
    if toc - tic >= 10000:
        meas_lin_vel, meas_ang_vel = balle.get_vels()
        out_msg = f"{meas_lin_vel}, {meas_ang_vel}\n"
#         out_msg = "PICO\n"
        sys.stdout.write(out_msg)
        tic = ticks_us()

