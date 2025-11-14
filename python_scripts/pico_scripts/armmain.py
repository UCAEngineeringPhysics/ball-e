#(main)
from arm_drive_controller import ArmDrive

"""
Rename this script to main.py, then upload to the pico board.
"""

import sys
import select
from diff_drive_controller import DiffDriveController
from machine import freq, Pin, PWM
from utime import ticks_us
from time import sleep

# --- SETUP ---
freq(200_000_000)  # Overclock Pico 2

balle = DiffDriveController(
    left_ids=((6, 7, 8), (11, 10)), right_ids=((2, 3, 4), (21, 20))
)

arm = ArmDrive(claw_pin = 12, arm_pin = 13)


# Create a poller
cmd_vel_listener = select.poll()
cmd_vel_listener.register(sys.stdin, select.POLLIN)

target_lin_vel, target_ang_vel = 0.0, 0.0
arm_msg = 0.0
# arm_msg: 
# -1.0 = lower arm and open claw
# 0 = close claw
# 1 = raise arm

tic = ticks_us()



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
            if arm_msg == -1.0:
                for _ in range(20):
                    arm.lower_arm()
                    arm.open_claw()
                    sleep(0.1)
            elif arm_msg == 0.0:
                for _ in range(20):
                    arm.close_claw()
                    sleep(0.1)  
            else:					#arm_msg = 1.0
                for _ in range(20):
                    arm.raise_arm()
                    sleep(0.1)
                    
    toc = ticks_us()
    if toc - tic >= 10000:
        meas_lin_vel, meas_ang_vel = balle.get_vel()
        out_msg = f"{meas_lin_vel}, {meas_ang_vel}\n"
#         out_msg = "PICO\n"
        sys.stdout.write(out_msg)
        tic = ticks_us()