"""
Rename this script to main.py, then upload to the pico board.
"""

import sys
import select
from diff_drive_controller import DiffDriveController
from machine import freq, reset
from utime import ticks_us

# SETUP
# Overclock
freq(200_000_000)  # Pico 2 original: 150_000_000
# Instantiate robot
balle = DiffDriveController(
    left_ids=((6, 7, 8), (11, 10)), right_ids=((2, 3, 4), (21, 20))
)
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
        # print(f"{balle.lin_vel},{balle.ang_vel}")
        if len(buffer) == 2:
            target_lin_vel = float(buffer[0])
            target_ang_vel = float(buffer[1])
            balle.set_vel(target_lin_vel, target_ang_vel)
    toc = ticks_us()
    if toc - tic >= 10000:
        meas_lin_vel, meas_ang_vel = balle.get_vel()
        out_msg = f"{meas_lin_vel}, {meas_ang_vel}\n"
#         out_msg = "PICO\n"
        sys.stdout.write(out_msg)
        tic = ticks_us()
