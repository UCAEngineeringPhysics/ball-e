"""
Rename this script to main.py, then upload to the pico board.
"""

import sys
import select
from diff_drive_controller import DiffDriveController
from machine import WDT, freq, reset

# SETUP
# Overclock
# freq(233_000_000)  # Pico 2 original: 150_000_000
# Instantiate robot
balle = DiffDriveController(
    left_ids=((6, 7, 8), (11, 10)), right_ids=((2, 3, 4), (21, 20))
)
# Create a poll to receive messages from host machine
cmd_vel_listener = select.poll()
cmd_vel_listener.register(sys.stdin, select.POLLIN)
event = cmd_vel_listener.poll()
# Config watchdog timer
wdt = WDT(timeout=500)  # ms

# LOOP
try:
    while True:
        target_lin_vel, target_ang_vel = 0.0, 0.0
        for msg, _ in event:
            if msg:
                wdt.feed()
                buffer = msg.readline().rstrip().split(",")
                print(f"{balle.lin_vel},{balle.ang_vel}")
                if len(buffer) == 2:
                    try:
                        target_lin_vel = float(buffer[0])
                        target_ang_vel = float(buffer[1])
                        balle.set_vel(target_lin_vel, target_ang_vel)
                    except ValueError:
                        balle.set_vel(0.0, 0.0)
                        print("ValueError!")  # debug
                        # reset()
            else:
                print("No message received")  # debug
                balle.set_vel(0.0, 0.0)
                # reset()

except Exception as e:
    print('Pico reset')  # debug
    reset()
finally:
    print('Pico reset')  # debug
    reset()