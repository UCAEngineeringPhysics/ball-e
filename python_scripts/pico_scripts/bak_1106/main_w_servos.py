"""
Rename this script to main.py, then upload to the pico board.
"""

import sys
import select
from diff_drive_controller import DiffDriveController
from machine import freq, reset, Pin, PWM
from utime import ticks_us
from time import sleep

# SETUP
# Overclock
freq(200_000_000)  # Pico 2 original: 150_000_000
# Instantiate robot
balle = DiffDriveController(
    left_ids=((6, 7, 8), (11, 10)), right_ids=((2, 3, 4), (21, 20))
)

# Instantiate servos
servo_claw = PWM(Pin(12))
servo_claw.freq(50)
servo_arm = PWM(Pin(13))
servo_arm.freq(50)

# Initial "rest" positions
servo_claw.duty_ns(1800000)  # mid
servo_arm.duty_ns(1650000)   # raised


# Create a poll to receive messages from host machine
cmd_vel_listener = select.poll()
cmd_vel_listener.register(sys.stdin, select.POLLIN)
event = cmd_vel_listener.poll()
target_lin_vel, target_ang_vel = 0.0, 0.0
tic = ticks_us()

# Function to grab ball (triggered when ball is detected and within 0.5 ft of robot)
def grab_sequence():
    """Lower arm, open claw, close claw, raise arm."""
    servo_arm.duty_ns(2300000)  # lower arm
    sleep(0.5)
    servo_claw.duty_ns(1550000)  # open claw
    sleep(0.5)
    servo_claw.duty_ns(2300000)  # close claw
    sleep(0.5)
    servo_arm.duty_ns(1650000)  # raise arm
    sleep(0.5)


# Function to return to rest position
def rest_sequence():
    """Open claw."""
    servo_claw.duty_ns(1800000)
    sleep(0.5)
    servo_arm.duty_ns(1650000)
    sleep(0.5)



# LOOP

while True:
    for msg, _ in event:
            
        buffer = msg.readline().strip().split(",")

        if len(buffer) == 2:
            target_lin_vel = float(buffer[0])
            target_ang_vel = float(buffer[1])
            balle.set_vel(target_lin_vel, target_ang_vel)


        # Handle servo commands
        if buffer == b"GRAB":
            grab_sequence()
            continue
        elif buffer == b"RELEASE":
            rest_sequence()
            
    toc = ticks_us()
    if toc - tic >= 10000:
        meas_lin_vel, meas_ang_vel = balle.get_vel()
        out_msg = f"{meas_lin_vel}, {meas_ang_vel}\n"
#         out_msg = "PICO\n"
        sys.stdout.write(out_msg)
        tic = ticks_us()
