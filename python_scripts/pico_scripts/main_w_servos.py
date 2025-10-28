"""
Rename this script to main.py, then upload to the pico board.
"""

import sys
import select
from diff_drive_controller import DiffDriveController
from machine import WDT, freq, reset, Pin, PWM
from time import sleep

# SETUP
# Overclock
freq(200_000_000)  # Pico 2 original: 150_000_000
# Instantiate robot
balle = DiffDriveController(
    left_ids=((6, 7, 8), (11, 10)), right_ids=((2, 3, 4), (21, 20))
)

# Instantiate servos
servo_claw = PWM(Pin(15))
servo_claw.freq(50)
servo_arm = PWM(Pin(14))
servo_arm.freq(50)

# Initial "rest" positions
servo_claw.duty_ns(1800000)  # mid
servo_arm.duty_ns(1650000)   # raised


# Create a poll to receive messages from host machine
cmd_vel_listener = select.poll()
cmd_vel_listener.register(sys.stdin, select.POLLIN)
event = cmd_vel_listener.poll()
# Config watchdog timer
wdt = WDT(timeout=500)  # ms

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
try:
    while True:
        target_lin_vel, target_ang_vel = 0.0, 0.0
        for msg, _ in event:
            if msg:
                wdt.feed()
                buffer = msg.readline().rstrip().split(",")
                # Handle servo commands
                if buffer == b"GRAB":
                    grab_sequence()
                    continue
                elif buffer == b"RELEASE":
                    rest_sequence()
                    continue

                # Otherwise handle velocity
                buffer = buffer.split(",")

                # print(f"{balle.lin_vel},{balle.ang_vel}")
                if len(buffer) == 2:
                    try:
                        target_lin_vel = float(buffer[0])
                        target_ang_vel = float(buffer[1])
                        balle.set_vel(target_lin_vel, target_ang_vel)
                    except ValueError:
                        balle.set_vel(0.0, 0.0)
                        # print("ValueError!")  # debug
                        # reset()
            else:
                # print("No message received")  # debug
                balle.set_vel(0.0, 0.0)
                # reset()

except Exception as e:
    # print('Pico reset')  # debug
    reset()
finally:
    # print('Pico reset')  # debug
    reset()