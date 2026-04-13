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

# --- Servo setup ---
servo_claw = PWM(Pin(12))
servo_claw.freq(50)
servo_arm = PWM(Pin(13))
servo_arm.freq(50)

# Rest positions
servo_claw.duty_ns(1800000)
servo_arm.duty_ns(1650000)

# Create a poller
cmd_vel_listener = select.poll()
cmd_vel_listener.register(sys.stdin, select.POLLIN)

target_lin_vel, target_ang_vel = 0.0, 0.0
tic = ticks_us()

# --- Servo motion sequences ---
def grab_sequence():
    servo_arm.duty_ns(2300000)  # lower arm
    sleep(0.5)
    servo_claw.duty_ns(1550000)  # open claw
    sleep(0.5)
    servo_claw.duty_ns(2300000)  # close claw
    sleep(0.5)
    servo_arm.duty_ns(1650000)  # raise arm
    sleep(0.5)

def rest_sequence():
    servo_claw.duty_ns(1800000)
    sleep(0.5)
    servo_arm.duty_ns(1650000)
    sleep(0.5)

# --- MAIN LOOP ---
while True:
    # Check for new data from host every 10 ms
    events = cmd_vel_listener.poll(10)
    for fd, _ in events:
        line = sys.stdin.readline().strip()
        if not line:
            continue

        # Convert bytes → string if necessary
        if isinstance(line, bytes):
            line = line.decode("utf-8")

        # Handle servo commands first
        if line == "GRAB":
            grab_sequence()
            continue
        elif line == "RELEASE":
            rest_sequence()
            continue

        # Handle velocity commands: "lin,ang"
        if "," in line:

            lin, ang = [float(x) for x in line.split(",")]
            target_lin_vel = lin
            target_ang_vel = ang
            balle.set_vel(target_lin_vel, target_ang_vel)


    # Send velocity feedback periodically
    toc = ticks_us()
    if toc - tic >= 10000:
        meas_lin_vel, meas_ang_vel = balle.get_vel()
        sys.stdout.write(f"{meas_lin_vel}, {meas_ang_vel}\n")
        tic = ticks_us()
