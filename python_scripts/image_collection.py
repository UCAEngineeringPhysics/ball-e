import sys
from pathlib import Path
import serial
import pygame
import json
from time import sleep
from time import time
from datetime import datetime
import cv2 as cv
from picamera2 import Picamera2
import csv

# SETUP
# Load configs
params_file_path = str(Path(__file__).parents[1].joinpath("scripts", "configs.json"))
with open(params_file_path, "r") as file:
    params = json.load(file)

# Init serial port
messenger = serial.Serial(port="/dev/ttyACM0", baudrate=115200)
print(f"Pico is connected to port: {messenger.name}")

# Init controller
pygame.display.init()
pygame.joystick.init()
js = pygame.joystick.Joystick(0)
# Init joystick axes values
ax_val_st = 0.0
ax_val_th = 0.0
# Flags, ordered by priority
is_stopped = False
is_paused = True
is_recording = False
mode = "p"

# MAIN LOOP
try:
    while not is_stopped:
        for e in pygame.event.get():  # read controller input
            if e.type == pygame.JOYBUTTONDOWN:
                if js.get_button(params["stop_btn"]):  # emergency stop
                    is_stopped = True
                    print("E-STOP PRESSED. TERMINATE")
                    pygame.quit()
                    messenger.close()
                    sys.exit()
                elif js.get_button(params["pause_btn"]):
                    is_paused = not is_paused
                    if is_paused:
                        is_recording = False
                    # print(f"Paused: {is_paused}")  # debug
                elif js.get_button(params["record_btn"]):
                    if not is_paused:
                        is_recording = not is_recording
                        # print(f"Recording: {is_recording}")  # debug
            elif e.type == pygame.JOYAXISMOTION:
                ax_val_st = round(
                    (js.get_axis(params["steering_joy_axis"])), 2
                )  # keep 2 decimals
                ax_val_th = round(
                    (js.get_axis(params["throttle_joy_axis"])), 2
                )  # keep 2 decimals
        # Calaculate steering and throttle value
        act_st = ax_val_st  # -1: left most; +1: right most
        act_th = -ax_val_th  # -1: max forward, +1: max backward
        # Encode steering value to dutycycle in nanosecond
        duty_st = params["steering_center"] + int(params["steering_range"] * act_st)
        # Encode throttle value to dutycycle in nanosecond
        if act_th > 0:
            duty_th = params["throttle_neutral"] + int(
                params["throttle_fwd_range"] * min(act_th, params["throttle_limit"])
            )
        elif act_th < 0:
            duty_th = params["throttle_neutral"] + int(
                params["throttle_fwd_range"] * max(act_th, -params["throttle_limit"])
            )
        else:
            duty_th = params["throttle_neutral"]
        if is_paused:
            mode = "p"
        else:
            if is_recording:
                mode = "r"
            else:
                mode = "n"
        msg = f"{mode}, {duty_st}, {duty_th}\n".encode("utf-8")
        messenger.write(msg)
        # Log action
        print(f"action: {act_st, act_th}")  # debug
        # 20Hz
        sleep(0.05)

# Take care terminal signal (Ctrl-c)
except KeyboardInterrupt:
    pygame.quit()
    messenger.close()
