import sys
from pathlib import Path
import serial
import pygame
import json
from time import sleep


# SETUP
# Load configs
params_file_path = str(Path(__file__).parents[1].joinpath("configs.json"))
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
ax_val_ang = 0.0
ax_val_lin = 0.0
# Flags, ordered by priority
is_stopped = False
is_enabled = True
mode = "d"

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
                elif js.get_button(params["enable_btn"]):
                    is_enabled = not is_enabled
            elif e.type == pygame.JOYAXISMOTION:
                ax_val_ang = round(
                    (js.get_axis(params["ang_joy_axis"])), 1
                )  # keep 2 decimals
                ax_val_lin = round(
                    (js.get_axis(params["lin_joy_axis"])), 1
                )  # keep 2 decimals
        # Calaculate steering and throttle value
        act_ang = -ax_val_ang * params["ang_vel_max"]  # -1: left most; +1: right most
        act_lin = (
            -ax_val_lin * params["lin_vel_max"]
        )  # -1: max forward, +1: max backward
        if is_enabled:
            mode = "e"
        else:
            mode = "d"
        msg = f"{act_lin}, {act_ang}\n".encode("utf-8")
        if not act_ang == 0.0 or not act_lin == 0.0:
            messenger.write(msg)
        # Log action
        print(f"mode: {mode}, action: {act_lin, act_ang}")  # debug
        # 20Hz
        sleep(0.05)

# Take care terminal signal (Ctrl-c)
except KeyboardInterrupt:
    pygame.quit()
    messenger.close()
    sys.exit()
