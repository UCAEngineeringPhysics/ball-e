import sys
from pathlib import Path
import serial
import pygame
import json
from time import sleep, time
import cv2 as cv
from picamera2 import Picamera2
import os


# SETUP
# Load configs
params_file_path = str(Path(__file__).parents[1].joinpath("image_configs.json"))
with open(params_file_path, "r") as file:
    params = json.load(file)
# Init serial port
messenger = serial.Serial(port="/dev/ttyACM0", baudrate=115200)
print(f"Pico is connected to port: {messenger.name}")
# Init controller
pygame.display.init()
pygame.joystick.init()
js = pygame.joystick.Joystick(0)
print(f"Controller: {js.get_name()}")

# Init Pi Camera
cv.startWindowThread()
cam = Picamera2()
cam.configure(
    cam.create_preview_configuration(
        main={"format": "RGB888", "size": (224, 224)},
        controls={
            "FrameDurationLimits": (
                int(1_000_000 / params["frame_rate"]),
                int(1_000_000 / params["frame_rate"]),
            )
        },
    )
)
cam.start()
# Camera warmup countdown
for i in reversed(range(3 * params["frame_rate"])):
    frame = cam.capture_array()
    if frame is None:
        print("No frame received. TERMINATE!")
        sys.exit()
    if not i % params["frame_rate"]:
        print(i / params["frame_rate"])  # count down 3, 2, 1 sec

# Init joystick axes values
ax_val_ang = 0.0
ax_val_lin = 0.0
act_lower, act_close = 0, 0

# Flags
is_stopped = False
is_paused = True  # Start in pause mode
is_recording = False
collected_images = []  # Store images in memory
last_frame_time = time()
frame_interval = 1.0 / params["frame_rate"]  # Time between frames

print("\n=== Controls ===")
print(f"Left stick: Forward/Backward (max {params['lin_vel_max']} m/s)")
print(f"Right stick: Turn Left/Right (max {params['ang_vel_max']} rad/s)")
print(f"Button {params['lower_button']}: Lower arm")
print(f"Button {params['raise_button']}: Raise arm")
print(f"Button {params['close_button']}: Close claw")
print(f"Button {params['open_button']}: Open claw")
print(f"Button {params['start_stop_collection']}: Start/Stop image collection")
print(f"Button {params['stop_program']}: Stop program and save images")
print("================\n")
print("Robot ready! Image collection is PAUSED. Press Ctrl+C or button to stop.\n")


def save_images_to_directory():
    """Save collected images to a directory with incrementing name"""
    base_dir = Path(__file__).parents[1]
    dir_num = 1
    while True:
        dir_name = f"images_{dir_num}"
        dir_path = base_dir / dir_name
        if not dir_path.exists():
            dir_path.mkdir()
            print(f"\nSaving {len(collected_images)} images to {dir_name}/")
            for idx, img in enumerate(collected_images):
                img_path = dir_path / f"image_{idx:06d}.jpg"
                cv.imwrite(str(img_path), cv.cvtColor(img, cv.COLOR_RGB2BGR))
            print(f"Images saved successfully to {dir_name}/")
            return
        dir_num += 1


# MAIN LOOP
try:
    while not is_stopped:
        # Process camera data
        frame = cam.capture_array()
        if frame is None:
            print("No frame received. TERMINATE!")
            break
        
        # Display camera feed
        cv.imshow("camera", frame)
        if cv.waitKey(1) == ord("q"):  # [q]uit
            print("Quit signal received.")
            break
        
        # Capture images at frame_rate when recording
        current_time = time()
        if is_recording and (current_time - last_frame_time >= frame_interval):
            collected_images.append(frame.copy())
            last_frame_time = current_time
        
        # Process gamepad data
        for e in pygame.event.get():  # read controller input
            if e.type == pygame.JOYBUTTONDOWN:
                # Stop program button
                if js.get_button(params["stop_program"]):
                    is_stopped = True
                    print("\nStop button pressed. Saving images...")
                    break
                # Start/Stop collection button
                elif js.get_button(params["start_stop_collection"]):
                    is_paused = not is_paused
                    if is_paused:
                        is_recording = False
                        print("Image collection PAUSED")
                    else:
                        is_recording = True
                        last_frame_time = time()  # Reset timer
                        print("Image collection STARTED")
                # Arm and claw controls
                if js.get_button(params["lower_button"]):
                    act_lower = 1
                elif js.get_button(params["raise_button"]):
                    act_lower = -1
                if js.get_button(params["close_button"]):
                    act_close = 1
                elif js.get_button(params["open_button"]):
                    act_close = -1
            elif e.type == pygame.JOYBUTTONUP:
                if not js.get_button(params["lower_button"]) and not js.get_button(params["raise_button"]):
                    act_lower = 0
                if not js.get_button(params["close_button"]) and not js.get_button(params["open_button"]):
                    act_close = 0
            elif e.type == pygame.JOYAXISMOTION:
                ax_val_ang = round(
                    (js.get_axis(params["ang_joy_axis"])), 1
                )  # keep 1 decimal
                ax_val_lin = round(
                    (js.get_axis(params["lin_joy_axis"])), 1
                )  # keep 1 decimal
        
        # Calculate steering and throttle value
        act_ang = -ax_val_ang * params["ang_vel_max"]  # -1: left most; +1: right most
        act_lin = (
            -ax_val_lin * params["lin_vel_max"]
        )  # -1: max forward, +1: max backward
        
        msg = f"{act_lin}, {act_ang}, {act_close}, {act_lower}\n".encode("utf-8")
        messenger.write(msg)
        
        # Drain feedback buffer to prevent clogging (non-blocking)
        # This keeps communication smooth even if feedback isn't being used
        feedback_count = 0
        while messenger.in_waiting > 0 and feedback_count < 10:  # Limit to prevent blocking
            try:
                messenger.readline()  # Discard feedback to prevent buffer buildup
            except:
                break
            feedback_count += 1
        
        # 100Hz control loop (10ms period) - matches blocking poll on Pico
        sleep(0.01)

# Take care terminal signal (Ctrl-c) and stop button
except KeyboardInterrupt:
    print("\nShutting down...")
    is_stopped = True

finally:
    # Stop robot
    messenger.write(b"0.0,0.0,0,0\n")
    sleep(0.1)
    
    # Save images if any were collected
    if collected_images:
        save_images_to_directory()
    else:
        print("\nNo images collected.")
    
    # Cleanup
    pygame.quit()
    messenger.close()
    cam.stop()
    cv.destroyAllWindows()
    print("Robot stopped. Goodbye!")
    sys.exit()
