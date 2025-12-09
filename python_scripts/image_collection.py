import sys
from pathlib import Path
import serial
import pygame
import json
from time import sleep
from datetime import datetime
import cv2 as cv
from picamera2 import Picamera2
import csv

# SETUP
# Define paths
project_root = Path(__file__).parents[1]
data_dir = project_root.joinpath("senior_design_data")
image_dir = str(data_dir.joinpath(datetime.now().strftime("%Y-%m-%d-%H-%M"), "images"))
Path(image_dir).mkdir(parents=True, exist_ok=True)
label_path = str(Path(image_dir).parent.joinpath("labels.csv"))

# Create CSV with header if it doesn't exist
if not Path(label_path).exists():
    with open(label_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["filename", "label"])  # Header row

# Load configs
params_file_path = str(project_root.joinpath("scripts", "configs.json"))
with open(params_file_path, "r") as file:
    params = json.load(file)

# Init serial port
messenger = serial.Serial(port="/dev/ttyACM0", baudrate=115200)
print(f"Pico is connected to port: {messenger.name}")

# Init controller
pygame.display.init()
pygame.joystick.init()
js = pygame.joystick.Joystick(0)

# Init camera
cv.startWindowThread()
cam = Picamera2()
cam.configure(
    cam.create_preview_configuration(
        main={"format": "RGB888", "size": (200, 180)},  # width x height
        controls={
            "FrameDurationLimits": (
                int(1_000_000 / params["frame_rate"]),
                int(1_000_000 / params["frame_rate"]),
            )
        },
    )
)
cam.start()

# Countdown
for i in reversed(range(3 * params["frame_rate"])):
    frame = cam.capture_array()
    if frame is None:
        print("No frame received. TERMINATE!")
        sys.exit()
    if not i % params["frame_rate"]:
        print(f"Starting in {i // params['frame_rate']}...")

# Available labels
LABELS = [
    "red_ball", "blue_ball", "green_ball", "yellow_ball",
    "red_bucket", "blue_bucket", "green_bucket", "yellow_bucket"
]

# Init variables
current_label_index = 0
current_label = LABELS[current_label_index]
print(f"\n=== CURRENT LABEL: {current_label} ===\n")

# Robot control variables
lin_vel = 0.0
ang_vel = 0.0
sho_vel = 0  # shoulder velocity
cla_vel = 0  # claw velocity
arm_state = 10  # 10 = idle/neutral

# Flags
is_stopped = False
is_paused = True
is_recording = False
show_preview = False

# Counters
frame_counts = 0
record_counts = 0

# MAIN LOOP
try:
    while not is_stopped:
        # Capture frame
        frame = cam.capture_array()
        if frame is None:
            print("No frame received. TERMINATE!")
            break
        frame_counts += 1
        
        # Show preview if enabled
        if show_preview:
            display_frame = frame.copy()
            cv.putText(display_frame, f"Label: {current_label}", (10, 20), 
                      cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv.putText(display_frame, f"Recording: {is_recording}", (10, 40),
                      cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0) if is_recording else (0, 0, 255), 2)
            cv.imshow("Camera Preview", display_frame)
            if cv.waitKey(1) == ord("q"):
                break
        
        # Process gamepad input
        for e in pygame.event.get():
            if e.type == pygame.JOYBUTTONDOWN:
                # Emergency stop
                if js.get_button(params["stop_btn"]):
                    is_stopped = True
                    print("E-STOP PRESSED. TERMINATE")
                    break
                
                # Pause/unpause
                elif js.get_button(params["pause_btn"]):
                    is_paused = not is_paused
                    if is_paused:
                        is_recording = False
                        lin_vel = 0.0
                        ang_vel = 0.0
                        sho_vel = 0
                        cla_vel = 0
                        arm_state = 10  # Reset to neutral when paused
                    print(f"Paused: {is_paused}")
                
                # Toggle recording
                elif js.get_button(params["record_btn"]) and not is_paused:
                    is_recording = not is_recording
                    print(f"Recording: {is_recording} | Label: {current_label}")
                
                # Change label
                elif js.get_button(params["label_switch_btn"]):
                    current_label_index = (current_label_index + 1) % len(LABELS)
                    current_label = LABELS[current_label_index]
                    print(f"\n=== LABEL CHANGED TO: {current_label} ===\n")
                
                # Toggle preview
                elif js.get_button(params["preview_btn"]):
                    show_preview = not show_preview
                    if not show_preview:
                        cv.destroyAllWindows()
                    print(f"Preview: {show_preview}")
                
                # Arm controls - lift
                elif js.get_button(params["arm_lift_btn"]):
                    sho_vel = -params["arm_speed"]
                    arm_state = 20  # Active control mode
                    print("Lifting arm")
                
                # Arm controls - lower
                elif js.get_button(params["arm_lower_btn"]):
                    sho_vel = params["arm_speed"]
                    arm_state = 20  # Active control mode
                    print("Lowering arm")
                
                # Claw controls - open
                elif js.get_button(params["claw_open_btn"]):
                    cla_vel = -params["claw_speed"]
                    arm_state = 20  # Active control mode
                    print("Opening claw")
                
                # Claw controls - close
                elif js.get_button(params["claw_close_btn"]):
                    cla_vel = params["claw_speed"]
                    arm_state = 20  # Active control mode
                    print("Closing claw")
            
            elif e.type == pygame.JOYBUTTONUP:
                # Stop arm when button released
                if js.get_button(params["arm_lift_btn"]) == 0 and js.get_button(params["arm_lower_btn"]) == 0:
                    sho_vel = 0
                    # If both arm and claw stopped, return to neutral
                    if cla_vel == 0:
                        arm_state = 10
                # Stop claw when button released
                if js.get_button(params["claw_open_btn"]) == 0 and js.get_button(params["claw_close_btn"]) == 0:
                    cla_vel = 0
                    # If both arm and claw stopped, return to neutral
                    if sho_vel == 0:
                        arm_state = 10

            
            elif e.type == pygame.JOYAXISMOTION:
                if not is_paused:
                    # Linear velocity (forward/backward) - left stick vertical
                    lin_vel = round(-js.get_axis(params["lin_vel_axis"]) * params["max_lin_vel"], 2)
                    # Angular velocity (turning) - right stick horizontal
                    ang_vel = round(-js.get_axis(params["ang_vel_axis"]) * params["max_ang_vel"], 2)
        
        # Send control message to Pico
        msg = f"{lin_vel}, {ang_vel}, {sho_vel}, {cla_vel}, {arm_state}\n".encode('utf-8')
        messenger.write(msg)
        
        # Save image and label if recording
        if is_recording:
            image_filename = f"{frame_counts:06d}.jpg"
            cv.imwrite(f"{image_dir}/{image_filename}", frame)
            
            # Write to CSV: filename, label
            with open(label_path, "a+", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([image_filename, current_label])
            
            record_counts += 1
            if record_counts % 100 == 0:
                print(f"Recorded {record_counts} frames with label '{current_label}'")
            
            # Auto-pause if hit record cap
            if record_counts >= params["record_cap"]:
                is_paused = True
                is_recording = False
                print(f"Reached record cap of {params['record_cap']} frames. Paused.")
        
        sleep(1 / params["frame_rate"])  # Maintain frame rate

except KeyboardInterrupt:
    print("\nKeyboard interrupt received. Shutting down...")

finally:
    cv.destroyAllWindows()
    pygame.quit()
    messenger.close()
    cam.stop()
    print(f"\nSession complete! Recorded {record_counts} total frames.")
    print(f"Data saved to: {Path(image_dir).parent}")
    sys.exit()
# End of file: images/sd_image_collection.py