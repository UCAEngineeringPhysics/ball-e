import sys
from pathlib import Path
import serial
import pygame
import json
from time import sleep, time
from datetime import datetime
import cv2 as cv
from picamera2 import Picamera2
import csv

# ============================================================================
# CONFIGURATION AND SETUP
# ============================================================================

def get_project_root():
    """Get project root directory (parent of images/ directory)."""
    return Path(__file__).resolve().parent.parent

def load_config(config_path=None):
    """Load configuration from JSON file with validation."""
    if config_path is None:
        # Try to find config in senior_design_stuff first, then project root
        project_root = get_project_root()
        config_path = project_root / "senior_design_stuff" / "configs.json"
        if not config_path.exists():
            config_path = project_root / "configs.json"
    
    try:
        with open(config_path, "r") as f:
            params = json.load(f)
        
        # Validate required parameters
        required_keys = [
            "lin_vel_axis", "ang_vel_axis", "max_lin_vel", "max_ang_vel",
            "record_btn", "pause_btn", "stop_btn", "label_switch_btn",
            "preview_btn", "arm_lift_btn", "arm_lower_btn",
            "claw_open_btn", "claw_close_btn", "arm_speed", "claw_speed",
            "frame_rate", "record_cap"
        ]
        
        missing_keys = [key for key in required_keys if key not in params]
        if missing_keys:
            raise ValueError(f"Missing required config keys: {missing_keys}")
        
        return params
    except FileNotFoundError:
        print(f"ERROR: Config file not found at {config_path}")
        sys.exit(1)
    except json.JSONDecodeError as e:
        print(f"ERROR: Invalid JSON in config file: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"ERROR: Failed to load config: {e}")
        sys.exit(1)

def setup_paths():
    """Set up data directory paths."""
    project_root = get_project_root()
    data_dir = project_root / "senior_design_data"
    session_dir = data_dir / datetime.now().strftime("%Y-%m-%d-%H-%M")
    image_dir = session_dir / "images"
    label_path = session_dir / "labels.csv"
    
    # Create directories
    image_dir.mkdir(parents=True, exist_ok=True)
    
    # Create CSV with header if it doesn't exist
    if not label_path.exists():
        try:
            with open(label_path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["filename", "label"])
        except Exception as e:
            print(f"ERROR: Failed to create label CSV: {e}")
            sys.exit(1)
    
    return str(image_dir), str(label_path)

def init_serial(port="/dev/ttyACM0", baudrate=115200, timeout=1.0):
    """Initialize serial connection to Pico with error handling."""
    try:
        messenger = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        print(f"✓ Pico connected to port: {messenger.name}")
        return messenger
    except serial.SerialException as e:
        print(f"ERROR: Failed to connect to Pico on {port}: {e}")
        print("Make sure the Pico is connected and the port is correct.")
        sys.exit(1)
    except Exception as e:
        print(f"ERROR: Unexpected error initializing serial: {e}")
        sys.exit(1)

def init_gamepad():
    """Initialize pygame and gamepad with error handling."""
    try:
        pygame.display.init()
        pygame.joystick.init()
        
        if pygame.joystick.get_count() == 0:
            print("ERROR: No gamepad detected!")
            sys.exit(1)
        
        js = pygame.joystick.Joystick(0)
        js.init()
        print(f"✓ Controller connected: {js.get_name()}")
        return js
    except pygame.error as e:
        print(f"ERROR: Failed to initialize pygame: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"ERROR: Failed to initialize gamepad: {e}")
        sys.exit(1)

def init_camera(params):
    """Initialize camera with error handling."""
    try:
        cv.startWindowThread()
        cam = Picamera2()
        cam.configure(
            cam.create_preview_configuration(
                main={"format": "RGB888", "size": (200, 180)},
                controls={
                    "FrameDurationLimits": (
                        int(1_000_000 / params["frame_rate"]),
                        int(1_000_000 / params["frame_rate"]),
                    )
                },
            )
        )
        cam.start()
        print("✓ Camera initialized")
        return cam
    except Exception as e:
        print(f"ERROR: Failed to initialize camera: {e}")
        sys.exit(1)

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================

def handle_gamepad_events(js, params, state):
    """Process gamepad button events and update state."""
    for e in pygame.event.get():
        if e.type == pygame.JOYBUTTONDOWN:
            # Emergency stop
            if js.get_button(params["stop_btn"]):
                state["is_stopped"] = True
                print("⚠ E-STOP PRESSED. TERMINATING...")
                return
            
            # Pause/unpause
            elif js.get_button(params["pause_btn"]):
                state["is_paused"] = not state["is_paused"]
                if state["is_paused"]:
                    state["is_recording"] = False
                    state["lin_vel"] = 0.0
                    state["ang_vel"] = 0.0
                    state["sho_vel"] = 0
                    state["cla_vel"] = 0
                    state["arm_state"] = 10  # Reset to neutral
                print(f"Paused: {state['is_paused']}")
            
            # Toggle recording
            elif js.get_button(params["record_btn"]) and not state["is_paused"]:
                state["is_recording"] = not state["is_recording"]
                print(f"Recording: {state['is_recording']} | Label: {state['current_label']}")
            
            # Change label
            elif js.get_button(params["label_switch_btn"]):
                state["current_label_index"] = (state["current_label_index"] + 1) % len(state["LABELS"])
                state["current_label"] = state["LABELS"][state["current_label_index"]]
                print(f"\n=== LABEL CHANGED TO: {state['current_label']} ===\n")
            
            # Toggle preview
            elif js.get_button(params["preview_btn"]):
                state["show_preview"] = not state["show_preview"]
                if not state["show_preview"]:
                    cv.destroyAllWindows()
                print(f"Preview: {state['show_preview']}")
            
            # Arm controls - only if not paused
            elif not state["is_paused"]:
                if js.get_button(params["arm_lift_btn"]):
                    state["sho_vel"] = -params["arm_speed"]
                    state["arm_state"] = 20
                    print("Lifting arm")
                elif js.get_button(params["arm_lower_btn"]):
                    state["sho_vel"] = params["arm_speed"]
                    state["arm_state"] = 20
                    print("Lowering arm")
                elif js.get_button(params["claw_open_btn"]):
                    state["cla_vel"] = -params["claw_speed"]
                    state["arm_state"] = 20
                    print("Opening claw")
                elif js.get_button(params["claw_close_btn"]):
                    state["cla_vel"] = params["claw_speed"]
                    state["arm_state"] = 20
                    print("Closing claw")
        
        elif e.type == pygame.JOYBUTTONUP:
            # Stop arm when button released
            if e.button == params["arm_lift_btn"] or e.button == params["arm_lower_btn"]:
                state["sho_vel"] = 0
                if state["cla_vel"] == 0:
                    state["arm_state"] = 10
            # Stop claw when button released
            elif e.button == params["claw_open_btn"] or e.button == params["claw_close_btn"]:
                state["cla_vel"] = 0
                if state["sho_vel"] == 0:
                    state["arm_state"] = 10

def update_joystick_velocities(js, params, state):
    """Update linear and angular velocities from joystick axes."""
    if not state["is_paused"]:
        state["lin_vel"] = round(-js.get_axis(params["lin_vel_axis"]) * params["max_lin_vel"], 2)
        state["ang_vel"] = round(-js.get_axis(params["ang_vel_axis"]) * params["max_ang_vel"], 2)
    else:
        state["lin_vel"] = 0.0
        state["ang_vel"] = 0.0

def send_control_message(messenger, state):
    """Send control message to Pico with error handling."""
    try:
        msg = f"{state['lin_vel']}, {state['ang_vel']}, {state['sho_vel']}, {state['cla_vel']}, {state['arm_state']}\n".encode('utf-8')
        messenger.write(msg)
    except serial.SerialException as e:
        print(f"WARNING: Failed to send message to Pico: {e}")
    except Exception as e:
        print(f"WARNING: Unexpected error sending message: {e}")

def show_preview(frame, state):
    """Display camera preview with status overlay."""
    try:
        display_frame = cv.resize(frame, (400, 360))
        cv.putText(display_frame, f"Label: {state['current_label']}", (10, 30),
                  cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv.putText(display_frame, f"Recording: {state['is_recording']}", (10, 60),
                  cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if state['is_recording'] else (0, 0, 255), 2)
        cv.putText(display_frame, f"Paused: {state['is_paused']}", (10, 90),
                  cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv.putText(display_frame, f"Frames: {state['record_counts']}", (10, 120),
                  cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv.imshow("Camera Preview", display_frame)
        cv.waitKey(1)  # Non-blocking
    except Exception as e:
        print(f"WARNING: Preview display error: {e}")

def save_image_with_label(frame, image_dir, label_path, state, csv_writer=None):
    """Save image and write label to CSV with buffered writing."""
    try:
        image_filename = f"{state['frame_counts']:06d}.jpg"
        image_path = Path(image_dir) / image_filename
        
        # Save image
        if not cv.imwrite(str(image_path), frame):
            print(f"WARNING: Failed to save image {image_filename}")
            return False
        
        # Write to CSV (buffered if writer provided, otherwise direct)
        if csv_writer:
            csv_writer.writerow([image_filename, state['current_label']])
        else:
            with open(label_path, "a", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([image_filename, state['current_label']])
        
        state["record_counts"] += 1
        return True
    except Exception as e:
        print(f"WARNING: Failed to save image/label: {e}")
        return False

# ============================================================================
# MAIN PROGRAM
# ============================================================================

def main():
    """Main program entry point."""
    # Load configuration
    params = load_config()
    
    # Setup paths
    image_dir, label_path = setup_paths()
    print(f"✓ Data directory: {Path(image_dir).parent}")
    
    # Initialize hardware
    messenger = init_serial()
    js = init_gamepad()
    cam = init_camera(params)
    
    # Countdown
    print("\nStarting countdown...")
    for i in reversed(range(3 * params["frame_rate"])):
        frame = cam.capture_array()
        if frame is None:
            print("ERROR: No frame received during countdown. TERMINATING!")
            sys.exit(1)
        if not i % params["frame_rate"]:
            print(f"Starting in {i // params['frame_rate']}...")
    
    # Available labels
    LABELS = [
        "red_ball", "blue_ball", "green_ball", "yellow_ball",
        "red_bucket", "blue_bucket", "green_bucket", "yellow_bucket"
    ]
    
    # Initialize state dictionary
    state = {
        "LABELS": LABELS,
        "current_label_index": 0,
        "current_label": LABELS[0],
        "lin_vel": 0.0,
        "ang_vel": 0.0,
        "sho_vel": 0,
        "cla_vel": 0,
        "arm_state": 10,
        "is_stopped": False,
        "is_paused": True,
        "is_recording": False,
        "show_preview": False,
        "frame_counts": 0,
        "record_counts": 0,
    }
    
    print(f"\n=== CURRENT LABEL: {state['current_label']} ===\n")
    print("\n=== CONTROLS ===")
    print("Button 0: Emergency Stop")
    print("Button 1: Switch Label")
    print("Button 3: Toggle Preview")
    print("Button 4: Pause/Unpause")
    print("Button 5: Start/Stop Recording")
    print("Button 6: Lower Arm")
    print("Button 7: Lift Arm")
    print("Button 8: Close Claw")
    print("Button 9: Open Claw")
    print("Left Stick Y: Forward/Backward")
    print("Right Stick X: Turn Left/Right")
    print("================\n")
    
    # Frame rate timing
    target_frame_time = 1.0 / params["frame_rate"]
    last_frame_time = time()
    
    # CSV buffering - open file once for writing
    csv_file = None
    csv_writer = None
    csv_buffer_count = 0
    CSV_FLUSH_INTERVAL = 10  # Flush every N writes
    
    try:
        # Open CSV file for buffered writing
        csv_file = open(label_path, "a", newline="")
        csv_writer = csv.writer(csv_file)
        
        # Main loop
        while not state["is_stopped"]:
            loop_start_time = time()
            
            # Capture frame
            try:
                frame = cam.capture_array()
                if frame is None:
                    print("WARNING: No frame received, skipping...")
                    sleep(target_frame_time)
                    continue
            except Exception as e:
                print(f"WARNING: Camera capture error: {e}")
                sleep(target_frame_time)
                continue
            
            state["frame_counts"] += 1
            
            # Show preview if enabled
            if state["show_preview"]:
                show_preview(frame, state)
            
            # Process gamepad events
            handle_gamepad_events(js, params, state)
            if state["is_stopped"]:
                break
            
            # Update joystick velocities
            update_joystick_velocities(js, params, state)
            
            # Send control message to Pico
            send_control_message(messenger, state)
            
            # Save image and label if recording
            if state["is_recording"]:
                if save_image_with_label(frame, image_dir, label_path, state, csv_writer):
                    csv_buffer_count += 1
                    
                    # Flush CSV buffer periodically
                    if csv_buffer_count >= CSV_FLUSH_INTERVAL:
                        csv_file.flush()
                        csv_buffer_count = 0
                    
                    # Progress update
                    if state["record_counts"] % 100 == 0:
                        print(f"Recorded {state['record_counts']} frames with label '{state['current_label']}'")
                    
                    # Auto-pause if hit record cap
                    if state["record_counts"] >= params["record_cap"]:
                        state["is_paused"] = True
                        state["is_recording"] = False
                        print(f"Reached record cap of {params['record_cap']} frames. Paused.")
            
            # Frame rate limiting
            elapsed = time() - loop_start_time
            sleep_time = max(0, target_frame_time - elapsed)
            if sleep_time > 0:
                sleep(sleep_time)
    
    except KeyboardInterrupt:
        print("\n⚠ Keyboard interrupt received. Shutting down...")
    
    except Exception as e:
        print(f"\nERROR: Unexpected error in main loop: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Cleanup
        print("\nCleaning up...")
        
        # Close CSV file
        if csv_file:
            csv_file.flush()
            csv_file.close()
        
        # Stop robot
        try:
            stop_msg = "0.0, 0.0, 0, 0, 10\n".encode('utf-8')
            messenger.write(stop_msg)
        except:
            pass
        
        # Close resources
        cv.destroyAllWindows()
        pygame.quit()
        try:
            messenger.close()
        except:
            pass
        try:
            cam.stop()
        except:
            pass
        
        print(f"\n✓ Session complete! Recorded {state['record_counts']} total frames.")
        print(f"✓ Data saved to: {Path(image_dir).parent}")

if __name__ == "__main__":
    main()

