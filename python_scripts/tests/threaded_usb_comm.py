import threading
from time import time, sleep
from serial import Serial


class DeadReckonNavigator:
    def __init__(self) -> None:
        self.pico_msngr = Serial(port="/dev/ttyACM0", baudrate=115200, timeout=0.01)
        print(f"Messenger initiated at: {self.pico_msngr.name}\n")
        self.targ_lin_vel = 0.0
        self.targ_ang_vel = 0.0
        self.motion_data = {key: 0.0 for key in ["est_lin_vel", "est_ang_vel"]}  # lin vel estimated from encoder, ang vel fused from encoder and imu
        self.last_ts = time()  # time stamp in s
        self.pico_thread = threading.Thread(target=self.process_pico_msgs, daemon=True)
        self.pico_thread.start()

    def process_pico_msgs(self):
        last_ts = time()
        while self.pico_msngr is not None:
            # Transmit velocity commands to Pico
            curr_ts = time()
            # if (current_ts - self.last_ts) >= 0.04:  # TX freq: 25 Hz
            if (curr_ts - last_ts) >= 0.04:  # TX freq: 25 Hz
                msg_to_pico = f"{self.targ_lin_vel:.3f},{self.targ_ang_vel:.3f}\n"
                # Encode string to bytes and send
                self.pico_msngr.write(msg_to_pico.encode("utf-8"))
                last_ts = curr_ts

            # Receive motion data from Pico
            if self.pico_msngr.inWaiting() > 0:
                msg_from_pico = (
                    self.pico_msngr.readline().decode("utf-8", "ignore").strip()
                )
                if msg_from_pico:
                    data_strings = msg_from_pico.split(",")
                    try:
                        self.motion_data.update(
                            zip(
                                self.motion_data.keys(),
                                map(
                                    float, data_strings
                                ),  # convert all str in list to float
                            )
                        )
                    except ValueError:
                        pass


if __name__ == "__main__":
    navigator = DeadReckonNavigator()
    while True:
        print(f"[{time()}]Received motion data:\n---\n{navigator.motion_data}")
        sleep(0.1)
