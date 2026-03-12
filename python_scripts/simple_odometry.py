import serial
import time
from math import sin, cos, pi, sqrt


class SimpleOdometry:
    def __init__(self, pico_serial_object, initial_theta=0.0):
        self.pico = pico_serial_object        
        # Position
        self.x = 0.0
        self.y = 0.0
        self.th = initial_theta # Set starting angle here
        
        # Velocities
        self.lin_vel = 0.0
        self.ang_vel = 0.0
        self.last_time = time.time()

    def reset_origin(self, new_theta=0.0):
        """Call this to zero out X/Y but set a specific facing direction."""
        self.x = 0.0
        self.y = 0.0
        self.th = new_theta
        print(f"\n[INFO] Origin reset. Heading: {new_theta:.2f} rad")

    def update(self):
        # 1. Serial Read
        if self.pico.in_waiting > 0:
            try:
                line = self.pico.readline().decode("utf-8", "ignore").strip()
                vels = line.split(",")
                if len(vels) == 2:
                    # Assuming Pico sends m/s and rad/s
                    self.lin_vel = float(vels[0]) 
                    self.ang_vel = float(vels[1])
            except (ValueError, IndexError):
                pass

        # 2. Timing
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        # 3. Dead Reckoning (Integration)
        # We update x and y in FEET directly now
        self.x += self.lin_vel * cos(self.th) * dt
        self.y += self.lin_vel * sin(self.th) * dt
        self.th += self.ang_vel * dt

        # Keep heading between -pi and pi
        self.th = (self.th + pi) % (2 * pi) - pi

    def get_distance_from_origin(self):
        """Calculates straight-line distance from starting corner."""
        return sqrt(self.x**2 + self.y**2)

