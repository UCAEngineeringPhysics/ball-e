from machine import Pin, PWM
from time import sleep


class ArmDrive:
    def __init__(self, claw_pin, arm_pin):
        self.claw_servo = PWM(Pin(claw_pin))
        self.arm_servo = PWM(Pin(arm_pin))
        self.claw_servo.freq(50)
        self.arm_servo.freq(50)

        # Initialize individual duty values (in nanoseconds)
        self.arm_duty = 1_650_000
        self.claw_duty = 1_800_000

        # Set initial positions
        self.set_neutral()

    def set_neutral(self):
        # Return to rest positions
        self.arm_duty = 1_650_000
        self.claw_duty = 1_800_000
        self.arm_servo.duty_ns(self.arm_duty)
        self.claw_servo.duty_ns(self.claw_duty)

    def lower_arm(self):  # Lower arm
        self.arm_duty += 20_000
        if self.arm_duty >= 2_800_000:
            self.arm_duty = 2_800_000
        self.arm_servo.duty_ns(self.arm_duty)

    def open_claw(self):  # Open claw
        self.claw_duty -= 10_000
        if self.claw_duty <= 1_550_000:
            self.claw_duty = 1_550_000
        self.claw_servo.duty_ns(self.claw_duty)

    def close_claw(self):  # Close claw
        self.claw_duty += 10_000
        if self.claw_duty >= 1_950_000:
            self.claw_duty = 1_950_000
        self.claw_servo.duty_ns(self.claw_duty)

    def raise_arm(self):  # Raise arm
        self.arm_duty -= 20_000
        if self.arm_duty <= 700_000:
            self.arm_duty = 700_000
        self.arm_servo.duty_ns(self.arm_duty)


# Example usage
if __name__ == "__main__":
    from utime import sleep
    ad = ArmDrive(12, 13)
    for _ in range(20):
        ad.close_claw()
        sleep(0.1)
    for _ in range(20):
        ad.open_claw()
        sleep(0.1)
    for _ in range(20):
        ad.lower_arm()
        sleep(0.1)
    for _ in range(20):
        ad.raise_arm()
        sleep(0.1)

