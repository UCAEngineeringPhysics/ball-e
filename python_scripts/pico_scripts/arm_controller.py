from machine import Pin, PWM
from time import sleep

ARM_NEUTRAL = 1_650_000
CLAW_NEUTRAL = 1_800_000


class ArmDrive:
    def __init__(self, claw_pin, arm_pin):
        self.claw_servo = PWM(Pin(claw_pin))
        self.arm_servo = PWM(Pin(arm_pin))
        self.claw_servo.freq(50)
        self.arm_servo.freq(50)
        # Set initial positions
        self.set_neutral()

    def set_neutral(self):
        # Return to rest positions
        self.arm_servo.duty_ns(ARM_NEUTRAL)
        self.claw_servo.duty_ns(CLAW_NEUTRAL)
        self.arm_duty = ARM_NEUTRAL
        self.claw_duty = CLAW_NEUTRAL

    def lower_arm(self, dc_inc=0):  # Lower arm
        assert -50_000 <= dc_inc <= 50_000
        self.arm_duty += dc_inc
        if self.arm_duty >= 2_800_000:
            self.arm_duty = 2_800_000
        elif self.arm_duty <= 700_000:
            self.arm_duty = 700_000
        self.arm_servo.duty_ns(self.arm_duty)

    # def open_claw(self, dc_inc=0):  # Open claw
    #     assert -50_000 <= dc_inc <= 0
    #     self.claw_duty += dc_inc
    #     if self.claw_duty <= 1_550_000:
    #         self.claw_duty = 1_550_000
    #     self.claw_servo.duty_ns(self.claw_duty)

    def close_claw(self, dc_inc=0):  # Close claw
        assert -50_000 <= dc_inc <= 50_000
        self.claw_duty += dc_inc
        if self.claw_duty >= 1_950_000:
            self.claw_duty = 1_950_000
        elif self.claw_duty <= 1_550_000:
            self.claw_duty = 1_550_000
        self.claw_servo.duty_ns(self.claw_duty)

    # def raise_arm(self, dc_inc=0):  # Raise arm
    #     assert -50_000 <= dc_inc <= 0
    #     self.arm_duty += dc_inc
    #     if self.arm_duty <= 700_000:
    #         self.arm_duty = 700_000
    #     self.arm_servo.duty_ns(self.arm_duty)


# Example usage
if __name__ == "__main__":
    from utime import sleep

    sleep(1)
    ad = ArmDrive(12, 13)
    for _ in range(20):
        ad.close_claw(10_000)
        sleep(0.1)
        print(f"Closing claw duty cycle: {ad.claw_duty}")
    for _ in range(20):
        ad.close_claw(-20_000)
        sleep(0.1)
        print(f"Opening claw duty cycle: {ad.claw_duty}")

    ad.set_neutral()
    sleep(1)
    print("arm set to neutral")

    for _ in range(20):
        ad.lower_arm(20_000)
        sleep(0.1)
        print(f"Lowering arm duty cycle: {ad.claw_duty}")
    for _ in range(20):
        ad.lower_arm(-20_000)
        sleep(0.1)
        print(f"Lifting arm duty cycle: {ad.claw_duty}")
