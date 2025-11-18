from machine import Pin, PWM
from time import sleep

SHOULDER_NEUTRAL = 1_650_000
CLAW_NEUTRAL = 1_800_000


class ArmController:
    def __init__(self, claw_pin, arm_pin):
        self.claw_servo = PWM(Pin(claw_pin))
        self.shoulder_servo = PWM(Pin(arm_pin))
        self.claw_servo.freq(50)
        self.shoulder_servo.freq(50)
        # Set initial positions
        self.set_neutral()

    def set_neutral(self):
        # Return to rest positions
        self.shoulder_servo.duty_ns(SHOULDER_NEUTRAL)
        self.claw_servo.duty_ns(CLAW_NEUTRAL)
        self.shoulder_duty = SHOULDER_NEUTRAL
        self.claw_duty = CLAW_NEUTRAL

    def lower_claw(self, dc_inc=0):  # Lower arm
        assert -50_000 <= dc_inc <= 50_000
        self.shoulder_duty += dc_inc
        if self.shoulder_duty >= 2_800_000:
            self.shoulder_duty = 2_800_000
        elif self.shoulder_duty <= 700_000:
            self.shoulder_duty = 700_000
        self.shoulder_servo.duty_ns(self.shoulder_duty)

    def close_claw(self, dc_inc=0):  # Close claw
        assert -50_000 <= dc_inc <= 50_000
        self.claw_duty += dc_inc
        if self.claw_duty >= 1_950_000:
            self.claw_duty = 1_950_000
        elif self.claw_duty <= 1_550_000:
            self.claw_duty = 1_550_000
        self.claw_servo.duty_ns(self.claw_duty)


# Example usage
if __name__ == "__main__":
    from utime import sleep

    sleep(1)
    ac = ArmController(12, 13)
    for _ in range(20):
        ac.close_claw(10_000)
        sleep(0.1)
        print(f"Closing claw duty cycle: {ac.claw_duty}")
    for _ in range(20):
        ac.close_claw(-20_000)
        sleep(0.1)
        print(f"Opening claw duty cycle: {ac.claw_duty}")

    ac.set_neutral()
    sleep(1)
    print("arm set to neutral")

    for _ in range(20):
        ac.lower_claw(20_000)
        sleep(0.1)
        print(f"Lowering claw duty cycle: {ac.shoulder_duty}")
    for _ in range(20):
        ac.lower_claw(-20_000)
        sleep(0.1)
        print(f"Lifting claw duty cycle: {ac.shoulder_duty}")
