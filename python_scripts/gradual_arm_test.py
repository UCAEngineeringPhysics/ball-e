"""
Simple, gradual arm controller for bring-up testing on Pico.

Why this file exists:
- Keep logic easier to review than ac2.py while we debug motion behavior.
- Provide direct, predictable shoulder/claw steps.
- Avoid heavy command scaling so behavior is easier to tune by observation.

This module is intentionally minimal and is safe to test standalone.
"""

from machine import Pin, PWM
from utime import sleep_ms


# Pin mapping follows your current setup.
DEFAULT_CLAW_PIN = 15
DEFAULT_SHOULDER_A_PIN = 13
DEFAULT_SHOULDER_B_PIN = 14


# Conservative defaults; update as needed for your hardware.
SHOULDER_A_NEUTRAL = 1_300_000
SHOULDER_B_NEUTRAL = 1_600_000
CLAW_NEUTRAL = 1_800_000

SHOULDER_A_MIN = 1_050_000
SHOULDER_A_MAX = 1_700_000
SHOULDER_B_MIN = 1_200_000
SHOULDER_B_MAX = 1_900_000
CLAW_MIN = 1_550_000
CLAW_MAX = 2_200_000


class GradualArmController:
    """Bare-bones arm controller with explicit step-based movement."""

    def __init__(self, claw_pin=DEFAULT_CLAW_PIN, shoulder_a_pin=DEFAULT_SHOULDER_A_PIN, shoulder_b_pin=DEFAULT_SHOULDER_B_PIN):
        self.claw_servo = PWM(Pin(claw_pin))
        self.shoulder_servo_a = PWM(Pin(shoulder_a_pin))
        self.shoulder_servo_b = PWM(Pin(shoulder_b_pin))

        self.claw_servo.freq(50)
        self.shoulder_servo_a.freq(50)
        self.shoulder_servo_b.freq(50)

        self.shoulder_duty_a = SHOULDER_A_NEUTRAL
        self.shoulder_duty_b = SHOULDER_B_NEUTRAL
        self.claw_duty = CLAW_NEUTRAL
        self._write_now()

    @staticmethod
    def _clamp(val, low, high):
        if val < low:
            return low
        if val > high:
            return high
        return val

    def _write_now(self):
        self.shoulder_servo_a.duty_ns(self.shoulder_duty_a)
        self.shoulder_servo_b.duty_ns(self.shoulder_duty_b)
        self.claw_servo.duty_ns(self.claw_duty)

    def set_neutral(self):
        """Immediate neutral set (single write)."""
        self.shoulder_duty_a = SHOULDER_A_NEUTRAL
        self.shoulder_duty_b = SHOULDER_B_NEUTRAL
        self.claw_duty = CLAW_NEUTRAL
        self._write_now()

    def step_shoulder(self, step_ns):
        """
        Step shoulders together.
        Positive step lowers arm, negative step raises arm.
        """
        self.shoulder_duty_a = self._clamp(self.shoulder_duty_a + int(step_ns), SHOULDER_A_MIN, SHOULDER_A_MAX)
        self.shoulder_duty_b = self._clamp(self.shoulder_duty_b - int(step_ns), SHOULDER_B_MIN, SHOULDER_B_MAX)
        self._write_now()

    def step_claw(self, step_ns):
        """Step claw. Positive closes, negative opens."""
        self.claw_duty = self._clamp(self.claw_duty + int(step_ns), CLAW_MIN, CLAW_MAX)
        self._write_now()

    def lower(self, step_ns=6_000, steps=20, delay_ms=20):
        for _ in range(max(0, int(steps))):
            self.step_shoulder(abs(int(step_ns)))
            sleep_ms(max(0, int(delay_ms)))

    def raise_arm(self, step_ns=8_000, steps=20, delay_ms=20):
        for _ in range(max(0, int(steps))):
            self.step_shoulder(-abs(int(step_ns)))
            sleep_ms(max(0, int(delay_ms)))

    def close(self, step_ns=8_000, steps=15, delay_ms=20):
        for _ in range(max(0, int(steps))):
            self.step_claw(abs(int(step_ns)))
            sleep_ms(max(0, int(delay_ms)))

    def open(self, step_ns=8_000, steps=15, delay_ms=20):
        for _ in range(max(0, int(steps))):
            self.step_claw(-abs(int(step_ns)))
            sleep_ms(max(0, int(delay_ms)))

    def move_neutral_gradual(self, shoulder_step_ns=6_000, claw_step_ns=6_000, delay_ms=20, max_iters=200):
        """
        Gradually return all joints to neutral. Useful to avoid snap-to-neutral.
        """
        iters = 0
        while iters < max_iters:
            iters += 1
            done = True

            if self.shoulder_duty_a < SHOULDER_A_NEUTRAL:
                self.shoulder_duty_a = min(self.shoulder_duty_a + shoulder_step_ns, SHOULDER_A_NEUTRAL)
                done = False
            elif self.shoulder_duty_a > SHOULDER_A_NEUTRAL:
                self.shoulder_duty_a = max(self.shoulder_duty_a - shoulder_step_ns, SHOULDER_A_NEUTRAL)
                done = False

            if self.shoulder_duty_b < SHOULDER_B_NEUTRAL:
                self.shoulder_duty_b = min(self.shoulder_duty_b + shoulder_step_ns, SHOULDER_B_NEUTRAL)
                done = False
            elif self.shoulder_duty_b > SHOULDER_B_NEUTRAL:
                self.shoulder_duty_b = max(self.shoulder_duty_b - shoulder_step_ns, SHOULDER_B_NEUTRAL)
                done = False

            if self.claw_duty < CLAW_NEUTRAL:
                self.claw_duty = min(self.claw_duty + claw_step_ns, CLAW_NEUTRAL)
                done = False
            elif self.claw_duty > CLAW_NEUTRAL:
                self.claw_duty = max(self.claw_duty - claw_step_ns, CLAW_NEUTRAL)
                done = False

            self._write_now()
            if done:
                break
            sleep_ms(max(0, int(delay_ms)))


if __name__ == "__main__":
    # Minimal self-test sequence:
    # neutral -> lower -> close -> open -> raise -> gradual neutral
    arm = GradualArmController()
    sleep_ms(500)
    arm.set_neutral()
    sleep_ms(400)
    arm.lower(step_ns=6_000, steps=18, delay_ms=20)
    sleep_ms(250)
    arm.close(step_ns=8_000, steps=12, delay_ms=20)
    sleep_ms(250)
    arm.open(step_ns=8_000, steps=12, delay_ms=20)
    sleep_ms(250)
    arm.raise_arm(step_ns=8_000, steps=18, delay_ms=20)
    sleep_ms(250)
    arm.move_neutral_gradual(shoulder_step_ns=5_000, claw_step_ns=5_000, delay_ms=20)