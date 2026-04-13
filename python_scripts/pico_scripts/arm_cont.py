from machine import Pin, PWM
from time import sleep

SHOULDER_NEUTRAL = 1_400_000  # Starting position for both arm servos
CLAW_NEUTRAL = 1_950_000  # Starting position for claw

class ArmController:
    def __init__(self, claw_pin, arm_pin_left, arm_pin_right):
        # Claw on pin 12
        self.claw_servo = PWM(Pin(claw_pin))
        # Left arm servo on pin 13 (servo_1)
        self.shoulder_servo_left = PWM(Pin(arm_pin_left))
        # Right arm servo on pin 14 (servo_2)
        self.shoulder_servo_right = PWM(Pin(arm_pin_right))
        
        self.claw_servo.freq(50)
        self.shoulder_servo_left.freq(50)
        self.shoulder_servo_right.freq(50)
        
        # Set initial positions
        self.shoulder_duty_left = SHOULDER_NEUTRAL
        self.shoulder_duty_right = SHOULDER_NEUTRAL
        self.claw_duty = CLAW_NEUTRAL
        
        self.set_neutral()
        
    def set_neutral(self):
        """Reset all servos to neutral position"""
        self.shoulder_duty_left = SHOULDER_NEUTRAL
        self.shoulder_duty_right = SHOULDER_NEUTRAL
        self.claw_duty = CLAW_NEUTRAL

        self.shoulder_servo_left.duty_ns(self.shoulder_duty_left)
        self.shoulder_servo_right.duty_ns(self.shoulder_duty_right)
        self.claw_servo.duty_ns(self.claw_duty)
        

    def lower_claw(self, dc_inc=0):
        """
        Control arm height (shoulder servos)
        Positive dc_inc: LOWER arm (increase left, decrease right)
        Negative dc_inc: RAISE arm (decrease left, increase right)
        """
        assert -50_000 <= dc_inc <= 50_000
        
        # When dc_inc is NEGATIVE (lift button): decrease left, increase right → RAISE
        # When dc_inc is POSITIVE (lower button): increase left, decrease right → LOWER
        self.shoulder_duty_left += dc_inc
        self.shoulder_duty_right -= dc_inc

        # Limit left servo
        if self.shoulder_duty_left >= 2_400_000:
            self.shoulder_duty_left = 2_400_000
        elif self.shoulder_duty_left <= 700_000:
            self.shoulder_duty_left = 700_000
            
        # Limit right servo
        if self.shoulder_duty_right >= 2_400_000:
            self.shoulder_duty_right = 2_400_000
        elif self.shoulder_duty_right <= 700_000:
            self.shoulder_duty_right = 700_000

        self.shoulder_servo_left.duty_ns(self.shoulder_duty_left)
        self.shoulder_servo_right.duty_ns(self.shoulder_duty_right)


    def close_claw(self, dc_inc=0):
        """
        Control claw open/close
        Positive dc_inc: OPEN claw (increase duty)
        Negative dc_inc: CLOSE claw (decrease duty)
        """
        assert -50_000 <= dc_inc <= 50_000
        self.claw_duty += dc_inc
        
        # Limits for claw
        if self.claw_duty >= 2_400_000:  # Fully open
            self.claw_duty = 2_400_000
        elif self.claw_duty <= 1_500_000:  # Fully closed
            self.claw_duty = 1_500_000
            
        self.claw_servo.duty_ns(self.claw_duty)