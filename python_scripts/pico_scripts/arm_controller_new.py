from machine import Pin, PWM, Timer
from time import sleep


# CONSTANTS
CLAW_OPEN = 1_800_000  # nano sec
CLAW_CLOSE = 2_550_000
SHOULDER_UP = 1_500_000
SHOULDER_MID = 1_000_000
SHOULDER_DOWN = 500_000
PULSE_WIDTH_INC_STEP = 5_000  # servo speed


class ArmController:
    # TODO: timer for setting target positions
    def __init__(self, claw_pin, shoulder_a_pin, shoulder_b_pin):
        self.claw = PWM(Pin(claw_pin))
        self.shoulder_a = PWM(Pin(shoulder_a_pin))
        self.shoulder_b = PWM(Pin(shoulder_b_pin))
        self.claw.freq(50)
        self.shoulder_a.freq(50)
        self.shoulder_b.freq(50)
        # Set to neutral
        self.claw.duty_ns(CLAW_OPEN)
        self.shoulder_a.duty_ns(SHOULDER_UP)
        self.shoulder_b.duty_ns(SHOULDER_UP)
        # Variables
        self.pulse_width_claw = CLAW_OPEN
        self.pulse_width_shoa = SHOULDER_UP
        self.pulse_width_shob = SHOULDER_UP
        self.target_claw = CLAW_OPEN
        self.target_shoa = SHOULDER_UP
        self.target_shob = SHOULDER_UP
        self.is_target_reached = True
        # Set joint pos timer
        self.joints_set_timer = Timer(
            freq=25,
            mode=Timer.PERIODIC,
            callback=self.manipulate_joints,
        )

    def set_joint_positions(
        self,
        target_claw=CLAW_OPEN,
        target_shoa=SHOULDER_UP,
    ):
        self.target_claw = target_claw
        self.target_shoa = target_shoa
        self.target_shob = SHOULDER_UP - (target_shoa - SHOULDER_UP)
        self.is_target_reached = False

    def manipulate_joints(self, timer):
        diff_claw = self.target_claw - self.pulse_width_claw
        diff_shoa = self.target_shoa - self.pulse_width_shoa
        self.pulse_width_claw += max(
            min(diff_claw, PULSE_WIDTH_INC_STEP), -PULSE_WIDTH_INC_STEP
        )
        self.pulse_width_shoa += max(
            min(diff_shoa, PULSE_WIDTH_INC_STEP), -PULSE_WIDTH_INC_STEP
        )
        self.pulse_width_shob -= max(
            min(diff_shoa, PULSE_WIDTH_INC_STEP), -PULSE_WIDTH_INC_STEP
        )
        self.claw.duty_ns(self.pulse_width_claw)
        self.shoulder_a.duty_ns(self.pulse_width_shoa)
        self.shoulder_b.duty_ns(self.pulse_width_shob)
        # Check target reached or not
        if all(
            [
                self.pulse_width_claw == self.target_claw,
                self.pulse_width_shoa == self.target_shoa,
                self.pulse_width_shob == self.target_shob,
            ]
        ):
            self.is_target_reached = True
        else:
            self.is_target_reached = False


# Example usage
if __name__ == "__main__":
    from utime import sleep

    print("To ease the shoulder servos, please lift the arm upright!")
    # SAFETY CHECK
    is_up = "n"
    while is_up != "y":
        print("Please lift arms.")
        is_up = input("Are the arms lifted upright? (y/n)")

    # SETUP
    ac = ArmController(15, 13, 14)
    sleep(2)
    # Set arm configs
    ac.set_joint_positions(CLAW_OPEN, SHOULDER_DOWN)  # lower arms to ball
    while ac.is_target_reached is False:
        sleep(0.1)
    print(f"arm reached position: {ac.pulse_width_claw}, {ac.pulse_width_shoa}")

    ac.set_joint_positions(CLAW_CLOSE, SHOULDER_DOWN)  # close claw, grab ball
    while ac.is_target_reached is False:
        sleep(0.1)
    print(f"arm reached position: {ac.pulse_width_claw}, {ac.pulse_width_shoa}")

    ac.set_joint_positions(CLAW_CLOSE, SHOULDER_UP)  # raise arms
    while ac.is_target_reached is False:
        sleep(0.1)
    print(f"arm reached position: {ac.pulse_width_claw}, {ac.pulse_width_shoa}")

    ac.set_joint_positions(CLAW_CLOSE, SHOULDER_MID)  # lower arms to bucket
    while ac.is_target_reached is False:
        sleep(0.1)
    print(f"arm reached position: {ac.pulse_width_claw}, {ac.pulse_width_shoa}")

    ac.set_joint_positions(CLAW_OPEN, SHOULDER_MID)  # open claw, drop ball
    while ac.is_target_reached is False:
        sleep(0.1)
    print(f"arm reached position: {ac.pulse_width_claw}, {ac.pulse_width_shoa}")

    ac.set_joint_positions(CLAW_OPEN, SHOULDER_UP)  # raise arm
    while ac.is_target_reached is False:
        sleep(0.1)
    print(f"arm reached position: {ac.pulse_width_claw}, {ac.pulse_width_shoa}")
