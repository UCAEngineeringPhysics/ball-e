from machine import Pin, PWM


class MotorDriver:
    def __init__(self, pwm_id, ina_id, inb_id, ):
        self.pwm_pin = PWM(Pin(pwm_id))
        self.pwm_pin.freq(2000)
        self.ina_pin = Pin(ina_id, Pin.OUT)
        self.inb_pin = Pin(inb_id, Pin.OUT)
        self.disable()

    def stop(self):
        self.pwm_pin.duty_u16(0)

    def forward(self, duty):
        assert 0 <= duty <= 1  # make sure dutycycle in [0, 1]
        self.ina_pin.on()
        self.inb_pin.off()
        self.pwm_pin.duty_u16(int(65535 * duty))

    def backward(self, duty):
        assert 0 <= duty <= 1  # make sure dutycycle in [0, 1]
        self.ina_pin.off()
        self.inb_pin.on()
        self.pwm_pin.duty_u16(int(65535 * duty))

    def disable(self):
        self.ina_pin.off()
        self.inb_pin.off()


if __name__ == "__main__":
    from time import sleep

    # SETUP
    # md = MotorDriver(pwm_id=16, ina_id=18, inb_id=17)  # right
    md = MotorDriver(pwm_id=21, ina_id=19, inb_id=20)  # left

    # LOOP
    for i in range(100):
        md.forward((i + 1) / 100)
        print(f"f, dc: {i}%")
        sleep(4 / 100)  # 4 seconds to ramp up
    for i in reversed(range(100)):
        md.forward((i + 1) / 100)
        print(f"f, dc: {i}%")
        sleep(4 / 100)  # 4 seconds to ramp down
    # Backwardly ramp up and down
    for i in range(100):
        md.backward((i + 1) / 100)
        print(f"b, dc: {i}%")
        sleep(4 / 100)  # 4 seconds to ramp up
    for i in reversed(range(100)):
        md.backward((i + 1) / 100)
        print(f"b, dc: {i}%")
        sleep(4 / 100)  # 4 seconds to ramp down

    # TERMINATE
    md.disable()
    print("Motor driver disabled")