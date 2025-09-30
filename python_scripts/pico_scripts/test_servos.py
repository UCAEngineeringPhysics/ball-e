from machine import Pin, PWM, reset
from time import sleep

# SAFETY CHECK

# SETUP
servo_0 = PWM(Pin(0))
servo_0.freq(50)
servo_0.duty_ns(1800000)
servo_1 = PWM(Pin(1))
servo_1.freq(50)
servo_1.duty_ns(1800000)

# LOOP
try:
    while True:    
        print("<<<--- <<-- <- W -> -->> --->>>\n")
        for i in range(1300000, 2200000, 10000):
            servo_0.duty_ns(i)
            servo_1.duty_ns(i)
            print(i)
            sleep(0.2)
        print("--->>> -->> -> W <- <<-- <<<---- \n")
        for i in reversed(range(1300000, 2200000, 10000)):
            servo_0.duty_ns(i)
            servo_1.duty_ns(i)
            print(i)
            sleep(0.2)
        servo_0.duty_ns(1800000)
        servo_1.duty_ns(1800000)
        sleep(1)
        servo_0.duty_ns(2200000)
        servo_1.duty_ns(2200000)
        sleep(1)
        servo_0.duty_ns(1800000)
        servo_1.duty_ns(1800000)
        sleep(1)
        servo_0.duty_ns(1300000)
        servo_1.duty_ns(1300000)
        sleep(1)
        servo_0.duty_ns(1800000)
        servo_1.duty_ns(1800000)
        sleep()
    except KeyboardInterrupt:
        servo_0.deinit()
        servo_1.deinit()
        reset()
