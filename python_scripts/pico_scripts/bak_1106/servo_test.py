from machine import Pin, PWM, reset
from time import sleep

# SAFETY CHECK

# SETUP

# SERVO 0 KEY INFO (CLAW)                                                                                               #
# Max Duty: (2100000) Fully closed claw
# Min Duty: (1550000) Open position before grabbing the ball
# Mid Duty: (1775000) 45 degrees higher duty closes and lower duty opens
# Grab Duty: (1725000) Duty to be set after claw is in position to grab. This will ensure good hold on the ball
servo_0 = PWM(Pin(12))
servo_0.freq(50)
servo_0.duty_ns(2100000)

# Servo 1 key info (ARM)
# Min duty (400000) This will be in the compact configuration to fit the 2ft x 2ft size constraint for now          #
# Max duty (2225000) This is too low and will likely hit the ground.                                                #
# Mid duty (1650000) This is going to be the starting position
# Grab Duty: (2190000) Grab angle
# Bucket Drop Duty: () To be found 
servo_1 = PWM(Pin(13))
servo_1.freq(50)
servo_1.duty_ns(400000)


# LOOP


# while True:
#     #Servo 1
#     print("<<<--- <<-- <- W -> -->> --->>>\n")
#     for i in range(1650000, 2600000, 50000):
#         #servo_0.duty_ns(i)
#         servo_1.duty_ns(i)
#         print(i)
#         sleep(0.2)
#     print("--->>> -->> -> W <- <<-- <<<---- \n")
#     for i in reversed(range(2600000, 700000, 50000)):
#         #servo_0.duty_ns(i)
#         servo_1.duty_ns(i)
#         print(i)
#         sleep(0.2)
#     # servo 1
#     for i in range(1800000, 2300000, 50000):
#         servo_0.duty_ns(i)
#         #servo_1.duty_ns(i)
#         print(i)
#         sleep(0.2)
#     print("--->>> -->> -> W <- <<-- <<<---- \n")
#     for i in reversed(range(2300000, 1550000, 10000)):
#         servo_0.duty_ns(i)
#         #servo_1.duty_ns(i)
#         print(i)
#         sleep(0.2)
#     servo_0.duty_ns(1800000)
#     servo_1.duty_ns(1800000)
#     sleep(1)
#     servo_0.duty_ns(2200000)
#     servo_1.duty_ns(2200000)
#     sleep(1)
#     servo_0.duty_ns(1800000)
#     servo_1.duty_ns(1800000)
#     sleep(1)
#     servo_0.duty_ns(1300000)
#     servo_1.duty_ns(1300000)
#     sleep(1)
#     servo_0.duty_ns(1800000)
#     servo_1.duty_ns(1800000)
#     sleep()