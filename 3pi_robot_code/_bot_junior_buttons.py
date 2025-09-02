from pololu_3pi_2040_robot import robot
import time

button_a = robot.ButtonA()
button_b = robot.ButtonB()

motors = robot.Motors()
speed = motors.MAX_SPEED # same as -6000 to 6000

while True:
    if button_a.check():
        time.sleep(0.5)
        motors.set_speeds(speed/3, speed/3)

    if button_b.check():
        motors.off()

