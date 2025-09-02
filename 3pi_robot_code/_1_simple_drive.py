from pololu_3pi_2040_robot import robot
import time

motors = robot.Motors()
speed = motors.MAX_SPEED # same as -6000 to 6000

while True:

    motors.set_speeds(speed, speed)
    time.sleep(1)

    motors.set_speeds(speed, -speed)
    time.sleep(2)

    motors.set_speeds(-speed, speed)
    time.sleep(1)

    motors.set_speeds(speed/4, speed)
    time.sleep(2)

    motors.off()


    #Make your robot drive in a sqaure