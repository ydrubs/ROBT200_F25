from pololu_3pi_2040_robot import robot
import time
from random import randint

rgb_leds = robot.RGBLEDs()
rgb_leds.set_brightness(31)


while True:

    rgb_leds.set(0, [randint(0,255),0,0])
    rgb_leds.set(1, [0, 255, 0])
    rgb_leds.set(2, [0, 0, 255])
    rgb_leds.set(3, [100, 100, 200])
    rgb_leds.set(4, [150, 45, 10])

    rgb_leds.show()
    time.sleep(1)

    rgb_leds.set(0, [255, 0, 0])
    rgb_leds.set(1, [0, 255, 0])
    rgb_leds.set(2, [0, 0, 255])
    rgb_leds.set(3, [100, 100, 200])
    rgb_leds.set(4, [150, 45, 10])

    rgb_leds.show()
    time.sleep(1)
