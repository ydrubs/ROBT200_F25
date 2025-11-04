#!/usr/bin/env python3
"""
Battery LED Status Indicator — Exploration Template

Goal:
    Change the color of an LED based on the robot’s battery level.

Description:
    This ROS 2 node subscribes to the robot's battery voltage topic
    and monitors the current battery level.

    If the voltage is above 7.5V (7500 mV), it turns LED 1 green
    to indicate a healthy battery.
    If the voltage is 7.5V or lower, it turns LED 1 red
    to indicate that charging is recommended soon.

    The node also logs the current voltage to the terminal using the ROS logger.

Student Task:
    1. Fill in the missing topic names and message imports.
    2. Adjust the threshold or LED colors if desired.
    3. Run and observe LED behavior on the MentorPi.
"""

import rclpy
from rclpy.node import Node

# TODO: Fill in the correct message imports based on your CLI exploration
# Hint: One message type handles battery voltage (simple integer value)
#       and two message types handle LED control (a container and a single LED entry)
from __________________________ import __________________________       # Battery voltage message
from __________________________ import __________________________, __________________________  # LED control messages


class BatteryStatus(Node):
    def __init__(self):
        # TODO: Give your node a descriptive name
        super().__init__('__________________________')

        # TODO: Create a subscriber for the battery voltage topic
        # Hint: The topic provides voltage in millivolts (UInt16)
        self.create_subscription(
            __________________________,     # Message type
            '__________________________',   # Topic name
            self.battery_callback,          # Callback function
            10                              # QoS depth
        )

        # TODO: Create a publisher for controlling the LED color
        self.publisher = self.create_publisher(
            __________________________,     # Message type (container for LEDs)
            '__________________________',   # Topic name
            10
        )

    def battery_callback(self, msg):
        # TODO: Retrieve the voltage value from the message
        volts = msg.data

        # TODO: Define your threshold (7500 mV by default)
        if msg.data > ________:
            self.get_logger().info(f"Battery is good, {volts}")

            # TODO: Build message for green LED
            rgb_msg = __________________________()
            rgb_state = __________________________()
            rgb_state.index = 1    # LED index

            # LED color: GREEN
            rgb_state.red = 0
            rgb_state.green = 255
            rgb_state.blue = 0

            rgb_msg.states.append(rgb_state)
            self.publisher.publish(rgb_msg)

        else:
            self.get_logger().info(f"Battery needs charging soon")

            # TODO: Build message for red LED
            rgb_msg = __________________________()
            rgb_state = __________________________()
            rgb_state.index = 1    # LED index

            # LED color: RED
            rgb_state.red = 255
            rgb_state.green = 0
            rgb_state.blue = 0

            rgb_msg.states.append(rgb_state)
            self.publisher.publish(rgb_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BatteryStatus()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
