#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class BatteryControllerNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("Battery_controller") # MODIFY NAME
        
        self.battery_status = 100

        self.timer = self.create_timer(2.0, self.battery_check)

        self.get_logger().info("System Battery Node Active")

    def battery_check(self):
        self.get_logger().info(f"The current battery level is {self.battery_status}%")
        self.battery_status -=5

        if self.battery_status <=0:
            self.battery_status = 100


def main(args=None):
    rclpy.init(args=args)
    node = BatteryControllerNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()