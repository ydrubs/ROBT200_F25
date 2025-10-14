"""Minimal ROS2 Node Script"""

"""
Ensures the script is treated as a Python 3 executable when run from the terminal.
"""
#!/usr/bin/env python3

"""
Required libraries to use ROS2 Python API and define custom nodes.
"""
import rclpy
from rclpy.node import Node

"""
All ROS2 nodes are defined as classes that inherit from Node. 
This allows the node to use publishing, subscribing, and logging features.
"""
class MyCustomNode(Node):
    def __init__(self):
        """
        Allows the node to appear using ROS2 CLI tools such as 'ros2 node list' with the name 'my_node_name'
        """
        super().__init__('my_node_name_w_timer')

        self.counter = 0 # Counter to keep track of some values 

        """
        A callback function that triggers a function called print_hello every 1.0 seconds
        """
        self.timer = self.create_timer(1.0, self.print_hello)

        """
        A logger that allows the node to display a message when started 
        """
        self.get_logger().info("Hello world")

    """
    The function is triggered by the self.timer command every second
    """
    def print_hello(self):
        self.get_logger().info(f"The current count is {self.counter}, yay!")
        self.counter += 1

"""
This function is where program execution starts

Nodes are designed to 'spin' - 
    they stay active and responsive in the background, waiting for events like messages or service requests 
    — all without interfering with other nodes or system operations.
"""
def main(args=None):

    rclpy.init(args=args)            # Initialize the ROS2 Python client library
    node = MyCustomNode()            # Create an instance of the custom node (registers it with ROS2)
    rclpy.spin(node)                 # Keep the node running and responsive to callbacks (e.g., subscriptions, timers)
    rclpy.shutdown()                 # Cleanly shut down the ROS2 client library after the node is done


if __name__ == '__main__':
    main()