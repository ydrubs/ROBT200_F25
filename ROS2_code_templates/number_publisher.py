#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int64 # Import the interface 

class NumberPublisherNode(Node):
    def __init__(self):
        super().__init__("number_publisher") # name of the node

        #Add an attribute to keep count
        self.number = 2
        
        """
        Create the number publisher with the following parameters:
            Topic interface: We’ll use Int64 from the example_interfaces package.
            Topic name: As defined previously, this is number.
            Queue size: If the messages are published too fast and subscribers can’t keep up, messages will be buffered (up to 10 here)
        """
        self.number_publisher = self.create_publisher(Int64, "number", 10)

        # Add an an attribute to trigger a callback based on a timer
        self.number_timer = self.create_timer(1.0, self.publish_number)

        # Add a log to show some feedback when the node starts
        self.get_logger().info("Number Publisher Started.")

    def publish_number(self):
        msg = Int64() # This is the variable that holds the interface that will be sent
        msg.data = self.number # The interface is stored as ‘data’ and its value is the value of our self.number attribute. We know the field is called ‘data’ because if we put in ros2 interface show example_interfaces/msg/Int64 in the CLI we see that field. 
        self.number_publisher.publish(msg) # This command is what actually publishes the message 

def main(args=None):
    rclpy.init(args=args) # initialize the ROS2 client library
    node = NumberPublisherNode() # Create an object from the class
    rclpy.spin(node) # Keep the node running
    rclpy.shutdown() # Allows node to be shutdown with ctrl + C or other methods


 

if __name__ == "__main__":
    main()