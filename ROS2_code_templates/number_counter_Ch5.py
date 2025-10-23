
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64

class NumberCounterNode(Node):
    def __init__(self):
        super().__init__('number_counter')
        self.counter = 0

        """
        Create a subscriber. We need to specify the following in the parameter:
            - Interface needs to be the same for both the publisher and subscriber(Int 64)
            - Name of the topic, again it needs to match the name of topic we are publishing to ("number")
            - callback function (self.callback_number)
            - Buffer depth size (10)
        """
        self.number_subscriber = self. create_subscription(
                                Int64, 
                                "number", 
                                self.callback_number, 
                                10)

        self.get_logger().info("Hello world")

    """
    Since we are subscibing to an interface, we need to add a parameter for the function that can 'capture' the data coming in.
    In the function definition, a parameter (msg) will store incoming data (it can technically be called anything else, but msg is a logical name)
    It is also good practice to specifiy the interface type (Int64), not required 
    """
    def callback_number(self, msg : Int64):
        self.counter += msg.data # Adds the value that the publisher sends to a counter (since our publisher is sending a number)
        self.get_logger().info(f"Counter value is at: {self.counter}") # Display the value of the counter in the terminal


def main(args=None):

    rclpy.init(args=args)            # Initialize the ROS2 Python client library
    node = NumberCounterNode()            # Create an instance of the custom node (registers it with ROS2)
    rclpy.spin(node)                 # Keep the node running and responsive to callbacks (e.g., subscriptions, timers)
    rclpy.shutdown()                 # Cleanly shut down the ROS2 client library after the node is done

if __name__ == '__main__':
    main()