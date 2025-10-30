
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from my_robot_interfaces.srv import ResetCounter # import the interface for the server

class MyCustomNode(Node):
    def __init__(self):
        super().__init__('reset_counter_client')

        """
        The service client has the following parameters:
            - The name of the interface (ResetClient)
            - The name of the service 
        These need to match what the server is using 
        """
        self.client = self.create_client(ResetCounter,
                                         "reset_counter")
        self.get_logger().info("Hello world")

    """
    This method triggers the call to the server. 
    The parameter is the value the function recieves and will pass on to the server (for reseting counter to THAT value)
    In practice - This is often triggered by a condition (i.e. battery level reads below a certain level)
    """
    def call_reset_counter(self, value):
        while not self.client.wait_for_service(1.0): # We wair for up to 1 second for the server to become available
            self.get_logger().warn("Waiting for service...") # Logger feedback while waiting for the server. The .warn fucntion makes the terminal text show up in red 
        
        request = ResetCounter.Request() # Holds the data for the request that will be sent to the server
        request.reset_value = value # The reset_value field (from the interface) holds the value passed to the function in the request
        """
        The line below does two important things:
            1) # This sends the request to the server asynchronously (which doesn't block other code from running)
            2) Creates an object called a Future which is a placeholder for the servers responce and has some built-in fucntionality important for service calls 
        """
        future = self.client.call_async(request) 
        future.add_done_callback(self.callback_reset_counter_responce) # This triggers the function in the parameter (from the Future object) when a responce is recieved 

    """
    When the server gives back a responce, the data is held in a variable (an object actually) and passed to this callback function from the previous one.
    """
    def callback_reset_counter_responce(self, future):
        response = future.result() # store the data recieved as a variable (result is a method of the Future object)
        self.get_logger().info(f"Success flag: {response.success}") # Shows T or F based on success field in the interface
        self.get_logger().info(f"Message: {response.message}") # Show the value of the message string from the interface

def main(args=None):

    rclpy.init(args=args)            # Initialize the ROS2 Python client library
    node = MyCustomNode()            # Create an instance of the custom node (registers it with ROS2)

    node.call_reset_counter(10)      # Call the function which will communicate with the server and pass in a value of 10 (to reset the count to)

    rclpy.spin(node)                 # Keep the node running and responsive to callbacks (e.g., subscriptions, timers)
    rclpy.shutdown()                 # Cleanly shut down the ROS2 client library after the node is done

if __name__ == '__main__':
    main()