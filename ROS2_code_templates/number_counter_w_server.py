
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from my_robot_interfaces.srv import ResetCounter

class NumberCounterNode(Node):
    def __init__(self):
        super().__init__('number_counter_w_server')
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
        
        """
        Create a service server. We need to specify the following parameters:
            - Interface we are using for the service. This needs to be the same for the server and client (ResetCounter)
            - The name of the service that the server and client will be using (similar to a topic name)
            - THe callback to the function that triggers when a client connects
        """
        self.reset_counter = self.create_service(ResetCounter, 
                                                 "reset_counter", 
                                                 self.callback_reset_counter)

        self.get_logger().info("Hello world")

    """
    Since we are subscibing to an interface, we need to add a parameter for the function that can 'capture' the data coming in.
    In the function definition, a parameter (msg) will store incoming data (it can technically be called anything else, but msg is a logical name)
    It is also good practice to specifiy the interface type (Int64), not required 
    """
    def callback_number(self, msg : Int64):
        self.counter += msg.data # Adds the value that the publisher sends to a counter (since our publisher is sending a number)
        self.get_logger().info(f"Counter value is at: {self.counter}") # Display the value of the counter in the terminal


    def callback_reset_counter_min(self, request : ResetCounter.Request, responce : ResetCounter.Response):

        self.counter = request.reset_value # Set the counter equal to the reset value requested by the client
        self.get_logger().info(f"Reset counter to {self.counter}") # Debug info from the node 
        responce.success = True # Format a return messgae to the client holding a boolean that the counter was reset 
        responce.message = "Succuess" # Send back a responce message
        return responce # Send the message to the client


    def callback_reset_counter(self, request : ResetCounter.Request, responce : ResetCounter.Response):
        
        # since ROS knows that 'request' is an interface from the server (since it's in the contructor), we can check it against the reset_value field of our interfaec (ResetCounter)
        if request.reset_value < 0: # If a negative value is requested
            responce.success = False # Return False, do not reset counter
            responce.message = "Cannot reset counter to a negative value" # Structure a responce string letting client know the issue
        
        elif request.reset_value > self.counter: # if a value greater then the current count is requested 
            responce.success = False
            responce.message = "Reset value has to be lower then current value"
        
        else: # Otherwise fulfill the request
            self.counter = request.reset_value
            self.get_logger().info(f"Reset counter to {self.counter}") # Debug info from the node 
            responce.success = True # Format a return messgae to the client holding a boolean that the counter was reset 
            responce.message = "Succuess" # Send back a responce message

        return responce # Send back the responce with the 'success' and 'message' fields filled out


def main(args=None):

    rclpy.init(args=args)            # Initialize the ROS2 Python client library
    node = NumberCounterNode()            # Create an instance of the custom node (registers it with ROS2)
    rclpy.spin(node)                 # Keep the node running and responsive to callbacks (e.g., subscriptions, timers)
    rclpy.shutdown()                 # Cleanly shut down the ROS2 client library after the node is done

if __name__ == '__main__':
    main()