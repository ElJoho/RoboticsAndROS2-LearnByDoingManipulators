import rclpy  # Import the ROS 2 Python client library
from rclpy.node import Node  # Import the Node base class to create ROS 2 nodes
from arduinobot_msgs.srv import AddTwoInts  # Import the service interface (request: a,b; response: sum)
import sys  # Import sys to access command-line arguments

class SimpleServiceClient(Node):  # Define a ROS 2 node class that acts as a service client
    def __init__(self, a , b):  # Constructor receives two numbers to send in the service request
        super().__init__("simple_service_client")  # Initialize the base Node with the name "simple_service_client"

        self.client_ = self.create_client(  # Create a client handle for a specific service type and name
            AddTwoInts,  # The service type/interface used by the client
            "add_two_ints"  # The service name to connect to (must match the server)
        )

        while not self.client_.wait_for_service( timeout_sec=1.0 ):  # Keep waiting until the server becomes available
            self.get_logger().info("Service not available, waiting more ... ")  # Informative log while waiting

        self.req_ = AddTwoInts.Request()  # Create an instance of the request message for AddTwoInts
        self.req_.a = a  # Fill the first operand of the request
        self.req_.b = b  # Fill the second operand of the request
        self.future_ = self.client_.call_async(self.req_)  # Call the service asynchronously; returns a Future
        self.future_.add_done_callback(self.responseCallback)  # Register a callback to run when the Future completes

    def responseCallback( self, future):  # Callback executed when the service response arrives
        self.get_logger().info("Service Response %d" % future.result().sum)  # Log the sum contained in the response

def main():  # Entry point of the script
    rclpy.init()  # Initialize the ROS 2 Python client library

    if len(sys.argv) != 3:  # Validate that exactly two numeric arguments (a, b) are provided
        print("Wrong number of arguments! Usage: simple_service  client A B")  # Print usage hint
        return -1  # Indicate error exit code for incorrect usage
    
    simple_service_client = SimpleServiceClient(int(sys.argv[1]), int(sys.argv[2]) )  # Create the client node with parsed integers
    rclpy.spin(simple_service_client)  # Keep the node alive to process the asynchronous response
    simple_service_client.destroy_node()  # Cleanly destroy the node after spinning ends
    rclpy.shutdown()  # Shutdown the ROS 2 client library
    return 0  # Return success

if __name__ == "__main__":  # Run main() only if this script is executed directly
    main()  # Invoke the main function
