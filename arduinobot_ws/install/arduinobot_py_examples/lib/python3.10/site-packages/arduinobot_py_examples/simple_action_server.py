import rclpy  # ROS 2 Python client library (init/spin/shutdown primitives)
import time  # Pauses to simulate work and pace feedback publication
from rclpy.node import Node  # Base class for creating ROS 2 nodes
from rclpy.action import ActionServer  # Server-side API for ROS 2 actions
from arduinobot_msgs.action import Fibonacci  # Action definition (Goal/Feedback/Result)

class SimpleActionServer(Node):  # Node hosting the Fibonacci action server
    def __init__(self):  # Constructor: set up node and action server
        super().__init__("simple_action_server")  # Initialize node with a name
        self.action_server = ActionServer(  # Create ActionServer instance
            self, Fibonacci, "fibonacci", self.goalCallback
        )  # node, action type, action name, execute callback
        self.get_logger().info("Starting the server")  # Log startup message
    
    def goalCallback(self, goal_handle):  # Executes an accepted goal
        self.get_logger().info(  # Log requested order (sequence length hint)
            "Received goal request with order %d" % goal_handle.request.order
        )
        feedback_msg = Fibonacci.Feedback()  # Feedback container
        feedback_msg.partial_sequence = [0, 1]  # Seed Fibonacci series

        for i in range(1, goal_handle.request.order):  # Build sequence iteratively
            feedback_msg.partial_sequence.append(  # Next = sum of two previous
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i - 1]
            )
            self.get_logger().info(  # Show current partial sequence
                "Feedback: {0}".format(feedback_msg.partial_sequence)
            )
            goal_handle.publish_feedback(feedback_msg)  # Stream feedback to client
            time.sleep(1)  # Slow down loop so feedback is observable

        goal_handle.succeed()  # Mark goal as successfully completed
        result = Fibonacci.Result()  # Result container
        result.sequence = feedback_msg.partial_sequence  # Return full sequence
        return result  # ActionServer sends this back to the client
    

def main():  # Program entry point
    rclpy.init()  # Initialize ROS 2 client library
    simple_action_server = SimpleActionServer()  # Create node instance
    rclpy.spin(simple_action_server)  # Keep node alive processing callbacks
    simple_action_server.destroy_node()  # Clean up node resources
    rclpy.shutdown()  # Shut down ROS 2 client lib

if __name__ == '__main__':  # Run main() when executed as a script
    main()  # Launch the action server node
