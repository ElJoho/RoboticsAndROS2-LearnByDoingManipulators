import rclpy  # Import the ROS 2 Python client library
from rclpy.node import Node  # Import the base Node class to create ROS 2 nodes
from rclpy.action import ActionClient  # Import the ActionClient to communicate with action servers
from arduinobot_msgs.action import Fibonacci  # Import the Fibonacci action interface (Goal/Feedback/Result)

class SimpleActionClient(Node):  # Define a node class that acts as an action client
    def __init__(self):  # Constructor for initializing the node and client
        super().__init__("simle_action_client")  # Initialize the Node with a name (note: "simle" is a typo kept as-is)
        self.action_client = ActionClient(self, Fibonacci, "fibonacci")  # Create an ActionClient bound to this node, using the Fibonacci interface and the "fibonacci" action name

        self.action_client.wait_for_server()  # Block here until an action server named "fibonacci" becomes available

        self.goal = Fibonacci.Goal()  # Create a new Goal message for the Fibonacci action

        self.goal.order = 10  # Request the Fibonacci sequence up to order 10

        self.future = self.action_client.send_goal_async(  # Asynchronously send the goal to the server
            self.goal,
            feedback_callback=self.feedbackCallback  # Register a callback to process feedback messages while the goal executes
        )

        self.future.add_done_callback(self.responseCallback)  # When the goal-acceptance future completes, run responseCallback to check accept/reject

    def responseCallback(self, future):  # Called when the server responds to the goal request (accept/reject)
        goal_handle = future.result()  # Extract the GoalHandle from the future (represents the accepted/rejected goal on the server)
        if not goal_handle.accepted:  # If the server rejected the goal
            self.get_logger().info("Goal Rejected")  # Log rejection (note: "Goa" is a typo in the original string)
            return  # Stop processing if the goal was not accepted
        
        self.get_logger().info("Goal Accepted")  # Inform that the goal was accepted by the server
        self.future = goal_handle.get_result_async()  # Ask asynchronously for the final Result once the action finishes
        self.future.add_done_callback(self.resultCallback)  # When the result future completes, call resultCallback

    def resultCallback(self, future):  # Called when the action server sends the final result
        result = future.result().result  # Extract the action Result message from the completed future
        self.get_logger().info("Result: {0}".format(result.sequence))  # Log the full Fibonacci sequence computed by the server
        rclpy.shutdown()  # Shut down the ROS 2 client library since this simple example is done
    
    def feedbackCallback(self, feedback_msg):  # Called whenever the server publishes feedback during execution
        self.get_logger().info(  # Log the partial sequence reported in the feedback
            "Received Feedback: {0}".format(feedback_msg.feedback.partial_sequence)
        )

def main():  # Entry point for running the node from the command line
    rclpy.init()  # Initialize the ROS 2 client library
    action_client = SimpleActionClient()  # Create an instance of the action client node
    rclpy.spin(action_client)  # Keep the node alive to process feedback and the final result

if __name__ == '__main__':  # Run main() only if this file is executed as a script
    main()  # Start the node
