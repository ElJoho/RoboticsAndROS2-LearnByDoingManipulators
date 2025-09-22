#include <rclcpp/rclcpp.hpp> // Core ROS 2 C++ client library headers
#include <rclcpp_action/rclcpp_action.hpp> // ROS 2 actions client/server API
#include <arduinobot_msgs/action/fibonacci.hpp> // Custom action definition: Fibonacci
#include <memory> // std::shared_ptr, etc.
#include <rclcpp_components/register_node_macro.hpp> // Macro to register component nodes
#include <sstream> // std::stringstream for building log strings

using namespace std::chrono_literals; // Enables literals like 1s for durations
using namespace std::placeholders; // Enables _1, _2 placeholders for std::bind

namespace arduinobot_cpp_examples{ // Begin package namespace to scope the node
    class SimpleActionClient : public rclcpp::Node{ // Define a node class inheriting from rclcpp::Node
        public:
            explicit SimpleActionClient(const rclcpp::NodeOptions& options) :  Node("simple_action_client", options){ // Constructor: name node and pass options
                client_ = rclcpp_action::create_client<arduinobot_msgs::action::Fibonacci>( // Create an action client for Fibonacci
                    this, // Use this node as the ROS interface
                    "fibonacci" // Action name as exposed by the server
                ); // End create_client
                timer_ = create_wall_timer( // Create a periodic timer
                    1s, // Execute first callback after 1 second
                    std::bind(&SimpleActionClient::timerCallback, this) // Bind timer to member function
                ); // End create_wall_timer
            } // End constructor

        private:
            rclcpp_action::Client<arduinobot_msgs::action::Fibonacci>::SharedPtr client_; // Handle to the action client
            rclcpp::TimerBase::SharedPtr timer_; // Handle to the startup timer

            void timerCallback(){ // Timer callback that kicks off the goal send
                timer_->cancel(); // Cancel timer so it only runs once
                
                if(!client_->wait_for_action_server()){ // Wait for server; if not available, handle error
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Action server not available after waiting"); // Log error if server missing
                    rclcpp::shutdown(); // Cleanly shut down ROS
                    return; // Exit callback early
                } // End availability check

                auto goal_msg = arduinobot_msgs::action::Fibonacci::Goal(); // Create a goal message object

                goal_msg.order = 10; // Request Fibonacci sequence up to order 10

                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Sending Goal"); // Inform that we’re sending a goal

                auto send_goal_options = rclcpp_action::Client<arduinobot_msgs::action::Fibonacci>::SendGoalOptions(); // Configure callbacks for send

                send_goal_options.goal_response_callback = std::bind(&SimpleActionClient::goalCallback, this, _1); // Set goal acceptance/rejection callback
                send_goal_options.feedback_callback = std::bind(&SimpleActionClient::feedbackCallback, this, _1,_2); // Set feedback callback for partial sequence
                send_goal_options.result_callback = std::bind(&SimpleActionClient::resultCallback, this, _1); // Set result callback when action completes

                client_->async_send_goal(goal_msg, send_goal_options); // Asynchronously send the goal with callbacks

            } // End timerCallback

            void goalCallback(const rclcpp_action::ClientGoalHandle<arduinobot_msgs::action::Fibonacci>::SharedPtr& goal_handle){ // Called when server accepts/rejects goal
                if(!goal_handle){ // Check if goal was rejected (null handle)
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Goal was rejected by the server"); // Log rejection
                }else{ // Otherwise, goal accepted
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Goal Accepted by the Server, waiting for result"); // Log acceptance
                } // End acceptance branch
            } // End goalCallback

            void feedbackCallback( // Called every time server publishes feedback
                rclcpp_action::ClientGoalHandle<arduinobot_msgs::action::Fibonacci>::SharedPtr, // Goal handle (unused here)
                const std::shared_ptr<const arduinobot_msgs::action::Fibonacci::Feedback> feedback // Feedback message with partial sequence
            ){
                std::stringstream ss; // Build a readable message
                ss << "Next number in sequence received: "; // Prefix text
                for(auto number : feedback->partial_sequence){ // Append all numbers from the partial sequence
                    ss << number << " "; // Add each number followed by a space
                } // End loop

                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),ss.str().c_str()); // Log the constructed feedback string
            } // End feedbackCallback

            void resultCallback( // Called when server finishes and returns result
                const rclcpp_action::ClientGoalHandle<arduinobot_msgs::action::Fibonacci>::WrappedResult& result // Wrapped result with status + data
            ){
                switch(result.code){ // Inspect completion status
                    case rclcpp_action::ResultCode::SUCCEEDED: // Action finished successfully
                        break; // Continue to print final sequence
                    case rclcpp_action::ResultCode::ABORTED: // Action aborted by server
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Goal was aborted"); // Log aborted
                        return; // Stop processing
                    case rclcpp_action::ResultCode::CANCELED: // Action canceled
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Goal was canceled"); // Log canceled
                        return; // Stop processing
                    default: // Any other/unexpected status code
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Unknown result code"); // Log unknown status
                        return; // Stop processing
                } // End switch
                
                std::stringstream ss; // Build a readable final sequence message
                ss << "Next number in sequence received: "; // Prefix text (consistent with feedback)
                for(auto number : result.result->sequence){ // Iterate full resulting sequence
                    ss << number << " "; // Append numbers to stream
                } // End loop

                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),ss.str().c_str()); // Log the full sequence

                rclcpp::shutdown(); // Cleanly shut down after finishing
            } // End resultCallback

    }; // End class SimpleActionClient

} // End namespace arduinobot_cpp_examples

RCLCPP_COMPONENTS_REGISTER_NODE(arduinobot_cpp_examples::SimpleActionClient); // Register this class as a component node
