#!/usr/bin/env python3
import rclpy  # ROS 2 Python client library (init/spin/shutdown primitives)
import time  # Pauses to simulate work and pace feedback publication
import numpy as np
from rclpy.node import Node  # Base class for creating ROS 2 nodes
from rclpy.action import ActionServer  # Server-side API for ROS 2 actions
from arduinobot_msgs.action import ArduinobotTask  # Action definition (Goal/Feedback/Result)
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState

class TaskServer(Node):  # Node hosting the ArduinobotTask action server
    def __init__(self):  # Constructor: set up node and action server
        super().__init__("task_server")  # Initialize node with a name
        self.action_server = ActionServer(  # Create ActionServer instance
            self, ArduinobotTask, "task_server", self.goalCallback
        )  # node, action type, action name, execute callback
        self.get_logger().info("Starting the server")  # Log startup message
        self.arduinobot = MoveItPy(node_name="moveit_py")
        self.arduinobot_arm = self.arduinobot.get_planning_component("arm")
        self.arduinobot_gripper = self.arduinobot.get_planning_component("gripper")


    def goalCallback(self, goal_handle):  # Executes an accepted goal
        self.get_logger().info(  # Log requested order (sequence length hint)
            "Received goal request with task number %d" % goal_handle.request.task_number
        )
        
        arm_state = RobotState(self.arduinobot.get_robot_model())
        gripper_state = RobotState(self.arduinobot.get_robot_model())

        arm_joint_goal =[]
        gripper_joint_goal = []

        if  goal_handle.request.task_number == 0:
            arm_joint_goal = np.array([0.0 , 0.0 , 0.0]) 
            gripper_joint_goal = np.array([-0.7, 0.7])
        elif goal_handle.request.task_number == 1:
            arm_joint_goal = np.array([-1.14 , -0.6 , -0.07]) 
            gripper_joint_goal = np.array([0.0, 0.0])
        elif goal_handle.request.task_number == 2:
            arm_joint_goal = np.array([-1.57 , 0.0 , -0.9]) 
            gripper_joint_goal = np.array([0.0, 0.0])
        else:
            self.get_logger().error("Invalid Task NUmber")
            return
        
        arm_state.set_joint_group_positions("arm", arm_joint_goal)
        gripper_state.set_joint_group_positions("gripper", gripper_joint_goal)

        self.arduinobot_arm.set_start_state_to_current_state()
        self.arduinobot_gripper.set_start_state_to_current_state()

        self.arduinobot_arm.set_goal_state(robot_state=arm_state)
        self.arduinobot_gripper.set_goal_state(robot_state=gripper_state)

        arm_plan_result= self.arduinobot_arm.plan()
        gripper_plan_result = self.arduinobot_gripper.plan()

        if arm_plan_result and gripper_plan_result:
            self.arduinobot.execute(arm_plan_result.trajectory, controllers=[])
            self.arduinobot.execute(gripper_plan_result.trajectory, controllers=[])
        else:
            self.get_logger().info("One or more planners failed")
        
        goal_handle.succeed()
        result = ArduinobotTask.Result()
        result.success = True
        return result
    


def main():  # Program entry point
    rclpy.init()  # Initialize ROS 2 client library
    task_server = TaskServer()  # Create node instance
    rclpy.spin(task_server)  # Keep node alive processing callbacks
    task_server.destroy_node()  # Clean up node resources
    rclpy.shutdown()  # Shut down ROS 2 client lib

if __name__ == '__main__':  # Run main() when executed as a script
    main()  # Launch the action server node
