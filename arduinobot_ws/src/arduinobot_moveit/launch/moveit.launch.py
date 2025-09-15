import os  # Standard library for composing filesystem paths
from launch import LaunchDescription  # Launch container type returned by this file
from moveit_configs_utils import MoveItConfigsBuilder  # Builder that assembles MoveIt parameter dictionaries
from launch_ros.actions import Node  # Launch action that starts a ROS 2 node process
from launch.actions import DeclareLaunchArgument  # Lets us declare CLI/launch-file arguments
from launch.substitutions import LaunchConfiguration  # Substitution that resolves to a runtime argument value
from ament_index_python.packages import get_package_share_directory  # Finds a package's installed share/ directory at runtime


def generate_launch_description():  # Launch entrypoint function expected by ROS 2 launch

    is_sim = LaunchConfiguration("is_sim")  # Substitution bound to the value of "is_sim" at runtime
    
    is_sim_arg = DeclareLaunchArgument(  # Declare the "is_sim" argument (default True)
        "is_sim",  # Argument name used on the CLI
        default_value="True"  # Default value for the argument (simulation enabled)
    )  # Close DeclareLaunchArgument call

    moveit_config = (  # Begin building the MoveIt configuration object
        MoveItConfigsBuilder("arduinobot", package_name="arduinobot_moveit")  # Initialize builder with robot name & config package
        .robot_description(file_path=os.path.join(  # Provide robot URDF/Xacro path for robot_description
            get_package_share_directory("arduinobot_description"),  # Locate share/ of description package
            "urdf",  # Enter its URDF folder
            "arduinobot.urdf.xacro"  # Use this main Xacro as the model
            )  # Close os.path.join args
        )  # Close robot_description(...)
        .robot_description_semantic(file_path="config/arduinobot.srdf")  # Provide SRDF path for semantic description
        .trajectory_execution(file_path="config/moveit_controllers.yaml")  # Bind trajectory execution to controllers YAML
        .to_moveit_configs()  # Finalize to MoveItConfigs object
    )  # Close the parenthesized builder chain

    move_group_node = Node(  # Define the MoveIt move_group node (planning/execution)
        package="moveit_ros_move_group",  # Package containing the executable
        executable="move_group",  # The MoveIt server executable
        output="screen",  # Print logs to screen
        parameters=[moveit_config.to_dict(),  # All MoveIt parameters as a single dict
                    {"use_sim_time": is_sim},  # Use simulated clock when running in Gazebo
                    {"publish_robot_description_semantic": True}],  # Publish SRDF so RViz/others can consume it
        arguments=["--ros-args", "--log-level", "info"],  # Additional ROS CLI args (log level)
    )  # Close Node(...)

    # RViz  # Section marker preserved from your code
    rviz_config = os.path.join(  # Path to RViz config to preload MoveIt panel
        get_package_share_directory("arduinobot_moveit"),  # Find the moveit package share/
            "config",  # Enter its config folder
            "moveit.rviz",  # Use this RViz session file
    )  # Close os.path.join(...)
    rviz_node = Node(  # Define the RViz 2 visualization node
        package="rviz2",  # RViz 2 package
        executable="rviz2",  # RViz 2 executable
        name="rviz2",  # Node name
        output="log",  # Log RViz output to files
        arguments=["-d", rviz_config],  # Launch RViz with the config (-d)
        parameters=[  # Give RViz robot/semantic/kinematics/limits params
            moveit_config.robot_description,  # URDF-based robot description
            moveit_config.robot_description_semantic,  # SRDF semantic description
            moveit_config.robot_description_kinematics,  # Kinematics (IK) config per group (if present)
            moveit_config.joint_limits,  # Joint limit overrides (if present)
        ],  # Close parameters list
    )  # Close Node(...)

    return LaunchDescription(  # Return the launch description with all actions
        [
            is_sim_arg,  # Expose the is_sim argument
            move_group_node,  # Start MoveIt backend
            rviz_node  # Start RViz visualization
        ]  # Close action list
    )  # Close LaunchDescription(...)
