import os  # Standard library for composing filesystem paths
from launch import LaunchDescription  # Launch container type returned by this file
from moveit_configs_utils import MoveItConfigsBuilder  # Builder that assembles MoveIt parameter dictionaries
from launch_ros.actions import Node  # Launch action that starts a ROS 2 node process
from ament_index_python.packages import get_package_share_directory  # Finds a package's installed share/ directory at runtime


def generate_launch_description():  # Launch entrypoint function expected by ROS 2 launch

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
        .moveit_cpp(file_path="config/planning_python_api.yaml")
        .to_moveit_configs()  # Finalize to MoveItConfigs object
    )  # Close the parenthesized builder chain

    simple_moveit_interface = Node(
        package="arduinobot_py_examples",
        executable="simple_moveit_interface",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time":True}
        ]
    )

    return LaunchDescription([
        simple_moveit_interface
    ])