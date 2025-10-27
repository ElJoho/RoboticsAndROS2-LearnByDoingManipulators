from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    is_sim = LaunchConfiguration("is_sim")
    # NEW (minimal): add is_ignition so we can explicitly disable it for real robot
    is_ignition = LaunchConfiguration("is_ignition")

    is_sim_arg = DeclareLaunchArgument("is_sim", default_value="false")
    # NEW (minimal): declare is_ignition (defaults to false)
    is_ignition_arg = DeclareLaunchArgument("is_ignition", default_value="false")

    # URDF/Xacro → robot_description
    xacro_path = os.path.join(get_package_share_directory("arduinobot_description"), "urdf", "arduinobot.urdf.xacro")
    # NEW (minimal): forward both args to xacro so it picks the correct ros2_control plugin
    robot_description = ParameterValue(
        Command([
            "xacro ", xacro_path,
            " is_sim:=", is_sim,
            " is_ignition:=", is_ignition,
        ]),
        value_type=str,
    )

    # Solo en REAL (no Gazebo): ros2_control_node
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            os.path.join(get_package_share_directory("arduinobot_controller"), "config", "arduinobot_controllers.yaml"),
        ],
        output="screen",
        condition=UnlessCondition(is_sim),   # <-- no lo arranques si usas el manager de Gazebo
    )

    # Spawners (cambia -c según sim/real)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
        condition=UnlessCondition(is_sim),   # en Gazebo usa /gazebo/controller_manager o crea un bloque alterno
    )
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
        output="screen",
        condition=UnlessCondition(is_sim),
    )
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager"],
        output="screen",
        condition=UnlessCondition(is_sim),
    )

    # Robot State Publisher (en ambos casos)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    return LaunchDescription([
        # NEW (minimal): include the new arg in the description
        is_sim_arg,
        is_ignition_arg,
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
    ])
