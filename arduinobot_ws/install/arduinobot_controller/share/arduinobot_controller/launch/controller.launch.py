from launch import LaunchDescription
from launch_ros.actions import Node                            # Importa la acción para definir y lanzar nodos de ROS 2.
from launch_ros.parameter_descriptions import ParameterValue   # Permite definir valores de parámetros (p.ej., robot_description).
from launch.actions import DeclareLaunchArgument               # Acción para declarar argumentos que el usuario puede pasar al launch.
from launch.substitutions import Command, LaunchConfiguration  # Herramientas para construir strings/valores dinámicos en tiempo de lanzamiento.
import os                                                      # Módulo estándar para manipular rutas y el sistema de archivos.
from ament_index_python.packages import get_package_share_directory  # Obtiene la ruta de instalación (share) de un paquete de ROS 2.


def generate_launch_description():

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(get_package_share_directory("arduinobot_description"),"urdf","arduinobot.urdf.xacro")
            ]
        ),
        value_type=str
    )
    robot_state_publisher_node = Node(                         # Define el nodo que publicará el URDF en /robot_description y TF.
        package="robot_state_publisher",                       # Paquete donde vive el ejecutable.
        executable="robot_state_publisher",                    # Nombre del ejecutable a lanzar.
        parameters=[{"robot_description": robot_description}]  # Pasa el parámetro 'robot_description' ya resuelto (URDF plano).
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description":robot_description},
            os.path.join(
                get_package_share_directory("arduinobot_controller"),
                "config",
                "arduinobot_controllers.yaml"
            )
        ]
    )



    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            controller_manager,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            gripper_controller_spawner
        ]
    )