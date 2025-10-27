#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource  # <-- needed
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Controllers
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(  # <-- wrap the path
            os.path.join(
                get_package_share_directory("arduinobot_controller"),
                "launch",
                "controller.launch.py",
            )
        ),
        launch_arguments={"is_sim": "False"}.items(),
    )

    # MoveIt
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(  # <-- wrap the path
            os.path.join(
                get_package_share_directory("arduinobot_moveit"),
                "launch",
                "moveit.launch.py",
            )
        ),
        launch_arguments={"is_sim": "False"}.items(),
    )

    # Remote interface
    remote_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(  # <-- wrap the path
            os.path.join(
                get_package_share_directory("arduinobot_remote"),
                "launch",
                "remote_interface.launch.py",
            )
        )
    )

    return LaunchDescription([controller, moveit, remote_interface])
