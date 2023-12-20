# #!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    EnvironmentVariable,
    PathJoinSubstitution,
)

# from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    # Declare substitutions
    example_ros_dir = get_package_share_directory("example_ros")
    example_ros_params = os.path.join(example_ros_dir, "params", "example_params.yaml")
    params_file = LaunchConfiguration("params_file", default=example_ros_params)

    remappings = [("/test", "test"), ("/test2", "test2")]

    example_node = Node(
        package="example_ros",
        executable="example_ros",
        name="example_ros",
        output="screen",
        remappings=remappings,
        parameters=params_file,
    )

    ld = LaunchDescription()
    ld.add_action(example_node)

    return ld
