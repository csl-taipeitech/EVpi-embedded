import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():


    return LaunchDescription([

        Node(
            package='evpi_bringup',
            executable='topic_simulator',
            output='screen'),

    ])