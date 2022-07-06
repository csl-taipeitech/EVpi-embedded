import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    can_param_dir = LaunchConfiguration(
        'can_param_dir',
        default=os.path.join(
            get_package_share_directory('evpi_bringup'),
            'config','can_table' + '.yaml'))

    return LaunchDescription([

        DeclareLaunchArgument(
            'can_param_dir',
            default_value=can_param_dir,
            description='Full path to can parameter file to load'),

        # Node(
        #     package='kvaser_interface',
        #     executable='kvaser_can_bridge',
        #     name='can_peripheral',
        #     parameters=[{
        #         'hardware_id': 74771,}],
        #     output='screen'),

        # Node(
        #     package='kvaser_interface',
        #     executable='kvaser_can_bridge',
        #     name='can_drive',
        #     parameters=[{
        #         'hardware_id': 74778,}],
        #     output='screen'),

        Node(
            package='evpi_bringup',
            executable='can_manager',
            parameters=[can_param_dir],
            output='screen'),

    ])