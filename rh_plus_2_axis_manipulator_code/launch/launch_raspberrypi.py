
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rh_plus_2_axis_manipulator_code',
            executable='camera_node',
            name='camera_node',
            # output='screen',
            # prefix='lxterminal -e'
        ),
        Node(
            package='rh_plus_2_axis_manipulator_code',
            executable='servo_control_node',
            name='servo_control_node',
            # output='screen',
            # prefix='lxterminal -e'
        )
    ])
