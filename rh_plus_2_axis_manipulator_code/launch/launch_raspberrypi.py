
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
            # namespace='camera',
            executable='camera_node',
            name='camera',
            # output='screen',
            # prefix='lxterminal -e'
        ),
        Node(
            package='rh_plus_2_axis_manipulator_code',
            # namespace='servo_control',
            executable='servo_control_node',
            name='servo_control',
            # output='screen',
            # prefix='lxterminal -e'
        )
    ])
