
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
            namespace='servo_info',
            executable='servo_info_node',
            name='servo_info',
            output='screen',
            prefix='lxterminal -e'
        ),
        ExecuteProcess(
            cmd= ['ros2','launch','rh_plus_2_axis_manipulator_moveit_config','demo.launch.py'],
            output='screen'
        )
    ])
