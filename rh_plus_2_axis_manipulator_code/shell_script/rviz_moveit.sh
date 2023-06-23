#!/bin/bash
source /opt/ros/humble/setup.bash
source $HOME/robot_ws/install/local_setup.bash

ros2 launch rh_plus_2_axis_manipulator_moveit_config demo.launch.py
