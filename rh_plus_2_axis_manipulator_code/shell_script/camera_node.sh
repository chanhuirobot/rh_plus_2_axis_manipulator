#!/bin/bash
source /opt/ros/humble/setup.bash
source $HOME/robot_ws/install/local_setup.bash

ros2 run rh_plus_2_axis_manipulator_code camera_node
