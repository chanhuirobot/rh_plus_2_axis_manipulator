################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.8)
project(rhplus_2axis_manipulator)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(msg_action_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)

include_directories(include)

add_executable(camera_node
  src/camera_node.cpp
  src/usb_camera_driver.cpp)
ament_target_dependencies(camera_node 
rclcpp std_msgs OpenCV sensor_msgs camera_info_manager image_transport)

add_executable(angular_control_node src/angular_control.cpp)
ament_target_dependencies(angular_control_node
  "msg_action_interfaces"
  "rclcpp"
  "rclcpp_action"
)

add_executable(servo_control_node
  src/servo_command.cpp
  src/servo_control.cpp
)
ament_target_dependencies(servo_control_node
  "msg_action_interfaces"
  "rclcpp"
  "rclcpp_action"
)

# install cpp executables
install(TARGETS
  servo_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  angular_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  camera_node
  DESTINATION lib/${PROJECT_NAME})

# install python modules
ament_python_install_package(${PROJECT_NAME})

# install Python executables
install(PROGRAMS
  rh_plus_2_axis_manipulator/vision_process_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

################################################################################
# Find and load build settings from external packages
################################################################################
ament_package()
