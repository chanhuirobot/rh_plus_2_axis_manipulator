################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.8)
project(rh_plus_2_axis_manipulator_code)

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
find_package(rclcpp REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)

include_directories(include)

add_executable(camera_node
  src/camera/camera_node.cpp
  src/camera/usb_camera_driver.cpp)
ament_target_dependencies(camera_node
  "rclcpp" "std_msgs" "OpenCV" "sensor_msgs" "camera_info_manager" "image_transport")

add_executable(vision_process_node
  src/camera/vision_process_node.cpp)
ament_target_dependencies(vision_process_node
  "rclcpp" "OpenCV" "cv_bridge" "moveit_ros_planning_interface")

add_executable(user_coord_input_node
  src/controller/user_coord_input_node.cpp)
ament_target_dependencies(user_coord_input_node
  "rclcpp" "moveit_ros_planning_interface")

add_executable(rectangle_drawing_node
  src/controller/rectangle_drawing_node.cpp)
ament_target_dependencies(rectangle_drawing_node
  "rclcpp" "moveit_ros_planning_interface")

# install cpp executables
install(TARGETS
  camera_node
  vision_process_node
  user_coord_input_node
  rectangle_drawing_node
  DESTINATION lib/${PROJECT_NAME}
)

# install python modules
# ament_python_install_package(${PROJECT_NAME})

# install Python executables
# nothing...

# install the launch directory
# install(DIRECTORY
#  launch
#  DESTINATION share/${PROJECT_NAME}/
#)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

################################################################################
# Find and load build settings from external packages
################################################################################
ament_package()
