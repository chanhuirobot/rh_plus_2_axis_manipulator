cmake_minimum_required(VERSION 3.8)
project(rh_plus_2_axis_manipulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)


# Include cpp "include" directory
include_directories(include)

# Create cpp executable
# for terminal $ ros2 run rh_plus_2_axis_manipulator {here}
add_executable(camera_node
  src/camera_node.cpp
  src/usb_camera_driver.cpp)
ament_target_dependencies(camera_node rclcpp std_msgs OpenCV sensor_msgs camera_info_manager image_transport)

# install cpp executables
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
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
