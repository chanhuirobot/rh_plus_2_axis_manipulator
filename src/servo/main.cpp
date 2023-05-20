#include <chrono>
#include <memory>
#include <functional>
#include <string>
#include <sstream>

#include "rh_plus_interface/msg/servo_read_data.hpp"
#include "rh_plus_interface/action/twoaxis_servo.hpp"

#include "servo/servo_command.hpp"
#include "global_variable.hpp"

#include "servo/servo.hpp"


// Node actually executes
int main(int argc, char * argv[])
{
  // ROS 2 initializes
  rclcpp::init(argc, argv);
  // starts processing data from the node, including callbacks from the timer
  rclcpp::spin(std::make_shared<ServoControlNode>());
  rclcpp::shutdown();
  return 0;
}
