#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "rh_plus_interface/msg/servo_read_data.hpp"
#include "rh_plus_interface/msg/twoaxis_ik.hpp"
#include "rh_plus_interface/action/twoaxis_servo.hpp"

#include "global_variable.hpp"

#include "angular/angular.hpp"



int main(int argc, char * argv[])
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<AngleControlNode>());
  rclcpp::shutdown();

  return 0;
}
