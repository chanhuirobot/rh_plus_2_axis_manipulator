#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "global_variable.hpp"

int main(int argc, char *argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "user_coord_input_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

  // get logger
  auto const logger = rclcpp::get_logger("user_coord_input_node_logger");

  // Target Pose 객체 생성
  geometry_msgs::msg::Pose target_pose;
  // set target pose. w,z는 항상 일정하므로
  // 여기부터
  double input_coord_x_ratio = 0.5;
  double input_coord_y_ratio = 0.5;

  // moveit의 plan 생성
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  // Plan 하고 Execute 하기
  move_group_interface.setPoseTarget(target_pose);
  auto success = static_cast<bool>(move_group_interface.plan(plan));
  if (success)
  {
    move_group_interface.execute(plan);
    RCLCPP_INFO(logger, "Success! Coord ratio : (x = %.3f, y = %.3f)", input_coord_x_ratio, input_coord_y_ratio);
  }
  else
    RCLCPP_ERROR(logger, "Planning failed! Maybe Out of Range...! :( ratio : (x = %.3f, y = %.3f)", input_coord_x_ratio, input_coord_y_ratio);

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
