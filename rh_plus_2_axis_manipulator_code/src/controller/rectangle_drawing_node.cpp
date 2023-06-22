#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "global_variable.hpp"

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  // logger
  auto const logger = rclcpp::get_logger("user_coord_input_node_logger");
  RCLCPP_INFO(logger, "Starting Rectangle Drawing Node. Start Moving.");

  auto const node = std::make_shared<rclcpp::Node>(
      "user_coord_input_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

  // Target Pose 객체 생성
  geometry_msgs::msg::Pose target_pose;
  // set target pose. w,z는 항상 일정하므로
  target_pose.orientation.w = 1.0;
  target_pose.orientation.z = 0.0;
  // plan 좌표 입력
  // 사각형 좌측 하단 이동
  target_pose.position.x = 0.50 * BOARD_WIDTH;
  target_pose.position.y = 0.05 * BOARD_HEIGHT;

  // moveit의 plan 생성
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  // Plan 하고 Execute 하기
  move_group_interface.setPoseTarget(target_pose);
  auto success = static_cast<bool>(move_group_interface.plan(plan));
  if(success){
    move_group_interface.execute(plan);
    RCLCPP_INFO(logger, "Plan Success!");
  }

  // 펜 내리기 위해 5초 대기
  RCLCPP_INFO(logger, "Start drawing in 5 seconds. Move Pen down.");
  rclcpp::sleep_for(std::chrono::seconds(5));

  // 사각형 그리기 위해 4점 넣기
  std::vector<geometry_msgs::msg::Pose> waypoints;

  // 우측 하단
  target_pose.position.x = 0.75 * BOARD_WIDTH;
  target_pose.position.y = 0.05 * BOARD_HEIGHT;
  waypoints.push_back(target_pose);

  // 우측 상단
  target_pose.position.x = 0.75 * BOARD_WIDTH;
  target_pose.position.y = 0.42 * BOARD_HEIGHT;
  waypoints.push_back(target_pose);


  // 좌측 상단
  target_pose.position.x = 0.50 * BOARD_WIDTH;
  target_pose.position.y = 0.42 * BOARD_HEIGHT;
  waypoints.push_back(target_pose);

  // 좌측 하단(약간 밖으로)
  target_pose.position.x = 0.50 * BOARD_WIDTH;
  target_pose.position.y = -0.50 * BOARD_HEIGHT;
  waypoints.push_back(target_pose);

  waypoints.push_back(target_pose);

  // plan
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  RCLCPP_INFO(logger, "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
  plan.trajectory_ = trajectory;

  // 실제 execute
  move_group_interface.execute(plan);

  RCLCPP_INFO(logger, "Drawing Complete. Move Pen Up.");

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}

