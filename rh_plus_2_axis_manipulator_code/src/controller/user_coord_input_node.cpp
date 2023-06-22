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
  target_pose.orientation.w = 1.0;
  target_pose.orientation.z = 0.0;

  // 무한 반복
  while (true)
  {
    // 사용자 입력 받기. 좌표 비율 받기. 만약 처음부터 'q'누르면 무한루프 종료
    // 만약 0.0 ~ 1.0 사이의 값을 넣지 않는다면 -> 에러 띄우고 다시 처음부터
    // 출력, 수신은 그냥 std::cout, std::cin 으로 하자.
    std::cout << "You can put x,y Coord Ratio(0.0 ~ 1.0). You quit this program with 'q'!" << std::endl;
    std::cout << "input x, y Coord Ratio >> ";
    std::string inputString;
    getline(std::cin, inputString);

    // 'q' 입력시 프로그램 종료
    if (inputString == "q")
    {
      RCLCPP_INFO(logger, "Program Exit!");
      break;
    }
    // 이외 입력시
    else
    {
      double input_coord_x_ratio, input_coord_y_ratio;
      input_coord_x_ratio = std::stod(inputString.substr(0, inputString.find(' ')));
      input_coord_y_ratio = std::stod(inputString.substr(inputString.find(' ') + 1));

      // 띄어쓰기 없을때(parameter 한개만 입력 했을 때)
      if (inputString.find(' ') == std::string::npos)
      {
        RCLCPP_ERROR(logger, "Wrong Input..!! You can put x, y. Only Two Argument");
        continue;
      }

      // 만약에 0.0~1.0 사이가 아니라면
      else if (input_coord_x_ratio < 0 || input_coord_x_ratio > 1 || input_coord_y_ratio < 0 || input_coord_y_ratio > 1)
      {
        RCLCPP_ERROR(logger, "Wrong Input..!! You can put x, y. Coord Ratio (0.0~1.0)");
        continue;
      }

      // plan 좌표 입력
      target_pose.position.x = input_coord_x_ratio * BOARD_WIDTH;
      target_pose.position.y = input_coord_y_ratio * BOARD_HEIGHT;

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
      else RCLCPP_ERROR(logger, "Planning failed! Maybe Out of Range...! :( ratio : (x = %.3f, y = %.3f)", input_coord_x_ratio, input_coord_y_ratio);
    }
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
