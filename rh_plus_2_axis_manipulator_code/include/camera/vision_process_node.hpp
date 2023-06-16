#ifndef VISION_PROCESS_NODE_
#define VISION_PROCESS_NODE_

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include "global_variable.hpp"
using moveit::planning_interface::MoveGroupInterface;

// 노드명 : VisionProcessor
// 기능 : 이미지 수신, 좌표 추적, 10초에 한번씩 moveit에서 좌표 보내기
class VisionProcessNode : public rclcpp::Node {
public:
  // 생성자 선언
  explicit VisionProcessNode();

private:
  // 멤버 변수, 객체
  // 좌표비율 멤버 변수 선언
  double coord_x_ratio_;
  double coord_y_ratio_;

  // 10초마다 moveit에다가 발신하도록 타이머 설정
  // 1초마다 카운트다운 할 수 있도록
  rclcpp::TimerBase::SharedPtr moveit_timer_;
  // moveit_node 생성
  std::shared_ptr<rclcpp::Node> const node_moveit = std::make_shared<rclcpp::Node>(
      "vision_process_node_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  // logger 변수 생성 + init
  rclcpp::Logger const logger = rclcpp::get_logger("vision_process_node");
  // MoveIt MoveGroup Interface 생성
  MoveGroupInterface move_group_interface = MoveGroupInterface(node_moveit, "arm");
  // 서브스크라이버로 사용되는 private 변수
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;

  // 멤버 함수
  // 이미지 수신, 좌표 추적(멤버 변수로 있는 좌표 최신화)
  void image_process_callback(sensor_msgs::msg::Image::SharedPtr data);
  // 10초를 세고 moveit을 plan 하고 execute 한다.
  void moveit2_controller();
};

#endif
