#ifndef VISION_PROCESS_NODE_
#define VISION_PROCESS_NODE_

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include "global_variable.hpp"
using moveit::planning_interface::MoveGroupInterface;

// 노드명 : vision_node
// 기능 : 이미지 수신, 좌표 최신화
class VisionNode : public rclcpp::Node {
public:
  // 생성자 선언
  VisionNode();

  // 멤버 변수
  // 좌표비율 멤버 변수 선언
  static double coord_x_ratio_;
  static double coord_y_ratio_;
private:
  // 서브스크라이버로 사용되는 private 변수
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;

  // 멤버 함수
  // 이미지 수신, 좌표 추적(static 변수로 있는 좌표 최신화)
  void image_process_callback(sensor_msgs::msg::Image::SharedPtr data);
};

// static 멤버 초기화
double VisionNode::coord_x_ratio_ = 0.5;
double VisionNode::coord_y_ratio_ = 0.5;







class MoveItController : public rclcpp::Node {
public:
  // 생성자 선언
  MoveItController();
private:
  // 10초마다 moveit에다가 발신하도록 타이머 설정
  // 1초마다 카운트다운 할 수 있도록
  rclcpp::TimerBase::SharedPtr moveit_timer_;
  // moveit_node 생성
  std::shared_ptr<rclcpp::Node> const node_moveit = std::make_shared<rclcpp::Node>(
      "vision_process_node_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  // MoveIt MoveGroup Interface 생성
  MoveGroupInterface move_group_interface = MoveGroupInterface(node_moveit, "arm");

  // 멤버함수
  void moveit2_controller();
};

#endif
