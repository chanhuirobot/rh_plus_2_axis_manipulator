
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

ServoControlNode::ServoControlNode(const rclcpp::NodeOptions & options)
: Node("motor_control", options)
{
  using namespace std::placeholders;

  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

  motor_data_pub_ = this->create_publisher<ReadData>("servo_data", QOS_RKL10V);

  this->action_server_ = rclcpp_action::create_server<AngleControl>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "motor_control",
    std::bind(&ServoControlNode::handle_goal, this, _1, _2),
    std::bind(&ServoControlNode::handle_cancel, this, _1),
    std::bind(&ServoControlNode::execute,this, _1));

    this->timer_=
      this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&ServoControlNode::topic_pub_servo_info, this));

}

ServoControlNode::~ServoControlNode()
{
}

void ServoControlNode::topic_pub_servo_info()
{
  auto message = ReadData();

  int * arr_temp = read_temp(SERVO_NUM);
  unsigned short int * arr_angle = read_angle(SERVO_NUM);

  // int arr_temp[2];
  // arr_temp[0] = 20; arr_temp[1] = 22;
  // unsigned short int arr_angle[2];
  // arr_angle[0] = 0x64; arr_angle[1] = 0x56;

  for (int i=0; i<SERVO_NUM; i++){
    message.angle_data.push_back(arr_angle[i]);
    message.temp_data.push_back(arr_temp[i]);
  }
  RCLCPP_INFO(this->get_logger(), "Servo Information transmit!! temp: %d \t %d, angle: %d \t %d",message.temp_data[0], message.temp_data[1], message.angle_data[0], message.angle_data[1]);
  motor_data_pub_->publish(message);
}

rclcpp_action::GoalResponse ServoControlNode::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const AngleControl::Goal> goal)
  {
    (void)uuid;
    (void)goal;
    std::stringstream ss;
    ss << "Received goal request with angle: ";
    for (int i=0; i < SERVO_NUM; i++){
      ss << goal->desired_angle[i] << " ";
    }
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

    for (int i=0; i < SERVO_NUM; i++){
      if (goal->desired_angle[i] > 240 || goal->desired_angle[i] < 0){
        return rclcpp_action::GoalResponse::REJECT;
      }
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

rclcpp_action::CancelResponse ServoControlNode::handle_cancel(
  const std::shared_ptr<GoalHandleRotate> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

void ServoControlNode::execute(const std::shared_ptr<GoalHandleRotate> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  rclcpp::Rate loop_rate(1);
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<AngleControl::Feedback>();
  auto & angle = feedback->current_angle;
  auto result = std::make_shared<AngleControl::Result>();

  // Publish feedback and Motor Communication
  for (int i=0; i<ROTATION_COUNT; i++){
    // Check if there is a cancel request
    if (goal_handle->is_canceling()) {
      for (int i=0; i<SERVO_NUM;i++){
        result->last_angle.push_back(angle[i]);
      }
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }

    // Write_angle(servo ID, Angle)
    for(int j=0; j < SERVO_NUM; j++){
      //0xFE(254): BroadCast ID
      write_angle(254,goal->desired_angle[j] / ROTATION_COUNT * (i+1));
      angle.push_back(goal->desired_angle[j] / ROTATION_COUNT * (i+1));
    }

    // Publish feedback
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), "Publish feedback");
    angle.clear();
    loop_rate.sleep();
  }

  // Check if goal is done
  if (rclcpp::ok()){
    result->last_angle = angle;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
}


