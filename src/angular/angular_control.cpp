
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

AngleControlNode::AngleControlNode(const rclcpp::NodeOptions & node_options)
  : Node("angular_control_client", node_options)
{
  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

  // IK Topic received
  ik_result_subscriber_ = this->create_subscription<IkAngle>(
        "ik_result",
        QOS_RKL10V,
        [this](const IkAngle::SharedPtr msg) -> void
        {
            for (int i=0; i<SERVO_NUM;i++){
              ik_result[i] = msg->twoaxis_ik[i];
            }

            std::stringstream ss;
            ss << "Angular node received angle from ik: ";
            for (int i= 0; i < SERVO_NUM; i++){
            ss << ik_result[i] << " ";
            }
            RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
        }
  );


  // Servo Topic received
  servo_result_subscriber_ = this->create_subscription<ReadData>(
        "servo_data",
        QOS_RKL10V,
        [this](const ReadData::SharedPtr msg) -> void
        {
          std::stringstream ss;
          ss << "Angular node received angle from servo: ";
          for (int i= 0; i < SERVO_NUM; i++){
            ss << msg->angle_data[i] << " ";
          }
          ss << "\nAngular node received temp from servo: ";
          for (int i=0; i < SERVO_NUM; i++){
            ss << msg->temp_data[i] << " ";
          }
          RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
        }
  );

  this->client_ptr_ = rclcpp_action::create_client<AngleControl>(
  this->get_node_base_interface(),
  this->get_node_graph_interface(),
  this->get_node_logging_interface(),
  this->get_node_waitables_interface(),
  "motor_control");

  // per 500ms, to transmit goal we set Timer
  this->timer_ = this->create_wall_timer(
  std::chrono::seconds(5),
  std::bind(&AngleControlNode::send_goal, this));

}

AngleControlNode::~AngleControlNode()
{
}

void AngleControlNode::send_goal()
{
  using namespace std::placeholders;

  // this->timer_->cancel();

  if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))){
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
    return;
  }

  // ik_result[0] = 120; ik_result[1] = 80;
  auto goal_msg = AngleControl::Goal();
  for (int i=0; i<SERVO_NUM; i++){
    goal_msg.desired_angle.push_back(ik_result[i]);
  }
  RCLCPP_INFO(this->get_logger(), "Sending goal");

  auto send_goal_options = rclcpp_action::Client<AngleControl>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&AngleControlNode::goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
    std::bind(&AngleControlNode::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
    std::bind(&AngleControlNode::result_callback, this, _1);
  this->client_ptr_->async_send_goal(goal_msg, send_goal_options);

}

void AngleControlNode::goal_response_callback(GoalHandleRotate::SharedPtr goal_handle)
{
  if (!goal_handle){
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    rclcpp::shutdown();
  } else{
    RCLCPP_INFO(this->get_logger(), "Goal Transmittion completes !!");
  }
}

void AngleControlNode::feedback_callback(
        GoalHandleRotate::SharedPtr,
        const std::shared_ptr<const AngleControl::Feedback> feedback)

{
  std::stringstream ss;
  ss << "Current angle feedback received: ";
  for (int i= 0; i < SERVO_NUM; i++){
    ss << feedback->current_angle[i] << " ";
  }

  RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
}

void AngleControlNode::result_callback(const GoalHandleRotate::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Action succeeded!");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
    }

  //rclcpp::shutdown();
}

