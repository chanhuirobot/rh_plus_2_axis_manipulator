#ifndef SERVO_HPP_
#define SERVO_HPP_

#include <chrono>
#include <memory>
#include <functional>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "rh_plus_interface/msg/servo_read_data.hpp"
#include "rh_plus_interface/action/twoaxis_servo.hpp"

#include "servo/servo_command.hpp"
#include "global_variable.hpp"


class ServoControlNode : public rclcpp::Node
{
  public:
    using ReadData = rh_plus_interface::msg::ServoReadData;
    using AngleControl = rh_plus_interface::action::TwoaxisServo;
    using GoalHandleRotate = rclcpp_action::ServerGoalHandle<AngleControl>;

    explicit ServoControlNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    virtual ~ServoControlNode();



  private:

    void topic_pub_servo_info();
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const AngleControl::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleRotate> goal_handle);
    void execute(const std::shared_ptr<GoalHandleRotate> goal_handle);

    rclcpp::Publisher<ReadData>::SharedPtr
      motor_data_pub_;

    rclcpp_action::Server<AngleControl>::SharedPtr
      action_server_;

};


#endif
