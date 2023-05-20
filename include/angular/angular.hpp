#ifndef ANGULAR_HPP_
#define ANGULAR_HPP_


#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "rh_plus_interface/msg/servo_read_data.hpp"
#include "rh_plus_interface/msg/twoaxis_ik.hpp"
#include "rh_plus_interface/action/twoaxis_servo.hpp"

#include "global_variable.hpp"

class AngleControlNode : public rclcpp::Node
{
  public:
    // To use the function without re-entering the namespace
    using IkAngle = rh_plus_interface::msg::TwoaxisIk;
    using ReadData = rh_plus_interface::msg::ServoReadData;
    using AngleControl = rh_plus_interface::action::TwoaxisServo;
    using GoalHandleRotate = rclcpp_action::ClientGoalHandle<AngleControl>;

    explicit AngleControlNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
    virtual ~AngleControlNode();


  private:
    int ik_result[SERVO_NUM] = {0,};
    void send_goal();
    rclcpp::Subscription<IkAngle>::SharedPtr ik_result_subscriber_;
    rclcpp::Subscription<ReadData>::SharedPtr servo_result_subscriber_;

    rclcpp_action::Client<AngleControl>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;

    void goal_response_callback(GoalHandleRotate::SharedPtr goal_handle);
    void feedback_callback(
        GoalHandleRotate::SharedPtr,
        const std::shared_ptr<const AngleControl::Feedback> feedback);
    void result_callback(const GoalHandleRotate::WrappedResult & result);


};

#endif
