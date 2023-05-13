
#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "msg_action_interfaces/msg/servo_read_data.hpp"
#include "msg_action_interfaces/msg/twoaxis_ik.hpp"
#include "msg_action_interfaces/action/twoaxis_servo.hpp"


#define SERVO_NUM 2

namespace Twoaxis_angular_control
{
class AngularControl : public rclcpp::Node
{
  public:

    // To use the function without re-entering the namespace
    using IkAngle = msg_action_interfaces::msg::TwoaxisIk;
    using ReadData = msg_action_interfaces::msg::ServoReadData;
    using AngleControl = msg_action_interfaces::action::TwoaxisServo;
    using GoalHandleRotate = rclcpp_action::ClientGoalHandle<AngleControl>;

    // To receive ik topic;s content
    int ik_result[SERVO_NUM] = {0,};

    explicit AngularControl(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
    : Node("angular_control_client", node_options)

    {
      this->client_ptr_ = rclcpp_action::create_client<AngleControl>(
        this->get_node_base_interface(),
        this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "angular_control");

        // per 500ms, to transmit goal we set Timer
        this->timer_ = this->create_wall_timer(
          std::chrono::milliseconds(500),
          std::bind(&AngularControl::send_goal, this));


        // IK Topic received
        ik_result_subscriber_ = this->create_subscription<IkAngle>(
            "ik_result",10,std::bind(&Twoaxis_angular_control::AngularControl::topic_callback_ik,this,std::placeholders::_1));
        // Servo Topic received
        servo_result_subscriber_ = this->create_subscription<ReadData>(
          "servo_data",10,std::bind(&Twoaxis_angular_control::AngularControl::topic_callback_servo,this,std::placeholders::_1));
    }

    void send_goal()
    {
      using namespace std::placeholders;

      this->timer_->cancel();

      // does open server?
      if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(100))){
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
        return;
      }


      auto goal_msg = AngleControl::Goal();
      for (int i=0; i<SERVO_NUM; i++){
        goal_msg.desired_angle.push_back(ik_result[i]);
      }
      RCLCPP_INFO(this->get_logger(), "Sending goal");

      auto send_goal_options = rclcpp_action::Client<AngleControl>::SendGoalOptions();
      send_goal_options.goal_response_callback =
        std::bind(&AngularControl::goal_response_callback, this, _1);
      send_goal_options.feedback_callback =
        std::bind(&AngularControl::feedback_callback, this, _1, _2);
      send_goal_options.result_callback =
        std::bind(&AngularControl::result_callback, this, _1);
      this->client_ptr_->async_send_goal(goal_msg, send_goal_options);

    }

    private:
      // receives the string message data
      void topic_callback_ik(const IkAngle::SharedPtr msg)
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
      rclcpp::Subscription<IkAngle>::SharedPtr ik_result_subscriber_;

      void topic_callback_servo(const ReadData::SharedPtr msg)
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
      rclcpp::Subscription<ReadData>::SharedPtr servo_result_subscriber_;

      rclcpp_action::Client<AngleControl>::SharedPtr client_ptr_;
      rclcpp::TimerBase::SharedPtr timer_;

      void goal_response_callback(GoalHandleRotate::SharedPtr goal_handle)
      {
        if (!goal_handle){
          RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
          rclcpp::shutdown();
        } else{
          RCLCPP_INFO(this->get_logger(), "Goal completes !!");
        }
      }

      void feedback_callback(
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


      void result_callback(const GoalHandleRotate::WrappedResult & result)
      {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
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

        // if result is succeeded
        std::stringstream ss;
        ss << "Result received: ";
        for (int i= 0; i < SERVO_NUM; i++){
          ss << result.result->last_angle[i] << " ";
        }

        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
        rclcpp::shutdown();
      }

}; // class AngularControl

} // namespace Twoaxis_angular_control


int main(int argc, char * argv[])
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<Twoaxis_angular_control::AngularControl>());
  rclcpp::shutdown();

  return 0;
}
