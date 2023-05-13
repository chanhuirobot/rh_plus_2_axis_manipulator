
#include <chrono>
#include <memory>
#include <functional>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "msg_action_interfaces/msg/servo_read_data.hpp"
#include "msg_action_interfaces/action/twoaxis_servo.hpp"

#include "rhplus_2axis_manipulator/servo_command.hpp"

#define SERVO_NUM 2
#define ROTATION_COUNT 5

namespace Twoaxis_servo_control
{
class ServoControl : public rclcpp::Node
{
  public:
    using ReadData = msg_action_interfaces::msg::ServoReadData;
    using AngleControl = msg_action_interfaces::action::TwoaxisServo;
    using GoalHandleRotate = rclcpp_action::ServerGoalHandle<AngleControl>;

    explicit ServoControl(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("motor_control", options)
    {
      using namespace std::placeholders;

      this->action_server_ = rclcpp_action::create_server<AngleControl>(
        this->get_node_base_interface(),
        this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "motor_control",
        std::bind(&ServoControl::handle_goal, this, _1, _2),
        std::bind(&ServoControl::handle_cancel, this, _1),
        std::bind(&ServoControl::handle_accepted, this, _1));


        // Topic pub
        motor_data_pub_ = this->create_publisher<ReadData>("topic", 10);
        timer_ = this->create_wall_timer(
          std::chrono::microseconds(500),
          std::bind(&Twoaxis_servo_control::ServoControl::timer_callback, this));

    }

    private:

      void timer_callback()
      {
        auto message = ReadData();

        int * arr_temp = read_temp(SERVO_NUM);
        unsigned short int * arr_angle = read_angle(SERVO_NUM);

        for (int i=0; i<SERVO_NUM; i++){
          message.temp_data[i] = arr_temp[i];
          message.angle_data[i] = arr_angle[i];
        }
        RCLCPP_INFO(this->get_logger(), "Servo Information transmit!!");
        motor_data_pub_->publish(message);
      }
      // Declaration of the timerand publisher
      rclcpp::TimerBase::SharedPtr timer_;
      rclcpp::Publisher<ReadData>::SharedPtr motor_data_pub_;


      rclcpp_action::Server<AngleControl>::SharedPtr action_server_;


      rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const AngleControl::Goal> goal)
      {
        (void)uuid;

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


      rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleRotate> goal_handle)
      {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
      }

      void handle_accepted(const std::shared_ptr<GoalHandleRotate> goal_handle)
      {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&ServoControl::execute, this, _1), goal_handle}.detach();
      }

      void execute(const std::shared_ptr<GoalHandleRotate> goal_handle)
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
            result->last_angle = angle;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
          }
          // Write_angle(servo ID, Angle)
          write_angle(1, int(goal->desired_angle[0])*i/5);
          write_angle(2, int(goal->desired_angle[1])*i/5);

          // Publish feedback
          goal_handle->publish_feedback(feedback);
          RCLCPP_INFO(this->get_logger(), "Publish feedback");
          loop_rate.sleep();
        }

        // Check if goal is done
        if (rclcpp::ok()){
          result->last_angle = angle;
          goal_handle->succeed(result);
          RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
      }
  }; // class ServoControl

} // namespace Servo_motor_control



// Node actually executes
int main(int argc, char * argv[])
{
  // ROS 2 initializes
  rclcpp::init(argc, argv);
  // starts processing data from the node, including callbacks from the timer
  rclcpp::spin(std::make_shared<Twoaxis_servo_control::ServoControl>());
  rclcpp::shutdown();
  return 0;
}
