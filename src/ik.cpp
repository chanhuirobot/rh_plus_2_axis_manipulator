
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "rmw/qos_profiles.h"
#include "rh_plus_interface/msg/twoaxis_ik.hpp"
#include "global_variable.hpp"

using namespace std::chrono_literals;

class Ik : public rclcpp::Node
{
public:
  using IkAngle = rh_plus_interface::msg::TwoaxisIk;
  // Node name set
  Ik()
  : Node("ik")
  {
    const auto QOS_RKL10V =
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

    // String message type, topic name:"topic", queue size(limit): 10
    publisher_ = this->create_publisher<IkAngle>("ik_result", QOS_RKL10V);
    // by initializing timer_, it cause timer_callback function executes twice a second.

    joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",QOS_RKL10V,std::bind(&Ik::topic_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(100ms, std::bind(&Ik::timer_callback, this));
  }

private:


  void topic_callback(const sensor_msgs::msg::JointState & msg) const
  {

    std::stringstream ss;
    ss << "Received angle from rviz is: ";
    for (int i=0; i < 10; i++){
      std::cout << std::to_string(msg.position[i]) << "\t";
    }
    std::cout << "\n";
    // RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
  }
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;

  // message data is set and the messages are actually published.
  // RCLCPP_INFO -> Console print
  void timer_callback()
  {
    int angle[2];
    std::cout << "desired angle input: ";
    std::cin >> angle[0] >> angle[1];

    auto message = rh_plus_interface::msg::TwoaxisIk();

    for (int i=0; i<SERVO_NUM;i++){
      message.twoaxis_ik.push_back(angle[i]);
    }
    std::stringstream ss;
    ss << " " << message.twoaxis_ik[0] << " " << message.twoaxis_ik[1];
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", ss.str().c_str());
    publisher_->publish(message);
  }
  // Declaration of the timer, publisher and counter field
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<IkAngle>::SharedPtr publisher_;
};

// Node actually executes
int main(int argc, char * argv[])
{
  // ROS 2 initializes
  rclcpp::init(argc, argv);
  // starts processing data from the node, including callbacks from the timer
  rclcpp::spin(std::make_shared<Ik>());
  rclcpp::shutdown();
  return 0;
}
