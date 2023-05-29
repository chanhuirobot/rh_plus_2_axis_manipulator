
#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"

#include "rh_plus_interface/msg/servo_read_data.hpp"

#include "global_variable.hpp"
#include "servo/servo_info.hpp"

ServoInfoNode::ServoInfoNode(const rclcpp::NodeOptions & node_options)
  : Node("servo_info_sub", node_options)
{
  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

  // Servo Topic received
  servo_info_subscriber_ = this->create_subscription<ReadData>(
        "servo_info",QOS_RKL10V,std::bind(&ServoInfoNode::servo_info_callback, this, std::placeholders::_1));
}

ServoInfoNode::~ServoInfoNode()
{
}

void ServoInfoNode::servo_info_callback(const ReadData::SharedPtr msg){

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

// Node actually executes
int main(int argc, char * argv[])
{
  // ROS 2 initializes
  rclcpp::init(argc, argv);
  // starts processing data from the node, including callbacks from the timer
  rclcpp::spin(std::make_shared<ServoInfoNode>());
  rclcpp::shutdown();
  return 0;
}
