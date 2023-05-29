#ifndef SERVO_INFO_HPP_
#define SERVO_INFO_HPP_

#include <chrono>
#include <memory>
#include <functional>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"

#include "rh_plus_interface/msg/servo_read_data.hpp"

#include "global_variable.hpp"


class ServoInfoNode : public rclcpp::Node
{
  public:
    using ReadData = rh_plus_interface::msg::ServoReadData;

    explicit ServoInfoNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    virtual ~ServoInfoNode();

  private:

    void servo_info_callback(const ReadData::SharedPtr msg);
    rclcpp::Subscription<ReadData>::SharedPtr servo_info_subscriber_;

};


#endif
