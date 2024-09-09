// Copyright 2020 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TWOMANI_HARDWARE_INTERFACE__TWOMANI_SYSTEM_HPP_
#define TWOMANI_HARDWARE_INTERFACE__TWOMANI_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "twomani_hardware_interface/visibility_control.hpp"
#include "twomani.hpp"

namespace twomani_hardware
{
class TWOManiSystemHardware
: public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TWOManiSystemHardware);

  TWOMANI_HARDWARE_INTERFACE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  TWOMANI_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  TWOMANI_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  TWOMANI_HARDWARE_INTERFACE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  TWOMANI_HARDWARE_INTERFACE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  TWOMANI_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  TWOMANI_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parameters for the TWOMani simulation
  double hw_start_sec_;
  double hw_stop_sec_;
  double hw_slowdown_;
  double joint_initial_value[7];

  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_commands_last_;

  std::vector<double> hw_states_;
  std::vector<std::string> hw_joint_name_;

  twomani::twomani twomani;
};

}  // namespace twomani_hardware

#endif  // TWOMANI_HARDWARE_INTERFACE__TWOMANI_SYSTEM_HPP_