// Copyright 2023 ros2_control Development Team
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

#ifndef khi_ux150__R6BOT_HARDWARE_HPP_
#define khi_ux150__R6BOT_HARDWARE_HPP_

#include "string"
#include "unordered_map"
#include "vector"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

using hardware_interface::return_type;


namespace khi_ux150_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class RobotSystem : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

protected:
  int telnetConnect(const std::string &ip, int port);
  std::string telnetRead();
  int telnetWrite(const std::string &telnet_command);
  void telnetDisconnect();
  std::vector<double> parsePositionData(const std::string& response);
  double degToRad(double degrees);
  std::string getJmoveCommand(std::vector<double> command);
  int sock_;
  bool is_connected_;
  std::vector<double> hw_state_position_;
  std::vector<double> hw_state_velocity_;
  std::vector<double> hw_command_position_;
  rclcpp::Time hw_previous_time_;
};

}  // namespace khi_ux150_controller

#endif  // khi_ux150__R6BOT_HARDWARE_HPP_
