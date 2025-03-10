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

#include "khi_ux150/khi_ux150_hardware.hpp"
#include <string>
#include <vector>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>
#include <iterator>
#include <math.h>

namespace khi_ux150_controller
{
CallbackReturn RobotSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

CallbackReturn RobotSystem::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // reset values always when configuring hardware
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_state(name, 0.0);
  }
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }
  for (const auto & [name, descr] : sensor_state_interfaces_)
  {
    set_state(name, 0.0);
  }
  RCLCPP_INFO(rclcpp::get_logger("HardwareInterface"), "Successfully configured");
  
  return CallbackReturn::SUCCESS;
}


CallbackReturn RobotSystem::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  if(telnetConnect("192.168.0.2", 23) < 0){
    RCLCPP_ERROR(rclcpp::get_logger("HardwareInterface"), "Failed to connect to the controller");
    return hardware_interface::CallbackReturn::ERROR;
  }
  /*TODO Retrieve robot info and status, clear errors, power motors,...*/
  telnetWrite("where");
  hw_state_position_ = parsePositionData(telnetRead());
  return hardware_interface::CallbackReturn::SUCCESS;
}

CallbackReturn RobotSystem::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  telnetDisconnect();
  return hardware_interface::CallbackReturn::SUCCESS;
}


return_type RobotSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  //Get position data
  telnetWrite("where");
  hw_state_position_ = parsePositionData(telnetRead());
  std::string tmp_pos;
  std::string tmp_vel;

  //Set hardware states
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    const auto name_vel = info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY;
    const auto name_pos = info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION;
    
    //Sim Mode
    //set_state(name_vel, get_command(name_vel));
    //set_state(name_pos, get_state(name_pos) + get_state(name_vel) * period.seconds());

    //Real Mode
    set_state(name_vel, (hw_state_position_[i]-get_state(name_pos))/period.seconds());
    set_state(name_pos, hw_state_position_[i]);
    tmp_pos.append(std::to_string(get_state(name_pos))+"\t");
    tmp_vel.append(std::to_string(get_state(name_vel))+"\t");
  }
  RCLCPP_INFO(rclcpp::get_logger("HardwareInterface"), "Set real pos values: %s", tmp_pos.c_str()); //TODO Remove
  RCLCPP_INFO(rclcpp::get_logger("HardwareInterface"), "Set real vel values: %s", tmp_vel.c_str()); //TODO Remove
  return return_type::OK;
}

return_type RobotSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  return return_type::OK;
}

int RobotSystem::telnetConnect(const std::string &ip, int port)
{
  const char * server_ip = ip.c_str();
  int server_port = port;
  sock_ = socket(AF_INET, SOCK_STREAM, 0);
  if (sock_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("HardwareInterface"), "Could not create socket");
    return -1;
  }

  struct sockaddr_in server_address;
  server_address.sin_family = AF_INET;
  server_address.sin_port = htons(server_port);
  
  // Convert IP address from text to binary form
  if (inet_pton(AF_INET, server_ip, &server_address.sin_addr) <= 0) {
    RCLCPP_ERROR(rclcpp::get_logger("HardwareInterface"), "Invalid address or adress not supported %s", server_ip);
    return -1;
  }
  
  // Connect to the server
  if (connect(sock_, (struct sockaddr*)&server_address, sizeof(server_address)) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("HardwareInterface"), "Unable to reach server");
    close(sock_);
    return -1;
  }

  is_connected_ = true;
  RCLCPP_INFO(rclcpp::get_logger("HardwareInterface"), "Connected to telnet server");
  return 0;
}

std::string RobotSystem::telnetRead()
{
    // Verify connection is up
    if (!is_connected_) {
        RCLCPP_ERROR(rclcpp::get_logger("HardwareInterface"), "Unable to reach server");
        return "";
    }

    char buffer[512];
    std::string response;
    int bytes_received;

    // Clear previous response
    response.clear();

    while (true) {
        bytes_received = recv(sock_, buffer, sizeof(buffer) - 1, 0);
        
        if (bytes_received < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("HardwareInterface"), "Read error: %s", strerror(errno));
            return "";
        }
        
        if (bytes_received == 0) {
            RCLCPP_WARN(rclcpp::get_logger("HardwareInterface"), "Connection closed by the peer");
            break; // Exit the loop if the connection is closed
        }
        
        buffer[bytes_received] = '\0'; // Null-terminate the buffer
        response += buffer; // Append to response

        // Check for termination character or condition
        if (response.find(">") != std::string::npos) {
            break; // Stop reading when prompt is detected
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("HardwareInterface.Telnet"), "READ: %s", response.c_str());

    return response;
}

int RobotSystem::telnetWrite(const std::string &telnet_command)
{
  if (!is_connected_) {
    RCLCPP_ERROR(rclcpp::get_logger("HardwareInterface"), "Unable to reach server");
    return -1;
  }
  std::string cmd = telnet_command + "\r\n"; // Append carriage return and newline
  if (send(sock_, cmd.c_str(), cmd.length(), 0) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("HardwareInterface"), "Failed to send command: %s", cmd.c_str());
    return -1;
  }else{
    RCLCPP_INFO(rclcpp::get_logger("HardwareInterface.Telnet"), "WRITE: %s", cmd.c_str());
  }
  return 0;
}

void RobotSystem::telnetDisconnect()
{
  if(is_connected_){
    close(sock_);
    is_connected_=false;
    RCLCPP_INFO(rclcpp::get_logger("HardwareInterface"), "Disconnected from server.");
  }
}

std::vector<double> RobotSystem::parsePositionData(const std::string &response)
{
  std::vector<double> positions; // Vector to store JT1 to JT6 values

  // Split the response into lines.
  std::istringstream stream(response);
  std::string line;
  bool dataLineFound = false;

  while (std::getline(stream, line)) {
      // Check for the presence of JT1 in a valid data line
      if (line.find("JT1") != std::string::npos) {
          dataLineFound = true; // Mark that we've found the JT data line
          continue; // Skip to the next line to find the values
      }

      // If we are after the line with JT1, let's capture values
      if (dataLineFound) {
          // Mark next lines as the one that holds values for JT1 to JT6
          // This should match the line with the actual values.
          std::istringstream jt_line_stream(line);
          std::string jt_value;
          // Read and convert each JT value to double
          while (jt_line_stream >> jt_value) {
              positions.push_back(degToRad(std::stod(jt_value))); // Convert string to double and store in the vector
          }
          break; // We found our values, exit the loop
      }
  }

  // If we did not find any positions, log an error
  if (positions.size() != 6) { // Ensure we have exactly 6 values for JT1 to JT6
      std::cerr << "Error: " << std::endl;
      RCLCPP_INFO(rclcpp::get_logger("HardwareInterface.Parser"), "Less than 6 JT data lines parsed.");
  }

  return positions; // Return the vector of parsed positions
}

double RobotSystem::degToRad(double degrees){
  return degrees * M_PI/180.0;
}


}  // namespace khi_ux150_controller



#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  khi_ux150_controller::RobotSystem, hardware_interface::SystemInterface)
