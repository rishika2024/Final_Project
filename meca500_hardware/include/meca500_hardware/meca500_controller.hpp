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
//
// Originally based on ros2_control demo examples.
// Modified for the Meca500 6-axis robot by rishika2024.

#ifndef MECA500_HARDWARE__MECA500_SYSTEM_HPP_
#define MECA500_HARDWARE__MECA500_SYSTEM_HPP_

#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace meca500_hardware
{

constexpr size_t NUM_JOINTS = 6;

class Meca500System : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Meca500System)
  // expose hw_positions_ / hw_velocities_ / hw_commands_ to the ros2_control framework
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // setup the hardware interface, read parameters from URDF, etc.
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  // open the socket to the robot. initalize data structures.
  // check if robot is reachable. but do not connect to motors yet.
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  
  // activate the robot: connect to motors, start communication threads, etc.
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  // deactivate the robot: stop communication threads, disconnect from motors, etc.
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  
 // close the socket, cleanup data structures, etc.
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  
  // read data from the robot
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
    
  // write commands to the robot
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Joint data — 6 joints
  std::vector<double> hw_positions_;   // current positions (from robot)
  std::vector<double> hw_velocities_;  // current velocities (from robot)
  std::vector<double> hw_commands_;    // commanded positions (from controller)

  // Connection params (from URDF)
  std::string robot_ip;
  int control_port = 0;  // port 10000 — send commands, receive responses
  int monitoring_port = 0;  // port 10001 — receive joint positions

  // TCP socket file descriptor for control port (10000) and monitoring port (10001)
  int control_fd = -1;
  int monitoring_fd = -1;

  // Calibration offsets (degrees): added to Mecademic angles to get URDF angles.
  // Derived from slider visual match [38,33,7,2] at actual home [-0.213,-0.196,-0.046,0.006].
  // Derived from slider visual match at actual home [-0.003879,-0.213362,-0.195776,-0.046034,0.00569,-0.02069].
  static constexpr double JOINT_OFFSET_DEG[NUM_JOINTS] = {0.0, 40.282, 34.984, -82.954, 2.023, 0.0};


  // Auto-detected at runtime in on_configure()
  bool use_fake_ = true;

  // Helper function to connect a TCP socket to ip:port, returns fd or -1 on failure
  int connect_socket(const std::string & ip, int port, int timeout_sec);
};

}  // namespace meca500_hardware

#endif  // MECA500_HARDWARE__MECA500_SYSTEM_HPP_