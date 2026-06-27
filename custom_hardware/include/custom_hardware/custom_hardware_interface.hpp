#pragma once

#include <string>
#include <vector>
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace custom_hardware
{

class CustomHardwareInterface : public hardware_interface::SystemInterface{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  bool open_serial_port();
  void close_serial_port();
  bool send_serial_command(const std::string & command);

  std::string serial_port_ = "/dev/ttyUSB0";
  int baud_rate_ = 115200;
  int serial_fd_ = -1;

  bool active_ = false;

  double left_cmd_ = 0.0;
  double right_cmd_ = 0.0;

  double left_pos_ = 0.0;
  double right_pos_ = 0.0;

  double left_vel_ = 0.0;
  double right_vel_ = 0.0;
};

}  // namespace custom_hardware