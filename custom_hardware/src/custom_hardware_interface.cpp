#include "custom_hardware/custom_hardware_interface.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace custom_hardware{
 
hardware_interface::CallbackReturn CustomHardwareInterface::on_init(const hardware_interface::HardwareInfo & info){
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS){
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.hardware_parameters.count("serial_port") > 0) {
    serial_port_ = info_.hardware_parameters["serial_port"];
  }

  if (info_.hardware_parameters.count("baud_rate") > 0) {
    baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> CustomHardwareInterface::export_state_interfaces(){
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back("chassis_to_leftWheel", hardware_interface::HW_IF_POSITION, &left_pos_);
  state_interfaces.emplace_back("chassis_to_leftWheel", hardware_interface::HW_IF_VELOCITY, &left_vel_);
  state_interfaces.emplace_back("chassis_to_rightWheel", hardware_interface::HW_IF_POSITION, &right_pos_);
  state_interfaces.emplace_back("chassis_to_rightWheel", hardware_interface::HW_IF_VELOCITY, &right_vel_);
  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> CustomHardwareInterface::export_command_interfaces(){
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back("chassis_to_leftWheel", hardware_interface::HW_IF_VELOCITY, &left_cmd_);
  command_interfaces.emplace_back("chassis_to_rightWheel", hardware_interface::HW_IF_VELOCITY, &right_cmd_);
  return command_interfaces;
}


hardware_interface::CallbackReturn CustomHardwareInterface::on_configure(const rclcpp_lifecycle::State &){
  left_cmd_ = 0.0;
  right_cmd_ = 0.0;

  left_pos_ = 0.0;
  right_pos_ = 0.0;

  left_vel_ = 0.0;
  right_vel_ = 0.0;

  if (!open_serial_port()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn CustomHardwareInterface::on_activate(const rclcpp_lifecycle::State &){
  active_ = true;
  send_serial_command("CMD 0.0 0.0\n");
  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn CustomHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &){
  active_ = false;

  send_serial_command("CMD 0.0 0.0\n");
  close_serial_port();

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::return_type CustomHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration & period){
  /*
    Temporary open-loop feedback.

    Since this version does not read encoder data from Arduino yet,
    we pretend the wheels actually moved at the commanded velocity.

    This is okay for early motor testing, but real odometry should come
    from encoders later.
  */

  if (!active_) {
    return hardware_interface::return_type::OK;
  }

  left_vel_ = left_cmd_;
  right_vel_ = right_cmd_;

  left_pos_ += left_vel_ * period.seconds();
  right_pos_ += right_vel_ * period.seconds();

  return hardware_interface::return_type::OK;
}


hardware_interface::return_type CustomHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &){
  if (!active_) {
    return hardware_interface::return_type::OK;
  }

  /*
    diff_drive_controller writes wheel velocity commands into:

      left_cmd_
      right_cmd_

    These are in rad/s because the command interface is velocity.
  */

  std::ostringstream command;
  command << "CMD " << left_cmd_ << " " << right_cmd_ << "\n";

  if (!send_serial_command(command.str())) {
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}


bool CustomHardwareInterface::open_serial_port(){
  serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("CustomHardwareInterface"),
      "Failed to open serial port %s: %s",
      serial_port_.c_str(),
      std::strerror(errno));
    return false;
  }

  termios tty{};
  if (tcgetattr(serial_fd_, &tty) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("CustomHardwareInterface"),
    "Failed to get serial attributes: %s", std::strerror(errno));
    close_serial_port();
    return false;
  }

  speed_t speed = B115200;

  if (baud_rate_ != 115200) {
    RCLCPP_WARN(
      rclcpp::get_logger("CustomHardwareInterface"),
      "Only 115200 is currently configured in this example. Using 115200.");
  }

  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_iflag &= ~IGNBRK;
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 5;

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("CustomHardwareInterface"),
      "Failed to set serial attributes: %s",
      std::strerror(errno));
    close_serial_port();
    return false;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("CustomHardwareInterface"),
    "Opened serial port %s at %d baud",
    serial_port_.c_str(),
    baud_rate_);

  return true;
}


void CustomHardwareInterface::close_serial_port(){
  if (serial_fd_ >= 0) {
    close(serial_fd_);
    serial_fd_ = -1;
  }
}


bool CustomHardwareInterface::send_serial_command(const std::string & command){
  if (serial_fd_ < 0) {
    return false;
  }

  const ssize_t bytes_written = ::write(serial_fd_, command.c_str(), command.size());

  if (bytes_written < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("CustomHardwareInterface"), "Serial write failed: %s", std::strerror(errno));
    return false;
  }

  return true;
}
}  // namespace custom_hardware


PLUGINLIB_EXPORT_CLASS(
  custom_hardware::CustomHardwareInterface,
  hardware_interface::SystemInterface
)