#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <termios.h>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"

namespace robot_arm_hardware
{

class RobotArmSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  static constexpr std::size_t kJointCount = 5;

  bool open_serial();
  void close_serial();
  bool configure_serial_port(int fd) const;
  bool send_line(const std::string & line);
  bool read_line(std::string & line, int timeout_ms);
  bool read_matching_line(
    std::string & line, int timeout_ms, const std::function<bool(const std::string &)> & predicate);
  bool request_state();
  bool parse_state_line(const std::string & line, const rclcpp::Duration & period);
  bool perform_handshake();
  bool write_command_if_needed();
  static ::speed_t baud_rate_constant(int baud_rate);

  int serial_fd_{-1};
  std::string serial_port_{"/dev/ttyACM0"};
  int baud_rate_{115200};
  double command_tolerance_rad_{1.0e-4};
  double state_poll_period_sec_{0.02};
  double min_command_period_sec_{0.15};
  std::int64_t min_step_delta_to_resend_{40};
  rclcpp::Time last_state_poll_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_command_send_time_{0, 0, RCL_ROS_TIME};
  bool has_polled_once_{false};
  bool has_sent_motion_command_{false};

  std::array<double, kJointCount> steps_per_radian_{};
  std::array<double, kJointCount> hw_states_position_{};
  std::array<double, kJointCount> hw_states_velocity_{};
  std::array<double, kJointCount> hw_commands_position_{};
  std::array<double, kJointCount> last_sent_commands_position_{};
  std::array<std::int64_t, kJointCount> current_steps_{};
  std::array<std::int64_t, kJointCount> target_steps_{};
  std::array<std::int64_t, kJointCount> previous_steps_{};
  bool motion_pending_{false};
  int unchanged_state_reads_{0};
};

}  // namespace robot_arm_hardware
