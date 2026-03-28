#include "robot_arm_hardware/robot_arm_system.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <sstream>
#include <limits>
#include <string>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/logger.hpp"

namespace robot_arm_hardware
{

namespace
{
constexpr double kTwoPi = 6.28318530717958647692;

bool try_parse_step_token(const std::string & token, std::int64_t & value)
{
  if (token.empty()) {
    return false;
  }

  std::size_t consumed = 0;
  try
  {
    value = std::stoll(token, &consumed);
  }
  catch (const std::exception &)
  {
    return false;
  }

  return consumed == token.size();
}

bool line_contains_state_payload(const std::string & line)
{
  std::istringstream stream(line);
  std::string token;
  std::size_t parsed_count = 0;
  while (stream >> token)
  {
    std::int64_t ignored_value = 0;
    if (try_parse_step_token(token, ignored_value))
    {
      ++parsed_count;
      if (parsed_count >= 5)
      {
        return true;
      }
    }
  }

  return line.find("STATE") != std::string::npos;
}
}

hardware_interface::CallbackReturn RobotArmSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != kJointCount)
  {
    RCLCPP_ERROR(
      get_logger(), "Expected %zu joints but URDF exposed %zu.", kJointCount, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto serial_it = info_.hardware_parameters.find("serial_port");
  if (serial_it != info_.hardware_parameters.end())
  {
    serial_port_ = serial_it->second;
  }

  auto baud_it = info_.hardware_parameters.find("baud_rate");
  if (baud_it != info_.hardware_parameters.end())
  {
    baud_rate_ = std::stoi(baud_it->second);
  }

  auto tolerance_it = info_.hardware_parameters.find("command_tolerance_rad");
  if (tolerance_it != info_.hardware_parameters.end())
  {
    command_tolerance_rad_ = std::stod(tolerance_it->second);
  }

  auto poll_it = info_.hardware_parameters.find("state_poll_period_sec");
  if (poll_it != info_.hardware_parameters.end())
  {
    state_poll_period_sec_ = std::stod(poll_it->second);
  }

  auto min_command_period_it = info_.hardware_parameters.find("min_command_period_sec");
  if (min_command_period_it != info_.hardware_parameters.end())
  {
    min_command_period_sec_ = std::stod(min_command_period_it->second);
  }

  auto min_step_delta_it = info_.hardware_parameters.find("min_step_delta_to_resend");
  if (min_step_delta_it != info_.hardware_parameters.end())
  {
    min_step_delta_to_resend_ = std::stoll(min_step_delta_it->second);
  }

  for (std::size_t index = 0; index < info_.joints.size(); ++index)
  {
    const auto & joint = info_.joints[index];
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(
        get_logger(), "Joint %s must expose exactly one position command interface.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2 ||
        joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
        joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_ERROR(
        get_logger(), "Joint %s must expose position and velocity state interfaces.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    auto steps_it = joint.parameters.find("steps_per_radian");
    if (steps_it == joint.parameters.end())
    {
      RCLCPP_ERROR(
        get_logger(), "Joint %s is missing required steps_per_radian parameter.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    steps_per_radian_[index] = std::stod(steps_it->second);
    hw_states_position_[index] = 0.0;
    hw_states_velocity_[index] = 0.0;
    hw_commands_position_[index] = 0.0;
    last_sent_commands_position_[index] = std::numeric_limits<double>::quiet_NaN();
    current_steps_[index] = 0;
    target_steps_[index] = 0;
    previous_steps_[index] = 0;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotArmSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  close_serial();

  if (!open_serial())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    get_logger(), "Opened serial connection on %s at %d baud.",
    serial_port_.c_str(), baud_rate_);

  if (!perform_handshake())
  {
    close_serial();
    return hardware_interface::CallbackReturn::ERROR;
  }

  has_polled_once_ = false;
  unchanged_state_reads_ = 0;
  motion_pending_ = false;
  has_sent_motion_command_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotArmSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (serial_fd_ < 0 && !open_serial())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  tcflush(serial_fd_, TCIFLUSH);

  if (!send_line("ENABLE\n"))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::string response;
  if (!read_matching_line(
        response, 500, [](const std::string & line) { return line.find("OK") != std::string::npos; }))
  {
    RCLCPP_ERROR(get_logger(), "Failed to enable Arduino outputs.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (std::size_t index = 0; index < kJointCount; ++index)
  {
    hw_commands_position_[index] = hw_states_position_[index];
    last_sent_commands_position_[index] = std::numeric_limits<double>::quiet_NaN();
    set_command(info_.joints[index].name + "/" + hardware_interface::HW_IF_POSITION, hw_states_position_[index]);
  }

  last_command_send_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  has_sent_motion_command_ = false;

  RCLCPP_INFO(get_logger(), "Arduino outputs enabled.");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotArmSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (serial_fd_ >= 0)
  {
    send_line("DISABLE\n");
    std::string ignored;
    read_line(ignored, 200);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotArmSystem::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  close_serial();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobotArmSystem::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (serial_fd_ < 0)
  {
    return hardware_interface::return_type::ERROR;
  }

  if (!has_polled_once_ ||
      (time - last_state_poll_time_).seconds() >= state_poll_period_sec_)
  {
    if (!request_state())
    {
      return hardware_interface::return_type::ERROR;
    }

    std::string line;
    if (!read_matching_line(
          line, 150, [](const std::string & response) { return line_contains_state_payload(response); }) ||
        !parse_state_line(line, period))
    {
      RCLCPP_ERROR(get_logger(), "Failed to read valid STATE line from Arduino.");
      return hardware_interface::return_type::ERROR;
    }

    last_state_poll_time_ = time;
    has_polled_once_ = true;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotArmSystem::write(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (serial_fd_ < 0)
  {
    return hardware_interface::return_type::ERROR;
  }

  if (last_command_send_time_.get_clock_type() != time.get_clock_type())
  {
    last_command_send_time_ = rclcpp::Time(0, 0, time.get_clock_type());
  }

  return write_command_if_needed() ? hardware_interface::return_type::OK
                                   : hardware_interface::return_type::ERROR;
}

bool RobotArmSystem::open_serial()
{
  serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (serial_fd_ < 0)
  {
    RCLCPP_ERROR(
      get_logger(), "Unable to open serial port %s: %s", serial_port_.c_str(), strerror(errno));
    return false;
  }

  if (!configure_serial_port(serial_fd_))
  {
    close_serial();
    return false;
  }

  tcflush(serial_fd_, TCIOFLUSH);
  usleep(2000000);
  return true;
}

void RobotArmSystem::close_serial()
{
  if (serial_fd_ >= 0)
  {
    close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool RobotArmSystem::configure_serial_port(int fd) const
{
  termios tty{};
  if (tcgetattr(fd, &tty) != 0)
  {
    RCLCPP_ERROR(get_logger(), "tcgetattr failed: %s", strerror(errno));
    return false;
  }

  cfmakeraw(&tty);
  const auto baud = baud_rate_constant(baud_rate_);
  cfsetispeed(&tty, baud);
  cfsetospeed(&tty, baud);

  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd, TCSANOW, &tty) != 0)
  {
    RCLCPP_ERROR(get_logger(), "tcsetattr failed: %s", strerror(errno));
    return false;
  }

  return true;
}

bool RobotArmSystem::send_line(const std::string & line)
{
  const auto bytes_written = ::write(serial_fd_, line.c_str(), line.size());
  if (bytes_written < 0 || static_cast<std::size_t>(bytes_written) != line.size())
  {
    RCLCPP_ERROR(get_logger(), "Serial write failed: %s", strerror(errno));
    return false;
  }

  tcdrain(serial_fd_);
  return true;
}

bool RobotArmSystem::read_line(std::string & line, int timeout_ms)
{
  line.clear();
  std::array<char, 1> ch{};

  while (true)
  {
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(serial_fd_, &readfds);

    timeval timeout{};
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;

    const int ready = select(serial_fd_ + 1, &readfds, nullptr, nullptr, &timeout);
    if (ready < 0)
    {
      RCLCPP_ERROR(get_logger(), "select failed: %s", strerror(errno));
      return false;
    }
    if (ready == 0)
    {
      return false;
    }

    const auto bytes_read = ::read(serial_fd_, ch.data(), ch.size());
    if (bytes_read < 0)
    {
      RCLCPP_ERROR(get_logger(), "Serial read failed: %s", strerror(errno));
      return false;
    }
    if (bytes_read == 0)
    {
      continue;
    }

    if (ch[0] == '\n')
    {
      if (!line.empty() && line.back() == '\r')
      {
        line.pop_back();
      }
      return !line.empty();
    }

    line.push_back(ch[0]);
  }
}

bool RobotArmSystem::read_matching_line(
  std::string & line, int timeout_ms, const std::function<bool(const std::string &)> & predicate)
{
  constexpr int slice_ms = 50;
  int elapsed_ms = 0;

  while (elapsed_ms < timeout_ms)
  {
    const int wait_ms = std::min(slice_ms, timeout_ms - elapsed_ms);
    std::string candidate;
    if (!read_line(candidate, wait_ms))
    {
      elapsed_ms += wait_ms;
      continue;
    }

    if (predicate(candidate))
    {
      line = candidate;
      return true;
    }

    RCLCPP_WARN(
      get_logger(), "Ignoring unexpected serial response while waiting for match: '%s'",
      candidate.c_str());
  }

  return false;
}

bool RobotArmSystem::request_state()
{
  return send_line("STATE\n");
}

bool RobotArmSystem::parse_state_line(
  const std::string & line, const rclcpp::Duration & period)
{
  std::array<std::int64_t, kJointCount> new_steps{};

  std::istringstream stream(line);
  std::string token;
  std::size_t parsed_count = 0;
  while (stream >> token)
  {
    std::int64_t parsed_value = 0;
    if (!try_parse_step_token(token, parsed_value))
    {
      continue;
    }

    if (parsed_count < kJointCount)
    {
      new_steps[parsed_count++] = parsed_value;
    }
  }

  if (parsed_count != kJointCount)
  {
    return false;
  }

  const double dt = std::max(period.seconds(), 1.0e-6);
  bool any_step_changed = false;
  for (std::size_t index = 0; index < kJointCount; ++index)
  {
    const double previous_position = hw_states_position_[index];
    previous_steps_[index] = current_steps_[index];
    current_steps_[index] = new_steps[index];
    hw_states_position_[index] = static_cast<double>(current_steps_[index]) / steps_per_radian_[index];
    hw_states_velocity_[index] = (hw_states_position_[index] - previous_position) / dt;
    set_state(
      info_.joints[index].name + "/" + hardware_interface::HW_IF_POSITION, hw_states_position_[index]);
    set_state(
      info_.joints[index].name + "/" + hardware_interface::HW_IF_VELOCITY, hw_states_velocity_[index]);
    any_step_changed = any_step_changed || (current_steps_[index] != previous_steps_[index]);
  }

  if (motion_pending_)
  {
    if (current_steps_ == target_steps_)
    {
      motion_pending_ = false;
      unchanged_state_reads_ = 0;
      RCLCPP_INFO(get_logger(), "Arduino reported that all joints reached the requested target.");
    }
    else if (any_step_changed)
    {
      unchanged_state_reads_ = 0;
    }
    else
    {
      ++unchanged_state_reads_;
      if (unchanged_state_reads_ == 25)
      {
        RCLCPP_WARN(
          get_logger(),
          "A MOVE command was accepted, but Arduino STATE has not changed for ~%.2f s. "
          "Check motor power, driver enable pins, wiring, and whether the uploaded board matches RobotArm1.ino pin usage.",
          unchanged_state_reads_ * state_poll_period_sec_);
      }
    }
  }

  return true;
}

bool RobotArmSystem::perform_handshake()
{
  if (!send_line("PING\n"))
  {
    return false;
  }

  std::string line;
  if (!read_matching_line(
        line, 1000,
        [](const std::string & response) { return response.find("PONG") != std::string::npos; }))
  {
    RCLCPP_ERROR(get_logger(), "Arduino did not answer PING.");
    return false;
  }

  if (!request_state())
  {
    return false;
  }

  if (!read_matching_line(
        line, 500, [](const std::string & response) { return line_contains_state_payload(response); }) ||
      !parse_state_line(line, rclcpp::Duration::from_seconds(0.02)))
  {
    RCLCPP_ERROR(get_logger(), "Unable to fetch initial state from Arduino.");
    return false;
  }

  RCLCPP_INFO(
    get_logger(), "Handshake complete. Initial Arduino state: [%ld, %ld, %ld, %ld, %ld]",
    static_cast<long>(current_steps_[0]), static_cast<long>(current_steps_[1]),
    static_cast<long>(current_steps_[2]), static_cast<long>(current_steps_[3]),
    static_cast<long>(current_steps_[4]));

  return true;
}

bool RobotArmSystem::write_command_if_needed()
{
  bool changed = false;
  std::array<std::int64_t, kJointCount> desired_steps{};

  for (std::size_t index = 0; index < kJointCount; ++index)
  {
    hw_commands_position_[index] =
      get_command<double>(info_.joints[index].name + "/" + hardware_interface::HW_IF_POSITION);

    if (!std::isfinite(hw_commands_position_[index]))
    {
      hw_commands_position_[index] = hw_states_position_[index];
      set_command(
        info_.joints[index].name + "/" + hardware_interface::HW_IF_POSITION,
        hw_commands_position_[index]);
    }

    desired_steps[index] = static_cast<std::int64_t>(
      std::llround(hw_commands_position_[index] * steps_per_radian_[index]));

    if (!std::isfinite(last_sent_commands_position_[index]) ||
        std::fabs(hw_commands_position_[index] - last_sent_commands_position_[index]) >
          command_tolerance_rad_)
    {
      changed = true;
    }
  }

  if (!changed)
  {
    return true;
  }

  bool large_enough_step_change = !has_sent_motion_command_;
  for (std::size_t index = 0; index < kJointCount; ++index)
  {
    if (std::llabs(desired_steps[index] - target_steps_[index]) >= min_step_delta_to_resend_)
    {
      large_enough_step_change = true;
      break;
    }
  }

  const bool command_period_elapsed =
    !has_sent_motion_command_ ||
    (get_clock()->now() - last_command_send_time_).seconds() >= min_command_period_sec_;

  if (!large_enough_step_change && !command_period_elapsed)
  {
    return true;
  }

  std::ostringstream command;
  command << "MOVE";
  for (const auto step : desired_steps)
  {
    command << ' ' << step;
  }
  command << '\n';

  if (!send_line(command.str()))
  {
    return false;
  }

  std::string response;
  if (!read_matching_line(
        response, 100, [](const std::string & line) { return line.find("OK") != std::string::npos; }))
  {
    RCLCPP_ERROR(get_logger(), "Arduino did not acknowledge MOVE.");
    return false;
  }

  target_steps_ = desired_steps;
  last_sent_commands_position_ = hw_commands_position_;
  motion_pending_ = true;
  unchanged_state_reads_ = 0;
  last_command_send_time_ = get_clock()->now();
  has_sent_motion_command_ = true;
  RCLCPP_INFO(
    get_logger(), "Sent MOVE target steps: [%ld, %ld, %ld, %ld, %ld]",
    static_cast<long>(target_steps_[0]), static_cast<long>(target_steps_[1]),
    static_cast<long>(target_steps_[2]), static_cast<long>(target_steps_[3]),
    static_cast<long>(target_steps_[4]));
  return true;
}

::speed_t RobotArmSystem::baud_rate_constant(int baud_rate)
{
  switch (baud_rate)
  {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    default:
      return B115200;
  }
}

}  // namespace robot_arm_hardware

PLUGINLIB_EXPORT_CLASS(robot_arm_hardware::RobotArmSystem, hardware_interface::SystemInterface)
