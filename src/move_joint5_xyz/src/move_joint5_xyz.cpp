#include <memory>
#include <string>
#include <cstdlib>
#include <chrono>
#include <cmath>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/parameter_client.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "moveit_msgs/msg/move_it_error_codes.hpp"

namespace
{

std::string moveItErrorCodeToString(const moveit::core::MoveItErrorCode & error_code)
{
  switch (error_code.val)
  {
    case moveit_msgs::msg::MoveItErrorCodes::SUCCESS:
      return "SUCCESS";
    case moveit_msgs::msg::MoveItErrorCodes::FAILURE:
      return "FAILURE";
    case moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED:
      return "PLANNING_FAILED";
    case moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN:
      return "INVALID_MOTION_PLAN";
    case moveit_msgs::msg::MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
      return "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE";
    case moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED:
      return "CONTROL_FAILED";
    case moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT:
      return "TIMED_OUT";
    case moveit_msgs::msg::MoveItErrorCodes::PREEMPTED:
      return "PREEMPTED";
    case moveit_msgs::msg::MoveItErrorCodes::START_STATE_IN_COLLISION:
      return "START_STATE_IN_COLLISION";
    case moveit_msgs::msg::MoveItErrorCodes::START_STATE_VIOLATES_PATH_CONSTRAINTS:
      return "START_STATE_VIOLATES_PATH_CONSTRAINTS";
    case moveit_msgs::msg::MoveItErrorCodes::GOAL_IN_COLLISION:
      return "GOAL_IN_COLLISION";
    case moveit_msgs::msg::MoveItErrorCodes::GOAL_VIOLATES_PATH_CONSTRAINTS:
      return "GOAL_VIOLATES_PATH_CONSTRAINTS";
    case moveit_msgs::msg::MoveItErrorCodes::GOAL_CONSTRAINTS_VIOLATED:
      return "GOAL_CONSTRAINTS_VIOLATED";
    case moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME:
      return "INVALID_GROUP_NAME";
    case moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS:
      return "INVALID_GOAL_CONSTRAINTS";
    case moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE:
      return "INVALID_ROBOT_STATE";
    case moveit_msgs::msg::MoveItErrorCodes::INVALID_LINK_NAME:
      return "INVALID_LINK_NAME";
    case moveit_msgs::msg::MoveItErrorCodes::INVALID_OBJECT_NAME:
      return "INVALID_OBJECT_NAME";
    case moveit_msgs::msg::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE:
      return "FRAME_TRANSFORM_FAILURE";
    case moveit_msgs::msg::MoveItErrorCodes::COLLISION_CHECKING_UNAVAILABLE:
      return "COLLISION_CHECKING_UNAVAILABLE";
    case moveit_msgs::msg::MoveItErrorCodes::ROBOT_STATE_STALE:
      return "ROBOT_STATE_STALE";
    case moveit_msgs::msg::MoveItErrorCodes::SENSOR_INFO_STALE:
      return "SENSOR_INFO_STALE";
    case moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION:
      return "NO_IK_SOLUTION";
    default:
      return "UNKNOWN";
  }
}

bool parseCoordinate(const char * text, const char * axis_name, double & value, std::string & error_message)
{
  try
  {
    std::size_t parsed_characters = 0;
    value = std::stod(text, &parsed_characters);
    if (parsed_characters != std::string(text).size())
    {
      error_message = std::string("Invalid ") + axis_name + " value: '" + text + "'";
      return false;
    }
    if (!std::isfinite(value))
    {
      error_message = std::string("Non-finite ") + axis_name + " value: '" + text + "'";
      return false;
    }
    return true;
  }
  catch (const std::exception &)
  {
    error_message = std::string("Invalid ") + axis_name + " value: '" + text + "'";
    return false;
  }
}

bool loadParameterFromMoveGroup(
  const std::shared_ptr<rclcpp::SyncParametersClient> & parameter_client,
  const std::string & parameter_name,
  const rclcpp::Logger & logger,
  std::vector<rclcpp::Parameter> & parameter_overrides)
{
  if (!parameter_client->wait_for_service(std::chrono::seconds(3)))
  {
    RCLCPP_WARN(
      logger,
      "Parameter service on /move_group not available; continuing without '%s'.",
      parameter_name.c_str());
    return false;
  }

  auto parameters = parameter_client->get_parameters({parameter_name});
  if (parameters.empty())
  {
    RCLCPP_WARN(logger, "Parameter '%s' was not returned by /move_group.", parameter_name.c_str());
    return false;
  }

  const auto & parameter = parameters.front();
  if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING || parameter.as_string().empty())
  {
    RCLCPP_WARN(logger, "Parameter '%s' from /move_group is empty or not a string.", parameter_name.c_str());
    return false;
  }

  parameter_overrides.emplace_back(parameter_name, parameter.as_string());
  RCLCPP_INFO(logger, "Loaded '%s' from /move_group.", parameter_name.c_str());
  return true;
}

bool loadParameterPrefixFromMoveGroup(
  const std::shared_ptr<rclcpp::SyncParametersClient> & parameter_client,
  const std::string & parameter_prefix,
  const rclcpp::Logger & logger,
  std::vector<rclcpp::Parameter> & parameter_overrides)
{
  if (!parameter_client->wait_for_service(std::chrono::seconds(3)))
  {
    RCLCPP_WARN(
      logger,
      "Parameter service on /move_group not available; continuing without prefix '%s'.",
      parameter_prefix.c_str());
    return false;
  }

  const auto listed_parameters = parameter_client->list_parameters({parameter_prefix}, 10);
  if (listed_parameters.names.empty())
  {
    RCLCPP_WARN(logger, "Parameter prefix '%s' was not returned by /move_group.", parameter_prefix.c_str());
    return false;
  }

  const auto prefixed_parameters = parameter_client->get_parameters(listed_parameters.names);
  if (prefixed_parameters.empty())
  {
    RCLCPP_WARN(logger, "Parameters under prefix '%s' could not be fetched from /move_group.", parameter_prefix.c_str());
    return false;
  }

  for (const auto & parameter : prefixed_parameters)
  {
    parameter_overrides.emplace_back(parameter.get_name(), parameter.get_parameter_value());
  }

  RCLCPP_INFO(logger, "Loaded '%s.*' from /move_group.", parameter_prefix.c_str());
  return true;
}

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  if (argc != 4)
  {
    std::cerr << "Usage:\n"
              << "  ros2 run move_joint5_xyz move_joint5_xyz_node X Y Z\n\n"
              << "Example:\n"
              << "  ros2 run move_joint5_xyz move_joint5_xyz_node 0.30 0.10 0.40\n";
    rclcpp::shutdown();
    return 1;
  }

  double target_x = 0.0;
  double target_y = 0.0;
  double target_z = 0.0;
  std::string parse_error_message;
  if (!parseCoordinate(argv[1], "X", target_x, parse_error_message) ||
      !parseCoordinate(argv[2], "Y", target_y, parse_error_message) ||
      !parseCoordinate(argv[3], "Z", target_z, parse_error_message))
  {
    std::cerr << parse_error_message << "\n";
    rclcpp::shutdown();
    return 1;
  }

  auto parameter_node = rclcpp::Node::make_shared(
    "move_joint5_xyz_param_loader",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto logger = rclcpp::get_logger("move_joint5_xyz");

  const std::string planning_group =
    parameter_node->declare_parameter<std::string>("planning_group", "arm");
  const std::string target_link =
    parameter_node->declare_parameter<std::string>("target_link", "Joint_5_Link");

  auto move_group_parameters =
    std::make_shared<rclcpp::SyncParametersClient>(parameter_node, "/move_group");
  std::vector<rclcpp::Parameter> parameter_overrides;
  parameter_overrides.emplace_back("planning_group", planning_group);
  parameter_overrides.emplace_back("target_link", target_link);

  const bool has_robot_description =
    loadParameterFromMoveGroup(
      move_group_parameters, "robot_description", logger, parameter_overrides);
  const bool has_robot_description_semantic =
    loadParameterFromMoveGroup(
      move_group_parameters, "robot_description_semantic", logger, parameter_overrides);
  const bool has_robot_description_kinematics =
    loadParameterPrefixFromMoveGroup(
      move_group_parameters, "robot_description_kinematics", logger, parameter_overrides);

  if (!has_robot_description || !has_robot_description_semantic)
  {
    RCLCPP_ERROR(
      logger,
      "Missing MoveIt robot description parameters. Start the MoveIt demo first, "
      "for example: ros2 launch my_robot_moveit_config demo.launch.py");
    rclcpp::shutdown();
    return 1;
  }

  if (!has_robot_description_kinematics)
  {
    RCLCPP_WARN(
      logger,
      "Kinematics parameters were not available from /move_group. Planning may still work, "
      "but IK-based planning can be limited.");
  }

  auto node = rclcpp::Node::make_shared(
    "move_joint5_xyz",
    rclcpp::NodeOptions()
      .automatically_declare_parameters_from_overrides(true)
      .parameter_overrides(parameter_overrides));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread executor_thread([&executor]() { executor.spin(); });
  const auto shutdown_node = [&executor, &executor_thread]() {
    executor.cancel();
    if (executor_thread.joinable())
    {
      executor_thread.join();
    }
    rclcpp::shutdown();
  };

  moveit::planning_interface::MoveGroupInterface move_group(node, planning_group);

  // Planner settings
  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(10);
  move_group.setMaxVelocityScalingFactor(0.3);
  move_group.setMaxAccelerationScalingFactor(0.3);

  // Start state monitor and wait a little so MoveIt receives /joint_states.
  move_group.startStateMonitor();
  rclcpp::sleep_for(std::chrono::seconds(2));

  auto current_state = move_group.getCurrentState(5.0);
  if (!current_state)
  {
    RCLCPP_ERROR(logger, "Failed to get current robot state.");
    shutdown_node();
    return 1;
  }

  // Read current pose of the requested link.
  geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose(target_link).pose;

  const double current_x = current_pose.position.x;
  const double current_y = current_pose.position.y;
  const double current_z = current_pose.position.z;

  const double dx = target_x - current_x;
  const double dy = target_y - current_y;
  const double dz = target_z - current_z;

  RCLCPP_INFO(logger, "----------------------------------------");
  RCLCPP_INFO(logger, "Planning group: %s", planning_group.c_str());
  RCLCPP_INFO(logger, "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "End effector link: %s", move_group.getEndEffectorLink().c_str());
  RCLCPP_INFO(logger, "Target link: %s", target_link.c_str());
  RCLCPP_INFO(logger, "----------------------------------------");
  RCLCPP_INFO(logger, "Current XYZ:");
  RCLCPP_INFO(logger, "  X: %.4f  Y: %.4f  Z: %.4f", current_x, current_y, current_z);
  RCLCPP_INFO(logger, "Target XYZ:");
  RCLCPP_INFO(logger, "  X: %.4f  Y: %.4f  Z: %.4f", target_x, target_y, target_z);
  RCLCPP_INFO(logger, "Delta XYZ:");
  RCLCPP_INFO(logger, "  dX: %.4f  dY: %.4f  dZ: %.4f", dx, dy, dz);
  RCLCPP_INFO(logger, "----------------------------------------");

  // Clear old targets and plan from the real current state.
  move_group.clearPoseTargets();
  move_group.setStartStateToCurrentState();

  // Set only XYZ target for this link.
  if (!move_group.setPositionTarget(target_x, target_y, target_z, target_link))
  {
    RCLCPP_ERROR(logger, "Failed to set position target.");
    shutdown_node();
    return 1;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto plan_result = move_group.plan(plan);

  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(
      logger, "Planning failed with code %s (%d).",
      moveItErrorCodeToString(plan_result).c_str(), plan_result.val);
    shutdown_node();
    return 1;
  }

  RCLCPP_INFO(logger, "Plan found. Executing...");
  auto exec_result = move_group.execute(plan);

  if (exec_result != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(
      logger, "Execution failed with code %s (%d).",
      moveItErrorCodeToString(exec_result).c_str(), exec_result.val);
    shutdown_node();
    return 1;
  }

  RCLCPP_INFO(logger, "Execution successful.");

  // Read final pose again after execution
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  auto final_state = move_group.getCurrentState(5.0);
  if (final_state)
  {
    geometry_msgs::msg::Pose final_pose = move_group.getCurrentPose(target_link).pose;
    RCLCPP_INFO(logger, "Final XYZ:");
    RCLCPP_INFO(logger, "  X: %.4f  Y: %.4f  Z: %.4f",
                final_pose.position.x,
                final_pose.position.y,
                final_pose.position.z);
  }

  shutdown_node();
  return 0;
}
