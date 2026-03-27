cmake_minimum_required(VERSION 3.8)
project(move_joint5_xyz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(move_joint5_xyz_node src/move_joint5_xyz.cpp)
add_executable(move_joint5_vertical_circle_node src/move_joint5_vertical_circle.cpp)

ament_target_dependencies(move_joint5_xyz_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

ament_target_dependencies(move_joint5_vertical_circle_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

install(TARGETS
  move_joint5_xyz_node
  move_joint5_vertical_circle_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
