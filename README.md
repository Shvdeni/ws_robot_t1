# ws_robot_t1

ROS 2 Jazzy workspace for a 5-axis robot arm with:

- a URDF and mesh description package
- a MoveIt configuration package
- a custom `ros2_control` hardware interface that talks to an Arduino over serial
- example motion nodes for moving the end effector in Cartesian space

## Workspace Layout

```text
ws_robot_t1/
├── src/
│   ├── my_robot_description/     # URDF, meshes, RViz display launch
│   ├── my_robot_moveit_config/   # MoveIt config and launch files
│   ├── move_joint5_xyz/          # Example Cartesian motion nodes
│   └── robot_arm_hardware/       # Serial ros2_control hardware plugin
├── RobotArm1.ino                 # Arduino firmware
└── Joint1_test.ino               # Arduino test sketch
```

## Packages

### `my_robot_description`

Contains the robot model, STL meshes, and a simple RViz display launch.

### `my_robot_moveit_config`

Contains the MoveIt setup, controller configuration, RViz config, and launch files for:

- simulation/demo mode
- real hardware mode through `ros2_control`

### `robot_arm_hardware`

Provides the `robot_arm_hardware/RobotArmSystem` plugin, which opens a serial connection to the Arduino and exposes the arm to `ros2_control`.

### `move_joint5_xyz`

Provides example executables:

- `move_joint5_xyz_node` to move the end effector to a target `X Y Z`
- `move_joint5_vertical_circle_node` to execute a vertical circle path

## Requirements

- Ubuntu or WSL with ROS 2 Jazzy installed
- `colcon`
- MoveIt 2 for ROS 2 Jazzy
- RViz 2
- an Arduino-connected 5-axis arm for real hardware mode

## Build

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ws_robot_t1
colcon build --symlink-install
source install/setup.bash
```

To build only the Cartesian motion package:

```bash
cd ~/ws_robot_t1
colcon build --packages-select move_joint5_xyz
source install/setup.bash
```

## Visualize The Robot Model

Launch the description package in RViz:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_robot_t1/install/setup.bash
ros2 launch my_robot_description display.launch.py
```

## Run MoveIt Demo

This starts the arm in mock hardware mode for planning and testing:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_robot_t1/install/setup.bash
ros2 launch my_robot_moveit_config demo.launch.py
```

## Run On Real Hardware

The real hardware launch uses the custom `robot_arm_hardware` plugin and passes serial settings into the robot description.

Before launching, make sure the Arduino is connected and visible as a serial device such as `/dev/ttyACM0`.

```bash
cd ~/ws_robot_t1
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/ros_logs_real_hw
ros2 launch my_robot_moveit_config real_hardware.launch.py serial_port:=/dev/ttyACM0
```

Optional launch arguments:

- `serial_port:=/dev/ttyACM0`
- `baud_rate:=115200`
- `launch_rviz:=true`

If old ROS processes are still running, these cleanup commands can help:

```bash
pkill -f /opt/ros/jazzy/lib/controller_manager/ros2_control_node
pkill -f /opt/ros/jazzy/lib/moveit_ros_move_group/move_group
pkill -f /opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher
```

## WSL USB Serial Notes

If you are using WSL and the Arduino is attached over USB, you may need to expose the device first:

```bash
usbipd list
usbipd bind --busid <BUSID>
usbipd attach --wsl --busid <BUSID>
ls /dev/ttyACM*
```

## Example Motion Commands

Start MoveIt first, then run one of the example nodes.

Move the end effector to a target Cartesian position:

```bash
source ~/ws_robot_t1/install/setup.bash
ros2 run move_joint5_xyz move_joint5_xyz_node 0.30 0.10 0.40
```

Example from the package notes:

```bash
source ~/ws_robot_t1/install/setup.bash
ros2 run move_joint5_xyz move_joint5_xyz_node -0.30 0.05 0.30
```

Run the vertical circle motion:

```bash
source ~/ws_robot_t1/install/setup.bash
ros2 run move_joint5_xyz move_joint5_vertical_circle_node
```

## Arduino Files

- `RobotArm1.ino` appears to be the main Arduino firmware for the arm
- `Joint1_test.ino` appears to be a smaller test sketch

Upload the correct sketch to the Arduino before using real hardware mode.

## Notes

- The MoveIt example nodes expect `/move_group` to already be running.
- The real hardware setup is configured for a 5-joint arm and serial communication at `115200` baud by default.
- The end-effector target link used by the example nodes is `Joint_5_Link`.
