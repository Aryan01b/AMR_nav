# Autonomous Mobile Robot Navigation with ROS2 and Gazebo Harmonic

## Overview
This project implements an autonomous mobile robot (AMR) navigation system using ROS2 Humble and Gazebo Harmonic. The system features a differential-drive robot equipped with LiDAR, IMU, and depth camera, capable of navigating to multiple goals while avoiding static obstacles using the Nav2 navigation stack.

## System Requirements
- **OS**: Ubuntu 22.04
- **ROS2 Distribution**: Humble Hawksbill
- **Gazebo**: Harmonic (source build required)
- **Python**: 3.8+
- **Git**

## Installation

### 1. Install Dependencies
Follow the official Gazebo Harmonic installation guide for ROS2 Humble:
[Gazebo Harmonic with ROS 2 Humble Installation Guide](https://gazebosim.org/docs/harmonic/ros_installation#gazebo-harmonic-with-ros-2-humble)

### 2. Clone and Build the Workspace
```bash
git clone https://github.com/Aryan01b/AMR_nav.git
cd AMR_nav/assignment_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Usage

### Launch the Navigation Stack
```bash
# Source the workspace
source ~/AMR_nav/assignment_ws/install/setup.bash

# Launch the full navigation stack
ros2 launch amr_bringup full_nav.launch.py
```

### Sending Navigation Goals
Use the `send_goal` node to command the robot to navigate to specific positions. Examples:

```bash
# Basic navigation (x, y with yaw=0)
ros2 run amr_nav2 send_goal --ros-args -p x:=2.0 -p y:=1.0

# With orientation
ros2 run amr_nav2 send_goal --ros-args -p x:=2.0 -p y:=1.0 -p yaw:=1.57
```
