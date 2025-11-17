# Autonomous Mobile Robot Navigation

## Overview
This project implements an autonomous mobile robot (AMR) navigation system using ROS2 and Gazebo. The system features a differential-drive robot equipped with LiDAR, IMU, and a depth camera, capable of navigating to multiple goals while avoiding both static obstacles using the Nav2 navigation stack.

## Features
- **Robot Model**: Differential-drive robot with realistic physics
- **Sensors**:
  - 360° LiDAR for obstacle detection
  - IMU for orientation and acceleration data
  - Depth camera for 3D environment perception
- **Navigation**:
  - Autonomous goal navigation
  - Dynamic obstacle avoidance
  - Path planning and execution
- **Simulation**:
  - Gazebo-based simulation environment
  - Customizable world with obstacles
  - Realistic physics simulation

## Prerequisites
- Ubuntu 22.04
- ROS2 Humble Hawksbill
- Gazebo Harmonic
- Python 3.8+
- Git

## Installation

### 1. Set up ROS2 Humble
```bash
# Set up locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe

# Install ROS2 Humble
sudo apt update && sudo apt install -y \
  ros-humble-desktop \
  ros-humble-nav2-bringup \
  ros-humble-turtlebot3* \
  python3-colcon-common-extensions

# Source ROS2 setup
source /opt/ros/humble/setup.bash
```

### 2. Clone the Repository
```bash
mkdir -p ~/amr_ws/src
cd ~/amr_ws/src
git clone <repository-url>
cd ~/amr_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the Workspace
```bash
cd ~/amr_ws
colcon build --symlink-install
source install/setup.bash
```

## Usage

### Launch the Simulation
```bash
# Source the workspace
source ~/amr_ws/install/setup.bash

# Launch the simulation
ros2 launch amr_navigation simulation.launch.py
```

### Sending Navigation Goals
```bash
# In a new terminal
source ~/amr_ws/install/setup.bash
ros2 run amr_navigation goal_sender
```

### Using RViz for Visualization
```bash
ros2 launch nav2_bringup rviz_launch.py
```

## Project Structure
```
amr_ws/
├── src/
│   ├── amr_description/      # Robot URDF and meshes
│   ├── amr_gazebo/          # Gazebo plugins and worlds
│   ├── amr_navigation/      # Navigation configuration and launch files
│   └── amr_teleop/          # Teleoperation nodes
```

## Demo Video
A demo video showing the robot's navigation capabilities is available [here](./demo.mp4).

## Development Timeline
- **Task 1 (4 hrs)**: Set up ROS2 workspace and basic robot model
- **Task 2 (6 hrs)**: Implement sensor configurations and Gazebo integration
- **Task 3 (8 hrs)**: Configure Nav2 stack and implement navigation
- **Task 4 (4 hrs)**: Develop goal sender and test navigation
- **Task 5 (3 hrs)**: Documentation and final testing

## Troubleshooting
- **Gazebo not launching**: Ensure hardware acceleration is enabled
- **Navigation issues**: Check TF tree and sensor data in RViz
- **Build errors**: Run `rosdep install` to install missing dependencies

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments
- ROS2 Navigation2 team
- Gazebo Simulator
- Open Robotics
