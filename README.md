# diffdrive_explore
## Prerequisites
+ ROS2 Foxy
+ Webots R2022a https://github.com/cyberbotics/webots
+ webots_ros2 package 1.2.2 https://github.com/cyberbotics/webots_ros2
## Build
```Bash
source /opt/ros/$ROS_DISTRO/local_setup.bash

# Retrieve the sources
cd /path/to/ros2_ws
git clone --recurse-submodules https://github.com/cyberbotics/webots_ros2.git src/webots_ros2
git clone https://github.com/TaoYibo1866/diffdrive_explore.git src/diffdrive_explore
# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO

# Building packages
colcon build

# Source this workspace (careful when also sourcing others)
source install/local_setup.bash
```
## Run
```Bash
ros2 launch diffdrive_webots localization.launch.py
ros2 launch ground_control ground_control.launch.py
```