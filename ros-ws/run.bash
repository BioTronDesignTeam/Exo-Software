#!/bin/bash

# Build the package
colcon build --packages-select bringup_robot

# Source the setup file
source install/setup.bash

# Launch RViz
ros2 launch bringup_robot rviz.launch.py
