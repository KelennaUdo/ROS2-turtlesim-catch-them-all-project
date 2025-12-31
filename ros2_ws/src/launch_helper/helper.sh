#!/bin/bash
# Shebang line: specifies this script should be executed with bash

# Change to the ROS 2 workspace directory
cd ~/turtlesim_project_01/ros2_ws

# Build only the turtlesim_main_py_pkg package using colcon build tool
colcon build --packages-select turtlesim_main_py_pkg

# Source the workspace setup script to add the built packages to the environment
source install/setup.bash

# Launch the turtlesim application using the specified launch file
ros2 launch turtlesim_bringup turtlesim_app.launch.xml

