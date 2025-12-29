#!/bin/bash

cd ~/ros2_ws

colcon build --packages-select turtlesim_main_py_pkg

source install/setup.bash

ros2 launch turtlesim_bringup turtlesim_app.launch.xml 