#!/bin/bash
set -e
source /opt/ros/$ROS_DISTRO/setup.bash
source /home/ros/ros2_ws/install/setup.bash

eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"

exec $@
