# ROS2 aliases
alias cb="cd ~/ros2_ws && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
alias cb_simple="cd ~/ros2_ws && colcon build"
alias sw="source /opt/ros/$ROS_DISTRO/setup.bash && source ~/ros2_ws/install/setup.bash"

# Realsense aliases
alias rs="ros2 launch realsense2_camera rs_launch.py"

# Vicon aliases
alias vc="ros2 launch vicon_receiver client.launch.py"

# Franka aliases
alias panda="ros2 launch franka_bringup franka.launch.py robot_ip:=172.16.0.2 arm_id:=fr3 use_rviz:=true"
alias panda_fake="ros2 launch franka_bringup franka.launch.py robot_ip:=172.16.0.2 arm_id:=fr3 use_rviz:=true use_fake_hardware:=true"

# Go to the lsy_franka repo
alias proj="cd ~/ros2_ws/src/lsy_franka"
