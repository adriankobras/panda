# Add ros aliases to .bashrc
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/src/lsy_franka/ros_aliases.bash" >> ~/.bashrc
