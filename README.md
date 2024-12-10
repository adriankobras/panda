# LSY Lego Panda

This repository contains code for teleoperation using Franka Emika FR3 robot.

## Setting up franka robot
- visit https://172.16.0.2/desk/
- Set franka in FCI mode
- Unlock franka
- Set Execution mode
- Franke should show green LEDs
- Gripper doesnt open and close? Goto Franka desk -> settings -> end-effector -> re-initialize

## Open
- Open lsy_franka in VSCode and open the container
- In one terminal, run "panda" (=ros2 launch franka_bringup franka.launch.py robot_ip:=172.16.0.2 arm_id:=fr3 use_rviz:=true) to connect to the robot
- In another terminal, run "rs" (=ros2 launch realsense2_camera rs_launch.py) to start the camera node
- In another terminal, run "vs" (=ros2 launch vicon_receiver client.launch.py) to connect to Vicon. If you have rebuilt the container, you need to change the IP in ~/ros2_ws/src/vicon_receiver/vicon_receiver/launch/client.launch.py to 10.157.163.191
- You can now access robot data, camera images and vicon data via the respective topics (see ros2 topic list)
- To start recording data (end-effector poses, vicon data, camera images), navigate to ~/ros2_ws/src/lsy_franka/demonstration_collection and run "python3 demonstration_collection/recorder.py"
- To visualize the data, run "python3 demonstration_collection/visualize_demonstrations.py

## Other
- Before running ros2 in terminal: source /opt/ros/humble/setup.bash
- Get topics:   ros2 topic list
- Get nodes:    ros2 node list
- Set domain id on laptop to be the same as on desktop: export ROS_DOMAIN_ID=100 (check with echo $ROS_DOMAIN_ID)
- Listen to topic with: ros2 topic echo /franka_robot_state_broadcaster/current_pose (or /camera/camera/color/image_rect_raw/compressed)

