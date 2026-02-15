#!/bin/bash
source /opt/ros/humble/setup.bash
source /opt/vislam_ws/install/setup.bash
cd /workspaces/RPW_vislam/slam_ws
colcon build --symlink-install
source /workspaces/RPW_vislam/slam_ws/install/setup.bash
ros2 launch imx219_slam rtabmap_stereo_imu.launch.py