#!/bin/bash
cd /workspaces/RPW_vislam/cam_ws
colcon build --symlink-install
source /workspaces/RPW_vislam/cam_ws/install/setup.bash
ros2 launch imx219-83_stereo_camera_ros2 stereo_camera.launch.py