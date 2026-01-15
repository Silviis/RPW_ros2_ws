#!/bin/sh

rosdep update && cd /workspaces/RPW_vislam/cam_ws && rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --continue-on-error

cd /workspaces/RPW_vislam/slam_ws && rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --continue-on-error
