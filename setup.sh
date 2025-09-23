#!/bin/sh

rosdep update && cd /workspaces/RPW_ros2_ws/ && rosdep install --from-paths src --ignore-src -r -y && cd -
colcon build --symlink-install --continue-on-error
