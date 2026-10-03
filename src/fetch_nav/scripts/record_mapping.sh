#!/bin/bash
# Records what mapping needs during a drive (small: no clouds or images):
#   rosrun fetch_nav record_mapping.sh lab     -> ~/projects/fetch/bags/mapping_lab.bag
# Run with bringup + armed bridge (odometry only flows while armed).
name=${1:-$(date +%Y%m%d_%H%M)}
mkdir -p ~/projects/fetch/bags
exec rosbag record -O ~/projects/fetch/bags/mapping_$name \
  /scan /cam_left/scan /cam_right/scan /cam_front/scan /cam_front_down/scan /cam_rear/scan \
  /odom /b1_bridge/imu /b1_bridge/state /tf /tf_static /safety_filter/status /cmd_vel
