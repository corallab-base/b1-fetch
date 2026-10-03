#!/bin/bash
# Records what mapping needs during a drive:
#   rosrun fetch_nav record_mapping.sh lab           -> ~/projects/fetch/bags/mapping_lab.bag
#   rosrun fetch_nav record_mapping.sh lab camera    -> also the front camera's infrared
#                                                       image + depth (bringup front_infra:=true),
#                                                       ~2.3 MB/s more
# Run with bringup + armed bridge (odometry only flows while armed), on the
# board with the front camera (.23) so the images don't cross the network.
name=${1:-$(date +%Y%m%d_%H%M)}
extra=()
if [ "$2" = camera ]; then
  extra=(/cam_front/infra1/image_rect_raw /cam_front/infra1/camera_info
         /cam_front/depth/image_rect_raw /cam_front/depth/camera_info)
fi
mkdir -p ~/projects/fetch/bags
exec rosbag record -O ~/projects/fetch/bags/mapping_$name \
  /scan /cam_left/scan /cam_right/scan /cam_front/scan /cam_front_down/scan /cam_rear/scan \
  /odom /b1_bridge/imu /b1_bridge/state /tf /tf_static /safety_filter/status /cmd_vel "${extra[@]}"
