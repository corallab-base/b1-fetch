#!/bin/bash
# roslaunch <machine env-loader> for the camera boards (.23 front, .25 rear).
# Copied to ~/projects/fetch/remote_env.sh on each board. roslaunch passes
# ROS_MASTER_URI; this sets up the workspace and advertises the board's IP
# (all NX boards are named "nx", so hostnames can't be used).
source ~/projects/fetch/devel/setup.bash
export ROS_IP=$(hostname -I | tr ' ' '\n' | grep -m1 '^192\.168\.123\.')
unset ROS_HOSTNAME
exec "$@"
