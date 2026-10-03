#!/bin/bash
# roslaunch <machine env-loader> for nodes started on another board over SSH.
# Same setup as env.sh, but keeps the ROS_MASTER_URI roslaunch passes in.
master=$ROS_MASTER_URI
source ~/projects/fetch/env.sh
export ROS_MASTER_URI=$master
exec "$@"
