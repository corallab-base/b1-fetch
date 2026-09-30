# Source in every terminal on this board (.24) before using the fetch stack:
#   source ~/projects/fetch/env.sh
#
# All three NX boards share the hostname "nx", so ROS must advertise IPs, and
# the master URI must be this board's IP (roslaunch hands it to .23/.25).
source ~/projects/fetch/devel/setup.bash
export ROS_IP=192.168.123.24
export ROS_MASTER_URI=http://192.168.123.24:11311
unset ROS_HOSTNAME
