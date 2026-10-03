# Source in every terminal before using the fetch stack, on any board:
#   source ~/projects/fetch/env.sh
#
# All NX boards share the hostname "nx", so ROS must advertise IPs: ROS_IP is
# this board's 192.168.123.x address. The master runs on .24 unless
# FETCH_MASTER_URI says otherwise (roslaunch hands it to the other boards).
source ~/projects/fetch/devel/setup.bash
export ROS_IP=$(hostname -I | tr ' ' '\n' | grep -m1 '^192\.168\.123\.')
export ROS_MASTER_URI=${FETCH_MASTER_URI:-http://192.168.123.24:11311}
unset ROS_HOSTNAME
