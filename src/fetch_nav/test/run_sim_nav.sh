#!/bin/bash
# Offline navigation test (no robot): own ROS master on :11411, sim_nav.launch
# (fake robot + cameras, real checkers/filter/RTAB-Map/move_base), then
# sim_nav_check.py sends two goals. Usage: src/fetch_nav/test/run_sim_nav.sh
HERE=$(cd "$(dirname "$0")" && pwd)
S=$(mktemp -d)
source ~/projects/fetch/env.sh >/dev/null 2>&1
export ROS_MASTER_URI=http://localhost:11411 ROS_IP=127.0.0.1
roslaunch -p 11411 fetch_nav sim_nav.launch database:=$S/sim.db >$S/launch.log 2>&1 & LAUNCH=$!
python2 $HERE/sim_nav_check.py; RESULT=$?
kill -INT $LAUNCH; wait $LAUNCH 2>/dev/null
echo "logs: $S/launch.log"
exit $RESULT
