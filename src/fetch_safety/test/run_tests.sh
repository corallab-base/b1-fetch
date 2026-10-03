#!/bin/bash
# Offline safety filter tests, no cameras or robot: own ROS master on :11411,
# filter + zone_checkers from test_safety.launch, fake clouds from
# test_safety_filter.py. Usage: src/fetch_safety/test/run_tests.sh
source ~/projects/fetch/env.sh >/dev/null 2>&1
export ROS_MASTER_URI=http://localhost:11411 ROS_IP=127.0.0.1
roslaunch -p 11411 fetch_safety test_safety.launch >/tmp/fetch_safety_test_launch.log 2>&1 & LAUNCH=$!
until rosnode list 2>/dev/null | grep -q zone_checker; do sleep 0.5; done; sleep 2
rosrun fetch_safety test_safety_filter.py; RESULT=$?
kill -INT $LAUNCH; wait $LAUNCH 2>/dev/null
exit $RESULT
