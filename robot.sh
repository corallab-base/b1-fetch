#!/bin/bash
# Cameras + safety filter + B1 bridge in one command, after safety checks:
#
#   ~/projects/fetch/robot.sh                  # checks, then roslaunch fetch_safety robot.launch
#   ~/projects/fetch/robot.sh front_down:=true # extra args go to roslaunch
#   ~/projects/fetch/robot.sh --check          # checks only
#
# Run it on the board ROS_MASTER_URI points at (env.sh: .23, the head);
# roslaunch starts the master there. It refuses to start when:
#   - a ROS master already runs here, or ROS/camera/bridge processes run on
#     any board (a leftover bridge commanding the robot preceded the
#     2026-09-26 rollover), or anything uses UDP port 8082 (the B1's);
#   - a board's clock isn't synchronized or is > 0.5 s off this one's
#     (TF breaks on a clock jump mid-run);
#   - a board can't be reached (then nothing on it can be checked).
# The bridge starts DISARMED. Stand the robot, check it stands, then
#   rosservice call /b1_bridge/arm
# Navigation in another terminal: roslaunch fetch_nav nav.launch
# Shutdown: release keys -> rosservice call /b1_bridge/disarm ->
# stand down (L2+A or b1_stand_down) -> Ctrl+C here -> power off.
set -e
source ~/projects/fetch/env.sh
BOARDS="23 24 25"
ME=$(echo $ROS_IP | cut -d. -f4)
MAX_SKEW=0.5

CHECK_ONLY=
[ "$1" = "--check" ] && { CHECK_ONLY=1; shift; }

fail=
bad() { echo "  NO: $*"; fail=1; }

if [ "$ROS_MASTER_URI" != "http://$ROS_IP:11311" ]; then
  bad "ROS_MASTER_URI is $ROS_MASTER_URI, not this board ($ROS_IP): run this on the master's board"
fi
if ss -tln | grep -q ':11311 '; then
  bad "a ROS master already runs here: stop the old stack first"
fi

# One probe per board: its clock, sync state, ROS/camera/bridge processes (by
# process name, so a shell whose command line mentions them doesn't count) and
# UDP 8082 users.
probe='date +%s.%N
timedatectl status | grep -q "synchronized: yes" && echo sync || echo nosync
pgrep -a "^(roslaunch|rosmaster|rosout|nodelet|realsense.*|b1_bridge_node|move_base|rtabmap)$" | cut -c1-120 | sed "s/^/proc /"
ss -uanp 2>/dev/null | grep ":8082 " | sed "s/^/udp /"
true'

for b in $BOARDS; do
  echo "== .$b"
  board_fail=
  before=$(date +%s.%N)
  if [ "$b" = "$ME" ]; then
    out=$(bash -c "$probe")
  else
    out=$(ssh -o BatchMode=yes -o ConnectTimeout=5 unitree@192.168.123.$b "$probe" 2>/dev/null) ||
      { bad "can't reach .$b over SSH"; continue; }
  fi
  after=$(date +%s.%N)
  # its clock vs ours at the middle of the round trip
  skew=$(python3 -c "print('%.2f' % ($(echo "$out" | head -1) - ($before + $after) / 2))")
  python3 -c "import sys; sys.exit(abs($skew) > $MAX_SKEW)" || { bad ".$b clock is ${skew} s off this board's"; board_fail=1; }
  echo "$out" | grep -qx sync || { bad ".$b clock not synchronized (timedatectl)"; board_fail=1; }
  echo "$out" | grep '^proc ' | sed 's/^proc /  running: /' | grep . && { bad "ROS/camera/bridge processes on .$b"; board_fail=1; }
  echo "$out" | grep '^udp ' | sed 's/^udp /  /' | grep . && { bad "UDP port 8082 in use on .$b (a bridge?)"; board_fail=1; }
  [ -n "$board_fail" ] || echo "  ok (clock ${skew} s)"
done

if [ -n "$fail" ]; then
  echo "Not starting: fix the above first."
  exit 1
fi
echo "All checks passed."
[ -n "$CHECK_ONLY" ] && exit 0
echo "Starting cameras + filter + bridge (DISARMED). Arm only after checking the robot stands."
exec roslaunch fetch_safety robot.launch "$@"
