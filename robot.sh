#!/bin/bash
# Cameras + safety filter + B1 bridge in one command, after safety checks:
#
#   ~/projects/fetch/robot.sh                  # checks, then roslaunch fetch_safety robot.launch
#   ~/projects/fetch/robot.sh front_down:=true # extra args go to roslaunch
#   ~/projects/fetch/robot.sh --check          # checks only
#   ~/projects/fetch/robot.sh --stop           # stop everything safely (also after a lost SSH session)
#   ~/projects/fetch/robot.sh --no-tmux ...    # run in this terminal, not in tmux
#
# With tmux installed the stack runs in tmux session "robot", so a dropped
# SSH connection doesn't kill it (without tmux it dies whenever sshd notices,
# minutes to hours later): `tmux attach -t robot` to get it back, Ctrl-b d to
# detach, Ctrl-b c for another window (source env.sh there, e.g. for nav).
#
# Run it on the board ROS_MASTER_URI points at (env.sh: .23, the head);
# roslaunch starts the master there. It refuses to start when:
#   - a ROS master already runs here, or ROS/camera/bridge processes run on
#     any board (a leftover bridge commanding the robot preceded the
#     2026-09-26 rollover), or anything uses UDP port 8082 (the B1's);
#   - a board's clock isn't synchronized or is > 0.5 s off this one's
#     (TF breaks on a clock jump mid-run);
#   - a board can't be reached (then nothing on it can be checked).
# Then it power-cycles every board's cameras (boot/usb_reset.sh), starts the
# stack and stops it again unless the front, left and right cameras publish
# within 60 s (mandatory; the rear camera only warns).
# The bridge starts DISARMED. Stand the robot, check it stands, then
#   rosservice call /b1_bridge/arm
# Navigation in another terminal: roslaunch fetch_nav nav.launch
# Shutdown: release keys -> rosservice call /b1_bridge/disarm ->
# stand down (L2+A or b1_stand_down) -> Ctrl+C here (or robot.sh --stop) -> power off.
set -e
source ~/projects/fetch/env.sh
BOARDS="23 24 25"
ME=$(echo $ROS_IP | cut -d. -f4)
MAX_SKEW=0.5
SESSION=robot

CHECK_ONLY= STOP= NO_TMUX=
while :; do
  case "$1" in
    --check) CHECK_ONLY=1; shift ;;
    --stop) STOP=1; shift ;;
    --no-tmux) NO_TMUX=1; shift ;;
    *) break ;;
  esac
done

# Stop in the safe order: cancel nav's goal, disarm the bridge (it stops a
# walking robot first, then goes silent), then end the launches, nav first.
# Leaves the robot as it is: if it stands, it keeps standing (nothing sends).
stop_stack() {
  if timeout 5 rosnode list >/dev/null 2>&1; then
    nodes=$(timeout 5 rosnode list 2>/dev/null)
    if echo "$nodes" | grep -qx /move_base; then
      echo "cancelling the navigation goal"; timeout 10 rosrun fetch_nav goal.py --cancel >/dev/null 2>&1 || true
    fi
    if echo "$nodes" | grep -qx /b1_bridge &&
       timeout 3 rostopic echo -n 1 /b1_bridge/armed 2>/dev/null | grep -q "data: True"; then
      echo "disarming the bridge"; timeout 10 rosservice call /b1_bridge/disarm >/dev/null 2>&1 || true
      sleep 2   # 1 s settle_time, then DISARMED
    fi
  fi
  # roslaunch processes by name (a shell whose command line mentions them doesn't count): nav first
  launches() { for p in $(pgrep -x roslaunch); do echo "$p $(ps -o args= -p $p)"; done; }
  for pass in nav rest; do
    pids=$(launches | if [ $pass = nav ]; then grep "fetch_nav"; else cat; fi | cut -d' ' -f1)
    [ -n "$pids" ] || continue
    echo "stopping: $(for p in $pids; do ps -o args= -p $p | sed 's#.*/roslaunch ##'; done | paste -sd, -)"
    kill -INT $pids 2>/dev/null || true
    for i in $(seq 30); do ps -p $(echo $pids | tr ' ' ,) >/dev/null || break; sleep 1; done
    ps -p $(echo $pids | tr ' ' ,) >/dev/null && { echo "  still running after 30 s: SIGTERM"; kill -TERM $pids 2>/dev/null || true; sleep 3; }
  done
  # a bridge outside any launch (started on its own) must not stay behind
  pgrep -x b1_bridge_node >/dev/null && { echo "stopping a separate b1_bridge_node"; pkill -INT -x b1_bridge_node; sleep 2; }
  command -v tmux >/dev/null && tmux kill-session -t $SESSION 2>/dev/null || true
  echo "Stopped. If the robot stands, lie it down with the remote (L2+A)."
}

if [ -n "$STOP" ]; then
  stop_stack
  echo; echo "Checking that nothing is left:"
  CHECK_ONLY=1
fi

fail= running=
bad() { echo "  NO: $*"; fail=1; }

if [ "$ROS_MASTER_URI" != "http://$ROS_IP:11311" ]; then
  bad "ROS_MASTER_URI is $ROS_MASTER_URI, not this board ($ROS_IP): run this on the master's board"
fi
if ss -tln | grep -q ':11311 '; then
  bad "a ROS master already runs here: stop the old stack first"; running=1
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
  echo "$out" | grep '^proc ' | sed 's/^proc /  running: /' | grep . && { bad "ROS/camera/bridge processes on .$b"; board_fail=1; running=1; }
  echo "$out" | grep '^udp ' | sed 's/^udp /  /' | grep . && { bad "UDP port 8082 in use on .$b (a bridge?)"; board_fail=1; running=1; }
  [ -n "$board_fail" ] || echo "  ok (clock ${skew} s)"
done

if [ -n "$fail" ]; then
  if [ -n "$running" ] && [ -z "$STOP" ]; then
    echo
    echo "A stack is still running (e.g. from a dropped SSH session). It keeps working:"
    command -v tmux >/dev/null && tmux has-session -t $SESSION 2>/dev/null &&
      echo "  get its terminal back:   tmux attach -t $SESSION"
    echo "  use it from here:        source ~/projects/fetch/env.sh; rosnode list"
    echo "  stop it safely:          ~/projects/fetch/robot.sh --stop"
  fi
  echo "Not starting: fix the above first."
  exit 1
fi
echo "All checks passed."
[ -n "$CHECK_ONLY" ] && exit 0

# Power-cycle every board's cameras before each start (no camera driver runs:
# the checks above made sure). A RealSense can wedge when its driver stops
# and starts again: on 2026-10-08 the right camera worked after boot, then
# after a restart only logged "failed to set power state" until a USB reset.
echo "Power-cycling the cameras on .${BOARDS// /, .} (~15 s)..."
pids=
for b in $BOARDS; do
  if [ "$b" = "$ME" ]; then ~/projects/fetch/boot/usb_reset.sh &
  else ssh -o BatchMode=yes unitree@192.168.123.$b ~/projects/fetch/boot/usb_reset.sh & fi
  pids="$pids $b:$!"
done
for bp in $pids; do wait ${bp#*:} || echo "  warning: USB reset on .${bp%%:*} failed (see boot/usb_reset.log there)"; done

# The front, left and right cameras are mandatory: without one of them nav
# and the filter are blind (or blocked) on that side. A wedged camera can
# still enumerate on USB, so wait for its data (zone_counts), not the device.
MANDATORY="cam_front cam_left cam_right"
OPTIONAL="cam_rear"
CAMERA_WAIT=60   # s; a normal start takes ~30 s
wait_cameras() {   # [pid to watch] -> 0 when all mandatory cameras publish
  local missing="$MANDATORY $OPTIONAL" still c deadline=$((SECONDS + CAMERA_WAIT))
  while [ -n "$missing" ] && [ $SECONDS -lt $deadline ]; do
    [ -n "$1" ] && ! kill -0 $1 2>/dev/null && return 0   # the launch ended (Ctrl+C): nothing to report
    still=
    for c in $missing; do timeout 4 rostopic echo -n 1 /$c/zone_counts >/dev/null 2>&1 || still="$still $c"; done
    missing=$(echo $still)
    [ -n "$missing" ] && sleep 2
  done
  local bad_cams= c2
  for c2 in $missing; do
    case " $MANDATORY " in *" $c2 "*) bad_cams="$bad_cams $c2" ;;
      *) echo "WARNING: $c2 not publishing after ${CAMERA_WAIT} s (not mandatory; its zones stay blocked)" ;; esac
  done
  if [ -n "$bad_cams" ]; then
    echo; echo "NOT STARTED: mandatory camera(s)$bad_cams not publishing after ${CAMERA_WAIT} s."
    echo "Stopping the stack. Check the camera's USB cable/connector, then run robot.sh again."
    return 1
  fi
  echo "Cameras OK: $MANDATORY publishing${missing:+ (not: $missing)}."
}

echo "Starting cameras + filter + bridge (DISARMED). Arm only after checking the robot stands."
if [ -z "$NO_TMUX" ] && [ -z "$TMUX" ] && command -v tmux >/dev/null; then
  # env.sh again inside: a tmux server that already runs keeps its own environment
  args=; [ $# -gt 0 ] && args=$(printf '%q ' "$@")
  tmux new-session -d -s $SESSION -n stack \
    "source ~/projects/fetch/env.sh && roslaunch fetch_safety robot.launch $args;
     echo; echo 'robot.launch ended. Enter closes this window.'; read"
  echo "Waiting for the cameras (up to ${CAMERA_WAIT} s)..."
  if ! wait_cameras; then
    echo "--- last launch output:"; tmux capture-pane -p -t $SESSION | grep . | tail -12
    stop_stack
    exit 1
  fi
  echo "In tmux session '$SESSION': Ctrl-b d detaches (the stack keeps running), tmux attach -t $SESSION returns."
  exec tmux attach -t $SESSION
fi
# In this terminal: roslaunch replaces this shell (Ctrl+C works as usual);
# a watcher stops it (SIGINT to this PID) if a mandatory camera stays silent.
( wait_cameras $$ || kill -INT $$ ) &
exec roslaunch fetch_safety robot.launch "$@"
