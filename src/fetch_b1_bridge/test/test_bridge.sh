#!/bin/bash
# Localhost tests of b1_bridge_node against fake_robot. Own ROS master on
# :11411, bridge target 127.0.0.1 -> nothing reaches the robot or the running
# bringup (master :11311).
# Usage: src/fetch_b1_bridge/test/test_bridge.sh   (after catkin_make)
# Builds fake_robot.cpp, runs 12 scenarios, prints what the fake robot
# received and what the bridge logged. Takes ~2.5 min (rostopic pub needs ~3 s
# to start here, and bridge stdout is only flushed at exit).
HERE=$(cd "$(dirname "$0")" && pwd)
S=$(mktemp -d)
SDK=${UNITREE_SDK_DIR:-$HOME/UnitreeSLAM/sdk/unitree_legged_sdk-3.8.3}
g++ -std=c++14 -O2 $HERE/fake_robot.cpp -I$SDK/include -L$SDK/lib/cpp/arm64 -lunitree_legged_sdk \
    -pthread -Wl,-rpath,$SDK/lib/cpp/arm64 -o $S/fake_robot || exit 1
source ~/projects/fetch/env.sh >/dev/null 2>&1
export ROS_MASTER_URI=http://localhost:11411 ROS_IP=127.0.0.1
BR=~/projects/fetch/devel/lib/fetch_b1_bridge/b1_bridge_node

start_master() { roscore -p 11411 >$S/core.log 2>&1 & MASTER=$!; until rosparam list >/dev/null 2>&1; do sleep 0.3; done; }
stop_master() { kill -INT $MASTER 2>/dev/null; wait $MASTER 2>/dev/null; }
start() {  # $1 = fake robot extra args
  : >$S/ctl
  $S/fake_robot $S/ctl $1 >$S/robot.log 2>&1 & ROBOT=$!
  $BR _target_ip:=127.0.0.1 _idle_mode:=1 >$S/bridge.log 2>&1 & BRIDGE=$!
  sleep 1.5
}
pub() { rostopic pub -r 20 /cmd_vel geometry_msgs/Twist "{linear: {x: $1, y: 0, z: 0}}" >/dev/null 2>&1 & PUB=$!; }
unpub() { [ -n "$PUB" ] && { kill $PUB 2>/dev/null; wait $PUB 2>/dev/null; }; PUB=; }
ctl() { echo "$*" >>$S/ctl; }
finish() { unpub; kill -INT $BRIDGE 2>/dev/null; sleep 0.5; kill $BRIDGE $ROBOT 2>/dev/null; wait $BRIDGE 2>/dev/null; wait $ROBOT 2>/dev/null; }
alive() { case "$(ps -o stat= -p $BRIDGE 2>/dev/null | cut -c1)" in ""|Z) echo exited;; *) echo alive;; esac; }
waitlog() {  # $1 pattern, $2 min count (default 1)
  for i in $(seq 1 100); do [ "$(grep -c -- "$1" $S/bridge.log)" -ge "${2:-1}" ] && return; sleep 0.1; done; echo "  (timeout waiting for: $1)"; }
show() {
  echo "  robot saw:"; grep -E "RX|CTL" $S/robot.log | sed 's/^/    /' | head -${1:-12}
  echo "  bridge:"; grep -E "\->|STOPPED|reports|WARN" $S/bridge.log | sed -E 's/^\[ *[A-Z]+\] \[[0-9.]+\]: /    /' | head -8
}

start_master

echo "== T1 no command: bridge stays QUIET, sends nothing"
start; sleep 1; echo "  packets at robot: $(grep -c RX $S/robot.log)"; finish

echo "== T2 walk 0.2, then stop: CHECK -> WALK -> SETTLE (1 s) -> QUIET"
start; pub 0.2; waitlog "-> WALK"; sleep 1; unpub; waitlog "-> QUIET"; sleep 0.5; show; echo "  bridge $(alive)"; finish

echo "== T3 clamps: vx 1.0 -> 0.30, vx -1.0 -> -0.10"
start; pub 1.0; waitlog "-> WALK"; sleep 0.5; unpub; pub -1.0; waitlog "-> WALK" 2; sleep 0.5; unpub; sleep 0.3
grep "RX mode=2" $S/robot.log | sed 's/^/    /'; finish

echo "== T4 body height drops to 0.30 while walking -> stop"
start; pub 0.2; waitlog "-> WALK"; sleep 0.5; ctl height 0.30; sleep 0.5; show; echo "  bridge $(alive)"; finish

echo "== T5 robot reports mode 7 while walking -> stop"
start; pub 0.2; waitlog "-> WALK"; sleep 0.5; ctl mode 7; sleep 0.5; show; echo "  bridge $(alive)"; finish

echo "== T6 remote button (L2+B = 0x0220) while walking -> stop"
start; pub 0.2; waitlog "-> WALK"; sleep 0.5; ctl btn 0220; sleep 0.5; show; echo "  bridge $(alive)"; finish

echo "== T7 remote stick while QUIET (robot streams state) -> stop"
start --stream; sleep 0.5; ctl lx 0.8; sleep 0.5; show; echo "  bridge $(alive)"; finish

echo "== T8 robot stops answering while walking -> stop after ~200 ms"
start; pub 0.2; waitlog "-> WALK"; sleep 0.5; ctl mute 1; sleep 0.6; show; echo "  bridge $(alive)"; finish

echo "== T9 robot lying (mode 7, h 0.07) when a command arrives -> stop in CHECK, no mode 2 sent"
start; ctl mode 7; ctl height 0.07; sleep 0.3; pub 0.2; waitlog "STOPPED"; sleep 0.3; show; echo "  mode 2 packets: $(grep -c 'mode=2' $S/robot.log)"; echo "  bridge $(alive)"; finish

echo "== T10 Ctrl+C while walking -> ~0.1 s of mode 1, then nothing"
start; pub 0.2; waitlog "-> WALK"; sleep 0.5; kill -INT $BRIDGE; sleep 1; show; echo "  bridge $(alive)"; finish

echo "== T11 ROS master killed while walking -> SETTLE, then exit"
start; pub 0.2; waitlog "-> WALK"; sleep 0.5; stop_master; sleep 2.5; show 14; echo "  bridge $(alive)"; finish

echo "== T12 master restarted (new PID) while QUIET -> exit"
start_master; start; sleep 0.5; stop_master; start_master; sleep 3; show; echo "  bridge $(alive)"; finish
stop_master
