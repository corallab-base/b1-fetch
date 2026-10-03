#!/bin/bash
# Localhost tests of b1_bridge_node against fake_robot. Own ROS master on
# :11411, bridge target 127.0.0.1 -> nothing reaches the robot or the running
# bringup (master :11311).
# Usage: src/fetch_b1_bridge/test/test_bridge.sh   (after catkin_make)
# Builds fake_robot.cpp, runs 16 scenarios, prints what the fake robot
# received and what the bridge logged. Takes ~4 min (rostopic/rosservice need
# ~3 s to start here).
HERE=$(cd "$(dirname "$0")" && pwd)
S=$(mktemp -d)
SDK=${UNITREE_SDK_DIR:-$HERE/../../../third_party/unitree_legged_sdk-3.8.3}
g++ -std=c++14 -O2 $HERE/fake_robot.cpp -I$SDK/include -L$SDK/lib/cpp/arm64 -lunitree_legged_sdk \
    -pthread -Wl,-rpath,$SDK/lib/cpp/arm64 -o $S/fake_robot || exit 1
source ~/projects/fetch/env.sh >/dev/null 2>&1
export ROS_MASTER_URI=http://localhost:11411 ROS_IP=127.0.0.1 ROSCONSOLE_STDOUT_LINE_BUFFERED=1
BR=~/projects/fetch/devel/lib/fetch_b1_bridge/b1_bridge_node

start_master() { roscore -p 11411 >$S/core.log 2>&1 & MASTER=$!; until rosparam list >/dev/null 2>&1; do sleep 0.3; done; }
stop_master() { kill -INT $MASTER 2>/dev/null; wait $MASTER 2>/dev/null; }
start() {  # $1 = fake robot extra args
  : >$S/ctl
  $S/fake_robot $S/ctl $1 >$S/robot.log 2>&1 & ROBOT=$!
  $BR _target_ip:=127.0.0.1 _idle_mode:=1 >$S/bridge.log 2>&1 & BRIDGE=$!
  sleep 1.5
}
arm() { rosservice call /b1_bridge/arm >$S/arm.log 2>&1; }
disarm() { rosservice call /b1_bridge/disarm >/dev/null 2>&1; }
armed() { arm; waitlog "-> HOLD"; }
pub() { rostopic pub -r 20 /cmd_vel geometry_msgs/Twist "{linear: {x: $1, y: 0, z: 0}, angular: {z: ${2:-0}}}" >/dev/null 2>&1 & PUB=$!; }
unpub() { [ -n "$PUB" ] && { kill $PUB 2>/dev/null; wait $PUB 2>/dev/null; }; PUB=; }
ctl() { echo "$*" >>$S/ctl; }
finish() { unpub; kill -INT $BRIDGE 2>/dev/null; sleep 0.5; kill $BRIDGE $ROBOT 2>/dev/null; wait $BRIDGE 2>/dev/null; wait $ROBOT 2>/dev/null; }
alive() { case "$(ps -o stat= -p $BRIDGE 2>/dev/null | cut -c1)" in ""|Z) echo exited;; *) echo alive;; esac; }
waitlog() {  # $1 pattern, $2 min count (default 1)
  for i in $(seq 1 100); do [ "$(grep -c -- "$1" $S/bridge.log)" -ge "${2:-1}" ] && return; sleep 0.1; done; echo "  (timeout waiting for: $1)"; }
show() {
  echo "  robot saw:"; grep -E "RX|CTL" $S/robot.log | sed 's/^/    /' | head -${1:-12}
  echo "  bridge:"; grep -E "\->|STOPPED|reports|WARN|ignored" $S/bridge.log | sed -E 's/^\[ *[A-Z]+\] \[[0-9.]+\]: /    /' | head -8
}

start_master

echo "== T1 /cmd_vel while DISARMED: nothing sent, warning logged"
start; pub 0.2; sleep 4; unpub; echo "  packets at robot: $(grep -c RX $S/robot.log)"; finish; show

echo "== T2 arm, no command: ARMING -> HOLD, mode 1 at zero velocity, no mode 2"
start; armed; sleep 1; show; echo "  mode 2 packets: $(grep -c 'mode=2' $S/robot.log)"; echo "  ~armed: $(rostopic echo -n1 /b1_bridge/armed 2>/dev/null | head -1)"; finish

echo "== T3 armed, walk 0.2, stop -> HOLD (mode 2 at zero); disarm -> DISARMING (1 s) -> DISARMED, packets stop"
start; armed; pub 0.2; waitlog "-> WALK"; sleep 1; unpub; waitlog "-> HOLD" 2; disarm; waitlog "-> DISARMED"; sleep 0.5; show 14; echo "  bridge $(alive)"; finish

echo "== T4 clamps: vx 1.0 -> 0.30, vx -1.0 -> -0.07"
start; armed; pub 1.0; waitlog "-> WALK"; sleep 0.5; unpub; pub -1.0; waitlog "-> WALK" 2; sleep 0.5; unpub; sleep 0.3
grep "RX mode=2" $S/robot.log | sed 's/^/    /'; finish

echo "== T5 body height drops to 0.30 while walking -> stop"
start; armed; pub 0.2; waitlog "-> WALK"; sleep 0.5; ctl height 0.30; sleep 0.5; show; echo "  bridge $(alive)"; finish

echo "== T6 robot reports mode 7 while HOLD (no command) -> stop"
start; armed; sleep 0.5; ctl mode 7; sleep 0.5; show; echo "  bridge $(alive)"; finish

echo "== T7 remote button (L2+B = 0x0220) while HOLD -> stop"
start; armed; sleep 0.5; ctl btn 0220; sleep 0.5; show; echo "  bridge $(alive)"; finish

echo "== T8 remote stick while DISARMED (robot streams state) -> stop"
start --stream; sleep 0.5; ctl lx 0.8; sleep 0.5; show; echo "  bridge $(alive)"; finish

echo "== T9 robot stops answering while walking -> stop after ~200 ms, ~0.1 s of mode 2 at zero first"
start; armed; pub 0.2; waitlog "-> WALK"; sleep 0.5; ctl mute 1; sleep 0.6; show; echo "  bridge $(alive)"; finish

echo "== T10 armed with the robot lying (mode 7, h 0.07) -> stop in ARMING, no mode 2"
start; ctl mode 7; ctl height 0.07; sleep 0.3; arm; waitlog "STOPPED"; sleep 0.3; show
echo "  packets at robot: $(grep -c RX $S/robot.log), mode 2: $(grep -c 'mode=2' $S/robot.log)"; echo "  bridge $(alive)"; finish

echo "== T11 Ctrl+C while walking -> ~0.1 s of mode 2 at zero, then nothing"
start; armed; pub 0.2; waitlog "-> WALK"; sleep 0.5; kill -INT $BRIDGE; sleep 1; show; echo "  bridge $(alive)"; finish

echo "== T12 ROS master killed while walking -> DISARMING (mode 2 at zero), then exit"
start; armed; pub 0.2; waitlog "-> WALK"; sleep 0.5; stop_master; sleep 2.5; show 14; echo "  bridge $(alive)"; finish

echo "== T13 master restarted (new PID) while DISARMED -> exit"
start_master; start; sleep 0.5; stop_master; start_master; sleep 3; show; echo "  bridge $(alive)"; finish

echo "== T14 a single latched /cmd_vel sent before arming doesn't start walking"
start; rostopic pub -1 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}}" >/dev/null 2>&1 & ONE=$!
sleep 3.5; armed; sleep 1; show; echo "  mode 2 packets: $(grep -c 'mode=2' $S/robot.log)"; wait $ONE 2>/dev/null; finish

echo "== T15 odometry: walk 0.2 m/s for ~2 s -> /odom x ~0.4, z 0.55, TF odom -> base_link"
start; armed; rostopic echo -n1 /odom/pose/pose/position >$S/odom0 2>&1 & E0=$!; wait $E0
pub 0.2; waitlog "-> WALK"; sleep 2; unpub; waitlog "-> HOLD" 2; sleep 0.3
rostopic echo -n1 /odom >$S/odom1 2>&1
timeout 4 rosrun tf tf_echo odom base_link >$S/tf 2>&1
echo "  before: $(tr '\n' ' ' <$S/odom0)"
echo "  after:  $(grep -A3 'position:' $S/odom1 | tr '\n' ' ')"
echo "          $(grep -E 'frame_id|child_frame_id' $S/odom1 | tr '\n' ' ')"
echo "  tf:     $(grep -m1 Translation $S/tf)"
echo "  imu:    $(timeout 5 rostopic hz -w 50 /b1_bridge/imu 2>&1 | grep -m1 'average rate')"; finish

echo "== T16 turn 0.3 rad/s, stop -> HOLD sends mode 2 at zero (robot ignores mode 1 while walking), robot stops turning"
start; armed; pub 0 0.3; waitlog "-> WALK"; sleep 1; unpub; waitlog "-> HOLD" 2; sleep 2.5
echo "  state:  $(rostopic echo -n1 /b1_bridge/state/data 2>/dev/null | head -1)"
disarm; waitlog "-> DISARMED"; sleep 0.5; show 14; echo "  still-moving errors: $(grep -c 'still moving' $S/bridge.log)"; finish

stop_master
