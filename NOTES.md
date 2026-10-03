# fetch — B1 safety filter: status checkpoint (2026-09-26)

## Startup (after a robot restart)
```bash
source ~/projects/fetch/env.sh                       # every terminal
roslaunch fetch_safety bringup.launch                # 4 cameras (.24/.23/.25 over SSH) + safety filter; front_down:=true adds the 5th
~/projects/fetch/devel/lib/fetch_b1_bridge/b1_stand_up
roslaunch fetch_b1_bridge b1_bridge.launch idle_mode:=1   # starts DISARMED, sends nothing
rosservice call /b1_bridge/arm                             # ONLY after checking the robot stands
rostopic pub -r 10 /cmd_vel_raw geometry_msgs/Twist '{linear: {x: 0.0, y: 0.1, z: 0.0}}'   # always /cmd_vel_raw
```
Shutdown: Ctrl+C pub -> `rosservice call /b1_bridge/disarm` -> Ctrl+C bridge -> `b1_stand_down` -> Ctrl+C bringup -> power off.

> **Do not use this with the robot standing until the open issues below are fixed.**

## Code on the boards (since 2026-10-04)
- One git repo, same code everywhere: GitHub origin (https://github.com/MahitMehta/fetch-unitree,
  pushed/pulled from .24 only; .24 has no stored GitHub login, the user pushes) -> .24 ->
  `./deploy.sh` pushes HEAD to ~/projects/fetch on .23/.25 over SSH (they have no internet)
  and runs catkin_make there. Commit before deploying. Never edit on .23/.25 directly
  (the push refuses if their checkout has local changes).
- build/, devel/, logs, bags are not in git; each board builds its own workspace.
- `env.sh` works on any board (ROS_IP auto-detected, master .24 unless FETCH_MASTER_URI is set);
  `remote_env.sh` is the roslaunch env-loader for nodes started on other boards.
- Per-board differences left: Unitree SDK only on .24 (fetch_b1_bridge skips its programs
  elsewhere, so the bridge can only run on .24); realsense-ros 2.3.2 is in src/ on .23/.25
  (ignored by git) but in ~/realsense_ws on .24. ~/.bashrc on .24 also sets ROS_IP/master.
- Every camera has a zone_checker on its own board (camera.launch) publishing
  /<cam>/zone_counts; the filter only reads those, so it (and later move_base) can run
  on any board. Clouds are still published for Foxglove and only cross the network
  while someone subscribes.

## Layout
| Board | Cameras (serial) | Role |
|---|---|---|
| .24 (this) | left 152122075367, right 150622073088 | side zones, filter, bridge |
| .23 | front 150622072829 (level), front_down 150622071014 (steep down) | forward |
| .25 | rear 150622073015 (straight down, USB 2.1 -> 480x270) | backward (sees only ~0.18 m past the tail) |
| .220 | B1 high-level controller (separate machine) | bridge target :8082 |
| .10 | motor MCU | |

## Incident: robot bobbed and rolled over (~05:3x)
Happened seconds after a camera bringup test started while the robot stood and a
leftover bridge (idle_mode 1) was still commanding it. No motion commands were
sent. Most likely cause: point clouds flooding the internal network. Measured
(robot lying): front camera alone 112 Mbit/s over .23 -> .24; standing estimate
for front + front_down + rear ~470 Mbit/s. Second suspect: CPU starvation of the
bridge on .24 (board runs 84-91 C and throttles).

Log review (2026-10-01, ~/.ros/log 67d474c8 / c711a900):
- Bridge started 05:13:10 against master A; it only ever sent mode 1 / zero velocity
  (never logged "command received").
- 05:18:20 bringup #1 Ctrl+C killed master A; the bridge kept running orphaned
  (roscpp doesn't exit on master loss) and kept asserting mode 1 at 500 Hz through
  bringup #2 (05:19:22-05:21:29). Possible tug-of-war if the robot tried to lower itself.
- Bobbing -> rollover: user cut power with the body power button (no damping).
  Use L2+B (damping) instead when the remote works.
- Same 4 cameras + bridge ran 05:13-05:18 with similar .24 throttling; unknown whether
  the robot was standing then. Congestion is unproven: the bridge isn't in the balance
  loop, so it would have to affect the .220 <-> .10 link. Safe check: robot lying, no
  bridge, cameras on, ping .220/.10 from .24 for latency/loss.

## Bandwidth reduction (2026-10-01)
- front_down camera off by default (`front_down:=false`; filter loads sources_no_front_down.yaml).
- All cameras use realsense decimation (2x2): max ~49 Mbit/s per cloud (rear ~62) instead of ~195/249.
  Filter point_stride 2 -> 1. Not yet tested on hardware.

## Bridge behaviour (since 2026-10-03: arm/disarm)
Why: any packet carries a mode, a lying B1 stands up from a few ms of mode 1, and
.220 only sends state in reply, so the bridge can't check standing before sending
(2026-10-01 real test: CHECK's 2 packets stood a lying robot up). A person checks.
- Starts DISARMED: sends nothing, /cmd_vel ignored (warning). `~arm` (std_srvs/Trigger)
  -> ARMING: idle_mode until the robot answers; answer lying/damping/low -> stop
  (2 packets reach the robot in the fake test, so arming a LYING robot can still stand it up).
- Armed: sends every cycle: HOLD (idle_mode, zero velocity) or WALK (mode 2) on
  /cmd_vel received after arming. `~disarm` -> 1 s idle_mode -> DISARMED (re-armable).
- Stops for good (node exits) on: mode 5/7, body height < 0.40, no state for 0.2 s,
  any Unitree remote button/stick, ROS master lost/restarted (disarms first).
- While armed publishes /odom + TF odom->base_link (robot's own position estimate,
  IMU quaternion, z = body height so the floor is z=0) and ~imu at 50 Hz, ~armed (latched).
  `publish_tf:=false` when an EKF takes over odom->base_link.
- **Stopping = mode 2 at zero velocity, not idle_mode.** 2026-10-03 real test
  (bags/odom_test1.bag): after turning at 0.3 rad/s, HOLD sent mode 1 for 12 s and the
  robot stayed in mode 2 turning at ~0.25 rad/s. The B1 ignores idle modes while walking
  and latches the last mode-2 velocity. Fixed: zero-velocity packets use mode 2 while the
  robot reports mode 2 (HOLD, DISARMING, Ctrl+C, and a 0.1 s burst on "no state");
  ERROR logged if the robot still moves 2 s into HOLD/DISARMING. Fake robot now latches too.
  Faults on height / mode 5,7 / remote still go silent at once (robot down or remote in charge).
  ANSWERED 2026-10-03: the B1 stops at once when packets stop entirely (user: Ctrl+C on the
  bridge, which sent 0.1 s of mode 1 then nothing, stopped the turn immediately). So silent
  fault exits do stop a walking robot; it's a steady stream of non-walk packets that keeps it going.
- Passed 16 localhost scenarios 2026-10-03 (`src/fetch_b1_bridge/test/test_bridge.sh`).
  Real robot 2026-10-03/04 (bags/odom_test1.bag, 28 min, 20 walks; first bag overwritten):
  fix works: every stop reached |v|<0.03, |w|<0.05 within 0.05-0.5 s of WALK->HOLD (plus
  cmd_timeout 0.5 s after the last /cmd_vel), no "still moving" errors; ~0.8 s after zero
  velocity the robot switches itself back to mode 1. 2 loop gaps > 20 ms (max 25 ms), CRC errors 0-27/10 s.
- Odometry (robot's own estimate, body frame per walk):
  - +x is forward (user confirmed). Forward 0.1 m/s: odom 0.589 m vs ~0.572 m by eye (+3-5%).
  - Lateral: odom shows a steady drift right (~-0.01 m/s reported vy while walking; 0.6 m
    forward -> 0.11 m right) but the user saw only 1-2 in -> the estimate's vy is biased;
    give vy low trust in the EKF. Every walk also gets ~3 cm right in start/stop.
  - Yaw: at 0.1 rad/s commanded the robot turned at ~0.05 (121 s -> 351 deg, user agreed
    "a little short of 360"), so the IMU yaw is right and the robot under-turns at low
    yaw commands. 0.3 cmd -> ~0.2-0.25. move_base: min rotation >= 0.2 rad/s, rely on closed loop.
  - Backward: cap 0.1, but odom shows 0.11-0.17 m/s (0.418 m in 3.7 s, 0.538 m in 4.8 s):
    the robot overshoots backward commands. Rear camera sees only 0.18 m past the tail.
  - The robot's position is its own body velocity integrated with its own IMU yaw (bag: 0.22 m
    apart after ~6 m of walking). So no EKF yet: fusing /odom with the bridge IMU adds nothing.
    Add robot_localization when an independent source exists (ICP/visual odom, D430i IMU).
- First real test (robot STANDING via remote, user runs it): arm, check HOLD and
  /odom; walk a measured 1-2 m straight and a 360 deg turn, compare /odom
  (position may be in an unknown frame / reset at boot; velocity is body frame per comm.h;
  IMU assumed aligned with the body). Lie down with L2+A (not L2+B: damping drops the
  robot); any remote button also stops the bridge. No bringup -> no safety filter: this test
  publishes /cmd_vel directly at 0.1 m/s in a clear area.
  Open: does the remote read zero when untouched/switched off (else arming trips at once)?

## Open issues (priority order)
1. **Stop sending full clouds between boards**: DONE in code 2026-10-04 (zone_checker per
   camera, filter reads counts; `src/fetch_safety/test/run_tests.sh` 22/22 offline, which also
   fixed the old 8 rear/front_down failures). Not yet run with real cameras. 4 cameras measured
   well below 100 Mbit/s by the user even before this, so no separate congestion check.
2. **Harden the bridge**: DONE in code (arm/disarm 2026-10-03) except real-time
   priority (needs setcap/sudo, not approved). Not yet run against the real robot
   -> see "Bridge behaviour" above.
3. Solve the **front_down camera mount** (placeholder pitch 0.82 in bringup.launch)
   from the floor plane with the robot standing — only after 1 and 2.
4. .24 overheating (84-91 C, throttle alerts); fans spin (no tachometer).
5. Remote e-stop: Unitree remote battery was dead.

## Rules
- No camera/bringup/network-heavy tests while the robot is standing or a bridge is connected, without explicit OK.
- Claude never sends motion commands; the user runs them.
- System changes outside ~/projects/fetch need approval.

## Changes made outside ~/projects/fetch (all reversible)
- Clock sync (2026-10-04, `sudo boot/setup_timesync.sh` once per board; boards boot in 2018):
  .24 /etc/chrony/chrony.conf: `makestep 1 -1`, `allow 192.168.123.0/24`, `local stratum 10`
  (original in chrony.conf.fetch-orig); .23/.25 /etc/systemd/timesyncd.conf.d/fetch.conf
  (NTP=192.168.123.24) + timesyncd enabled. Undo: restore the .fetch-orig / delete fetch.conf,
  `timedatectl set-ntp false`. Replaces the manual `sudo date -s "$(curl ...)"` after boot.
  Installed + verified 2026-10-04: .23 -0.5 ms, .25 +0.2 ms vs .24 (NTP query). After boot,
  wait for `timedatectl` "synchronized: yes" before starting ROS.
- Unitree camera autostart disabled on .23/.24/.25 (`X-GNOME-Autostart-enabled=false` in
  `~/.config/autostart/b1camera.desktop`; originals in `~/projects/fetch/backup/`).
- `~/.config/autostart/fetch_usb_reset.desktop` on all three boards -> `~/projects/fetch/boot/usb_reset.sh`
  (USB hub reset + uvcvideo reload at login, which the Unitree autostart used to do).
- `ros-melodic-ddynamic-reconfigure` apt-installed on .23 and .25; realsense-ros built in `~/projects/fetch` there.
- .24's SSH key added to `~/.ssh/authorized_keys` on .23/.25; their RSA host keys added to .24's known_hosts.
- .23/.25 internet (only when needed, not persistent): `sudo ip route replace default via 192.168.123.165` + date set.

## Verified
- Filter offline tests (15/15), side mounts (floor -0.55..-0.58), front mount (level), rear mount (floor -0.575).
- Bridge: levelFlag 0xee, caps vx 0.3 / back 0.07 (was 0.1) / vy 0.2 / wz 0.5, idle_mode, stand-on-timeout (localhost tests).
- Robot walks via /cmd_vel_raw -> filter -> bridge (user confirmed).
- One nodelet manager per board fixes "device NOT found" when two cameras share a board.
