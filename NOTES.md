# fetch — B1 safety filter: status checkpoint (2026-09-26)

## Startup (after a robot restart)
Everything runs from .23, the head (`ssh unitree@192.168.123.23`); .24 overheats.
After boot, on .23: `sudo ip route replace default via 192.168.123.165` (internet through the
user's Mac; deliberately not permanent), wait ~1 min until the date is right on .23 and
`timedatectl` says synchronized: yes, then start ROS (a clock jump mid-run breaks TF).
```bash
source ~/projects/fetch/env.sh                       # every terminal (master: .23)
roslaunch fetch_safety bringup.launch                # cameras on their boards, filter + /scan on .23
# stand the robot (remote L2+A, or b1_stand_up)
roslaunch fetch_b1_bridge b1_bridge.launch idle_mode:=1   # starts DISARMED, sends nothing
rosservice call /b1_bridge/arm                       # ONLY after checking the robot stands
rosrun fetch_nav teleop_keys.py                      # or: rostopic pub -r 10 /cmd_vel_raw ...
```
Shutdown: release keys -> `rosservice call /b1_bridge/disarm` -> Ctrl+C bridge ->
L2+A or `b1_stand_down` -> Ctrl+C bringup -> power off.

## Code on the boards (since 2026-10-04)
- One git repo, same code everywhere: GitHub origin (https://github.com/MahitMehta/fetch-unitree,
  pushed/pulled from .24 only; .24 has no stored GitHub login, the user pushes) -> .24 ->
  `./deploy.sh` pushes HEAD to ~/projects/fetch on the other boards over SSH (.23/.25 have no
  internet) and runs catkin_make there. Commit before deploying. Never edit on .23/.25 directly
  (the push refuses if their checkout has local changes).
- build/, devel/, logs, bags are not in git; each board builds its own workspace.
- `env.sh` works on any board (ROS_IP auto-detected, master .23 unless FETCH_MASTER_URI is set);
  `remote_env.sh` is the roslaunch env-loader for nodes started on other boards.
- Unitree legged_sdk 3.8.3 (BSD-3, headers + arm64 lib) is in third_party/, so the bridge
  builds on every board. `.23` has SSH keys authorized on .24/.25 (2026-10-04), so
  bringup can be launched from .23; .24 also reaches .23/.25. roslaunch logs in with paramiko
  2.0.0, which can't use ed25519 keys: the RSA key (~/.ssh/id_rsa) is the one it needs.
- Per-board differences left: realsense-ros 2.3.2 is in src/ on .23/.25
  (ignored by git) but in ~/realsense_ws on .24. ~/.bashrc on .24 also sets ROS_IP/master.
- Every camera has a zone_checker on its own board (camera.launch) publishing
  /<cam>/zone_counts; the filter only reads those, so it (and later move_base) can run
  on any board. Clouds are still published for Foxglove and only cross the network
  while someone subscribes.

## Layout
| Board | Cameras (serial) | Role |
|---|---|---|
| .23 (head) | front 150622072829 (level), front_down 150622071014 (steep down) | ROS master, filter, scan merger, mounts, bridge, mapping (since 2026-10-04) |
| .24 (body) | left 152122075367, right 150622073088 | side cameras; development (git hub, Claude); NTP server (Wi-Fi 192.168.11.110). Runs 85 C even idle -> hardware/airflow issue |
| .25 (tail) | rear 150622073015 (straight down, USB 2.1 -> 480x270) | rear camera (can't see behind, see Rear coverage); MODE_15W_2CORE unlike the others |
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

## Mapping (since 2026-10-04)
- Every zone_checker also publishes /<cam>/scan (LaserScan in base_link, 360 x 1 deg, <= 4 m,
  nearest point in the safety height band, robot body_box excluded; +inf = saw floor only,
  NaN = not looked). scan_merger_node (in bringup) -> /scan, ~1.5 KB/msg.
- fetch_nav: `mapping.launch` (RTAB-Map 0.20 scan-only ICP SLAM on /scan + TF odom, grid
  /rtabmap/grid_map, db in ~/projects/fetch/maps/<map>.db, git-ignored), `teleop_keys.py`
  (hold-to-move -> /cmd_vel_raw), `record_mapping.sh` (small bag: scans, odom, tf, state).
- Offline smoke test: synthetic 4x3 m room -> walls reproduced to 1 cm. Not yet on the robot.
- Coverage: front + sides only (~260 deg minus gaps), 3 m camera clip, so maps need slow
  drives with turns; no loop closure by appearance (no images), only by proximity.
- First real drive 2026-10-04 (bags/mapping_lab.bag on .23, 37 min, 40.5 m, room ~9.5x6.3 m
  with a table/boxes island; maps/lab.db): 225 nodes, NO loop closure although the robot came
  back within 0.65 m. Map shows the room but smeared/doubled walls (drift). Replays found why:
  ICP on /scan almost always fails (<= ~255 points, mean 44 correspondences, ratio ~0.17,
  rejected), so both neighbour refinement and proximity closures fail; the map is ~pure leg
  odometry. Fixes in progress: front-camera place recognition (mapping.launch camera:=true,
  Reg/Strategy 0, infrared image with 3x3 median vs projector dots, 161 words/image);
  denser scans for ICP (obstacle cloud instead of 1 deg nearest-point scan) still open.

## Rear coverage (2026-10-04, robot standing, measured on .25)
The rear camera (88.7 deg down) sees the floor only from x -0.71 to -0.21, mostly under the
robot. At obstacle height it reaches at most 7 cm into the rear zones (x <= -0.60), nothing
above ~0.3 m off the floor; points in the z band are the hind legs (footprint). Hand/feet
behind the robot never trigger. So:
- Filter inputs (safety_filter.yaml): `manual` = /cmd_vel_raw (highest priority, backward
  allowed, bridge cap 0.07) and `nav` = /cmd_vel_nav for move_base (backward always blocked).
  Manual overrides nav while fresh. Status shows `backward=OK[not covered]`.
- Proper fix (user's call, hardware): remount the rear camera ~30-40 deg down from horizontal
  to cover ~0.6-1.5 m behind the tail, then re-solve its mount from the floor.
- Bringup with real cameras 2026-10-04: all 4 zone_counts at 15 Hz; front/left/right react to
  a hand; rear never does (above).

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
  MOVED to .23 (2026-10-04): `sudo boot/setup_timesync.sh` on .23 (chrony installed, makestep
  1 -1, serves 192.168.123.0/24, local stratum 10), .24 (chrony disabled, its conf restored from
  .fetch-orig, timesyncd -> .23), .25 (timesyncd -> .23). Verified: .24/.25 -0.1 ms vs .23.
  .23's internet route is added by hand each boot (user's choice); until then it serves its own
  (2018) clock and the boards still agree. boot/setup_gateway.sh (permanent route) exists, unused.
- All boards' /etc/rc.local run `jetson_clocks` (CPU/GPU pinned at max clocks) and fan 255:
  boards stay warm even idle; part of why .24 sits at ~85 C. Not changed.
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
