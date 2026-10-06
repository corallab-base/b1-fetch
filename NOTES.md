# fetch — B1 safety filter: status checkpoint (2026-09-26)

## Startup (after a robot restart)
Everything runs from .23, the head (`ssh unitree@192.168.123.23`); .24 overheats.
After boot, on .23: `sudo ip route replace default via 192.168.123.165` (internet through the
user's Mac; deliberately not permanent), wait ~1 min until the date is right on .23 and
`timedatectl` says synchronized: yes, then start ROS (a clock jump mid-run breaks TF).
```bash
source ~/projects/fetch/env.sh                       # every terminal (master: .23)
~/projects/fetch/robot.sh      # checks (clocks, no leftover ROS/bridge/UDP 8082 on any board), then
                               # roslaunch fetch_safety robot.launch: bringup (front_infra on) + bridge
# stand the robot (remote L2+A, or b1_stand_up)
rosservice call /b1_bridge/arm                       # ONLY after checking the robot stands
rosrun fetch_nav teleop_keys.py                      # or: rostopic pub -r 10 /cmd_vel_raw ...
roslaunch fetch_nav nav.launch                       # navigation, own terminal (restart freely)
```
Shutdown: release keys -> `rosservice call /b1_bridge/disarm` -> L2+A or `b1_stand_down` ->
Ctrl+C robot.sh -> power off. The bridge is not required in robot.launch: when it exits
(remote used, robot lying, no state) cameras + filter stay up; restart it alone with
`roslaunch fetch_b1_bridge b1_bridge.launch idle_mode:=1`. A camera/filter failure still stops
everything, bridge included (silence stops a walking B1). `robot.sh --check` = checks only.
Separate commands still work: `roslaunch fetch_safety bringup.launch front_infra:=true` + bridge.

## Code on the boards (since 2026-10-04)
- One git repo, same code everywhere: GitHub origin (https://github.com/MahitMehta/fetch-unitree,
  pushed/pulled from the development board only, the user pushes) -> development board ->
  `./deploy.sh` pushes HEAD to ~/projects/fetch on the other boards over SSH and runs
  catkin_make there. Commit before deploying. Development board: .23 since 2026-10-07 (was .24).
  Never edit on the other boards directly
  (the push refuses if their checkout has local changes).
- build/, devel/, logs, bags are not in git; each board builds its own workspace.
- `env.sh` works on any board (ROS_IP auto-detected, master .23 unless FETCH_MASTER_URI is set);
  `remote_env.sh` is the roslaunch env-loader for nodes started on other boards.
- Unitree legged_sdk 3.8.3 (BSD-3, headers + arm64 lib) is in third_party/, so the bridge
  builds on every board. `.23` has SSH keys authorized on .24/.25 (2026-10-04), so
  bringup can be launched from .23; .24 also reaches .23/.25. roslaunch logs in with paramiko
  2.0.0, which can't use ed25519 keys: the RSA key (~/.ssh/id_rsa) is the one it needs.
- realsense-ros (Unitree's 2.3.2 + D430i support, realsense2_camera only) is in the repo
  since 2026-10-06, pinned to librealsense exactly 2.50.0 (/usr/local on .23/.25). .24 is the
  exception: its OpenCV 3.2 core library was moved to ~/opencv3_backup_safe (found via some
  runtime path, not the system dir), so ROS's cv_bridge can't be linked against there; deploy.sh
  puts a CATKIN_IGNORE on the repo copy and chains ~/realsense_ws (built before the move).
  Moving OpenCV 3.2 back would make .24 identical (needs sudo; ask why it was moved first).
  Verified: all 4 cameras at 15 Hz in bringup after the change. ~/.bashrc on .24 also sets
  ROS_IP/master (env.sh overrides).
- Every camera has a zone_checker on its own board (camera.launch) publishing
  /<cam>/zone_counts; the filter only reads those, so it (and later move_base) can run
  on any board. Clouds are still published for Foxglove and only cross the network
  while someone subscribes.

## Layout
| Board | Cameras (serial) | Role |
|---|---|---|
| .23 (head) | front 150622072829 (level), front_down 150622071014 (steep down) | ROS master, filter, scan merger, mounts, bridge, mapping (since 2026-10-04); development (git hub, Claude) since 2026-10-07 |
| .24 (body) | left 152122075367, right 150622073088 | side cameras; development until 2026-10-06; NTP server (Wi-Fi 192.168.11.110). Runs 85 C even idle -> hardware/airflow issue |
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
  denser scans for ICP: DONE 2026-10-06: zone_checker obstacle_cloud (left/right/front: band
  points thinned to 5 cm voxels, ~5 Hz) -> cloud_merger -> /obstacles; RTAB-Map uses that
  (scan_cloud) instead of /scan. Synthetic test: odometry 10% short, RTAB-Map corrected to the
  true 1.20 m with 0 rejected matches. Not yet on the robot. /scan stays (move_base clearing,
  +inf = clear); plan for move_base: /obstacles marks, /scan clears.
- Second drive 2026-10-06 (`lab2`, camera:=true + /obstacles; bags/mapping_lab2.bag 9.0 GB on .23
  incl. infrared + depth; maps/lab2.db): 686 nodes, 44 camera + 199 proximity loop closures
  (first drive: 0). Lab is a clean rectangle with the table island, plus a ~10 m space to the
  right. Exported maps/lab2.pgm/.yaml (21.8 x 15.6 m, 5 cm) for map_server (committed).
  RTAB-Map took ~0.5 s per update by the end (1 s budget): cap it (Rtabmap/TimeThr) for long
  continuous mapping. Images make bags ~2.3 MB/s: record them only when needed.

## Navigation (since 2026-10-06)
- `roslaunch fetch_nav nav.launch` (default map:=lab2, camera:=true): RTAB-Map keeps mapping into
  maps/<map>.db (continuous mapping) + move_base on its live grid. Goals: /move_base_simple/goal
  (Foxglove publish tool), frame map. Output /cmd_vel_nav -> filter (nav input: never reverses,
  ignores slow zones, stop zones + stale cameras still stop; manual /cmd_vel_raw overrides).
- Global costmap: rolling 30x30 m window with RTAB-Map's grid copied in (a map-sized costmap made
  goals in unmapped space unplannable). Obstacle layer: /obstacles marks, /scan clears.
- Local planner TEB (DWA dithered and never turned around when the path was behind the robot).
  oscillation_timeout 40 s (turning around took > 15 s). RTAB-Map node every 20 cm / ~6 deg.
- Offline sim `src/fetch_nav/test/run_sim_nav.sh` (fake robot + cameras in a 9x6 m room with a
  table island, odometry 3% short, real checkers/filter/RTAB-Map/move_base): both goals reached
  (40 s / 47 s, 0.16 m from target), >= 0.26 m from obstacles, no backward command.
- First real run 2026-10-06: Foxglove's goals didn't arrive (it advertised hostname `nx`, which
  every board resolves to itself: set Foxglove's ROS hostname to the Mac's IP, 192.168.123.165).
  Then no plan: the robot stood inside inflated cost next to clutter, and the global obstacle
  layer had piled up marks over the free map. Then a turn in place swung the left rear leg near
  an obstacle no camera saw; user stopped the bridge. Fixes: global costmap = RTAB-Map grid +
  inflation only (0.35 m); local costmap also has the RTAB-Map grid (remembers obstacles out of
  view, e.g. behind the rear legs); no rotate recovery; TEB obstacles up to 1.5 m behind; gentler
  turning (max 0.35 rad/s, acc 0.4) and weight_kinematics_nh 50 (heading had weaved +-80 deg).
  Sim after: both goals, 0.08/0.18 m, turning below the path's own bends, >= 0.22 m clearance.
  Terminal goals: `rosrun fetch_nav goal.py <ahead> [left] [turn deg]`, `goal.py --cancel`.
- Runs 2026-10-07 (logs analysed offline): jerky stop-go and corrections, then an abort.
  - TEB treated every local-costmap cell as an obstacle (2700-3900 with the RTAB-Map grid):
    control loop overran 1468x; nav commands went stale 82x (filter "commands from nav").
    Fixed: costmap_converter polygons (offline 39 -> 8 overruns), feasibility_check_no_poses 2,
    RTAB-Map under `nice -n 10` (81% of slow costmap loops fell in its >= 0.3 s updates).
    Next run: 0 control/costmap overruns.
  - "Failed to get a plan" (770x): Melodic GlobalPlanner ignores default_tolerance, so goals
    within ~0.28 m of a mapped obstacle never plan (reproduced offline). navfn honours the
    tolerance but its paths bend ~2x and TEB spun in the sim: kept GlobalPlanner. Open: snap
    goals to the nearest free cell; meanwhile keep goals >= 0.5 m from mapped furniture.
  - Abort "no valid control": map cells a few cm off touched the robot's right side (camera saw
    nothing there), so every TEB trajectory started in collision; recovery only clears > 2 m
    away. Fixed: map_relay.py gives the local costmap the map only behind/beside-rear (5/10 cm
    margins) and the global costmap the map without lone 1-4 cell specks in open floor (47 in
    today's map, each a ~0.7 m disc once inflated). Offline at a side-touching pose: old config
    held ~0 m/s, new drove 0.17 m/s. RTAB-Map adds map data only every 20 cm / ~6 deg of motion.
  - lab2.db has 6 sessions and 40% more obstacle cells than the 10-06 export: re-map
    (`nav.launch map:=lab3 fresh:=true`; without map:= fresh deletes lab2.db).

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
