# fetch — B1 safety filter: status checkpoint (2026-09-26)

## Startup (after a robot restart)
```bash
source ~/projects/fetch/env.sh                       # every terminal
roslaunch fetch_safety bringup.launch                # 4 cameras (.24/.23/.25 over SSH) + safety filter; front_down:=true adds the 5th
~/projects/fetch/devel/lib/fetch_b1_bridge/b1_stand_up
roslaunch fetch_b1_bridge b1_bridge.launch idle_mode:=1
rostopic pub -r 10 /cmd_vel_raw geometry_msgs/Twist '{linear: {x: 0.0, y: 0.1, z: 0.0}}'   # always /cmd_vel_raw
```
Shutdown: Ctrl+C pub -> Ctrl+C bridge -> `b1_stand_down` -> Ctrl+C bringup -> power off.

> **Do not use this with the robot standing until the open issues below are fixed.**

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
- test_safety_filter.py only fakes left/right/front clouds: with rear/front_down in the default
  sources it fails 8 cases (stale sources block forward/backward). Passes 15/15 with only those
  3 sources loaded. The test needs updating.

## Bridge behaviour (since 2026-10-01)
- Silent until /cmd_vel arrives. CHECK: sends idle_mode until the robot answers
  standing (usually 1 packet), then WALK. After /cmd_vel stops: 1 s of idle_mode, then silent.
- Stops for good (sends nothing more, node exits with an ERROR line) on: reported
  mode 5/7, body height < 0.40, no state for 0.2 s while sending, any Unitree remote
  button/stick, ROS master lost/restarted. Restart the bridge to resume.
- Logs mode changes, loop gaps > 20 ms, and a 10 s summary (sent/received, worst
  gap, UDP errors) to rosout. First localhost run already showed a 46 ms gap on .24
  with bringup running.
- Open questions for the first real test (robot LYING, user runs it):
  1. Does the remote's data read as zero when untouched / switched off, or does it
     trip the remote check at once?
  2. ANSWERED 2026-10-01: .220 sends state only in reply (received 0 while QUIET,
     also during remote L2+A up/down). So the remote is only watched while sending,
     and CHECK's 1-2 packets can't be avoided. Remote L2+A stand/lie worked with
     the bridge running QUIET (sent 0).
  3. ANSWERED 2026-10-01, BAD: zero /cmd_vel_raw with robot lying -> CHECK sent ~2
     mode-1 packets (4 ms until the reply), bridge stopped on "mode 7", but the robot
     STOOD UP anyway: a lying B1 completes a stand-up from a few ms of mode 1.
     Any packet carries a mode, and state only comes back in reply, so CHECK can't
     be made safe. Proposed redesign (not yet approved): start disarmed, arm by
     explicit service call with robot standing, then send continuously and exit on
     remote use / height / mode 5,7 / no state / master loss.
  4. Does L2+B still work while the bridge is sending? (It should also stop the bridge.)

## Open issues (priority order)
1. **Stop sending full clouds between boards**: run the zone check on each camera
   board and send only per-zone point counts; keep a decimated/low-rate cloud
   (or compressedDepth image) for Foxglove, sent only when viewed.
2. **Harden the bridge**: DONE in code 2026-10-01 except real-time priority (needs
   setcap/sudo, not approved). Passed 12 localhost scenarios
   (`src/fetch_b1_bridge/test/test_bridge.sh`, fake robot on 127.0.0.1). Not yet
   run against the real robot -> see "Bridge behaviour" below.
3. Solve the **front_down camera mount** (placeholder pitch 0.82 in bringup.launch)
   from the floor plane with the robot standing — only after 1 and 2.
4. .24 overheating (84-91 C, throttle alerts); fans spin (no tachometer).
5. Remote e-stop: Unitree remote battery was dead.

## Rules
- No camera/bringup/network-heavy tests while the robot is standing or a bridge is connected, without explicit OK.
- Claude never sends motion commands; the user runs them.
- System changes outside ~/projects/fetch need approval.

## Changes made outside ~/projects/fetch (all reversible)
- Unitree camera autostart disabled on .23/.24/.25 (`X-GNOME-Autostart-enabled=false` in
  `~/.config/autostart/b1camera.desktop`; originals in `~/projects/fetch/backup/`).
- `~/.config/autostart/fetch_usb_reset.desktop` on all three boards -> `~/projects/fetch/boot/usb_reset.sh`
  (USB hub reset + uvcvideo reload at login, which the Unitree autostart used to do).
- `ros-melodic-ddynamic-reconfigure` apt-installed on .23 and .25; realsense-ros built in `~/projects/fetch` there.
- .24's SSH key added to `~/.ssh/authorized_keys` on .23/.25; their RSA host keys added to .24's known_hosts.
- .23/.25 internet (only when needed, not persistent): `sudo ip route replace default via 192.168.123.165` + date set.

## Verified
- Filter offline tests (15/15), side mounts (floor -0.55..-0.58), front mount (level), rear mount (floor -0.575).
- Bridge: levelFlag 0xee, caps vx 0.3 / back 0.1 / vy 0.2 / wz 0.5, idle_mode, stand-on-timeout (localhost tests).
- Robot walks via /cmd_vel_raw -> filter -> bridge (user confirmed).
- One nodelet manager per board fixes "device NOT found" when two cameras share a board.
