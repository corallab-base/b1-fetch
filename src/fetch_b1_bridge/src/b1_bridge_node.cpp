// /cmd_vel -> B1 high-level controller (unitree_legged_sdk 3.8.3).
//
// Built the way Unitree's B1 example_walk.cpp talks to the robot: the UDP
// link is opened with HIGHLEVEL, the command is initialised once with
// InitCmdData() and every packet starts from that template, so header and
// levelFlag are never overwritten.
//
// The bridge only sends while it has something to say:
//
//   QUIET   nothing is sent; the robot holds its pose on its own
//   CHECK   fresh /cmd_vel arrived: send idle_mode (zero velocity) until the
//           robot answers and reports standing, then WALK
//   WALK    mode 2 (walk at velocity), clamped to max_* params
//   SETTLE  /cmd_vel went stale: send idle_mode for settle_time so the robot
//           stops walking, then QUIET
//
// It never argues with the robot. While it is sending, any of these stops it
// for good (nothing more is sent, the node exits; restart it to resume):
//   - the robot reports lying down / damping (mode 5 or 7) or its body height
//     drops below min_body_height
//   - the robot stops answering for state_timeout
//   - the Unitree remote is used (any button, or a stick past stick_deadband)
// The remote is watched whenever state arrives, including while QUIET.
// If the ROS master goes away or restarts, the bridge settles and exits:
// without a master nothing can command or stop it through ROS.
//
// Mode changes, faults, and a timing summary every 10 s go to rosout (and
// its log file); the reported state is also published on ~state at ~10 Hz.

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

using namespace UNITREE_LEGGED_SDK;
using Clock = std::chrono::steady_clock;

namespace {

double clamp(double v, double limit) { return std::max(-limit, std::min(limit, v)); }
double clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }
double secondsSince(Clock::time_point t) { return std::chrono::duration<double>(Clock::now() - t).count(); }

enum class Phase { kQuiet, kCheck, kWalk, kSettle };
const char* name(Phase p) {
  switch (p) {
    case Phase::kQuiet: return "QUIET";
    case Phase::kCheck: return "CHECK";
    case Phase::kWalk: return "WALK";
    case Phase::kSettle: return "SETTLE";
  }
  return "?";
}

// Polls the master once a second from its own thread (the call can block).
// Lost = unreachable, or answering with a different PID (a new master that
// doesn't know this node).
class MasterWatch {
 public:
  explicit MasterWatch(double period) {
    thread_ = std::thread([this, period] {
      int first_pid = -1;
      while (!stop_) {
        XmlRpc::XmlRpcValue args, result, payload;
        args[0] = ros::this_node::getName();
        int pid = -1;
        if (ros::master::execute("getPid", args, result, payload, false)) pid = static_cast<int>(payload);
        if (first_pid < 0) first_pid = pid;
        if (pid < 0 || pid != first_pid) {
          lost_ = true;
          return;
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(period));
      }
    });
  }
  ~MasterWatch() {
    stop_ = true;
    thread_.detach();  // may be stuck in an XML-RPC call; don't wait for it
  }
  bool lost() const { return lost_; }

 private:
  std::thread thread_;
  std::atomic<bool> stop_{false}, lost_{false};
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "b1_bridge");
  ros::NodeHandle nh, pnh("~");

  std::string target_ip;
  int target_port, local_port, gait_type, idle_mode;
  double rate, cmd_timeout, max_vx, max_vx_back, max_vy, max_wz;
  double settle_time, state_timeout, min_body_height, stick_deadband, max_loop_gap;
  bool watch_remote;
  pnh.param<std::string>("target_ip", target_ip, "192.168.123.220");
  pnh.param("target_port", target_port, 8082);
  pnh.param("local_port", local_port, 8090);
  pnh.param("rate", rate, 500.0);
  pnh.param("cmd_timeout", cmd_timeout, 0.5);
  pnh.param("max_vx", max_vx, 0.3);
  // The rear camera only sees ~0.18 m past the tail, so backing up gets a
  // much lower cap than walking forward.
  pnh.param("max_vx_back", max_vx_back, 0.1);
  pnh.param("max_vy", max_vy, 0.2);
  pnh.param("max_wz", max_wz, 0.5);
  pnh.param("gait_type", gait_type, 0);  // Unitree's B1 example walks with 0
  pnh.param("idle_mode", idle_mode, 0);  // 0 idle stand, 1 balance stand
  pnh.param("settle_time", settle_time, 1.0);
  pnh.param("state_timeout", state_timeout, 0.2);
  // Reported height: ~0.53-0.56 standing, ~0.07 lying (2026-09-26 runs).
  pnh.param("min_body_height", min_body_height, 0.40);
  pnh.param("watch_remote", watch_remote, true);
  pnh.param("stick_deadband", stick_deadband, 0.3);
  pnh.param("max_loop_gap", max_loop_gap, 0.02);

  UDP udp(HIGHLEVEL, local_port, target_ip.c_str(), target_port);
  HighCmd templ = {0};
  udp.InitCmdData(templ);
  templ.mode = static_cast<uint8_t>(idle_mode);
  HighState state = {0};

  geometry_msgs::Twist last_cmd;
  ros::Time last_cmd_time;
  ros::Subscriber sub = nh.subscribe<geometry_msgs::Twist>(
      "cmd_vel", 1, [&](const geometry_msgs::Twist::ConstPtr& msg) {
        last_cmd = *msg;
        last_cmd_time = ros::Time::now();
      });
  ros::Publisher state_pub = pnh.advertise<std_msgs::String>("state", 1);
  MasterWatch master(1.0);

  ROS_INFO("b1_bridge: %s:%d from local port %d, levelFlag=0x%02x, idle mode %d, limits vx %.2f (back %.2f) vy %.2f wz %.2f",
           target_ip.c_str(), target_port, local_port, templ.levelFlag, idle_mode, max_vx, max_vx_back, max_vy, max_wz);
  ROS_INFO("b1_bridge: stops on body height < %.2f, mode 5/7, no state for %.2f s%s; QUIET until /cmd_vel arrives",
           min_body_height, state_timeout, watch_remote ? ", remote use" : "");

  const auto period = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / rate));
  Phase phase = Phase::kQuiet;
  auto phase_start = Clock::now();
  auto episode_start = Clock::now();  // when sending started (QUIET -> CHECK)
  auto last_state = Clock::now();
  unsigned long long last_rx = 0;
  bool have_state = false;
  uint8_t last_mode = 0xff;
  std::string fault;

  // timing / link stats, reported every 10 s
  auto next = Clock::now(), last_tick = Clock::now(), stats_start = Clock::now();
  double worst_gap = 0.0;
  int long_gaps = 0, sent = 0, received = 0;
  unsigned long long crc_errors_before = 0;

  auto enter = [&](Phase p, const char* why) {
    ROS_INFO("b1_bridge: %s -> %s (%s)", name(phase), name(p), why);
    phase = p;
    phase_start = Clock::now();
  };

  for (long tick = 0; ros::ok(); ++tick) {
    const auto now = Clock::now();
    const double gap = std::chrono::duration<double>(now - last_tick).count();
    last_tick = now;
    if (tick > 0) {
      worst_gap = std::max(worst_gap, gap);
      if (gap > max_loop_gap) {
        ++long_gaps;
        ROS_WARN_THROTTLE(1.0, "b1_bridge: loop gap %.1f ms (limit %.0f ms) in %s", gap * 1e3, max_loop_gap * 1e3, name(phase));
      }
    }

    ros::spinOnce();

    // ---- robot state
    udp.Recv();
    const bool fresh_state = udp.udpState.RecvCount != last_rx;
    if (fresh_state) {
      last_rx = udp.udpState.RecvCount;
      udp.GetRecv(state);
      last_state = now;
      have_state = true;
      ++received;
      if (state.mode != last_mode) {
        ROS_INFO("b1_bridge: robot reports mode %u, height %.3f (%s)", state.mode, state.bodyHeight, name(phase));
        last_mode = state.mode;
      }
    }
    const bool sending = phase != Phase::kQuiet;
    char why[160] = "";
    if (fresh_state && watch_remote) {
      xRockerBtnDataStruct keys;
      std::memcpy(&keys, &state.wirelessRemote[0], sizeof(keys));
      const float stick = std::max({std::fabs(keys.lx), std::fabs(keys.ly), std::fabs(keys.rx), std::fabs(keys.ry)});
      if (keys.btn.value != 0 || stick > stick_deadband)
        std::snprintf(why, sizeof(why), "Unitree remote used (buttons 0x%04x, stick %.2f)", keys.btn.value, stick);
    }
    if (!why[0] && sending && fresh_state) {
      if (state.mode == 5 || state.mode == 7)
        std::snprintf(why, sizeof(why), "robot reports mode %u (lying down / damping), height %.3f", state.mode, state.bodyHeight);
      else if (state.bodyHeight < min_body_height)
        std::snprintf(why, sizeof(why), "body height %.3f below %.2f (mode %u)", state.bodyHeight, min_body_height, state.mode);
    }
    if (!why[0] && sending && secondsSince(std::max(last_state, episode_start)) > state_timeout)
      std::snprintf(why, sizeof(why), "no state from the robot for %.0f ms of sending",
                    secondsSince(std::max(last_state, episode_start)) * 1e3);
    if (why[0]) {
      fault = why;
      break;
    }

    // ---- phase
    const bool master_lost = master.lost();
    const bool fresh_cmd = !master_lost && !last_cmd_time.isZero() &&
                           (ros::Time::now() - last_cmd_time).toSec() <= cmd_timeout;
    switch (phase) {
      case Phase::kQuiet:
        if (master_lost) {
          fault = "ROS master lost or restarted";
        } else if (fresh_cmd) {
          enter(Phase::kCheck, "command received");
          episode_start = phase_start;
          have_state = false;  // CHECK needs an answer to this episode's packets
        }
        break;
      case Phase::kCheck:
        if (!fresh_cmd) enter(Phase::kSettle, "command stopped before the robot answered");
        else if (have_state) enter(Phase::kWalk, "robot answered, standing");
        break;
      case Phase::kWalk:
        if (!fresh_cmd) enter(Phase::kSettle, master_lost ? "ROS master lost" : "no command");
        break;
      case Phase::kSettle:
        if (fresh_cmd) enter(Phase::kWalk, "command received");
        else if (secondsSince(phase_start) >= settle_time) enter(Phase::kQuiet, "settled");
        break;
    }
    if (!fault.empty()) break;

    // ---- send
    if (phase != Phase::kQuiet) {
      HighCmd cmd = templ;  // idle_mode, zero velocity
      if (phase == Phase::kWalk) {
        cmd.mode = 2;
        cmd.gaitType = static_cast<uint8_t>(gait_type);
        cmd.velocity[0] = clamp(last_cmd.linear.x, -max_vx_back, max_vx);
        cmd.velocity[1] = clamp(last_cmd.linear.y, max_vy);
        cmd.yawSpeed = clamp(last_cmd.angular.z, max_wz);
      }
      udp.SetSend(cmd);
      udp.Send();
      ++sent;
    }

    // ---- reporting
    if (tick % std::max(1, static_cast<int>(rate / 10.0)) == 0) {
      char buf[256];
      std::snprintf(buf, sizeof(buf), "%s | robot mode=%u gait=%u vel=(%+.3f, %+.3f) yaw=%+.3f height=%.3f | state age %.0f ms",
                    name(phase), state.mode, state.gaitType, state.velocity[0], state.velocity[1], state.yawSpeed,
                    state.bodyHeight, secondsSince(last_state) * 1e3);
      std_msgs::String msg;
      msg.data = buf;
      state_pub.publish(msg);
    }
    if (secondsSince(stats_start) >= 10.0) {
      // RecvLoseError is not reported: the SDK counts every empty non-blocking
      // Recv() as "lost" (~500/s while QUIET), which says nothing about the link.
      ROS_INFO("b1_bridge: %s, last 10 s: sent %d, received %d, CRC errors %llu, worst loop gap %.1f ms, gaps > %.0f ms: %d",
               name(phase), sent, received, udp.udpState.RecvCRCError - crc_errors_before, worst_gap * 1e3,
               max_loop_gap * 1e3, long_gaps);
      crc_errors_before = udp.udpState.RecvCRCError;
      stats_start = Clock::now();
      worst_gap = 0.0;
      long_gaps = sent = received = 0;
    }

    // ---- wait; after a stall, restart the schedule instead of bursting to catch up
    next += period;
    if (Clock::now() > next + period) next = Clock::now();
    std::this_thread::sleep_until(next);
  }

  if (!fault.empty()) {
    ROS_ERROR("b1_bridge: STOPPED, nothing more is sent: %s. Restart the bridge to command the robot again.", fault.c_str());
    return 1;
  }
  // Ctrl+C: if we were commanding, leave the robot standing still, then go quiet.
  if (phase == Phase::kWalk || phase == Phase::kSettle) {
    for (int i = 0; i < 50; ++i) {
      udp.SetSend(templ);
      udp.Send();
      std::this_thread::sleep_for(period);
    }
  }
  return 0;
}
