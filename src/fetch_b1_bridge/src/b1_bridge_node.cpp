// /cmd_vel -> B1 high-level controller (unitree_legged_sdk 3.8.3).
//
// Built the way Unitree's B1 example_walk.cpp talks to the robot: the UDP
// link is opened with HIGHLEVEL, the command is initialised once with
// InitCmdData() and every packet starts from that template, so header and
// levelFlag are never overwritten.
//
// Every packet carries a mode, and a lying B1 stands up from a few ms of
// mode 1 (2026-10-01). The robot only sends state in reply to packets, so the
// bridge cannot check whether the robot stands before sending. A person does:
// the bridge starts DISARMED and only sends after ~arm is called.
//
//   DISARMED   nothing is sent; /cmd_vel is ignored
//   ARMING     ~arm called (the person has checked the robot stands): send
//              idle_mode until the robot answers; the answer must not be
//              lying / damping / low, or the bridge stops
//   HOLD       armed, no fresh /cmd_vel: stand still (see below)
//   WALK       armed, fresh /cmd_vel: mode 2, clamped to max_* params
//   DISARMING  ~disarm called or ROS master lost: stand still for
//              settle_time, then DISARMED
//
// Standing still: while the robot reports mode 2 it ignores idle_mode and
// keeps walking at the last mode-2 velocity (2026-10-03: kept turning at
// 0.25 rad/s for 12 s of mode 1). So "stand still" is mode 2 at zero
// velocity while the robot reports mode 2, idle_mode otherwise, and the last
// mode-2 packet the bridge sends always has zero velocity.
//
// While armed the bridge sends every cycle, so state arrives continuously:
// it is published as /odom (+ TF odom -> base_link) and ~imu, and these stop
// the bridge for good (nothing more is sent, the node exits; restart it to
// resume):
//   - the robot reports lying down / damping (mode 5 or 7) or its body height
//     drops below min_body_height
//   - the robot stops answering for state_timeout
//   - the Unitree remote is used (any button, or a stick past stick_deadband)
// Only /cmd_vel received after ~arm counts, so a stale or latched message
// can't start the robot walking. If the ROS master goes away or restarts,
// the bridge disarms and exits: without a master nothing can command or stop
// it through ROS.
//
// Mode changes, faults, and a timing summary every 10 s go to rosout (and
// its log file); a state summary is published on ~state at ~10 Hz and the
// armed flag (latched) on ~armed.

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_broadcaster.h>
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

enum class Phase { kDisarmed, kArming, kHold, kWalk, kDisarming };
const char* name(Phase p) {
  switch (p) {
    case Phase::kDisarmed: return "DISARMED";
    case Phase::kArming: return "ARMING";
    case Phase::kHold: return "HOLD";
    case Phase::kWalk: return "WALK";
    case Phase::kDisarming: return "DISARMING";
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

// Diagonal 6x6 covariance (x y z roll pitch yaw).
void setDiagonal(boost::array<double, 36>& cov, std::initializer_list<double> diag) {
  cov.fill(0.0);
  int i = 0;
  for (double d : diag) cov[i * 6 + i] = d, ++i;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "b1_bridge");
  ros::NodeHandle nh, pnh("~");

  std::string target_ip, odom_frame, base_frame;
  int target_port, local_port, gait_type, idle_mode;
  double rate, cmd_timeout, max_vx, max_vx_back, max_vy, max_wz;
  double settle_time, state_timeout, min_body_height, stick_deadband, max_loop_gap, odom_rate;
  bool watch_remote, publish_tf;
  pnh.param<std::string>("target_ip", target_ip, "192.168.123.220");
  pnh.param("target_port", target_port, 8082);
  pnh.param("local_port", local_port, 8090);
  pnh.param("rate", rate, 500.0);
  pnh.param("cmd_timeout", cmd_timeout, 0.5);
  pnh.param("max_vx", max_vx, 0.3);
  // The rear camera only sees ~0.18 m past the tail, so backing up gets a
  // much lower cap than walking forward. The robot overshoots backward
  // commands (0.1 -> 0.11-0.17 m/s, 2026-10-04).
  pnh.param("max_vx_back", max_vx_back, 0.07);
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
  pnh.param<std::string>("odom_frame", odom_frame, "odom");
  pnh.param<std::string>("base_frame", base_frame, "base_link");
  pnh.param("odom_rate", odom_rate, 50.0);
  // Turn off when an EKF (robot_localization) publishes odom -> base_link.
  pnh.param("publish_tf", publish_tf, true);

  UDP udp(HIGHLEVEL, local_port, target_ip.c_str(), target_port);
  HighCmd templ = {0};
  udp.InitCmdData(templ);
  templ.mode = static_cast<uint8_t>(idle_mode);
  HighState state = {0};
  // zero velocity in the robot's current mode (see "Standing still" above)
  auto stillCmd = [&]() {
    HighCmd cmd = templ;
    if (state.mode == 2) {
      cmd.mode = 2;
      cmd.gaitType = static_cast<uint8_t>(gait_type);
    }
    return cmd;
  };

  geometry_msgs::Twist last_cmd;
  ros::Time last_cmd_time;
  ros::Subscriber sub = nh.subscribe<geometry_msgs::Twist>(
      "cmd_vel", 1, [&](const geometry_msgs::Twist::ConstPtr& msg) {
        last_cmd = *msg;
        last_cmd_time = ros::Time::now();
      });

  // Services only raise a flag; the loop below acts on it.
  bool arm_requested = false, disarm_requested = false;
  Phase phase = Phase::kDisarmed;
  ros::ServiceServer arm_srv = pnh.advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>(
      "arm", [&](std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
        res.success = phase == Phase::kDisarmed;
        res.message = res.success ? "arming: the robot must already be standing; watch ~armed / rosout"
                                  : std::string("not disarmed (") + name(phase) + ")";
        if (res.success) arm_requested = true;
        return true;
      });
  ros::ServiceServer disarm_srv = pnh.advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>(
      "disarm", [&](std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
        res.success = true;
        res.message = phase == Phase::kDisarmed ? "already disarmed" : "disarming";
        disarm_requested = true;
        return true;
      });

  ros::Publisher state_pub = pnh.advertise<std_msgs::String>("state", 1);
  ros::Publisher armed_pub = pnh.advertise<std_msgs::Bool>("armed", 1, true);
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
  ros::Publisher imu_pub = pnh.advertise<sensor_msgs::Imu>("imu", 10);
  tf2_ros::TransformBroadcaster tf_pub;
  MasterWatch master(1.0);

  auto publishArmed = [&](bool armed) {
    std_msgs::Bool msg;
    msg.data = armed;
    armed_pub.publish(msg);
  };
  publishArmed(false);

  ROS_INFO("b1_bridge: %s:%d from local port %d, levelFlag=0x%02x, idle mode %d, limits vx %.2f (back %.2f) vy %.2f wz %.2f",
           target_ip.c_str(), target_port, local_port, templ.levelFlag, idle_mode, max_vx, max_vx_back, max_vy, max_wz);
  ROS_INFO("b1_bridge: stops on body height < %.2f, mode 5/7, no state for %.2f s%s",
           min_body_height, state_timeout, watch_remote ? ", remote use" : "");
  ROS_INFO("b1_bridge: DISARMED, sends nothing. With the robot STANDING: rosservice call %s",
           arm_srv.getService().c_str());

  const auto period = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / rate));
  auto phase_start = Clock::now();
  auto episode_start = Clock::now();  // when sending started (DISARMED -> ARMING)
  auto last_state = Clock::now();
  auto last_odom = Clock::now();
  ros::Time arm_time;  // /cmd_vel received before this is ignored
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
    const bool was_armed = phase != Phase::kDisarmed, armed = p != Phase::kDisarmed;
    phase = p;
    phase_start = Clock::now();
    if (armed != was_armed) publishArmed(armed);
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
    const bool sending = phase != Phase::kDisarmed;
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
    const bool fresh_cmd = !master_lost && !last_cmd_time.isZero() && last_cmd_time >= arm_time &&
                           (ros::Time::now() - last_cmd_time).toSec() <= cmd_timeout;
    const bool disarm = disarm_requested || master_lost;
    disarm_requested = false;
    switch (phase) {
      case Phase::kDisarmed:
        if (master_lost) {
          fault = "ROS master lost or restarted";
        } else if (arm_requested) {
          enter(Phase::kArming, "arm requested");
          episode_start = phase_start;
          arm_time = ros::Time::now();
          have_state = false;  // ARMING needs an answer to this episode's packets
        } else if (!last_cmd_time.isZero() && (ros::Time::now() - last_cmd_time).toSec() <= cmd_timeout) {
          ROS_WARN_THROTTLE(5.0, "b1_bridge: /cmd_vel ignored, DISARMED (rosservice call %s)", arm_srv.getService().c_str());
        }
        break;
      case Phase::kArming:
        if (disarm) enter(Phase::kDisarming, master_lost ? "ROS master lost" : "disarm requested");
        else if (have_state) enter(Phase::kHold, "robot answered, standing");
        break;
      case Phase::kHold:
        if (disarm) enter(Phase::kDisarming, master_lost ? "ROS master lost" : "disarm requested");
        else if (fresh_cmd) enter(Phase::kWalk, "command received");
        break;
      case Phase::kWalk:
        if (disarm) enter(Phase::kDisarming, master_lost ? "ROS master lost" : "disarm requested");
        else if (!fresh_cmd) enter(Phase::kHold, "no command");
        break;
      case Phase::kDisarming:
        if (secondsSince(phase_start) >= settle_time) enter(Phase::kDisarmed, "settled");
        break;
    }
    arm_requested = false;
    if (!fault.empty()) break;

    // ---- send
    if (phase != Phase::kDisarmed) {
      HighCmd cmd = stillCmd();
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

    // ---- odometry: the robot's own estimate (position "usually drifts",
    // SDK comm.h), body velocities and IMU attitude. Only from fresh state of
    // a standing robot, i.e. while armed.
    if (fresh_state && phase != Phase::kDisarmed && phase != Phase::kArming && secondsSince(last_odom) >= 1.0 / odom_rate) {
      last_odom = now;
      const ros::Time stamp = ros::Time::now();
      geometry_msgs::Quaternion q;
      q.w = state.imu.quaternion[0];
      q.x = state.imu.quaternion[1];
      q.y = state.imu.quaternion[2];
      q.z = state.imu.quaternion[3];

      nav_msgs::Odometry odom;
      odom.header.stamp = stamp;
      odom.header.frame_id = odom_frame;
      odom.child_frame_id = base_frame;
      odom.pose.pose.position.x = state.position[0];
      odom.pose.pose.position.y = state.position[1];
      // base_link is the body centre: its height keeps the floor at z = 0
      odom.pose.pose.position.z = state.bodyHeight;
      odom.pose.pose.orientation = q;
      odom.twist.twist.linear.x = state.velocity[0];
      odom.twist.twist.linear.y = state.velocity[1];
      odom.twist.twist.angular.z = state.yawSpeed;
      // rough guesses until measured: leg odometry drifts, attitude is good
      setDiagonal(odom.pose.covariance, {0.01, 0.01, 0.01, 0.001, 0.001, 0.01});
      setDiagonal(odom.twist.covariance, {0.01, 0.01, 0.01, 0.01, 0.01, 0.01});
      odom_pub.publish(odom);

      if (publish_tf) {
        geometry_msgs::TransformStamped tf;
        tf.header = odom.header;
        tf.child_frame_id = base_frame;
        tf.transform.translation.x = odom.pose.pose.position.x;
        tf.transform.translation.y = odom.pose.pose.position.y;
        tf.transform.translation.z = odom.pose.pose.position.z;
        tf.transform.rotation = q;
        tf_pub.sendTransform(tf);
      }

      sensor_msgs::Imu imu;
      imu.header.stamp = stamp;
      imu.header.frame_id = base_frame;  // assumes the IMU is aligned with the body
      imu.orientation = q;
      imu.angular_velocity.x = state.imu.gyroscope[0];
      imu.angular_velocity.y = state.imu.gyroscope[1];
      imu.angular_velocity.z = state.imu.gyroscope[2];
      imu.linear_acceleration.x = state.imu.accelerometer[0];
      imu.linear_acceleration.y = state.imu.accelerometer[1];
      imu.linear_acceleration.z = state.imu.accelerometer[2];
      imu_pub.publish(imu);
    }

    // ---- the robot should come to rest within a few seconds of standing still
    const bool moving = std::fabs(state.velocity[0]) > 0.1 || std::fabs(state.velocity[1]) > 0.1 ||
                        std::fabs(state.yawSpeed) > 0.15;
    if ((phase == Phase::kHold || phase == Phase::kDisarming) && moving && secondsSince(phase_start) > 2.0)
      ROS_ERROR_THROTTLE(1.0, "b1_bridge: robot still moving %.1f s into %s: vel (%+.2f, %+.2f) yaw %+.2f, mode %u",
                         secondsSince(phase_start), name(phase), state.velocity[0], state.velocity[1], state.yawSpeed,
                         state.mode);

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
      // Recv() as "lost" (~500/s while DISARMED), which says nothing about the link.
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
    // The robot keeps walking at the last mode-2 velocity. If it stopped
    // answering while walking, try to leave it at zero before going silent.
    // Not on the other faults: the remote is in charge, or the robot is down.
    if (phase != Phase::kDisarmed && state.mode == 2 && fault.rfind("no state", 0) == 0) {
      HighCmd still = stillCmd();
      for (int i = 0; i < 50; ++i) {
        udp.SetSend(still);
        udp.Send();
        std::this_thread::sleep_for(period);
      }
    }
    publishArmed(false);
    ROS_ERROR("b1_bridge: STOPPED, nothing more is sent: %s. Restart the bridge to command the robot again.", fault.c_str());
    return 1;
  }
  // Ctrl+C while armed: leave the robot standing still, then go quiet. Not
  // while ARMING: the robot hasn't confirmed it stands.
  if (phase == Phase::kHold || phase == Phase::kWalk || phase == Phase::kDisarming) {
    HighCmd still = stillCmd();
    for (int i = 0; i < 50; ++i) {
      udp.SetSend(still);
      udp.Send();
      std::this_thread::sleep_for(period);
    }
  }
  return 0;
}
