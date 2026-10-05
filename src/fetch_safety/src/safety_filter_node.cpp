// Velocity safety filter (ROS1 take on Nav2's collision_monitor).
//
// Subscribes to raw velocity commands and to point clouds from one or more
// depth cameras. Points are transformed into the robot base frame, cropped to
// a height band, and counted inside per-direction polygon zones. A zone with
// enough points either stops or slows the velocity component heading toward
// it. A source whose data goes stale blocks every direction it covers, so a
// dead or disconnected camera fails safe.
//
// A source is either a cloud (type: cloud, counted here) or the per-zone
// counts a zone_checker_node computed from a cloud on the camera's own board
// (type: counts), so remote cameras send counts instead of clouds over the
// network. Counts whose zone names don't match this node's zones are ignored
// (the source goes stale and blocks).
//
// Commands come from one or more inputs in priority order (~inputs; default
// just cmd_vel_raw): the first one with a fresh command drives, so manual
// teleop overrides navigation. Each input can have directions it may never
// move in (blocked_directions), e.g. navigation never reverses because no
// camera covers the zone behind the robot (~uncovered_directions, shown in
// the status), and can ignore slowdown zones (ignore_slowdown): move_base
// avoids obstacles with its costmaps, so for it the filter only stops.

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <fetch_safety/ZoneCounts.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <boost/bind.hpp>

#include <sstream>

#include "fetch_safety/zones.h"

namespace fetch_safety {

struct Source {
  std::string name;
  std::string topic;
  bool counts;  // true: fetch_safety/ZoneCounts from a zone_checker, false: PointCloud2
  std::vector<Direction> directions;
  ros::Subscriber sub;
  ros::Time last_update;        // receive time of the last cloud/counts used; zero = never
  std::vector<int> zone_counts;  // points per zone in the last cloud
};

struct Input {
  std::string name;
  std::vector<Direction> blocked;  // never moves this way, whatever the cameras see
  bool ignore_slowdown = false;    // only stop zones and stale cameras apply
  ros::Subscriber sub;
  geometry_msgs::Twist cmd;
  ros::Time time;  // receive time of the last command; zero = never
};

struct DirectionState {
  bool stop = false;
  double scale = 1.0;
  std::vector<std::string> reasons;
};

class SafetyFilter {
 public:
  SafetyFilter(ros::NodeHandle& nh, ros::NodeHandle& pnh) : tf_listener_(tf_buffer_) {
    pnh.param<std::string>("base_frame", base_frame_, "base_link");
    pnh.param("rate", rate_, 20.0);
    pnh.param("cmd_timeout", cmd_timeout_, 0.5);
    pnh.param("source_timeout", source_timeout_, 0.3);
    pnh.param("min_z", min_z_, -0.35);
    pnh.param("max_z", max_z_, 0.35);
    pnh.param("point_stride", point_stride_, 1);
    pnh.param("side_block_yaw_scale", side_block_yaw_scale_, 1.0);
    point_stride_ = std::max(1, point_stride_);

    zones_ = loadZones(pnh);
    loadSources(pnh);
    loadInputs(pnh);
    std::vector<std::string> uncovered;
    pnh.param("uncovered_directions", uncovered, uncovered);
    for (const auto& d : uncovered) uncovered_.push_back(parseDirection(d));

    for (size_t i = 0; i < sources_.size(); ++i) {
      sources_[i].zone_counts.assign(zones_.size(), 0);
      if (sources_[i].counts) {
        sources_[i].sub = nh.subscribe<fetch_safety::ZoneCounts>(
            sources_[i].topic, 1, boost::bind(&SafetyFilter::countsCallback, this, _1, i));
      } else {
        sources_[i].sub = nh.subscribe<sensor_msgs::PointCloud2>(
            sources_[i].topic, 1, boost::bind(&SafetyFilter::cloudCallback, this, _1, i));
      }
    }

    for (size_t i = 0; i < inputs_.size(); ++i) {
      inputs_[i].sub = nh.subscribe<geometry_msgs::Twist>(
          topics_[i], 1, boost::bind(&SafetyFilter::cmdCallback, this, _1, i));
    }
    cmd_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    status_pub_ = pnh.advertise<std_msgs::String>("status", 1, true);
    marker_pub_ = pnh.advertise<visualization_msgs::MarkerArray>("zones", 1);

    timer_ = nh.createTimer(ros::Duration(1.0 / rate_), &SafetyFilter::timerCallback, this);
    marker_timer_ = nh.createTimer(ros::Duration(0.2), &SafetyFilter::markerCallback, this);

    ROS_INFO("safety_filter: %zu zones, %zu sources, base_frame=%s, z band [%.2f, %.2f]",
             zones_.size(), sources_.size(), base_frame_.c_str(), min_z_, max_z_);
  }

 private:
  // ~inputs: [{name, topic, blocked_directions: [...]}, ...], highest priority first
  void loadInputs(ros::NodeHandle& pnh) {
    XmlRpc::XmlRpcValue list;
    if (!pnh.getParam("inputs", list)) {
      inputs_.resize(1);
      inputs_[0].name = "cmd_vel_raw";
      topics_.push_back("cmd_vel_raw");
    } else {
      if (list.getType() != XmlRpc::XmlRpcValue::TypeArray) throw std::runtime_error("~inputs must be a list");
      for (int i = 0; i < list.size(); ++i) {
        XmlRpc::XmlRpcValue& in = list[i];
        Input input;
        input.name = static_cast<std::string>(in["name"]);
        topics_.push_back(static_cast<std::string>(in["topic"]));
        if (in.hasMember("ignore_slowdown")) input.ignore_slowdown = static_cast<bool>(in["ignore_slowdown"]);
        if (in.hasMember("blocked_directions")) {
          XmlRpc::XmlRpcValue& dirs = in["blocked_directions"];
          for (int d = 0; d < dirs.size(); ++d) input.blocked.push_back(parseDirection(static_cast<std::string>(dirs[d])));
        }
        inputs_.push_back(input);
      }
    }
  }

  void loadSources(ros::NodeHandle& pnh) {
    XmlRpc::XmlRpcValue list;
    if (!pnh.getParam("sources", list) || list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      throw std::runtime_error("~sources must be a list");
    }
    for (int i = 0; i < list.size(); ++i) {
      XmlRpc::XmlRpcValue& s = list[i];
      Source source;
      source.name = static_cast<std::string>(s["name"]);
      source.topic = static_cast<std::string>(s["topic"]);
      const std::string type = s.hasMember("type") ? static_cast<std::string>(s["type"]) : "cloud";
      if (type != "cloud" && type != "counts") {
        throw std::runtime_error("source " + source.name + ": type must be cloud or counts");
      }
      source.counts = type == "counts";
      XmlRpc::XmlRpcValue& dirs = s["directions"];
      for (int d = 0; d < dirs.size(); ++d) {
        source.directions.push_back(parseDirection(static_cast<std::string>(dirs[d])));
      }
      sources_.push_back(source);
    }
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg, size_t index) {
    Source& source = sources_[index];

    geometry_msgs::TransformStamped tf_msg;
    try {
      // Cameras are rigidly mounted, so the latest transform is good enough.
      tf_msg = tf_buffer_.lookupTransform(base_frame_, msg->header.frame_id, ros::Time(0));
    } catch (const tf2::TransformException& e) {
      ROS_WARN_THROTTLE(2.0, "safety_filter: [%s] no transform %s -> %s: %s", source.name.c_str(),
                        msg->header.frame_id.c_str(), base_frame_.c_str(), e.what());
      return;
    }
    const auto tf = tf2::transformToEigen(tf_msg);
    std::string error;
    if (!countZones(*msg, tf.linear().cast<float>(), tf.translation().cast<float>(), zones_, min_z_, max_z_,
                    point_stride_, source.zone_counts, error)) {
      ROS_WARN_THROTTLE(2.0, "safety_filter: [%s] %s", source.name.c_str(), error.c_str());
      return;
    }
    source.last_update = ros::Time::now();
  }

  void countsCallback(const fetch_safety::ZoneCounts::ConstPtr& msg, size_t index) {
    Source& source = sources_[index];
    bool match = msg->zones.size() == zones_.size() && msg->counts.size() == zones_.size();
    for (size_t k = 0; match && k < zones_.size(); ++k) match = msg->zones[k] == zones_[k].name;
    if (!match) {
      ROS_WARN_THROTTLE(2.0, "safety_filter: [%s] zone_counts zones don't match this filter's zones "
                        "(zone_checker running an old config?); ignoring them", source.name.c_str());
      return;
    }
    source.zone_counts.assign(msg->counts.begin(), msg->counts.end());
    source.last_update = ros::Time::now();
  }

  std::vector<DirectionState> computeState() const {
    std::vector<DirectionState> states(NUM_DIRECTIONS);
    const ros::Time now = ros::Time::now();

    for (Direction d : uncovered_) states[d].reasons.push_back("not covered");

    std::vector<int> totals(zones_.size(), 0);
    for (const Source& s : sources_) {
      const bool fresh = !s.last_update.isZero() && (now - s.last_update).toSec() <= source_timeout_;
      if (!fresh) {
        for (Direction d : s.directions) {
          states[d].stop = true;
          states[d].reasons.push_back(s.name + " stale");
        }
        continue;
      }
      for (size_t k = 0; k < zones_.size(); ++k) totals[k] += s.zone_counts[k];
    }

    for (size_t k = 0; k < zones_.size(); ++k) {
      const Zone& z = zones_[k];
      if (totals[k] < z.min_points) continue;
      DirectionState& st = states[z.direction];
      if (z.stop) {
        st.stop = true;
      } else {
        st.scale = std::min(st.scale, z.slowdown_ratio);
      }
      st.reasons.push_back(z.name + " " + std::to_string(totals[k]) + "pts");
    }
    return states;
  }

  geometry_msgs::Twist apply(const Input& input, const std::vector<DirectionState>& states) const {
    geometry_msgs::Twist out = input.cmd;
    auto limit = [&states, &input](double& v, Direction positive, Direction negative) {
      if (v == 0.0) return;
      const Direction d = v > 0.0 ? positive : negative;
      const DirectionState& st = states[d];
      const bool blocked = std::find(input.blocked.begin(), input.blocked.end(), d) != input.blocked.end();
      v = st.stop || blocked ? 0.0 : input.ignore_slowdown ? v : v * st.scale;
    };
    limit(out.linear.x, FORWARD, BACKWARD);
    limit(out.linear.y, LEFT, RIGHT);
    if (states[LEFT].stop || states[RIGHT].stop) out.angular.z *= side_block_yaw_scale_;
    return out;
  }

  void publishStatus(const std::vector<DirectionState>& states) {
    std::ostringstream ss;
    for (int d = 0; d < NUM_DIRECTIONS; ++d) {
      const DirectionState& st = states[d];
      if (d) ss << "  ";
      ss << kDirectionNames[d] << "=";
      if (st.stop) {
        ss << "STOP";
      } else if (st.scale < 1.0) {
        ss << "SLOW(" << st.scale << ")";
      } else {
        ss << "OK";
      }
      if (!st.reasons.empty()) {
        ss << "[";
        for (size_t i = 0; i < st.reasons.size(); ++i) ss << (i ? ", " : "") << st.reasons[i];
        ss << "]";
      }
    }
    const std::string status = ss.str();
    if (status == last_status_) return;
    last_status_ = status;
    std_msgs::String msg;
    msg.data = status;
    status_pub_.publish(msg);
    ROS_INFO("safety_filter: %s", status.c_str());
  }

  bool fresh(const Input& input) const {
    return !input.time.isZero() && (ros::Time::now() - input.time).toSec() <= cmd_timeout_;
  }

  // The highest-priority input with a fresh command, or -1.
  int activeInput() const {
    for (size_t i = 0; i < inputs_.size(); ++i) {
      if (fresh(inputs_[i])) return static_cast<int>(i);
    }
    return -1;
  }

  void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg, size_t index) {
    inputs_[index].cmd = *msg;
    inputs_[index].time = ros::Time::now();
    const int active = activeInput();
    if (active != static_cast<int>(index)) return;  // a higher-priority input is driving
    if (inputs_.size() > 1 && active != last_active_) {
      ROS_INFO("safety_filter: commands from %s", inputs_[active].name.c_str());
      last_active_ = active;
    }
    const auto states = computeState();
    publishStatus(states);
    cmd_pub_.publish(apply(inputs_[active], states));
  }

  void timerCallback(const ros::TimerEvent&) {
    const auto states = computeState();
    publishStatus(states);
    // Keep re-filtering the latest command so a newly seen obstacle takes
    // effect even if the command source publishes slowly. A stale command
    // gets a single zero, then silence, so the bridge goes back to HOLD.
    const int active = activeInput();
    if (active >= 0) {
      cmd_pub_.publish(apply(inputs_[active], states));
      idle_zero_sent_ = false;
    } else if (!idle_zero_sent_) {
      cmd_pub_.publish(geometry_msgs::Twist());
      idle_zero_sent_ = true;
      last_active_ = -1;
    }
  }

  void markerCallback(const ros::TimerEvent&) {
    if (marker_pub_.getNumSubscribers() == 0) return;
    const auto states = computeState();
    std::vector<int> totals(zones_.size(), 0);
    for (const Source& s : sources_) {
      for (size_t k = 0; k < zones_.size(); ++k) totals[k] += s.zone_counts[k];
    }

    visualization_msgs::MarkerArray array;
    for (size_t k = 0; k < zones_.size(); ++k) {
      const Zone& z = zones_[k];
      visualization_msgs::Marker m;
      m.header.frame_id = base_frame_;
      m.header.stamp = ros::Time::now();
      m.ns = "safety_zones";
      m.id = static_cast<int>(k);
      m.type = visualization_msgs::Marker::LINE_STRIP;
      m.action = visualization_msgs::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.scale.x = z.stop ? 0.03 : 0.015;
      m.color.a = 1.0;
      const bool active = totals[k] >= z.min_points || states[z.direction].stop;
      if (!active) {
        m.color.g = 1.0;
      } else if (z.stop) {
        m.color.r = 1.0;
      } else {
        m.color.r = 1.0;
        m.color.g = 0.6;
      }
      for (size_t p = 0; p <= z.polygon.size(); ++p) {
        geometry_msgs::Point pt;
        pt.x = z.polygon[p % z.polygon.size()].x();
        pt.y = z.polygon[p % z.polygon.size()].y();
        pt.z = min_z_;
        m.points.push_back(pt);
      }
      array.markers.push_back(m);
    }
    marker_pub_.publish(array);
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string base_frame_;
  double rate_, cmd_timeout_, source_timeout_, min_z_, max_z_, side_block_yaw_scale_;
  int point_stride_;

  std::vector<Zone> zones_;
  std::vector<Source> sources_;

  std::vector<Input> inputs_;
  std::vector<std::string> topics_;  // per input
  std::vector<Direction> uncovered_;
  int last_active_ = -1;
  ros::Publisher cmd_pub_, status_pub_, marker_pub_;
  ros::Timer timer_, marker_timer_;

  bool idle_zero_sent_ = true;  // nothing to stop before the first command
  std::string last_status_;
};

}  // namespace fetch_safety

int main(int argc, char** argv) {
  ros::init(argc, argv, "safety_filter");
  ros::NodeHandle nh, pnh("~");
  try {
    fetch_safety::SafetyFilter filter(nh, pnh);
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL("safety_filter: %s", e.what());
    return 1;
  }
  return 0;
}
