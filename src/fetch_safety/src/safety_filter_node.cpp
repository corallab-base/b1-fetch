// Velocity safety filter (ROS1 take on Nav2's collision_monitor).
//
// Subscribes to raw velocity commands and to point clouds from one or more
// depth cameras. Points are transformed into the robot base frame, cropped to
// a height band, and counted inside per-direction polygon zones. A zone with
// enough points either stops or slows the velocity component heading toward
// it. A source whose data goes stale blocks every direction it covers, so a
// dead or disconnected camera fails safe.

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Geometry>
#include <boost/bind.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fetch_safety {

enum Direction { FORWARD = 0, BACKWARD, LEFT, RIGHT, NUM_DIRECTIONS };
const char* const kDirectionNames[NUM_DIRECTIONS] = {"forward", "backward", "left", "right"};

Direction parseDirection(const std::string& s) {
  for (int d = 0; d < NUM_DIRECTIONS; ++d) {
    if (s == kDirectionNames[d]) return static_cast<Direction>(d);
  }
  throw std::runtime_error("unknown direction '" + s + "' (expected forward|backward|left|right)");
}

double toDouble(XmlRpc::XmlRpcValue& v) {
  if (v.getType() == XmlRpc::XmlRpcValue::TypeInt) return static_cast<int>(v);
  if (v.getType() == XmlRpc::XmlRpcValue::TypeDouble) return static_cast<double>(v);
  throw std::runtime_error("expected a number");
}

struct Zone {
  std::string name;
  Direction direction;
  bool stop;              // false = slowdown
  double slowdown_ratio;  // velocity multiplier while a slowdown zone is active
  int min_points;
  std::vector<Eigen::Vector2f> polygon;
  float min_x, max_x, min_y, max_y;  // bounding box for quick rejection

  bool contains(float x, float y) const {
    if (x < min_x || x > max_x || y < min_y || y > max_y) return false;
    bool inside = false;
    for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
      const Eigen::Vector2f& a = polygon[i];
      const Eigen::Vector2f& b = polygon[j];
      if ((a.y() > y) != (b.y() > y) &&
          x < (b.x() - a.x()) * (y - a.y()) / (b.y() - a.y()) + a.x()) {
        inside = !inside;
      }
    }
    return inside;
  }
};

struct Source {
  std::string name;
  std::string topic;
  std::vector<Direction> directions;
  ros::Subscriber sub;
  ros::Time last_update;        // receive time of last processed cloud; zero = never
  std::vector<int> zone_counts;  // points per zone in the last cloud
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

    loadZones(pnh);
    loadSources(pnh);

    for (size_t i = 0; i < sources_.size(); ++i) {
      sources_[i].zone_counts.assign(zones_.size(), 0);
      sources_[i].sub = nh.subscribe<sensor_msgs::PointCloud2>(
          sources_[i].topic, 1, boost::bind(&SafetyFilter::cloudCallback, this, _1, i));
    }

    cmd_sub_ = nh.subscribe("cmd_vel_raw", 1, &SafetyFilter::cmdCallback, this);
    cmd_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    status_pub_ = pnh.advertise<std_msgs::String>("status", 1, true);
    marker_pub_ = pnh.advertise<visualization_msgs::MarkerArray>("zones", 1);

    timer_ = nh.createTimer(ros::Duration(1.0 / rate_), &SafetyFilter::timerCallback, this);
    marker_timer_ = nh.createTimer(ros::Duration(0.2), &SafetyFilter::markerCallback, this);

    ROS_INFO("safety_filter: %zu zones, %zu sources, base_frame=%s, z band [%.2f, %.2f]",
             zones_.size(), sources_.size(), base_frame_.c_str(), min_z_, max_z_);
  }

 private:
  void loadZones(ros::NodeHandle& pnh) {
    XmlRpc::XmlRpcValue list;
    if (!pnh.getParam("zones", list) || list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      throw std::runtime_error("~zones must be a list");
    }
    for (int i = 0; i < list.size(); ++i) {
      XmlRpc::XmlRpcValue& z = list[i];
      Zone zone;
      zone.name = static_cast<std::string>(z["name"]);
      zone.direction = parseDirection(static_cast<std::string>(z["direction"]));
      const std::string action = static_cast<std::string>(z["action"]);
      if (action != "stop" && action != "slowdown") {
        throw std::runtime_error("zone " + zone.name + ": action must be stop or slowdown");
      }
      zone.stop = action == "stop";
      zone.slowdown_ratio = z.hasMember("slowdown_ratio") ? toDouble(z["slowdown_ratio"]) : 0.3;
      zone.min_points = z.hasMember("min_points") ? static_cast<int>(z["min_points"]) : 20;

      XmlRpc::XmlRpcValue& poly = z["polygon"];
      if (poly.getType() != XmlRpc::XmlRpcValue::TypeArray || poly.size() < 3) {
        throw std::runtime_error("zone " + zone.name + ": polygon needs at least 3 [x, y] points");
      }
      for (int p = 0; p < poly.size(); ++p) {
        zone.polygon.emplace_back(toDouble(poly[p][0]), toDouble(poly[p][1]));
      }
      zone.min_x = zone.max_x = zone.polygon[0].x();
      zone.min_y = zone.max_y = zone.polygon[0].y();
      for (const auto& v : zone.polygon) {
        zone.min_x = std::min(zone.min_x, v.x());
        zone.max_x = std::max(zone.max_x, v.x());
        zone.min_y = std::min(zone.min_y, v.y());
        zone.max_y = std::max(zone.max_y, v.y());
      }
      zones_.push_back(zone);
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
    const Eigen::Matrix3f R = tf.linear().cast<float>();
    const Eigen::Vector3f t = tf.translation().cast<float>();

    int ox = -1, oy = -1, oz = -1;
    for (const auto& f : msg->fields) {
      if (f.datatype != sensor_msgs::PointField::FLOAT32) continue;
      if (f.name == "x") ox = f.offset;
      if (f.name == "y") oy = f.offset;
      if (f.name == "z") oz = f.offset;
    }
    if (ox < 0 || oy < 0 || oz < 0) {
      ROS_WARN_THROTTLE(2.0, "safety_filter: [%s] cloud has no float32 x/y/z", source.name.c_str());
      return;
    }
    if (msg->height > 0 && msg->width > 0 &&
        msg->data.size() < (msg->height - 1) * size_t(msg->row_step) + msg->width * size_t(msg->point_step)) {
      ROS_WARN_THROTTLE(2.0, "safety_filter: [%s] malformed cloud", source.name.c_str());
      return;
    }

    std::vector<int> counts(zones_.size(), 0);
    const size_t n = size_t(msg->width) * msg->height;
    for (size_t i = 0; i < n; i += point_stride_) {
      const uint8_t* p = &msg->data[(i / msg->width) * msg->row_step + (i % msg->width) * msg->point_step];
      float x, y, z;
      std::memcpy(&x, p + ox, sizeof(float));
      std::memcpy(&y, p + oy, sizeof(float));
      std::memcpy(&z, p + oz, sizeof(float));
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

      const Eigen::Vector3f q = R * Eigen::Vector3f(x, y, z) + t;
      if (q.z() < min_z_ || q.z() > max_z_) continue;
      for (size_t k = 0; k < zones_.size(); ++k) {
        if (zones_[k].contains(q.x(), q.y())) ++counts[k];
      }
    }

    source.zone_counts.swap(counts);
    source.last_update = ros::Time::now();
  }

  std::vector<DirectionState> computeState() const {
    std::vector<DirectionState> states(NUM_DIRECTIONS);
    const ros::Time now = ros::Time::now();

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

  geometry_msgs::Twist apply(const geometry_msgs::Twist& in, const std::vector<DirectionState>& states) const {
    geometry_msgs::Twist out = in;
    auto limit = [&states](double& v, Direction positive, Direction negative) {
      if (v == 0.0) return;
      const DirectionState& st = states[v > 0.0 ? positive : negative];
      v = st.stop ? 0.0 : v * st.scale;
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

  void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    last_cmd_ = *msg;
    last_cmd_time_ = ros::Time::now();
    const auto states = computeState();
    publishStatus(states);
    cmd_pub_.publish(apply(last_cmd_, states));
  }

  void timerCallback(const ros::TimerEvent&) {
    const auto states = computeState();
    publishStatus(states);
    // Keep re-filtering the latest command so a newly seen obstacle takes
    // effect even if the command source publishes slowly. A stale command
    // gets a single zero, then silence: the B1 bridge (a2_ros2udp) switches
    // to walk mode on every /cmd_vel message, so streaming zeros would keep
    // the robot stepping in place instead of letting it time out to stand.
    const bool cmd_fresh =
        !last_cmd_time_.isZero() && (ros::Time::now() - last_cmd_time_).toSec() <= cmd_timeout_;
    if (cmd_fresh) {
      cmd_pub_.publish(apply(last_cmd_, states));
      idle_zero_sent_ = false;
    } else if (!idle_zero_sent_) {
      cmd_pub_.publish(geometry_msgs::Twist());
      idle_zero_sent_ = true;
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

  ros::Subscriber cmd_sub_;
  ros::Publisher cmd_pub_, status_pub_, marker_pub_;
  ros::Timer timer_, marker_timer_;

  geometry_msgs::Twist last_cmd_;
  ros::Time last_cmd_time_;
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
