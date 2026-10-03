// Zone counting shared by safety_filter_node and zone_checker_node, so the
// counts a camera board sends are exactly what the filter would compute from
// the cloud itself.
#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace fetch_safety {

enum Direction { FORWARD = 0, BACKWARD, LEFT, RIGHT, NUM_DIRECTIONS };
const char* const kDirectionNames[NUM_DIRECTIONS] = {"forward", "backward", "left", "right"};

inline Direction parseDirection(const std::string& s) {
  for (int d = 0; d < NUM_DIRECTIONS; ++d) {
    if (s == kDirectionNames[d]) return static_cast<Direction>(d);
  }
  throw std::runtime_error("unknown direction '" + s + "' (expected forward|backward|left|right)");
}

inline double toDouble(XmlRpc::XmlRpcValue& v) {
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

inline std::vector<Zone> loadZones(ros::NodeHandle& pnh) {
  XmlRpc::XmlRpcValue list;
  if (!pnh.getParam("zones", list) || list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    throw std::runtime_error("~zones must be a list");
  }
  std::vector<Zone> zones;
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
    zones.push_back(zone);
  }
  return zones;
}

// Points of `cloud` per zone, after transforming into the base frame with
// (R, t) and keeping only z in [min_z, max_z]. Returns false (counts
// untouched) and sets `error` if the cloud can't be read.
inline bool countZones(const sensor_msgs::PointCloud2& cloud, const Eigen::Matrix3f& R, const Eigen::Vector3f& t,
                       const std::vector<Zone>& zones, double min_z, double max_z, int point_stride,
                       std::vector<int>& counts, std::string& error) {
  int ox = -1, oy = -1, oz = -1;
  for (const auto& f : cloud.fields) {
    if (f.datatype != sensor_msgs::PointField::FLOAT32) continue;
    if (f.name == "x") ox = f.offset;
    if (f.name == "y") oy = f.offset;
    if (f.name == "z") oz = f.offset;
  }
  if (ox < 0 || oy < 0 || oz < 0) {
    error = "cloud has no float32 x/y/z";
    return false;
  }
  if (cloud.height > 0 && cloud.width > 0 &&
      cloud.data.size() < (cloud.height - 1) * size_t(cloud.row_step) + cloud.width * size_t(cloud.point_step)) {
    error = "malformed cloud";
    return false;
  }

  std::vector<int> result(zones.size(), 0);
  const size_t n = size_t(cloud.width) * cloud.height;
  for (size_t i = 0; i < n; i += std::max(1, point_stride)) {
    const uint8_t* p = &cloud.data[(i / cloud.width) * cloud.row_step + (i % cloud.width) * cloud.point_step];
    float x, y, z;
    std::memcpy(&x, p + ox, sizeof(float));
    std::memcpy(&y, p + oy, sizeof(float));
    std::memcpy(&z, p + oz, sizeof(float));
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

    const Eigen::Vector3f q = R * Eigen::Vector3f(x, y, z) + t;
    if (q.z() < min_z || q.z() > max_z) continue;
    for (size_t k = 0; k < zones.size(); ++k) {
      if (zones[k].contains(q.x(), q.y())) ++result[k];
    }
  }
  counts.swap(result);
  return true;
}

}  // namespace fetch_safety
