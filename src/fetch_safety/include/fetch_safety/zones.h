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
#include <limits>
#include <unordered_set>
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

// Planar obstacle scan around the base frame origin, built in the same pass
// as the zone counts: per angle bin, the nearest point in the height band.
// Bins where the camera saw anything (floor included) but no obstacle are
// clear (+inf); bins it didn't see at all are unknown (NaN). Points inside
// the body box (the robot itself, e.g. the legs) are ignored.
// With collect_cloud, it also keeps the band's points thinned to one per
// voxel-sized cube (`cloud`): a denser obstacle view for scan matching.
struct ScanBuilder {
  bool collect_cloud = false;
  float voxel = 0.05f;               // m
  std::vector<Eigen::Vector3f> cloud;
  std::unordered_set<uint64_t> voxels;
  int bins = 360;          // over a full turn, starting at -pi
  float range_max = 4.0f;  // m from the base frame origin
  float body_min_x = 0, body_max_x = 0, body_min_y = 0, body_max_y = 0;
  std::vector<float> nearest;  // per bin, +inf = nothing in the band
  std::vector<uint8_t> seen;

  void reset() {
    nearest.assign(bins, std::numeric_limits<float>::infinity());
    seen.assign(bins, 0);
    cloud.clear();
    voxels.clear();
  }
  void add(const Eigen::Vector3f& q, bool in_band) {
    if (q.x() >= body_min_x && q.x() <= body_max_x && q.y() >= body_min_y && q.y() <= body_max_y) return;
    const float r = std::hypot(q.x(), q.y());
    if (r > range_max) return;
    const int b = std::min(bins - 1, std::max(0, int((std::atan2(q.y(), q.x()) + float(M_PI)) / (2 * float(M_PI)) * bins)));
    seen[b] = 1;
    if (!in_band) return;
    nearest[b] = std::min(nearest[b], r);
    if (collect_cloud) {
      // 21 bits per axis: +-52 km at 5 cm, far beyond range_max
      auto cell = [this](float v) { return uint64_t(int64_t(std::floor(v / voxel)) & 0x1FFFFF); };
      if (voxels.insert(cell(q.x()) | cell(q.y()) << 21 | cell(q.z()) << 42).second) cloud.push_back(q);
    }
  }
  // LaserScan ranges: nearest obstacle, +inf where seen and clear, NaN where unseen
  std::vector<float> ranges() const {
    std::vector<float> out(bins);
    for (int b = 0; b < bins; ++b) out[b] = seen[b] ? nearest[b] : std::numeric_limits<float>::quiet_NaN();
    return out;
  }
};

// Points of `cloud` per zone, after transforming into the base frame with
// (R, t) and keeping only z in [min_z, max_z]; also fills `scan` if given.
// Returns false (counts untouched) and sets `error` if the cloud can't be read.
inline bool countZones(const sensor_msgs::PointCloud2& cloud, const Eigen::Matrix3f& R, const Eigen::Vector3f& t,
                       const std::vector<Zone>& zones, double min_z, double max_z, int point_stride,
                       std::vector<int>& counts, std::string& error, ScanBuilder* scan = nullptr) {
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
  if (scan) scan->reset();
  const size_t n = size_t(cloud.width) * cloud.height;
  for (size_t i = 0; i < n; i += std::max(1, point_stride)) {
    const uint8_t* p = &cloud.data[(i / cloud.width) * cloud.row_step + (i % cloud.width) * cloud.point_step];
    float x, y, z;
    std::memcpy(&x, p + ox, sizeof(float));
    std::memcpy(&y, p + oy, sizeof(float));
    std::memcpy(&z, p + oz, sizeof(float));
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

    const Eigen::Vector3f q = R * Eigen::Vector3f(x, y, z) + t;
    const bool in_band = q.z() >= min_z && q.z() <= max_z;
    if (scan) scan->add(q, in_band);
    if (!in_band) continue;
    for (size_t k = 0; k < zones.size(); ++k) {
      if (zones[k].contains(q.x(), q.y())) ++result[k];
    }
  }
  counts.swap(result);
  return true;
}

}  // namespace fetch_safety
