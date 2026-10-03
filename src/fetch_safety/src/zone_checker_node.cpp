// Counts one camera's cloud into the safety zones on the camera's own board
// and publishes only the counts (fetch_safety/ZoneCounts), so the cloud
// itself never has to cross the network to the safety filter.
//
//   points (sensor_msgs/PointCloud2)  ->  zone_counts (fetch_safety/ZoneCounts)
//
// Loads the same config as the filter (zones, min_z, max_z, base_frame,
// point_stride). One count message per cloud; if the camera or this node
// stops, the counts stop and the filter blocks that camera's directions.
// The cloud stays published as before, for Foxglove/RViz to subscribe when
// someone is watching.

#include <ros/ros.h>
#include <fetch_safety/ZoneCounts.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include "fetch_safety/zones.h"

namespace fetch_safety {

class ZoneChecker {
 public:
  ZoneChecker(ros::NodeHandle& nh, ros::NodeHandle& pnh) : tf_listener_(tf_buffer_) {
    pnh.param<std::string>("base_frame", base_frame_, "base_link");
    pnh.param("min_z", min_z_, -0.35);
    pnh.param("max_z", max_z_, 0.35);
    pnh.param("point_stride", point_stride_, 1);
    zones_ = loadZones(pnh);
    for (const Zone& z : zones_) names_.push_back(z.name);

    pub_ = nh.advertise<fetch_safety::ZoneCounts>("zone_counts", 1);
    sub_ = nh.subscribe("points", 1, &ZoneChecker::cloudCallback, this);
    ROS_INFO("zone_checker: %s -> %s, %zu zones, base_frame=%s, z band [%.2f, %.2f]",
             sub_.getTopic().c_str(), pub_.getTopic().c_str(), zones_.size(), base_frame_.c_str(), min_z_, max_z_);
  }

 private:
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    geometry_msgs::TransformStamped tf_msg;
    try {
      // Cameras are rigidly mounted, so the latest transform is good enough.
      tf_msg = tf_buffer_.lookupTransform(base_frame_, msg->header.frame_id, ros::Time(0));
    } catch (const tf2::TransformException& e) {
      ROS_WARN_THROTTLE(2.0, "zone_checker: no transform %s -> %s: %s", msg->header.frame_id.c_str(),
                        base_frame_.c_str(), e.what());
      return;
    }
    const auto tf = tf2::transformToEigen(tf_msg);

    fetch_safety::ZoneCounts out;
    std::string error;
    if (!countZones(*msg, tf.linear().cast<float>(), tf.translation().cast<float>(), zones_, min_z_, max_z_,
                    point_stride_, out.counts, error)) {
      ROS_WARN_THROTTLE(2.0, "zone_checker: %s", error.c_str());
      return;
    }
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = base_frame_;
    out.zones = names_;
    pub_.publish(out);
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string base_frame_;
  double min_z_, max_z_;
  int point_stride_;
  std::vector<Zone> zones_;
  std::vector<std::string> names_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
};

}  // namespace fetch_safety

int main(int argc, char** argv) {
  ros::init(argc, argv, "zone_checker");
  ros::NodeHandle nh, pnh("~");
  try {
    fetch_safety::ZoneChecker checker(nh, pnh);
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL("zone_checker: %s", e.what());
    return 1;
  }
  return 0;
}
