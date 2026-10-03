// Merges the per-camera obstacle scans from the zone_checkers into one
// LaserScan for mapping and move_base:
//
//   ~scans: [/cam_left/scan, ...]  ->  scan (sensor_msgs/LaserScan)
//
// All inputs are in the base frame with the same bins, so per bin: the
// nearest obstacle any fresh camera reports, else +inf if some fresh camera
// saw that bin clear, else NaN (no camera looked). A camera whose scan is
// older than ~timeout is left out, so its bins turn unknown, not clear.
// Runs on any board; publishes at ~rate.

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/bind.hpp>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace {

struct Input {
  std::string topic;
  ros::Subscriber sub;
  sensor_msgs::LaserScan::ConstPtr last;
  ros::Time received;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "scan_merger");
  ros::NodeHandle nh, pnh("~");
  std::vector<std::string> topics;
  double rate, timeout;
  pnh.param("scans", topics, topics);
  pnh.param("rate", rate, 15.0);
  pnh.param("timeout", timeout, 0.3);
  if (topics.empty()) {
    ROS_FATAL("scan_merger: ~scans is empty");
    return 1;
  }

  std::vector<Input> inputs(topics.size());
  for (size_t i = 0; i < topics.size(); ++i) {
    inputs[i].topic = topics[i];
    inputs[i].sub = nh.subscribe<sensor_msgs::LaserScan>(
        topics[i], 1, boost::function<void(const sensor_msgs::LaserScan::ConstPtr&)>(
                          [&inputs, i](const sensor_msgs::LaserScan::ConstPtr& msg) {
                            inputs[i].last = msg;
                            inputs[i].received = ros::Time::now();
                          }));
  }
  ros::Publisher pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1);
  ROS_INFO("scan_merger: %zu scans -> %s at %.0f Hz", topics.size(), pub.getTopic().c_str(), rate);

  ros::Rate loop(rate);
  while (ros::ok()) {
    ros::spinOnce();
    sensor_msgs::LaserScan out;
    bool have = false;
    for (const Input& in : inputs) {
      if (!in.last || (ros::Time::now() - in.received).toSec() > timeout) continue;
      const sensor_msgs::LaserScan& s = *in.last;
      if (!have) {
        out = s;
        out.ranges.assign(s.ranges.size(), std::numeric_limits<float>::quiet_NaN());
        have = true;
      }
      if (s.ranges.size() != out.ranges.size() || s.header.frame_id != out.header.frame_id) {
        ROS_WARN_THROTTLE(5.0, "scan_merger: %s doesn't match the other scans (bins/frame), skipped", in.topic.c_str());
        continue;
      }
      if (s.header.stamp > out.header.stamp) out.header.stamp = s.header.stamp;
      for (size_t b = 0; b < s.ranges.size(); ++b) {
        const float r = s.ranges[b], o = out.ranges[b];
        if (std::isnan(r)) continue;                  // this camera didn't look
        if (std::isnan(o) || r < o) out.ranges[b] = r;  // nearest wins; +inf only if nothing nearer
      }
    }
    if (have) pub.publish(out);
    loop.sleep();
  }
  return 0;
}
