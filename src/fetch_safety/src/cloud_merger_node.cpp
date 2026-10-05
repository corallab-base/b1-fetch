// Merges the zone_checkers' thinned obstacle clouds into one cloud for
// RTAB-Map's scan matching:
//
//   ~clouds: [/cam_left/obstacles, ...]  ->  obstacles (sensor_msgs/PointCloud2)
//
// All inputs are xyz float32 clouds in the base frame, so the merge is a
// concatenation of the latest cloud from every camera that is fresh (newer
// than ~timeout); the stamp is the newest input's. Runs on any board;
// publishes at ~rate.

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/function.hpp>

#include <string>
#include <vector>

namespace {

struct Input {
  std::string topic;
  ros::Subscriber sub;
  sensor_msgs::PointCloud2::ConstPtr last;
  ros::Time received;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "cloud_merger");
  ros::NodeHandle nh, pnh("~");
  std::vector<std::string> topics;
  double rate, timeout;
  pnh.param("clouds", topics, topics);
  pnh.param("rate", rate, 5.0);
  pnh.param("timeout", timeout, 0.5);
  if (topics.empty()) {
    ROS_FATAL("cloud_merger: ~clouds is empty");
    return 1;
  }

  std::vector<Input> inputs(topics.size());
  for (size_t i = 0; i < topics.size(); ++i) {
    inputs[i].topic = topics[i];
    inputs[i].sub = nh.subscribe<sensor_msgs::PointCloud2>(
        topics[i], 1, boost::function<void(const sensor_msgs::PointCloud2::ConstPtr&)>(
                          [&inputs, i](const sensor_msgs::PointCloud2::ConstPtr& msg) {
                            inputs[i].last = msg;
                            inputs[i].received = ros::Time::now();
                          }));
  }
  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("obstacles", 1);
  ROS_INFO("cloud_merger: %zu clouds -> %s at %.0f Hz", topics.size(), pub.getTopic().c_str(), rate);

  ros::Rate loop(rate);
  while (ros::ok()) {
    ros::spinOnce();
    sensor_msgs::PointCloud2 out;
    bool have = false;
    for (const Input& in : inputs) {
      if (!in.last || (ros::Time::now() - in.received).toSec() > timeout) continue;
      const sensor_msgs::PointCloud2& c = *in.last;
      if (!have) {
        out = c;
        out.data.clear();
        out.width = 0;
        have = true;
      } else if (c.point_step != out.point_step || c.fields.size() != out.fields.size() ||
                 c.header.frame_id != out.header.frame_id) {
        ROS_WARN_THROTTLE(5.0, "cloud_merger: %s doesn't match the other clouds (fields/frame), skipped", in.topic.c_str());
        continue;
      }
      if (c.header.stamp > out.header.stamp) out.header.stamp = c.header.stamp;
      out.data.insert(out.data.end(), c.data.begin(), c.data.begin() + size_t(c.width) * c.height * c.point_step);
      out.width += c.width * c.height;
    }
    if (have) {
      out.height = 1;
      out.row_step = out.width * out.point_step;
      out.is_dense = true;
      pub.publish(out);
    }
    loop.sleep();
  }
  return 0;
}
