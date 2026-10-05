#!/usr/bin/env python
"""Sends a move_base goal relative to where the robot is now:

  rosrun fetch_nav goal.py 1.0            # 1 m straight ahead
  rosrun fetch_nav goal.py 1.0 0.5 90     # 1 m ahead, 0.5 m left, then face 90 deg left
  rosrun fetch_nav goal.py --cancel       # cancel the current goal

dx, dy in metres in the robot's frame (x forward, y left), dyaw in degrees
(counter-clockwise). The goal is converted to the map frame and published
on /move_base_simple/goal.
"""
import math
import sys

import rospy
import tf2_ros
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseStamped


def main():
    rospy.init_node('goal', anonymous=True)
    args = rospy.myargv()[1:]
    if args and args[0] == '--cancel':
        pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1, latch=True)
        pub.publish(GoalID())
        rospy.sleep(0.5)
        print('cancelled')
        return
    if not args:
        print(__doc__)
        sys.exit(1)
    dx = float(args[0])
    dy = float(args[1]) if len(args) > 1 else 0.0
    dyaw = math.radians(float(args[2])) if len(args) > 2 else 0.0

    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    t = buf.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(3.0)).transform
    q = t.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    gx = t.translation.x + dx * math.cos(yaw) - dy * math.sin(yaw)
    gy = t.translation.y + dx * math.sin(yaw) + dy * math.cos(yaw)
    gyaw = yaw + dyaw

    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x, goal.pose.position.y = gx, gy
    goal.pose.orientation.z, goal.pose.orientation.w = math.sin(gyaw / 2), math.cos(gyaw / 2)
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1, latch=True)
    rospy.sleep(0.5)   # let move_base connect
    pub.publish(goal)
    rospy.sleep(0.5)
    print('goal sent: map (%.2f, %.2f), facing %.0f deg (robot at (%.2f, %.2f), %.0f deg)' % (
        gx, gy, math.degrees(gyaw), t.translation.x, t.translation.y, math.degrees(yaw)))


if __name__ == '__main__':
    main()
