#!/usr/bin/env python
"""Accepts "close enough" when move_base can't finish a goal.

Near the goal TEB keeps making small corrections to get within
xy/yaw_goal_tolerance, and the B1 steps in place for each one, sometimes for
a minute or more (2026-10-08). Once the robot is within xy_tolerance and
yaw_tolerance of the active goal and its error hasn't improved by
min_progress_xy / min_progress_yaw for settle_time s (or it has been near for
max_near_time s), the goal is cancelled: move_base stops the robot where it
is. place.py reports that as arrived. Any goal source is covered (RViz,
Foxglove, goal.py, place.py): it watches move_base's action topics.
"""
import math

import rospy
import tf2_ros
from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray
from move_base_msgs.msg import MoveBaseActionGoal


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Settler(object):
    def __init__(self):
        self.xy_tol = rospy.get_param('~xy_tolerance', 0.4)
        self.yaw_tol = math.radians(rospy.get_param('~yaw_tolerance_deg', 25.0))
        self.settle_time = rospy.get_param('~settle_time', 5.0)
        self.max_near = rospy.get_param('~max_near_time', 20.0)
        self.min_xy = rospy.get_param('~min_progress_xy', 0.03)
        self.min_yaw = math.radians(rospy.get_param('~min_progress_yaw_deg', 3.0))
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf)
        self.goal = None      # MoveBaseActionGoal
        self.active = False
        self.reset()
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
        rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.on_goal)
        rospy.Subscriber('/move_base/status', GoalStatusArray, self.on_status)
        rospy.Timer(rospy.Duration(0.5), self.tick)

    def reset(self):
        self.near_since = None
        self.best_xy = self.best_yaw = None
        self.last_progress = None

    def on_goal(self, msg):
        self.goal = msg
        self.active = False
        self.reset()

    def on_status(self, msg):
        if self.goal is None:
            return
        for s in msg.status_list:
            if s.goal_id.id == self.goal.goal_id.id:
                self.active = s.status == GoalStatus.ACTIVE
                return

    def error(self):
        p = self.goal.goal.target_pose
        t = self.buf.lookup_transform(p.header.frame_id or 'map', 'base_link', rospy.Time(0)).transform
        dxy = math.hypot(p.pose.position.x - t.translation.x, p.pose.position.y - t.translation.y)
        dyaw = yaw_of(p.pose.orientation) - yaw_of(t.rotation)
        return dxy, abs(math.atan2(math.sin(dyaw), math.cos(dyaw)))

    def tick(self, _):
        if self.goal is None or not self.active:
            return
        try:
            dxy, dyaw = self.error()
        except tf2_ros.TransformException:
            return
        now = rospy.Time.now()
        if dxy > self.xy_tol or dyaw > self.yaw_tol:
            self.reset()
            return
        if self.near_since is None:
            self.near_since = self.last_progress = now
            self.best_xy, self.best_yaw = dxy, dyaw
            return
        if dxy < self.best_xy - self.min_xy or dyaw < self.best_yaw - self.min_yaw:
            self.last_progress = now
        self.best_xy, self.best_yaw = min(self.best_xy, dxy), min(self.best_yaw, dyaw)
        stalled = (now - self.last_progress).to_sec() >= self.settle_time
        too_long = (now - self.near_since).to_sec() >= self.max_near
        if stalled or too_long:
            rospy.loginfo('goal_settler: accepting the goal %.2f m / %.0f deg off (%s)', dxy, math.degrees(dyaw),
                          'no progress for %.0f s' % self.settle_time if stalled else 'near for %.0f s' % self.max_near)
            self.cancel_pub.publish(self.goal.goal_id)
            self.active = False
            self.reset()


if __name__ == '__main__':
    rospy.init_node('goal_settler')
    Settler()
    rospy.spin()
