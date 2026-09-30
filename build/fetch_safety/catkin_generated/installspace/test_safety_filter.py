#!/usr/bin/env python2
"""Offline check of safety_filter_node using synthetic clouds (no cameras).

Run with the node already up on the default config:
  roslaunch fetch_safety safety_filter.launch
  rosrun fetch_safety test_safety_filter.py

Clouds are published in base_link directly, so no TF is needed.
"""
from __future__ import print_function

import sys
import threading

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

TOPICS = {
    'left': '/cam_left/depth/color/points',
    'right': '/cam_right/depth/color/points',
    'front': '/cam_front/depth/color/points',
}


class Harness(object):
    def __init__(self):
        self.cloud_pubs = {k: rospy.Publisher(t, PointCloud2, queue_size=1) for k, t in TOPICS.items()}
        self.cmd_pub = rospy.Publisher('/cmd_vel_raw', Twist, queue_size=1)
        self.lock = threading.Lock()
        self.points = {k: [] for k in TOPICS}   # points each fake camera "sees"
        self.live = set()                        # cameras currently publishing
        self.out = None
        self.out_count = 0
        rospy.Subscriber('/cmd_vel', Twist, self._on_out)
        rospy.Timer(rospy.Duration(1.0 / 15), self._publish_clouds)
        self.failures = 0

    def _on_out(self, msg):
        self.out = msg
        self.out_count += 1

    def _publish_clouds(self, _):
        with self.lock:
            snapshot = {k: list(self.points[k]) for k in self.live}
        for k, pts in snapshot.items():
            header = Header(stamp=rospy.Time.now(), frame_id='base_link')
            self.cloud_pubs[k].publish(point_cloud2.create_cloud_xyz32(header, pts))

    def set_scene(self, live, points=None):
        with self.lock:
            self.live = set(live)
            self.points = {k: [] for k in TOPICS}
            for k, pts in (points or {}).items():
                self.points[k] = pts
        rospy.sleep(0.5)  # > source_timeout, lets stale/fresh state settle

    def command(self, vx=0.0, vy=0.0, wz=0.0, publish=True):
        if publish:
            t = Twist()
            t.linear.x, t.linear.y, t.angular.z = vx, vy, wz
            end = rospy.Time.now() + rospy.Duration(0.3)
            while rospy.Time.now() < end:
                self.cmd_pub.publish(t)
                rospy.sleep(0.05)
        else:
            rospy.sleep(0.8)  # > cmd_timeout
        o = self.out
        return (round(o.linear.x, 3), round(o.linear.y, 3), round(o.angular.z, 3))

    def check(self, name, got, expected):
        ok = all(abs(a - b) < 1e-3 for a, b in zip(got, expected))
        print('%s %-55s got %s expected %s' % ('PASS' if ok else 'FAIL', name, got, expected))
        if not ok:
            self.failures += 1


def blob(x, y, z=0.0, n=60):
    """n points clustered around (x, y, z)."""
    return [(x + 0.01 * (i % 6), y + 0.01 * (i // 6 % 6), z) for i in range(n)]


def main():
    rospy.init_node('test_safety_filter')
    h = Harness()
    rospy.sleep(1.0)
    everyone = ['left', 'right', 'front']

    h.set_scene([])
    h.check('no cameras: forward/left/right blocked, backward ok',
            h.command(0.5, 0.3, 0.2), (0.0, 0.0, 0.2))
    h.check('no cameras: backward + right',
            h.command(-0.4, -0.3), (-0.4, 0.0, 0.0))

    h.set_scene(everyone)
    h.check('all cameras clear: passes through', h.command(0.5, 0.3, 0.2), (0.5, 0.3, 0.2))

    h.set_scene(['left', 'right'])
    h.check('front camera missing: forward blocked only', h.command(0.5, 0.3), (0.0, 0.3, 0.0))

    h.set_scene(everyone, {'front': blob(0.8, 0.0)})
    h.check('obstacle in front_stop: vx zeroed, vy kept', h.command(0.5, -0.3), (0.0, -0.3, 0.0))
    h.check('obstacle in front_stop: reversing allowed', h.command(-0.5, 0.0), (-0.5, 0.0, 0.0))

    h.set_scene(everyone, {'front': blob(1.3, 0.0)})
    h.check('obstacle in front_slow: vx scaled 0.3', h.command(0.5, 0.0), (0.15, 0.0, 0.0))

    h.set_scene(everyone, {'front': blob(0.8, 0.0, n=10)})
    h.check('below min_points: ignored as noise', h.command(0.5, 0.0), (0.5, 0.0, 0.0))

    h.set_scene(everyone, {'front': blob(0.8, 0.0, z=-0.55)})
    h.check('ground points below min_z: ignored', h.command(0.5, 0.0), (0.5, 0.0, 0.0))

    h.set_scene(everyone, {'left': blob(0.0, 0.45)})
    h.check('obstacle in left_stop: vy>0 zeroed', h.command(0.3, 0.3), (0.3, 0.0, 0.0))
    h.check('obstacle in left_stop: moving right allowed', h.command(0.0, -0.3), (0.0, -0.3, 0.0))

    h.set_scene(everyone, {'right': blob(0.0, -0.8)})
    h.check('obstacle in right_slow: vy scaled 0.3', h.command(0.0, -0.5), (0.0, -0.15, 0.0))

    h.set_scene(everyone, {'left': blob(0.0, 0.1)})
    h.check('points inside footprint (no zone): ignored', h.command(0.5, 0.3), (0.5, 0.3, 0.0))

    h.set_scene(everyone)
    h.command(0.5, 0.3)
    h.check('command source stops: output goes to zero', h.command(publish=False), (0.0, 0.0, 0.0))
    before = h.out_count
    rospy.sleep(1.0)
    h.check('then goes quiet (bridge times out to stand)', (h.out_count - before,), (0,))

    print('\n%d failure(s)' % h.failures)
    sys.exit(1 if h.failures else 0)


if __name__ == '__main__':
    main()
