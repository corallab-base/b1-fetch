#!/usr/bin/env python
"""Offline check of safety_filter_node + zone_checker_node using synthetic
clouds (no cameras). Run everything with:
  src/fetch_safety/test/run_tests.sh
(test_safety.launch: filter without front_down, one zone_checker per fake
camera). Clouds are published in base_link directly, so no TF is needed.
"""
from __future__ import print_function

import sys
import threading

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs import point_cloud2
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Header, String
from fetch_safety.msg import ZoneCounts

TOPICS = {
    'left': '/cam_left/depth/color/points',
    'right': '/cam_right/depth/color/points',
    'front': '/cam_front/depth/color/points',
    'rear': '/cam_rear/depth/color/points',
}


class Harness(object):
    def __init__(self):
        self.cloud_pubs = {k: rospy.Publisher(t, PointCloud2, queue_size=1) for k, t in TOPICS.items()}
        self.cmd_pub = rospy.Publisher('/cmd_vel_raw', Twist, queue_size=1)
        self.nav_pub = rospy.Publisher('/cmd_vel_nav', Twist, queue_size=1)
        self.status = ''
        self.scan = None
        rospy.Subscriber('/scan', LaserScan, self._on_scan)
        rospy.Subscriber('/safety_filter/status', String, self._on_status)
        self.lock = threading.Lock()
        self.points = {k: [] for k in TOPICS}   # points each fake camera "sees"
        self.live = set()                        # cameras currently publishing
        self.out = None
        self.out_count = 0
        rospy.Subscriber('/cmd_vel', Twist, self._on_out)
        rospy.Timer(rospy.Duration(1.0 / 15), self._publish_clouds)
        self.failures = 0

    def _on_scan(self, msg):
        self.scan = msg

    def scan_at(self, deg):
        """Merged /scan range at a bearing (degrees, 0 = forward, 90 = left)."""
        import math
        s = self.scan
        # bin whose span [centre - inc/2, centre + inc/2) contains the bearing
        b = int(math.floor((math.radians(deg) - s.angle_min) / s.angle_increment + 0.5)) % len(s.ranges)
        return s.ranges[b]

    def _on_status(self, msg):
        self.status = msg.data

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

    def command(self, vx=0.0, vy=0.0, wz=0.0, publish=True, nav=None):
        """Manual command (vx, vy, wz) and/or nav command (nav=(vx, vy, wz))."""
        if publish:
            t = Twist()
            t.linear.x, t.linear.y, t.angular.z = vx, vy, wz
            n = Twist()
            if nav:
                n.linear.x, n.linear.y, n.angular.z = nav
            manual = nav is None or (vx, vy, wz) != (0.0, 0.0, 0.0)
            end = rospy.Time.now() + rospy.Duration(0.3)
            while rospy.Time.now() < end:
                if manual:
                    self.cmd_pub.publish(t)
                if nav:
                    self.nav_pub.publish(n)
                rospy.sleep(0.05)
        else:
            rospy.sleep(0.8)  # > cmd_timeout
        o = self.out
        return (round(o.linear.x, 3), round(o.linear.y, 3), round(o.angular.z, 3))

    def check_scan(self, name, deg, lo, hi=None):
        """/scan at a bearing is in [lo, hi], or 'nan' / 'inf'. Blobs start
        exactly at 0/90 deg, a bin boundary, so bearings are picked inside them."""
        rospy.sleep(0.3)
        r = self.scan_at(deg)
        if lo == 'nan':
            ok = r != r
        elif lo == 'inf':
            ok = r == float('inf')
        else:
            ok = lo <= r <= hi
        print('%s %-55s got %s expected %s' % ('PASS' if ok else 'FAIL', name, r, lo if hi is None else '%s..%s' % (lo, hi)))
        if not ok:
            self.failures += 1
            s = self.scan
            print('     scan stamp %s, non-NaN bins: %s' % (s and s.header.stamp.to_sec(),
                  s and [(i, round(x, 2)) for i, x in enumerate(s.ranges) if x == x][:6]))

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
    everyone = ['left', 'right', 'front', 'rear']

    h.set_scene([])
    h.check('no cameras: every direction blocked, turning kept',
            h.command(0.5, 0.3, 0.2), (0.0, 0.0, 0.2))
    h.check('no cameras: backward + right blocked',
            h.command(-0.4, -0.3), (0.0, 0.0, 0.0))

    h.set_scene(everyone)
    h.check('all cameras clear: passes through', h.command(0.5, 0.3, 0.2), (0.5, 0.3, 0.2))
    h.check('status marks backward not covered', ('backward=OK[not covered]' in h.status,), (True,))
    h.check('manual backward allowed (no rear coverage)', h.command(-0.4, 0.0), (-0.4, 0.0, 0.0))
    rospy.sleep(0.8)
    h.check('nav forward/sideways/turn pass', h.command(nav=(0.5, 0.3, 0.2)), (0.5, 0.3, 0.2))
    h.check('nav backward blocked, rest kept', h.command(nav=(-0.4, 0.3, 0.2)), (0.0, 0.3, 0.2))
    h.check('manual overrides nav', h.command(0.0, -0.2, 0.0, nav=(0.5, 0.0, 0.0)), (0.0, -0.2, 0.0))
    rospy.sleep(0.8)

    h.set_scene(['left', 'right', 'rear'])
    h.check('front camera missing: forward blocked only', h.command(0.5, 0.3), (0.0, 0.3, 0.0))
    h.check('front camera missing: backward ok', h.command(-0.4, 0.0), (-0.4, 0.0, 0.0))

    h.set_scene(['left', 'right', 'front'])
    h.check('rear camera missing: backward blocked only', h.command(-0.4, 0.3), (0.0, 0.3, 0.0))

    h.set_scene(everyone, {'front': blob(0.8, 0.0)})
    h.check('obstacle in front_stop: vx zeroed, vy kept', h.command(0.5, -0.3), (0.0, -0.3, 0.0))
    h.check('obstacle in front_stop: reversing allowed', h.command(-0.5, 0.0), (-0.5, 0.0, 0.0))
    h.check_scan('/scan: front obstacle at ~0.8 m straight ahead', 1, 0.79, 0.86)
    h.check_scan('/scan: nothing seen behind -> unknown (NaN)', 180, 'nan')

    h.set_scene(everyone, {'rear': blob(-0.8, 0.0)})
    h.check('obstacle in rear_stop: vx<0 zeroed', h.command(-0.4, 0.0), (0.0, 0.0, 0.0))
    h.check('obstacle in rear_stop: forward allowed', h.command(0.4, 0.0), (0.4, 0.0, 0.0))

    h.set_scene(everyone, {'front': blob(1.3, 0.0)})
    h.check('obstacle in front_slow: vx scaled 0.3', h.command(0.5, 0.0), (0.15, 0.0, 0.0))

    h.set_scene(everyone, {'front': blob(0.8, 0.0, n=10)})
    h.check('below min_points: ignored as noise', h.command(0.5, 0.0), (0.5, 0.0, 0.0))

    h.set_scene(everyone, {'front': blob(0.8, 0.0, z=-0.55)})
    h.check('ground points below min_z: ignored', h.command(0.5, 0.0), (0.5, 0.0, 0.0))
    h.check_scan('/scan: floor only ahead -> clear (+inf)', 1, 'inf')

    h.set_scene(everyone, {'left': blob(0.0, 0.45)})
    h.check('obstacle in left_stop: vy>0 zeroed', h.command(0.3, 0.3), (0.3, 0.0, 0.0))
    h.check('obstacle in left_stop: moving right allowed', h.command(0.0, -0.3), (0.0, -0.3, 0.0))

    h.set_scene(everyone, {'right': blob(0.0, -0.8)})
    h.check('obstacle in right_slow: vy scaled 0.3', h.command(0.0, -0.5), (0.0, -0.15, 0.0))

    h.set_scene(everyone, {'left': blob(0.0, 0.1)})
    h.check('points inside footprint (no zone): ignored', h.command(0.5, 0.3), (0.5, 0.3, 0.0))
    h.check_scan('/scan: points inside the body box ignored', 87, 'nan')

    h.set_scene(everyone, {'left': blob(0.0, 0.45), 'front': blob(1.3, 0.0)})
    h.check_scan('/scan: merged, left camera obstacle at ~0.45 m', 87, 0.44, 0.52)
    h.check_scan('/scan: merged, front camera obstacle at ~1.3 m', 1, 1.29, 1.36)

    # counts from a checker with a different zone config are ignored -> stale
    h.set_scene(['left', 'right', 'rear'])
    bad = rospy.Publisher('/cam_front/zone_counts', ZoneCounts, queue_size=1)
    rospy.sleep(1.0)
    end = rospy.Time.now() + rospy.Duration(0.6)
    while rospy.Time.now() < end:
        bad.publish(ZoneCounts(zones=['old_zone'], counts=[0]))
        rospy.sleep(0.05)
    h.check('counts with mismatched zone names: forward stays blocked', h.command(0.5, 0.0), (0.0, 0.0, 0.0))
    bad.unregister()

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
