#!/usr/bin/env python
"""Fake B1 + cameras for offline navigation tests (no robot, no cameras).

- Drives on /cmd_vel (the safety filter's output) like the bridge+robot:
  body-frame velocity, held for 0.5 s after the last command.
- Publishes TF odom -> base_link at 30 Hz; odometry under-reports distance by
  ~odom_scale (3% by default, like the real B1) so mapping must correct it.
- Publishes what the left/right/front cameras would see of a lab-like room
  (9 x 6 m walls, a 2 x 1 m table island) as base_link clouds on
  /cam_*/depth/color/points at 15 Hz, cropped to each camera's view.
- ~true_pose (geometry_msgs/Pose2D) is the real pose, for checking.
"""
import math
import rospy
import tf2_ros
from geometry_msgs.msg import Pose2D, TransformStamped, Twist
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


def room():
    pts = []
    def segment(x0, y0, x1, y1, step=0.04):
        n = int(math.hypot(x1 - x0, y1 - y0) / step) + 1
        for i in range(n):
            t = float(i) / max(1, n - 1)
            pts.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))
    segment(-2, -3, 7, -3); segment(7, -3, 7, 3); segment(7, 3, -2, 3); segment(-2, 3, -2, -3)
    segment(1.5, -0.5, 3.5, -0.5); segment(3.5, -0.5, 3.5, 0.5); segment(3.5, 0.5, 1.5, 0.5); segment(1.5, 0.5, 1.5, -0.5)
    segment(5.0, 2.2, 6.0, 2.2); segment(-1.0, -2.0, -1.0, -1.2)   # a desk and a box
    return pts


# camera: (bearing of its centre, half field of view), in base_link
CAMS = {'front': (0.0, math.radians(43)), 'left': (math.pi / 2, math.radians(43)), 'right': (-math.pi / 2, math.radians(43))}


def main():
    rospy.init_node('sim_robot')
    scale = rospy.get_param('~odom_scale', 0.97)
    world = room()
    x = y = yaw = 0.0          # true pose
    ox = oy = 0.0              # odometry (scaled distances)
    def on_cmd(msg):
        on_cmd.cmd, on_cmd.time = msg, rospy.Time.now()
    on_cmd.cmd, on_cmd.time = Twist(), rospy.Time(0)
    rospy.Subscriber('/cmd_vel', Twist, on_cmd)
    br = tf2_ros.TransformBroadcaster()
    pose_pub = rospy.Publisher('~true_pose', Pose2D, queue_size=1)
    pubs = {c: rospy.Publisher('/cam_%s/depth/color/points' % c, PointCloud2, queue_size=1) for c in CAMS}
    rate = rospy.Rate(30)
    last = rospy.Time.now()
    tick = 0
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        dt = (now - last).to_sec()
        last = now
        c = on_cmd.cmd if (now - on_cmd.time).to_sec() < 0.5 else Twist()
        dx = (c.linear.x * math.cos(yaw) - c.linear.y * math.sin(yaw)) * dt
        dy = (c.linear.x * math.sin(yaw) + c.linear.y * math.cos(yaw)) * dt
        x += dx; y += dy; ox += scale * dx; oy += scale * dy
        yaw += c.angular.z * dt
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = now, 'odom', 'base_link'
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = ox, oy, 0.55
        t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        br.sendTransform(t)
        pose_pub.publish(Pose2D(x, y, yaw))
        tick += 1
        if tick % 2 == 0:   # cameras at 15 Hz
            cy, sy = math.cos(-yaw), math.sin(-yaw)
            local = [(cy * (wx - x) - sy * (wy - y), sy * (wx - x) + cy * (wy - y)) for wx, wy in world]
            h = Header(stamp=now, frame_id='base_link')
            for cam, (centre, half) in CAMS.items():
                pts = []
                for lx, ly in local:
                    r = math.hypot(lx, ly)
                    d = (math.atan2(ly, lx) - centre + math.pi) % (2 * math.pi) - math.pi
                    if 0.3 < r < 3.5 and abs(d) < half:
                        pts.append((lx, ly, -0.3)); pts.append((lx, ly, 0.0))
                pubs[cam].publish(point_cloud2.create_cloud_xyz32(h, pts))
        rate.sleep()


if __name__ == '__main__':
    main()
