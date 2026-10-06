#!/usr/bin/env python
"""RTAB-Map's grid, prepared for move_base's two costmaps.

  /rtabmap/grid_map -> global (latched): the grid without specks
                    -> rear   (latched, ~rate Hz): only the part behind the
                       robot, everything else free

Specks: obstacle clusters of at most ~speck_max_cells cells standing in open
floor (at least ~speck_free_ratio of the cells around them known free).
Inflated by the costmaps they become ~0.7 m discs that block planning, and
most are depth noise or people seen while mapping (lab2, 2026-10-07: 61 of
123 obstacle clusters). Walls and furniture are bigger clusters and stay.
The live cameras still mark small real objects (e.g. a chair leg) in the
local costmap, and the safety filter still stops on them.

Rear: the local costmap uses the map only where the cameras don't look:
behind the rear edge (base_link x < -~rear_start) and beside the rear half
(x < -~side_start, |y| > ~side_clear; no camera covers the rear corners), so
obstacles out of view still count when the rear legs swing through a turn.
Everything else comes from the live cameras alone. The margins to the body
(rear edge x -0.60, sides |y| 0.28) keep a map cell a few cm off from
sitting against it and making every TEB trajectory infeasible (2026-10-07
abort: map cells touching the right side, nothing there for the camera).

  rosrun fetch_nav map_relay.py   (started by nav.launch)
"""
import math

import cv2
import numpy as np
import rospy
import tf2_ros
from nav_msgs.msg import OccupancyGrid

OCCUPIED, FREE = 100, 0


def despeckle(grid, max_cells, ring, free_ratio):
    """grid (int8, -1/0/100) without small obstacle clusters in open floor;
    returns (new grid, clusters removed, cells removed)."""
    occ = (grid == OCCUPIED).astype(np.uint8)
    n, labels, stats, _ = cv2.connectedComponentsWithStats(occ, connectivity=8)
    out = grid.copy()
    removed = cells = 0
    h, w = grid.shape
    for i in range(1, n):
        area = stats[i, cv2.CC_STAT_AREA]
        if area > max_cells:
            continue
        x, y = stats[i, cv2.CC_STAT_LEFT], stats[i, cv2.CC_STAT_TOP]
        x0, y0 = max(0, x - ring), max(0, y - ring)
        x1 = min(w, x + stats[i, cv2.CC_STAT_WIDTH] + ring)
        y1 = min(h, y + stats[i, cv2.CC_STAT_HEIGHT] + ring)
        mine = labels[y0:y1, x0:x1] == i
        around = grid[y0:y1, x0:x1][~mine]
        if around.size and np.count_nonzero(around == FREE) >= free_ratio * around.size:
            out[y0:y1, x0:x1][mine] = FREE
            removed += 1
            cells += area
    return out, removed, cells


def main():
    rospy.init_node('map_relay')
    base_frame = rospy.get_param('~base_frame', 'base_link')
    max_cells = rospy.get_param('~speck_max_cells', 4)       # 4 cells = 10 x 10 cm at 5 cm
    ring = rospy.get_param('~speck_ring', 3)                 # cells around a speck that must be free
    free_ratio = rospy.get_param('~speck_free_ratio', 0.8)
    rear_start = rospy.get_param('~rear_start', 0.65)        # m behind base_link; rear edge 0.60 + 5 cm
    side_start = rospy.get_param('~side_start', 0.30)        # m: beside the body from here back
    side_clear = rospy.get_param('~side_clear', 0.38)        # m from the centre line; side 0.28 + 10 cm
    rate = rospy.get_param('~rate', 2.0)

    pub_global = rospy.Publisher('~global', OccupancyGrid, queue_size=1, latch=True)
    pub_rear = rospy.Publisher('~rear', OccupancyGrid, queue_size=1, latch=True)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    state = {}

    def on_map(msg):
        info = msg.info
        grid = np.array(msg.data, dtype=np.int8).reshape(info.height, info.width)
        clean, removed, cells = despeckle(grid, max_cells, ring, free_ratio)
        out = OccupancyGrid(header=msg.header, info=info)
        out.data = clean.ravel().tolist()
        pub_global.publish(out)
        q = info.origin.orientation   # RTAB-Map's grid is axis-aligned; cell centres in the map frame
        if abs(q.z) > 1e-3 or abs(q.x) > 1e-3 or abs(q.y) > 1e-3:
            rospy.logwarn_throttle(30.0, 'map_relay: rotated map origin not supported, rear map off')
            state.clear()
            return
        r = info.resolution
        xs = info.origin.position.x + (np.arange(info.width, dtype=np.float32) + 0.5) * r
        ys = info.origin.position.y + (np.arange(info.height, dtype=np.float32) + 0.5) * r
        state.update(msg=msg, clean=clean, X=xs[np.newaxis, :], Y=ys[:, np.newaxis])
        rospy.loginfo_throttle(60.0, 'map_relay: %dx%d map, removed %d specks (%d cells)',
                               info.width, info.height, removed, cells)

    def on_timer(_):
        if not state:
            return
        msg = state['msg']
        try:
            t = buf.lookup_transform(msg.header.frame_id, base_frame, rospy.Time(0),
                                     rospy.Duration(0.1)).transform
        except tf2_ros.TransformException as e:
            rospy.logwarn_throttle(10.0, 'map_relay: no %s pose, rear map not updated: %s', base_frame, e)
            return
        q = t.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        # every cell in the robot frame; keep the map behind the body and beside its rear half
        dx, dy = state['X'] - t.translation.x, state['Y'] - t.translation.y
        c, s = math.cos(yaw), math.sin(yaw)
        bx, by = c * dx + s * dy, -s * dx + c * dy
        keep = (bx < -rear_start) | ((bx < -side_start) & (np.abs(by) > side_clear))
        rear = np.where(keep, state['clean'], np.int8(FREE))
        out = OccupancyGrid(info=msg.info)
        out.header.frame_id = msg.header.frame_id
        out.header.stamp = rospy.Time.now()
        out.data = rear.ravel().tolist()
        pub_rear.publish(out)

    rospy.Subscriber('/rtabmap/grid_map', OccupancyGrid, on_map, queue_size=1, buff_size=2 ** 24)
    rospy.Timer(rospy.Duration(1.0 / rate), on_timer)
    rospy.spin()


if __name__ == '__main__':
    main()
