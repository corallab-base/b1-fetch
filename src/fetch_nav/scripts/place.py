#!/usr/bin/env python
"""Named places on the map: save them, list them, drive to them.

  rosrun fetch_nav place.py save kitchen          # where the robot stands now
  rosrun fetch_nav place.py save table --click    # a pose clicked in Foxglove (publish-pose
                                                  # tool on /place_click, frame map)
  rosrun fetch_nav place.py go kitchen            # drive there (Ctrl+C cancels)
  rosrun fetch_nav place.py go kitchen --no-wait  # send the goal and return
  rosrun fetch_nav place.py list
  rosrun fetch_nav place.py delete table
  ... --map lab3                                  # another map than the one nav.launch runs

Places live per map in ~/projects/fetch/maps/<map>_places.yaml (x, y in
metres and heading in degrees, map frame). A place is where the robot should
stand, facing the way it should face (for "table": in front of it, facing
it). Saving the robot's own pose is the reliable way: it fits there.

move_base (Melodic GlobalPlanner) finds no plan to a goal closer to a mapped
obstacle than half the robot's width. So clicked places, and every goal
before it is sent, are checked on the global costmap; a goal in such a cell
moves to the nearest cell the robot fits within ~snap m (said so on screen).
"""
import argparse
import math
import os
import sys

import actionlib
import rosgraph
import rospy
import tf2_ros
import yaml
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid

MAPS = os.path.expanduser('~/projects/fetch/maps')
BLOCKED = 99   # costmap topic scale: 100 lethal, 99 inscribed (robot centre can't be there), -1 unknown


def places_file(map_name):
    return os.path.join(MAPS, '%s_places.yaml' % map_name)


def load(path):
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        return yaml.safe_load(f) or {}


def store(path, places):
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        f.write('# Named places for move_base (place.py): x, y [m] and heading [deg] in the map frame.\n')
        yaml.safe_dump(places, f, default_flow_style=False)
    os.rename(tmp, path)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def robot_pose(timeout=3.0):
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    t = buf.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(timeout)).transform
    return t.translation.x, t.translation.y, yaw_of(t.rotation)


def check(x, y, snap):
    """(x, y) if the robot fits there, else the nearest cell it fits within
    snap m, else None. Also returns a note for the user."""
    try:
        m = rospy.wait_for_message('/move_base/global_costmap/costmap', OccupancyGrid, 5.0)
    except rospy.ROSException:
        return (x, y), 'no global costmap (is nav.launch running?): not checked'
    r, w, h = m.info.resolution, m.info.width, m.info.height
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    cx, cy = int((x - ox) / r), int((y - oy) / r)
    if not (0 <= cx < w and 0 <= cy < h):
        return (x, y), 'outside the global costmap window: not checked'
    cost = m.data[cy * w + cx]
    if 0 <= cost < BLOCKED:
        return (x, y), None
    n = int(snap / r)
    best = None
    for j in range(max(0, cy - n), min(h, cy + n + 1)):
        for i in range(max(0, cx - n), min(w, cx + n + 1)):
            c = m.data[j * w + i]
            d = math.hypot(i - cx, j - cy) * r
            if 0 <= c < BLOCKED and d <= snap and (best is None or d < best[0]):
                best = (d, ox + (i + 0.5) * r, oy + (j + 0.5) * r)
    what = 'unknown space' if cost < 0 else 'too close to an obstacle'
    if best is None:
        return None, '(%.2f, %.2f) is %s and nothing within %.2f m fits the robot' % (x, y, what, snap)
    return (best[1], best[2]), '(%.2f, %.2f) is %s: moved %.2f m to (%.2f, %.2f)' % (x, y, what, best[0], best[1], best[2])


def main():
    ap = argparse.ArgumentParser(description='Named places on the map.')
    ap.add_argument('command', choices=['save', 'go', 'list', 'delete'])
    ap.add_argument('name', nargs='?')
    ap.add_argument('--click', action='store_true', help='save: wait for a pose on /place_click')
    ap.add_argument('--no-wait', action='store_true', help='go: send the goal and return')
    ap.add_argument('--map', help='map name (default: the one nav.launch runs)')
    ap.add_argument('--snap', type=float, default=0.5, help='max distance to move a blocked goal [m]')
    a = ap.parse_args(rospy.myargv()[1:])
    online = rosgraph.is_master_online()
    if online:   # own Ctrl+C handling: "go" cancels its goal before exiting
        rospy.init_node('place', anonymous=True, disable_signals=True)
    elif a.command in ('save', 'go'):
        sys.exit('no ROS master: start robot.sh and nav.launch first')
    map_name = a.map or (rospy.get_param('/fetch_nav/map', None) if online else None)
    if not map_name:
        sys.exit('which map? nav.launch is not running: give --map NAME')
    path = places_file(map_name)
    places = load(path)
    if a.command != 'list' and not a.name:
        sys.exit('give a place name')

    if a.command == 'list':
        if not places:
            print('no places for map %s yet (%s)' % (map_name, path))
            return
        rx = ry = None
        if online:
            try:
                rx, ry, _ = robot_pose(2.0)
            except Exception:
                pass
        print('places on map %s:' % map_name)
        for name in sorted(places):
            p = places[name]
            dist = '' if rx is None else '  %.1f m away' % math.hypot(p['x'] - rx, p['y'] - ry)
            print('  %-16s (%6.2f, %6.2f) facing %4.0f deg%s' % (name, p['x'], p['y'], p['heading'], dist))

    elif a.command == 'delete':
        if a.name not in places:
            sys.exit('no place "%s" on map %s' % (a.name, map_name))
        del places[a.name]
        store(path, places)
        print('deleted %s' % a.name)

    elif a.command == 'save':
        if a.click:
            print('click the place in Foxglove: publish-pose tool on /place_click (drag for the heading)...')
            msg = rospy.wait_for_message('/place_click', PoseStamped)
            if msg.header.frame_id.lstrip('/') != 'map':
                sys.exit('the click was in frame "%s": set the tool / display frame to map' % msg.header.frame_id)
            x, y, yaw = msg.pose.position.x, msg.pose.position.y, yaw_of(msg.pose.orientation)
            xy, note = check(x, y, a.snap)
            if note:
                print(note)
            if xy is None:
                sys.exit('not saved')
            x, y = xy
        else:
            x, y, yaw = robot_pose()
        old = ' (replaces the old one)' if a.name in places else ''
        places[a.name] = {'x': round(x, 3), 'y': round(y, 3), 'heading': round(math.degrees(yaw), 1)}
        store(path, places)
        print('saved %s on map %s: (%.2f, %.2f) facing %.0f deg%s' % (a.name, map_name, x, y, math.degrees(yaw), old))

    elif a.command == 'go':
        if a.name not in places:
            sys.exit('no place "%s" on map %s (place.py list)' % (a.name, map_name))
        p = places[a.name]
        xy, note = check(p['x'], p['y'], a.snap)
        if note:
            print(note)
        if xy is None:
            sys.exit('not sent')
        yaw = math.radians(p['heading'])
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x, goal.target_pose.pose.position.y = xy
        goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        if not client.wait_for_server(rospy.Duration(5.0)):
            sys.exit('move_base is not running (nav.launch)')
        client.send_goal(goal)
        print('going to %s (%.2f, %.2f), facing %.0f deg' % (a.name, xy[0], xy[1], p['heading']))
        if a.no_wait:
            return
        try:
            while not client.wait_for_result(rospy.Duration(5.0)):
                try:
                    rx, ry, _ = robot_pose(1.0)
                    print('  %.1f m to go' % math.hypot(xy[0] - rx, xy[1] - ry))
                except tf2_ros.TransformException:
                    pass
        except KeyboardInterrupt:
            client.cancel_goal()
            rospy.sleep(0.5)   # let the cancel go out
            print('cancelled')
            return
        state = client.get_state()
        print('arrived at %s' % a.name if state == GoalStatus.SUCCEEDED else
              'not reached: %s' % {GoalStatus.ABORTED: 'move_base gave up (no plan / no valid control)',
                                   GoalStatus.PREEMPTED: 'replaced or cancelled'}.get(state, 'state %d' % state))
        sys.exit(0 if state == GoalStatus.SUCCEEDED else 1)


if __name__ == '__main__':
    main()
