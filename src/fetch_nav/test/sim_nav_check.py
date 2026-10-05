#!/usr/bin/env python
"""Sends move_base goals to the simulated robot (sim_nav.launch) and checks
it arrives, never touches an obstacle (true pose vs the sim room) and never
gets a backward command from navigation."""
from __future__ import print_function
import math
import sys
import actionlib
import rospy
from geometry_msgs.msg import Pose2D, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
sys.path.insert(0, __file__.rsplit('/', 1)[0])
from sim_robot import room

HALF_L, HALF_W = 0.565, 0.235   # real body (footprint in the costmap is larger)
GOALS = [(5.0, 0.0, 0.0, 'around the table island'), (0.5, -1.8, math.pi, 'back, facing the other way')]


def main():
    rospy.init_node('sim_nav_check')
    world = room()
    state = {'pose': None, 'min_clear': 9.0, 'min_vx': 0.0, 'turned': 0.0, 'path_turn': 0.0, 'seg': None, 'dir': None}

    def on_pose(p):
        if state['pose'] is not None:
            state['turned'] += abs(p.theta - state['pose'].theta)   # total rotation, both ways
        # how much the direction of travel bends (every 10 cm of motion)
        if state['seg'] is None:
            state['seg'] = (p.x, p.y)
        elif math.hypot(p.x - state['seg'][0], p.y - state['seg'][1]) > 0.1:
            d = math.atan2(p.y - state['seg'][1], p.x - state['seg'][0])
            if state['dir'] is not None:
                state['path_turn'] += abs((d - state['dir'] + math.pi) % (2 * math.pi) - math.pi)
            state['dir'], state['seg'] = d, (p.x, p.y)
        state['pose'] = p
        c, s = math.cos(-p.theta), math.sin(-p.theta)
        for wx, wy in world:
            lx, ly = c * (wx - p.x) - s * (wy - p.y), s * (wx - p.x) + c * (wy - p.y)
            dx, dy = max(0.0, abs(lx) - HALF_L), max(0.0, abs(ly) - HALF_W)
            state['min_clear'] = min(state['min_clear'], math.hypot(dx, dy))
    rospy.Subscriber('/sim_robot/true_pose', Pose2D, on_pose)
    rospy.Subscriber('/cmd_vel', Twist, lambda t: state.__setitem__('min_vx', min(state['min_vx'], t.linear.x)))

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    if not client.wait_for_server(rospy.Duration(60)):
        print('FAIL move_base action server not up'); sys.exit(1)
    rospy.sleep(5.0)   # let RTAB-Map publish its first grid
    failures = 0
    for gx, gy, gyaw, what in GOALS:
        g = MoveBaseGoal()
        g.target_pose.header.frame_id = 'map'
        g.target_pose.header.stamp = rospy.Time.now()
        g.target_pose.pose.position.x, g.target_pose.pose.position.y = gx, gy
        g.target_pose.pose.orientation.z, g.target_pose.pose.orientation.w = math.sin(gyaw / 2), math.cos(gyaw / 2)
        t0 = rospy.Time.now()
        start = state['pose']
        state['turned'] = state['path_turn'] = 0.0
        state['seg'] = state['dir'] = None
        client.send_goal(g)
        done = client.wait_for_result(rospy.Duration(180))
        p = state['pose']
        err = math.hypot(p.x - gx, p.y - gy)
        yaw_err = abs((p.theta - gyaw + math.pi) % (2 * math.pi) - math.pi)
        # turning the robot has to do: as much as its path bends, plus the
        # net change from start to goal heading; flag anything well beyond
        needed = abs((gyaw - start.theta + math.pi) % (2 * math.pi) - math.pi) + state['path_turn']
        spin_ok = state['turned'] < needed + math.radians(120)
        ok = done and client.get_state() == 3 and err < 0.35 and yaw_err < 0.4 and spin_ok
        print('%s goal (%.1f, %.1f) %-28s %s in %.0f s: true pose (%.2f, %.2f, %.0f deg), %.2f m / %.0f deg off' % (
            'PASS' if ok else 'FAIL', gx, gy, what, 'SUCCEEDED' if client.get_state() == 3 else 'state %d' % client.get_state(),
            (rospy.Time.now() - t0).to_sec(), p.x, p.y, math.degrees(p.theta), err, math.degrees(yaw_err)))
        print('     turned %.0f deg in total (path bends + heading change: %.0f deg)%s' % (math.degrees(state['turned']), math.degrees(needed),
              '' if spin_ok else '  <- needless spinning'))
        failures += not ok
    ok = state['min_clear'] > 0.0
    print('%s closest the true body came to an obstacle: %.2f m' % ('PASS' if ok else 'FAIL', state['min_clear']))
    failures += not ok
    ok = state['min_vx'] >= 0.0
    print('%s most negative forward speed sent to the robot: %.3f m/s' % ('PASS' if ok else 'FAIL', state['min_vx']))
    failures += not ok
    print('\n%d failure(s)' % failures)
    sys.exit(1 if failures else 0)


if __name__ == '__main__':
    main()
