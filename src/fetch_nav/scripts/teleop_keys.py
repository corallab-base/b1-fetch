#!/usr/bin/env python
"""Hold-to-move keyboard teleop -> /cmd_vel_raw (the safety filter's manual
input, which overrides navigation). The robot only moves while a key is
held: when key repeat stops for `hold` seconds, publishing stops, the filter
sends one zero and the armed bridge holds the robot still.

  w / s   forward / backward      a / d   left / right (sideways)
  q / e   turn left / right       space   stop now      Ctrl+C  quit

  rosrun fetch_nav teleop_keys.py _vx:=0.2 _vy:=0.15 _wz:=0.4
The bridge still caps speeds (vx 0.3, backward 0.07, vy 0.2, wz 0.5).
"""
from __future__ import print_function

import select
import sys
import termios
import tty

import rospy
from geometry_msgs.msg import Twist

KEYS = {'w': (1, 0, 0), 's': (-1, 0, 0), 'a': (0, 1, 0), 'd': (0, -1, 0), 'q': (0, 0, 1), 'e': (0, 0, -1)}


def main():
    rospy.init_node('teleop_keys')
    vx, vy, wz = rospy.get_param('~vx', 0.2), rospy.get_param('~vy', 0.15), rospy.get_param('~wz', 0.4)
    # first key repeat comes ~0.5 s after the press, so hold must outlast it
    hold = rospy.get_param('~hold', 0.6)
    pub = rospy.Publisher('cmd_vel_raw', Twist, queue_size=1)
    print(__doc__.split('\n\n')[1])
    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    cmd, last_key = None, rospy.Time(0)
    try:
        while not rospy.is_shutdown():
            if select.select([sys.stdin], [], [], 0.1)[0]:
                k = sys.stdin.read(1).lower()
                if k in KEYS:
                    sx, sy, sw = KEYS[k]
                    cmd = Twist()
                    cmd.linear.x, cmd.linear.y, cmd.angular.z = sx * vx, sy * vy, sw * wz
                    last_key = rospy.Time.now()
                elif k == ' ':
                    cmd = None
                    pub.publish(Twist())
            if cmd is not None and (rospy.Time.now() - last_key).to_sec() > hold:
                cmd = None  # key released
            if cmd is not None:
                pub.publish(cmd)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)


if __name__ == '__main__':
    main()
