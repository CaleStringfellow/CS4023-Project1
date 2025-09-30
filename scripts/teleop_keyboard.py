#!/usr/bin/env python
"""
Simple keyboard teleop publisher that sends Twist messages to /teleop_cmd.
Controls:
  W/S : forward/back
  A/D : left/right (angular)
  Space or x : stop

Run this in a terminal and focus it to control the robot.
"""

import rospy
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty


def get_key(timeout=0.1):
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


def teleop():
    rospy.init_node('teleop_keyboard')
    pub = rospy.Publisher('/teleop_cmd', Twist, queue_size=1)

    linear_speed = rospy.get_param('~linear_speed', 0.2)
    angular_speed = rospy.get_param('~angular_speed', 0.5)

    print("Teleop started. Use WASD to drive, space to stop, Ctrl-C to exit.")

    rate = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():
            key = get_key(0.1)
            twist = Twist()
            if key in ['w', 'W']:
                twist.linear.x = linear_speed
            elif key in ['s', 'S']:
                twist.linear.x = -linear_speed
            elif key in ['a', 'A']:
                twist.angular.z = angular_speed
            elif key in ['d', 'D']:
                twist.angular.z = -angular_speed
            elif key in [' ', 'x', 'X']:
                twist = Twist()
            elif key == '\x03':  # Ctrl-C
                break

            pub.publish(twist)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    teleop()
