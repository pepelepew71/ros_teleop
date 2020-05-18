#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Control mrobot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

moveBindings = {
    'i':(1,0),
    'o':(1,-1),
    'j':(0,1),
    'l':(0,-1),
    'u':(1,1),
    ',':(-1,0),
    '.':(-1,1),
    'm':(-1,-1)}

speedBindings={
    'q':(1.1,1.1),
    'z':(.9,.9),
    'w':(1.1,1),
    'x':(.9,1),
    'e':(1,1.1),
    'c':(1,.9)}

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":

    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node(name='teleop_key', anonymous=True)
    topic = rospy.get_param(param_name="~topic")
    pub = rospy.Publisher(name=topic, data_class=Twist, queue_size=5)

    speed = rospy.get_param(param_name="~vel_x")
    turn = rospy.get_param(param_name="~rot_z")

    x = 0
    th = 0
    count = 0

    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0

    print(msg)
    print(vels(speed,turn))

    try:
        while True:
            key = getKey()

            if key in moveBindings.keys():  # -- key: direction
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                count = 0
            elif key in speedBindings.keys():  # -- key: speed
                speed = speed * speedBindings[key][0]  # -- linear speed incresed 0.1x
                turn = turn * speedBindings[key][1]    # -- angular speed incresed 0.1x
                count = 0
                print(vels(speed,turn))
            elif key == ' ' or key == 'k':  # -- key: stop
                x = 0
                th = 0
                control_speed = 0
                control_turn = 0
            else:
                count = count + 1
                if count > 4:
                    x = 0
                    th = 0
                if key == '\x03':
                    break

            # -- current speed = speed * direction
            target_speed = speed * x
            target_turn = turn * th

            # -- speed limit
            if target_speed > control_speed:
                control_speed = min(target_speed, control_speed + 0.02)
            elif target_speed < control_speed:
                control_speed = max(target_speed, control_speed - 0.02)
            else:
                control_speed = target_speed

            if target_turn > control_turn:
                control_turn = min(target_turn, control_turn + 0.1)
            elif target_turn < control_turn:
                control_turn = max(target_turn, control_turn - 0.1)
            else:
                control_turn = target_turn

            # -- publush to /cmd_vel
            twist = Twist()
            twist.linear.x = control_speed
            twist.angular.z = control_turn
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        pub.publish(twist)  # all zero

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
