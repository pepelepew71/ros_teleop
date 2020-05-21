#!/usr/bin/env python

"""
Node converts joystick inputs into commands for Turtlesim
"""

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

def callback(data):
    if abs(data.axes[5]) > 0.1:
        global SPEED
        SPEED = abs(SPEED*(data.axes[5] + 0.1))  # increase speed by 10%
        rospy.loginfo("current max linear speed: %f", SPEED)
    elif abs(data.axes[4]) > 0.1:
        global TURN
        TURN = abs(TURN*(data.axes[4] + 0.1))  # increase turn by 10%
        rospy.loginfo("current max angular speed: %f", TURN)
    else:
        twist = Twist()
        twist.linear.x = data.axes[1] * SPEED
        twist.angular.z = data.axes[0] * TURN
        rospy.loginfo("twist.linear: %f ; angular %f", twist.linear.x, twist.angular.z)
        PUB.publish(twist)

if __name__ == '__main__':

    rospy.init_node(name="teleop_joy", anonymous=True)

    SPEED = rospy.get_param(param_name="~speed")
    TURN = rospy.get_param(param_name="~turn")

    PUB = rospy.Publisher(name="cmd_vel", data_class=Twist, queue_size=1)
    rospy.Subscriber(name="joy", data_class=Joy, callback=callback, queue_size=1)

    rospy.spin()
