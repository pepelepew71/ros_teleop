#!/usr/bin/env python

"""
Node converts joystick inputs into commands for Turtlesim
"""

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

def callback(data):
    twist = Twist()
    twist.linear.x = data.axes[1]
    twist.angular.z = data.axes[0]
    rospy.loginfo("twist.linear: %f ; angular %f", twist.linear.x, twist.angular.z)
    PUB.publish(twist)

if __name__ == '__main__':

    try:
        rospy.init_node(name="teleop_joy", anonymous=True)
        PUB = rospy.Publisher(name="/cmd_vel", data_class=Twist, queue_size=1)
        rospy.Subscriber(name="joy", data_class=Joy, callback=callback, queue_size=1)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



