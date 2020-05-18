#!/usr/bin/env python

"""
Node converts joystick inputs into commands for Turtlesim
"""

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

def callback(data):
    if abs(data.axes[5]) > 0.1:  # increase VEL_X 10%
        global VEL_X
        VEL_X = abs(VEL_X*(data.axes[5] + VEL_X_INCR))
        rospy.loginfo("current max linear speed: %f", VEL_X)
    elif abs(data.axes[4]) > 0.1:  # increase ROT_Z 10%
        global ROT_Z
        ROT_Z = abs(ROT_Z*(data.axes[4] + ROT_Z_INCR))
        rospy.loginfo("current max angular speed: %f", ROT_Z)
    else:
        twist = Twist()
        twist.linear.x = data.axes[1] * VEL_X
        twist.angular.z = data.axes[0] * ROT_Z
        rospy.loginfo("twist.linear: %f ; angular %f", twist.linear.x, twist.angular.z)
        PUB.publish(twist)

if __name__ == '__main__':

    rospy.init_node(name="teleop_joy", anonymous=True)

    topic = rospy.get_param(param_name="~topic")
    VEL_X = rospy.get_param(param_name="~vel_x")
    ROT_Z = rospy.get_param(param_name="~rot_z")
    VEL_X_INCR = rospy.get_param(param_name="~vel_x_incr")
    ROT_Z_INCR = rospy.get_param(param_name="~rot_z_incr")

    PUB = rospy.Publisher(name=topic, data_class=Twist, queue_size=1)
    rospy.Subscriber(name="joy", data_class=Joy, callback=callback, queue_size=1)

    rospy.spin()
