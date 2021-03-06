#!/usr/bin/env python

"""
Convert ros-kinetic-joy inputs.
Caution:
    The parameter deadzone of ros-kinetic-joy should not equal to zero.
    Make sure the release state of joy can publish the zero command.
"""

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

def callback(data):
    if abs(data.axes[5]) > 0.1:
        global VX_MAX
        VX_MAX = abs(VX_MAX*(data.axes[5] + 0.1))  # increase speed by 10%
        rospy.loginfo("current max linear speed: %f", VX_MAX)
    elif abs(data.axes[4]) > 0.1:
        global WZ_MAX
        WZ_MAX = abs(WZ_MAX*(data.axes[4] + 0.1))  # increase turn by 10%
        rospy.loginfo("current max angular speed: %f", WZ_MAX)
    else:
        global VX_LAST
        global WZ_LAST
        twist = Twist()
        vx = data.axes[1] * VX_MAX
        wz = data.axes[0] * WZ_MAX
        if vx == 0.0 and wz == 0.0 and VX_LAST == 0.0 and WZ_LAST == 0.0:
            pass
        else:
            twist.linear.x = vx
            twist.angular.z = wz
            rospy.loginfo("twist.linear: %f ; angular %f", twist.linear.x, twist.angular.z)
            PUB.publish(twist)
        VX_LAST = vx
        WZ_LAST = wz

if __name__ == '__main__':

    rospy.init_node(name="teleop_joy", anonymous=True)

    VX_MAX = rospy.get_param(param_name="~vx_max")
    WZ_MAX = rospy.get_param(param_name="~wz_max")
    VX_LAST = 0.0
    WZ_LAST = 0.0

    PUB = rospy.Publisher(name="cmd_vel", data_class=Twist, queue_size=1)
    rospy.Subscriber(name="joy", data_class=Joy, callback=callback, queue_size=1)

    rospy.spin()
