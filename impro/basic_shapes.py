#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import os


def make_label(x, y, z):

    marker = Marker()
    marker.header.frame_id = '/my_frame'
    marker.id = 0
    marker.ns = 'basic_shapes'
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1
    marker.color.a = 1.0
    marker.color.r = 0
    marker.color.g = 1
    marker.color.b = 0
    marker.lifetime = rospy.Duration(10)
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    return marker


def talker():
    pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rospy.init_node('basic_shapes', anonymous=True)
    rate = rospy.Rate(0.5) # hz
    base = 0
    while not rospy.is_shutdown():
        x = base // 4
        base = base +1
        pub.publish(make_label(1, 0, 0))
        # pub.publish(make_label(x, 0, 0))
        rate.sleep()


if __name__ == '__main__':
    print(os.environ['ROS_ROOT'])
    print(os.environ['ROS_PACKAGE_PATH'])
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
