#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from ros_dyn_cfg import TresholdColorPickerConfig


def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {colour_delta}""".format(**config))
    return config


if __name__ == "__main__":
    rospy.init_node("dynamic_tutorials", anonymous=False)

    srv = Server(TresholdColorPickerConfig, callback)
    rospy.spin()
