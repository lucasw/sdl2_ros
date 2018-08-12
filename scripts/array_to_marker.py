#!/usr/bin/env python
# Copyright 2017 Lucas Walter
#
# Generate triangle list Marker

import math
import rospy

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, Int16MultiArray
from visualization_msgs.msg import Marker


class ArrayToMarker():
    def __init__(self):
        self.pub = rospy.Publisher("marker", Marker, queue_size=2)

        marker = Marker()
        marker.header.frame_id = rospy.get_param("~frame_id", "map")
        marker.ns = "plot"  # marker.header.frame_id
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        marker.color.a = 1.0
        marker.color.r = rospy.get_param("~r", 1.0)
        marker.color.g = rospy.get_param("~g", 1.0)
        marker.color.b = rospy.get_param("~b", 1.0)
        self.marker = marker

        self.sub = rospy.Subscriber("audio", Int16MultiArray,
                                    self.callback, queue_size=1)

    def callback(self, msg):
        self.marker.points = []
        for i in range(len(msg.data)):
            if (i > 2000):
                break
            pt = Point()
            pt.x = i / 1000.0
            pt.y = msg.data[i] / float(2**15)
            pt.z = 0
            self.marker.points.append(pt)

        self.marker.header.stamp = rospy.Time.now()
        rospy.loginfo(len(self.marker.points))
        self.pub.publish(self.marker)

if __name__ == '__main__':
    rospy.init_node('array_to_marker')
    array_to_marker = ArrayToMarker()
    rospy.spin()
