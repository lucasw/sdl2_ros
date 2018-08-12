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
        # should be smaller then index_scale if individual points can vary a lot
        scale = rospy.get_param("~line_thickness", 0.01)
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale

        marker.color.a = 1.0
        marker.color.r = rospy.get_param("~r", 1.0)
        marker.color.g = rospy.get_param("~g", 1.0)
        marker.color.b = rospy.get_param("~b", 1.0)
        self.marker = marker

        self.max_points = rospy.get_param("~max_points", 1000)
        self.step = rospy.get_param("~step", 1)
        self.index_scale = rospy.get_param("~index_scale", 0.001)
        self.y_scale = rospy.get_param("~y_scale", 1.0 / (2**15))
        self.sub = rospy.Subscriber("audio", Int16MultiArray,
                                    self.callback, queue_size=1)

    def callback(self, msg):
        self.marker.points = []
        for i in range(0, len(msg.data), self.step):
            if (i > self.max_points):
                break
            pt = Point()
            pt.x = i * self.index_scale
            pt.y = msg.data[i] * self.y_scale
            pt.z = 0
            self.marker.points.append(pt)

        self.marker.header.stamp = rospy.Time.now()
        rospy.loginfo(len(self.marker.points))
        self.pub.publish(self.marker)

if __name__ == '__main__':
    rospy.init_node('array_to_marker')
    array_to_marker = ArrayToMarker()
    rospy.spin()
