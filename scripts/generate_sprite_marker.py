#!/usr/bin/env python
# Copyright 2017 Lucas Walter
#
# Generate a Marker full of points to use as sprites

import random

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


def make_marker(frame_id: str, scale=0.1) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = "plot"  # marker.header.frame_id
    marker.id = 0
    marker.type = Marker.POINTS
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.color.r = 0.5
    marker.color.g = 0.4
    marker.color.b = 0.3
    marker.color.a = 1.0
    return marker


class SpriteMarkerArray():
    def __init__(self):
        self.pub = rospy.Publisher("marker_array", MarkerArray, queue_size=2, latch=True)

        marker_array = MarkerArray()

        frame = rospy.get_param("~frame0", "map")
        image = rospy.get_param("~image0")
        marker = make_marker(frame_id=frame)
        marker.id = 0
        # this has to be in sprites 'images' param dictionary
        # TODO(lucasw) make this the path to the image
        marker.mesh_resource = image
        scale = 0.2
        # make them overlap a little
        marker.scale.x = scale * 1.1
        num_x = 12
        num_y = 16
        for xi in range(num_x):
            for yi in range(num_y):
                pt = Point((xi - num_x // 2) * scale, (yi - num_y // 2) * scale, 0.0)
                marker.points.append(pt)
        marker_array.markers.append(marker)

        frame = rospy.get_param("~frame1", "map")
        image = rospy.get_param("~image1")
        marker = make_marker(frame_id=frame)
        marker.id = 1
        # this has to be in sprites 'images' param dictionary
        # TODO(lucasw) make this the path to the image
        marker.mesh_resource = image
        # make them overlap a little
        marker.scale.x = scale
        marker.color.r = 1.0
        scale = 1.0
        num_x = 2
        num_y = 3
        for xi in range(num_x):
            for yi in range(num_y):
                x = (xi - num_x // 2) * scale + (random.random() - 0.5) * scale
                y = (yi - num_y // 2) * scale + (random.random() - 0.5) * scale
                z = 0.05
                pt = Point(x, y, z)
                marker.points.append(pt)
        marker_array.markers.append(marker)
        self.pub.publish(marker_array)


if __name__ == '__main__':
    rospy.init_node('array_to_marker')
    node = SpriteMarkerArray()
    rospy.spin()
