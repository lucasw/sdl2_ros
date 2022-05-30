#!/usr/bin/env python
# Copyright 2017 Lucas Walter
#
# Generate a Marker full of points to use as sprites

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


def make_marker(frame_id: str) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = "plot"  # marker.header.frame_id
    marker.id = 0
    marker.type = Marker.POINTS
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    scale = 0.1
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
        marker = make_marker(frame_id="map")
        # this has to be in sprites 'images' param dictionary
        # TODO(lucasw) make this the path to the image
        marker.mesh_resource = "map"

        num_x = 12
        num_y = 16
        for xi in range(num_x):
            for yi in range(num_y):
                pt = Point((xi - num_x // 2) * 0.2, (yi - num_y // 2) * 0.2, xi * 0.01 + yi * 0.01)
                marker.points.append(pt)

        marker_array.markers.append(marker)
        self.pub.publish(marker_array)


if __name__ == '__main__':
    rospy.init_node('array_to_marker')
    node = SpriteMarkerArray()
    rospy.spin()
