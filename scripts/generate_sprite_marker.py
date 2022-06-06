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

        camera_frame = rospy.get_param("~camera_frame", "camera1")
        world_frame = rospy.get_param("~world_frame", "map")

        marker_array = MarkerArray()

        # background image behind all the other sprites
        frame = rospy.get_param("~frame_bg", camera_frame)
        image = rospy.get_param("~image_bg")
        scale = rospy.get_param("~scale_bg", 8.0)
        marker = make_marker(frame_id=frame)
        marker.ns = "background"
        marker.id = 0
        # this has to be in sprites 'images' param dictionary
        # TODO(lucasw) make this the path to the image
        marker.mesh_resource = image
        marker.scale.x = scale * 1.25
        marker.scale.y = scale * 3.25
        marker.points = [Point(8.0, -0.4, 0.0)]
        marker_array.markers.append(marker)

        # these ground and objects will be commingled together and sorted by z
        # because both have marker.id 0
        frame = rospy.get_param("~frame0", world_frame)
        image = rospy.get_param("~image0")
        scale = rospy.get_param("~scale0", 2.0)
        marker = make_marker(frame_id=frame)
        marker.ns = "ground"
        marker.id = 5
        # this has to be in sprites 'images' param dictionary
        # TODO(lucasw) make this the path to the image
        marker.mesh_resource = image
        marker.scale.x = scale
        num_x = 16
        num_y = 8
        # make them overlap a little
        object_scale = 1.0
        for xi in range(num_x):
            for yi in range(num_y):
                pt = Point((xi) * object_scale * 1.0, (yi - num_y // 2) * object_scale * 1.95, 0.0)
                marker.points.append(pt)
        marker_array.markers.append(marker)

        if False:
            frame = rospy.get_param("~frame1", world_frame)
            image = rospy.get_param("~image1")
            scale = rospy.get_param("~scale1", 0.5)
            marker = make_marker(frame_id=frame)
            marker.ns = "object"
            marker.id = 5
            # this has to be in sprites 'images' param dictionary
            # TODO(lucasw) make this the path to the image
            marker.mesh_resource = image
            # make them overlap a little
            marker.scale.x = scale
            marker.color.r = 1.0
            object_scale = 4.0
            num_objects = 8
            for i in range(num_objects):
                x = (random.random() - 0.5) * object_scale
                y = (random.random() - 0.5) * object_scale
                z = 0.05
                pt = Point(x, y, z)
                marker.points.append(pt)
            marker_array.markers.append(marker)

        # this will be overlayed because marker.id is higher than above layers
        frame = rospy.get_param("~frame_hud", camera_frame)
        image = rospy.get_param("~image_hud", "hud")
        marker = make_marker(frame_id=frame, scale=0.8)
        marker.ns = "hud"
        marker.id = 10
        marker.mesh_resource = image
        marker.points = [Point(0.0, 0.5, 1.0)]
        marker_array.markers.append(marker)

        self.pub.publish(marker_array)


if __name__ == '__main__':
    rospy.init_node('array_to_marker')
    node = SpriteMarkerArray()
    rospy.spin()
