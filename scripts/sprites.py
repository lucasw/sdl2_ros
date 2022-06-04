#!/usr/bin/env python
# display sprites in an image with ros integration- publish the image out, only optionally
# show an sdl window
# rosrun sdl2_ros sdl2_sprites.py _image1:=`rospack find vimjay`/data/plasma.png

import queue
from threading import Lock
from typing import List

import cv2
import numpy as np
import rospy
import sdl2
import sdl2.ext
import sdl2.sdlgfx
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from sensor_msgs import point_cloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from tf import transformations


def points_to_point_cloud2(points: List[Point], header) -> PointCloud2:
    field_points = []
    for pt in points:
        pt = [pt.x, pt.y, pt.z]
        field_points.append(pt)

    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              ]

    pc2 = point_cloud2.create_cloud(header, fields, field_points)
    # pc2.header.stamp = stamp
    # pc2.header.frame_id = frame_id
    return pc2


def marker_to_point_cloud2(marker: Marker) -> PointCloud2:
    return points_to_point_cloud2(marker.points, marker.header)


class SDL2Sprite(object):
    def __init__(self, sprite, sprite_original, x=0.0, y=0.0):
        self.sprite = sprite
        self.sprite_original = sprite_original
        self.update_position(x, y)
        self.is_active = True

    def __repr__(self):
        return f"x {self.px:0.2f} y {self.py:0.2f}, {self.sprite.position}"

    def update_position(self, x=0.0, y=0.0):
        self.px = x
        self.py = y
        self.sprite.position = (int(self.px), int(self.py))

    def rotozoom(self, angle=0.0, zoom=1.0):
        rotozoom = sdl2.sdlgfx.rotozoomSurface
        surface = rotozoom(self.sprite_original.surface,
                           angle,
                           zoom,
                           1).contents
        sdl2.SDL_FreeSurface(self.sprite.surface)
        self.sprite.surface = surface


class SDL2Sprites(object):
    def __init__(self):
        self.lock = Lock()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # fill tf buffer
        rospy.sleep(1.0)

        self.cv_bridge = CvBridge()
        rospy.loginfo(f"SDL2 Version {sdl2.__version__}")

        while not rospy.is_shutdown():
            try:
                self.init_camera_info = rospy.wait_for_message("camera_info", CameraInfo, timeout=5.0)
                break
            except Exception as ex:
                rospy.logdebug(ex)
                rospy.logwarn("waiting for camera_info...")
                rospy.sleep(2.0)
        image_width = self.init_camera_info.width
        image_height = self.init_camera_info.height

        # can either be 3 or 4, 4 will be faster here but downstream nodes may not like
        # the alpha channel at all or will work slower
        # num_chan 3 is 15 ms, while 4 is < 1ms - maybe a downstream nodelet is faster?
        self.num_chan = rospy.get_param("~num_chan", 3)

        # maybe useful for debug if True, or as an example to make an pysdl2 image_view node
        self.show_sdl_window = rospy.get_param("~show_sdl_window", False)

        self.window = sdl2.ext.Window("sdl2_sprites", size=(image_width, image_height))
        if self.show_sdl_window:
            self.window.show()

        self.factory = sdl2.ext.SpriteFactory(sdl2.ext.SOFTWARE)

        images = rospy.get_param("~images")
        rospy.loginfo(images)

        self.sprite_renderer = self.factory.create_sprite_render_system(self.window)
        # self.renderer = sdl2.ext.Renderer(window)
        # sdl2.ext.set_texture_scale_quality(method="nearest")

        # max per sprite type, not total max
        max_sprites = rospy.get_param("~max_sprites", 200)

        self.sdl2_sprites = {}
        for key, image_path in images.items():
            self.sdl2_sprites[key] = []
            for i in range(max_sprites):
                sprite = self.factory.from_image(image_path)
                # TODO(lucasw) how to make copy of sprite?
                sprite_original = self.factory.from_image(image_path)
                sdl2_sprite = SDL2Sprite(sprite, sprite_original, 0, 0)
                # sdl2_sprite.rotozoom(angle=5.0 * i, zoom=0.5 + 0.1 * i)
                self.sdl2_sprites[key].append(sdl2_sprite)

        self.image_pub = rospy.Publisher("image", Image, queue_size=5)

        self.count = 0

        self.camera_infos = queue.Queue()
        self.marker_array = None

        self.camera_info_sub = rospy.Subscriber("camera_info", CameraInfo,
                                                self.camera_info_callback, queue_size=5)
        self.marker_sub = rospy.Subscriber("marker_array", MarkerArray,
                                           self.marker_callback, queue_size=5)

        update_rate = rospy.get_param("~update_rate", 100)
        # dt = rospy.Duration(1.0 / update_rate)
        rospy.loginfo(f"{update_rate}")
        # TODO(lucasw) this doesn't work because of threading issues,
        # but if initialization of sdl window etc. was moved into the update that probably works
        # self.timer = rospy.Timer(dt, self.update)
        rate = rospy.Rate(update_rate)
        old_t0 = rospy.Time.now()
        while not rospy.is_shutdown():
            t0 = rospy.Time.now()
            event = rospy.timer.TimerEvent(old_t0, old_t0, t0, t0, t0 - old_t0)
            event.current_real = t0
            event.last_real = old_t0
            # rospy.loginfo(f"{(t0 - old_t0).to_sec():0.3f}")
            if True:  # try:
                self.update(event=None, camera_info=self.camera_infos.get(timeout=1.0))
            # except Exception as ex:
            #     rospy.logwarn_throttle(5.0, ex)
            #     continue
            rate.sleep()
            old_t0 = t0

            queue_size = self.camera_infos.qsize()
            max_size = 3
            if queue_size > max_size:
                # rospy.logwarn_throttle(2.0, f"draining queue {queue_size}")
                while self.camera_infos.qsize() > 1:
                    self.camera_infos.get()

    def camera_info_callback(self, msg):
        if msg.width != self.init_camera_info.width or msg.height != self.init_camera_info.height:
            text = f"{msg.width} {msg.height} != {self.init_camera_info.width} {self.init_camera_info.height}"
            rospy.logwarn_throttle(2.0, text)
            return
        # TODO(lucasw) drain queue down if it gets too big
        self.camera_infos.put(msg)

    def marker_callback(self, msg):
        with self.lock:
            self.marker_array = msg

    def update(self, event, camera_info):
        with self.lock:
            # take possession of the marker_array (avoid having to copy it)
            marker_array = self.marker_array
            if marker_array is None:
                return
            self.marker_array = None

        events = sdl2.ext.get_events()
        for event in events:
            if event.type == sdl2.SDL_QUIT:
                rospy.logwarn("sdl2 quit")
                rospy.signal_shutdown("sdl2 quit")

        # get transforms of sprites relative to camera
        rvec = np.zeros((1, 3))
        tvec = np.zeros((1, 3))
        camera_matrix = np.zeros((3, 3))
        for y_ind in range(3):
            for x_ind in range(3):
                ind = y_ind * 3 + x_ind
                camera_matrix[y_ind, x_ind] = camera_info.K[ind]
        fx = camera_info.K[0]
        # cx = camera_info.K[1]
        # cy = camera_info.K[5]
        dist_coeff = np.asarray(camera_info.D)

        camera_frame = camera_info.header.frame_id
        stamp = camera_info.header.stamp

        for key, sprites in self.sdl2_sprites.items():
            for sprite in sprites:
                sprite.is_active = False

        min_z = 0.01

        for marker in marker_array.markers:
            if marker.mesh_resource not in self.sdl2_sprites.keys():
                rospy.logwarn_throttle(4.0, f"{marker.mesh_resource} not in {self.sdl2_sprites.keys()}")
                # TODO(lucasw) load it right now and use it, don't have the config yaml param
                continue
            sprites = self.sdl2_sprites[marker.mesh_resource]

            # TODO(lucasw) ignore marker.pose for now, but incorporate it later
            try:
                tfs = self.tf_buffer.lookup_transform(camera_frame, marker.header.frame_id,
                                                      stamp, timeout=rospy.Duration(0.2))
            except tf2_ros.LookupException as ex:
                rospy.logwarn_throttle(4.0, ex)
                return

            pc2_in = marker_to_point_cloud2(marker)
            pc2_in.header.stamp = stamp
            # TODO(lucasw) probably a more efficient transform on a Point array than converting to PointCloud2
            pc2_out = do_transform_cloud(pc2_in, tfs)
            pc2_points = point_cloud2.read_points(pc2_out, field_names=("x", "y", "z"), skip_nans=True)

            # don't want points behind the camera
            points3d = [pt for pt in pc2_points if pt[2] > min_z]
            # sort pc2_points by z, largest first, so can do painters algorithm
            points3d.sort(key=lambda pt: pt[2], reverse=True)
            num_points = len(points3d)
            if num_points == 0:
                continue
            points_in_camera_frame = np.zeros((num_points, 3))
            for pt_ind, pt in enumerate(points3d):
                points_in_camera_frame[pt_ind, 0] = pt[0]
                points_in_camera_frame[pt_ind, 1] = pt[1]
                points_in_camera_frame[pt_ind, 2] = pt[2]

            points2d, _ = cv2.projectPoints(points_in_camera_frame, rvec, tvec, camera_matrix, dist_coeff)
            # rospy.loginfo(f"{camera_frame} to {key}:
            #               {point.x:0.2f} {point.y:0.2f}
            #               {point.z:0.2f}, {point2d[0]}")

            # TODO(lucasw) if multiple markers use the same 'mesh_resource' only the last one will work
            # - make it so they can all use sprites from the same pool (up to the max_sprites limit)
            for point_ind, (point3d, sprite) in enumerate(zip(points3d, sprites)):
                sprite.is_active = True
                use_rotozoom = True
                if not use_rotozoom:
                    continue
                rotation = tfs.transform.rotation
                rotation_array = [rotation.x, rotation.y, rotation.z, rotation.w]

                # want the z axis first, that's the angle around the axis pointing into the optical frame
                # TODO(lucasw) not 100% that zxz is correct, but tried a lot of relative camera angles
                euler = transformations.euler_from_quaternion(rotation_array, 'szxz')
                angle = -euler[2]
                # rospy.loginfo_throttle(1.0, f"euler {euler[0]:0.2f} {euler[1]:0.2f} {euler[2]:0.2f}")

                sprite_width = sprite.sprite_original.size[0]
                # TODO(lucasw) the right scale differs acrros the image depending on the
                # instrinsics, otherwise there will be gaps- need to adjust for that
                sprite_width_meters = marker.scale.x
                # pixels * (meters / pixels) / meters -> unitless scale
                zoom = fx * (sprite_width_meters / sprite_width) / point3d[2]
                # rospy.loginfo(f"zoom {zoom:0.3f}, angle {angle:0.2f}")
                sprite.rotozoom(zoom=zoom, angle=np.degrees(angle))

                rotated_width = sprite.sprite.size[0]
                rotated_height = sprite.sprite.size[1]
                sprite.update_position(points2d[point_ind, 0, 0] - rotated_width * 0.5,
                                       points2d[point_ind, 0, 1] - rotated_height * 0.5)

        # blank image
        sdl2.ext.fill(self.sprite_renderer.surface, (0, 0, 0))

        rospy.logdebug_throttle(1.0, f"update {len(self.sdl2_sprites.keys())} sprites")
        sprites_to_render = []
        for key, sprite_array in self.sdl2_sprites.items():
            for sprite in sprite_array:
                if not sprite.is_active:
                    continue
                sprites_to_render.append(sprite.sprite)
        self.sprite_renderer.render(sprites_to_render)

        if self.show_sdl_window:
            self.window.refresh()

        publish_image = True
        if publish_image:
            t0 = rospy.Time.now()
            image_data = sdl2.ext.pixels2d(self.sprite_renderer.surface).T
            # t1 = rospy.Time.now()
            image_rgb = image_data.view(np.uint8).reshape(image_data.shape + (4,))[..., :self.num_chan]
            t2 = rospy.Time.now()
            # cv bridge needs to convert from uint32 packed bytes to color channels
            if self.num_chan == 3:
                image_msg = self.cv_bridge.cv2_to_imgmsg(image_rgb, "bgr8")
            elif self.num_chan == 4:
                image_msg = self.cv_bridge.cv2_to_imgmsg(image_rgb, "bgra8")
            else:
                text = f"bad number of channels {self.num_chan}"
                rospy.signal_shutdown(text)
                raise Exception(text)
            # image_msg = self.cv_bridge.cv2_to_imgmsg(image_data, "bgr8")
            t3 = rospy.Time.now()
            image_msg.header = camera_info.header
            self.image_pub.publish(image_msg)
            text = f"sdl2 to numpy array in {(t3 - t2).to_sec():0.3f}s + {(t2 - t0).to_sec():0.3f}s"
            text += f", {image_data.shape} {image_rgb.shape} {image_msg.width} x {image_msg.height}"
            rospy.logdebug_throttle(4.0, text)

        self.count += 1

        with self.lock:
            # give up possession
            if self.marker_array is None:
                self.marker_array = marker_array


if __name__ == "__main__":
    rospy.init_node("sdl2_sprites")
    node = SDL2Sprites()
    rospy.spin()
