#!/usr/bin/env python
# display sprites in an image with ros integration- publish the image out, only optionally
# show an sdl window
# rosrun sdl2_ros sdl2_sprites.py _image1:=`rospack find vimjay`/data/plasma.png

import queue
from threading import Lock

import cv2
import numpy as np
import rospy
import sdl2
import sdl2.ext
import sdl2.sdlgfx
import tf2_ros
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from tf import transformations


class SDL2Sprite(object):
    def __init__(self, sprite, sprite_original, x=0.0, y=0.0):
        self.sprite = sprite
        self.sprite_original = sprite_original
        self.update_position(x, y)
        self.is_active = True
        self.width_meters = 0.2

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
        # TODO(lucasw) could get these from a camera_info
        image_width = rospy.get_param("~width", 640)
        image_height = rospy.get_param("~height", 360)

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

        self.sdl2_sprites = {}
        for key, image_path in images.items():
            sprite = self.factory.from_image(image_path)
            # TODO(lucasw) how to make copy of sprite?
            sprite_original = self.factory.from_image(image_path)
            sdl2_sprite = SDL2Sprite(sprite, sprite_original, 0, 0)
            # sdl2_sprite.rotozoom(angle=5.0 * i, zoom=0.5 + 0.1 * i)
            self.sdl2_sprites[key] = sdl2_sprite

        self.image_pub = rospy.Publisher("image", Image, queue_size=5)

        self.count = 0

        self.camera_infos = queue.Queue()

        self.camera_info_sub = rospy.Subscriber("camera_info", CameraInfo,
                                                self.camera_info_callback, queue_size=5)

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

    def camera_info_callback(self, msg):
        # TODO(lucasw) drain queue down if it gets too big
        self.camera_infos.put(msg)

    def update(self, event, camera_info):
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

        # num = len(self.sdl2_sprites.keys())
        for ind, (key, sdl2_sprite) in enumerate(self.sdl2_sprites.items()):
            tfs = self.tf_buffer.lookup_transform(camera_frame, key, stamp, timeout=rospy.Duration(0.2))
            point = tfs.transform.translation

            point_in_camera_frame = np.zeros((1, 3))
            point_in_camera_frame[0, 0] = point.x
            point_in_camera_frame[0, 1] = point.y
            point_in_camera_frame[0, 2] = point.z

            # don't want points behind the camera
            min_z = 0.01
            if point.z < min_z:
                sdl2_sprite.is_active = False
                continue
            sdl2_sprite.is_active = True

            sprite_width = sdl2_sprite.sprite_original.size[0]
            sprite_height = sdl2_sprite.sprite_original.size[1]

            point2d, _ = cv2.projectPoints(point_in_camera_frame, rvec, tvec, camera_matrix, dist_coeff)
            # rospy.loginfo(f"{camera_frame} to {key}: {point.x:0.2f} {point.y:0.2f} {point.z:0.2f}, {point2d[0]}")

            sdl2_sprite.update_position(point2d[0, 0, 0] - sprite_width * 0.5,
                                        point2d[0, 0, 1] - sprite_height * 0.5)
            # TODO(lucasw) do something with rotation from tfs.transform.rotation yaw
            rotation = tfs.transform.rotation
            rotation_array = [rotation.x, rotation.y, rotation.z, rotation.w]
            euler = transformations.euler_from_quaternion(rotation_array)
            yaw = -euler[2]

            # pixels * (meters / pixels) / meters -> unitless scale
            zoom = fx * (sdl2_sprite.width_meters / sprite_width) / point.z
            # rospy.loginfo(f"zoom {zoom:0.3f}, yaw {yaw:0.2f}")
            sdl2_sprite.rotozoom(zoom=zoom, angle=np.degrees(yaw))

        # blank image
        sdl2.ext.fill(self.sprite_renderer.surface, (0, 0, 0))

        rospy.logdebug_throttle(1.0, f"update {len(self.sdl2_sprites.keys())} sprites")
        sprites_to_render = []
        for key, sdl2_sprite in self.sdl2_sprites.items():
            if not sdl2_sprite.is_active:
                continue
            sprites_to_render.append(sdl2_sprite.sprite)
        self.sprite_renderer.render(sprites_to_render)
        rospy.logdebug_throttle(1.0, f"{ind} {sdl2_sprite}")

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
            image_msg.header.stamp = t0
            image_msg.header.frame_id = "map"
            self.image_pub.publish(image_msg)
            text = f"sdl2 to numpy array in {(t3 - t2).to_sec():0.3f}s + {(t2 - t0).to_sec():0.3f}s"
            text += f", {image_data.shape} {image_rgb.shape} {image_msg.width} x {image_msg.height}"
            rospy.logdebug_throttle(4.0, text)

        self.count += 1


if __name__ == "__main__":
    rospy.init_node("sdl2_sprites")
    node = SDL2Sprites()
    rospy.spin()
