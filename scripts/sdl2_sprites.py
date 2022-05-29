#!/usr/bin/env python
# display sprites in an image with ros integration- publish the image out, only optionally
# show an sdl window
# rosrun sdl2_ros sdl2_sprites.py _image1:=`rospack find vimjay`/data/plasma.png

import numpy as np
import rospy
import sdl2
import sdl2.ext
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class SDL2Sprite(object):
    def __init__(self, sprite, x=0.0, y=0.0):
        self.px = x
        self.py = y
        self.sprite = sprite
        self.update_position()

    def __repr__(self):
        return f"x {self.px:0.2f} y {self.py:0.2f}, {self.sprite.position}"

    def update_position(self, dx=0.0, dy=0.0):
        self.px += dx
        self.py += dy
        self.sprite.position = (int(self.px), int(self.py))


class SDL2Sprites(object):
    def __init__(self):
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

        sprite1_path = rospy.get_param("~image1")
        rospy.loginfo(sprite1_path)

        self.sprite_renderer = self.factory.create_sprite_render_system(self.window)

        self.sdl2_sprites = []
        for i in range(10):
            ground_sprite = self.factory.from_image(sprite1_path)
            self.sdl2_sprites.append(SDL2Sprite(ground_sprite, 20 + i * 60, 20 + i * 4))

        self.image_pub = rospy.Publisher("image", Image, queue_size=5)

        self.count = 0

        update_rate = rospy.get_param("~update_rate", 30)
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
            self.update(event=None)
            rate.sleep()
            old_t0 = t0

    def update(self, event):
        events = sdl2.ext.get_events()
        for event in events:
            if event.type == sdl2.SDL_QUIT:
                rospy.logwarn("sdl2 quit")
                rospy.signal_shutdown("sdl2 quit")

        for ind, sdl2_sprite in enumerate(self.sdl2_sprites):
            dx = 2 + 0.5 * ind
            dy = 5 + 1.0 * ind
            if int(self.count / 100.0) % 2 != 0:
                dx *= -1.0
                dy *= -1.0
            sdl2_sprite.update_position(dx, dy)

        # blank image
        sdl2.ext.fill(self.sprite_renderer.surface, (0, 0, 0))

        rospy.logdebug_throttle(1.0, f"update {len(self.sdl2_sprites)} sprites")
        self.sprite_renderer.render([sdl2_sprite.sprite for sdl2_sprite in self.sdl2_sprites])
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
            rospy.loginfo_throttle(4.0, text)

        self.count += 1


if __name__ == "__main__":
    rospy.init_node("sdl2_sprites")
    node = SDL2Sprites()
    rospy.spin()
