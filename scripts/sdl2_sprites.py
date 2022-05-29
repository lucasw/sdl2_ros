#!/usr/bin/env python
# display sprites in an image with ros integration- publish the image out, only optionally
# show an sdl window
# rosrun sdl2_ros sdl2_sprites.py _image1:=`rospack find vimjay`/data/plasma.png


import rospy
import sdl2
import sdl2.ext


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
        rospy.loginfo(f"SDL2 Version {sdl2.__version__}")
        # TODO(lucasw) could get these from a camera_info
        image_width = rospy.get_param("~width", 1280)
        image_height = rospy.get_param("~height", 720)
        self.window = sdl2.ext.Window("sdl2_sprites", size=(image_width, image_height))
        self.window.show()

        self.factory = sdl2.ext.SpriteFactory(sdl2.ext.SOFTWARE)

        sprite1_path = rospy.get_param("~image1")
        rospy.loginfo(sprite1_path)

        self.sprite_renderer = self.factory.create_sprite_render_system(self.window)

        self.sdl2_sprites = []
        for i in range(10):
            ground_sprite = self.factory.from_image(sprite1_path)
            self.sdl2_sprites.append(SDL2Sprite(ground_sprite, 20 + i * 60, 20 + i * 4))

        self.count = 0

        dt = rospy.Duration(0.03667)
        # TODO(lucasw) this doesn't work because of threading issues,
        # but if initialization of sdl window etc. was moved into the update that probably works
        # self.timer = rospy.Timer(dt, self.update)
        while not rospy.is_shutdown():
            self.update(event=None)
            rospy.sleep(dt)

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

        self.window.refresh()
        self.count += 1


if __name__ == "__main__":
    rospy.init_node("sdl2_sprites")
    node = SDL2Sprites()
    rospy.spin()
